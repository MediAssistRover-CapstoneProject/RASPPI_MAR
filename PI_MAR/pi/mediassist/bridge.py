import asyncio
import json
import logging
import ssl
from datetime import datetime, timezone
from urllib.parse import urlparse
from websockets.asyncio.client import connect
from .contracts import read_json, validate, StepError, canonical
from . import __version__
from .store import utcnow


def tls_context(config):
    """System CA by default; an optional project CA still verifies peer + hostname."""
    context = ssl.create_default_context(cafile=config.get("tls_ca_file") or None)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    return context


class Bridge:
    def __init__(self, runtime, config, remote, root, token):
        self.runtime, self.cfg, self.remote, self.token = runtime, config, remote, token
        self.schema = read_json(root / "schemas/command.schema.json")
        self.job_task = None
        self.sent = set()
        self.send_lock = asyncio.Lock()

    async def command(self, msg):
        validate(msg, self.schema)
        if msg["robot_id"] != self.cfg["robot_id"]: raise StepError("WRONG_ROBOT")
        cached = self.runtime.store.cached_command(msg)
        if cached: return cached
        if datetime.fromisoformat(msg["expires_at"].replace("Z", "+00:00")) <= datetime.now(timezone.utc):
            raise StepError("COMMAND_EXPIRED")
        code = msg["command"]
        if code == "DISPATCH_JOB":
            job = msg["job"]
            self.runtime.prepare(job)
            if self.job_task is not None and not self.job_task.done(): raise StepError("ROBOT_BUSY")
            states = self.runtime.store.states()
            if any(s == "RECOVERY_REQUIRED" for s in states.values()): raise StepError("MANUAL_RECONCILIATION_REQUIRED")
            previous = self.runtime.store.add_job(job)
            response = {"type": "COMMAND_ACK", "command_id": msg["command_id"], "accepted": True,
                        "job_id": job["job_id"], "status": previous or "ACCEPTED"}
            # Persist ACK before starting. Crash before execution is recovered conservatively at boot.
            self.runtime.store.remember_command(msg, response)
            if previous is None:
                self.runtime.store.event(job["job_id"], "JOB_ACCEPTED", {"command_id": msg["command_id"]})
                self.job_task = asyncio.create_task(self.runtime.run(job))
            return response
        if msg["job_id"] != self.runtime.active: raise StepError("JOB_NOT_ACTIVE")
        self.runtime.cancel.set()
        await self.runtime.adapter.stop()
        response = {"type": "COMMAND_ACK", "command_id": msg["command_id"], "accepted": True, "status": "CANCEL_REQUESTED"}
        self.runtime.store.remember_command(msg, response)
        self.runtime.store.event(msg["job_id"], "CANCEL_REQUESTED", {"command_id": msg["command_id"]})
        return response

    async def run(self):
        uri = urlparse(self.cfg["server_url"])
        local_plain = uri.scheme == "ws" and uri.hostname in ("127.0.0.1", "localhost", "::1") and self.cfg.get("allow_insecure_localhost")
        if uri.scheme != "wss" and not local_plain: raise StepError("TLS_REQUIRED")
        if uri.username is not None or uri.password is not None: raise StepError("TOKEN_IN_URL_FORBIDDEN")
        if len(self.token) < 16: raise StepError("TOKEN_TOO_SHORT", "Use a per-robot token from environment (at least 16 characters)")
        ssl_options = {"ssl": tls_context(self.cfg)} if uri.scheme == "wss" else {}
        delay = 1
        while True:
            try:
                async with connect(self.cfg["server_url"], additional_headers={"Authorization": "Bearer " + self.token},
                                   max_size=1048576, ping_interval=10, ping_timeout=10, proxy=None, **ssl_options) as ws:
                    delay = 1
                    self.sent.clear()
                    async def send(message):
                        async with self.send_lock: await ws.send(canonical(message))
                    self.remote.send = send
                    await send({"protocol_version": 1, "type": "HELLO", "robot_id": self.cfg["robot_id"],
                                "runtime_version": __version__, "simulation": self.runtime.adapter.simulated,
                                "capabilities": self.cfg["capabilities"], "jobs": self.runtime.store.states()})
                    outbox = asyncio.create_task(self._outbox(send))
                    try:
                        while True:
                            receiving = asyncio.create_task(ws.recv())
                            try:
                                done, _ = await asyncio.wait({receiving, outbox}, return_when=asyncio.FIRST_COMPLETED)
                                if outbox in done:
                                    outbox.result()
                                    raise StepError("OUTBOX_STOPPED")
                                raw = receiving.result()
                            finally:
                                receiving.cancel()
                                await asyncio.gather(receiving, return_exceptions=True)
                            if not isinstance(raw, str): raise StepError("TEXT_JSON_REQUIRED")
                            msg = json.loads(raw)
                            if not isinstance(msg, dict): raise StepError("INVALID_MESSAGE")
                            if msg.get("type") == "EVENT_ACK":
                                # Only acknowledge events sent during this authenticated connection.
                                if msg.get("event_id") in self.sent:
                                    self.runtime.store.ack(msg["event_id"])
                            elif msg.get("type") == "SERVICE_RESPONSE":
                                self.remote.reply(msg)
                            elif msg.get("type") == "COMMAND":
                                try:
                                    ack = await self.command(msg)
                                except (StepError, ValueError) as exc:
                                    ack = {"type": "COMMAND_ACK", "command_id": msg.get("command_id"), "accepted": False, "error_code": getattr(exc, "code", "INVALID_COMMAND")}
                                await send(ack)
                            else:
                                await send({"type": "PROTOCOL_ERROR", "error_code": "UNKNOWN_MESSAGE_TYPE"})
                    finally:
                        outbox.cancel()
                        await asyncio.gather(outbox, return_exceptions=True)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logging.warning("Connection closed (%s); motion stopped; retry in %ss", type(exc).__name__, delay)
            finally:
                self.remote.disconnected()
                self.runtime.cancel.set()
                await self.runtime.adapter.stop()
                if self.job_task and not self.job_task.done():
                    self.job_task.cancel()
                    await asyncio.gather(self.job_task, return_exceptions=True)
            await asyncio.sleep(delay)
            delay = min(delay * 2, 30)

    async def _outbox(self, send):
        tick = 0
        while True:
            for event in self.runtime.store.pending():
                self.sent.add(event["event_id"])
                await send(event)
            if tick % 4 == 0:
                telemetry = self.runtime.adapter.telemetry() if hasattr(self.runtime.adapter, "telemetry") else {"simulation": True}
                await send({"protocol_version": 1, "type": "HEARTBEAT", "robot_id": self.cfg["robot_id"], "active_job_id": self.runtime.active,
                            "sent_at": utcnow(), "telemetry": telemetry})
            tick += 1
            await asyncio.sleep(0.5)
