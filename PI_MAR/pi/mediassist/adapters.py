"""Backend service bridge and an explicitly simulated adapter for contract tests."""
import asyncio
import uuid
from datetime import datetime, timedelta, timezone
from .contracts import StepError
from .store import utcnow


class SimAdapter:
    simulated = True

    def __init__(self, overrides=None):
        self.calls = []
        self.overrides = overrides or {}

    def check_capabilities(self, wf):
        return None

    async def stop(self):
        return None

    async def execute(self, kind, p, meta):
        self.calls.append((kind, p, meta.copy()))
        override = self.overrides.get(meta["step_code"])
        if override is not None:
            if isinstance(override, Exception): raise override
            return override
        await asyncio.sleep(0)
        if kind == "MOVE":
            return {"reached": True, "endpoint_id": p.get("endpoint_id", (p.get("candidate_endpoint_ids") or [None])[0]), "endpoint_group_id": p.get("endpoint_group_id"), "attempts": [], "action_status": 4, "last_feedback_pose": None, "distance_remaining_m": 0.0}
        if kind == "CONTROL_HARDWARE":
            return {"published": True, "controller_ack": False, "verified": False, "commanded_state": p["action"],
                    "position_raw": 1500 if p["action"] == "LOCK" else 2400, "ros_servo_id": 4}
        if kind == "INTERACT":
            return {"completed": True, "confirmed": True if p["action"] == "CONFIRM" else None, "input_data": {}, "actor_user_id": "SIM_STAFF"}
        if kind == "DATA_SERVICE":
            return {"request_id": str(uuid.uuid4()), "data": {"answer": "Dữ liệu mô phỏng, chưa kết nối dữ liệu bệnh viện.", "incident_id": "SIM_INCIDENT"}}
        if kind == "AUTHENTICATE":
            return {"authenticated": True, "verification_id": "SIM_VER_" + meta["step_code"], "actor_user_id": "SIM_STAFF", "purpose": p["purpose"],
                    "valid_until": (datetime.now(timezone.utc) + timedelta(minutes=2)).isoformat()}
        if kind == "HANDLE_PAYLOAD":
            return {"confirmed": True, "verification_mode": "STAFF_CONFIRMED", "actor_user_id": "SIM_STAFF", "items": p["items"], "weight_kg": None, "temperature_c": None}
        if kind == "EXECUTE_SENSING":
            metrics = {"sensors_ready": True, "battery_raw": 12000, "battery_percent": None} if p["task_type"] == "ROBOT_HEALTH" else (
                {"left_m": 0.8, "right_m": 0.8, "left_fresh": True, "right_fresh": True} if p["task_type"] == "SIDE_CLEARANCE" else
                {"count": 12} if p["task_type"] == "CROWD_DENSITY" else {"intrusion_detected": True})
            return {"task_type": p["task_type"], "metrics": metrics, "captured_at": utcnow(), "evidence_ref": None}
        if kind == "NOTIFY":
            return {"notification_id": str(uuid.uuid4()), "delivered": True, "acked": p["require_ack"]}
        if kind == "WAIT":
            return {"completed": True, "wake_reason": p["event_type"], "elapsed_sec": 0.0,
                    "event": {"type": p["event_type"], "correlation_key": p["correlation_key"], "simulation": True,
                          "event_id": str(uuid.uuid4()), "job_id": meta["job_id"], "actor_user_id": "SIM_STAFF", "occurred_at": utcnow()}}
        raise StepError("UNSUPPORTED_STEP", kind)


def validate_remote_result(kind, p, meta, data):
    """Check the meaning of a backend result before the workflow branches on it."""
    if kind == "AUTHENTICATE" and data.get("authenticated") is True:
        try: expiry = datetime.fromisoformat(data["valid_until"].replace("Z", "+00:00"))
        except (KeyError, ValueError, TypeError, AttributeError): raise StepError("INVALID_AUTH_RESPONSE") from None
        if expiry.tzinfo is None or expiry <= datetime.now(timezone.utc) or data.get("purpose") != p["purpose"]:
            raise StepError("AUTH_EXPIRED_OR_WRONG_SCOPE")
    if kind == "NOTIFY" and (data.get("delivered") is not True or (p["require_ack"] and data.get("acked") is not True)):
        raise StepError("NOTIFICATION_NOT_COMPLETED")
    if kind == "INTERACT" and data.get("completed") is not True: raise StepError("INTERACTION_NOT_COMPLETED")
    if kind == "INTERACT" and p["prompt_key"].startswith("STAFF_") and data.get("confirmed") is True and not data.get("actor_user_id"):
        raise StepError("STAFF_ACTOR_REQUIRED")
    if kind == "HANDLE_PAYLOAD" and data.get("confirmed") is True and data.get("items") != p["items"]:
        raise StepError("PAYLOAD_MISMATCH")
    if kind == "WAIT":
        event = data.get("event") or {}
        if (data.get("completed") is not True or event.get("correlation_key") != p["correlation_key"]
                or event.get("type") != p["event_type"] or event.get("job_id") != meta["job_id"]
                or not event.get("event_id") or not event.get("occurred_at")):
            raise StepError("EVENT_NOT_MATCHED")
        if p["event_type"].startswith("STAFF_") and not event.get("actor_user_id"):
            raise StepError("STAFF_ACTOR_REQUIRED")
    if kind == "EXECUTE_SENSING" and data.get("task_type") != p["task_type"]:
        raise StepError("SENSING_TASK_MISMATCH")
    return data


class SimRemoteAdapter(SimAdapter):
    """Simulate physical I/O while exercising backend services over the real socket."""
    def __init__(self, remote):
        super().__init__()
        if not remote.allow_simulation: raise StepError("SIMULATION_REMOTE_REQUIRED")
        self.remote = remote

    async def execute(self, kind, p, meta):
        if kind in ("MOVE", "CONTROL_HARDWARE") or (kind == "EXECUTE_SENSING" and p["task_type"] in ("ROBOT_HEALTH", "SIDE_CLEARANCE")):
            return await super().execute(kind, p, meta)
        self.calls.append((kind, p, meta.copy()))
        data = await self.remote.call(kind, p, meta)
        return validate_remote_result(kind, p, meta, data)


class RemoteServices:
    def __init__(self, allow_simulation=False):
        self.send = None
        self.pending = {}
        self.allow_simulation = allow_simulation

    def disconnected(self):
        self.send = None
        for fut in self.pending.values():
            if not fut.done(): fut.set_exception(StepError("CONNECTION_LOST"))

    def reply(self, message):
        fut = self.pending.get(message.get("request_id"))
        if fut is None or fut.done(): return
        if message.get("ok") is not True:
            fut.set_exception(StepError(message.get("error_code", "SERVICE_FAILED")))
        elif message.get("simulation") is True and not self.allow_simulation:
            fut.set_exception(StepError("SIMULATED_RESULT_REJECTED_ON_LIVE"))
        else:
            fut.set_result(message.get("data"))

    async def call(self, kind, p, meta):
        if self.send is None: raise StepError("SERVER_UNAVAILABLE")
        if meta.get("simulation") is True and not self.allow_simulation:
            raise StepError("SIMULATION_META_ON_LIVE")
        request_id = str(uuid.uuid4())
        fut = asyncio.get_running_loop().create_future()
        self.pending[request_id] = fut
        try:
            await self.send({"protocol_version": 1, "type": "SERVICE_REQUEST", "request_id": request_id,
                             "step_type": kind, "params": p, **meta, "simulation": self.allow_simulation})
            result = await asyncio.wait_for(fut, meta["timeout_sec"])
            if not isinstance(result, dict): raise StepError("INVALID_SERVICE_RESPONSE")
            return result
        finally:
            self.pending.pop(request_id, None)
