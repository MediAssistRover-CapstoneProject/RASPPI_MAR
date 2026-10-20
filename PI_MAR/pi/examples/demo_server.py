"""Loopback-only protocol server. Refuses real hardware clients and persists events before ACK."""
import argparse
import asyncio
import hmac
import json
import os
import sqlite3
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from websockets.asyncio.server import serve
from mediassist.contracts import Catalog, canonical, digest, read_json


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workflow", default="WF_ROBOT_STARTUP_STANDBY")
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--db", default="state/demo_server.sqlite3")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    token = os.environ.get("MEDIASSIST_ROBOT_TOKEN", "")
    if len(token) < 16: raise SystemExit("Set MEDIASSIST_ROBOT_TOKEN (at least 16 characters)")
    catalog = Catalog(args.root)
    wf = catalog.get(args.workflow, 4)
    Path(args.db).parent.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(args.db)
    db.execute("CREATE TABLE IF NOT EXISTS events(id TEXT PRIMARY KEY, body TEXT NOT NULL)")
    job = {"job_id": str(uuid.uuid4()), "workflow_code": wf["code"], "workflow_version": wf["version"],
           "workflow_hash": digest(wf), "input": read_json(args.root / "examples" / (wf["code"] + ".input.json"))}
    command = {"protocol_version": 1, "type": "COMMAND", "command_id": str(uuid.uuid4()), "robot_id": "mentorpi-m1-01",
               "command": "DISPATCH_JOB", "expires_at": (datetime.now(timezone.utc) + timedelta(hours=1)).isoformat(), "job": job}

    async def handler(ws):
        if not hmac.compare_digest(ws.request.headers.get("Authorization", ""), "Bearer " + token):
            await ws.close(1008, "Authentication required"); return
        hello = json.loads(await asyncio.wait_for(ws.recv(), 5))
        if hello.get("type") != "HELLO" or hello.get("simulation") is not True or hello.get("robot_id") != command["robot_id"]:
            await ws.close(1008, "Simulation clients only"); return
        await ws.send(canonical(command))
        async for raw in ws:
            msg = json.loads(raw)
            if msg.get("type") == "EVENT":
                body = canonical(msg)
                old = db.execute("SELECT body FROM events WHERE id=?", (msg["event_id"],)).fetchone()
                if old and old[0] != body:
                    await ws.close(1008, "Event ID conflict"); return
                with db:
                    db.execute("INSERT OR IGNORE INTO events VALUES(?,?)", (msg["event_id"], body))
                await ws.send(canonical({"type": "EVENT_ACK", "event_id": msg["event_id"]}))
                print(msg["event_type"], msg["job_id"], flush=True)
                if msg["event_type"] == "JOB_RESULT": print(json.dumps(msg["data"], ensure_ascii=False), flush=True)
            elif msg.get("type") == "COMMAND_ACK":
                print(json.dumps(msg, ensure_ascii=False), flush=True)
    async with serve(handler, "127.0.0.1", args.port, max_size=1048576):
        print(f"SIMULATION ONLY ws://127.0.0.1:{args.port}; workflow={wf['code']}", flush=True)
        await asyncio.Future()


if __name__ == "__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: pass
