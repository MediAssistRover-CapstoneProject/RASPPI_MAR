"""Operator tools: profile check, command generation and explicit local hardware tests."""
import argparse
import asyncio
import json
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from .contracts import Catalog, StepError, read_json, canonical, digest, validate
from .configuration import load_config
from .cli import instance_lock
from .store import Store
from .adapters import RemoteServices


def command_for(catalog, robot_id, code, inputs, ttl=60):
    wf = catalog.get(code, 4)
    validate(inputs, wf["input_schema"])
    return {"protocol_version": 1, "type": "COMMAND", "command_id": str(uuid.uuid4()),
            "robot_id": robot_id, "command": "DISPATCH_JOB",
            "expires_at": (datetime.now(timezone.utc) + timedelta(seconds=ttl)).isoformat(),
            "job": {"job_id": str(uuid.uuid4()), "workflow_code": code, "workflow_version": 4,
                    "workflow_hash": digest(wf), "input": inputs}}


async def hardware(args, cfg, store):
    from .ros_adapter import RosAdapter
    adapter = RosAdapter(cfg, RemoteServices())
    catalog = Catalog(args.root)
    job_id = None
    result = None
    try:
        # Let subscriptions discover the real publishers before freshness checks.
        await asyncio.sleep(2)
        if args.command == "sensors":
            return {"mode": "LOCAL_SENSOR_READ", "health": adapter.health(), "telemetry": adapter.telemetry()}
        if any(v == "RECOVERY_REQUIRED" for v in store.states().values()):
            raise StepError("MANUAL_RECONCILIATION_REQUIRED")
        if not args.execute: raise StepError("EXPLICIT_EXECUTE_REQUIRED")
        if args.command == "servo":
            if not args.mechanism_clear: raise StepError("MECHANISM_CLEAR_REQUIRED")
            params = {"device_id": "cargo_lock", "action": args.action, "verification_mode": "COMMAND_ONLY"}
            # This explicit operator maintenance test is not an authenticated delivery workflow.
            kind = "CONTROL_HARDWARE"
        else:
            params = read_json(args.params)
            catalog.check_params("MOVE", params)
            kind = "MOVE"
        job_id = str(uuid.uuid4())
        store.add_job({"job_id": job_id, "source": "LOCAL_OPERATOR_TEST", "type": kind, "params": params})
        store.status(job_id, "RUNNING")
        adapter.event_sink = store.event
        meta = {"job_id": job_id, "job_step_id": str(uuid.uuid4()), "step_code": "MANUAL_" + args.command.upper(),
                "task_code": "MAINTENANCE", "visit_no": 1, "attempt_no": 1, "timeout_sec": args.timeout,
                "simulation": False, "source": "LOCAL_OPERATOR_TEST"}
        meta["operation_key"] = meta["job_step_id"]
        store.event(job_id, "MANUAL_TEST_STARTED", {**meta, "type": kind, "resolved_input": params})
        data = await asyncio.wait_for(adapter.execute(kind, params, meta), args.timeout)
        catalog.check_output(kind, data)
        result = {"status": "SUCCEEDED", "data": data, "job_id": job_id}
        return result
    except BaseException as exc:
        if job_id:
            result = {"status": "RECOVERY_REQUIRED", "job_id": job_id,
                      "error_code": getattr(exc, "code", type(exc).__name__)}
        raise
    finally:
        if args.command != "sensors":
            confirmed = await adapter.stop()
            if job_id:
                if not confirmed:
                    if result is None: result = {}
                    result.update(status="RECOVERY_REQUIRED", job_id=job_id, error_code="GOAL_STOP_UNCONFIRMED")
                result = result or {"status": "RECOVERY_REQUIRED", "job_id": job_id, "error_code": "TEST_INTERRUPTED"}
                store.status(job_id, result["status"])
                store.event(job_id, "MANUAL_TEST_RESULT", result)
        adapter.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, default=Path.cwd())
    sub = p.add_subparsers(dest="command", required=True)
    for name in ("check-config", "sensors", "servo", "move", "journal"):
        a = sub.add_parser(name)
        a.add_argument("--config", type=Path, default=Path("config/site.local.json"))
        if name in ("servo", "move"):
            a.add_argument("--execute", action="store_true")
            a.add_argument("--timeout", type=float, default=30)
        if name == "servo":
            a.add_argument("--action", choices=("LOCK", "UNLOCK"), required=True)
            a.add_argument("--mechanism-clear", action="store_true")
        if name == "move": a.add_argument("--params", type=Path, required=True)
    a = sub.add_parser("make-command")
    a.add_argument("--robot-id", default="mentorpi-m1-01")
    a.add_argument("--workflow", required=True)
    a.add_argument("--input", type=Path, required=True)
    a.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    try:
        if args.command == "make-command":
            result = command_for(Catalog(args.root), args.robot_id, args.workflow, read_json(args.input))
            validate(result, read_json(args.root / "schemas/command.schema.json"))
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as out: out.write(canonical(result) + "\n")
            print("Created command, valid for 60 seconds; no network request sent.")
            return
        cfg = load_config(args.config)
        if args.command == "check-config":
            print(canonical({"valid": True, "hardware_tested": False, "robot_id": cfg["robot_id"],
                             "motion_commissioned": cfg["motion"]["commissioned"], "servo_commissioned": cfg["servo"]["commissioned"],
                             "allowed_workflows": cfg["allowed_workflows"]})); return
        if hasattr(args, "timeout"):
            from .configuration import number
            number(args.timeout, 1, 600, "timeout")
        state = Path(cfg["state_path"])
        state.parent.mkdir(parents=True, exist_ok=True)
        with instance_lock(state):
            store = Store(state, recover=True)
            try:
                if args.command == "journal": result = {"jobs": store.states(), "pending_events": len(store.pending(100000))}
                else: result = asyncio.run(hardware(args, cfg, store))
                print(json.dumps(result, ensure_ascii=False, indent=2))
                if result.get("status") == "RECOVERY_REQUIRED": raise SystemExit(1)
            finally: store.close()
    except (StepError, ValueError, OSError) as exc:
        print(canonical({"error_code": getattr(exc, "code", type(exc).__name__), "message": str(exc)}))
        raise SystemExit(1)
    except KeyboardInterrupt:
        print("Interrupted. Inspect journal and physical state before another actuator test.")
        raise SystemExit(130)


if __name__ == "__main__": main()
