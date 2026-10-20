import argparse
import asyncio
import json
import os
import sqlite3
import uuid
from datetime import datetime, timezone
from contextlib import contextmanager, closing
from pathlib import Path
from .contracts import Catalog, read_json, digest, StepError
from .runtime import Runtime
from .store import Store
from .adapters import SimAdapter, SimRemoteAdapter, RemoteServices


@contextmanager
def instance_lock(path):
    """Only one runtime or reconciliation process may operate this journal."""
    path = Path(str(path) + ".lock")
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a+b")
    try:
        if os.name == "nt":
            import msvcrt
            if path.stat().st_size == 0: handle.write(b"0"); handle.flush()
            handle.seek(0)
            msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    finally:
        handle.close()


def reconcile_legacy(config_path, job_id, reason, state_path=None):
    """Resolve the same journal as configure_mqtt; never create a replacement DB."""
    from .configuration import load_config
    config_path = Path(config_path).resolve(strict=True)
    config = load_config(config_path)
    if not reason.strip():
        raise StepError("RECONCILIATION_REASON_REQUIRED")
    state = Path(state_path) if state_path is not None else Path(config["state_path"])
    if not state.is_absolute():
        state = config_path.parent.parent / state
    state = state.resolve(strict=True)
    with instance_lock(state):
        db = sqlite3.connect(state.as_uri() + "?mode=ro", uri=True)
        try:
            row = db.execute("SELECT status FROM jobs WHERE id=?", (job_id,)).fetchone()
            if row is None or row[0] != "RECOVERY_REQUIRED":
                raise StepError("JOB_NOT_RECOVERY_REQUIRED")
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            backup = state.with_name(state.name + ".before-reconcile-" + stamp + ".bak")
            with closing(sqlite3.connect(backup)) as snapshot:
                db.backup(snapshot)
            backup.chmod(0o600)
        finally:
            db.close()
        store = Store(state, recover=False)
        try:
            store.reconcile(job_id, reason.strip())
        finally:
            store.close()
    return {"state_path": str(state), "backup": str(backup),
            "job_id": job_id, "status": "CANCELLED"}


def main():
    p = argparse.ArgumentParser(description="MediAssist Pi runtime. Simulation is explicit; hardware defaults uncommissioned.")
    p.add_argument("--root", type=Path, default=Path.cwd(), help="Bundle directory containing schemas/workflows")
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("validate")
    sim = sub.add_parser("simulate")
    sim.add_argument("--workflow", required=True)
    sim.add_argument("--workflow-version", type=int, default=4)
    sim.add_argument("--input", type=Path)
    sim.add_argument("--events", type=Path)
    run = sub.add_parser("connect")
    run.add_argument("--config", type=Path, required=True)
    mode = run.add_mutually_exclusive_group()
    mode.add_argument("--simulate", action="store_true", help="Simulate physical I/O and business service results locally")
    mode.add_argument("--simulate-remote", action="store_true", help="Simulate physical I/O; obtain business service results from the connected backend")
    reconcile = sub.add_parser("reconcile")
    reconcile.add_argument("--config", type=Path, required=True)
    reconcile.add_argument("--job-id", required=True)
    reconcile.add_argument("--reason", required=True)
    reconcile.add_argument("--state-path", type=Path, help="Use legacy_state_path from mqtt.worker.json")
    fingerprint = sub.add_parser("map-hash")
    fingerprint.add_argument("yaml_path", type=Path)
    args = p.parse_args()
    if args.command == "map-hash":
        from .map_tools import map_fingerprint
        print(map_fingerprint(args.yaml_path)); return
    if args.command == "reconcile":
        print(json.dumps(reconcile_legacy(args.config, args.job_id, args.reason, args.state_path))); return
    catalog = Catalog(args.root)
    if args.command == "validate":
        from jsonschema import Draft202012Validator
        for st in catalog.types.values():
            Draft202012Validator.check_schema(st["input_schema"])
            Draft202012Validator.check_schema(st["output_schema"])
        print(json.dumps({"step_types": len(catalog.types), "workflows": len(catalog.workflows), "valid": True})); return
    if args.command == "simulate":
        async def sim_run():
            wf = catalog.get(args.workflow, args.workflow_version)
            store = Store(":memory:")
            runtime = Runtime(catalog, store, SimAdapter(), {"max_speed_mps": 0.15})
            job = {"job_id": str(uuid.uuid4()), "workflow_code": wf["code"], "workflow_version": wf["version"],
                   "workflow_hash": digest(wf), "input": read_json(args.input or args.root / "examples" / (wf["code"] + ".input.json"))}
            store.add_job(job)
            result = await runtime.run(job)
            print(json.dumps({"simulation": True, **result}, ensure_ascii=False))
            if args.events:
                args.events.parent.mkdir(parents=True, exist_ok=True)
                args.events.write_text("\n".join(json.dumps(e, ensure_ascii=False) for e in store.pending(10000)) + "\n", encoding="utf-8")
            store.close()
            if result["status"] != "SUCCEEDED": raise SystemExit(1)
        asyncio.run(sim_run()); return
    from .configuration import load_config
    simulation = args.command == "connect" and (args.simulate or args.simulate_remote)
    config = load_config(args.config, simulation=simulation)
    state = Path(config["state_path"])
    state.parent.mkdir(parents=True, exist_ok=True)
    with instance_lock(state):
        store = Store(state, recover=True)
        async def connect_run():
            from .bridge import Bridge
            remote = RemoteServices(allow_simulation=args.simulate_remote)
            if args.simulate_remote: adapter = SimRemoteAdapter(remote)
            elif args.simulate: adapter = SimAdapter()
            else:
                from .ros_adapter import RosAdapter
                adapter = RosAdapter(config, remote)
            runtime = Runtime(catalog, store, adapter, {"max_speed_mps": config["max_speed_mps"]})
            try:
                await Bridge(runtime, config, remote, args.root, os.environ.get("MEDIASSIST_ROBOT_TOKEN", "")).run()
            finally:
                await adapter.stop()
                if hasattr(adapter, "close"): adapter.close()
                store.close()
        asyncio.run(connect_run())


if __name__ == "__main__": main()
