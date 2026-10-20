"""Usage: python -m mediassist.worker_cli run --config /absolute/worker.json"""
import argparse
import asyncio
import logging
import os
import json
import signal
import sqlite3
from pathlib import Path
from .cli import instance_lock
from .contracts import StepError
from .worker_config import load
from .step_store import StepStore

def main():
    if os.name=="posix":
        os.umask(0o077)
    p=argparse.ArgumentParser(description="MAR-STEP-2 MQTT robot worker; BE owns workflows")
    sub=p.add_subparsers(dest="action",required=True)
    for name in ("run","reconcile"):
        q=sub.add_parser(name)
        q.add_argument("--config",type=Path,required=True)
        if name=="reconcile":
            q.add_argument("--job-step-id",required=True)
            q.add_argument("--reason",required=True)
    args=p.parse_args()
    logging.basicConfig(level=logging.INFO,format="%(asctime)s %(levelname)s %(message)s")
    cfg,hardware=load(args.config)
    Path(cfg["state_path"]).parent.mkdir(parents=True,exist_ok=True)
    # Same lock as the old runtime: never allow both to control the chassis.
    with instance_lock(cfg["legacy_state_path"]):
        legacy=Path(cfg["legacy_state_path"])
        if legacy.exists():
            db=sqlite3.connect(legacy.as_uri()+"?mode=ro",uri=True)
            try:
                if db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='jobs'").fetchone():
                    if db.execute("SELECT 1 FROM jobs WHERE status IN ('ACCEPTED','RUNNING','RECOVERY_REQUIRED') LIMIT 1").fetchone():
                        raise StepError("LEGACY_JOB_REQUIRES_RECONCILIATION")
            finally:
                db.close()
        with instance_lock(cfg["state_path"]):
            store=StepStore(cfg["state_path"],cfg["robot_id"])
            pid_file=Path(cfg["state_path"]).with_suffix(".pid.json")
            try:
                if args.action=="reconcile":
                    if not args.reason.strip():
                        raise StepError("RECONCILIATION_REASON_REQUIRED")
                    store.reconcile(args.job_step_id,args.reason)
                    print("Local step reconciled; BE reconciliation is still required.")
                    return
                if os.name != "posix":
                    raise StepError("RUN_INSIDE_MENTORPI_CONTAINER")
                ticks=Path("/proc/self/stat").read_text().rsplit(")",1)[1].split()[19]
                pid_file.write_text(json.dumps({"pid":os.getpid(),"startTicks":ticks,
                    "config":str(args.config.resolve())}),encoding="utf-8")
                async def run():
                    from .worker_adapter import WorkerAdapter
                    from .step_worker import StepWorker
                    adapter=WorkerAdapter(hardware,cfg)
                    worker=StepWorker(cfg,hardware,store,adapter)
                    loop=asyncio.get_running_loop()
                    task=asyncio.current_task()
                    for sig in (signal.SIGINT,signal.SIGTERM):
                        loop.add_signal_handler(sig,task.cancel)
                    try:
                        await worker.run()
                    finally:
                        await worker.disconnect("PROCESS_SHUTDOWN")
                        adapter.close()
                try:
                    asyncio.run(run())
                except (asyncio.CancelledError,KeyboardInterrupt):
                    logging.info("Worker stopped; review journal for interrupted steps.")
            finally:
                if args.action=="run":
                    pid_file.unlink(missing_ok=True)
                store.close()

if __name__=="__main__":
    try:
        main()
    except (StepError,ValueError,FileNotFoundError) as exc:
        raise SystemExit(getattr(exc,"code",str(exc)))
