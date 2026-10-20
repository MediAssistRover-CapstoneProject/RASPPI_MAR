"""Stop only the matching worker process recorded by worker_cli."""
import argparse
import json
import os
from pathlib import Path
import signal

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--config",type=Path,required=True)
    args=p.parse_args()
    cfg=json.loads(args.config.read_text())
    pid_file=Path(cfg['state_path']).with_suffix('.pid.json')
    if not pid_file.exists(): return
    record=json.loads(pid_file.read_text())
    pid=int(record["pid"])
    if pid<=1: raise SystemExit("Invalid worker PID")
    proc=Path("/proc")/str(pid)
    if not proc.exists(): return
    ticks=proc.joinpath("stat").read_text().rsplit(")",1)[1].split()[19]
    argv=proc.joinpath("cmdline").read_bytes().split(b"\0")
    if (ticks!=record["startTicks"] or b"mediassist.worker_cli" not in argv
        or os.fsencode(record["config"]) not in argv):
        raise SystemExit("PID identity changed; refusing to signal it")
    os.kill(pid,signal.SIGTERM)
if __name__=="__main__": main()
