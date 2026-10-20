#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "Run inside the MentorPi Linux aarch64 container, not on Windows." >&2; exit 1
fi
python3 -c 'import sys; assert sys.version_info[:2] == (3,10), "Use Python 3.10 in MentorPi"'
cd "$root/pi"
if [[ ! -x .venv/bin/python ]]; then
  python3 "$root/offline/virtualenv.pyz" --no-download --no-periodic-update .venv
fi
.venv/bin/python -m pip install --no-index --find-links "$root/offline/wheels" -r "$root/offline/requirements-pi.lock.txt"
# ROS lives in the image, not in downloaded wheels.
.venv/bin/python -c 'from pathlib import Path; p=Path(".venv/pyvenv.cfg"); s=p.read_text(); p.write_text(s.replace("include-system-site-packages = false","include-system-site-packages = true"))'
echo "Offline dependencies installed. Source ROS before running the worker."
echo "Run from $root/pi with .venv/bin/python -m mediassist.worker_cli ..."
