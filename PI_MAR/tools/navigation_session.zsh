#!/usr/bin/env zsh
source /home/ubuntu/.zshrc
set -e
root="${0:A:h:h}"
cd "$root"
export PYTHONPATH="$root/pi${PYTHONPATH:+:$PYTHONPATH}"
exec "$root/pi/.venv/bin/python" -u "$root/tools/run_navigation_session.py" "$@"
