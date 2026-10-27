#!/usr/bin/env zsh
if (( EUID == 0 )); then
    if ! command -v runuser >/dev/null 2>&1; then
        print -u2 'ROS_ENV_USER_REQUIRED: switch to ubuntu before starting navigation'
        exit 1
    fi
    exec runuser --pty -u ubuntu -- /bin/zsh "${0:A}" "$@"
fi
source /home/ubuntu/.zshrc
set -e
if ! command -v ros2 >/dev/null 2>&1; then
    print -u2 'ROS_ENV_NOT_READY: ros2 is missing; no navigation process started'
    exit 1
fi
root="${0:A:h:h}"
cd "$root"
export PYTHONPATH="$root/pi${PYTHONPATH:+:$PYTHONPATH}"
exec "$root/pi/.venv/bin/python" -u "$root/tools/run_navigation_session.py" "$@"
