#!/usr/bin/env zsh
# Run INSIDE MentorPi. No network credentials or ROS nodes are created by this wrapper.
source /home/ubuntu/.zshrc
set -e
root="${0:A:h:h}"
cd "$root/pi"
export PYTHONPATH="$root/pi${PYTHONPATH:+:$PYTHONPATH}"
exec "$root/pi/.venv/bin/python" -u "$root/tools/robot_stack.py" "$@"
