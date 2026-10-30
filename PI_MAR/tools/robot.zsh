#!/usr/bin/env zsh
# Run INSIDE MentorPi. No network credentials or ROS nodes are created by this wrapper.
if (( EUID == 0 )); then
    if ! command -v runuser >/dev/null 2>&1; then
        print -u2 'ROS_ENV_USER_REQUIRED: switch to ubuntu before starting the robot'
        exit 1
    fi
    # A PTY forwards Ctrl+C to the child terminal so STOP can finish normally.
    exec runuser --pty -u ubuntu -- /bin/zsh "${0:A}" "$@"
fi
source /home/ubuntu/.zshrc
set -e
# All project ROS nodes run in this container. Keep DDS discovery off shared
# campus Wi-Fi; MQTT and the camera HTTP server still use the normal network.
export ROS_LOCALHOST_ONLY=1
if ! command -v ros2 >/dev/null 2>&1; then
    print -u2 'ROS_ENV_NOT_READY: ros2 is missing; no robot process started'
    exit 1
fi
root="${0:A:h:h}"
cd "$root/pi"
export PYTHONPATH="$root/pi${PYTHONPATH:+:$PYTHONPATH}"
exec "$root/pi/.venv/bin/python" -u "$root/tools/robot_stack.py" "$@"
