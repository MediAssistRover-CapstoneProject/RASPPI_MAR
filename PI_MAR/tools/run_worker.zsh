#!/usr/bin/env zsh
set -e
source /home/ubuntu/.zshrc
set -e
root="${0:A:h:h}"
cd "$root/pi"
exec "$root/pi/.venv/bin/python" -m mediassist.worker_cli run --config "$root/pi/config/mqtt.worker.json"
