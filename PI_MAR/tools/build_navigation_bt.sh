#!/bin/bash
set -eo pipefail
task_root=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
set -u
cmake -S "$task_root/pi/native/navigation_bt" -B "$task_root/pi/native/navigation_bt/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$task_root/pi/native/navigation_bt/build" --parallel 1
ctest --test-dir "$task_root/pi/native/navigation_bt/build" --output-on-failure
