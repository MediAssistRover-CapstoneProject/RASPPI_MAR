#!/bin/bash
set -eo pipefail
task_root=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
set -u
build_testing="${MAR_BUILD_TESTING:-ON}"
if [[ "$build_testing" != ON && "$build_testing" != OFF ]]; then
    echo 'MAR_BUILD_TESTING must be ON or OFF' >&2
    exit 1
fi
cmake -S "$task_root/pi/native/navigation_bt" -B "$task_root/pi/native/navigation_bt/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING="$build_testing"
cmake --build "$task_root/pi/native/navigation_bt/build" --parallel 1
if [[ "$build_testing" == ON ]]; then
    ctest --test-dir "$task_root/pi/native/navigation_bt/build" --output-on-failure
fi
