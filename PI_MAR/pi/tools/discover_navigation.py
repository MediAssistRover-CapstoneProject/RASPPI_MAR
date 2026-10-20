"""Read installed launch files and package locations; start no nodes and move nothing."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('navigation-discovery.txt'))
    args = parser.parse_args()
    lines = ['READ ONLY: no launch, no motor command', json.dumps({k: os.getenv(k) for k in
             ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'MACHINE_TYPE', 'LIDAR_TYPE', 'need_compile')})]
    for package in ('slam', 'navigation', 'nav2_bringup', 'slam_toolbox', 'nav2_map_server', 'rviz2'):
        try:
            done = subprocess.run(['ros2', 'pkg', 'prefix', package], capture_output=True, text=True, timeout=10)
            lines += ['$ ros2 pkg prefix ' + package, done.stdout, done.stderr]
        except (OSError, subprocess.TimeoutExpired) as exc: lines.append(str(exc))
    for path in (
        '/home/ubuntu/.stop_ros.sh',
        '/home/ubuntu/ros2_ws/src/slam/launch/slam.launch.py',
        '/home/ubuntu/ros2_ws/src/slam/launch/include/robot.launch.py',
        '/home/ubuntu/ros2_ws/src/navigation/launch/navigation.launch.py',
        '/home/ubuntu/ros2_ws/src/peripherals/launch/teleop_key_control.launch.py',
    ):
        target = Path(path)
        lines.append('\nFILE: ' + path)
        lines.append(target.read_text(errors='replace') if target.is_file() else 'MISSING')
    text = '\n'.join(lines)
    args.output.write_text(text, encoding='utf-8')
    print(text)
    print('Saved:', args.output)


if __name__ == '__main__': main()
