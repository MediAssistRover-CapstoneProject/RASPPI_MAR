#!/usr/bin/env python3
"""Read-only ROS inventory. Run inside MentorPi after sourcing its ROS environment.

Does not launch stacks, publish commands, call actuator services, or change settings.
This report does not certify readiness to move. --output writes a local report only.
"""
import argparse
import datetime
import os
from pathlib import Path
import shutil
import subprocess
import sys


def run_command(args, timeout=9):
    try:
        result = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=timeout,
                                errors='replace', check=False)
        return result.returncode, result.stdout.strip()
    except subprocess.TimeoutExpired as error:
        data = error.stdout or ''
        if isinstance(data, bytes):
            data = data.decode('utf-8', errors='replace')
        return 124, data.strip() + '\nTIMEOUT: no complete response within limit.'
    except OSError as error:
        return 127, str(error)


def collect():
    lines = ['MediAssist read-only preflight',
             datetime.datetime.now(datetime.timezone.utc).isoformat(),
             'No movement commands are issued. Missing optional interfaces are not a hardware diagnosis.']
    for key in ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                'ROS_LOCALHOST_ONLY', 'MACHINE_TYPE', 'LIDAR_TYPE',
                'DEPTH_CAMERA_TYPE', 'need_compile'):
        lines.append(key + '=' + os.environ.get(key, '<unset>'))
    ros = shutil.which('ros2')
    lines.append('ros2=' + str(ros))
    lines.append('python=' + sys.executable)
    lines.append('python_version=' + sys.version.split()[0])
    if not ros:
        lines.append('STOP: enter MentorPi container and source /home/ubuntu/.zshrc first.')
        return 2, '\n'.join(lines)
    checks = [
        ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '3'],
        ['ros2', 'topic', 'list', '-t', '--no-daemon', '--spin-time', '3'],
        ['ros2', 'action', 'list', '-t'],
        ['ros2', 'pkg', 'prefix', 'nav2_msgs'],
        ['ros2', 'pkg', 'prefix', 'ros_robot_controller_msgs'],
        ['ros2', 'pkg', 'prefix', 'mentorpi_description'],
    ]
    topic_output = ''
    incomplete = False
    for command in checks:
        rc, output = run_command(command)
        lines.extend(['\n$ ' + ' '.join(command), 'exit=' + str(rc), output])
        incomplete = incomplete or rc != 0
        if command[1:3] == ['topic', 'list'] and rc == 0:
            topic_output = output
    topic_names = {line.split()[0] for line in topic_output.splitlines() if line.startswith('/')}
    for topic in ('/scan_raw', '/map', '/amcl_pose', '/tf', '/tf_static',
                  '/controller/cmd_vel', '/cmd_vel', '/ros_robot_controller/battery',
                  '/ultrasonic/left/range', '/ultrasonic/right/range',
                  '/vocal_detect/wakeup', '/vocal_detect/asr_result'):
        lines.append('interface ' + topic + ': ' + ('DISCOVERED' if topic in topic_names else 'NOT_DISCOVERED'))
    lines.append('\nDiscovery does not prove fresh sensor data, stable TF, actuator feedback, or Nav2 success.')
    return (1 if incomplete else 0), '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Write a UTF-8 report; refuses to overwrite.')
    args = parser.parse_args()
    code, report = collect()
    print(report)
    if args.output:
        try:
            with args.output.open('x', encoding='utf-8') as handle:
                handle.write(report + '\n')
        except OSError as error:
            print('Cannot write report: ' + str(error), file=sys.stderr)
            return 2
    return code


if __name__ == '__main__':
    raise SystemExit(main())
