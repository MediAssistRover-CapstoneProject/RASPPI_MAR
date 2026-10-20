#!/usr/bin/env python3
"""Map-bound named Nav2 goals. ROS imports occur only for live operations."""
import argparse
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import sys
import tempfile
import time

TERMINAL = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def number(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('Coordinates/angles must be finite.')
    return value


def goal_dict(x, y, yaw):
    angle = math.radians(number(yaw)) / 2
    return {'pose': {'header': {'frame_id': 'map'}, 'pose': {
        'position': {'x': number(x), 'y': number(y), 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(angle), 'w': math.cos(angle)}}}}


def pose_values(pose, covariance):
    cov = [number(v) for v in covariance]
    if len(cov) != 36 or any(cov[i] < 0 for i in (0, 7, 35)):
        raise ValueError('Invalid AMCL covariance.')
    q = [number(getattr(pose.orientation, k)) for k in ('x', 'y', 'z', 'w')]
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-6 or abs(norm - 1.0) > 0.05:
        raise ValueError('AMCL quaternion is not normalized.')
    qx, qy, qz, qw = (v / norm for v in q)
    yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz)))
    return {'x': number(pose.position.x), 'y': number(pose.position.y),
            'yaw_deg': yaw, 'covariance': cov}


def require_map(args, data=None):
    if not args.map_name or not re.fullmatch(r'[A-Za-z0-9_-]+', args.map_name):
        raise ValueError('Supply --map-name with the saved map basename (without .yaml).')
    if data is not None and data['map_name'] != args.map_name:
        raise ValueError('Map mismatch: file is bound to ' + data['map_name'])


def load_data(path):
    with path.open(encoding='utf-8') as handle:
        data = json.load(handle)
    if (not isinstance(data, dict) or data.get('version') != 1 or data.get('frame_id') != 'map' or
            not isinstance(data.get('map_name'), str) or not re.fullmatch(r'[A-Za-z0-9_-]+', data['map_name']) or
            not isinstance(data.get('locations'), dict)):
        raise ValueError('Invalid locations file metadata.')
    for name, location in data['locations'].items():
        if not isinstance(name, str) or not name.strip() or not isinstance(location, dict):
            raise ValueError('Invalid location name.')
        goal_dict(location['x'], location['y'], location['yaw_deg'])
    return data


def write_data(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix=path.name + '.', delete=False) as handle:
            temporary = handle.name
            json.dump(data, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write('\n')
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


class RosClient:
    def __init__(self, timeout):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        self.ros, self.action_type, self.timeout = rclpy, NavigateToPose, timeout
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('mentorpi_locations')
        self.action = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')

    def close(self):
        self.action.destroy()
        self.node.destroy_node()
        if self.ros.ok():
            self.ros.shutdown()

    def wait(self, future, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while not future.done() and time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('ROS response timeout; outcome may be uncertain.')
        return future.result()

    def service(self, kind, name, request):
        client = self.node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise TimeoutError('Service unavailable: ' + name)
            return self.wait(client.call_async(request))
        finally:
            self.node.destroy_client(client)

    def validate_map(self, expected):
        from rcl_interfaces.srv import GetParameters
        request = GetParameters.Request()
        request.names = ['yaml_filename']
        response = self.service(GetParameters, '/map_server/get_parameters', request)
        filename = response.values[0].string_value if response.values else ''
        if PurePosixPath(filename.replace('\\', '/')).stem != expected:
            raise ValueError('Live map_server YAML does not match --map-name: ' + repr(filename))
        return filename

    def record(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        samples = []

        def receive(msg):
            age = (self.node.get_clock().now().nanoseconds -
                   (msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)) / 1e9
            try:
                value = pose_values(msg.pose.pose, msg.pose.covariance)
                if msg.header.frame_id != 'map' or not 0 <= age <= 3:
                    return
                if max(value['covariance'][0], value['covariance'][7]) > 0.04 or value['covariance'][35] > 0.07:
                    return
                samples.append(value)
            except (ValueError, AttributeError):
                samples.clear()

        subscription = self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', receive, 10)
        deadline = time.monotonic() + self.timeout
        try:
            while not samples and time.monotonic() < deadline:
                self.ros.spin_once(self.node, timeout_sec=0.1)
            if not samples:
                raise TimeoutError('No fresh low-covariance AMCL pose. Check RViz/clock; stationary AMCL may need a no-motion update. Nothing saved.')
            return samples[-1]
        finally:
            self.node.destroy_subscription(subscription)

    def record_clicked(self, yaw_deg):
        from geometry_msgs.msg import PointStamped
        points = []
        def receive(msg):
            age = (self.node.get_clock().now().nanoseconds -
                   (msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)) / 1e9
            if msg.header.frame_id == 'map' and 0 <= age <= 3:
                points.append({'x': number(msg.point.x), 'y': number(msg.point.y),
                               'yaw_deg': number(yaw_deg), 'source': 'RVIZ_CLICKED_POINT'})
        subscription = self.node.create_subscription(PointStamped, '/clicked_point', receive, 10)
        print('In RViz: Fixed Frame=map; click Publish Point then known free floor. No navigation goal is sent.')
        deadline = time.monotonic() + self.timeout
        try:
            while not points and time.monotonic() < deadline:
                self.ros.spin_once(self.node, timeout_sec=0.1)
            if not points: raise TimeoutError('No fresh map-frame /clicked_point received; nothing saved.')
            return points[-1]
        finally:
            self.node.destroy_subscription(subscription)

    def status(self, response):
        name = TERMINAL.get(response.status, 'NONTERMINAL/UNKNOWN')
        print('GoalStatus:', response.status, name)
        print('Nav2 result:', response.result)
        return response.status

    def cancel_own(self, handle, result_future):
        try:
            response = self.wait(handle.cancel_goal_async(), min(self.timeout, 5))
            print('Cancel return_code:', response.return_code)
            status = self.status(self.wait(result_future, min(self.timeout, 5)))
            if status not in TERMINAL:
                print('UNCERTAIN: goal is not terminal; robot stop is not confirmed.', file=sys.stderr)
        except (Exception, KeyboardInterrupt) as error:
            print('UNCERTAIN: cancellation/terminal status not confirmed:', error, file=sys.stderr)

    def go(self, values, goal_timeout):
        handle, sent, result_future = None, None, None
        try:
            if not self.action.wait_for_server(timeout_sec=self.timeout):
                raise TimeoutError('NavigateToPose action server unavailable; no goal sent.')
            goal = self.action_type.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            pose = values['pose']['pose']
            goal.pose.pose.position.x, goal.pose.pose.position.y = pose['position']['x'], pose['position']['y']
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = pose['orientation']['z'], pose['orientation']['w']
            sent = self.action.send_goal_async(goal)
            handle = self.wait(sent)
            if not handle.accepted:
                print('Goal REJECTED; no execution accepted.')
                return 2
            result_future = handle.get_result_async()
            return 0 if self.status(self.wait(result_future, goal_timeout)) == 4 else 2
        except (KeyboardInterrupt, Exception) as error:
            print('Interrupted/timeout:', error, file=sys.stderr)
            if handle is None and sent is not None:
                try:
                    handle = self.wait(sent, min(self.timeout, 5))
                except (TimeoutError, KeyboardInterrupt):
                    print('UNCERTAIN: goal acceptance unknown; use cancel --execute.', file=sys.stderr)
            if handle is not None and handle.accepted:
                self.cancel_own(handle, result_future or handle.get_result_async())
            return 130 if isinstance(error, KeyboardInterrupt) else 2

    def cancel(self):
        from action_msgs.srv import CancelGoal
        response = self.service(CancelGoal, '/navigate_to_pose/_action/cancel_goal', CancelGoal.Request())
        print('Cancel return_code:', response.return_code, 'goals_canceling:', len(response.goals_canceling))
        if not response.goals_canceling:
            print('No goals confirmed canceling; robot stop is not confirmed.')
            return 2 if response.return_code else 0
        kind = self.action_type.Impl.GetResultService
        for info in response.goals_canceling:
            request = kind.Request()
            request.goal_id = info.goal_id
            try:
                if self.status(self.service(kind, '/navigate_to_pose/_action/get_result', request)) not in TERMINAL:
                    raise TimeoutError('Nonterminal status')
            except TimeoutError:
                print('UNCERTAIN: terminal result/robot stop not confirmed.', file=sys.stderr)
                return 2
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, default=Path(__file__).with_suffix('.json'))
    parser.add_argument('--map-name', help='Saved map basename; required for record/go/xy.')
    parser.add_argument('--timeout', type=float, default=15, help='Discovery/record/service timeout seconds.')
    parser.add_argument('--goal-timeout', type=float, default=120, help='Execution timeout before cancel attempt.')
    commands = parser.add_subparsers(dest='command', required=True)
    record = commands.add_parser('record', help='Save a fresh low-covariance AMCL pose; validate stability in RViz.')
    record.add_argument('name')
    record.add_argument('--overwrite', action='store_true')
    clicked = commands.add_parser('record-click', help='Record RViz Publish Point with explicit heading; no movement.')
    clicked.add_argument('name')
    clicked.add_argument('--yaw-deg', required=True, type=float)
    clicked.add_argument('--overwrite', action='store_true')
    commands.add_parser('list', help='List local saved points; no ROS required.')
    go = commands.add_parser('go', help='Named goal; dry-run unless --execute.')
    go.add_argument('name')
    xy = commands.add_parser('xy', help='Map-frame x/y metres and yaw degrees; dry-run unless --execute.')
    for field in ('x', 'y', 'yaw_deg'):
        xy.add_argument(field, type=float)
    cancel = commands.add_parser('cancel', help='Cancel all NavigateToPose goals only with --execute.')
    for command in (go, xy, cancel):
        command.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    client = None
    try:
        if not 0 < number(args.timeout) <= 300 or not 0 < number(args.goal_timeout) <= 3600:
            raise ValueError('Timeouts must be positive and bounded (300/3600 seconds).')
        data = load_data(args.file) if args.command in ('list', 'go') or args.file.exists() and args.command in ('record', 'record-click') else None
        if args.command in ('record', 'record-click', 'go', 'xy') or args.map_name and data:
            require_map(args, data)
        if args.command == 'list':
            print(json.dumps(data, ensure_ascii=False, indent=2))
            return 0
        if args.command in ('record', 'record-click'):
            if not args.name.strip() or len(args.name) > 64:
                raise ValueError('Name must contain 1..64 characters.')
            data = data or {'version': 1, 'map_name': args.map_name, 'frame_id': 'map', 'locations': {}}
            if args.name in data['locations'] and not args.overwrite:
                raise ValueError('Location already exists; use --overwrite deliberately.')
            client = RosClient(args.timeout)
            client.validate_map(args.map_name)
            data['locations'][args.name] = client.record() if args.command == 'record' else client.record_clicked(args.yaw_deg)
            data['map_yaml'] = client.validate_map(args.map_name)
            write_data(args.file, data)
            print('Saved:', args.name, 'in', args.file)
            print('One fresh pose is not proof of stability/safety: validate stationary AMCL alignment and this location in RViz before using it as a goal.')
            return 0
        values = goal_dict(args.x, args.y, args.yaw_deg) if args.command == 'xy' else None
        if args.command == 'go':
            location = data['locations'][args.name]
            values = goal_dict(location['x'], location['y'], location['yaw_deg'])
        if not args.execute:
            print(json.dumps(values or {'cancel_all': '/navigate_to_pose'}, indent=2, allow_nan=False))
            print('DRY RUN: nothing sent. Validate map/initial pose/free-space in RViz; add --execute to send.')
            return 0
        client = RosClient(args.timeout)
        if args.command == 'cancel':
            return client.cancel()
        client.validate_map(args.map_name)
        return client.go(values, args.goal_timeout)
    except Exception as error:
        print('ERROR:', error, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('Interrupted; no terminal navigation/robot-stop confirmation.', file=sys.stderr)
        return 130
    finally:
        if client is not None:
            client.close()


if __name__ == '__main__':
    sys.exit(main())
