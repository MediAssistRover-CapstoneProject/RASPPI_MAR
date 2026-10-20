"""Offline regression tests; no ROS connection, signals, GPIO or motor commands."""
import copy
import importlib.util
from pathlib import Path
import subprocess
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'pi'), str(ROOT / 'tools')]
import robot_stack as stack
from mediassist.velocity_graph import check_velocity_graph
from mediassist.contracts import StepError


def graph():
    node = Mock()
    pubs = {'/safe': ['mediassist_safety', 'mediassist_motor_watchdog'],
            '/ros_robot_controller/set_motor': ['odom_publisher', 'mediassist_motor_watchdog']}
    node.get_publishers_info_by_topic.side_effect = lambda topic: [SimpleNamespace(node_name=n) for n in pubs.get(topic, [])]
    node.get_subscriptions_info_by_topic.return_value = [SimpleNamespace(node_name='odom_publisher')]
    node.get_node_names_and_namespaces.return_value = [('odom_publisher', '/')]
    node.get_subscriber_names_and_types_by_node.return_value = [('/safe', ['geometry_msgs/msg/Twist']),
        ('/mediassist/unused_cmd_vel', ['geometry_msgs/msg/Twist']), ('/mediassist/unused_app_cmd_vel', ['geometry_msgs/msg/Twist'])]
    return node, pubs


class GraphTests(unittest.TestCase):
    def test_valid_graph(self):
        node, _ = graph(); check_velocity_graph(node, {'safe_cmd_vel': '/safe'})

    def test_all_bypasses_including_unknown_and_renamed_input(self):
        for topic in ('/cmd_vel', '/controller/cmd_vel', '/app/cmd_vel', '/some_other_velocity'):
            node, _ = graph()
            node.get_subscriber_names_and_types_by_node.return_value.append((topic, ['geometry_msgs/msg/Twist']))
            with self.subTest(topic=topic), self.assertRaises(StepError) as ctx:
                check_velocity_graph(node, {'safe_cmd_vel': '/safe'})
            self.assertEqual(ctx.exception.code, 'CHASSIS_BYPASS_DETECTED')
        node, pubs = graph(); pubs['/mediassist/unused_app_cmd_vel'] = ['joystick']
        with self.assertRaises(StepError): check_velocity_graph(node, {'safe_cmd_vel': '/safe'})

    def test_duplicate_driver_gate_missing_watchdog_and_motor_bypass(self):
        node, _ = graph(); node.get_node_names_and_namespaces.return_value *= 2
        with self.assertRaises(StepError): check_velocity_graph(node, {'safe_cmd_vel': '/safe'})
        for topic, names in (('/safe', ['mediassist_safety']), ('/safe', ['mediassist_safety', 'mediassist_safety', 'mediassist_motor_watchdog']),
                             ('/ros_robot_controller/set_motor', ['odom_publisher']),
                             ('/ros_robot_controller/set_motor', ['odom_publisher', 'mediassist_motor_watchdog', 'rogue'])):
            node, pubs = graph(); pubs[topic] = names
            with self.subTest(names=names), self.assertRaises(StepError): check_velocity_graph(node, {'safe_cmd_vel': '/safe'})


class ProcessTests(unittest.TestCase):
    def test_optional_video_failure_does_not_stop_motion_stack(self):
        with tempfile.TemporaryDirectory() as directory:
            supervisor=stack.Supervisor(Path(directory))
            camera=Mock();camera.poll.return_value=1
            worker=Mock();worker.poll.return_value=None
            supervisor.children=[('camera',camera,None),('worker',worker,None)]
            supervisor.optional.add('camera')
            with patch.object(stack,'processes',return_value={}):supervisor.alive()
            self.assertIn('camera',supervisor.reported_optional)
            worker.poll.return_value=1
            with patch.object(stack,'processes',return_value={}),self.assertRaises(RuntimeError):supervisor.alive()

    def test_only_exact_vendor_bringup_matches(self):
        self.assertTrue(stack.vendor_bringup({'argv': ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'bringup', 'bringup.launch.py']}))
        for argv in (['echo', 'ros2 launch bringup bringup.launch.py'], ['ros2', 'launch', 'other', 'bringup.launch.py'], ['python3', 'startup_check']):
            self.assertFalse(stack.vendor_bringup({'argv': argv}))

    def test_descendants_exclude_unrelated_ros_and_shell(self):
        root = {'pid': 40, 'ppid': 1}
        table = {40: root, 41: {'pid': 41, 'ppid': 40}, 42: {'pid': 42, 'ppid': 41}, 99: {'pid': 99, 'ppid': 1}}
        self.assertEqual({p['pid'] for p in stack.descendants(root, table)}, {40, 41, 42})

    def test_pid_reuse_never_signals_other_process(self):
        old = {'pid': 123456, 'ticks': '10', 'argv': ['ros2']}
        with patch.object(stack, 'proc', return_value={**old, 'ticks': '11'}), patch.object(stack.os, 'kill') as kill:
            stack.send_verified(old, 15); kill.assert_not_called()

    def test_pid_one_refused(self):
        with self.assertRaises(RuntimeError): stack.send_verified({'pid': 1}, 15)

    def test_shutdown_tracks_exec_and_stubborn_orphan_without_signalling_unrelated_pid(self):
        old = {'pid': 40000, 'ppid': 1, 'ticks': '10', 'argv': ['ros2', 'launch']}
        active = {40000: {**old, 'argv': ['python3', 'ros2', 'launch']},
                  40001: {'pid': 40001, 'ppid': 40000, 'ticks': '11', 'argv': ['slam-node']},
                  49999: {'pid': 49999, 'ppid': 1, 'ticks': '99', 'argv': ['unrelated']}}
        clock = [0.]
        sent = []
        def kill(pid, sig):
            sent.append((pid, sig))
            if pid == 40000:
                active.pop(pid, None)
                active[40001]['ppid'] = 1
            elif sig == signal.SIGKILL:
                active.pop(pid, None)
        with patch.object(stack.signal, 'SIGKILL', 9, create=True), \
                patch.object(stack, 'processes', side_effect=lambda: dict(active)), \
                patch.object(stack, 'proc', side_effect=lambda pid: active.get(pid)), \
                patch.object(stack.os, 'kill', side_effect=kill), \
                patch.object(stack.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(stack.time, 'sleep', side_effect=lambda value: clock.__setitem__(0, clock[0]+value)):
            stack.terminate_tree(old, grace=.2)
        self.assertEqual(sent, [(40000, signal.SIGINT), (40001, signal.SIGTERM), (40001, 9)])
        self.assertEqual(set(active), {49999})

    def test_shutdown_never_follows_descendants_of_reused_root_pid(self):
        old = {'pid': 40000, 'ppid': 1, 'ticks': '10', 'argv': ['ros2']}
        active = {40000: {**old, 'ticks': '20'},
                  40001: {'pid': 40001, 'ppid': 40000, 'ticks': '21', 'argv': ['unrelated-child']}}
        with patch.object(stack, 'processes', return_value=active), \
                patch.object(stack, 'proc', side_effect=lambda pid: active.get(pid)), \
                patch.object(stack.os, 'kill') as kill:
            stack.terminate_tree(old, grace=0)
        kill.assert_not_called()

    def test_explicit_leftover_stop_rejects_unrelated_processes(self):
        with patch.object(stack, 'proc', return_value=None), patch.object(stack, 'terminate_tree') as terminate:
            stack.stop_leftover(40000)
            terminate.assert_not_called()
        for argv in (['bash'], ['python3', '/another/project/slam.launch.py']):
            info = {'pid': 40000, 'ppid': 1, 'ticks': '10', 'argv': argv}
            with self.subTest(argv=argv), patch.object(stack, 'proc', return_value=info), \
                    patch.object(stack, 'terminate_tree') as terminate:
                with self.assertRaises(RuntimeError): stack.stop_leftover(40000)
                terminate.assert_not_called()
        info = {'pid': 40000, 'ppid': 1, 'ticks': '10',
                'argv': ['ros2', 'launch', str(stack.ROOT / 'pi/launch/mediassist_demo_slam.launch.py')]}
        with patch.object(stack, 'proc', return_value=info), patch.object(stack, 'terminate_tree') as terminate:
            stack.stop_leftover(40000)
            terminate.assert_called_once_with(info, grace=5.)
        info['argv'] = ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'slam', 'slam.launch.py']
        with patch.object(stack, 'proc', return_value=info), patch.object(stack, 'terminate_tree') as terminate:
            stack.stop_leftover(40000)
            terminate.assert_called_once_with(info, grace=5.)

    def test_stationary_run_keeps_motion_flags_and_single_slam_owner(self):
        cfg = {'mapping': {'enabled': True, 'launch_argv': [], 'control_source': 'mqtt'}}
        hw = {'motion': {k: True for k in ('commissioned', 'cart_footprint_validated', 'watchdog_validated')}}
        stack.validate_mode('run', cfg, hw)
        for key in hw['motion']:
            bad = copy.deepcopy(hw); bad['motion'][key] = False
            stack.validate_mode('prepare', cfg, bad)
            stack.validate_mode('run', cfg, bad)
            self.assertFalse(bad['motion'][key])
        cfg['mapping']['launch_argv'] = ['ros2', 'launch', 'slam']
        with self.assertRaises(RuntimeError): stack.validate_mode('run', cfg, hw)

    def test_failure_is_not_silently_restarted(self):
        supervisor = stack.Supervisor(Path('.'))
        child = Mock(); child.poll.return_value = 1; child.returncode = 1
        supervisor.children = [('us100', child, None)]
        with patch.object(stack, 'processes', return_value={}), self.assertRaisesRegex(RuntimeError, 'us100 exited'):
            supervisor.alive()

    def test_shutdown_keeps_gate_and_watchdog_until_after_driver(self):
        supervisor = stack.Supervisor(Path('.'))
        supervisor.children = [(name, Mock(), {'pid': i + 100}) for i, name in enumerate(('watchdog', 'gate', 'us100', 'slam', 'worker'))]
        stopped = []
        with patch.object(stack, 'terminate_tree', side_effect=lambda root, **kw: stopped.append(root['pid'])), patch.object(stack, 'say'):
            supervisor.close()
        self.assertEqual(stopped, [104, 103, 102, 101, 100])

    def test_us100_direct_execution_can_import_project_without_pythonpath(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / 'not-present.json'
            result = subprocess.run([sys.executable, str(ROOT / 'pi/scripts/us100_dual_ros2.py'), '--config', str(missing)],
                                    cwd=directory, capture_output=True, text=True)
        self.assertNotIn("No module named 'mediassist'", result.stderr)
        self.assertIn('FileNotFoundError', result.stderr)


if __name__ == '__main__': unittest.main(verbosity=2)
