"""Joystick state transitions and mapping arbitration; no ROS, USB or motor access."""
import copy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'pi'), str(ROOT/'tools')]
from mediassist.gamepad import GamepadInput, configuration, driver_argv
from mediassist.mapping import MappingManager
from mediassist.mapping_control import MappingControlLease, VelocityRamp
from mediassist.contracts import StepError
import robot_stack

SESSION = '22222222-2222-4222-8222-222222222222'
ROBOT = '11111111-1111-4111-8111-111111111111'
JOB = '33333333-3333-4333-8333-333333333333'
TRANSPORT = '44444444-4444-4444-8444-444444444444'


def pad():
    return GamepadInput(configuration({}))


def arm(item, session=SESSION, start=10.):
    for i in range(8):
        now = start + i*.05
        item.feed([0.]*6, now)
        item.current(session, True, now)
    assert item.armed
    return start + .35


class InputTests(unittest.TestCase):
    def test_starting_held_stick_never_moves_until_neutral(self):
        item = pad()
        for i in range(30):
            item.feed([0., 1., 0., 0., 0., 0.], i*.05)
            self.assertEqual(item.current(SESSION, True, i*.05), (0., 0.))
        arm(item)
        item.feed([1., 1., 0., 0., 0., 0.], 10.4)
        self.assertEqual(item.current(SESSION, True, 10.4), (.10, .35))

    def test_reverse_scaling_and_release_stop_without_ramp(self):
        item = pad(); arm(item)
        item.feed([-1., -1., 0., 0., 0., 0.], 10.4)
        self.assertEqual(item.current(SESSION, True, 10.4), (-.08, -.35))
        item.feed([0.]*6, 10.45)
        self.assertEqual(item.current(SESSION, True, 10.45), (0., 0.))

    def test_timeout_requires_new_neutral_even_if_stick_returns_held(self):
        item = pad(); arm(item)
        item.feed([0., 1.], 10.4)
        self.assertNotEqual(item.current(SESSION, True, 10.4), (0., 0.))
        self.assertEqual(item.current(SESSION, True, 10.7), (0., 0.))
        item.feed([0., 1.], 10.75)
        self.assertEqual(item.current(SESSION, True, 10.75), (0., 0.))
        arm(item, start=11.)

    def test_gap_in_input_is_detected_even_between_control_ticks(self):
        item = pad(); arm(item)
        item.feed([0., 1.], 11.)
        self.assertEqual(item.current(SESSION, True, 11.), (0., 0.))

    def test_autorepeating_held_command_has_bounded_run(self):
        item = pad(); arm(item)
        for i in range(65):
            now = 10.4 + i*.05
            item.feed([0., 1.], now)
            velocity = item.current(SESSION, True, now)
        self.assertEqual(velocity, (0., 0.))
        self.assertFalse(item.armed)

    def test_web_stop_and_new_session_never_resume_held_command(self):
        for kind in ('stop', 'session', 'blocked'):
            item = pad(); arm(item); item.feed([0., 1.], 10.4)
            item.current(SESSION, True, 10.4)
            if kind == 'stop': item.stop()
            if kind == 'blocked': item.current(SESSION, False, 10.41)
            session = ROBOT if kind == 'session' else SESSION
            item.feed([0., 1.], 10.45)
            self.assertEqual(item.current(session, True, 10.45), (0., 0.))
            self.assertFalse(item.armed)

    def test_invalid_axes_or_replayed_stamps_disarm(self):
        for axes in ([], [0.], [0., float('nan')], [0., float('inf')], [0., 2.]):
            item = pad(); arm(item); item.feed(axes, 10.4)
            self.assertEqual(item.current(SESSION, True, 10.4), (0., 0.))
            self.assertFalse(item.armed)
        item = pad(); arm(item); item.feed([0., 1.], 10.4, 100)
        item.feed([0., 1.], 10.45, 100)
        self.assertFalse(item.armed)

    def test_no_session_and_not_ready_always_zero(self):
        item = pad(); arm(item); item.feed([0., 1.], 10.4)
        self.assertEqual(item.current(None, True, 10.4), (0., 0.))
        self.assertEqual(item.current(SESSION, False, 10.45), (0., 0.))

    def test_signs_and_configuration_bounds(self):
        cfg = configuration({}); cfg.update(linear_sign=-1, angular_sign=-1)
        item = GamepadInput(cfg); arm(item); item.feed([1., 1.], 10.4)
        self.assertEqual(item.current(SESSION, True, 10.4), (-.08, -.35))
        for change in ({'timeout_sec': 5}, {'linear_axis': True}, {'forward_mps': .5},
                       {'angular_rps': float('nan')}, {'topic': '/joy'}, {'angular_axis': 1}):
            with self.subTest(change=change), self.assertRaises(StepError):
                configuration({'gamepad': change})

    def test_driver_uses_isolated_topic_and_periodic_input_only(self):
        argv = driver_argv({})
        self.assertIn('/joy:=/mediassist/gamepad/input', argv)
        self.assertIn('autorepeat_rate:=20.0', argv)
        self.assertNotIn('controller/cmd_vel', ' '.join(argv))
        robot_stack.validate_mode('run', {'mapping': {'enabled': True, 'launch_argv': [],
                                                     'control_source': 'gamepad'}}, {})


class MappingTests(unittest.IsolatedAsyncioTestCase):
    def manager(self):
        item = object.__new__(MappingManager)
        item.cfg = {'control_source': 'gamepad'}
        item.gamepad = GamepadInput(configuration(item.cfg))
        item.state = {'session_open': True, 'state': 'MAPPING', 'session_id': SESSION, 'job_id': JOB}
        item.remote_control = MappingControlLease(); item.lock = threading.Lock()
        item.manual = (0., 0., 0.); item.ramp = VelocityRamp(); item.last_graph_check = 0.
        node = Mock(); node.get_publishers_info_by_topic.return_value = [NS(node_name='mediassist_gamepad_joy')]
        item.adapter = NS(node=node, worker={'robot_id': ROBOT}, motion_uncertain=False,
                          cfg={'max_speed_mps': .15, 'motion': {'commissioned': True,
                               'cart_footprint_validated': True, 'watchdog_validated': True,
                               'max_angular_speed_rps': .5}},
                          health=lambda: {'sensors_ready': True},
                          telemetry=lambda: {'odom_speed': {'age_sec': .01}},
                          _check_graph=Mock(), lease_message=Mock())
        item.gamepad_status = Mock(); item.gamepad_report_at = 0.; item.gamepad_report = None
        item.velocity = Mock()
        return item

    def stop_packet(self, **changes):
        now = datetime.now(timezone.utc)
        return {'messageType': 'MAPPING_CONTROL', 'robotId': ROBOT, 'transportSessionId': TRANSPORT,
                'mappingSessionId': SESSION, 'jobId': JOB, 'commandId': ROBOT, 'sequenceNo': 1,
                'linearX': 0., 'angularZ': 0., 'durationMs': 400,
                'issuedAt': now.isoformat(), 'expiresAt': (now+timedelta(seconds=1)).isoformat(), **changes}

    def test_web_move_refused_but_scoped_stop_disarms(self):
        item = self.manager(); arm(item.gamepad)
        with self.assertRaises(StepError) as ctx:
            item.receive_control(self.stop_packet(linearX=.05), TRANSPORT, True, True)
        self.assertEqual(ctx.exception.code, 'MAPPING_CONTROL_SOURCE_GAMEPAD')
        item.receive_control(self.stop_packet(), TRANSPORT, False, False)
        self.assertFalse(item.gamepad.armed)
        with self.assertRaises(StepError):
            item.receive_control(self.stop_packet(mappingSessionId=ROBOT, sequenceNo=2), TRANSPORT, True, True)

    def test_bad_sensor_odom_or_multiple_publishers_requires_neutral(self):
        variants = ('sensor', 'odom', 'publishers', 'commissioning', 'busy', 'mqtt')
        for kind in variants:
            item = self.manager(); arm(item.gamepad); item.gamepad.feed([0., 1.], 10.4)
            if kind == 'sensor': item.adapter.health = lambda: {'sensors_ready': False}
            if kind == 'odom': item.adapter.telemetry = lambda: {'odom_speed': {'age_sec': 2.}}
            if kind == 'publishers': item.adapter.node.get_publishers_info_by_topic.return_value *= 2
            if kind == 'commissioning': item.adapter.cfg['motion']['commissioned'] = False
            with patch('mediassist.mapping.time.monotonic', return_value=10.4), \
                    patch.dict(sys.modules, {'std_msgs.msg': NS(String=lambda **kw: NS(**kw))}):
                velocity = item.gamepad_velocity(kind != 'mqtt', kind != 'busy')
            self.assertEqual(velocity, (0., 0.), kind)
            self.assertFalse(item.gamepad.armed)

    async def test_20hz_tick_uses_existing_ramp_and_same_lease_then_stops_on_release(self):
        item = self.manager(); arm(item.gamepad); item.gamepad.feed([0., 1.], 10.4)
        def twist(): return NS(linear=NS(x=0.), angular=NS(z=0.))
        with patch.dict(sys.modules, {'geometry_msgs.msg': NS(Twist=twist),
                                     'std_msgs.msg': NS(String=lambda **kw: NS(**kw))}), \
                patch('mediassist.mapping.time.monotonic', return_value=10.4) as clock:
            await item.tick(True, True)
            self.assertGreater(item.velocity.publish.call_args.args[0].linear.x, 0.)
            self.assertLess(item.velocity.publish.call_args.args[0].linear.x, .10)
            item.adapter._check_graph.assert_called_once()
            item.gamepad.feed([0., 0.], 10.45)
            clock.return_value = 10.45
            await item.tick(True, True)
            self.assertEqual(item.velocity.publish.call_args.args[0].linear.x, 0.)
            self.assertFalse(item.adapter.lease_message.call_args.args[0])

    def test_stale_source_timestamp_is_not_treated_as_new_input(self):
        item = self.manager(); arm(item.gamepad)
        item.adapter.node.get_clock.return_value.now.return_value.nanoseconds = 2_000_000_000
        item.on_gamepad(NS(header=NS(stamp=NS(sec=1, nanosec=0)), axes=[0., 1.]))
        self.assertFalse(item.gamepad.armed)
        self.assertIsNone(item.gamepad.received)


if __name__ == '__main__':
    unittest.main(verbosity=2)
