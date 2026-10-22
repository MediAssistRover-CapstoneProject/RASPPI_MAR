"""Offline policy and installation tests. Never open ROS/GPIO or send motion."""
import ast
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))



SOURCE = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SOURCE / 'pi'), str(SOURCE / 'tools')]


def function(source, name, **bindings):
    tree = ast.parse(source)
    nodes = list(ast.walk(tree))
    node = next(n for n in nodes if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = {"math": math, **bindings}
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<offline>', 'exec'), namespace)
    return namespace[name]


class OptionalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sources = {relative: (SOURCE / relative).read_text(encoding='utf-8-sig')
                       for relative in ('pi/mediassist/safety.py', 'pi/mediassist/ros_adapter.py',
                                        'pi/mediassist/configuration.py', 'tools/stack_probe.py',
                                        'pi/tools/bench_motion.py')}

    def motion(self, **changes):
        value = dict(commissioned=True, cart_footprint_validated=True, watchdog_validated=True,
                     sensor_max_age_sec=.6, lease_timeout_sec=.4, command_timeout_sec=.3,
                     stop_distance_m=.025, max_angular_speed_rps=.5)
        value.update(changes)
        return value

    def gate(self, motion, ranges=None, **times):
        env = {}
        exec(self.sources['pi/mediassist/safety.py'], env)
        return env['gate_velocity']((.1, 0., .2), [(None, 0.), (float('nan'), 0.)] if ranges is None else ranges,
                                    100., times.get('command', 100.), times.get('lease', 100.),
                                    times.get('scan', 100.), motion, .05)

    def test_default_and_required_still_block_missing_echo(self):
        for config in (self.motion(), self.motion(us100_required=True)):
            self.assertEqual(self.gate(config), ((0., 0., 0.), 'STALE_OR_INVALID_US100'))

    def test_optional_missing_invalid_or_near_echo_does_not_block(self):
        for ranges in ([], [(None, 0.), (float('nan'), 0.)], [(.02, 100.), (.5, 0.)]):
            velocity, reason = self.gate(self.motion(us100_required=False), ranges)
            self.assertEqual(reason, 'CLEAR')
            self.assertAlmostEqual(velocity[0], .05)  # Speed cap still enforced.
            self.assertEqual(velocity[1:], (0., .2))

    def test_optional_keeps_lidar_command_and_lease_timeouts(self):
        for timing, reason in (({'scan': 99.}, 'STALE_LIDAR'),
                               ({'command': 99.}, 'STALE_COMMAND'), ({'lease': 99.}, 'NO_LEASE')):
            self.assertEqual(self.gate(self.motion(us100_required=False), **timing), ((0., 0., 0.), reason))

    def test_optional_does_not_enable_uncommissioned_motion(self):
        self.assertEqual(self.gate(self.motion(us100_required=False, commissioned=False))[1], 'NOT_COMMISSIONED')

    def health(self, policy, samples):
        method = function(self.sources['pi/mediassist/ros_adapter.py'], 'health')
        obj = SimpleNamespace(cfg={'motion': self.motion(**policy)}, telemetry=lambda: samples)
        return method(obj)

    def test_worker_optional_accepts_scan_without_faking_range_freshness(self):
        r = self.health({'us100_required': False}, {'scan': {'value': True, 'age_sec': .1}})
        self.assertTrue(r['sensors_ready'])
        self.assertFalse(r['left_fresh']); self.assertFalse(r['right_fresh'])
        self.assertIsNone(r['left_m']); self.assertIsNone(r['right_m'])

    def test_worker_default_requires_both_ranges(self):
        self.assertFalse(self.health({}, {'scan': {'value': True, 'age_sec': .1}})['sensors_ready'])
        samples = {'scan': {'value': True, 'age_sec': .1}, 'left': {'value': .5, 'age_sec': .1},
                   'right': {'value': .7, 'age_sec': .1}}
        self.assertTrue(self.health({}, samples)['sensors_ready'])

    def test_worker_optional_still_rejects_stale_missing_or_invalid_scan(self):
        for samples in ({}, {'scan': {'value': False, 'age_sec': .1}}, {'scan': {'value': True, 'age_sec': .7}}):
            self.assertFalse(self.health({'us100_required': False}, samples)['sensors_ready'])

    def test_policy_requires_a_real_boolean(self):
        class Error(ValueError):
            pass
        source = self.sources['pi/mediassist/configuration.py']
        number = function(source, 'number', StepError=Error)
        validate = function(source, 'validate_config', StepError=Error, number=number)
        profile = json.loads((SOURCE / 'pi/config/site.example.json').read_text())
        for value in (False, True):
            profile['motion']['us100_required'] = value
            validate(profile)
        for value in (None, 0, 1, 'false', 'true', [], {}):
            profile['motion']['us100_required'] = value
            with self.subTest(value=value), self.assertRaises(Error):
                validate(profile)

    def test_probe_optional_requires_scan_odom_gate_map_tf_but_not_us100(self):
        fake_time = SimpleNamespace(monotonic=lambda: 100.)
        method = function(self.sources['tools/stack_probe.py'], 'missing', time=fake_time)
        transform = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)))
        obj = SimpleNamespace(hardware={'motion': self.motion(us100_required=False), 'ros': {
            'map_frame': 'map', 'base_frame': 'base_link'}}, mapping={'max_map_age_sec': 2.},
            samples={k: 100. for k in ('scan', 'odom', 'gate', 'map')}, graph=lambda: None,
            buffer=SimpleNamespace(lookup_transform=lambda *args: transform),
            rclpy=SimpleNamespace(time=SimpleNamespace(Time=lambda: None)),
            node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000))))
        self.assertEqual(method(obj), [])
        obj.samples.pop('scan')
        self.assertIn('scan', method(obj))
        obj.hardware['motion']['us100_required'] = True
        self.assertTrue({'left', 'right'}.issubset(method(obj)))

    def test_bench_optional_still_requires_remaining_sensors(self):
        class Error(ValueError):
            pass
        check = function(self.sources['pi/tools/bench_motion.py'], 'check_test_sensors', StepError=Error)
        check({'sensors_ready': True, 'left_m': None, 'right_m': None}, .025, False)
        with self.assertRaises(Error):
            check({'sensors_ready': False}, .025, False)
        with self.assertRaises(Error):
            check({'sensors_ready': True, 'left_m': .02, 'right_m': .5}, .025)


class ConfigurationTests(unittest.TestCase):
    def test_change_preserves_other_settings_and_required_mode_restores_policy(self):
        import configure_us100 as tool
        import configure_mapping
        from mediassist.configuration import validate_config
        hardware = json.loads((SOURCE / 'pi/config/site.example.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'mqtt.worker.json'
            profile = Path(tmp) / 'original.json'
            profile.write_text(json.dumps(hardware))
            cfg = {'hardware_profile': str(profile), 'state_path': '/preserved/journal',
                   'mqtt': {'password_file': '/private/mqtt'}, 'speech': {'base_url': 'https://be.example'}}
            path.write_text(json.dumps(cfg))
            def read_config(p):
                candidate = json.loads(Path(p).read_text())
                return candidate, validate_config(json.loads(Path(candidate['hardware_profile']).read_text()))
            with patch.object(tool, 'load', side_effect=read_config), patch.object(configure_mapping, 'load', side_effect=read_config), patch.object(tool, 'stopped_configuration', return_value=contextlib.nullcontext()):
                output = tool.configure(path, 'optional')
                tuned = json.loads(Path(output['hardware_profile']).read_text())
                expected = copy.deepcopy(hardware)
                expected['motion']['us100_required'] = False
                self.assertEqual(tuned, expected)
                self.assertEqual(json.loads(profile.read_text()), hardware)
                self.assertEqual({k:v for k,v in output.items() if k!='hardware_profile'},
                                 {k:v for k,v in cfg.items() if k!='hardware_profile'})
                before = path.read_bytes()
                self.assertEqual(tool.configure(path, 'optional'), output)
                self.assertEqual(path.read_bytes(), before)
                restored = tool.configure(path, 'required')
                self.assertIs(json.loads(Path(restored['hardware_profile']).read_text())['motion']['us100_required'], True)

    def test_running_stack_refuses_update(self):
        import configure_us100 as tool
        with tempfile.NamedTemporaryFile() as f, patch.object(tool, 'stopped_configuration', side_effect=RuntimeError('STOP_ROBOT_STACK_FIRST')), patch.object(tool, 'write_configuration') as writer:
            with self.assertRaisesRegex(RuntimeError, 'STOP_ROBOT_STACK_FIRST'):
                tool.configure(f.name, 'optional')
            writer.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
