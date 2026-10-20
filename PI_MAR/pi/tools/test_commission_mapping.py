"""Offline tests for commissioning records/configuration; no ROS or motor access."""
import copy
from contextlib import nullcontext
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'pi'), str(ROOT / 'tools')]
import commission_mapping as cm
from mediassist.safety_node import bench_pause_allowed
from mediassist.safety import gate_velocity
from mediassist.watchdog import stop_required
from mediassist.contracts import StepError


class CommissionTests(unittest.TestCase):
    def setUp(self):
        self.hardware = json.loads((ROOT / 'pi/config/site.example.json').read_text())
        self.hardware['motion'].update(commissioned=False, cart_footprint_validated=False, watchdog_validated=False)
        self.cfg = {'hardware_profile': '/original/profile.json', 'state_path': '/worker.sqlite', 'legacy_state_path': '/legacy.sqlite',
                    'mqtt': {'host': 'unchanged', 'password_file': '/secrets/password'},
                    'mapping': {'enabled': False, 'launch_argv': ['old'], 'upload_base_url': 'https://example.invalid/upload',
                                'upload_token_file': '/secrets/token', 'upload_ca_file': '/secrets/ca'}}
        self.context = {'run_id': 'test-run', 'tests': {k: {'observed_pass': True} for k in cm.TESTS}}

    def test_missing_or_failed_physical_observation_blocks_activation(self):
        for key in cm.TESTS:
            bad = copy.deepcopy(self.context); bad['tests'].pop(key)
            with self.subTest(test=key), self.assertRaises(RuntimeError): cm.require_observations(bad)
            bad['tests'][key] = {'observed_pass': False}
            with self.assertRaises(RuntimeError): cm.require_observations(bad)
            bad['tests'][key] = {'observed_pass': 1}
            with self.assertRaises(RuntimeError): cm.require_observations(bad)

    def test_activation_preserves_credentials_state_gpio_servo_and_upload(self):
        original = copy.deepcopy((self.cfg, self.hardware))
        worker, hw = cm.activated_profiles(self.cfg, self.hardware, self.context)
        self.assertEqual((self.cfg, self.hardware), original)
        for key in ('mqtt', 'state_path', 'legacy_state_path'):
            self.assertEqual(worker[key], self.cfg[key])
        for key in ('upload_base_url', 'upload_token_file', 'upload_ca_file'):
            self.assertEqual(worker['mapping'][key], self.cfg['mapping'][key])
        for key in ('servo', 'us100', 'ros', 'state_path'):
            self.assertEqual(hw[key], self.hardware[key])
        self.assertTrue(worker['mapping']['enabled'])
        self.assertEqual(worker['mapping']['launch_argv'], [])
        self.assertTrue(all(hw['motion'][k] for k in ('commissioned', 'cart_footprint_validated', 'watchdog_validated')))
        self.assertLessEqual(hw['max_speed_mps'], .05)
        self.assertEqual(hw['allowed_workflows'], [])

    def test_activation_does_not_invent_geometry_measurements(self):
        _, hw = cm.activated_profiles(self.cfg, self.hardware, self.context)
        record = hw['navigation']['mapping_commissioning']
        self.assertFalse(record['geometry_measured_by_activation'])
        self.assertNotIn('overall_length_m', record)
        self.assertNotIn('overall_width_m', record)
        self.assertEqual(record['observations'], self.context['tests'])

    def test_bench_does_not_change_real_profile_or_worker(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); config = root / 'pi/config/mqtt.worker.json'
            profile = root / 'original.json'; cm.save_json(profile, self.hardware)
            cfg = copy.deepcopy(self.cfg); cfg['hardware_profile'] = str(profile); cm.save_json(config, cfg)
            before_config, before_profile = config.read_bytes(), profile.read_bytes()
            directory = root / 'pi/state/stack'
            candidate, path, record = cm.create_bench(config, cfg, self.hardware, directory)
            self.assertEqual(config.read_bytes(), before_config)
            self.assertEqual(profile.read_bytes(), before_profile)
            self.assertFalse(record['ready'])
            self.assertEqual(record['tests'], {})
            self.assertLessEqual(candidate['max_speed_mps'], .03)
            self.assertIn('TEMPORARY WHEELS-OFF-GROUND', candidate['navigation']['operator_test_notes'])
            cm.context_for(config)
            cm.save_json(profile, {**self.hardware, 'new': 'change'})
            with self.assertRaisesRegex(RuntimeError, 'BENCH_CONFIG_CHANGED'): cm.context_for(config)

    def test_modified_bench_profile_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); config = root / 'pi/config/mqtt.worker.json'
            profile = root / 'original.json'; cm.save_json(profile, self.hardware)
            cfg = copy.deepcopy(self.cfg); cfg['hardware_profile'] = str(profile); cm.save_json(config, cfg)
            candidate, path, record = cm.create_bench(config, cfg, self.hardware, root / 'pi/state/stack')
            candidate['max_speed_mps'] = .3; cm.save_json(Path(path), candidate)
            with self.assertRaisesRegex(RuntimeError, 'BENCH_CONFIG_CHANGED'): cm.context_for(config)

    def test_pause_signal_is_only_enabled_for_bench_gate(self):
        hw = copy.deepcopy(self.hardware)
        self.assertFalse(bench_pause_allowed(hw, False))
        hw['max_speed_mps'] = .03
        hw['allowed_workflows'] = []
        hw['navigation']['operator_test_notes'] = 'TEMPORARY WHEELS-OFF-GROUND TEST'
        self.assertTrue(bench_pause_allowed(hw, False))
        self.assertFalse(bench_pause_allowed(hw, True))
        hw['max_speed_mps'] = .05
        self.assertFalse(bench_pause_allowed(hw, False))

    def test_commands_and_lease_expiry_stop_without_disabling_gate(self):
        config = copy.deepcopy(self.hardware['motion'])
        config.update(commissioned=True, cart_footprint_validated=True, watchdog_validated=True,
                      command_timeout_sec=.3, lease_timeout_sec=.4, sensor_max_age_sec=.6, stop_distance_m=.25)
        clear = [(1., 2.), (1., 2.)]
        cmd = (.03, 0., 0.)
        self.assertEqual(gate_velocity(cmd, clear, 2., 2., 2., 2., config, .03)[1], 'CLEAR')
        self.assertEqual(gate_velocity(cmd, clear, 2., 1., 2., 2., config, .03)[0], (0., 0., 0.))
        self.assertEqual(gate_velocity(cmd, clear, 2., 2., 1., 2., config, .03)[0], (0., 0., 0.))
        self.assertTrue(stop_required(2., 1., True, 1., 2.))
        self.assertFalse(stop_required(2., 2., True, 2., 2.))

    def configured_change(self, folder):
        """A real validated worker, then the same .025 edit configure_mapping makes."""
        from mediassist.worker_config import load
        root = Path(folder)
        config = root / 'pi/config/mqtt.worker.json'
        source = root / 'original.json'
        cm.save_json(source, self.hardware)
        password = root / 'mqtt-password.txt'; password.write_text('local-test-placeholder')
        robot = '0e3f1083-2546-4f45-8004-5eda5cb75a6f'
        cfg = {'robot_id': robot, 'transport': 'mqtt5', 'hardware_profile': str(source),
               'state_path': str(root / 'state.sqlite'), 'legacy_state_path': str(root / 'legacy.sqlite'),
               'mqtt': {'host': 'broker.example.invalid', 'port': 8883, 'username': robot,
                        'password_file': str(password), 'be_timeout_sec': 15},
               'unlock_policy': 'DISABLED', 'compartments': {}, 'enabled_step_types': ['WAIT', 'EXECUTE_SENSING'],
               'ai': {'enabled': False, 'topic': '/detections', 'max_age_sec': 2},
               'mapping': {'enabled': False, 'storage_dir': str(root / 'maps'), 'launch_argv': [],
                           'map_topic': '/map', 'max_cells': 4000000, 'start_timeout_sec': 30,
                           'max_map_age_sec': 10}}
        cm.save_json(config, cfg)
        cfg, hw = load(config)
        directory = root / 'pi/state/stack'
        _, _, record = cm.create_bench(config, cfg, hw, directory)
        record['tests'] = copy.deepcopy(self.context['tests'])
        cm.save_json(directory / 'bench.json', record)
        backup = config.with_name('mqtt.worker.20261020.bak')
        backup.write_bytes(config.read_bytes())
        hw['motion']['stop_distance_m'] = .025
        target = config.parent / 'site.motion.test.json'; cm.save_json(target, hw)
        cfg['hardware_profile'] = str(target)
        cfg['mapping']['enabled'] = True
        cfg['mapping']['control_source'] = 'mqtt'
        cfg['mapping']['camera']['enabled'] = True
        cm.save_json(config, cfg)
        return config, cfg, hw, directory, backup, record

    def test_reuses_pass_after_only_mapping_and_threshold_changes(self):
        with tempfile.TemporaryDirectory() as folder:
            config, cfg, hw, directory, backup, record = self.configured_change(folder)
            evidence = (directory / 'bench.json').read_bytes()
            with self.assertRaisesRegex(RuntimeError, 'BENCH_CONFIG_CHANGED'):
                cm.context_for(config)
            _, accepted = cm.context_for(config, reuse_stop_tests=True)
            review = accepted['activation_settings_review']
            self.assertEqual(review['tested_config_backup'], str(backup))
            self.assertEqual(review['tested_stop_distance_m'], .25)
            self.assertEqual(review['active_stop_distance_m'], .025)
            worker, active = cm.activated_profiles(cfg, hw, accepted)
            self.assertEqual(active['motion']['stop_distance_m'], .025)
            self.assertTrue(active['motion']['commissioned'])
            self.assertTrue(active['motion']['cart_footprint_validated'])
            self.assertTrue(active['motion']['watchdog_validated'])
            self.assertEqual(active['navigation']['mapping_commissioning']['settings_review'], review)
            self.assertEqual(worker['mqtt'], cfg['mqtt'])
            self.assertEqual((directory / 'bench.json').read_bytes(), evidence)
            self.assertEqual(accepted['worker_sha256'], record['worker_sha256'])

    def test_reuse_rejects_missing_matching_backup_or_observation(self):
        with tempfile.TemporaryDirectory() as folder:
            config, _, _, directory, backup, record = self.configured_change(folder)
            backup.unlink()
            with self.assertRaisesRegex(RuntimeError, 'BENCH_CONFIG_BACKUP_MISSING'):
                cm.context_for(config, True)
            record['tests'].pop('watchdog')
            cm.save_json(directory / 'bench.json', record)
            with self.assertRaisesRegex(RuntimeError, 'BENCH_OBSERVATIONS_MISSING'):
                cm.context_for(config, True)

    def test_reuse_still_rejects_source_or_bench_tampering(self):
        for key in ('source_hardware', 'bench_hardware'):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as folder:
                config, _, _, _, _, record = self.configured_change(folder)
                path = Path(record[key]); path.write_bytes(path.read_bytes() + b' ')
                with self.assertRaisesRegex(RuntimeError, 'BENCH_CONFIG_CHANGED'):
                    cm.context_for(config, True)

    def test_reuse_rejects_hardware_limits_timeout_gpio_servo_changes(self):
        mutations = (
            lambda h: h['motion'].update(command_timeout_sec=.4),
            lambda h: h['motion'].update(max_angular_speed_rps=.4),
            lambda h: h.update(max_speed_mps=.1),
            lambda h: h['us100']['left'].update(trig_bcm=27),
            lambda h: h['servo'].update(unlock_pulse_us=2200),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate), tempfile.TemporaryDirectory() as folder:
                config, cfg, hardware, _, _, _ = self.configured_change(folder)
                mutate(hardware); cm.save_json(Path(cfg['hardware_profile']), hardware)
                with self.assertRaises((RuntimeError, StepError)):
                    cm.context_for(config, True)

    def test_reuse_rejects_other_worker_changes(self):
        for name in ('mqtt', 'state_path', 'enabled_step_types'):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as folder:
                config, cfg, _, _, _, _ = self.configured_change(folder)
                if name == 'mqtt': cfg['mqtt']['host'] = 'another.example.invalid'
                elif name == 'state_path': cfg[name] = str(config.parent / 'different.sqlite')
                else: cfg[name].append('MOVE')
                cm.save_json(config, cfg)
                with self.assertRaisesRegex(RuntimeError, 'BENCH_CRITICAL_CONFIG_CHANGED'):
                    cm.context_for(config, True)

    def test_activation_detects_config_profile_or_evidence_edit_before_write(self):
        for name in ('config', 'profile', 'evidence'):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as folder:
                config, cfg, _, directory, _, _ = self.configured_change(folder)
                snapshot = cm.activation_snapshot(config)
                cm.unchanged_activation(config, snapshot, True)
                target = {'config': config, 'profile': Path(cfg['hardware_profile']),
                          'evidence': directory / 'bench.json'}[name]
                target.write_bytes(target.read_bytes() + b' ')
                with self.assertRaisesRegex(RuntimeError, 'ACTIVATION_CONFIG_CHANGED'):
                    cm.unchanged_activation(config, snapshot, True)

    def test_activation_writes_current_profile_and_preserves_original_evidence(self):
        from mediassist.worker_config import load
        with tempfile.TemporaryDirectory() as folder:
            config, cfg, _, directory, _, _ = self.configured_change(folder)
            old_config = config.read_bytes()
            evidence = (directory / 'bench.json').read_bytes()
            with patch.object(cm.sys.stdin, 'isatty', return_value=False), \
                    patch('robot_stack.processes', return_value={}), \
                    patch('robot_stack.locked', side_effect=lambda *_: nullcontext()), \
                    patch('builtins.input', side_effect=AssertionError('Activation must not prompt')):
                cm.enable_mapping(config, reuse_stop_tests=True)
            active_cfg, active_hw = load(config)
            self.assertNotEqual(active_cfg['hardware_profile'], cfg['hardware_profile'])
            self.assertEqual(active_hw['motion']['stop_distance_m'], .025)
            for key in ('commissioned', 'cart_footprint_validated', 'watchdog_validated'):
                self.assertTrue(active_hw['motion'][key])
            self.assertEqual((directory / 'bench.json').read_bytes(), evidence)
            backups = list(config.parent.glob('mqtt.worker.json.before-mapping-*.bak'))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_bytes(), old_config)
            # A second activation must not compare the activated profile to the
            # pre-activation bench profile or write yet another profile/backup.
            committed = config.read_bytes()
            with patch.object(cm, 'context_for', side_effect=AssertionError('Already enabled')), \
                    patch('builtins.input', side_effect=AssertionError('No prompts')):
                cm.enable_mapping(config, reuse_stop_tests=True)
            self.assertEqual(config.read_bytes(), committed)
            self.assertEqual(len(list(config.parent.glob('mqtt.worker.json.before-mapping-*.bak'))), 1)

    def test_changed_paths_reports_field_names_without_secret_values(self):
        paths = cm.changed_paths({'mqtt': {'password': 'old-secret'}, 'motion': {'commissioned': False}},
                                 {'mqtt': {'password': 'new-secret'}, 'motion': {'commissioned': True}})
        self.assertEqual(paths, ['motion.commissioned', 'mqtt.password'])
        self.assertNotIn('secret', ','.join(paths))


if __name__ == '__main__': unittest.main(verbosity=2)
