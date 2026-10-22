"""Offline HP60C configuration and rollback; no ROS, camera, GPIO or network."""
import ast
from contextlib import nullcontext
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'pi'), str(ROOT / 'tools')]
import configure_camera as tool
from mediassist.worker_config import camera_config, load
from mediassist.contracts import StepError

DRIVER = b'''# Preserve TF and depth features.
def generate_launch_description():
    return [Node(namespace="ascamera", package='ascamera', executable='ascamera_node',
                 parameters=[{"fps": 15}, {"rgb_width": 640}, {"rgb_height": 480},
                             {"depth_width": 640}, {"depth_height": 480},
                             {"color_pcl": True}, {"pub_tfTree": True}]),
            Node(package='tf2_ros', executable='static_transform_publisher', arguments=['-1.57'])]
'''


class CameraConfigTests(unittest.TestCase):
    def fixture(self, folder):
        folder = Path(folder)
        config_dir = folder / 'pi/config'
        config_dir.mkdir(parents=True)
        hardware = config_dir / 'hardware.json'
        hardware.write_bytes((ROOT / 'pi/config/site.example.json').read_bytes())
        password = folder / 'password.txt'
        password.write_text('test-only-password')
        robot = '11111111-1111-4111-8111-111111111111'
        cfg = {'robot_id': robot, 'transport': 'mqtt5', 'hardware_profile': str(hardware),
               'state_path': str(folder / 'journal.sqlite'), 'legacy_state_path': str(folder / 'legacy.sqlite'),
               'mqtt': {'host': 'broker.example', 'port': 8883, 'username': robot,
                        'password_file': str(password), 'be_timeout_sec': 15},
               'unlock_policy': 'DISABLED', 'compartments': {}, 'enabled_step_types': ['INTERACT'],
               'speech': {'enabled': False, 'device': 'auto'},
               'ai': {'enabled': False, 'topic': '/ai', 'max_age_sec': 2},
               'mapping': {'enabled': True, 'storage_dir': str(folder / 'maps'), 'launch_argv': [],
                           'map_topic': '/map', 'max_cells': 100000, 'start_timeout_sec': 30,
                           'max_map_age_sec': 10, 'camera': {'enabled': True, 'rate_hz': .5,
                           'max_width': 160, 'max_height': 120, 'jpeg_quality': 40}}}
        config = config_dir / 'mqtt.worker.json'
        config.write_text(json.dumps(cfg))
        driver = folder / 'ascamera.launch.py'
        driver.write_bytes(DRIVER)
        return config, driver, hardware

    def test_patch_only_changes_known_camera_literals_and_is_idempotent(self):
        result = tool.patch_driver(DRIVER, 20)
        self.assertEqual(result, DRIVER.replace(b'"fps": 15', b'"fps": 20'))
        self.assertEqual(tool.patch_driver(result, 20), result)
        self.assertEqual(tool.patch_driver(result, 15), DRIVER)

    def test_unknown_ambiguous_or_dynamic_parameters_fail_closed(self):
        for source in (b'not_camera = True', DRIVER + DRIVER,
                       DRIVER.replace(b'"fps": 15', b'"fps": variable'),
                       DRIVER.replace(b'"fps": 15', b'"set_fps": 15')):
            with self.subTest(source=source), self.assertRaises(RuntimeError):
                tool.patch_driver(source, 20)
        with self.assertRaises(ValueError):
            tool.patch_driver(DRIVER, 30)

    def test_legacy_camera_defaults_preserve_mqtt_and_stream_size(self):
        mapping = {'camera': {'max_width': 160, 'max_height': 120, 'jpeg_quality': 40}}
        camera = camera_config(mapping)
        self.assertEqual(camera['stream_max_width'], 160)
        self.assertEqual(camera['stream_max_height'], 120)
        self.assertEqual(camera['stream_jpeg_quality'], 40)
        self.assertEqual(camera['stream_rate_hz'], 15)
        for key, value in (('stream_max_width', True), ('stream_max_height', 481),
                           ('stream_jpeg_quality', 100), ('stream_max_frame_age_sec', float('nan'))):
            bad = copy.deepcopy(mapping)
            bad['camera'][key] = value
            with self.subTest(key=key), self.assertRaises(StepError):
                camera_config(bad)

    def test_readonly_plan_does_not_write_and_preserves_unrelated_config(self):
        with tempfile.TemporaryDirectory() as folder:
            config, driver, _ = self.fixture(folder)
            before = {p: p.read_bytes() for p in Path(folder).rglob('*') if p.is_file()}
            result = tool.plan(config, driver)
            self.assertEqual(before, {p: p.read_bytes() for p in Path(folder).rglob('*') if p.is_file()})
            cfg = json.loads(result[config][1])
            old = json.loads(before[config])
            current_camera = cfg['mapping'].pop('camera')
            previous_camera = old['mapping'].pop('camera')
            self.assertEqual(cfg, old)
            for key, value in previous_camera.items():
                self.assertEqual(current_camera[key], value)
            self.assertEqual(current_camera['stream_rate_hz'], 20)

    def test_apply_backs_up_files_keeps_hardware_and_supports_15fps_comparison(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(tool, 'stopped_configuration', return_value=nullcontext()):
            config, driver, hardware = self.fixture(folder)
            original_config, original_hardware = config.read_bytes(), hardware.read_bytes()
            backup = tool.apply(config, driver)
            manifest = json.loads((backup / 'manifest.json').read_text())
            self.assertEqual(len(manifest), 2)
            originals = {Path(item['path']): (backup / item['backup']).read_bytes() for item in manifest}
            self.assertEqual(originals[config], original_config)
            self.assertEqual(originals[driver], DRIVER)
            cfg, _ = load(config)
            self.assertEqual(cfg['mapping']['camera']['stream_rate_hz'], 20)
            self.assertEqual(cfg['mapping']['camera']['stream_max_width'], 320)
            self.assertEqual(cfg['mapping']['camera']['max_width'], 160)
            self.assertIsNone(tool.apply(config, driver))
            tool.apply(config, driver, fps=15, width=640, height=480)
            self.assertEqual(driver.read_bytes(), DRIVER)
            self.assertEqual(hardware.read_bytes(), original_hardware)

    def test_write_failure_restores_driver_and_worker_config(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(tool, 'stopped_configuration', return_value=nullcontext()):
            config, driver, _ = self.fixture(folder)
            before = config.read_bytes()
            original_write = tool.atomic_write
            def failing(path, raw, mode):
                if path == config:
                    raise OSError('injected failure')
                return original_write(path, raw, mode)
            with patch.object(tool, 'atomic_write', side_effect=failing), self.assertRaisesRegex(OSError, 'injected'):
                tool.apply(config, driver)
            self.assertEqual(config.read_bytes(), before)
            self.assertEqual(driver.read_bytes(), DRIVER)

    def test_active_stack_refuses_all_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            config, driver, _ = self.fixture(folder)
            before = config.read_bytes()
            with patch.object(tool, 'stopped_configuration', side_effect=RuntimeError('STOP_ROBOT_STACK_FIRST')):
                with self.assertRaisesRegex(RuntimeError, 'STOP_ROBOT_STACK_FIRST'):
                    tool.apply(config, driver)
            self.assertEqual(config.read_bytes(), before)
            self.assertEqual(driver.read_bytes(), DRIVER)
            self.assertFalse((Path(folder) / 'pi/state').exists())

    def test_source_path_matches_vendor_need_compile_switch(self):
        with patch.dict(tool.os.environ, {'DEPTH_CAMERA_TYPE': 'ascamera', 'need_compile': 'False'}):
            self.assertEqual(tool.vendor_launch(), Path('/home/ubuntu/ros2_ws/src/peripherals/launch/include/ascamera.launch.py'))
        with patch.dict(tool.os.environ, {'DEPTH_CAMERA_TYPE': 'usb_cam'}):
            with self.assertRaisesRegex(RuntimeError, 'HP60C_REQUIRED'):
                tool.vendor_launch()


if __name__ == '__main__':
    unittest.main()
