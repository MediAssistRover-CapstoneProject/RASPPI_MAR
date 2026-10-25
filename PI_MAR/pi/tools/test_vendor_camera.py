"""Backend switching, port ownership and startup checks without Pi hardware."""
from contextlib import nullcontext
import copy
import json
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from unittest.mock import patch
from urllib.parse import parse_qs, urlsplit

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools'),str(ROOT/'pi/tools')]
from mediassist import vendor_camera as vendor
from mediassist.worker_config import camera_config
from mediassist.contracts import StepError
import configure_camera_backend as tool
import test_camera_config as fixtures
from check_camera_stream import jpeg_starts


class VendorCameraTests(unittest.TestCase):
    def test_existing_config_keeps_python_and_existing_url(self):
        c=camera_config({'camera':{'enabled':True}})
        self.assertEqual(c['stream_backend'],'python')
        self.assertEqual(c['stream_port'],8081)
        self.assertIn('mediassist.camera_stream',vendor.camera_argv('/python','/config',c))

    def test_vendor_defaults_cannot_advertise_ignored_port_or_bind(self):
        c={'enabled':True,'stream_backend':'web_video_server','stream_port':8080}
        camera_config({'camera':c})
        for key,value in [('stream_backend','unknown'),('stream_port',8082),('stream_bind','127.0.0.1')]:
            with self.subTest(key=key),self.assertRaises(StepError):
                camera_config({'camera':{**c,key:value}})

    def test_switch_preserves_driver_and_mqtt_limits_and_disables_optional_preview(self):
        with tempfile.TemporaryDirectory() as folder:
            config,_,_=fixtures.CameraConfigTests().fixture(folder)
            original=json.loads(config.read_bytes())
            original['mapping']['camera'].update(stream_enabled=True,stream_backend='python',stream_port=8081,stream_bind='0.0.0.0',mqtt_preview_enabled=True)
            before=json.dumps(original).encode()
            native=json.loads(tool.plan(before,'web_video_server'))
            expected=copy.deepcopy(original)
            expected['mapping']['camera'].update(stream_backend='web_video_server',stream_port=8080,mqtt_preview_enabled=False)
            self.assertEqual(native,expected)
            self.assertEqual(json.loads(tool.plan(json.dumps(native).encode(),'python')),original)

    def test_apply_checks_dependency_backs_up_and_preserves_driver_hardware(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(tool,'stopped_configuration',return_value=nullcontext()),patch.object(tool,'vendor_executable'),patch.object(tool,'check_port_available'):
            config,driver,hardware=fixtures.CameraConfigTests().fixture(folder)
            before=config.read_bytes(); driver_before=driver.read_bytes(); hardware_before=hardware.read_bytes()
            backup=tool.apply(config,'web_video_server')
            self.assertEqual(backup.read_bytes(),before)
            self.assertEqual(driver.read_bytes(),driver_before)
            self.assertEqual(hardware.read_bytes(),hardware_before)
            self.assertIsNone(tool.apply(config,'web_video_server'))

    def test_running_stack_or_busy_port_keeps_config_intact(self):
        with tempfile.TemporaryDirectory() as folder:
            config,_,_=fixtures.CameraConfigTests().fixture(folder)
            before=config.read_bytes()
            with patch.object(tool,'stopped_configuration',side_effect=RuntimeError('STOP_ROBOT_STACK_FIRST')):
                with self.assertRaisesRegex(RuntimeError,'STOP_ROBOT_STACK_FIRST'):tool.apply(config,'web_video_server')
            with patch.object(tool,'stopped_configuration',return_value=nullcontext()),patch.object(tool,'vendor_executable'),patch.object(tool,'check_port_available',side_effect=RuntimeError('CAMERA_PORT_8080_BUSY')):
                with self.assertRaisesRegex(RuntimeError,'CAMERA_PORT_8080_BUSY'):tool.apply(config,'web_video_server')
            self.assertEqual(config.read_bytes(),before)

    def test_missing_package_keeps_config_intact(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(tool,'stopped_configuration',return_value=nullcontext()),patch.object(tool,'vendor_executable',side_effect=RuntimeError('WEB_VIDEO_SERVER_NOT_INSTALLED')):
            config,_,_=fixtures.CameraConfigTests().fixture(folder)
            before=config.read_bytes()
            with self.assertRaisesRegex(RuntimeError,'NOT_INSTALLED'):tool.apply(config,'web_video_server')
            self.assertEqual(config.read_bytes(),before)

    def test_stream_url_uses_tested_mjpeg_parameters(self):
        c=camera_config({'camera':{'stream_backend':'web_video_server','stream_port':8080,'stream_max_width':640,'stream_max_height':480,'stream_jpeg_quality':60}})
        query=parse_qs(urlsplit(vendor.stream_path(c)).query)
        self.assertEqual(query,{'topic':[c['topic']],'type':['mjpeg'],'width':['640'],'height':['480'],'quality':['60']})
        self.assertIn('mediassist.vendor_camera',vendor.camera_argv('/python','/config',c))

    def test_executable_resolved_from_installed_package(self):
        with tempfile.TemporaryDirectory() as folder:
            exe=Path(folder)/'lib/web_video_server/web_video_server'
            with self.assertRaisesRegex(RuntimeError,'EXECUTABLE_MISSING'):vendor.vendor_executable(folder)
            exe.parent.mkdir(parents=True);exe.write_bytes(b'test')
            with patch.object(vendor.os,'access',return_value=True):self.assertEqual(vendor.vendor_executable(folder),exe)

    def test_live_listener_is_never_adopted_or_killed(self):
        # Isolate the real socket test from services using 8080 on the laptop.
        with socket.socket() as listener:
            if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):
                listener.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
            listener.bind(('127.0.0.1',0));listener.listen()
            port=listener.getsockname()[1]
            real_socket=socket.socket
            class Probe:
                def __enter__(self):self.sock=real_socket();return self
                def __exit__(self,*args):self.sock.close()
                def setsockopt(self,*args):self.sock.setsockopt(*args)
                def bind(self,address):self.sock.bind(('0.0.0.0',port))
                def settimeout(self,*args):self.sock.settimeout(*args)
                def connect_ex(self,address):return self.sock.connect_ex(('127.0.0.1',port))
            with patch.object(vendor.socket,'socket',return_value=Probe()),self.assertRaisesRegex(RuntimeError,'PORT_8080_BUSY'):
                vendor.check_port_available()

    def test_exec_same_native_binary_with_no_port_overrides(self):
        c=camera_config({'camera':{'enabled':True,'stream_backend':'web_video_server','stream_port':8080}})
        with patch.object(sys,'argv',['vendor_camera','--config','worker.json']),patch('mediassist.worker_config.load',return_value=({'mapping':{'camera':c}},{})),patch.object(vendor,'vendor_executable',return_value=Path('/opt/native')),patch.object(vendor,'check_port_available'),patch.object(vendor.os,'execv') as execute:
            vendor.main()
            native=str(Path('/opt/native'))
            execute.assert_called_once_with(native,[native])

    def test_frame_counter_handles_native_boundaries_and_split_jpegs(self):
        chunks=[b'--boundary\r\n\xff',b'\xd8payload\xff\xd9\r\n--boundary\r\n',b'\xff\xd8payload\xff\xd9']
        total=0;tail=b''
        for data in chunks:
            count,tail=jpeg_starts(tail,data);total+=count
        self.assertEqual(total,2)


if __name__=='__main__':unittest.main()
