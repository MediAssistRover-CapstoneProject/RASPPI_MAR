"""Regression: a SLAM subprocess must retain the sourced ROS and vendor Python paths."""
import ast
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]


class EnvironmentTests(unittest.TestCase):
    def environment(self,root):
        # Load the pure helper without requiring ROS launch on the test machine.
        source=ast.parse((ROOT/'pi/launch/mediassist_demo_slam.launch.py').read_text())
        helper=next(node for node in source.body if isinstance(node,ast.FunctionDef) and node.name=='engine_environment')
        namespace={'os':os}
        exec(compile(ast.Module(body=[helper],type_ignores=[]),'slam_environment','exec'),namespace)
        return namespace['engine_environment'](root)

    @unittest.skipIf(sys.flags.ignore_environment,'Embedded Python deliberately ignores PYTHONPATH; run this regression on Linux/Pi Python')
    def test_child_can_import_sourced_ros_and_vendor_packages(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);ros=root/'ros';vendor=root/'vendor'
            ros.mkdir();vendor.mkdir()
            (ros/'rclpy.py').write_text('ROS_TEST_MARKER = True\n')
            (vendor/'hiwonder_test_support.py').write_text('VENDOR_TEST_MARKER = True\n')
            with patch.dict(os.environ,{'PYTHONPATH':os.pathsep.join([str(ros),str(vendor)])}):
                environment={**os.environ,**self.environment(root)}
            result=subprocess.run([sys.executable,'-c','import rclpy,hiwonder_test_support; assert rclpy.ROS_TEST_MARKER and hiwonder_test_support.VENDOR_TEST_MARKER'],
                env=environment,cwd=directory,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_missing_and_repeated_paths_are_handled_without_empty_entries(self):
        with patch.dict(os.environ,{},clear=True):
            self.assertEqual(self.environment(ROOT)['PYTHONPATH'],str(ROOT/'pi'))
        with patch.dict(os.environ,{'PYTHONPATH':os.pathsep.join([str(ROOT/'pi'),'/opt/ros/site-packages','','/opt/ros/site-packages'])}):
            self.assertEqual(self.environment(ROOT)['PYTHONPATH'].split(os.pathsep),[str(ROOT/'pi'),'/opt/ros/site-packages'])


if __name__=='__main__':unittest.main()
