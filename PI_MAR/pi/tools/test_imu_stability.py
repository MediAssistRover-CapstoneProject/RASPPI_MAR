"""Offline IMU/launch regressions. These tests never connect to or move a robot."""
import ast
import gzip
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools')]
from mediassist.imu_stability import StationaryGyro
from vendor_hardware_launch import prepare,replace_include
import robot_stack

class ImuTests(unittest.TestCase):
    def samples(self,c,start,end,gyro=(.07,.03,-.004),moving=False,changing=False):
        for i in range(round(start*50),round(end*50)):
            t=i/50
            c.command((.1 if moving else 0,0,0),t)
            if i%5==0:c.scan([2+t*.15 if changing else 2]*360,t)
            out=c.feed(gyro,(0,0,9.81),t,t)
        return out

    def test_still_bias_removed_and_real_rotation_preserved(self):
        c=StationaryGyro();out=self.samples(c,0,4)
        self.assertEqual(c.phase,'READY');self.assertLess(max(map(abs,out)),1e-8)
        out=self.samples(c,4,5,gyro=(.07,.03,.396),moving=True)
        self.assertAlmostEqual(out[2],.4);self.assertEqual(c.phase,'READY')

    def test_no_calibration_with_motion_or_changing_scan(self):
        for kwargs in ({'moving':True},{'changing':True}):
            c=StationaryGyro();self.samples(c,0,5,**kwargs)
            self.assertIsNone(c.bias)
        c=StationaryGyro()
        for i in range(300):c.feed((0,0,0),(0,0,9.81),i/50,i/50)
        self.assertIsNone(c.bias)

    def test_calibration_resumes_after_motion_and_fault_does_not_rebias(self):
        c=StationaryGyro();self.samples(c,0,2,moving=True);self.samples(c,2,6)
        self.assertEqual(c.phase,'READY');before=c.bias
        self.samples(c,6,11,gyro=(.07,.03,.296))
        self.assertEqual(c.reason,'IMU_STATIONARY_DRIFT');self.assertEqual(c.bias,before)
        self.assertIsNone(self.samples(c,11,15));self.assertEqual(c.phase,'FAULT')

    def test_invalid_samples_replay_and_unmeasured_scan_cannot_calibrate(self):
        c=StationaryGyro();self.assertIsNone(c.feed((math.nan,0,0),(0,0,9.81),0,0))
        for i in range(250):
            t=i/50;c.command((0,0,0),t);c.scan([math.nan]*360,t)
            c.feed((0,0,0),(0,0,9.81),t,t)
        self.assertIsNone(c.bias)
        self.assertIsNone(c.feed((0,0,0),(0,0,9.81),10,1))
        self.assertEqual(c.reason,'IMU_TIMESTAMP_NOT_INCREASING')

    def test_recorded_stationary_lidar_and_raw_gyro(self):
        # Gyro and scan are recorded from the Pi. Acceleration was not recorded;
        # replay uses a stationary acceleration fixture, tested separately above.
        data=json.loads(gzip.decompress((Path(__file__).parent/'fixtures/imu_stationary_20261026.json.gz').read_bytes()))
        c=StationaryGyro();ready=None;outputs=[]
        events=sorted([(r[0],'g',r[1:]) for r in data['gyro']]+[(r[0],'s',r[1]) for r in data['scan']])
        for t,kind,values in events:
            c.command((0,0,0),t)
            if kind=='s':c.scan(values,t)
            else:
                out=c.feed(values,(0,0,9.81),t,t)
                if out is not None:outputs.append(out)
            if c.phase=='READY' and ready is None:ready=t
        self.assertIsNotNone(ready);self.assertLess(ready-events[0][0],5)
        self.assertEqual(c.phase,'READY')
        self.assertLess(max(abs(sum(o[k] for o in outputs)/len(outputs)) for k in range(3)),.002)

class LaunchTests(unittest.TestCase):
    def test_overlay_changes_one_include_keeps_originals_and_vendor_nodes(self):
        fixtures=Path(__file__).parent/'fixtures'
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            robot=fixtures/'vendor_robot_20261026.txt';controller=fixtures/'vendor_controller_20261026.txt'
            before=(robot.read_bytes(),controller.read_bytes())
            result=prepare(root,robot,controller)
            generated=result.read_text();child=result.with_name('controller.launch.py').read_text()
            ast.parse(generated);ast.parse(child)
            self.assertIn('mediassist_imu.launch.py',child)
            self.assertNotIn("'launch/imu_filter.launch.py'",child)
            self.assertIn('odom_publisher',child);self.assertIn('ekf',child)
            self.assertIn('lidar',generated);self.assertIn('depth_camera',generated)
            self.assertEqual(before,(robot.read_bytes(),controller.read_bytes()))

    def test_unknown_vendor_layout_refused(self):
        for code in ('x=1',"x=os.path.join(p,'imu')\ny=os.path.join(p,'imu')"):
            with self.assertRaisesRegex(RuntimeError,'VENDOR_LAUNCH_LAYOUT_CHANGED'):
                replace_include(code,'p','imu','replacement')

    def test_process_checks_are_throttled_but_child_exit_is_immediate(self):
        with tempfile.TemporaryDirectory() as directory:
            supervisor=robot_stack.Supervisor(Path(directory));child=Mock();child.poll.return_value=None
            supervisor.children=[('worker',child,None)]
            with patch.object(robot_stack,'processes',return_value={}) as scan,patch.object(robot_stack.time,'monotonic',return_value=1):
                for _ in range(100):supervisor.alive()
                self.assertEqual(scan.call_count,1)
                child.poll.return_value=1
                with self.assertRaises(RuntimeError):supervisor.alive()

if __name__=='__main__':unittest.main()
