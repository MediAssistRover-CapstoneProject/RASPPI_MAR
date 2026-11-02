"""Brief scan degradation slows one goal; graph validation remains strict and bounded."""
import sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'pi/tools')]
from mediassist.localization import QualityGate
from mediassist.ros_adapter import RosAdapter
from mediassist.contracts import StepError
from test_localization import cfg,evidence

class TrackingTests(unittest.TestCase):
    def locked(self):
        g=QualityGate(cfg())
        for i in range(10):g.update(i*.5,evidence(i))
        return g
    def test_short_scan_quality_dip_retains_lock_for_two_seconds_only(self):
        g=self.locked();s=evidence(11);s['scan_match']=.50
        self.assertEqual(g.update(5.7,s)['reason'],'TRACKING_SCAN_GRACE')
        self.assertFalse(g.update(6.6,s)['ready'])
    def test_stale_sensors_or_pose_jump_are_never_covered_by_scan_grace(self):
        for change in ({'scan_age_sec':2.},{'pose':dict(x_m=1.,y_m=0.,yaw_rad=0.)}):
            s=evidence(11);s['scan_match']=.5;s.update(change)
            with self.subTest(change=change):self.assertFalse(self.locked().update(5.,s)['ready'])
    def test_uninitialized_robot_cannot_bootstrap_from_weak_scan(self):
        g=QualityGate(cfg());s=evidence();s['scan_match']=.5
        for i in range(15):s['scan_sequence']=i;self.assertFalse(g.update(i*.5,s)['ready'])

class RuntimeTests(unittest.TestCase):
    def test_resume_ramp_preserves_pursuit_curvature_and_never_delays_braking(self):
        a=RosAdapter.__new__(RosAdapter);a.worker={'navigation_runtime':{'controller':'REGULATED_PURSUIT','forward_only':True}}
        a.cfg={'max_speed_mps':.8,'motion':{'max_angular_speed_rps':1.2}}
        a.navigation_velocity_output=Mock();a.navigation_resume_ramp=Mock()
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
            a.navigation_velocity=lambda:((.08,0.,.2),True)
            a.navigation_resume_ramp.update.return_value=(.5,0.,1.)
            a.publish_navigation_velocity(.6)
            self.assertEqual(a.last_navigation_output,(.08,0.,.2))
            a.navigation_velocity=lambda:((.6,0.,.3),True)
            a.navigation_resume_ramp.update.return_value=(.15,0.,.2)
            a.publish_navigation_velocity(.6)
            self.assertEqual(a.last_navigation_output,(.15,0.,.075))
            a.navigation_velocity=lambda:((0.,0.,.2),True)
            a.navigation_resume_ramp.update.return_value=(0.,0.,1.)
            a.publish_navigation_velocity(.6,rotation_only=True)
            self.assertEqual(a.last_navigation_output,(0.,0.,.2))
    def test_scan_grace_caps_speed_and_preserves_curvature_then_restores(self):
        a=RosAdapter.__new__(RosAdapter);a.localization_required=True
        a.worker={'navigation_runtime':{'controller':'REGULATED_PURSUIT','forward_only':True}}
        a.cfg={'max_speed_mps':.8,'motion':{'allow_strafe':False,'max_angular_speed_rps':1.2}}
        quality={'ready':True,'reason':'TRACKING_SCAN_GRACE'};a.telemetry=lambda:{'localization':quality}
        a.navigation_velocity=lambda:((.6,0.,.6),True);a.navigation_velocity_output=Mock()
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
            a.publish_navigation_velocity(.6);m=a.navigation_velocity_output.publish.call_args.args[0]
            self.assertAlmostEqual(m.linear.x,.15);self.assertAlmostEqual(m.angular.z,.15)
            quality['reason']='LOCALIZED';a.publish_navigation_velocity(.6)
            self.assertAlmostEqual(a.navigation_velocity_output.publish.call_args.args[0].linear.x,.6)
    def test_graph_checks_are_cached_only_after_success(self):
        a=RosAdapter.__new__(RosAdapter);a.node=Mock();a.cfg={'ros':{}}
        with patch('mediassist.velocity_graph.check_velocity_graph') as check,patch('mediassist.ros_adapter.time.monotonic',return_value=10.):
            a._check_graph();a._check_graph();check.assert_called_once()
        a.last_graph_verified_at=-float('inf')
        with patch('mediassist.velocity_graph.check_velocity_graph',side_effect=StepError('CHASSIS_COUNT_INVALID')) as check:
            for _ in range(2):
                with self.assertRaises(StepError):a._check_graph()
            self.assertEqual(check.call_count,2)
if __name__=='__main__':unittest.main()
