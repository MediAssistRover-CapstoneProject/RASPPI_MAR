"""Focused regressions for heading-first MOVE and localization continuity."""
import sys,math,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,AsyncMock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools'),str(ROOT/'pi/tools')]
from mediassist.navigation_setup import nav2_params
from mediassist.ros_adapter import RosAdapter
from mediassist.localization import QualityGate
from mediassist.contracts import StepError
from test_localization import cfg,evidence
from configure_move_profile import tune

class ForwardProfileTests(unittest.TestCase):
    def params(self,**kw):
        b={'amcl':{'ros__parameters':{}},'velocity_smoother':{'ros__parameters':{}},'planner_server':{'ros__parameters':{'GridBased':{}}}}
        for n in ('local_costmap','global_costmap'):b[n]={n:{'ros__parameters':{'inflation_layer':{}}}}
        c={'controller_server':{'ros__parameters':{'FollowPath':{'plugin':'dwb_core::DWBLocalPlanner','critics':['BaseObstacle','RotateToGoal']},'progress_checker':{},'general_goal_checker':{}}}}
        return nav2_params(b,c,[],.6,.8,mecanum=True,**kw)
    def test_forward_constraint_is_consistent_in_planner_controller_and_smoother(self):
        p=self.params(fast_navigation=True,position_then_spin=True,footprint_planner=True,forward_only=True)
        f=p['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(f['min_vel_x'],0.);self.assertEqual(f['max_vel_y'],0.)
        self.assertEqual(f['primary_controller'],'dwb_core::DWBLocalPlanner')
        self.assertFalse(f['rotate_to_goal_heading']);self.assertNotIn('RotateToGoal',f['critics'])
        self.assertEqual(p['planner_server']['ros__parameters']['GridBased']['motion_model_for_search'],'DUBIN')
        self.assertEqual(p['velocity_smoother']['ros__parameters']['min_velocity'][0],0.)
    def test_exploration_remains_holonomic_and_reversible(self):
        p=self.params(fast_mapping=True);f=p['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(f['min_vel_x'],-.6);self.assertEqual(f['max_vel_y'],.6)
        self.assertEqual(f['plugin'],'dwb_core::DWBLocalPlanner')
    def test_move_speed_edit_does_not_change_hardware_or_exploration(self):
        from test_motion_speed import MotionSpeedTests
        _,h=MotionSpeedTests().profile();h['max_speed_mps']=.8
        before=__import__('copy').deepcopy(h);r=tune(h,.6)
        self.assertEqual(h,before);self.assertEqual(r['motion'],h['motion'])
        self.assertEqual(r['max_speed_mps'],h['max_speed_mps']);self.assertEqual(r['navigation']['max_speed_mps'],.6)
    def test_final_command_cannot_reverse_even_if_a_resume_ramp_retains_reverse(self):
        a=RosAdapter.__new__(RosAdapter);a.worker={'navigation_runtime':{'forward_only':True}}
        a.cfg={'max_speed_mps':.8,'motion':{'allow_reverse':True,'max_reverse_speed_mps':.8,'max_angular_speed_rps':.8}}
        a.navigation_velocity=lambda:((-.2,.3,.2),True);a.navigation_velocity_output=Mock()
        a.navigation_resume_ramp=N(update=lambda *args:(-.2,.1,.2))
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
            a.publish_navigation_velocity(.6)
        m=a.navigation_velocity_output.publish.call_args.args[0]
        self.assertEqual((m.linear.x,m.linear.y),(0.,0.))
    def test_holonomic_move_preserves_lateral_command_and_reverse_cap(self):
        a=RosAdapter.__new__(RosAdapter)
        a.worker={'navigation_runtime':{'controller':'HOLONOMIC_DWB'}}
        a.cfg={'max_speed_mps':.35,'motion':{'allow_strafe':True,'allow_reverse':True,
               'max_reverse_speed_mps':.15,'max_angular_speed_rps':.8}}
        a.navigation_velocity=lambda:((-.3,.12,.04),True)
        a.navigation_velocity_output=Mock()
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
            a.publish_navigation_velocity(.2)
            move=a.navigation_velocity_output.publish.call_args.args[0]
            self.assertAlmostEqual(move.linear.x,-.15)
            self.assertAlmostEqual(move.linear.y,.06)
            self.assertAlmostEqual(move.angular.z,.02)
            a.publish_navigation_velocity(.2,rotation_only=True)
            rotation=a.navigation_velocity_output.publish.call_args.args[0]
            self.assertEqual((rotation.linear.x,rotation.linear.y),(0.,0.))

    def test_holonomic_caps_scale_the_full_trajectory_without_changing_sign_or_curvature(self):
        cases=[
            ('job_speed',(.3,.12,.2),.15,None,.35,.15,.8),
            ('hardware_speed',(.3,.12,.2),.35,None,.15,.15,.8),
            ('zone_speed',(.3,-.12,.2),.35,.12,.35,.15,.8),
            ('near_zero_zone',(.3,-.12,.2),.35,.005,.35,.15,.8),
            ('reverse',(-.3,.12,-.2),.35,None,.35,.15,.8),
            ('angular',(.12,.05,1.6),.35,None,.35,.15,.8),
        ]
        for name,desired,speed,zone,hardware,reverse,angular in cases:
            with self.subTest(cause=name):
                a=RosAdapter.__new__(RosAdapter)
                a.worker={'navigation_runtime':{'controller':'HOLONOMIC_DWB'}}
                a.cfg={'max_speed_mps':hardware,'motion':{'allow_strafe':True,'allow_reverse':True,
                    'max_reverse_speed_mps':reverse,'max_angular_speed_rps':angular}}
                a.navigation_velocity=lambda:(desired,True);a.navigation_velocity_output=Mock()
                if zone is not None:
                    a.zones=N(zones=[1],cap=lambda pose,requested:zone)
                    a.telemetry=lambda:{'map_pose':{'fresh':True}}
                with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
                    a.publish_navigation_velocity(speed)
                move=a.navigation_velocity_output.publish.call_args.args[0]
                actual=(move.linear.x,move.linear.y,move.angular.z)
                ratios=[value/wanted for value,wanted in zip(actual,desired) if wanted]
                self.assertGreater(min(ratios),0.);self.assertLessEqual(max(ratios),1.)
                for ratio in ratios:self.assertAlmostEqual(ratio,ratios[0])
                self.assertLessEqual(math.hypot(*actual[:2]),min(speed,hardware,zone if zone is not None else speed)+1e-9)
                self.assertLessEqual(abs(actual[2]),angular+1e-9)
                if desired[0]<0.:self.assertLessEqual(abs(actual[0]),reverse+1e-9)

    def test_holonomic_forbidden_axes_and_zero_zone_still_stop(self):
        for reverse_allowed,forward_only,zone in ((False,False,None),(True,True,None),(True,False,0.)):
            with self.subTest(reverse=reverse_allowed,forward_only=forward_only,zone=zone):
                a=RosAdapter.__new__(RosAdapter)
                a.worker={'navigation_runtime':{'controller':'HOLONOMIC_DWB','forward_only':forward_only}}
                a.cfg={'max_speed_mps':.35,'motion':{'allow_strafe':True,'allow_reverse':reverse_allowed,
                    'max_reverse_speed_mps':.15,'max_angular_speed_rps':.8}}
                a.navigation_velocity=lambda:((-.2,.1,.2),True);a.navigation_velocity_output=Mock()
                if zone is not None:
                    a.zones=N(zones=[1],cap=lambda pose,speed:zone)
                    a.telemetry=lambda:{'map_pose':{'fresh':True}}
                with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
                    a.publish_navigation_velocity(.35)
                move=a.navigation_velocity_output.publish.call_args.args[0]
                self.assertEqual(move.linear.x,0.)
                if zone is not None:self.assertEqual((move.linear.y,move.angular.z),(0.,0.))
                else:self.assertGreater(move.linear.y,0.)

class TrackingTests(unittest.TestCase):
    def lock(self):
        g=QualityGate(cfg())
        for i in range(10):g.update(i*.5,evidence(i))
        return g
    def test_recent_correction_can_be_carried_by_live_scan_and_odom(self):
        g=self.lock();s=evidence(11);s['pose_age_sec']=5.
        r=g.update(5.,s);self.assertTrue(r['ready']);self.assertIn('AMCL_CORRECTION_HELD_WITH_LIVE_SCAN',r['warnings'])
    def test_bridge_is_bounded_and_does_not_hide_sensor_loss_or_bad_match(self):
        for changes in ({'pose_age_sec':9.},{'pose_age_sec':5.,'scan_age_sec':1.},{'pose_age_sec':5.,'scan_match':.5}):
            with self.subTest(changes=changes):
                s=evidence(11);s.update(changes);self.assertFalse(self.lock().update(5.,s)['ready'])

class MoveTests(unittest.IsolatedAsyncioTestCase):
    async def test_workflow_cannot_raise_local_move_ceiling(self):
        a=RosAdapter.__new__(RosAdapter);a.cfg={'navigation':{'max_speed_mps':.6}};a.event_sink=None
        async def selected(p,*args):return p['max_speed_mps']
        with patch('mediassist.navigation.select_and_move',side_effect=selected):
            self.assertEqual(await a._move({'max_speed_mps':.8},{}),.6)
    async def test_no_final_rotation_when_arrival_pose_is_outside_position_window(self):
        a=RosAdapter.__new__(RosAdapter);a.cfg={'endpoints':{'A':dict(x_m=0.,y_m=0.)}}
        a.node=Mock()
        a.lease_message=Mock();a._navigation_context=Mock(return_value=True)
        a.telemetry=lambda:{'map_pose':{'fresh':True,'value':dict(x_m=.2,y_m=0.,yaw_rad=0.)}}
        with self.assertRaisesRegex(StepError,'ENDPOINT_POSITION_NOT_CONFIRMED'):await a._wait_arrival_pose('A')
if __name__=='__main__':unittest.main()
