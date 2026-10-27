"""Offline localization regression: no ROS installation, network, GPIO or motor output."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools')]
from mediassist.localization import configuration,QualityGate,GridMatcher,guarded_pose,covariance_std,particle_support
from mediassist.contracts import StepError
from mediassist.ros_adapter import RosAdapter
from mediassist.localization_node import LocalizationNode

ID='11111111-1111-4111-8111-111111111111';HASH='a'*64
def cfg():return configuration(dict(enabled=True,map_id=ID,map_asset_hash=HASH))
def evidence(seq=1):
    cov=[0.]*36;cov[0]=cov[7]=.01;cov[35]=.01
    return dict(map_verified=True,amcl_active=True,tf_fresh=True,scan_age_sec=.1,odom_age_sec=.1,
                pose_age_sec=.1,covariance=cov,pose=dict(x_m=0.,y_m=0.,yaw_rad=0.),
                amcl_pose=dict(x_m=0.,y_m=0.,yaw_rad=0.),scan_match=.9,known_fraction=.9,valid_beams=100,scan_sequence=seq,particle_support=.95)

class LocalizationTests(unittest.TestCase):
    def test_distinct_scan_evidence_and_elapsed_time_required(self):
        gate=QualityGate(cfg())
        for i in range(7):self.assertFalse(gate.update(i*.5,evidence(i))['ready'])
        self.assertTrue(gate.update(3.5,evidence(7))['ready'])
    def test_repeated_scan_does_not_converge(self):
        gate=QualityGate(cfg())
        for i in range(20):self.assertFalse(gate.update(i,evidence(1))['ready'])
    def test_each_missing_or_stale_input_blocks(self):
        cases=dict(map_verified=False,amcl_active=False,tf_fresh=False,scan_age_sec=2,odom_age_sec=2,
                   pose_age_sec=2,scan_match=.1,known_fraction=.1,valid_beams=2)
        for key,value in cases.items():
            gate=QualityGate(cfg());sample=evidence();sample[key]=value
            with self.subTest(key=key):
                for i in range(10):self.assertFalse(gate.update(i,sample)['ready'])
    def test_covariance_is_checked_including_correlation(self):
        s=evidence();s['covariance'][1]=s['covariance'][6]=.02
        self.assertIn('AMCL_COVARIANCE_INVALID',QualityGate(cfg()).update(0,s)['reasons'])
        s=evidence();s['covariance'][0]=.25
        self.assertIn('AMCL_UNCERTAIN',QualityGate(cfg()).update(0,s)['reasons'])
        with self.assertRaises(ValueError):covariance_std([0.]*36)
    def test_nan_future_stamp_and_disagreeing_pose_block(self):
        for change in ({'scan_age_sec':math.nan},{'scan_age_sec':-.1},{'pose':dict(x_m=1.,y_m=0.,yaw_rad=0.)}):
            s=evidence();s.update(change);self.assertFalse(QualityGate(cfg()).update(0,s)['ready'])
    def test_loss_resets_stability_and_pose_jump_is_detected(self):
        gate=QualityGate(cfg())
        for i in range(10):gate.update(i*.5,evidence(i))
        s=evidence(11);s['pose']['x_m']=s['amcl_pose']['x_m']=1.
        self.assertIn('POSE_JUMP',gate.update(5,s)['reasons'])
        self.assertFalse(gate.update(5.5,s)['ready'])
    def test_matcher_rejects_blank_and_wrong_locations(self):
        cells=[0]*100
        for y in range(10):cells[y*10+8]=100
        matcher=GridMatcher(10,10,.1,(0,0,0),cells,.1)
        self.assertEqual(matcher.score([(.85,.25),(.85,.65)])['scan_match'],1)
        self.assertEqual(matcher.score([(.2,.2),(.2,.6)])['scan_match'],0)
        self.assertEqual(GridMatcher(10,10,.1,(0,0,0),[0]*100).score([(.2,.2)])['scan_match'],0)
        self.assertEqual(GridMatcher(10,10,.1,(0,0,0),[-1]*100).score([(.2,.2)])['known_fraction'],0)
    def test_matcher_uses_rotated_origin(self):
        matcher=GridMatcher(10,10,.1,(2,3,math.pi/2),[0]*100)
        self.assertEqual(matcher.index(1.85,3.25),(2,1))
    def test_guard_pose_identity_age_and_expiry(self):
        status=dict(ready=True,map_id=ID,map_asset_hash=HASH,frame_id='map',observed_at_sec=100.,
                    pose_age_sec=.1,pose=evidence()['pose'])
        result=guarded_pose(status,.1,100.1,ID,HASH)
        self.assertTrue(result['fresh']);self.assertEqual(set(result['value']),{'map_id','x_m','y_m','yaw_rad'})
        for kwargs in ({'received_age':1.},{'ros_now':102.},{'map_id':'wrong'},{'asset_hash':'wrong'}):
            args=dict(status=status,received_age=.1,ros_now=100.1,map_id=ID,asset_hash=HASH);args.update(kwargs)
            self.assertFalse(guarded_pose(**args)['fresh'])
    def test_seed_requires_real_coordinates(self):
        with self.assertRaises(StepError):configuration(dict(enabled=True,mode='seed',map_id=ID,map_asset_hash=HASH))
        configuration(dict(enabled=True,mode='seed',map_id=ID,map_asset_hash=HASH,initial_pose=dict(x_m=1.,y_m=2.,yaw_rad=.2)))
    def test_standalone_worker_cannot_bypass_localization_setup(self):
        from mediassist.worker_adapter import WorkerAdapter
        with self.assertRaisesRegex(StepError,'CONFIGURE_LOCALIZATION_FIRST'):
            WorkerAdapter({},dict(mapping={'enabled':False},enabled_step_types=['MOVE']))
    def test_invalid_status_cannot_become_a_new_mqtt_field(self):
        adapter=object.__new__(RosAdapter);adapter.samples={};import threading
        adapter.lock=threading.Lock();adapter._localization_status(N(data='invalid'))
        self.assertEqual(adapter.samples,{})
        adapter._localization_status(N(data=json.dumps({'ready':False})))
        self.assertIn('localization_status',adapter.samples)
    def test_mecanum_configuration_preserves_controller_and_speed(self):
        import configure_localization
        hw={'ros':{'map_frame':'map','base_frame':'base_footprint','scan':'/scan_raw'}}
        original={'amcl':{'ros__parameters':{}},'controller_server':{'ros__parameters':{'max_speed':.15}}}
        result=configure_localization.nav_parameters(original,hw)
        self.assertEqual(result['amcl']['ros__parameters']['robot_model_type'],'nav2_amcl::OmniMotionModel')
        self.assertFalse(result['amcl']['ros__parameters']['set_initial_pose'])
        self.assertNotEqual(result['amcl']['ros__parameters']['save_pose_rate'],0)
        self.assertEqual(result['controller_server'],original['controller_server'])
        self.assertEqual(original['amcl']['ros__parameters'],{})
    def test_localization_only_mode_accepts_no_endpoints_without_enabling_motion(self):
        from robot_stack import validate_mode
        hw={'map_asset_sha256':HASH,'endpoints':{},'motion':dict(commissioned=False,cart_footprint_validated=False,watchdog_validated=False)}
        settings={'mapping':{'enabled':False},'localization':{'enabled':True},'navigation_runtime':{'localization_only':True}}
        validate_mode('run',settings,hw,'navigation')
        settings['navigation_runtime']['localization_only']=False
        with self.assertRaises(RuntimeError):validate_mode('run',settings,hw,'navigation')
    def test_no_velocity_publisher_in_localizer(self):
        source=(ROOT/'pi/mediassist/localization_node.py').read_text()
        self.assertNotIn('cmd_vel',source);self.assertNotIn('Twist',source)
    def test_global_distribution_with_two_possible_places_is_not_ready(self):
        pose=dict(x_m=0.,y_m=0.,yaw_rad=0.)
        points=[(0.,0.,0.,1.)]*50+[(2.,2.,0.,1.)]*50
        self.assertEqual(particle_support(points,pose),.5)
        s=evidence();s['particle_support']=.5
        self.assertIn('PARTICLES_NOT_CONVERGED',QualityGate(cfg()).update(0,s)['reasons'])
        self.assertEqual(particle_support([(0.,0.,0.,1.)]*100,pose),1.)

class BootstrapTests(unittest.TestCase):
    def fake_guard(self,mode='seed'):
        guard=object.__new__(LocalizationNode)
        guard.cfg={**cfg(),'mode':mode,'initial_pose':dict(x_m=2.,y_m=2.,yaw_rad=.4)}
        guard.hw={'ros':{'map_frame':'map'}};guard.gate=QualityGate(guard.cfg)
        guard.map_ready=Mock(return_value=True);guard.lifecycle_ready=Mock(return_value=True)
        guard.ros_now=Mock(return_value=10.)
        guard.odom=N(header=N(stamp=N(sec=10,nanosec=0)),twist=N(twist=N(linear=N(x=0.,y=0.),angular=N(z=0.))))
        guard.initialized=False;guard.boot_at=None;guard.boot_ros=math.inf;guard.pose=None;guard.particles=[]
        guard.last_nomotion=100.;guard.ever_ready=False;guard.initial=Mock();guard.initial.get_subscription_count.return_value=1
        guard.matcher=GridMatcher(10,10,1.,(0,0,0),[0]*100);guard.grid=N(data=[0]*100)
        guard.node=Mock();guard.node.get_clock.return_value.now.return_value.to_msg.return_value=N(sec=10,nanosec=0)
        guard.pending={};guard.global_reset=Mock();guard.call=Mock(return_value=False)
        guard.authorization_error=None
        return guard
    def modules(self):
        class Pose:
            def __init__(self):
                self.header=N(frame_id='',stamp=None)
                self.pose=N(pose=N(position=N(x=0.,y=0.),orientation=N(z=0.,w=1.)),covariance=[0.]*36)
        return {'geometry_msgs':N(),'geometry_msgs.msg':N(PoseWithCovarianceStamped=Pose),
                'std_srvs':N(),'std_srvs.srv':N(Empty=N(Request=lambda:N()))}
    def test_seed_bootstrap_does_not_wait_for_map_tf(self):
        guard=self.fake_guard()
        with patch.dict(sys.modules,self.modules()):guard.bootstrap(100.)
        guard.initial.publish.assert_called_once()
        message=guard.initial.publish.call_args.args[0]
        self.assertEqual(message.pose.pose.position.x,2.)
        self.assertAlmostEqual(message.pose.pose.orientation.z,math.sin(.2))
        self.assertTrue(guard.initialized)
    def test_initialization_refuses_moving_robot_and_unknown_cell(self):
        for moving in (True,False):
            guard=self.fake_guard()
            if moving:guard.odom.twist.twist.linear.x=.1
            else:guard.grid.data[22]=-1
            with patch.dict(sys.modules,self.modules()):guard.bootstrap(100.)
            guard.initial.publish.assert_not_called();self.assertFalse(guard.initialized)
    def test_global_initialization_uses_service_and_no_guessed_pose(self):
        guard=self.fake_guard('global')
        def call(key,client,request,done):
            if key=='global':done(N());return True
            return False
        guard.call=Mock(side_effect=call)
        with patch.dict(sys.modules,self.modules()):guard.bootstrap(100.)
        self.assertEqual(guard.call.call_args.args[0],'global')
        guard.initial.publish.assert_not_called();self.assertTrue(guard.initialized)

class MotionLossTests(unittest.IsolatedAsyncioTestCase):
    async def test_localization_loss_during_move_uses_the_stop_path(self):
        from unittest.mock import AsyncMock
        import asyncio
        adapter=object.__new__(RosAdapter);adapter.localization_required=True
        adapter.clear_navigation_velocity=Mock();adapter.navigation_speed_limit=Mock();adapter.hold_navigation_velocity=Mock()
        adapter._prerequisites=AsyncMock();adapter._pose=Mock(return_value=N());adapter._check_graph=Mock()
        adapter.nav=Mock();adapter.nav.server_is_ready.return_value=True
        pending=asyncio.get_running_loop().create_future()
        goal=N(accepted=True,get_result_async=lambda:pending)
        adapter.future=AsyncMock(return_value=goal);adapter.stop=AsyncMock(return_value=True);adapter.lease_message=Mock()
        adapter.telemetry=Mock(return_value={'map_pose':{'fresh':False},'gate':{'age_sec':0.}})
        modules={'nav2_msgs':N(),'nav2_msgs.action':N(NavigateToPose=N(Goal=lambda:N()))}
        with patch.dict(sys.modules,modules):
            with self.assertRaisesRegex(StepError,'LOCALIZATION_LOST'):
                await adapter._move_one({'max_speed_mps':.05},'endpoint')
        adapter.stop.assert_awaited_once();adapter.lease_message.assert_called_once_with(False)

if __name__=='__main__':unittest.main()
