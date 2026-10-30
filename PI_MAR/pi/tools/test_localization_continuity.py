"""Regression checks for delayed TF, sparse AMCL updates and endpoint geometry."""
import sys,math,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,patch
from collections import deque
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'pi/tools')]
from test_localization import cfg,evidence,BootstrapTests
from mediassist.localization import QualityGate,tracking_pose
from mediassist.localization_node import LocalizationNode
from mediassist.ros_adapter import RosAdapter
from mediassist.contracts import StepError
from mediassist.map_tools import load_trinary_grid

class ContinuityTests(unittest.TestCase):
    def test_move_uses_fresh_tf_between_amcl_corrections(self):
        status=dict(ready=True,map_id='map-1',map_asset_hash='hash-1',frame_id='map',
                    observed_at_sec=10.,pose=dict(x_m=0.,y_m=0.,yaw_rad=0.))
        tf=dict(value=dict(map_id='map-1',x_m=.4,y_m=.1,yaw_rad=.2),age_sec=.1,fresh=True)
        pose=tracking_pose(status,.9,10.9,'map-1','hash-1','map',tf)
        self.assertTrue(pose['fresh']);self.assertEqual(pose['value']['x_m'],.4)
        for changed,age,live in (({**status,'ready':False},.1,tf),
                                 (status,1.3,tf),(status,.1,{**tf,'fresh':False}),
                                 ({**status,'map_asset_hash':'other'},.1,tf)):
            with self.subTest(changed=changed,age=age):
                self.assertFalse(tracking_pose(changed,age,10.9,'map-1','hash-1','map',live)['fresh'])

    def locked(self):
        gate=QualityGate(cfg())
        for i in range(8):gate.update(i*.5,evidence(i))
        return gate
    def test_tracking_accepts_sparse_amcl_only_with_fresh_scan_odom_and_tf(self):
        gate=self.locked();s=evidence(8);s['pose_age_sec']=1.2
        self.assertTrue(gate.update(4,s)['ready'])
        s['scan_age_sec']=.9
        self.assertFalse(gate.update(4.1,s)['ready'])
        gate=self.locked();s=evidence(8);s['pose_age_sec']=2.1
        self.assertFalse(gate.update(4,s)['ready'])
        self.assertFalse(QualityGate(cfg()).update(0,{**evidence(),'pose_age_sec':1.2})['ready'])
    def test_scan_match_hysteresis_requires_an_established_lock(self):
        s=evidence(8);s['scan_match']=.58
        self.assertFalse(QualityGate(cfg()).update(0,s)['ready'])
        gate=self.locked();self.assertTrue(gate.update(4,s)['ready'])
        s['scan_match']=.40
        self.assertFalse(gate.update(4.1,s)['ready'])
    def test_missing_scan_transform_is_not_reported_as_a_bad_scan_match(self):
        s=evidence();s.pop('scan_match');s.pop('known_fraction');s.pop('valid_beams');s['scan_tf_error']='ExtrapolationException'
        result=QualityGate(cfg()).update(0,s)
        self.assertIn('SCAN_TF_UNAVAILABLE',result['reasons']);self.assertNotIn('SCAN_MATCH_LOW',result['reasons'])
    def test_forced_filter_update_waits_for_actual_pose_age(self):
        helper=BootstrapTests();g=helper.fake_guard('global');g.initialized=True;g.ever_ready=True;g.last_nomotion=0;g.nomotion=Mock()
        g.gate=self.locked()
        g.pose=N(header=N(stamp=N(sec=9,nanosec=800000000)))
        with patch.dict(sys.modules,helper.modules()):g.bootstrap(100)
        g.call.assert_not_called()
        g.pose.header.stamp=N(sec=8,nanosec=900000000)
        with patch.dict(sys.modules,helper.modules()):g.bootstrap(101)
        self.assertEqual(g.call.call_args.args[0],'nomotion')
        g.call.reset_mock()
        g.odom.twist.twist.linear.x=.08
        with patch.dict(sys.modules,helper.modules()):g.bootstrap(102)
        g.call.assert_not_called()
    def test_recovery_does_not_cycle_on_point_eight_second_pose_age(self):
        gate=self.locked()
        # Recorded failure: poor scan for ~3 s, then good scans alternating
        # with AMCL pose ages above .8 s. Recovery must finish before MOVE's 5 s hold.
        for i in range(12):
            self.assertFalse(gate.update(4+i*.25,{**evidence(8+i),'scan_match':.31,'pose_age_sec':1.1})['ready'])
        for i in range(2):
            self.assertFalse(gate.update(7+i*.25,{**evidence(20+i),'pose_age_sec':.9})['ready'])
        self.assertTrue(gate.update(7.5,{**evidence(22),'pose_age_sec':1.1})['ready'])
    def test_recovery_memory_expires_and_never_bypasses_hard_loss(self):
        for loss in ({'body_clear':False},{'tf_fresh':False},{'map_verified':False}):
            gate=self.locked()
            self.assertFalse(gate.update(4,{**evidence(8),**loss})['ready'])
            self.assertFalse(gate.had_lock)
            self.assertFalse(gate.update(4.5,evidence(9))['ready'])
        gate=self.locked()
        for i in range(12):
            self.assertFalse(gate.update(4+i,{**evidence(8+i),'scan_match':.3})['ready'])
        self.assertFalse(gate.had_lock)
        self.assertFalse(gate.update(16,{**evidence(30),'pose_age_sec':1.1})['ready'])
    def test_recovery_requests_amcl_update_without_reinitializing(self):
        helper=BootstrapTests();g=helper.fake_guard('global');g.initialized=True;g.ever_ready=True;g.last_nomotion=0;g.nomotion=Mock()
        g.pose=N(header=N(stamp=N(sec=9,nanosec=500000000)))
        with patch.dict(sys.modules,helper.modules()):g.bootstrap(100)
        self.assertEqual(g.call.call_args.args[0],'nomotion')
        g.initial.publish.assert_not_called()
    def test_uses_recent_older_scan_when_latest_tf_has_not_arrived(self):
        g=object.__new__(LocalizationNode);g.cfg=cfg();g.hw={'ros':{'map_frame':'map','base_frame':'base'}}
        g.map_ready=Mock(return_value=True);g.lifecycle_ready=Mock(return_value=True);g.ros_now=lambda:10.
        g.odom=None;g.pose=None;g.grid=None;g.particles=[]
        def scan(ns):return N(header=N(stamp=N(sec=9,nanosec=ns),frame_id='laser'),ranges=[1.]*60,range_min=.1,range_max=10.,angle_min=0.,angle_increment=.01)
        older,newest=scan(800000000),scan(950000000);g.scans=deque([older,newest]);g.scan=newest;g.matcher=N(score=lambda points:dict(scan_match=.9,known_fraction=.9,valid_beams=len(points)))
        transform=N(header=N(stamp=N(sec=9,nanosec=900000000)),transform=N(translation=N(x=0.,y=0.),rotation=N(x=0.,y=0.,z=0.,w=1.)))
        def lookup(target,source,stamp):
            if stamp is newest.header.stamp:raise RuntimeError('future TF')
            return transform
        g.tf=N(lookup_transform=lookup)
        class Time:
            @staticmethod
            def from_msg(stamp):return stamp
        with patch.dict(sys.modules,{'rclpy.time':N(Time=Time)}):s=g.evidence(10)
        self.assertEqual(s['scan_sequence'],(9,800000000));self.assertAlmostEqual(s['scan_age_sec'],.2)
        self.assertEqual(s['scan_match'],.9);self.assertNotIn('scan_tf_error',s)
        g.ros_now=lambda:11.
        with patch.dict(sys.modules,{'rclpy.time':N(Time=Time)}):s=g.evidence(11)
        self.assertNotIn('scan_match',s)
    def test_white_endpoint_center_can_still_collide_with_wall(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);im=Image.new('L',(40,40),254)
            for y in range(40):im.putpixel((22,y),0)
            im.save(p/'map.pgm');(p/'map.yaml').write_text('image: map.pgm\nresolution: 0.025\norigin: [0, 0, 0]\n')
            a=object.__new__(RosAdapter);a.worker={'navigation_runtime':{'map_yaml':str(p/'map.yaml')},'mapping':{'autonomy':{'footprint':dict(front=.12,rear=.12,left=.1,right=.1)}}}
            a.cfg={'endpoints':{'test':dict(x_m=.5,y_m=.5,yaw_rad=0.)}};a.node=Mock()
            grid=load_trinary_grid(p/'map.yaml');self.assertEqual(grid['data'][20*40+20],0)
            with self.assertRaises(StepError) as cm:a._check_endpoint_footprint('test')
            self.assertEqual(cm.exception.code,'ENDPOINT_FOOTPRINT_NOT_CLEAR')
            a.cfg['endpoints']['test']['x_m']=.25;a._check_endpoint_footprint('test')
if __name__=='__main__':unittest.main()
