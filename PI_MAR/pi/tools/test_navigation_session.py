"""Session/map/initialization/MOVE boundaries, and saved-map reload semantics. No ROS or motors."""
import copy,json,math,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'pi/tools')]
from mediassist.navigation_session import validate_descriptor,command_context,expired
from mediassist.contracts import StepError
from mediassist.footprint import body_clear
from mediassist.frontier_mapping import plan
from mediassist.mapping import MappingManager
import test_autonomous_mapping as auto

ROBOT='11111111-1111-4111-8111-111111111111';MAP='22222222-2222-4222-8222-222222222222';SESSION='33333333-3333-4333-8333-333333333333'
CTX=dict(session_id=SESSION,map_id=MAP,map_asset_hash='a'*64,expires_at='2099-01-01T00:00:00Z')

class SessionTests(unittest.TestCase):
    def test_nav2_candidate_slows_before_obstacles_and_preserves_a_valid_turn(self):
        from mediassist.autonomous_mapping import nav2_assist
        import test_frontier_mapping as fixtures
        cfg=fixtures.profile();view={'front_m':.3,'rotation_clear':False}
        x,z=nav2_assist((.2,.2),view,cfg)
        self.assertGreater(x,0);self.assertLess(x,.2);self.assertEqual(z,.2)
        self.assertEqual(nav2_assist((.2,.2),{**view,'front_m':.05},cfg),(0.,.2))
    def descriptor(self):return dict(sessionId=SESSION,robotId=ROBOT,mapId=MAP,mapAssetHash='a'*64,state='LOCALIZING',expiresAt=CTX['expires_at'],initialization=dict(mode='MANUAL',revision=0,pose=None))
    def test_loading_image_or_session_does_not_invent_an_initial_pose(self):
        value=validate_descriptor(self.descriptor(),ROBOT,SESSION)
        self.assertIsNone(value['initialization']['pose']);self.assertEqual(value['initialization']['mode'],'MANUAL')
    def test_foreign_expired_closed_or_nonfinite_descriptors_rejected(self):
        for mutate in [lambda v:v.update(robotId=MAP),lambda v:v.update(sessionId=MAP),lambda v:v.update(state='CLOSED'),
            lambda v:v.update(expiresAt='2020-01-01T00:00:00Z'),lambda v:v['initialization'].update(mode='SEED',pose=dict(x_m=math.nan,y_m=0.,yaw_rad=0.))]:
            value=self.descriptor();mutate(value)
            with self.assertRaises(StepError):validate_descriptor(value,ROBOT,SESSION)
    def test_move_requires_exact_map_session_initialization_and_applied_profile(self):
        context={k:CTX[k] for k in ('session_id','map_id','map_asset_hash')};context.update(initialization_revision=2,profile_hash='b'*64)
        local=dict(ready=True,age_sec=.1,initialization_revision=2)
        adapter=NS(telemetry=lambda:{'localization':local},navigation_profile_hash='b'*64)
        cfg={'navigation_session':CTX}
        command_context({'navigationContext':context},cfg,{},adapter)
        for key,value in [('map_id',ROBOT),('session_id',ROBOT),('map_asset_hash','c'*64),('initialization_revision',1),('profile_hash','c'*64)]:
            with self.subTest(key=key),self.assertRaises(StepError):command_context({'navigationContext':{**context,key:value}},cfg,{},adapter)
        local['ready']=False
        with self.assertRaises(StepError):command_context({'navigationContext':context},cfg,{},adapter)
        local.update(ready=True,age_sec=2.)
        with self.assertRaises(StepError):command_context({'navigationContext':context},cfg,{},adapter)
    def test_legacy_worker_cannot_accept_a_session_command(self):
        with self.assertRaises(StepError):command_context({'navigationContext':CTX},{},{},None)
    def test_export_and_nav2_trinary_reload_preserves_unknown(self):
        import yaml
        snap=dict(width=3,height=2,resolution=.025,origin=[-1.,2.,.7],frame_id='map',data=bytes([0,100,255,25,64,65]))
        with tempfile.TemporaryDirectory() as folder:
            MappingManager.save_files(None,Path(folder),snap,SESSION)
            config=yaml.safe_load((Path(folder)/'map.yaml').read_text());raw=(Path(folder)/'map.pgm').read_bytes().split(b'\n',3)[3]
            values=[100 if (255-v)/255.>config['occupied_thresh'] else 0 if (255-v)/255.<config['free_thresh'] else 255 for v in raw]
            restored=values[3:]+values[:3]
            self.assertEqual(restored,[0,100,255,0,255,100]);self.assertEqual(config['origin'],snap['origin'])
    def test_oriented_body_distinguishes_actual_overlap_from_rotation_envelope(self):
        w=h=80;r=.025;raw=bytearray([0]*(w*h));pose=dict(x_m=1.0125,y_m=1.0125,yaw_rad=0.)
        raw[47*w+40]=100
        snapshot=dict(width=w,height=h,resolution=r,origin=[0.,0.,0.],frame_id='map',data=bytes(raw))
        f=dict(front=.12,rear=.12,left=.10,right=.10)
        self.assertTrue(body_clear(snapshot,pose,f)['clear'])
        raw[43*w+40]=100;snapshot['data']=bytes(raw)
        self.assertEqual(body_clear(snapshot,pose,f)['kind'],'occupied')
    def test_nav2_can_plan_from_clear_body_without_requiring_a_full_buffer_circle(self):
        import test_frontier_mapping as fixtures
        cfg=fixtures.profile();cfg['frontier_controller']='nav2';cfg['footprint']=dict(front=.12,rear=.12,left=.1,right=.1)
        snapshot=fixtures.FrontierTests().fine_corridor();raw=bytearray(snapshot['data'])
        raw[32*160+40]=100;snapshot['data']=bytes(raw)
        pose=dict(x_m=1.0125,y_m=.6125,yaw_rad=0.)
        self.assertEqual(plan(snapshot,pose,{**cfg,'frontier_controller':'reactive'})['state'],'BLOCKED')
        result=plan(snapshot,pose,cfg)
        self.assertEqual(result['state'],'FRONTIERS_PRESENT');self.assertTrue(result['seed_body_clear'])
        # A candidate is not permission to publish raw-path velocities: the manager uses Nav2.
        raw[25*160+40]=100;snapshot['data']=bytes(raw)
        self.assertEqual(plan(snapshot,pose,cfg)['state'],'BLOCKED')

if __name__=='__main__':unittest.main()
