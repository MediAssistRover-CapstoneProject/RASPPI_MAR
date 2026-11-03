"""MQTT continuity, non-blocking plan reporting and centimetre clearance; no motors/network."""
import asyncio,copy,json,sys,tempfile,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
from datetime import datetime,timezone
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools'),str(ROOT/'pi/tools')]
from mediassist.navigation_authorization import receive,read
from mediassist.navigation_session import SessionPoller
from mediassist.ros_adapter import RosAdapter
from mediassist.motion_braking import limit_velocity
from configure_motion_clearance import tune
import test_motion_speed as speed_fixtures
from configure_motion_speed import tune as speed_tune

ROBOT='11111111-1111-4111-8111-111111111111';SESSION='22222222-2222-4222-8222-222222222222';MAP='33333333-3333-4333-8333-333333333333'

class AuthorizationTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.cfg={'robot_id':ROBOT,'localization':{'status_file':str(Path(self.folder.name)/'status.json')},
                  'navigation_session':dict(session_id=SESSION,map_id=MAP,map_asset_hash='a'*64,expires_at='2099-01-01T00:00:00Z')}
        self.value=dict(robotId=ROBOT,sessionId=SESSION,mapId=MAP,mapAssetHash='a'*64,state='OPEN',expiresAt='2099-01-01T00:00:00Z',
                        observedAt=datetime.now(timezone.utc).isoformat(),initialization=dict(mode='MANUAL',revision=1,pose=None))
        self.poller=object.__new__(SessionPoller);p=self.poller
        p.client=N(cfg=self.cfg);p.context=self.cfg['navigation_session'];p.lock=threading.Lock();p.value=None;p.at=-float('inf');p.error=None
    def test_cached_packet_never_renews_itself_or_needs_https(self):
        with patch('mediassist.navigation_authorization.time.monotonic',return_value=100):receive(self.cfg,self.value)
        with patch('mediassist.navigation_authorization.time.monotonic',return_value=110):
            self.assertTrue(self.poller.mqtt_current());self.assertAlmostEqual(self.poller.at,100,places=2)
            at=self.poller.at
            self.assertTrue(self.poller.mqtt_current());self.assertEqual(self.poller.at,at)
        with patch('mediassist.navigation_authorization.time.monotonic',return_value=116):self.assertFalse(self.poller.mqtt_current())
    def test_replayed_packet_cannot_extend_authorization(self):
        receive(self.cfg,self.value);at=read(self.cfg)[0]
        with patch('mediassist.navigation_authorization.time.monotonic',return_value=at+10):
            receive(self.cfg,self.value);self.assertEqual(read(self.cfg)[0],at)
    def test_revocation_and_wrong_map_clear_previous_grant(self):
        for change in ({'state':'CLOSED'},{'mapId':ROBOT},{'sessionId':ROBOT},{'robotId':MAP}):
            self.poller.value=copy.deepcopy(self.value)
            with patch('mediassist.navigation_authorization.read',return_value=(time.monotonic(),{**self.value,**change})):
                self.assertTrue(self.poller.mqtt_current());self.assertIsNone(self.poller.value)
    def test_delayed_packet_has_only_its_remaining_lifetime(self):
        observed=datetime.fromisoformat(self.value['observedAt']).timestamp()
        with patch('mediassist.navigation_authorization.time.time',return_value=observed+12.),patch('mediassist.navigation_authorization.time.monotonic',return_value=100.):
            receive(self.cfg,self.value);self.assertEqual(read(self.cfg)[0],88.)
        with patch('mediassist.navigation_authorization.time.monotonic',return_value=104.):self.assertIsNone(read(self.cfg))
    def test_old_revision_cannot_reinitialize_amcl(self):
        self.poller.value={**self.value,'initialization':dict(mode='MANUAL',revision=2,pose=None)}
        with patch('mediassist.navigation_authorization.read',return_value=(time.monotonic(),self.value)):
            self.assertFalse(self.poller.mqtt_current());self.assertEqual(self.poller.value['initialization']['revision'],2)
    def test_old_be_without_mailbox_uses_fallback(self):
        self.assertFalse(self.poller.mqtt_current())

class ReportingTests(unittest.IsolatedAsyncioTestCase):
    async def test_slow_display_upload_cannot_hold_motion_and_coalesces_paths(self):
        adapter=object.__new__(RosAdapter);gate=asyncio.Event();sent=[]
        async def publish(body):sent.append(body);await gate.wait()
        adapter._publish_plan=publish
        adapter._report_plan('first');await asyncio.sleep(0)
        first=adapter.navigation_plan_upload
        adapter._report_plan('obsolete');adapter._report_plan('latest')
        self.assertIs(adapter.navigation_plan_upload,first);self.assertEqual(sent,['first'])
        gate.set();await first;self.assertEqual(sent,['first','latest'])

class ClearanceTests(unittest.TestCase):
    def test_clearance_preserves_network_calibration_and_speed_and_brakes_before_close_range(self):
        raw,hw=speed_tune(*speed_fixtures.MotionSpeedTests().profile(),.8,.8,.8)
        before=copy.deepcopy((raw,hw));candidate,tuned=tune(raw,hw)
        self.assertEqual((raw,hw),before)
        self.assertEqual(candidate['mapping']['upload_base_url'],raw['mapping']['upload_base_url'])
        self.assertEqual(tuned['max_speed_mps'],hw['max_speed_mps'])
        self.assertEqual(candidate['mapping']['autonomy']['speed_mps'],.8)
        self.assertAlmostEqual(tuned['motion']['rear_stop_distance_m'],.14)
        profile=tuned['motion']['lidar_braking']
        view=dict(valid=True,nearest_lidar_m=1.,rotation_clear=True,obstacle_points=[(.16,0.)])
        velocity,reason=limit_velocity((.6,0.,.3),view,.05,profile)
        self.assertLess(velocity[0],.1);self.assertIsNone(reason)
        self.assertAlmostEqual(velocity[2]/velocity[0],.5)
        view['obstacle_points']=[(.14,0.)]
        self.assertEqual(limit_velocity((.6,0.,.3),view,.05,profile)[0],(0.,0.,0.))
        view['obstacle_points']=[(.6,0.)]
        self.assertLess(limit_velocity((.8,0.,0.),view,.05,profile)[0][0],.8)

if __name__=='__main__':unittest.main()
