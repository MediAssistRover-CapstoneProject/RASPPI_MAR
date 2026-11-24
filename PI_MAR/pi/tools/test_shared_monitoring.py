"""Shared inference identity, ground projection and debounce; no ROS or motion."""
import asyncio
import time
from types import SimpleNamespace as N
import unittest
from unittest.mock import AsyncMock,Mock,patch

from mediassist.fall_depth import project_ground
from mediassist.contracts import StepError
from mediassist.restricted_monitor import SharedRestrictedMonitor
from mediassist.patrol_fall import PatrolFallObserver


class SharedMonitoringTests(unittest.IsolatedAsyncioTestCase):
    async def test_checkpoint_feeds_the_shared_zone_observation_without_a_second_inference(self):
        step=dict(stepType='EXECUTE_SENSING',jobId='patrol',jobStepId='sense',
                  params=dict(sensing_type='RESTRICTED_ZONE',zone_id='zone',window_sec=.4,fall_detection=True))
        adapter=N(worker={'navigation_session':{'map_id':'map'}},cfg={'ros':{'map_frame':'map'}},
                  navigation_profile_hash='profile',zones=N(zones=[dict(kind='KEEP_OUT',zone_id='zone',
                  points=[(0.,0.),(2.,0.),(2.,2.),(0.,2.)])]))
        observer=PatrolFallObserver.__new__(PatrolFallObserver)
        observer.adapter=adapter;observer.config={'token_file':'unused','restricted_monitor':{'enabled':True},'depth':{'floor_z_m':0.}}
        observer.camera={};observer.work=None;observer.restricted_enabled=True
        observer.monitor_worker=N(active=step)
        observer.stationary_revision=lambda:('session',1)
        observer.display_overlay=Mock()
        observer.analyze=Mock(return_value=dict(algorithm='shared',suspected_fall=False))
        observer.be=N(request=Mock(return_value=dict(model_version='shared',producer_ref='shared')))
        observer.restricted=SharedRestrictedMonitor(observer)
        observer.restricted.project=lambda *args:([dict(track_id=1,confidence=.9,
                    location=dict(xM=1.,yM=1.,rangeM=1.))],True,None)
        class Capture:
            def __init__(self,*args):
                self.preview=N(take_sample_with_header=lambda *args:((b'pixels',640,480,'capture',time.monotonic()),None))
                self.depth=None
            async def set_enabled(self,enabled):pass
        with patch('mediassist.patrol_fall.Capture',Capture), \
                patch('mediassist.patrol_fall.read_token',return_value='token'), \
                patch('mediassist.patrol_fall.encode_jpeg',return_value=dict(jpeg=b'jpeg',captured_at='capture',width=640,height=480)), \
                patch('mediassist.media_runtime.audio_reserved',return_value=False):
            frames,observation=await asyncio.gather(observer.observe(step),observer.restricted.observe(step['params']))
        self.assertGreaterEqual(frames,3)
        self.assertTrue(observation['metrics']['intrusion_detected'])
        # A final in-flight frame may cross the bounded checkpoint deadline.
        self.assertLessEqual(observer.analyze.call_count,frames+1)
        await observer.restricted.close()

    async def test_profile_reply_cannot_bind_a_foreground_step_that_replaced_the_checkpoint(self):
        params=dict(sensing_type='RESTRICTED_ZONE',zone_id='zone',window_sec=3.)
        step=dict(stepType='EXECUTE_SENSING',jobId='patrol',jobStepId='sense',params=params)
        worker=N(active=step)
        observer=N(adapter=N(),monitor_worker=worker,
                   config={'restricted_monitor':{'enabled':True},'depth':{}})
        def profile(*args):
            worker.active=dict(stepType='MOVE',jobId='delivery',jobStepId='move',params={'mode':'NORMAL'})
            return dict(model_version='shared',producer_ref='shared')
        observer.be=N(request=profile)
        monitor=SharedRestrictedMonitor(observer)
        with self.assertRaises(StepError) as error:
            await monitor.observe(params)
        self.assertEqual(error.exception.code,'AI_RESTRICTED_CHECKPOINT_CHANGED')
        self.assertIsNone(monitor.patrol_job)
        self.assertIsNone(monitor.latest)

    def test_ground_projection_rejects_an_above_floor_anchor(self):
        transform=N(rotation=N(x=0.,y=0.,z=0.,w=1.),translation=N(x=1.,y=2.,z=0.))
        with patch('mediassist.fall_depth.camera_transform') as convert:
            import numpy as np
            convert.return_value=(np.eye(3),np.array([1.,2.,0.]))
            position=project_ground(([(0.,0.,.1),(.2,0.,.1)],10_000_000),transform,0.)
            self.assertEqual((position['xM'],position['yM'],position['depthSkewMs']),(1.1,2.,10.))
            self.assertIsNone(project_ground(([(0.,0.,1.),(.2,0.,1.)],0),transform,0.))

    async def test_same_track_requires_sustained_presence_and_only_one_report(self):
        adapter=N(navigation_profile_hash='profile',zones=N(zones=[dict(kind='KEEP_OUT',zone_id='zone',
                  points=[(0.,0.),(2.,0.),(2.,2.),(0.,2.)])]))
        observer=N(adapter=adapter,config={'restricted_monitor':{'enabled':True},'depth':{}})
        monitor=SharedRestrictedMonitor(observer);monitor.report=AsyncMock()
        worker=N(active=dict(stepType='MOVE',params={'mode':'PATROL'},jobId='patrol',jobStepId='step'))
        person=dict(track_id=1,confidence=.9,location=dict(xM=1.,yM=1.,rangeM=2.))
        monitor.project=lambda *args:([person],True,None)
        with patch('mediassist.restricted_monitor.time.monotonic',side_effect=[0.,.25,.5,.75,1.]):
            for i in range(5):monitor.update(worker,{'algorithm':'shared'},None,(640,480),'capture',('session',1))
        await asyncio.sleep(0)
        monitor.report.assert_awaited_once()
        body=monitor.report.await_args.args[0]
        self.assertEqual((body['jobId'],body['zoneId'],body['trackId']),('patrol','zone',1))
        await monitor.close()

    async def test_foreground_job_cannot_reuse_a_previous_patrol_track(self):
        adapter=N(navigation_profile_hash='profile',zones=N(zones=[]))
        monitor=SharedRestrictedMonitor(N(adapter=adapter,config={'restricted_monitor':{'enabled':True},'depth':{}}))
        monitor.patrol_job='patrol';monitor.project=lambda *args:self.fail('foreground must not project')
        monitor.update(N(active=dict(stepType='MOVE',params={'mode':'NORMAL'},jobId='delivery',jobStepId='step')),
                       {},None,(640,480),'capture',('session',1))
        self.assertIsNone(monitor.latest)


if __name__=='__main__':unittest.main()
