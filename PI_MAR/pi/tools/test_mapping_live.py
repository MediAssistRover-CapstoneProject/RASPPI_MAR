"""Offline tests: transport scope, expiry, motion watchdog and media separation. No robot motion."""
import asyncio,copy,json,os,sys,time,unittest
import threading
import tempfile
import urllib.error
import io
from pathlib import Path
from datetime import datetime,timezone,timedelta
from types import SimpleNamespace
from unittest.mock import AsyncMock,Mock,patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
for wheel in (ROOT.parent/'offline/wheels').glob('paho_mqtt-*.whl'):sys.path.insert(0,str(wheel))
from mediassist.mapping_control import MappingControlLease,VALIDATOR,VelocityRamp
from mediassist.mapping import MappingManager
from mediassist.mqtt_transport import MqttSession
from mediassist.step_worker import StepWorker
from mediassist.worker_adapter import WorkerAdapter
from mediassist.ros_adapter import RosAdapter
from mediassist.contracts import StepError
from mediassist.contracts import parse_timestamp
from mediassist.step_contract import expiry
R='11111111-1111-4111-8111-111111111111';S='22222222-2222-4222-8222-222222222222'
J='33333333-3333-4333-8333-333333333333';T='44444444-4444-4444-8444-444444444444'
NOW=datetime(2026,10,14,10,tzinfo=timezone.utc)
def message():return {'messageType':'MAPPING_CONTROL','robotId':R,'transportSessionId':T,'jobId':J,'mappingSessionId':S,
    'commandId':'55555555-5555-4555-8555-555555555555','sequenceNo':1,'issuedAt':NOW.isoformat(),
    'expiresAt':(NOW+timedelta(milliseconds=750)).isoformat(),'linearX':.05,'angularZ':0.,'durationMs':400}
STATE={'session_id':S,'job_id':J,'state':'MAPPING','session_open':True}


class OverlayTests(unittest.IsolatedAsyncioTestCase):
    def test_end_on_candidate_requires_associated_anchors_and_stationary_depth_proof(self):
        from mediassist.patrol_fall import PatrolFallObserver
        observer=object.__new__(PatrolFallObserver);observer.camera_moving=Mock(return_value=False)
        observer.patient_location=Mock(return_value={'xM':1.,'yM':2.,'floorHeightM':.2})
        person=dict(image_width=640,image_height=480,bbox=[.2,.2,.6,.6],
                    keypoints=[dict(x=.4,y=.4),dict(x=.5,y=.6)],keypoint_roles=['hip','ankle'])
        candidate=dict(reason='END_ON_LOWER_BODY',confidence=.55,ground_confirmation_required=True,
                       visual_review_eligible=False,person=person,track_id=3)
        original=dict(suspected_fall=False,algorithm='test',floor_candidate=candidate,
                      overlay={'detections':[dict(track_id=3,state='PERSON'),dict(track_id=4,state='PERSON')]})
        approved,position=observer.observation_location(None,original,(640,480),None,(S,1))
        self.assertTrue(approved['suspected_fall']);self.assertFalse(original['suspected_fall'])
        self.assertEqual(approved['detection_kind'],'PERSON_ON_FLOOR');self.assertIsNotNone(position)
        self.assertEqual([d['state'] for d in approved['overlay']['detections']],['PERSON_ON_FLOOR','PERSON'])
        self.assertEqual(original['overlay']['detections'][0]['state'],'PERSON')
        self.assertTrue(observer.patient_location.call_args.kwargs['stationary'])
        self.assertFalse(observer.visual_floor_review(approved,None,False))
        for change in ({'keypoint_roles':['ankle','hip']}, {'keypoints':[dict(x=.1,y=.4),dict(x=.5,y=.6)]}):
            observer.patient_location.reset_mock()
            invalid={**original,'floor_candidate':{**candidate,'person':{**person,**change}}}
            self.assertEqual(observer.observation_location(None,invalid,(640,480),None,(S,1)),(None,None))
            observer.patient_location.assert_not_called()
        observer.patient_location.return_value=None
        self.assertEqual(observer.observation_location(None,original,(640,480),None,(S,1)),(None,None))
        observer.patient_location.reset_mock()
        self.assertEqual(observer.observation_location(None,original,(640,480),None,(S,1),moving=True),(None,None))
        observer.patient_location.assert_not_called()

    def test_depth_failures_identify_missing_anchor_overlap_and_intrinsics(self):
        import numpy as np
        from mediassist.fall_depth import measure_person
        header=SimpleNamespace(frame_id='camera_optical',stamp=SimpleNamespace(sec=1,nanosec=0))
        rgb=SimpleNamespace(header=header,width=160,height=120)
        values=np.full((120,160),1000,dtype='<u2')
        depth=SimpleNamespace(header=header,width=160,height=120,encoding='16UC1',is_bigendian=False,
                              step=320,data=values.tobytes())
        def info(focal=100.):return SimpleNamespace(header=header,width=160,height=120,binning_x=0,binning_y=0,
            roi=SimpleNamespace(x_offset=0,y_offset=0,width=0,height=0),distortion_model='',d=[],
            k=[focal,0.,80.,0.,100.,60.,0.,0.,1.])
        pair=(rgb,depth,info(),info());points=[dict(x=.4,y=.5),dict(x=.6,y=.5)]
        diagnostics={};self.assertIsNotNone(measure_person(pair,points,{'rgb_rectified':False},diagnostics))
        for invalid,expected in (([points[0],points[0]],'DEPTH_ANCHORS_OVERLAP'),
                                 ([dict(x=1.,y=.5),points[1]],'DEPTH_KEYPOINT_INVALID')):
            self.assertIsNone(measure_person(pair,invalid,{'rgb_rectified':False},diagnostics))
            self.assertEqual(diagnostics['reason'],expected)
        values[58:63,62:67]=0;depth.data=values.tobytes()
        self.assertIsNone(measure_person(pair,points,{'rgb_rectified':False},diagnostics))
        self.assertEqual(diagnostics['reason'],'DEPTH_KEYPOINT_UNAVAILABLE');self.assertEqual(diagnostics['anchor_index'],0)
        self.assertIsNone(measure_person((rgb,depth,info(),info(120.)),points,{'rgb_rectified':False},diagnostics))
        self.assertEqual(diagnostics['reason'],'DEPTH_INTRINSICS_MISMATCH')

    def test_end_on_projection_needs_two_low_anchors_and_retains_normal_span_guard(self):
        from mediassist.fall_depth import project_person,configuration
        transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.,z=.5),
                                  rotation=SimpleNamespace(x=-.5,y=.5,z=-.5,w=.5))
        measurement=([(0.,.3,1.),(0.,.35,1.7)],0)
        diagnostics={}
        self.assertIsNone(project_person(measurement,transform,0.,diagnostics=diagnostics))
        self.assertEqual(diagnostics['reason'],'DEPTH_BODY_SPAN_INVALID')
        position=project_person(measurement,transform,0.,end_on=True)
        self.assertAlmostEqual(position['xM'],1.);self.assertAlmostEqual(position['yM'],0.)
        self.assertAlmostEqual(position['floorHeightM'],.2)
        hips=([(0.,.3,1.),(.2,.35,1.1)],0)
        midpoint=project_person(hips,transform,0.,hip_midpoint=True)
        self.assertAlmostEqual(midpoint['xM'],1.05)
        self.assertAlmostEqual(midpoint['yM'],-.1)
        self.assertAlmostEqual(midpoint['floorHeightM'],.2)
        for points,expected in (([(0.,.05,1.),(0.,.05,1.5)],'PERSON_NOT_NEAR_FLOOR'),
                ([(0.,.3,1.),(0.,.3,1.1)],'DEPTH_BODY_SPAN_INVALID'),
                ([(0.,.2,1.),(0.,.45,1.7)],'PERSON_NOT_NEAR_FLOOR'),
                ([(0.,.7,1.),(0.,.7,1.7)],'DEPTH_BELOW_FLOOR')):
            self.assertIsNone(project_person((points,0),transform,0.,end_on=True,diagnostics=diagnostics))
            self.assertEqual(diagnostics['reason'],expected)
        config=dict(enabled=True,registered_depth_verified=True,depth_topic='/depth',rgb_info_topic='/rgb_info',depth_info_topic='/depth_info')
        self.assertEqual(configuration(config)['end_on_max_height_m'],.30)
        for value in (True,.1,.46,float('nan')):
            with self.assertRaises(StepError):configuration({**config,'end_on_max_height_m':value})

    def test_end_on_candidate_requires_base_floor_transform_and_fresh_map_context(self):
        from mediassist.patrol_fall import PatrolFallObserver
        observer=object.__new__(PatrolFallObserver);observer.camera_moving=Mock(return_value=False)
        observer.config={'depth':{'rgb_rectified':False,'floor_z_m':0.,'end_on_max_height_m':.30}}
        transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.,z=.5),
                                  rotation=SimpleNamespace(x=-.5,y=.5,z=-.5,w=.5))
        calls=[]
        def lookup(target,source,stamp):calls.append((target,source,stamp));return SimpleNamespace(transform=transform)
        observer.adapter=SimpleNamespace(cfg={'ros':{'base_frame':'base_footprint','map_frame':'map'}},
            worker={'navigation_session':{'session_id':S,'map_id':'map-id','map_asset_hash':'hash'}},
            tf_buffer=SimpleNamespace(lookup_transform=lookup))
        observer.stationary_revision=Mock(return_value=(S,1))
        rgb=SimpleNamespace(header=SimpleNamespace(frame_id='camera_optical',stamp='capture-time'))
        result=dict(detection_kind='PERSON_ON_FLOOR',_depth_only_candidate=True,
                    person=dict(image_width=640,image_height=480,keypoints=[{},{}]))
        modules={'rclpy.time':SimpleNamespace(Time=SimpleNamespace(from_msg=lambda v:v)),
                 'tf2_ros':SimpleNamespace(TransformException=RuntimeError)}
        with patch.dict(sys.modules,modules),patch('mediassist.patrol_fall.measure_person',return_value=([(0.,.3,1.),(0.,.35,1.7)],0)):
            position=observer.patient_location((rgb,None,None,None),result,(640,480),'captured',(S,1))
            self.assertIsNotNone(position)
            self.assertEqual(calls,[('base_footprint','camera_optical','capture-time'),('map','camera_optical','capture-time')])
            observer.stationary_revision.return_value=(S,2)
            self.assertIsNone(observer.patient_location((rgb,None,None,None),result,(640,480),'captured',(S,1)))
            self.assertEqual(observer.location_reason,'NAVIGATION_CONTEXT_CHANGED')
            observer.adapter.cfg['ros']['base_frame']='base_link'
            self.assertIsNone(observer.patient_location((rgb,None,None,None),result,(640,480),'captured',(S,1)))
            self.assertEqual(observer.location_reason,'BASE_FLOOR_FRAME_UNAVAILABLE')

    def test_edge_boxes_survive_every_metadata_validation_hop(self):
        from mediassist.media_runtime import overlay_value,overlay_packet
        value=dict(status='OK',width=640,height=480,captured_at=datetime.now(timezone.utc).isoformat(),
            detections=[dict(track_id=1,bbox=[.00025,.2,.99975,.4],confidence=.9,state='PERSON')])
        clean=overlay_value(value)
        packet=json.loads(overlay_packet(clean,S,1))
        self.assertEqual(len(packet['detections']),1)
        self.assertLessEqual(sum(packet['detections'][0]['bbox'][::2]),1.000001)
        self.assertEqual(overlay_value(clean),clean)

    def test_static_floor_review_separates_missing_pose_from_person_above_floor(self):
        from mediassist.patrol_fall import PatrolFallObserver
        observer=object.__new__(PatrolFallObserver)
        observer.config={'depth':{'near_floor_max_height_m':.45}}
        result={'detection_kind':'PERSON_ON_FLOOR'}
        self.assertIsNone(observer.patient_location(None,result,(640,480),None,None))
        self.assertFalse(observer.ground_valid(result,None))
        self.assertEqual(observer.location_reason,'LOCALIZATION_NOT_READY')
        self.assertIsNone(observer.patient_location(None,result,(640,480),None,(S,1)))
        self.assertEqual(observer.location_reason,'DEPTH_PAIR_UNAVAILABLE')
        self.assertFalse(observer.ground_valid(result,{'floorHeightM':.7}))
        self.assertEqual(observer.location_reason,'PERSON_NOT_NEAR_FLOOR')
        self.assertTrue(observer.ground_valid(result,{'floorHeightM':.2}))

    def test_latest_boxes_are_bounded_fresh_and_session_scoped(self):
        from mediassist.media_runtime import overlay_packet
        now=time.time()
        box=dict(track_id=1,bbox=[.1,.2,.3,.4],confidence=.95,state='PERSON')
        value=dict(status='OK',width=640,height=480,captured_at=datetime.fromtimestamp(now,timezone.utc).isoformat(),
                   detections=[{**box,'track_id':i} for i in range(8)])
        packet=overlay_packet(value,S,7,now)
        self.assertLessEqual(len(packet),1200)
        self.assertEqual(json.loads(packet)['media_session_id'],S)
        self.assertEqual(json.loads(packet)['seq'],7)
        self.assertEqual(json.loads(overlay_packet({**value,'detections':[]},S,8,now))['detections'],[])
        delayed={**value,'captured_at':datetime.fromtimestamp(now-2,timezone.utc).isoformat()}
        self.assertEqual(json.loads(overlay_packet(delayed,S,9,now))['seq'],9)
        for change in ({'captured_at':datetime.fromtimestamp(now-3.1,timezone.utc).isoformat()},
                       {'detections':[{**box,'bbox':[.9,.1,.5,.2]}]},
                       {'detections':[{**box,'confidence':float('nan')}]}, {'detections':[box]*9}):
            with self.subTest(change=change),self.assertRaises(ValueError):overlay_packet({**value,**change},S,9,now)

    async def test_slow_overlay_transport_cannot_accumulate_a_retry_queue(self):
        from mediassist.media_bridge import Bridge
        value=dict(status='OK',width=640,height=480,captured_at=datetime.now(timezone.utc).isoformat(),detections=[])
        bridge=object.__new__(Bridge);bridge.source=SimpleNamespace(latest_overlay=lambda:(1,value))
        attempts=[]
        async def blocked(*a,**k):attempts.append(k);await asyncio.Future()
        task=asyncio.create_task(bridge.overlays(SimpleNamespace(local_participant=SimpleNamespace(publish_data=blocked)),S))
        try:
            await asyncio.sleep(.85)
            self.assertEqual(len(attempts),1)
            self.assertFalse(attempts[0]['reliable'])
            self.assertEqual(attempts[0]['topic'],'mar.ai.overlay.v1')
            self.assertFalse(task.done())
        finally:
            task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_assist_audio_reservation_does_not_pause_camera_overlay(self):
        from mediassist.patrol_fall import PatrolFallObserver
        observer=object.__new__(PatrolFallObserver)
        observer.config=dict(rate_hz=4,min_available_memory_mb=512,cooldown_sec=60,request_timeout_sec=.5,token_file='unused')
        observer.monitoring_session=S;observer.work=None;observer.camera={}
        observer.monitor_counts=dict(no_frame=0,submitted=0,ok=0,failed=0,dropped=0)
        observer.monitor_health=Mock();observer.current_revision=Mock(return_value=None)
        observer.camera_moving=Mock(return_value=False);observer.provider_outcome=Mock();observer.state=Mock()
        observer.analyze=Mock(return_value={'suspected_fall':False,'overlay':{'detections':[]}})
        observer.be=SimpleNamespace(request=Mock());seen=[]
        def display(result=None,status='OK'):
            if status=='OK':
                seen.append(result)
                if len(seen)==2:raise asyncio.CancelledError()
        observer.display_overlay=display
        class Capture:
            depth=None
            preview=SimpleNamespace(take_sample_with_header=lambda _:((b'',640,480,1920,'rgb8',datetime.now(timezone.utc).isoformat(),time.monotonic()),None))
            async def set_enabled(self,enabled):pass
        worker=SimpleNamespace(connection_ready=True,shutting_down=False,session_fault=False,staff_qr=None,active=None)
        with patch('mediassist.patrol_fall.Capture',return_value=Capture()),\
             patch('mediassist.patrol_fall.enough_memory',return_value=True),\
             patch('mediassist.media_runtime.audio_reserved',return_value=True),\
             patch('mediassist.patrol_fall.read_token',return_value='test'),\
             patch('mediassist.patrol_fall.encode_jpeg',return_value=dict(jpeg=b'x',width=640,height=480,captured_at=NOW.isoformat())):
            with self.assertRaises(asyncio.CancelledError):await asyncio.wait_for(observer.monitor(worker),2)
        self.assertEqual(observer.analyze.call_count,2)
        observer.be.request.assert_not_called()

    async def test_fall_report_rejection_is_visible_without_secrets_or_state_flapping(self):
        from mediassist.patrol_fall import PatrolFallObserver
        observer=object.__new__(PatrolFallObserver)
        observer.config=dict(rate_hz=4,min_available_memory_mb=512,cooldown_sec=60,request_timeout_sec=.5,token_file='unused')
        observer.monitoring_session=S;observer.monitor_state=None;observer.work=None;observer.camera={}
        observer.monitor_counts=dict(no_frame=0,submitted=0,ok=0,failed=0,dropped=0)
        observer.monitor_health=Mock();observer.current_revision=Mock(return_value=None)
        observer.camera_moving=Mock(return_value=False);observer.provider_outcome=Mock()
        observer.patient_location=Mock(return_value=None);observer.ground_valid=Mock(return_value=True)
        observer.location_reason='LOCALIZATION_NOT_READY'
        observer.analyze=Mock(return_value=dict(suspected_fall=True,confidence=.9,algorithm='test'))
        observer.be=SimpleNamespace(request=Mock(side_effect=StepError('NAVIGATION_SESSION_HTTP_422','private-token')))
        seen=[]
        def display(result=None,status='OK'):
            if status=='OK':
                seen.append(result)
                if len(seen)==3:raise asyncio.CancelledError()
        observer.display_overlay=display
        class Capture:
            depth=None
            preview=SimpleNamespace(take_sample_with_header=lambda _:((b'',640,480,1920,'rgb8',datetime.now(timezone.utc).isoformat(),time.monotonic()),None))
            async def set_enabled(self,enabled):pass
        worker=SimpleNamespace(connection_ready=True,shutting_down=False,session_fault=False,staff_qr=None,active=None)
        with patch('mediassist.patrol_fall.Capture',return_value=Capture()),\
             patch('mediassist.patrol_fall.enough_memory',return_value=True),\
             patch('mediassist.media_runtime.audio_reserved',return_value=False),\
             patch('mediassist.patrol_fall.read_token',return_value='test'),\
             patch('mediassist.patrol_fall.encode_jpeg',return_value=dict(jpeg=b'x',width=640,height=480,captured_at=NOW.isoformat())),\
             self.assertLogs(level='INFO') as logs:
            with self.assertRaises(asyncio.CancelledError):await asyncio.wait_for(observer.monitor(worker),2)
        output='\n'.join(logs.output)
        self.assertIn('FALL_MONITOR_REPORT_RETRY',output)
        self.assertIn('code=NAVIGATION_SESSION_HTTP_422',output)
        self.assertNotIn('private-token',output)
        self.assertEqual(output.count('FALL_MONITOR_STATE REPORT_PENDING'),1)
        self.assertEqual(observer.be.request.call_count,1)

class MappingTests(unittest.IsolatedAsyncioTestCase):
    def test_upload_refreshes_once_after_http_auth_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);(folder/'map.tar.gz').write_bytes(b'local-map-artifact')
            manager=object.__new__(MappingManager);manager.state={'session_id':S}
            manager.adapter=SimpleNamespace(worker={'robot_id':R})
            manager.cfg={'upload_base_url':'https://be.example/api/v1/pi/mapping-sessions',
                         'upload_token_file':str(folder/'token'),'device_secret_file':str(folder/'secret')}
            sent=[]
            def response(request,**kwargs):
                sent.append(request.get_header('Authorization'))
                if len(sent)==1:raise urllib.error.HTTPError('https://be.example',401,'expired',{},None)
                return io.BytesIO(json.dumps({'artifactId':R,'assetHash':'a'*64,'stored':True}).encode())
            opener=Mock();opener.open.side_effect=response
            with patch('mediassist.mapping.urllib.request.build_opener',return_value=opener),patch('mediassist.be_token.upload_token',side_effect=['old-test-token','fresh-test-token']) as tokens:
                self.assertTrue(manager.upload(folder,{'artifact_id':R,'asset_hash':'a'*64}))
                self.assertTrue(tokens.call_args.kwargs['force'])
            self.assertEqual(sent,['Bearer old-test-token','Bearer fresh-test-token'])
            self.assertTrue((folder/'map.tar.gz').is_file())

    def test_map_upload_http_failure_preserves_artifact_and_reports_auth_code(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);(folder/'token').write_text('test-only-device-token');(folder/'map.tar.gz').write_bytes(b'local-map-artifact')
            manager=object.__new__(MappingManager);manager.state={'session_id':S}
            manager.cfg={'upload_base_url':'https://be.example/api/v1/pi/mapping-sessions','upload_token_file':str(folder/'token')}
            for status,expected in ((401,'MAP_UPLOAD_UNAUTHORIZED'),(500,'MAP_UPLOAD_HTTP_ERROR')):
                opener=Mock();opener.open.side_effect=urllib.error.HTTPError('https://be.example',status,'failed',{},None)
                with patch('mediassist.mapping.urllib.request.build_opener',return_value=opener),self.assertRaises(StepError) as caught:
                    manager.upload(folder,{'artifact_id':R,'asset_hash':'a'*64})
                self.assertEqual(caught.exception.code,expected)
                self.assertNotIn('test-only-device-token',str(caught.exception))
                self.assertTrue((folder/'map.tar.gz').is_file())

    def test_rfc3339_fractions_and_offsets_on_python310(self):
        for fraction,microsecond in (('',0),('.1',100000),('.123',123000),('.12345',123450),('.123456',123456),('.123456789',123456)):
            with self.subTest(fraction=fraction):
                self.assertEqual(parse_timestamp('2026-10-14T10:00:00'+fraction+'Z'),NOW.replace(microsecond=microsecond))
        self.assertEqual(expiry('2026-10-14T17:00:00.123456789+07:00'),NOW.replace(microsecond=123456))
        for bad in ('2026-10-14T10:00:00','2026-10-14T10:00:00.1234567890Z','2026-99-14T10:00:00Z',None):
            with self.subTest(bad=bad),self.assertRaises(ValueError):parse_timestamp(bad)

    async def test_java_nanosecond_control_survives_transport_and_lease_on_python310(self):
        # Java Instant.now() uses nine fractional digits; fixed-second fixtures
        # hid the Python 3.10 datetime.fromisoformat incompatibility.
        m={**message(),'issuedAt':'2026-10-14T10:00:00.225096447Z',
           'expiresAt':'2026-10-14T10:00:00.975096447Z','sequenceNo':1792508043041007}
        session=object.__new__(MqttSession);session.failure=None
        session.cfg={'robot_id':R,'mqtt':{'be_timeout_sec':15}};session.id=T
        session.last_be=time.monotonic();session.queue=asyncio.Queue()
        await session.queue.put(('mar/robots/'+R+'/mapping/control',json.dumps(m).encode()))
        actual=json.loads(await session.recv());lease=MappingControlLease()
        lease.accept(actual,R,T,STATE,now=NOW+timedelta(milliseconds=226),mono=1)
        self.assertEqual(lease.current(1.1),(.05,0.))
        self.assertEqual(lease.current(1.401),(0.,0.))
        with self.assertRaises(StepError):
            MappingControlLease().accept(actual,R,T,STATE,now=NOW+timedelta(seconds=1),mono=1)

    def test_future_tf_stamp_does_not_produce_negative_heartbeat_age(self):
        class FakeTime:
            @staticmethod
            def from_msg(stamp):return stamp
        adapter=object.__new__(RosAdapter);adapter.lock=threading.Lock();adapter.samples={}
        adapter.cfg={'ros':{'map_frame':'map','base_frame':'base_footprint'}}
        adapter.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000)))
        transform=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(nanoseconds=1_100_000_000)),
            transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        adapter.tf_buffer=SimpleNamespace(lookup_transform=lambda *_:transform)
        adapter.cfg['map_id']=R
        with patch.dict(sys.modules,{'rclpy.time':SimpleNamespace(Time=FakeTime)}):
            pose=adapter.telemetry()['map_pose']
        self.assertEqual(pose['age_sec'],0.);self.assertFalse(pose['fresh'])

    async def test_real_mapping_telemetry_produces_only_contract_pose_fields(self):
        adapter=object.__new__(WorkerAdapter)
        adapter.mapping=SimpleNamespace(active=True,state={'session_id':S})
        raw={'map_pose':{'value':{'map_id':R,'x_m':.1,'y_m':.2,'yaw_rad':0.},'age_sec':.02,'fresh':True}}
        with patch('mediassist.worker_adapter.RosAdapter.telemetry',return_value=raw):
            telemetry=adapter.telemetry()
        self.assertEqual(telemetry['map_pose']['value']['mapping_session_id'],S)
        manager=object.__new__(MappingManager)
        transform=SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=.1,y=.2),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        lookup=Mock(return_value=transform)
        manager.adapter=SimpleNamespace(telemetry=lambda:telemetry,tf_buffer=SimpleNamespace(lookup_transform=lookup),
            cfg={'ros':{'base_frame':'base_footprint'}})
        manager.cfg={'max_preview_rate_hz':1.,'max_map_age_sec':10,'max_preview_bytes':131072}
        manager.state={**STATE,'preview_rate_hz':1.}
        manager.lock=threading.Lock();manager.last_preview=0;manager.last_sent=0;manager.revision=3
        manager.grid=({'width':2,'height':2,'resolution':.05,'origin':[0.,0.,0.],
                       'frame_id':'map','captured_at':NOW.isoformat(),'data':bytes([255,0,50,100])},time.monotonic())
        from mediassist.footprint import cached_body_clear
        from mediassist.contracts import canonical
        cached_body_clear(manager.grid[0],dict(x_m=.05,y_m=.05,yaw_rad=0.),
                          dict(front=.01,rear=.01,left=.01,right=.01))
        with patch.dict(sys.modules,{'rclpy.time':SimpleNamespace(Time=lambda **kwargs:kwargs)}):
            payload=await manager.preview()
        packet={'robotId':R,'transportSessionId':T,'messageType':'MAP_PREVIEW','payload':payload}
        self.assertEqual(json.loads(canonical(packet)),packet)
        self.assertEqual(set(payload),{'width','height','resolution','origin','frame_id','captured_at',
                                      'revision','mapping_session_id','pose','encoding','data'})
        self.assertEqual(lookup.call_args.args,('map','base_footprint',{'nanoseconds':round(NOW.timestamp()*1e9)}))
        self.assertEqual(set(payload['pose']['value']),{'x_m','y_m','yaw_rad'})
        self.assertEqual(payload['mapping_session_id'],S)
        # Preview normalization must not mutate heartbeat session identity.
        self.assertEqual(telemetry['map_pose']['value']['mapping_session_id'],S)
        target=os.environ.get('MAR_PI_PREVIEW_FIXTURE')
        if target:
            Path(target).write_text(json.dumps({'robotId':R,'transportSessionId':T,'messageType':'MAP_PREVIEW','payload':payload}),encoding='utf-8')

    def test_preview_missing_historic_tf_does_not_use_latest_heartbeat(self):
        manager=object.__new__(MappingManager)
        telemetry=Mock()
        manager.adapter=SimpleNamespace(telemetry=telemetry,tf_buffer=SimpleNamespace(lookup_transform=Mock(side_effect=LookupError())),
            cfg={'ros':{'base_frame':'base_footprint'}})
        with patch.dict(sys.modules,{'rclpy.time':SimpleNamespace(Time=lambda **kwargs:kwargs)}):
            result=manager.preview_pose({'captured_at':NOW.isoformat(),'frame_id':'map'})
        self.assertFalse(result['fresh']);self.assertIsNone(result['value']);telemetry.assert_not_called()

    def test_idle_slam_does_not_report_navigation_pose(self):
        adapter=object.__new__(WorkerAdapter);adapter.mapping=SimpleNamespace(active=False)
        raw={'map_pose':{'value':{'map_id':R,'x_m':1,'y_m':2,'yaw_rad':0},'age_sec':.1,'fresh':True}}
        with patch('mediassist.worker_adapter.RosAdapter.telemetry',return_value=raw):
            self.assertEqual(adapter.telemetry()['map_pose'],{'value':None,'age_sec':None,'fresh':False})

    async def test_heartbeat_does_not_advertise_installed_map_in_slam_mode(self):
        for active in (False,True):
            mapping=SimpleNamespace(active=active,state={'session_id':S,'session_open':active})
            adapter=SimpleNamespace(mapping=mapping,telemetry=lambda:{})
            store=SimpleNamespace(transient=lambda kind,payload:payload)
            worker=StepWorker({'heartbeat_sec':3},{'capabilities':['sensors'],'map_id':R,'map_asset_sha256':'a'*64},store,adapter)
            worker.status=Mock(return_value='IDLE');sent=[]
            async def send(payload):sent.append(payload);raise asyncio.CancelledError
            with self.assertRaises(asyncio.CancelledError):await worker.publish(send)
            self.assertIsNone(sent[0]['currentMapId']);self.assertIsNone(sent[0]['mapAssetHash'])

    def test_deadman_and_stop_reject_old_move(self):
        lease=MappingControlLease();m=message();lease.accept(m,R,T,STATE,now=NOW,mono=1)
        self.assertEqual(lease.current(1.2),(.05,0));self.assertEqual(lease.current(1.401),(0,0))
        stop={**m,'sequenceNo':2,'linearX':0};lease.accept(stop,R,T,STATE,now=NOW,mono=1.25)
        self.assertEqual(lease.current(1.26),(0,0))
        with self.assertRaises(StepError):lease.accept(m,R,T,STATE,now=NOW,mono=1.27)
        self.assertEqual(lease.current(1.28),(0,0))
    def test_delayed_packet_only_gets_remaining_lease(self):
        lease=MappingControlLease();lease.accept(message(),R,T,STATE,now=NOW+timedelta(milliseconds=700),mono=1)
        self.assertEqual(lease.current(1.06),(0,0))
    def test_future_clock_tolerance_does_not_extend_motion_lease(self):
        for offset in (.64,1.):
            lease=MappingControlLease()
            lease.accept(message(),R,T,STATE,now=NOW-timedelta(seconds=offset),mono=1.)
            self.assertEqual(lease.current(1.2),(.05,0.))
            self.assertEqual(lease.current(1.401),(0.,0.))
        for offset in (1.001,2.):
            with self.assertRaisesRegex(StepError,'MAPPING_CONTROL_EXPIRED_OR_CLOCK_SKEW'):
                MappingControlLease().accept(message(),R,T,STATE,now=NOW-timedelta(seconds=offset),mono=1.)
    def test_future_tolerance_does_not_accept_expired_or_long_ttl(self):
        with self.assertRaisesRegex(StepError,'MAPPING_CONTROL_EXPIRED_OR_CLOCK_SKEW'):
            MappingControlLease().accept(message(),R,T,STATE,now=NOW+timedelta(seconds=.75),mono=1.)
        m={**message(),'expiresAt':(NOW+timedelta(seconds=2)).isoformat()}
        with self.assertRaisesRegex(StepError,'MAPPING_CONTROL_EXPIRED_OR_CLOCK_SKEW'):
            MappingControlLease().accept(m,R,T,STATE,now=NOW,mono=1.)
    def test_wrong_session_transport_expired_and_invalid_velocities(self):
        cases=[{'mappingSessionId':R},{'transportSessionId':R},{'durationMs':5000},{'linearX':-.31},{'linearX':float('nan')},{'linearX':.31},{'angularZ':1.01}]
        for changes in cases:
            with self.subTest(changes=changes),self.assertRaises(StepError):MappingControlLease().accept({**message(),**changes},R,T,STATE,now=NOW,mono=1)
        with self.assertRaises(StepError):MappingControlLease().accept(message(),R,T,STATE,now=NOW+timedelta(seconds=1),mono=1)
    def test_renewed_jog_remains_live_then_zero_stop_cancels_old_packets(self):
        lease=MappingControlLease()
        m={**message(),'linearX':.08,'durationMs':600,'expiresAt':(NOW+timedelta(milliseconds=1500)).isoformat()}
        lease.accept(m,R,T,STATE,now=NOW+timedelta(milliseconds=450),mono=1.)
        self.assertEqual(lease.current(1.5),(.08,0.))
        second={**m,'sequenceNo':2,'issuedAt':(NOW+timedelta(milliseconds=300)).isoformat(),
                'expiresAt':(NOW+timedelta(milliseconds=1800)).isoformat()}
        lease.accept(second,R,T,STATE,now=NOW+timedelta(milliseconds=750),mono=1.3)
        self.assertEqual(lease.current(1.8),(.08,0.))
        self.assertEqual(lease.current(1.901),(0.,0.))
        lease.accept({**second,'sequenceNo':3,'linearX':0},R,T,STATE,now=NOW+timedelta(milliseconds=800),mono=1.4)
        with self.assertRaises(StepError):lease.accept(second,R,T,STATE,now=NOW+timedelta(milliseconds=850),mono=1.45)
        self.assertEqual(lease.current(1.5),(0.,0.))

    def test_m1_reverse_and_longer_lease_cover_jitter_but_still_expire(self):
        lease=MappingControlLease()
        m={**message(),'linearX':-.1,'angularZ':-.45,'durationMs':900,'expiresAt':(NOW+timedelta(milliseconds=1500)).isoformat()}
        lease.accept(m,R,T,STATE,now=NOW+timedelta(milliseconds=500),mono=1.)
        self.assertEqual(lease.current(1.85),(-.1,-.45))
        self.assertEqual(lease.current(1.901),(0.,0.))

    def test_ramp_is_bounded_and_zero_or_expiry_stops_without_ramp(self):
        ramp=VelocityRamp();values=[ramp.update((.08,.2),i*.05) for i in range(15)]
        self.assertGreater(values[0][0],0);self.assertLess(values[0][0],.08)
        self.assertTrue(all(0 <= x <= .08 and 0 <= z <= .2 for x,z in values))
        self.assertEqual(values[-1],(.08,.2))
        self.assertEqual(ramp.update((0.,0.),1.),(0.,0.))
        self.assertLess(ramp.update((.08,0.),10.)[0],.08)
    def test_actual_be_serialized_command_if_available(self):
        fixture=Path(os.environ.get('MAR_BE_CONTROL_FIXTURE',''))
        if not fixture.is_file():self.skipTest('Set MAR_BE_CONTROL_FIXTURE to the command generated by BE MappingLiveTest')
        m=json.loads(fixture.read_text());VALIDATOR.validate(m)
        lease=MappingControlLease();lease.accept(m,R,T,STATE,now=NOW,mono=1);self.assertEqual(lease.current(1.1),(.05,0))
    async def test_transport_delivers_control_with_session_and_rejects_retained(self):
        session=object.__new__(MqttSession);session.failure=None;session.cfg={'robot_id':R,'mqtt':{'be_timeout_sec':15}};session.id=T
        session.last_be=time.monotonic();session.queue=asyncio.Queue();await session.queue.put(('mar/robots/'+R+'/mapping/control',json.dumps(message()).encode()))
        actual=json.loads(await session.recv());self.assertEqual(actual['transportSessionId'],T)
        session.loop=asyncio.get_running_loop();session.ready=asyncio.Event()
        session.on_message(None,None,SimpleNamespace(retain=True,payload=b'{}'))
        await asyncio.sleep(0);self.assertEqual(session.failure,'RETAINED_OR_OVERSIZE_MESSAGE')
    async def test_worker_returns_ack_without_starting_a_job_step(self):
        mapping=SimpleNamespace(receive_control=Mock())
        adapter=SimpleNamespace(mapping=mapping);worker=StepWorker({}, {},SimpleNamespace(session=R),adapter);worker.connection_ready=True
        peer=SimpleNamespace(id=T,publish=AsyncMock());await worker.mapping_control(message(),peer)
        ack=peer.publish.call_args.args[1];self.assertTrue(ack['accepted']);self.assertEqual(ack['commandId'],message()['commandId'])
        mapping.receive_control.side_effect=StepError('SENSORS_NOT_READY');await worker.mapping_control(message(),peer)
        self.assertFalse(peer.publish.call_args.args[1]['accepted'])
    async def test_mapping_result_supplies_real_stop_confirmation(self):
        store=SimpleNamespace(emit=Mock(return_value={'messageId':R}),set_running=Mock(),finish=Mock())
        adapter=SimpleNamespace(mapping=SimpleNamespace(active=False),perform=AsyncMock(return_value={}),stop=AsyncMock(return_value=True),motion_uncertain=False)
        worker=StepWorker({}, {},store,adapter);worker.acknowledged=AsyncMock()
        step={'stepType':'EXECUTE_SENSING','params':{'sensing_type':'MAPPING'},'timeoutSec':1}
        with patch('mediassist.step_worker.validate_result'):
            await worker.execute(step,R,R,(datetime.now(timezone.utc)+timedelta(minutes=1)).isoformat())
        self.assertTrue(worker.last_result['physicalStopConfirmed']);adapter.stop.assert_awaited_once()
    async def test_control_tick_is_not_blocked_by_slow_media(self):
        calls=[];four_ticks=asyncio.Event();media_started=asyncio.Event();release_media=asyncio.Event()
        async def tick(*a):
            calls.append(time.monotonic())
            if len(calls)>=4:four_ticks.set()
        async def preview():
            media_started.set()
            await release_media.wait()
        mapping=SimpleNamespace(tick=tick,preview=preview,camera=None)
        worker=StepWorker({}, {},SimpleNamespace(blocked=lambda:False),SimpleNamespace(mapping=mapping));worker.connection_ready=True
        a=asyncio.create_task(worker.mapping_loop(None));b=asyncio.create_task(worker.mapping_media_loop(None))
        try:
            await asyncio.wait_for(media_started.wait(),2)
            await asyncio.wait_for(four_ticks.wait(),2)
            self.assertFalse(release_media.is_set())
            self.assertGreaterEqual(len(calls),4)
        finally:
            a.cancel();b.cancel();await asyncio.gather(a,b,return_exceptions=True)
if __name__=='__main__':unittest.main(verbosity=2)
