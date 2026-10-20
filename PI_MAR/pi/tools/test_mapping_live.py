"""Offline tests: transport scope, expiry, motion watchdog and media separation. No robot motion."""
import asyncio,copy,json,os,sys,time,unittest
import threading
import tempfile
import urllib.error
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
class MappingTests(unittest.IsolatedAsyncioTestCase):
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
        manager.adapter=SimpleNamespace(telemetry=lambda:telemetry)
        manager.cfg={'max_preview_rate_hz':1.,'max_map_age_sec':10,'max_preview_bytes':131072}
        manager.state={**STATE,'preview_rate_hz':1.}
        manager.lock=threading.Lock();manager.last_preview=0;manager.last_sent=0;manager.revision=3
        manager.grid=({'width':2,'height':2,'resolution':.05,'origin':[0.,0.,0.],
                       'frame_id':'map','captured_at':NOW.isoformat(),'data':bytes([255,0,50,100])},time.monotonic())
        payload=await manager.preview()
        self.assertEqual(set(payload['pose']['value']),{'x_m','y_m','yaw_rad'})
        self.assertEqual(payload['mapping_session_id'],S)
        # Preview normalization must not mutate heartbeat session identity.
        self.assertEqual(telemetry['map_pose']['value']['mapping_session_id'],S)
        target=os.environ.get('MAR_PI_PREVIEW_FIXTURE')
        if target:
            Path(target).write_text(json.dumps({'robotId':R,'transportSessionId':T,'messageType':'MAP_PREVIEW','payload':payload}),encoding='utf-8')

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
    def test_wrong_session_transport_expired_and_invalid_velocities(self):
        cases=[{'mappingSessionId':R},{'transportSessionId':R},{'durationMs':5000},{'linearX':-.01},{'linearX':float('nan')},{'linearX':.11},{'angularZ':.31}]
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
        worker=StepWorker({}, {},None,SimpleNamespace(mapping=mapping));worker.connection_ready=True
        a=asyncio.create_task(worker.mapping_loop(None));b=asyncio.create_task(worker.mapping_media_loop(None))
        try:
            await asyncio.wait_for(media_started.wait(),2)
            await asyncio.wait_for(four_ticks.wait(),2)
            self.assertFalse(release_media.is_set())
            self.assertGreaterEqual(len(calls),4)
        finally:
            a.cancel();b.cancel();await asyncio.gather(a,b,return_exceptions=True)
if __name__=='__main__':unittest.main(verbosity=2)
