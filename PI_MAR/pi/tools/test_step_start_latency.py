"""ACK gating and live observations without ROS, network or motor commands."""
import asyncio,sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,AsyncMock,patch
from datetime import datetime,timedelta,timezone
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.step_worker import StepWorker
from mediassist.mqtt_transport import MqttSession
from mediassist.contracts import StepError

class StartTests(unittest.IsolatedAsyncioTestCase):
    async def test_initial_hello_waits_configured_20_seconds_and_reports_its_own_timeout(self):
        cfg={'robot_id':'robot','enabled_step_types':['MOVE'],'unlock_policy':'DISABLED','mapping':{'enabled':False},'hello_timeout_sec':20}
        store=Mock();store.transient.return_value={};store.states.return_value=[]
        worker=StepWorker(cfg,{'capabilities':[]},store,N(mapping=None));session=N(send=AsyncMock(),recv=AsyncMock())
        async def timeout(coro,seconds):
            self.assertEqual(seconds,20);coro.close();raise asyncio.TimeoutError()
        with patch('mediassist.step_worker.asyncio.wait_for',side_effect=timeout):
            with self.assertRaises(StepError) as error:await worker.connected(session)
        self.assertEqual(error.exception.code,'HELLO_ACK_TIMEOUT');self.assertFalse(worker.connection_ready)

    async def test_hello_gets_its_own_window_after_slow_tcp_setup(self):
        session=object.__new__(MqttSession);session.cfg={'robot_id':'robot','mqtt':{'be_timeout_sec':15}}
        session.id='current';session.failure=None;session.last_be=0.;session.hello_deadline=215.;session.queue=asyncio.Queue()
        session.queue.put_nowait(('mar/robots/robot/event-acks',b'{"robotId":"robot","transportSessionId":"current","messageType":"HELLO_ACK","accepted":true}'))
        with patch('mediassist.mqtt_transport.time',N(monotonic=lambda:200.)):
            self.assertIn('HELLO_ACK',await session.recv())
        self.assertIsNone(session.hello_deadline);self.assertEqual(session.last_be,200.)

    async def test_hello_deadline_cannot_be_extended_by_a_recent_heartbeat(self):
        session=object.__new__(MqttSession);session.cfg={'robot_id':'robot','mqtt':{'be_timeout_sec':15}}
        session.failure=None;session.last_be=199.;session.hello_deadline=199.
        with patch('mediassist.mqtt_transport.time',N(monotonic=lambda:200.)):
            with self.assertRaises(StepError) as error:await session.recv()
        self.assertEqual(error.exception.code,'HELLO_ACK_TIMEOUT')

    async def test_physical_effects_still_wait_for_both_committed_acks_without_sqlite_polling(self):
        acked=set();store=Mock();store.is_acked.side_effect=lambda mid:mid in acked
        store.emit.return_value={'messageId':'started'}
        adapter=N(mapping=None,motion_uncertain=False,perform=AsyncMock(return_value={}),stop=AsyncMock(return_value=True))
        w=StepWorker({'ack_timeout_sec':1},{},store,adapter);w.connection_ready=True
        step={'stepType':'MOVE','params':{},'timeoutSec':1}
        with patch('mediassist.step_worker.validate_result'):
            task=asyncio.create_task(w.execute(step,'correlation','accepted',(datetime.now(timezone.utc)+timedelta(seconds=20)).isoformat()))
            await asyncio.sleep(.01);adapter.perform.assert_not_awaited();store.emit.assert_not_called()
            self.assertEqual(store.is_acked.call_count,1)
            acked.add('accepted');w.ack_waiters['accepted'].set()
            async def until_started():
                while 'started' not in w.ack_waiters:await asyncio.sleep(.01)
            await asyncio.wait_for(until_started(),.5);adapter.perform.assert_not_awaited()
            self.assertIn('started',w.ack_waiters)
            acked.add('started');w.ack_waiters['started'].set();await task
        adapter.perform.assert_awaited_once();self.assertEqual(w.last_result['status'],'SUCCEEDED');self.assertFalse(w.ack_waiters)

    async def test_negotiated_move_still_waits_for_one_committed_start_and_emits_no_second_request(self):
        acked=set();store=Mock();store.is_acked.side_effect=lambda mid:mid in acked
        adapter=N(mapping=None,motion_uncertain=False,perform=AsyncMock(return_value={}),stop=AsyncMock(return_value=True))
        w=StepWorker({'ack_timeout_sec':1},{},store,adapter);w.connection_ready=True
        step={'stepType':'MOVE','params':{},'timeoutSec':1}
        with patch('mediassist.step_worker.validate_result'):
            task=asyncio.create_task(w.execute(step,'correlation','combined',(datetime.now(timezone.utc)+timedelta(seconds=20)).isoformat(),combined=True))
            await asyncio.sleep(.01);adapter.perform.assert_not_awaited();store.set_running.assert_not_called()
            acked.add('combined');w.ack_waiters['combined'].set();await task
        adapter.perform.assert_awaited_once();store.set_running.assert_called_once_with(step)
        self.assertFalse(any(call.args[0]=='STEP_STARTED' for call in store.emit.call_args_list))
        self.assertEqual(w.last_result['status'],'SUCCEEDED')

    async def test_expired_combined_permission_never_moves(self):
        store=Mock();store.is_acked.return_value=True
        adapter=N(mapping=None,motion_uncertain=False,perform=AsyncMock(),stop=AsyncMock(return_value=True))
        w=StepWorker({'ack_timeout_sec':1},{},store,adapter);w.connection_ready=True
        await w.execute({'stepType':'MOVE','params':{},'timeoutSec':1},'c','ack',(datetime.now(timezone.utc)-timedelta(seconds=1)).isoformat(),combined=True)
        adapter.perform.assert_not_awaited();store.set_running.assert_not_called()
        self.assertEqual(w.last_result['errorCode'],'COMMAND_EXPIRED_BEFORE_EXECUTION')

    async def test_cancellation_during_combined_ack_wait_never_moves(self):
        store=Mock();store.is_acked.return_value=False
        adapter=N(mapping=None,motion_uncertain=False,perform=AsyncMock(),stop=AsyncMock(return_value=True))
        w=StepWorker({'ack_timeout_sec':1},{},store,adapter);w.connection_ready=True
        step={'stepType':'MOVE','params':{},'timeoutSec':1}
        task=asyncio.create_task(w.execute(step,'c','ack',(datetime.now(timezone.utc)+timedelta(seconds=20)).isoformat(),combined=True))
        await asyncio.sleep(.01);task.cancel();await task
        adapter.perform.assert_not_awaited();self.assertEqual(w.last_result['status'],'CANCELLED')
        self.assertTrue(w.last_result['physicalStopConfirmed'])

    def test_combined_capability_is_only_advertised_for_navigation_move(self):
        hardware={'capabilities':['nav2']}
        w=StepWorker({'enabled_step_types':['MOVE']},hardware,Mock(),Mock())
        self.assertNotIn('move-start-commit-v1',w.advertised_capabilities())
        w.cfg['navigation_session']={'session_id':'current'}
        self.assertIn('move-start-commit-v1',w.advertised_capabilities())
        self.assertEqual(hardware['capabilities'],['nav2'])

    async def test_new_receipt_wakes_the_outbox_instead_of_waiting_for_heartbeat(self):
        pending=[];store=Mock();store.pending.side_effect=lambda:list(pending);store.transient.side_effect=lambda kind,body:{'messageType':kind}
        adapter=N(mapping=None,telemetry=lambda:{})
        w=StepWorker({'heartbeat_sec':3},{'capabilities':[],'map_id':'map','map_asset_sha256':'hash'},store,adapter)
        w.status=lambda:'IDLE';sent=asyncio.Queue()
        async def send(value):await sent.put(value)
        task=asyncio.create_task(w.publish(send))
        try:
            self.assertEqual((await sent.get())['messageType'],'HEARTBEAT')
            event={'messageId':'receipt','messageType':'COMMAND_ACK'};store.emit.return_value=event;pending.append(event)
            w.receipt({'commandId':'command','commandType':'EXECUTE_STEP','correlationId':'correlation'},True)
            self.assertEqual(await asyncio.wait_for(sent.get(),.15),event)
        finally:task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_observation_uses_the_same_session_and_never_creates_a_step_or_heartbeat(self):
        context={'session_id':'session','map_id':'map','map_asset_hash':'a'*64}
        value={'navigation_session_id':'session','initialization_revision':1,'x_m':.1,'y_m':.2,'yaw_rad':.3}
        adapter=N(telemetry=lambda:{'map_pose':{'fresh':True,'age_sec':.01,'value':value},'localization':{'ready':True}})
        store=Mock();store.session='producer'
        w=StepWorker({'robot_id':'robot','navigation_session':context},{},store,adapter);w.connection_ready=True
        sample=[]
        async def publish(topic,payload,qos):sample.append((topic,payload,qos));raise asyncio.CancelledError()
        with self.assertRaises(asyncio.CancelledError):await w.navigation_pose_loop(N(id='transport',publish=publish))
        topic,payload,qos=sample[0];self.assertEqual((topic,qos),('/navigation/pose',0));self.assertEqual(payload['navigationSessionId'],'session')
        self.assertEqual(payload['pose'],dict(xM=.1,yM=.2,yawRad=.3));store.emit.assert_not_called()
if __name__=='__main__':unittest.main()
