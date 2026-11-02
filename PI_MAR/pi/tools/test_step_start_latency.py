"""ACK gating and live observations without ROS, network or motor commands."""
import asyncio,sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,AsyncMock,patch
from datetime import datetime,timedelta,timezone
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.step_worker import StepWorker

class StartTests(unittest.IsolatedAsyncioTestCase):
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
