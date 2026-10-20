"""Offline regression tests; no GPIO, ROS motion or external network."""
import asyncio
import copy
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import AsyncMock
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
for wheel in (ROOT.parent/'offline/wheels').glob('paho_mqtt-*.whl'):
    sys.path.insert(0,str(wheel))
from mediassist.navigation_profile import apply_profile,validate_profile
from mediassist.mqtt_transport import MqttSession
from mediassist.contracts import StepError
from mediassist.step_contract import validate_step
from mediassist.step_worker import StepWorker
from mediassist.step_contract import CONTRACT
R='11111111-1111-4111-8111-111111111111'
M='22222222-2222-4222-8222-222222222222'
E='33333333-3333-4333-8333-333333333333'
class BridgeTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.hardware={'map_id':M,'map_asset_sha256':'a'*64,'ros':{'map_frame':'map'},'endpoints':{}}
        self.message={'messageType':'NAV_PROFILE','robotId':R,'profileVersion':'BE-NAV-PROFILE-1','profileHash':'b'*64,
          'profile':{'map_id':M,'map_asset_sha256':'a'*64,'frame_id':'map','map_version':1,'endpoints':{E:{'map_id':M,'x_m':1,'y_m':2,'yaw_rad':0}}}}
    async def test_updates_endpoints_only_after_real_map_check(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg={'robot_id':R,'state_path':str(Path(folder)/'worker.sqlite3')}
            class Adapter: pass
            adapter=Adapter();adapter.cfg=self.hardware;adapter._check_map=AsyncMock()
            await apply_profile(self.message,cfg,self.hardware,adapter)
            adapter._check_map.assert_awaited_once_with(M)
            self.assertEqual(self.hardware['endpoints'][E]['x_m'],1)
            self.assertTrue(Path(folder,'worker.navigation.json').is_file())
    async def test_wrong_map_never_applies_coordinates(self):
        for path,value in [('map_id',R),('map_asset_sha256','c'*64),('frame_id','odom')]:
            msg=copy.deepcopy(self.message);msg['profile'][path]=value
            with self.assertRaises(StepError): validate_profile(msg,R,self.hardware)
        self.assertEqual(self.hardware['endpoints'],{})
    async def test_failed_live_map_check_does_not_write_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            class Adapter: pass
            adapter=Adapter();adapter.cfg=self.hardware;adapter._check_map=AsyncMock(side_effect=StepError('MAP_HASH_MISMATCH'))
            with self.assertRaises(StepError):
                await apply_profile(self.message,{'robot_id':R,'state_path':str(Path(folder)/'worker.sqlite3')},self.hardware,adapter)
            self.assertEqual(list(Path(folder).iterdir()),[])
    async def test_invalid_endpoint_numbers_rejected(self):
        for number in [True,float('nan'),float('inf'),100001]:
            msg=copy.deepcopy(self.message);msg['profile']['endpoints'][E]['x_m']=number
            with self.assertRaises(StepError):validate_profile(msg,R,self.hardware)
    async def test_gate_covers_multiple_topics(self):
        class Result:
            rc=0
            def is_published(self):return True
        class Client:
            def __init__(self):self.times=[]
            def publish(self,*a,**k):self.times.append(time.monotonic());return Result()
        session=object.__new__(MqttSession)
        session.cfg={'mqtt':{'max_publish_per_sec':8},'connect_timeout_sec':2}
        session.failure=None;session.prefix='mar/robots/'+R
        session.publish_lock=asyncio.Lock();session.next_publish=0;session.client=Client()
        await asyncio.gather(*(session.publish('/'+lane,{},qos=1) for lane in ['events','heartbeat','profile-acks','presence']))
        self.assertTrue(all(b-a>=.12 for a,b in zip(session.client.times,session.client.times[1:])))
    async def test_duplicate_profile_and_move_use_be_endpoint(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg={'robot_id':R,'state_path':str(Path(folder)/'worker.sqlite3'),'enabled_step_types':['MOVE']}
            class Adapter: pass
            self.hardware.update(capabilities=['nav2'],max_speed_mps=.15)
            adapter=Adapter();adapter.cfg=self.hardware;adapter._check_map=AsyncMock()
            step={'stepType':'MOVE','params':{'map_id':M,'endpoint_id':E,'mode':'NORMAL','max_speed_mps':.05}}
            with self.assertRaises(StepError):validate_step(step,cfg,self.hardware)
            await apply_profile(self.message,cfg,self.hardware,adapter)
            self.assertIs(validate_step(step,cfg,self.hardware),step)
            target=Path(folder)/'worker.navigation.json'
            before=target.stat().st_mtime_ns
            await apply_profile(self.message,cfg,self.hardware,adapter)
            self.assertEqual(before,target.stat().st_mtime_ns)
            self.assertEqual(adapter._check_map.await_count,2)
    async def test_ack_id_registered_before_send_yields(self):
        class Store:
            def pending(self):return [{'messageId':'event-1'}]
        worker=object.__new__(StepWorker)
        worker.store=Store();worker.sent={};worker.cfg={'heartbeat_sec':10**12}
        async def send(event):
            self.assertIn(event['messageId'],worker.sent)
            raise asyncio.CancelledError()
        with self.assertRaises(asyncio.CancelledError):await worker.publish(send)

    def hello_worker(self):
        class Store:
            def states(self):return []
            def transient(self,kind,payload):return {'messageType':kind,'payload':payload}
        class Adapter: mapping=None
        cfg={'robot_id':R,'enabled_step_types':['WAIT'],'unlock_policy':'DISABLED','mapping':{'enabled':False}}
        worker=StepWorker(cfg,{'capabilities':[]},Store(),Adapter())
        async def outgoing(send):await asyncio.Future()
        worker.publish=outgoing
        return worker

    def hello_ack(self,**changes):
        return {'messageType':'HELLO_ACK','robotId':R,'contract':CONTRACT,'accepted':True,**changes}

    async def test_repeated_hello_ack_keeps_receiver_and_commands_alive(self):
        worker=self.hello_worker()
        class Session:
            def __init__(self):self.queue=asyncio.Queue();self.sent=[]
            async def send(self,raw):self.sent.append(json.loads(raw))
            async def recv(self):return await self.queue.get()
        session=Session()
        for message in [self.hello_ack(),self.hello_ack(),self.hello_ack(),{'messageType':'COMMAND'}]:
            session.queue.put_nowait(json.dumps(message))
        received=asyncio.Event()
        async def command(message):received.set()
        worker.command=AsyncMock(side_effect=command)
        task=asyncio.create_task(worker.connected(session))
        try:
            await asyncio.wait_for(received.wait(),1)
            self.assertFalse(task.done())
            self.assertTrue(worker.connection_ready)
            worker.command.assert_awaited_once()
            self.assertEqual(len(session.sent),1) # No new HELLO / repeated physical action.
        finally:
            task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_invalid_repeated_hello_ack_still_rejected(self):
        for changes in ({'accepted':False},{'accepted':1},{'contract':'OTHER'},{'robotId':M}):
            with self.subTest(changes=changes):
                worker=self.hello_worker()
                class Session:
                    def __init__(self,messages):self.messages=iter(messages)
                    async def send(self,raw):pass
                    async def recv(self):return json.dumps(next(self.messages))
                session=Session([self.hello_ack(),self.hello_ack(**changes)])
                with self.assertRaises(StepError) as caught:
                    await worker.connected(session)
                self.assertEqual(caught.exception.code,'HELLO_REJECTED')

    async def test_unrecognized_message_still_fails_closed(self):
        worker=self.hello_worker()
        class Session:
            def __init__(self):self.messages=iter([self_outer.hello_ack(),{'messageType':'UNSUPPORTED'}])
            async def send(self,raw):pass
            async def recv(self):return json.dumps(next(self.messages))
        self_outer=self
        with self.assertRaises(StepError) as caught:
            await worker.connected(Session())
        self.assertEqual(caught.exception.code,'UNKNOWN_MESSAGE_TYPE')

    async def test_transport_drops_old_session_ack_and_consumes_server_heartbeat(self):
        session=object.__new__(MqttSession)
        session.cfg={'robot_id':R,'mqtt':{'be_timeout_sec':15}}
        session.id='current-session';session.failure=None;session.last_be=time.monotonic()
        session.queue=asyncio.Queue()
        for message in [
            {**self.hello_ack(),'transportSessionId':'old-session'},
            {'messageType':'SERVER_HEARTBEAT','robotId':R,'transportSessionId':session.id},
            {**self.hello_ack(),'transportSessionId':session.id}]:
            session.queue.put_nowait(('mar/robots/'+R+'/event-acks',json.dumps(message).encode()))
        result=json.loads(await asyncio.wait_for(session.recv(),1))
        self.assertEqual(result,self.hello_ack())
        self.assertTrue(session.queue.empty())
if __name__=='__main__': unittest.main(verbosity=2)
