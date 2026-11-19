"""Opt-in local broker test: MAR_TEST_MQTT_PORT=18884. No ROS, BE server or physical robot."""
import asyncio
import json
import os
import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import AsyncMock
from types import SimpleNamespace as N
from uuid import uuid4
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.step_store import StepStore
from mediassist.step_worker import StepWorker


@unittest.skipUnless(os.getenv('MAR_TEST_MQTT_PORT'), 'Local broker test is opt-in')
class BrokerCancellationTest(unittest.IsolatedAsyncioTestCase):
    async def test_qos_one_cancel_stops_worker_and_returns_durable_receipt(self):
        import paho.mqtt.client as mqtt
        loop=asyncio.get_running_loop()
        received=asyncio.Queue()
        ready=asyncio.Event()
        topic='mar/cancellation-test/'+str(uuid4())
        client=mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect=lambda c,u,f,r,p: c.subscribe(topic+'/#',qos=1)
        client.on_subscribe=lambda *args: loop.call_soon_threadsafe(ready.set)
        client.on_message=lambda c,u,m: loop.call_soon_threadsafe(received.put_nowait,(m.topic,json.loads(m.payload)))
        client.connect('127.0.0.1',int(os.environ['MAR_TEST_MQTT_PORT']))
        client.loop_start()
        try:
            await asyncio.wait_for(ready.wait(),5)
            with tempfile.TemporaryDirectory() as folder:
                robot=str(uuid4())
                store=StepStore(str(Path(folder)/'worker.db'),robot)
                worker=StepWorker.__new__(StepWorker)
                worker.store=store;worker.cfg={'robot_id':robot};worker.shutting_down=False
                worker.outbox_wakeup=asyncio.Event();worker.adapter=N(stop=AsyncMock(return_value=True))
                step=dict(jobId=str(uuid4()),jobStepId=str(uuid4()),operationKey='test:wait:1',stepType='WAIT',params={})
                store.accept(step);store.set_running(step)
                worker.active=step;worker.last_result=None
                worker.task=asyncio.create_task(asyncio.sleep(60))
                identifier=str(uuid4())
                command=dict(protocolVersion='1.0',contract='MAR-STEP-2',messageType='COMMAND',robotId=robot,
                    commandType='CANCEL_STEP',commandId=identifier,correlationId=identifier,
                    expiresAt=(datetime.now(timezone.utc)+timedelta(seconds=30)).isoformat(),
                    jobId=step['jobId'],jobStepId=step['jobStepId'])
                try:
                    client.publish(topic+'/command',json.dumps(command),qos=1,retain=False)
                    route,body=await asyncio.wait_for(received.get(),5)
                    self.assertEqual(route,topic+'/command')
                    await worker.command(body)
                    self.assertIsNone(worker.active);self.assertTrue(worker.task.done())
                    self.assertEqual(store.previous(step)['status'],'CANCELLED')
                    receipt=next(e for e in store.pending() if e['messageType']=='COMMAND_ACK')
                    client.publish(topic+'/receipt',json.dumps(receipt),qos=1,retain=False)
                    route,body=await asyncio.wait_for(received.get(),5)
                    self.assertEqual(route,topic+'/receipt')
                    self.assertTrue(body['payload']['accepted'])
                    self.assertTrue(body['payload']['physicalStopConfirmed'])
                    await worker.command(command)  # QoS replay must not stop twice.
                    worker.adapter.stop.assert_awaited_once()
                finally:
                    worker.task.cancel();await asyncio.gather(worker.task,return_exceptions=True)
                    store.close()
        finally:
            client.disconnect();client.loop_stop()


if __name__=='__main__':
    unittest.main()
