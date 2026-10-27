"""Local handoff, source mapping and continuous input without robot hardware."""
import asyncio
from contextlib import nullcontext
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import AsyncMock,Mock,patch

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools'),str(ROOT/'pi/tools')]
from mediassist.gamepad import GamepadInput,configuration
from mediassist.manual_drive import ManualDrive
from mediassist.step_worker import StepWorker
from test_gamepad import MappingTests,arm
from test_camera_config import CameraConfigTests
import configure_bare_mecanum as tool


def cfg():
    return configuration({'gamepad':{'layout':'vendor_mecanum','angular_axis':2,
        'session_required':False,'max_run_sec':0.,'forward_mps':.25,'reverse_mps':.25,'lateral_mps':.25,'angular_rps':.8}})


def twist():return NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
ROS={'geometry_msgs.msg':NS(Twist=twist),'std_msgs.msg':NS(String=lambda **kw:NS(**kw))}


class ManualTests(unittest.TestCase):
    def test_vendor_axes_keep_left_xy_and_right_yaw_separate(self):
        pad=GamepadInput(cfg());arm(pad)
        pad.feed([1.,1.,-1.,0.,0.,0.],10.4)
        self.assertEqual(pad.current('22222222-2222-4222-8222-222222222222',True,10.4),(.25,.25,-.8))

    def test_continuous_hold_has_no_three_second_cutoff_but_stale_stops(self):
        pad=GamepadInput(cfg());arm(pad)
        for i in range(200):
            now=10.4+i*.05;pad.feed([0.,1.,0.],now)
            self.assertEqual(pad.current('22222222-2222-4222-8222-222222222222',True,now),(.25,0.,0.))
        self.assertEqual(pad.current('22222222-2222-4222-8222-222222222222',True,now+.26),(0.,0.,0.))
        pad.feed([0.,1.,0.],now+.3)
        self.assertEqual(pad.current('22222222-2222-4222-8222-222222222222',True,now+.3),(0.,0.,0.))

    def manager(self):
        m=MappingTests().manager();c=cfg();m.cfg['gamepad']=c;m.gamepad=GamepadInput(c)
        m.state.update(session_open=False,state='SAVED')
        m.adapter.cfg['max_speed_mps']=.25;m.adapter.cfg['motion']['max_angular_speed_rps']=.8
        m.manual_drive=ManualDrive(c)
        arm(m.gamepad,'local-manual')
        m.gamepad.feed([.5,1.,.5],10.4)
        return m

    def test_manual_drives_without_mapping_session_or_mqtt(self):
        m=self.manager()
        with patch.dict(sys.modules,ROS),patch('mediassist.manual_drive.time.monotonic',return_value=10.4):
            m.manual_drive.tick(m,False,True)
        v=m.velocity.publish.call_args.args[0]
        self.assertGreater(v.linear.x,0);self.assertGreater(v.linear.y,0);self.assertGreater(v.angular.z,0)
        self.assertTrue(m.adapter.lease_message.call_args.args[0])

    def test_vector_speed_cap_and_release_stop(self):
        d=ManualDrive(cfg())
        for i in range(40):v=d.ramp((.25,.25,.8),i*.05,.25)
        self.assertLessEqual(math.hypot(*v[:2]),.25000001)
        self.assertEqual(d.ramp((0.,0.,0.),2.,.25),(0.,0.,0.))

    def test_workflow_takes_authority_without_repeated_zero_against_it(self):
        m=self.manager()
        with patch.dict(sys.modules,ROS),patch('mediassist.manual_drive.time.monotonic',return_value=10.4):
            m.manual_drive.tick(m,True,True)
            m.manual_drive.tick(m,True,False)
            m.velocity.publish.reset_mock();m.adapter.lease_message.reset_mock()
            m.manual_drive.tick(m,True,False)
        m.velocity.publish.assert_not_called();m.adapter.lease_message.assert_not_called()
        self.assertFalse(m.gamepad.armed)

    def test_shutdown_recovery_and_missing_source_block_local_manual(self):
        for variant in ('inhibit','recovery','source','sensor'):
            m=self.manager()
            if variant=='inhibit':m.manual_inhibited=True
            if variant=='recovery':m.state['state']='RECOVERY_REQUIRED'
            if variant=='source':m.adapter.node.get_publishers_info_by_topic.return_value=[]
            if variant=='sensor':m.adapter.health=lambda:{'sensors_ready':False}
            with patch.dict(sys.modules,ROS),patch('mediassist.manual_drive.time.monotonic',return_value=10.4):
                m.manual_drive.tick(m,False,True)
            self.assertFalse(m.adapter.lease_message.call_args.args[0],variant)

    def test_body_preset_preserves_secrets_camera_journal_endpoints_and_flags(self):
        with tempfile.TemporaryDirectory() as folder:
            path,driver,hw=CameraConfigTests().fixture(folder)
            before=json.loads(path.read_bytes());old_hw=json.loads(hw.read_bytes());camera=copy.deepcopy(before['mapping']['camera'])
            with patch.object(tool,'stopped_configuration',return_value=nullcontext()):
                result=tool.configure(path,apply=True)
            new_hw=json.loads(Path(result['hardware_profile']).read_bytes())
            
            for k,v in before['mqtt'].items():self.assertEqual(result['mqtt'][k],v)
            self.assertEqual(result['state_path'],before['state_path'])
            # load() adds camera defaults; the existing configured values are preserved.
            for k,v in camera.items():self.assertEqual(result['mapping']['camera'][k],v)
            for k in ('commissioned','cart_footprint_validated','watchdog_validated'):
                self.assertEqual(new_hw['motion'][k],old_hw['motion'][k])
            self.assertEqual(new_hw['endpoints'],old_hw['endpoints'])
            self.assertTrue(result['mapping']['autonomy']['enabled'])
            self.assertFalse(result['mapping']['gamepad']['session_required'])
            self.assertEqual(json.loads(hw.read_bytes()),old_hw)


class WorkerTests(unittest.IsolatedAsyncioTestCase):
    async def test_scoped_cleanup_stop_preempts_held_stick_but_other_steps_do_not(self):
        for action,job,session,accepted in [('STOP','job','session',True),('STOP','other','session',False),
                                           ('STOP','job','other',False),('START','job','session',False),('SAVE','job','session',False)]:
            mapping=NS(state={'job_id':'job','session_id':'session'},active=True,manual_drive=Mock(),gamepad=Mock())
            mapping.manual_drive.busy.return_value=True
            store=Mock();store.cached_command.return_value=False;store.previous.return_value=None;store.blocked.return_value=False
            store.emit.return_value={'messageId':'ack'}
            worker=StepWorker({'robot_id':'robot'}, {}, store, NS(mapping=mapping,motion_uncertain=False));worker.execute=AsyncMock()
            command={'commandType':'EXECUTE_STEP','commandId':'cmd','correlationId':'correlation','expiresAt':'later',
                'step':{'stepType':'EXECUTE_SENSING','jobId':job,'params':{'sensing_type':'MAPPING','action':action,'mapping_session_id':session}}}
            with patch('mediassist.step_worker.validate_command'),patch('mediassist.step_worker.validate_step'):
                await worker.command(command)
                if worker.task:await worker.task
            self.assertEqual(store.accept.called,accepted,(action,job,session))
            if accepted:
                mapping.gamepad.stop.assert_called_once_with('WORKFLOW_HANDOFF_RETURN_NEUTRAL')
                self.assertIsNotNone(worker.active)
            else:worker.execute.assert_not_called()

    async def test_local_loop_runs_while_mqtt_connection_is_waiting(self):
        mapping=NS(manual_drive=Mock(),gamepad=NS(cfg={'session_required':False}),tick=AsyncMock())
        store=Mock();store.blocked.return_value=False
        worker=StepWorker({}, {}, store,NS(mapping=mapping))
        worker.disconnect=AsyncMock()
        class WaitingSession:
            def __init__(self,*args):pass
            async def __aenter__(self):await asyncio.Event().wait()
            async def __aexit__(self,*args):pass
        with patch('mediassist.step_worker.MqttSession',WaitingSession):
            run=asyncio.create_task(worker.run())
            try:
                async def three_ticks():
                    while mapping.tick.await_count<3:await asyncio.sleep(.01)
                await asyncio.wait_for(three_ticks(),1.)
            finally:
                run.cancel();await asyncio.gather(run,return_exceptions=True)
        self.assertGreaterEqual(mapping.tick.await_count,3)
        self.assertTrue(mapping.manual_inhibited)


if __name__=='__main__':unittest.main()
