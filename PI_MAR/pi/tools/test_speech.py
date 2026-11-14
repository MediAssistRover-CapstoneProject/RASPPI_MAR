"""Contract and process lifecycle tests; no speaker or robot is used."""
import asyncio
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import AsyncMock, Mock, patch
from types import SimpleNamespace as N
from contextlib import nullcontext
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mediassist.speech import SpeechPlayer,audio_device
from mediassist.contracts import StepError
from mediassist.step_contract import validate_step,validate_result,validate_command

PARAMS={'prompt_key':'DEMO_SPEAK','action':'SPEAK','data':{'text':'Xin chào. Robot đã nhận lệnh.','language_code':'vi'}}

class SpeechTests(unittest.IsolatedAsyncioTestCase):
    def test_contract_rejects_commands_and_unbounded_text(self):
        import copy
        cfg={'enabled_step_types':['INTERACT'],'speech':{'enabled':True}}
        step={'stepType':'INTERACT','params':PARAMS,'timeoutSec':90}
        validate_step(step,cfg,{})
        for data in ({'text':''},{'text':' '*5},{'text':'a'*501},{'text':'hi','volume':101},
                     {'text':'hi','speed_wpm':80.5},{'text':'hi','language_code':'vi;reboot'}, {'text':'hi','script':'reboot'}):
            s=copy.deepcopy(step);s['params']['data']=data
            with self.subTest(data=data),self.assertRaises(StepError):validate_step(s,cfg,{})
        with self.assertRaises(StepError):validate_step(step,{**cfg,'speech':{'enabled':False}},{})

    def test_usb_card_identity_not_numeric_index(self):
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/'cards'
            f.write_text(' 0 [HDMI ]: HDA - HDMI\n 2 [Device ]: USB-Audio - USB Audio Device\n')
            self.assertEqual(audio_device({'device':'auto'},f),'plughw:CARD=Device,DEV=0')
            f.write_text(f.read_text()+' 3 [Second ]: USB-Audio - Second USB\n')
            with self.assertRaises(StepError):audio_device({'device':'auto'},f)

    async def test_text_uses_stdin_result_only_after_playback(self):
        player=SpeechPlayer({'enabled':True,'device':'plughw:CARD=Device,DEV=0'})
        calls=[]
        async def process(argv,content=None):
            calls.append((argv,content))
            if '-w' in argv:Path(argv[argv.index('-w')+1]).write_bytes(b'W'*100)
        player.process=process
        params={**PARAMS,'data':{'text':'--help; $(reboot)'}}
        with patch('mediassist.speech.shutil.which',side_effect=lambda x:'/usr/bin/'+x):
            result=await player.speak(params)
        self.assertEqual(calls[0][1],b'--help; $(reboot)')
        self.assertNotIn(params['data']['text'],calls[0][0])
        self.assertEqual(calls[1][0][0],'/usr/bin/aplay')
        validate_result({'stepType':'INTERACT','params':params},result)
        self.assertFalse(result['values']['audible_verified'])
        self.assertFalse(Path(calls[1][0][-1]).exists())

    async def test_cancellation_terminates_audio_child(self):
        from unittest.mock import Mock
        proc=Mock(returncode=None);started=asyncio.Event()
        async def communicate(_):started.set();await asyncio.Future()
        proc.communicate=communicate;proc.wait=AsyncMock(return_value=0)
        player=SpeechPlayer({'enabled':True})
        with patch('mediassist.speech.asyncio.create_subprocess_exec',AsyncMock(return_value=proc)):
            task=asyncio.create_task(player.process(['aplay','test.wav']))
            await started.wait();task.cancel()
            with self.assertRaises(asyncio.CancelledError):await task
        proc.terminate.assert_called_once()
        proc.wait.assert_awaited_once()

    async def test_playback_start_callback_follows_the_last_guard_and_precedes_alsa(self):
        player=SpeechPlayer({'enabled':True,'device':'plughw:CARD=Device,DEV=0'})
        events=[]
        def guard():events.append('guard');return True
        async def process(argv,content=None):
            if '-w' in argv:
                events.append('synthesis');Path(argv[argv.index('-w')+1]).write_bytes(b'W'*100)
            else:events.append('playback')
        player.process=process
        with patch('mediassist.speech.shutil.which',side_effect=lambda name:'/usr/bin/'+name):
            await player.speak(PARAMS,playback_guard=guard,on_playback_start=lambda:events.append('started'))
        self.assertEqual(events,['guard','synthesis','guard','started','playback'])

    async def test_revoked_speech_never_signals_playback_start(self):
        player=SpeechPlayer({'enabled':True,'device':'plughw:CARD=Device,DEV=0'})
        started=Mock()
        async def process(argv,content=None):
            self.assertIn('-w',argv)
            Path(argv[argv.index('-w')+1]).write_bytes(b'W'*100)
        player.process=process
        with patch('mediassist.speech.shutil.which',side_effect=lambda name:'/usr/bin/'+name):
            with self.assertRaisesRegex(StepError,'NAVIGATION_GUIDANCE_EXPIRED'):
                await player.speak(PARAMS,playback_guard=Mock(side_effect=[True,False]),on_playback_start=started)
        started.assert_not_called()

    async def test_process_failure_keeps_bounded_stage_diagnostics(self):
        player=SpeechPlayer({'enabled':True})
        with self.assertLogs(level='ERROR'):
            with self.assertRaises(StepError) as caught:
                await player.process([sys.executable,'-c',"import sys; sys.stderr.write('x'*9000+' ALSA failure'); sys.exit(7)"])
        self.assertEqual(caught.exception.details['exitCode'],7)
        self.assertLessEqual(len(caught.exception.details['message']),2048)
        self.assertTrue(caught.exception.details['message'].endswith('ALSA failure'))

    def test_be_exported_speech_command(self):
        import os
        fixture=os.environ.get('MAR_BE_SPEECH_FIXTURE')
        if not fixture:self.skipTest('Set MAR_BE_SPEECH_FIXTURE for the Java/Python contract check')
        command=json.loads(Path(fixture).read_text())
        command.pop('transportSessionId')
        validate_command(command,command['robotId'],check_expiry=False)
        validate_step(command['step'],{'enabled_step_types':['INTERACT'],'speech':{'enabled':True}}, {})

class NavigationGuidanceTests(unittest.IsolatedAsyncioTestCase):
    def guidance(self):
        from mediassist.navigation_guidance import NavigationGuidance
        context = dict(session_id='session', map_id='map', map_asset_hash='a'*64)
        step = dict(jobId='job', jobStepId='move', params={'max_speed_mps': .35})
        state = {'localization': {'ready': True, 'reason': 'LOCALIZED'},
                 'map_pose': {'fresh': True, 'age_sec': .1, 'value': {
                     'x_m': .5, 'y_m': 0., 'yaw_rad': 0., 'map_id': 'map',
                     'navigation_session_id': 'session', 'initialization_revision': 1}}}
        path = dict(jobId='job', jobStepId='move', sessionId='session', mapId='map',
                    mapAssetHash='a'*64, initializationRevision=1,
                    points=[{'xM': x, 'yM': y} for x, y in ((0., 0.), (1., 0.), (1., 1.))])
        adapter = N(active_navigation_context={'initialization_revision': 1}, motion_uncertain=False,
                    navigation_guidance_path=path, navigation_guidance_tracking=True,
                    telemetry=lambda: state, speech=N(play_wav=AsyncMock()))
        worker = N(cfg={'navigation_session': context, 'speech': {'engine': 'be-tts'}},
                   adapter=adapter, active=step, connection_ready=True, navigation_audio={'LEFT': b'cached'},
                   shutting_down=False, session_fault=False,
                   store=N(guidance_start=Mock(return_value=True), guidance_result=Mock()))
        return NavigationGuidance(worker, step), state, path

    def test_uses_live_motion_direction_and_suppresses_late_centimetres(self):
        guidance, state, _ = self.guidance()
        self.assertEqual('LEFT', guidance.current_cue().direction)
        state['map_pose']['value'].update(x_m=1., y_m=.55, yaw_rad=1.57)
        self.assertEqual('NEAR', guidance.current_cue().direction)
        state['map_pose']['value'].update(y_m=.95)
        self.assertIsNone(guidance.current_cue())

    def test_only_started_phrase_can_finish_during_a_bounded_pose_gap(self):
        guidance, state, path = self.guidance()
        with patch('mediassist.navigation_guidance.time.monotonic', return_value=10.):
            cue = guidance.current_cue()
            self.assertTrue(guidance.can_play(cue, path))
        state['localization'].update(ready=False, reason='TF_NOT_READY')
        state['map_pose']['fresh'] = False
        with patch('mediassist.navigation_guidance.time.monotonic', return_value=11.):
            self.assertFalse(guidance.can_play(cue, path))
            guidance.playback_started_at = 11.
        with patch('mediassist.navigation_guidance.time.monotonic', return_value=13.):
            self.assertTrue(guidance.can_play(cue, path))
        with patch('mediassist.navigation_guidance.time.monotonic', return_value=16.1):
            self.assertFalse(guidance.can_play(cue, path))

    def test_stop_different_replan_pose_jump_and_media_revoke_voice(self):
        for reason in ('stop', 'different_replan', 'finish', 'pose_jump', 'media'):
            guidance, state, path = self.guidance()
            with patch('mediassist.navigation_guidance.time.monotonic', return_value=10.):
                cue = guidance.current_cue()
            guidance.playback_started_at = 10.
            if reason == 'stop':
                guidance.worker.shutting_down = True
            if reason == 'different_replan':
                guidance.worker.adapter.navigation_guidance_path = {
                    **path, 'points': [{'xM': x, 'yM': y} for x, y in ((0., 0.), (1., 0.), (1., -1.))]}
            if reason == 'finish':
                guidance.worker.adapter.navigation_guidance_tracking = False
            if reason == 'pose_jump':
                state['localization'].update(ready=False, reason='POSE_JUMP')
                state['map_pose']['fresh'] = False
            with patch('mediassist.navigation_guidance.time.monotonic', return_value=11.), \
                 patch('mediassist.media_runtime.audio_reserved', return_value=reason == 'media'):
                with self.subTest(reason=reason):
                    self.assertFalse(guidance.can_play(cue, path))

    async def test_replan_does_not_repeat_audio_already_started_near_same_corner(self):
        guidance, _, path = self.guidance()
        played = asyncio.Event()

        async def play(*args, **kwargs):
            self.assertTrue(kwargs['playback_guard']())
            kwargs['on_playback_start']()
            played.set()

        guidance.worker.adapter.speech.play_wav.side_effect = play
        task = asyncio.create_task(guidance.run())
        try:
            await asyncio.wait_for(played.wait(), 1.)
            guidance.worker.adapter.navigation_guidance_path = {
                **path, 'points': [{'xM': x, 'yM': y} for x, y in ((0., 0.), (1.15, 0.), (1.15, 1.))]}
            await asyncio.sleep(.25)
            guidance.worker.adapter.speech.play_wav.assert_awaited_once()
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)




class ManagedVoiceTests(unittest.IsolatedAsyncioTestCase):
    def test_cached_state_rejects_stale_foreign_or_reused_worker_identity(self):
        import time,stat
        from mediassist.voice_runtime import StateReader
        reader=StateReader.__new__(StateReader);reader.session='current'
        value=dict(connected=True,status='IDLE',sessionId='current',pid=123,startTicks='tick',expiresMonotonic=time.monotonic()+7)
        reader.path=N(lstat=lambda:N(st_mode=stat.S_IFREG,st_uid=42,st_size=250),read_text=lambda:json.dumps(value))
        proc='123 (worker) '+' '.join(['S']+['0']*18+['tick'])
        with patch('mediassist.voice_runtime.os.getuid',return_value=42,create=True),patch.object(Path,'read_text',return_value=proc):
            self.assertEqual(reader.read()['status'],'IDLE')
            value['sessionId']='old';self.assertEqual(reader.read(),{})
            value['sessionId']='current';value['expiresMonotonic']=0;self.assertEqual(reader.read(),{})
            value['expiresMonotonic']=time.monotonic()+7;value['startTicks']='reused';self.assertEqual(reader.read(),{})

    def runtime(self):
        from mediassist.voice_runtime import VoiceRuntime
        state={'connected':True,'status':'IDLE'}
        runtime=VoiceRuntime({'voice_chat':{}},reader=N(read=lambda:dict(state)))
        return runtime,state

    async def test_admin_step_cancels_capture_and_releases_audio(self):
        runtime,state=self.runtime();started=asyncio.Event();released=asyncio.Event()
        async def capture():
            started.set()
            try:await asyncio.Future()
            finally:released.set()
        task=asyncio.create_task(runtime.guarded(capture()))
        await started.wait();state['status']='BUSY'
        self.assertEqual(await asyncio.wait_for(task,1),(False,None))
        self.assertTrue(released.is_set())

    async def test_confirmed_handoff_can_complete_but_disconnect_still_preempts(self):
        runtime,state=self.runtime();runtime.submitting();state['status']='BUSY'
        self.assertEqual(await runtime.guarded(asyncio.sleep(0,result='receipt')),(True,'receipt'))
        state['connected']=False
        self.assertEqual(await runtime.guarded(asyncio.sleep(10)),(False,None))

    async def test_idle_wait_never_queries_be_and_rearms_after_a_conversation(self):
        await self.check_rearm(robot_available=True)

    async def test_server_side_workflow_blocks_chat_even_in_a_gap_between_pi_steps(self):
        await self.check_rearm(robot_available=False)

    async def check_rearm(self,robot_available):
        runtime,_=self.runtime();runtime.options['idle_settle_sec']=0
        woke=asyncio.Event();rearmed=asyncio.Event();count=0
        async def wake(worker,ai):
            nonlocal count
            count+=1
            if count==1:await woke.wait()
            else:rearmed.set();await asyncio.Future()
        catalog=Mock(return_value=dict(robotAvailable=robot_available,
            navigationSession={'sessionId':'same'},endpoints=[]))
        runtime.worker['navigation_session']={'session_id':'same'}
        with tempfile.TemporaryDirectory() as folder:
            config=Path(folder)/'ai.json';config.write_text('{}');runtime.options['config_file']=str(config)
            with patch('mediassist.voice_chat.controller_lock',return_value=nullcontext()),\
                 patch('mediassist.voice_chat.AiClient',return_value=N(config={})),\
                 patch('mediassist.voice_chat.wait_for_wake',side_effect=wake),\
                 patch('mediassist.voice_chat.conversation',new=AsyncMock(return_value=False)) as chat,\
                 patch('mediassist.voice_client.VoiceClient'),\
                 patch('mediassist.voice_actions.VoiceActions',return_value=N(catalog=catalog)):
                task=asyncio.create_task(runtime.run())
                try:
                    await asyncio.sleep(.05);catalog.assert_not_called();woke.set()
                    await asyncio.wait_for(rearmed.wait(),3)
                    self.assertEqual(chat.await_count,int(robot_available))
                    self.assertEqual(catalog.call_count,1)
                finally:task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_goodbye_ends_once_without_calling_rag_or_dispatching(self):
        from mediassist.voice_chat import conversation,INTRO,GOODBYE,goodbye
        self.assertTrue(goodbye('Không cần nữa.'));self.assertTrue(goodbye('Goodbye!'))
        self.assertFalse(goodbye('Từ tạm biệt nghĩa là gì?'))
        ai=N(config={'stt_language':'auto'},marker=Path('unused'),request=Mock(return_value={'state':'READY'}),ask=Mock())
        be=N(transcribe=Mock(return_value='Không cần nữa.'));history=Mock();history.start.return_value='session'
        with patch('mediassist.voice_chat.AiClient',return_value=ai),\
             patch('mediassist.voice_chat.VoiceClient',return_value=be),\
             patch('mediassist.voice_chat.SpeechPlayer'),\
             patch('mediassist.voice_chat.VoiceActions') as actions,\
             patch('mediassist.voice_chat.History',return_value=history),\
             patch('mediassist.voice_chat.listen',new=AsyncMock(return_value=b'pcm')),\
             patch('mediassist.voice_chat.say',new=AsyncMock()) as say:
            self.assertFalse(await conversation({'speech':{}},'unused',initial_catalog={'endpoints':[]},log_transcripts=False))
        self.assertEqual([c.args[1] for c in say.await_args_list],[INTRO,GOODBYE])
        history.close.assert_called_once_with('COMPLETED');ai.ask.assert_not_called()
        actions.return_value.consume.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
