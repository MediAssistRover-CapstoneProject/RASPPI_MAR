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
        runtime,state=self.runtime();rejected=runtime.submitting();state['status']='BUSY'
        self.assertEqual(await runtime.guarded(asyncio.sleep(0,result='receipt')),(True,'receipt'))
        rejected()
        self.assertEqual(await runtime.guarded(asyncio.sleep(10)),(False,None))
        runtime.submitting()
        self.assertEqual(await runtime.guarded(asyncio.sleep(0,result='accepted')),(True,'accepted'))
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

    async def test_declined_staff_navigation_keeps_conversation_open_for_next_question(self):
        from mediassist.voice_chat import conversation,INTRO,GOODBYE
        ai=N(config={'stt_language':'auto'},marker=Path('unused'),
             request=Mock(return_value={'state':'READY'}),
             ask=Mock(return_value=dict(speech_text='Tôi đã nhận câu hỏi mới.',action=None,end_session=False)))
        be=N(staff_question=Mock(return_value=dict(recognized=True,code='STAFF_NAVIGATION_READY',
             answer='Nhân viên Trung đang trực tại phòng A.',
             endpointId='44444444-4444-4444-8444-444444444444',destinationLabel='phòng A')))
        history=Mock();history.start.return_value='session'
        actions=Mock();actions.pending=None
        def prepare(action,interaction):
            actions.pending=dict(staff_navigation=True)
            return action['confirmation_prompt']
        async def consume(text,speak):
            if actions.pending and text=='không':
                actions.pending=None
                return 'Được ạ. Bạn có cần tôi giúp gì thêm không?',None
            return None
        actions.prepare.side_effect=prepare;actions.consume.side_effect=consume
        with patch('mediassist.voice_chat.VoiceActions',return_value=actions),\
             patch('mediassist.voice_chat.History',return_value=history),\
             patch('mediassist.voice_chat.SpeechPlayer'),\
             patch('mediassist.voice_chat.listen',new=AsyncMock(return_value=b'pcm')),\
             patch('mediassist.voice_chat.transcribe',new=AsyncMock(side_effect=[
                 'Dẫn tôi đến phòng của nhân viên Trung','không','Xe lăn ở đâu?','không cần nữa'])),\
             patch('mediassist.voice_chat.say',new=AsyncMock()) as say:
            self.assertFalse(await conversation({'speech':{}},'unused',max_turns=4,
                initial_catalog={'endpoints':[]},log_transcripts=False,ai_client=ai,be_client=be))
        spoken=[call.args[1] for call in say.await_args_list]
        self.assertEqual(spoken[0],INTRO)
        self.assertIn('Bạn có muốn tôi dẫn',spoken[1])
        self.assertEqual(spoken[2],'Được ạ. Bạn có cần tôi giúp gì thêm không?')
        self.assertEqual(spoken[3],'Tôi đã nhận câu hỏi mới.')
        self.assertEqual(spoken[4],GOODBYE)
        ai.ask.assert_called_once()

class VoiceNavigationConfirmationTests(unittest.IsolatedAsyncioTestCase):
    def catalog(self,state='READY',revision=1):
        return dict(robotAvailable=True,workflowPrepared=True,workflowOwnsInvite=False,
            navigationSession=dict(sessionId='33333333-3333-4333-8333-333333333333',
                robotId='11111111-1111-4111-8111-111111111111',
                mapId='22222222-2222-4222-8222-222222222222',mapAssetHash='a'*64,
                state=state,expiresAt='2099-01-01T00:00:00Z',
                initialization=dict(mode='SEED',revision=revision,pose=dict(x_m=0.,y_m=0.,yaw_rad=0.))),
            endpoints=[dict(endpointId='44444444-4444-4444-8444-444444444444',
                mapId='22222222-2222-4222-8222-222222222222',endpointCode='A',endpointName='A')])

    def actions(self,folder):
        from mediassist.voice_actions import VoiceActions
        be=Mock(cfg={'robot_id':self.catalog()['navigationSession']['robotId']})
        be.value.side_effect=json.loads
        actions=VoiceActions(be,{'actions_enabled':True,'navigation_ready_timeout_sec':.5},Path(folder)/'voice.json')
        actions.catalog=Mock(return_value=self.catalog())
        return actions,be

    async def test_staff_destination_decline_or_current_duty_confirmation(self):
        with tempfile.TemporaryDirectory() as folder:
            actions,be=self.actions(folder)
            endpoint=self.catalog()['endpoints'][0]['endpointId']
            action=dict(kind='NAVIGATE',endpoint_id=endpoint,destination_label='Phòng Trung',
                        staff_navigation=True,staff_question='Dẫn tôi đến phòng của nhân viên Trung',
                        confirmation_prompt='Nhân viên Trung đang trực tại Phòng Trung. Bạn có muốn tôi dẫn đến đó không?')
            self.assertIn('Bạn có muốn',actions.prepare(action,'interaction'))
            self.assertEqual((await actions.consume('không',AsyncMock()))[0],
                             'Được ạ. Bạn có cần tôi giúp gì thêm không?')
            self.assertIsNone(actions.pending)
            be.request.assert_not_called()

            actions.prepare(action,'interaction')
            be.staff_question.return_value=dict(code='STAFF_OFF_DUTY',endpointId=None)
            reply,request=await actions.consume('có',AsyncMock())
            self.assertIsNone(request);self.assertIn('đã thay đổi',reply)
            be.request.assert_not_called()

            actions.prepare(action,'interaction')
            be.staff_question.return_value=dict(code='STAFF_NAVIGATION_READY',endpointId=endpoint)
            be.request.return_value=(200,{},json.dumps({'serviceRequestId':'55555555-5555-4555-8555-555555555555'}))
            reply,request=await actions.consume('có',AsyncMock())
            self.assertEqual(request,'55555555-5555-4555-8555-555555555555')
            self.assertEqual(be.request.call_count,1)

    async def test_same_initialized_session_survives_readiness_gap_and_post_race_once(self):
        with tempfile.TemporaryDirectory() as folder:
            actions,be=self.actions(folder);actions.ready_timeout=2.
            actions.catalog.return_value=self.catalog('LOCALIZING')
            self.assertIn('phải không',actions.prepare({'kind':'NAVIGATE','destination_label':'A'},'interaction'))
            pending=actions.pending;key=pending['key']
            actions.catalog.side_effect=[self.catalog('WAITING_FOR_PI'),self.catalog(),self.catalog(),self.catalog()]
            receipt=json.dumps({'serviceRequestId':'55555555-5555-4555-8555-555555555555'})
            be.request.side_effect=[StepError('NAVIGATION_SESSION_NOT_LOCALIZED'),(200,{},receipt)]
            rejected,accepted=Mock(),Mock()
            actions.before_dispatch=Mock(side_effect=[rejected,accepted])
            invite=AsyncMock()
            with patch('mediassist.voice_actions.asyncio.sleep',new=AsyncMock()):
                reply,request=await actions.consume('xác nhận',invite)
            self.assertIsNotNone(request);self.assertIsNone(actions.pending)
            self.assertFalse(actions.marker.exists());invite.assert_awaited_once()
            self.assertEqual(be.request.call_count,2)
            self.assertEqual([call.kwargs['idempotency_key'] for call in be.request.call_args_list],[key,key])
            self.assertEqual([call.kwargs['timeout'] for call in be.request.call_args_list],[15.,15.])
            self.assertEqual(be.request.call_args_list[0].args[1],be.request.call_args_list[1].args[1])
            self.assertEqual(pending['context']['revision'],1)
            rejected.assert_called_once();accepted.assert_not_called()

    async def test_wait_timeout_preserves_consent_but_identity_change_or_closed_session_clears_it(self):
        with tempfile.TemporaryDirectory() as folder:
            actions,be=self.actions(folder)
            actions.prepare({'kind':'NAVIGATE','destination_label':'A'},'interaction')
            pending=actions.pending;actions.catalog.return_value=self.catalog('LOCALIZING')
            invite=AsyncMock()
            reply,request=await actions.consume('xác nhận',invite)
            self.assertIsNone(request);self.assertIn('phiên hiện tại',reply)
            self.assertIs(actions.pending,pending);be.request.assert_not_called();invite.assert_not_awaited()
            self.assertLessEqual(actions.catalog.call_args.kwargs['timeout'],.5)
            for change in ('revision','mapAssetHash','closed'):
                actions.pending=pending
                changed=self.catalog(revision=2) if change=='revision' else self.catalog()
                if change=='mapAssetHash':changed['navigationSession']['mapAssetHash']='b'*64
                actions.catalog.side_effect=StepError('NAVIGATION_SESSION_CLOSED_OR_EXPIRED') if change=='closed' else None
                actions.catalog.return_value=changed
                _,request=await actions.consume('xác nhận',invite)
                self.assertIsNone(request);self.assertIsNone(actions.pending)
            be.request.assert_not_called()

    async def test_uninitialized_session_and_ambiguous_post_never_create_a_second_request(self):
        with tempfile.TemporaryDirectory() as folder:
            actions,be=self.actions(folder);actions.catalog.return_value=self.catalog('LOCALIZING',0)
            self.assertIn('vị trí khởi tạo',actions.prepare({'kind':'NAVIGATE','destination_label':'A'},'interaction'))
            self.assertIsNone(actions.pending)
            actions.catalog.return_value=self.catalog()
            actions.prepare({'kind':'NAVIGATE','destination_label':'A'},'interaction')
            be.request.side_effect=TimeoutError('ambiguous POST')
            with self.assertRaises(TimeoutError):await actions.consume('xác nhận',AsyncMock())
            self.assertTrue(actions.marker.exists());self.assertIsNone(actions.pending)
            self.assertEqual(be.request.call_count,1)
            self.assertIn('chờ kiểm tra',actions.prepare({'kind':'NAVIGATE','destination_label':'A'},'interaction'))
            self.assertEqual(be.request.call_count,1)

    def test_voice_http_client_preserves_only_recognized_navigation_reasons(self):
        import io,urllib.error
        from mediassist.voice_client import VoiceClient
        client=VoiceClient.__new__(VoiceClient);client.auth={};client.cfg={'robot_id':'robot'}
        client.origin='https://be.example';client.opener=Mock()
        for message,expected in (
                ('NAVIGATION_SESSION_NOT_LOCALIZED: wait for a verified pose','NAVIGATION_SESSION_NOT_LOCALIZED'),
                ('NAVIGATION_SESSION_CLOSED_OR_EXPIRED','NAVIGATION_SESSION_CLOSED_OR_EXPIRED'),
                ('unrecognized private detail','BUSINESS_RULE_VIOLATION')):
            payload=json.dumps({'error':'BUSINESS_RULE_VIOLATION','message':message}).encode()
            client.opener.open.side_effect=urllib.error.HTTPError(client.origin,422,'',{},io.BytesIO(payload))
            with patch('mediassist.voice_client.upload_token',return_value='token'):
                with self.assertRaises(StepError) as caught:client.request('/api/v1/pi/voice/navigation-requests')
            self.assertEqual(caught.exception.code,expected)
            self.assertNotIn(message,str(caught.exception))


class VoicePipelineLatencyTests(unittest.IsolatedAsyncioTestCase):
    def test_noise_floor_ignores_speech_and_loud_non_speech_cannot_wake(self):
        import array,math
        from mediassist.voice_wake import WakeSegmenter,PcmConditioner,FRAME_BYTES
        detector=N(speaking=False)
        detector.is_speech=lambda frame,rate:detector.speaking
        segmenter=WakeSegmenter({'noise_filter_enabled':False},detector)
        def tone(amplitude):
            return array.array('h',[round(amplitude*math.sin(2*math.pi*1000*i/16000)) for i in range(480)]).tobytes()
        for _ in range(50):self.assertIsNone(segmenter.push(tone(700)))
        floor=segmenter.noise_floor
        for _ in range(3):self.assertIsNone(segmenter.push(tone(12000)))
        detector.speaking=True
        for _ in range(4):self.assertIsNone(segmenter.push(tone(1000)))
        self.assertFalse(segmenter.frames)
        for _ in range(8):self.assertIsNone(segmenter.push(tone(5000)))
        for _ in range(6):self.assertIsNone(segmenter.push(tone(1500)))
        self.assertAlmostEqual(segmenter.noise_floor,floor,delta=.001)
        detector.speaking=False
        result=None
        for _ in range(22):result=segmenter.push(bytes(FRAME_BYTES)) or result
        self.assertIsInstance(result,bytes)
        conditioner=PcmConditioner()
        dc=array.array('h',[2000]*480).tobytes()
        for _ in range(5):clean=conditioner.process(dc)
        self.assertLess(max(abs(value) for value in array.array('h',clean)),5)

    async def test_wake_keeps_one_microphone_open_during_inference_and_reaps_it(self):
        import threading
        from mediassist.voice_wake import wait_phrase,WakeSegmenter
        first_started=threading.Event();second_captured=threading.Event();calls=[]
        class Segmenter:
            valid_number=staticmethod(WakeSegmenter.valid_number)
            def __init__(self,_):self.count=0
            def push(self,frame):
                self.count+=1
                if self.count==2:second_captured.set()
                return bytes(9600)
            def reset(self):pass
        class Source:
            def __init__(self):self.reads=0
            async def readexactly(self,size):
                self.reads+=1
                if self.reads==2:
                    self.assert_ready=await asyncio.to_thread(first_started.wait,1)
                    if not self.assert_ready:raise AssertionError('capture stopped during inference')
                elif self.reads>2:await asyncio.Future()
                return bytes(size)
        def detect(pcm,request_id):
            calls.append(request_id)
            if len(calls)==1:
                first_started.set()
                if not second_captured.wait(1):raise AssertionError('second candidate was not captured')
                return False
            return True
        proc=N(stdout=Source());player=N(cfg={'device':'test'},lock=asyncio.Lock())
        ai=N(detect_wake=detect,cancel_pending=Mock())
        with patch('mediassist.voice_wake.WakeSegmenter',Segmenter),\
             patch('mediassist.voice_wake.shutil.which',return_value='/usr/bin/arecord'),\
             patch('mediassist.voice_wake.asyncio.create_subprocess_exec',new=AsyncMock(return_value=proc)) as spawned,\
             patch('mediassist.voice_chat.stop_process',new=AsyncMock()) as stopped:
            await asyncio.wait_for(wait_phrase(player,ai,{}),2)
        self.assertEqual(len(calls),2);self.assertNotEqual(*calls)
        spawned.assert_awaited_once();stopped.assert_awaited_once_with(proc)
        ai.cancel_pending.assert_not_called()
        self.assertFalse(player.lock.locked())

    async def test_cancel_before_stream_headers_disposes_late_response_without_playback(self):
        import threading
        from mediassist.voice_chat import play_stream
        started=threading.Event();release=threading.Event();disposed=asyncio.Event()
        loop=asyncio.get_running_loop()
        response=N(abort=Mock(side_effect=lambda:loop.call_soon_threadsafe(disposed.set)))
        def opening(_):
            started.set();release.wait(2)
            return response,'request'
        ai=N(open_speech_stream=opening,cancel_pending=Mock())
        player=N(cfg={'enabled':True,'device':'test'})
        with patch('mediassist.voice_chat.shutil.which',return_value='/usr/bin/aplay'),\
             patch('mediassist.voice_chat.asyncio.create_subprocess_exec',new=AsyncMock()) as spawned:
            task=asyncio.create_task(play_stream(player,ai,'Câu hoàn chỉnh.'))
            self.assertTrue(await asyncio.to_thread(started.wait,1))
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):await task
            release.set()
            await asyncio.wait_for(disposed.wait(),1)
        response.abort.assert_called_once();spawned.assert_not_awaited()
        ai.cancel_pending.assert_called_once()

    async def test_partly_played_stream_is_not_repeated_through_wav_fallback(self):
        from mediassist.voice_chat import say
        failure=StepError('VOICE_TTS_STREAM_INTERRUPTED');failure.playback_started=True
        ai=N(supports_stream=True,cached_speech=Mock(return_value=None),synthesize=Mock())
        player=N(cfg={},speak=AsyncMock())
        with patch('mediassist.voice_chat.play_stream',new=AsyncMock(side_effect=failure)):
            with self.assertRaises(StepError):await say(player,'Câu hoàn chỉnh.',False,ai)
        ai.synthesize.assert_not_called();player.speak.assert_not_awaited()

    def test_https_reuses_only_drained_responses_and_never_replays_a_post(self):
        import io,urllib.request,http.client
        from email.message import Message
        from mediassist.voice_client import VoiceHTTPS
        class Response:
            status=200;will_close=False
            def __init__(self):self.body=io.BytesIO(b'abc');self.headers=Message()
            def read(self,count=None):return self.body.read(count)
            def isclosed(self):return self.body.tell()==3
            def close(self):pass
        connection=Mock(sock=None)
        connection.getresponse.side_effect=lambda:Response()
        pool=VoiceHTTPS('https://example.test',object())
        with patch('mediassist.voice_client.http.client.HTTPSConnection',return_value=connection) as create:
            for _ in range(2):
                with pool.open(urllib.request.Request('https://example.test/a')) as response:
                    self.assertEqual(response.read(),b'abc')
            create.assert_called_once()
            with pool.open(urllib.request.Request('https://example.test/a')) as response:response.read(1)
            self.assertEqual(pool.idle,[])
            connection.getresponse.side_effect=http.client.RemoteDisconnected('closed')
            before=connection.request.call_count
            with self.assertRaises(http.client.RemoteDisconnected):
                pool.open(urllib.request.Request('https://example.test/job',data=b'{}'))
            self.assertEqual(connection.request.call_count-before,1)
            self.assertEqual(pool.leased,0)
        pool.close()


if __name__=='__main__':unittest.main(verbosity=2)
