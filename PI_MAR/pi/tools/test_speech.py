"""Contract and process lifecycle tests; no speaker or robot is used."""
import asyncio
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import AsyncMock, patch
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

if __name__=='__main__':unittest.main(verbosity=2)
