import asyncio
import hashlib
import io
import struct
import tempfile
import unittest
from pathlib import Path
import sys
import urllib.error
import wave
from unittest.mock import AsyncMock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.cloud_speech import download, fetch_once, volume_wav, validate_cloud_config
from mediassist.speech import SpeechPlayer
from mediassist.contracts import StepError

def wav():
    out=io.BytesIO()
    with wave.open(out,'wb') as w:
        w.setnchannels(1);w.setsampwidth(2);w.setframerate(16000);w.writeframes(b'\x10\x00'*100)
    return out.getvalue()

class CloudSpeechTests(unittest.IsolatedAsyncioTestCase):
    async def test_pending_does_not_play_until_audio_ready(self):
        with patch('mediassist.cloud_speech.fetch_once',side_effect=[None,wav()]),patch('mediassist.cloud_speech.asyncio.sleep',AsyncMock()):
            self.assertEqual(await download({'wait_timeout_sec':5},{'text':'Xin chào'}),wav())
    async def test_cancellation_during_poll_does_not_play_audio(self):
        started=asyncio.Event()
        async def pause(_):started.set();await asyncio.Future()
        with patch('mediassist.cloud_speech.fetch_once',return_value=None),patch('mediassist.cloud_speech.asyncio.sleep',pause):
            task=asyncio.create_task(download({'wait_timeout_sec':45},{'text':'Xin chào'}))
            await started.wait();task.cancel()
            with self.assertRaises(asyncio.CancelledError):await task
    async def test_cloud_uses_only_aplay_and_preserves_result_contract(self):
        player=SpeechPlayer({'enabled':True,'engine':'be-tts','device':'plughw:CARD=Device,DEV=0'})
        player.process=AsyncMock()
        with patch('mediassist.cloud_speech.download',AsyncMock(return_value=wav())),patch('mediassist.speech.shutil.which',side_effect=lambda p:'/usr/bin/aplay' if p=='aplay' else None):
            result=await player.speak({'prompt_key':'TEST','action':'SPEAK','data':{'text':'Xin chào','language_code':'vi'}})
        self.assertEqual(result['values']['engine'],'be-tts')
        self.assertTrue(result['values']['playback_completed'])
        self.assertEqual(player.process.await_count,1)
        self.assertFalse(Path(player.process.call_args.args[0][-1]).exists())
    def test_volume_zero_is_silence(self):
        with wave.open(io.BytesIO(volume_wav(wav(),0)),'rb') as w:
            self.assertEqual(w.readframes(100),b'\0\0'*100)
    def test_streaming_header_uses_actual_pcm_length_at_volume_70(self):
        raw=bytearray(wav())
        struct.pack_into('<I',raw,4,0xffffffff)
        struct.pack_into('<I',raw,40,0xffffffff)
        result=volume_wav(bytes(raw),70)
        self.assertEqual(struct.unpack_from('<I',result,4)[0],len(result)-8)
        with wave.open(io.BytesIO(result),'rb') as w:
            self.assertEqual(w.getnframes(),100)
            self.assertEqual(w.readframes(100),b'\x0b\x00'*100)
    def test_full_volume_also_normalizes_streaming_header(self):
        raw=bytearray(wav())
        struct.pack_into('<I',raw,4,0xffffffff)
        struct.pack_into('<I',raw,40,0xffffffff)
        result=volume_wav(bytes(raw),100)
        with wave.open(io.BytesIO(result),'rb') as w:
            self.assertEqual(w.getnframes(),100)
            self.assertEqual(w.readframes(100),b'\x10\x00'*100)
    def test_partial_sample_is_rejected(self):
        raw=bytearray(wav()[:-1])
        struct.pack_into('<I',raw,4,len(raw)-8)
        struct.pack_into('<I',raw,40,len(raw)-44)
        with self.assertRaises(StepError) as caught:volume_wav(bytes(raw),70)
        self.assertEqual(caught.exception.code,'TTS_WAV_FORMAT_UNSUPPORTED')
    def test_http_and_missing_token_are_rejected(self):
        with self.assertRaises(StepError):validate_cloud_config({'synthesize_url':'http://be/api/v1/pi/speech/synthesize'})
    def test_https_response_hash_and_auth_errors(self):
        class Response:
            status=200
            headers={'X-Audio-SHA256':hashlib.sha256(wav()).hexdigest()}
            def read(self,_):return wav()
            def __enter__(self):return self
            def __exit__(self,*_):pass
        with tempfile.TemporaryDirectory() as d:
            token=Path(d)/'token';token.write_text('test-device-token')
            cfg={'synthesize_url':'https://be/api/v1/pi/speech/synthesize','token_file':str(token)}
            opener=unittest.mock.Mock();opener.open.return_value=Response()
            with patch('mediassist.cloud_speech.urllib.request.build_opener',return_value=opener):
                self.assertEqual(fetch_once(cfg,{'text':'Xin chào'}),wav())
                Response.headers={'X-Audio-SHA256':'wrong'}
                with self.assertRaises(StepError) as e:fetch_once(cfg,{'text':'Xin chào'})
                self.assertEqual(e.exception.code,'TTS_AUDIO_HASH_MISMATCH')
                opener.open.side_effect=urllib.error.HTTPError('https://be',401,'Unauthorized',{},None)
                with self.assertRaises(StepError) as e:fetch_once(cfg,{'text':'Xin chào'})
                self.assertEqual(e.exception.code,'TTS_DEVICE_AUTH_FAILED')

if __name__=='__main__':unittest.main(verbosity=2)
