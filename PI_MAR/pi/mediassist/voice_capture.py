"""Explicit microphone tests; no background listener, movement or intent dispatch."""
import asyncio
import io
import logging
import math
from pathlib import Path
import shutil
import struct
import tempfile
import wave
from .contracts import StepError
from .speech import audio_device
from .store import utcnow
from .voice_client import VoiceClient,describe_audio


def recording_cue():
    """Short local cue; never sent to STT or mixed into the recording."""
    rate=16000;count=2400
    pcm=b''.join(struct.pack('<h',int(2400*min(1,i/160,(count-1-i)/160)*math.sin(2*math.pi*880*i/rate))) for i in range(count))
    out=io.BytesIO()
    with wave.open(out,'wb') as w:
        w.setnchannels(1);w.setsampwidth(2);w.setframerate(rate);w.writeframes(pcm)
    return out.getvalue()


async def record(player,seconds,folder):
    if not player.cfg.get('enabled'):
        raise StepError('SPEECH_NOT_ENABLED')
    recorder,output=shutil.which('arecord'),shutil.which('aplay')
    if not recorder or not output:
        raise StepError('VOICE_ALSA_TOOLS_MISSING')
    device=audio_device(player.cfg)
    cue=Path(folder)/'ready.wav';cue.write_bytes(recording_cue())
    await asyncio.wait_for(player.process([output,'-q','-D',device,str(cue)]),3)
    await asyncio.sleep(.15)
    path=Path(folder)/'capture.wav'
    logging.info('VOICE_RECORDING_STARTED duration_sec=%s',seconds)
    try:
        # Shared process helper terminates and reaps arecord on cancellation.
        await asyncio.wait_for(player.process([recorder,'-q','-D',device,'-t','wav','-f','S16_LE',
            '-r','16000','-c','1','-d',str(seconds),str(path)]),seconds+5)
    except asyncio.TimeoutError:
        raise StepError('VOICE_RECORDING_TIMEOUT') from None
    if path.stat().st_size>1024*1024:
        raise StepError('VOICE_RECORDING_TOO_LARGE')
    raw=path.read_bytes();info=describe_audio(raw)
    if abs(info['duration_sec']-seconds)>.1:
        raise StepError('VOICE_RECORDING_INCOMPLETE')
    captured=utcnow()
    logging.info('VOICE_RECORDING_FINISHED duration_sec=%s rms=%.5f peak=%.5f',info['duration_sec'],info['rms'],info['peak'])
    return raw,info,captured,device


async def loopback(player,params):
    # One audio lock covers recording and replay; no overlapping SPEAK/capture.
    async with player.lock:
        with tempfile.TemporaryDirectory(prefix='mar-loopback-') as folder:
            raw,info,captured,device=await record(player,params['duration_sec'],folder)
            from .cloud_speech import volume_wav
            path=Path(folder)/'replay.wav'
            path.write_bytes(await asyncio.to_thread(volume_wav,raw,params['volume']))
            await asyncio.wait_for(player.process([shutil.which('aplay'),'-q','-D',device,str(path)]),params['duration_sec']+5)
    return dict(sensing_type='AUDIO_LOOPBACK',captured_at=captured,evidence_ref=None,
        metrics=dict(duration_sec=info['duration_sec'],sample_rate_hz=16000,channels=1,bits_per_sample=16,
            volume=params['volume'],original_recording=True,playback_completed=True,audible_verified=False,
            rms=info['rms'],peak=info['peak']))


async def capture(worker,player,params):
    async with player.lock:
        with tempfile.TemporaryDirectory(prefix='mar-capture-') as folder:
            raw,info,captured,_=await record(player,params['duration_sec'],folder)
    # Bounded HTTPS runs away from MQTT/ROS callbacks. Cancellation cannot
    # restart recording or cause later playback from the abandoned request.
    try:
        result=await asyncio.wait_for(asyncio.to_thread(VoiceClient(worker).transcribe_result,raw,params['language_code']),55)
    except asyncio.TimeoutError:
        raise StepError('VOICE_STT_TIMEOUT') from None
    finally:
        del raw
    text=result['text']
    if not isinstance(text,str) or not text.strip() or len(text)>2000:
        raise StepError('VOICE_TRANSCRIPT_INVALID')
    return dict(sensing_type='VOICE_CAPTURE',captured_at=captured,evidence_ref=None,
        metrics=dict(text=text,playback_text=text[:500],language_code=params['language_code'],
            duration_sec=info['duration_sec'],sample_rate_hz=16000,channels=1,bits_per_sample=16,
            processing_ms=result['pi_request_ms']))
