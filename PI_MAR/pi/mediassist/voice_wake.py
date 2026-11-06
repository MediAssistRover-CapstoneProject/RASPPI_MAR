"""Quiet VAD capture for a Vietnamese wake phrase; bounded RAM, no cloud STT or WAV."""
import asyncio
from collections import deque
import math
import shutil
from .contracts import StepError
from .speech import audio_device

SAMPLE_RATE = 16000
FRAME_SECONDS = .03
FRAME_BYTES = int(SAMPLE_RATE * FRAME_SECONDS) * 2


async def capture_phrase(player,settings):
    from .voice_chat import rms,stop_process
    threshold = settings.get('vad_threshold', .008)
    silence = settings.get('silence_sec', .45)
    seconds = settings.get('max_seconds', 4)
    if (type(threshold) not in (int, float) or not .002 <= threshold <= .1
            or type(silence) not in (int, float) or not .3 <= silence <= 1
            or type(seconds) is not int or not 2 <= seconds <= 4):
        raise StepError('VOICE_WAKE_CONFIG_INVALID')
    recorder = shutil.which('arecord')
    if not recorder:
        raise StepError('VOICE_ALSA_TOOLS_MISSING')

    lead = deque(maxlen=10)
    frames = []
    voiced = quiet = 0
    overlong = False
    silence_frames = math.ceil(silence / FRAME_SECONDS)
    frame_limit = int(seconds / FRAME_SECONDS)
    async with player.lock:
        proc = await asyncio.create_subprocess_exec(
            recorder, '-q', '-D', audio_device(player.cfg), '-t', 'raw',
            '-f', 'S16_LE', '-r', str(SAMPLE_RATE), '-c', '1',
            stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        try:
            while True:
                try:
                    frame = await asyncio.wait_for(proc.stdout.readexactly(FRAME_BYTES), 1.5)
                except (asyncio.IncompleteReadError, asyncio.TimeoutError):
                    raise StepError('VOICE_MIC_STREAM_FAILED') from None

                active = rms(frame) >= threshold
                quiet = 0 if active else quiet + 1
                if overlong:
                    if quiet >= silence_frames:
                        overlong = False
                    continue
                if not frames:
                    lead.append(frame)
                    if active:
                        frames = list(lead)
                        lead.clear()
                        voiced = 1
                    continue

                frames.append(frame)
                voiced += int(active)
                if quiet >= silence_frames:
                    if voiced >= 8:
                        return b''.join(frames)
                    frames.clear()
                elif len(frames) >= frame_limit:
                    # Reject a long utterance rather than recognize a clipped prefix.
                    frames.clear()
                    overlong = True
        finally:
            await stop_process(proc)


async def wait_phrase(player,ai,settings):
    phrase = settings.get('phrase', 'Chào cậu')
    if not isinstance(phrase, str) or not 1 <= len(phrase.strip()) <= 80:
        raise StepError('VOICE_WAKE_CONFIG_INVALID')
    print(f'VOICE_WAKE_LISTENING: {phrase}', flush=True)
    while True:
        pcm = await capture_phrase(player, settings)
        try:
            if await asyncio.to_thread(ai.detect_wake, pcm):
                return
        finally:
            del pcm
