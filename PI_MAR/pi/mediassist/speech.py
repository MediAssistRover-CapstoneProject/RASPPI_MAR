"""Offline Vietnamese/English speech through Linux ALSA (WonderEcho Pro USB audio)."""
import asyncio
import logging
import re
import shutil
import tempfile
from pathlib import Path
from .contracts import StepError
from .store import utcnow


def audio_device(config, cards_file=Path('/proc/asound/cards')):
    configured = config.get('device', 'auto')
    if configured != 'auto':
        return configured
    try:
        cards = cards_file.read_text()
    except OSError:
        raise StepError('AUDIO_DEVICE_NOT_VISIBLE') from None
    usb = re.findall(r'^\s*\d+\s+\[([^\]]+)\]\s*:\s*USB-Audio\b', cards, re.MULTILINE)
    if len(usb) != 1:
        raise StepError('AUDIO_USB_DEVICE_AMBIGUOUS' if usb else 'AUDIO_USB_DEVICE_MISSING')
    return 'plughw:CARD=' + usb[0].strip() + ',DEV=0'


class SpeechPlayer:
    def __init__(self, config):
        self.cfg = config
        self.lock = asyncio.Lock()

    async def process(self, argv, content=None):
        # Keep bounded diagnostic text; never include the supplied speech text in argv/logs.
        with tempfile.TemporaryFile() as diagnostic:
            process = await asyncio.create_subprocess_exec(*argv,
                stdin=asyncio.subprocess.PIPE if content is not None else asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.DEVNULL, stderr=diagnostic)
            try:
                await process.communicate(content)
                if process.returncode != 0:
                    diagnostic.seek(0, 2)
                    diagnostic.seek(max(0, diagnostic.tell()-2048))
                    message=diagnostic.read(2048).decode('utf-8',errors='replace').strip()
                    message=re.sub(r'[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]', '?', message)
                    program=Path(argv[0]).name
                    code={'espeak-ng':'AUDIO_SYNTHESIS_FAILED','aplay':'AUDIO_PLAYBACK_FAILED'}.get(program,'AUDIO_PROCESS_FAILED')
                    details={'code':code,'program':program,'exitCode':process.returncode,'message':message}
                    logging.error('%s: %s',code,details)
                    error=StepError(code,program+' exited '+str(process.returncode))
                    error.details=details
                    raise error
            finally:
                if process.returncode is None:
                    try:
                        process.terminate()
                    except ProcessLookupError:
                        pass
                    try:
                        await asyncio.wait_for(process.wait(), 1)
                    except asyncio.TimeoutError:
                        try:
                            process.kill()
                        except ProcessLookupError:
                            pass
                        await process.wait()

    async def speak(self, params):
        if not self.cfg.get('enabled'):
            raise StepError('SPEECH_NOT_ENABLED')
        engine = self.cfg.get('engine', 'espeak-ng')
        synth, player = shutil.which('espeak-ng'), shutil.which('aplay')
        if not player or (engine == 'espeak-ng' and not synth):
            raise StepError('SPEECH_DEPENDENCY_MISSING', 'Install espeak-ng and alsa-utils inside MentorPi')
        device = audio_device(self.cfg)
        data = params['data']
        async with self.lock:
            with tempfile.TemporaryDirectory(prefix='mar-speech-') as folder:
                wav = str(Path(folder) / 'speech.wav')
                if engine == 'be-tts':
                    from .cloud_speech import download, volume_wav
                    raw = await download(self.cfg, data)
                    raw = await asyncio.to_thread(volume_wav, raw, data.get('volume', 70))
                    Path(wav).write_bytes(raw)
                elif engine == 'espeak-ng':
                    await self.process([synth, '-v', data.get('language_code', 'vi'),
                        '-s', str(data.get('speed_wpm', 150)), '-a', str(data.get('volume', 70)),
                        '-w', wav, '--stdin'], data['text'].encode('utf-8'))
                else:
                    raise StepError('INVALID_SPEECH_ENGINE')
                if not Path(wav).is_file() or Path(wav).stat().st_size <= 44:
                    raise StepError('SPEECH_EMPTY_AUDIO')
                await self.process([player, '-q', '-D', device, wav])
        # Playback process completion is not a microphone verification of audibility.
        return {'completed': True, 'action': 'SPEAK', 'prompt_key': params['prompt_key'],
                'source': 'AUDIO_CLIENT', 'actor_account_id': None, 'completed_at': utcnow(),
                'values': {'playback_completed': True, 'audible_verified': False,
                           'engine': engine, 'device': device,
                           'language_code': data.get('language_code', 'vi')}}
