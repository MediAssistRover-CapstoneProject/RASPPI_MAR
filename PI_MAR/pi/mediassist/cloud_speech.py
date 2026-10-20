"""Fetch BE-generated WAV over verified HTTPS; never sends a provider key to the robot."""
import asyncio
import hashlib
import io
import json
from pathlib import Path
import ssl
import struct
import time
import urllib.error
import urllib.request
import wave
from urllib.parse import urlparse
from .contracts import StepError

MAX_AUDIO_BYTES = 4 * 1024 * 1024


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


def validate_cloud_config(cfg):
    uri = urlparse(cfg.get('synthesize_url', ''))
    if (uri.scheme != 'https' or not uri.hostname or uri.username or uri.password
            or uri.query or uri.fragment or uri.path != '/api/v1/pi/speech/synthesize'):
        raise StepError('INVALID_TTS_URL')
    token = Path(cfg.get('token_file', ''))
    if not token.is_absolute() or not token.is_file():
        raise StepError('TTS_TOKEN_FILE_REQUIRED')
    if cfg.get('ca_file') and not Path(cfg['ca_file']).is_file():
        raise StepError('TTS_CA_FILE_NOT_FOUND')
    wait = cfg.get('wait_timeout_sec', 45)
    if type(wait) not in (int, float) or not 5 <= wait <= 180:
        raise StepError('INVALID_TTS_WAIT_TIMEOUT')


def fetch_once(cfg, data):
    validate_cloud_config(cfg)
    if data.get('language_code', 'vi') not in ('vi', 'en'):
        raise StepError('TTS_LANGUAGE_UNSUPPORTED')
    if len(data['text']) < 1:
        raise StepError('TTS_TEXT_INVALID')
    token = Path(cfg['token_file']).read_text(encoding='utf-8-sig').strip()
    if not token or '\n' in token or '\r' in token:
        raise StepError('TTS_TOKEN_INVALID')
    context = ssl.create_default_context(cafile=cfg.get('ca_file') or None)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect(),
        urllib.request.HTTPSHandler(context=context))
    body = json.dumps({'text': data['text'], 'languageCode': data.get('language_code', 'vi'),
                       'speedWpm': data.get('speed_wpm', 150)}, ensure_ascii=False).encode('utf-8')
    request = urllib.request.Request(cfg['synthesize_url'], data=body, method='POST', headers={
        'Content-Type': 'application/json; charset=utf-8', 'Authorization': 'Bearer ' + token})
    try:
        with opener.open(request, timeout=8) as response:
            if response.status == 202:
                return None
            raw = response.read(MAX_AUDIO_BYTES + 1)
            if (response.status != 200 or len(raw) <= 44 or len(raw) > MAX_AUDIO_BYTES
                    or raw[:4] != b'RIFF' or raw[8:12] != b'WAVE'):
                raise StepError('TTS_INVALID_WAV')
            checksum = response.headers.get('X-Audio-SHA256', '')
            if not checksum or hashlib.sha256(raw).hexdigest() != checksum:
                raise StepError('TTS_AUDIO_HASH_MISMATCH')
            return raw
    except urllib.error.HTTPError as error:
        if error.code in (401, 403):
            raise StepError('TTS_DEVICE_AUTH_FAILED', 'Renew the BE device token') from None
        raw = error.read(8192)
        try:
            code = json.loads(raw).get('code', '')
        except (ValueError, AttributeError):
            code = ''
        if isinstance(code, str) and code.startswith('TTS_') and code.replace('_', '').isalnum():
            raise StepError(code) from None
        raise StepError('TTS_BE_HTTP_ERROR', str(error.code)) from None
    except (OSError, urllib.error.URLError):
        raise StepError('TTS_BE_UNREACHABLE', 'Check BE HTTPS address, CA, clock and firewall') from None


def volume_wav(raw, volume):
    """Scale PCM and rebuild lengths, including provider WAVs with streaming headers."""
    try:
        with wave.open(io.BytesIO(raw), 'rb') as source:
            params = source.getparams()
            if source.getcomptype() != 'NONE' or source.getsampwidth() not in (1, 2, 3, 4):
                raise StepError('TTS_WAV_FORMAT_UNSUPPORTED')
            width = source.getsampwidth()
            frame_size = width * source.getnchannels()
            frames = source.readframes(MAX_AUDIO_BYTES // frame_size + 1)
        if not frames or len(frames) > MAX_AUDIO_BYTES or len(frames) % frame_size:
            raise StepError('TTS_WAV_FORMAT_UNSUPPORTED')
        output = frames
        if volume != 100:
            output = bytearray(len(frames))
            for start in range(0, len(frames), width):
                value = int.from_bytes(frames[start:start+width], 'little', signed=width != 1)
                if width == 1:
                    value -= 128
                value = round(value * volume / 100)
                if width == 1:
                    value += 128
                output[start:start+width] = value.to_bytes(width, 'little', signed=width != 1)
        buffer = io.BytesIO()
        with wave.open(buffer, 'wb') as target:
            # A streaming provider may declare 0xffffffff bytes rather than the
            # actual length. Never reuse its frame count in the output header.
            target.setparams(params._replace(nframes=len(output) // frame_size))
            target.writeframes(output)
        return buffer.getvalue()
    except (wave.Error, EOFError, ValueError, struct.error):
        raise StepError('TTS_WAV_FORMAT_UNSUPPORTED') from None


async def download(cfg, data):
    deadline = time.monotonic() + cfg.get('wait_timeout_sec', 45)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise StepError('TTS_NOT_READY', 'Prepare this text on BE first, then create a new speech Job')
        try:
            raw = await asyncio.wait_for(asyncio.to_thread(fetch_once, cfg, data), timeout=remaining)
        except asyncio.TimeoutError:
            raise StepError('TTS_NOT_READY') from None
        if raw is not None:
            return raw
        await asyncio.sleep(min(2, max(0, deadline - time.monotonic())))
