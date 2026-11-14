"""Optional, stationary RGB fall observation during a patrol sensing step."""
import asyncio
import json
import logging
import math
import ssl
import time
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit
from uuid import uuid4

from .be_token import NoRedirect
from .camera_preview import CameraPreview, encode_jpeg
from .contracts import StepError
from .navigation_session import SessionClient


def configuration(ai, camera):
    config = ai.get('patrol_fall')
    if config is None:
        return None
    if not isinstance(config, dict) or type(config.get('enabled')) is not bool:
        raise StepError('INVALID_PATROL_FALL_CONFIG')
    if not config['enabled']:
        return None
    origin = config.get('base_url')
    if not isinstance(origin, str):
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'base_url')
    try:
        uri = urlsplit(origin)
        uri.port
    except ValueError:
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'base_url') from None
    if (not camera.get('enabled') or uri.scheme != 'https' or not uri.hostname
            or uri.path not in ('', '/') or uri.username or uri.password or uri.query or uri.fragment):
        raise StepError('INVALID_PATROL_FALL_CONFIG')
    for key in ('token_file', 'ca_file'):
        value = config.get(key)
        if key == 'ca_file' and value is None:
            continue
        if not isinstance(value, str) or not Path(value).is_absolute():
            raise StepError('INVALID_PATROL_FALL_CONFIG', key)
    rate = config.get('rate_hz', 4)
    if type(rate) not in (int, float) or not 2 <= rate <= 5:
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'rate_hz')
    return {**config, 'base_url': uri.scheme + '://' + uri.netloc, 'rate_hz': rate}


def read_token(path):
    try:
        token = Path(path).read_text().strip()
    except (OSError, UnicodeError):
        raise StepError('PATROL_FALL_TOKEN_UNAVAILABLE') from None
    if len(token) < 32 or not token.isascii() or any(c.isspace() for c in token):
        raise StepError('PATROL_FALL_TOKEN_INVALID')
    return token


class PatrolFallObserver:
    def __init__(self, adapter, config, topic):
        self.adapter = adapter
        self.config = config
        self.camera = dict(topic=topic, rate_hz=config['rate_hz'], max_width=640,
                           max_height=480, jpeg_quality=55, max_jpeg_bytes=256 * 1024)
        read_token(config['token_file'])
        self.be = SessionClient(adapter.worker)
        self.work = None
        tls = ssl.create_default_context(cafile=config.get('ca_file'))
        tls.minimum_version = ssl.TLSVersion.TLSv1_2
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect(),
                                                  urllib.request.HTTPSHandler(context=tls))

    def stationary_revision(self):
        state = self.adapter.telemetry()
        pose = state.get('map_pose', {})
        localization = state.get('localization', {})
        twist = self.adapter.odom_twist_sample
        if (pose.get('fresh') is True and localization.get('ready') is True and twist
                and time.monotonic() - twist[1] <= .5 and math.hypot(*twist[0][:2]) <= .01
                and abs(twist[0][2]) <= .03):
            return localization.get('session_id'), localization.get('initialization_revision')
        return None

    def analyze(self, jpeg, session, token):
        request = urllib.request.Request(self.config['base_url'] + '/v1/fall/frame', jpeg,
                                         {'Authorization': 'Bearer ' + token, 'Content-Type': 'image/jpeg',
                                          'X-Fall-Session': session})
        with self.opener.open(request, timeout=.9) as response:
            raw = response.read(4097)
        if len(raw) > 4096:
            raise ValueError('fall response is too large')
        value = json.loads(raw)
        if not isinstance(value, dict) or type(value.get('suspected_fall')) is not bool:
            raise ValueError('fall response is invalid')
        if value['suspected_fall']:
            confidence = value.get('confidence')
            algorithm = value.get('algorithm')
            if (type(confidence) not in (int, float) or not 0 <= confidence <= 1
                    or not isinstance(algorithm, str) or not 1 <= len(algorithm) <= 100):
                raise ValueError('fall detection data is invalid')
        return value

    async def report(self, step, result, captured_at):
        body = dict(eventId=str(uuid4()), jobStepId=step['jobStepId'], detectedAt=captured_at,
                    confidence=result['confidence'], algorithm=result['algorithm'])
        try:
            await asyncio.wait_for(asyncio.to_thread(self.be.request, '/api/v1/pi/patrol-fall-events', body), 5.)
        except (StepError, OSError, asyncio.TimeoutError):
            logging.warning('PATROL_FALL_REPORT_UNCONFIRMED event=%s', body['eventId'])
            raise StepError('PATROL_FALL_REPORT_UNCONFIRMED') from None
        logging.warning('PATROL_FALL_REVIEW_REQUIRED event=%s', body['eventId'])
        raise StepError('PATROL_FALL_REVIEW_REQUIRED')

    async def observe(self, step):
        from .media_runtime import audio_reserved

        if self.work is not None and not self.work.done():
            raise StepError('PATROL_FALL_PROVIDER_BUSY')
        token = read_token(self.config['token_file'])
        window = step['params']['window_sec']
        deadline = time.monotonic() + window
        preview = CameraPreview(self.adapter, self.camera, lambda: not audio_reserved())
        successful = 0
        revision = None
        session = None
        first_frame_after = math.inf
        try:
            while time.monotonic() < deadline:
                await asyncio.sleep(.05)
                if audio_reserved():
                    raise StepError('PATROL_INTERRUPTED_BY_SOS')
                live = self.stationary_revision()
                if live != revision:
                    revision, session = live, str(uuid4())
                    first_frame_after = time.monotonic()
                    successful = 0
                if live is None:
                    continue
                sample = preview.take_sample(.35)
                if not sample or sample[-1] < first_frame_after:
                    continue
                try:
                    def evaluate():
                        frame = encode_jpeg(sample, self.camera)
                        return self.analyze(frame['jpeg'], session, token) if frame else None

                    self.work = asyncio.create_task(asyncio.to_thread(evaluate))
                    self.work.add_done_callback(lambda task: task.exception() if not task.cancelled() else None)
                    result = await asyncio.wait_for(asyncio.shield(self.work), max(.01, deadline - time.monotonic()))
                except asyncio.TimeoutError:
                    break
                except (OSError, ValueError):
                    continue
                if (not result or time.monotonic() >= deadline or audio_reserved() or self.stationary_revision() != revision
                        or time.monotonic() - sample[-1] > 1.2):
                    continue
                successful += 1
                if result['suspected_fall']:
                    await self.report(step, result, sample[-2])
            if successful < 3:
                raise StepError('PATROL_FALL_DATA_UNAVAILABLE')
            logging.info('PATROL_FALL_WINDOW_COMPLETED samples=%d step=%s', successful, step['jobStepId'])
            return successful
        finally:
            self.adapter.node.destroy_subscription(preview.subscription)
