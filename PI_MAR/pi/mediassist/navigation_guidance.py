"""Authenticated BE INTERACT/SPEAK hints scoped to a running MOVE."""
import asyncio
import json
import logging
import math
import time
from datetime import datetime, timezone

from .navigation_session import SessionClient
from .contracts import parse_timestamp


async def run_navigation_guidance(worker, step):
    try:
        await NavigationGuidance(worker, step).run()
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        logging.warning('NAVIGATION_GUIDANCE_UNAVAILABLE %s', type(exc).__name__)


class NavigationGuidance:
    def __init__(self, worker, step):
        self.worker = worker
        self.step = step
        self.client = SessionClient(worker.cfg)
        self.latest = None
        self.speech = None
        self.request = None
        self.current_cue = None
        self.last_attempt = None
        self.attempted = set()
        self.received_at = 0

    async def prepare_audio(self):
        config = self.worker.cfg.get('speech', {})
        if config.get('engine') != 'be-tts':
            return
        from .cloud_speech import download
        async def prepare(text):
            try:
                await download(config, {'text': text, 'language_code': 'vi', 'speed_wpm': 150})
            except Exception as exc:
                logging.debug('NAVIGATION_GUIDANCE_PREPARE %s', type(exc).__name__)
        await asyncio.gather(*(prepare('Phía trước '+turn) for turn in ('rẽ trái', 'rẽ phải', 'quay lại')))

    def fresh(self):
        if not self.worker.connection_ready or self.worker.active is not self.step:
            return False
        context = self.worker.cfg['navigation_session']
        state = self.worker.adapter.telemetry()
        pose = state.get('map_pose', {})
        value = pose.get('value') or {}
        return (pose.get('fresh') and pose.get('age_sec', 99) <= 1
                and state.get('localization', {}).get('ready')
                and value.get('navigation_session_id') == context['session_id']
                and value.get('initialization_revision') == (self.worker.adapter.active_navigation_context or {}).get('initialization_revision'))

    def valid_reply(self, value):
        plan = value.get('plan', {})
        context = self.worker.cfg['navigation_session']
        captured = parse_timestamp(value['capturedAt'])
        age = (datetime.now(timezone.utc) - captured).total_seconds()
        return (value.get('state') == 'TRACKING' and -1 <= age <= 1
                and plan.get('jobId') == self.step['jobId']
                and plan.get('jobStepId') == self.step['jobStepId']
                and plan.get('sessionId') == context['session_id']
                and plan.get('mapId') == context['map_id']
                and plan.get('mapAssetHash') == context['map_asset_hash']
                and plan.get('initializationRevision') == self.worker.adapter.active_navigation_context.get('initialization_revision'))

    def can_play(self, cue):
        return (self.fresh() and time.monotonic() - self.received_at < 1.2
                and self.latest is not None and self.latest.get('cueId') == cue
                and self.latest.get('interaction') is not None)

    async def play(self, command):
        cue = command['cueId']
        try:
            result = await self.worker.adapter.speech.speak(command['params'],
                    playback_guard=lambda: self.can_play(cue))
            self.worker.store.guidance_result(self.step, cue, 'COMPLETED', result)
        except asyncio.CancelledError:
            self.worker.store.guidance_result(self.step, cue, 'CANCELLED', None)
            raise
        except Exception as exc:
            self.worker.store.guidance_result(self.step, cue, 'FAILED', {'errorCode': str(getattr(exc, 'code', type(exc).__name__))})
            logging.warning('NAVIGATION_GUIDANCE_AUDIO_FAILED %s', type(exc).__name__)

    async def run(self):
        next_request = 0
        preparation = asyncio.create_task(self.prepare_audio())
        try:
            while True:
                now = time.monotonic()
                if self.speech and not self.speech.done() and not self.can_play(self.current_cue):
                    self.speech.cancel()
                    await asyncio.gather(self.speech, return_exceptions=True)
                if self.request is not None and self.request.done():
                    try:
                        value = json.loads(self.request.result())
                        self.latest = value if self.valid_reply(value) else None
                        self.received_at = now
                    except Exception:
                        self.latest = None
                    self.request = None
                    next_request = now + .25
                if self.request is None and now >= next_request:
                    self.request = asyncio.create_task(asyncio.to_thread(self.client.request,
                        '/api/v1/pi/navigation-session/guidance', {'jobStepId': self.step['jobStepId']}))
                command = (self.latest or {}).get('interaction')
                if command and (self.speech is None or self.speech.done()):
                    cue = command.get('cueId')
                    params = command.get('params', {})
                    if (command.get('stepType') == 'INTERACT' and params.get('action') == 'SPEAK'
                            and command.get('parentJobStepId') == self.step['jobStepId']
                            and isinstance(params.get('data', {}).get('text'), str)
                            and 0 < len(params['data']['text']) <= 500
                            and cue not in self.attempted and self.can_play(cue)):
                        turn = self.latest.get('turn', {})
                        # Replans can nudge the same corner into an adjacent 5 cm bucket.
                        duplicate = (self.last_attempt and self.last_attempt[0] == self.latest.get('direction')
                            and math.hypot(turn['xM']-self.last_attempt[1], turn['yM']-self.last_attempt[2]) < .15)
                        self.attempted.add(cue)
                        if not duplicate and self.worker.store.guidance_start(self.step, cue):
                            self.last_attempt = (self.latest.get('direction'), turn['xM'], turn['yM'])
                            self.current_cue = cue
                            self.speech = asyncio.create_task(self.play(command))
                await asyncio.sleep(.05)
        finally:
            pending = [task for task in (self.speech, self.request, preparation) if task is not None]
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
