"""Scoped speech alongside MOVE; brief telemetry gaps do not clip a valid phrase."""
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
    MAX_REPLY_AGE = 2.5
    WAIT_TEXT = 'Tôi đang tìm lại đường đi. Bạn vui lòng chờ.'

    def __init__(self, worker, step):
        self.worker = worker
        self.step = step
        self.client = SessionClient(worker.cfg)
        self.context = dict(worker.cfg['navigation_session'])
        self.latest = None
        self.speech = None
        self.request = None
        self.current_cue = None
        self.last_attempt = None
        self.attempted = set()
        self.received_at = 0
        self.current_plan = None
        self.current_local = False
        self.last_pose_good = -math.inf
        self.cancel_reason = 'STEP_ENDED'
        self.initialization_revision = None
        self.last_localization_reason = None

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
        texts = ['Phía trước '+turn for turn in ('rẽ trái', 'rẽ phải', 'quay lại')]
        await asyncio.gather(*(prepare(text) for text in texts + ['Mời bạn đi thẳng theo tôi.', self.WAIT_TEXT]))

    def context_valid(self):
        active = self.worker.adapter.active_navigation_context or {}
        return (self.worker.connection_ready and self.worker.active is self.step
                and all(self.worker.cfg.get('navigation_session',{}).get(key) == self.context[key]
                        for key in ('session_id','map_id','map_asset_hash'))
                and active.get('initialization_revision') is not None
                and (self.initialization_revision is None
                     or active['initialization_revision'] == self.initialization_revision))

    def fresh(self):
        if not self.context_valid():
            return False
        context = self.worker.cfg['navigation_session']
        state = self.worker.adapter.telemetry()
        self.last_localization_reason = state.get('localization',{}).get('reason')
        pose = state.get('map_pose', {})
        value = pose.get('value') or {}
        valid = (pose.get('fresh') and pose.get('age_sec', 99) <= 1.5
                and state.get('localization', {}).get('ready')
                and value.get('navigation_session_id') == context['session_id']
                and value.get('initialization_revision') == (self.worker.adapter.active_navigation_context or {}).get('initialization_revision'))
        if valid:
            self.initialization_revision = self.worker.adapter.active_navigation_context['initialization_revision']
            self.last_pose_good = time.monotonic()
        return bool(valid)

    def valid_reply(self, value):
        plan = value.get('plan', {})
        context = self.worker.cfg['navigation_session']
        try:age = (datetime.now(timezone.utc) - parse_timestamp(value['capturedAt'])).total_seconds()
        except (KeyError, TypeError, ValueError):return False
        return (value.get('state') == 'TRACKING' and -1 <= age <= self.MAX_REPLY_AGE
                and plan.get('jobId') == self.step['jobId']
                and plan.get('jobStepId') == self.step['jobStepId']
                and plan.get('sessionId') == context['session_id']
                and plan.get('mapId') == context['map_id']
                and plan.get('mapAssetHash') == context['map_asset_hash']
                and plan.get('initializationRevision') == (self.worker.adapter.active_navigation_context or {}).get('initialization_revision'))

    @staticmethod
    def revision(value):
        return value.get('planRevision') or value.get('plan', {}).get('plannedAt')

    def can_play(self, cue):
        if not self.context_valid():return False
        if self.current_local:
            return bool(getattr(self.worker.adapter, 'navigation_obstacle_wait', None))
        # The next cue on the SAME path must not cut the current short phrase.
        # A changed route, initialization or step still revokes it immediately.
        if (self.latest is None or time.monotonic()-self.received_at > self.MAX_REPLY_AGE
                or self.revision(self.latest) != self.current_plan):return False
        if self.fresh():return True
        benign = self.last_localization_reason in ('VERIFYING','AMCL_UNCERTAIN','POSE_AGE_SEC_STALE','LOCALIZED')
        return benign and time.monotonic()-self.last_pose_good <= 2.

    def waiting_command(self):
        waiting = getattr(self.worker.adapter, 'navigation_obstacle_wait', None)
        if not waiting or time.monotonic()-waiting['since'] < 1.:return None
        return dict(cueId=self.step['jobStepId']+':WAITING_FOR_PATH', stepType='INTERACT',
                    parentJobStepId=self.step['jobStepId'], direction='WAITING',
                    params=dict(action='SPEAK', prompt_key='NAVIGATION_WAIT', data=dict(
                        text=self.WAIT_TEXT, language_code='vi', speed_wpm=150, volume=70)))

    async def play(self, command):
        cue = command['cueId']
        started = time.monotonic()
        try:
            result = await self.worker.adapter.speech.speak(command['params'],
                    playback_guard=lambda: self.can_play(cue))
            self.worker.store.guidance_result(self.step, cue, 'COMPLETED', result)
            logging.info('NAVIGATION_GUIDANCE_COMPLETED kind=%s elapsed_ms=%d',
                         command.get('direction','TURN'),round((time.monotonic()-started)*1000))
        except asyncio.CancelledError:
            self.worker.store.guidance_result(self.step, cue, 'CANCELLED', {'reason': self.cancel_reason})
            logging.info('NAVIGATION_GUIDANCE_CANCELLED reason=%s',self.cancel_reason)
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
                if self.request is not None and self.request.done():
                    try:
                        value = json.loads(self.request.result())
                        if self.valid_reply(value):
                            self.latest = value
                            self.received_at = now
                        elif value.get('state') in ('IDLE','STOPPED','ARRIVED'):
                            self.latest = None
                        elif self.latest and value.get('plan') and (
                                value['plan'].get('sessionId') != self.context['session_id']
                                or value['plan'].get('jobId') != self.step['jobId']
                                or value['plan'].get('jobStepId') != self.step['jobStepId']
                                or value['plan'].get('mapAssetHash') != self.context['map_asset_hash']
                                or value['plan'].get('initializationRevision') != self.latest['plan'].get('initializationRevision')
                                or self.revision(value) != self.revision(self.latest)):
                            self.latest = None
                    except Exception:
                        pass  # One failed poll never renews or instantly revokes the last valid snapshot.
                    self.request = None
                    next_request = now + .5
                if self.request is None and now >= next_request:
                    self.request = asyncio.create_task(asyncio.to_thread(self.client.request,
                        '/api/v1/pi/navigation-session/guidance', {'jobStepId': self.step['jobStepId']}))
                if self.speech and not self.speech.done() and not self.can_play(self.current_cue):
                    self.cancel_reason = ('PATH_CHANGED' if self.latest and self.revision(self.latest) != self.current_plan
                                          else 'GUIDANCE_NO_LONGER_CURRENT')
                    self.speech.cancel()
                    await asyncio.gather(self.speech, return_exceptions=True)
                command = self.waiting_command()
                local = command is not None
                if command is None and self.latest and self.valid_reply(self.latest) and self.fresh():
                    command = self.latest.get('interaction')
                if command and (self.speech is None or self.speech.done()):
                    cue = command.get('cueId')
                    params = command.get('params', {})
                    if (self.context_valid() and command.get('stepType') == 'INTERACT' and params.get('action') == 'SPEAK'
                            and command.get('parentJobStepId') == self.step['jobStepId']
                            and isinstance(params.get('data', {}).get('text'), str)
                            and 0 < len(params['data']['text']) <= 500
                            and cue not in self.attempted):
                        turn = command.get('anchor') or (self.latest or {}).get('turn')
                        direction = command.get('direction') or (self.latest or {}).get('direction')
                        # Replans can nudge the same corner into an adjacent 5 cm bucket.
                        duplicate = (turn and self.last_attempt and self.last_attempt[0] == direction
                            and math.hypot(turn['xM']-self.last_attempt[1], turn['yM']-self.last_attempt[2]) < .15)
                        self.attempted.add(cue)
                        if not duplicate and self.worker.store.guidance_start(self.step, cue):
                            if turn:self.last_attempt = (direction, turn['xM'], turn['yM'])
                            self.current_cue = cue
                            self.current_plan = self.revision(self.latest or {})
                            self.current_local = local
                            self.speech = asyncio.create_task(self.play(command))
                await asyncio.sleep(.1)
        finally:
            self.cancel_reason = 'STEP_ENDED'
            pending = [task for task in (self.speech, self.request, preparation) if task is not None]
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
