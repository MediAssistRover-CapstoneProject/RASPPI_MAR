"""Plan navigation speech on the Pi from the accepted Nav2 path and live pose."""
import asyncio
from dataclasses import dataclass
import logging
import math
import time

from .contracts import StepError
from .store import utcnow


PHRASES = {
    'STRAIGHT': 'Tiếp tục đi thẳng.',
    'LEFT': 'Chuẩn bị rẽ trái.',
    'RIGHT': 'Chuẩn bị rẽ phải.',
    'UTURN': 'Chuẩn bị quay lại.',
    'NEAR': 'Sắp tới nơi.',
    'WAITING': 'Tôi đang tìm lại đường đi. Bạn vui lòng chờ.',
}
TRANSIENT_LOCALIZATION = {
    'VERIFYING', 'AMCL_UNCERTAIN', 'POSE_AGE_SEC_STALE', 'LOCALIZED',
    'SCAN_AGE_SEC_STALE', 'ODOM_AGE_SEC_STALE', 'TF_NOT_READY',
    'AMCL_TF_NOT_READY', 'SCAN_TF_UNAVAILABLE', 'SCAN_MATCH_LOW',
}


@dataclass(frozen=True)
class Cue:
    direction: str
    anchor: tuple[float, float] | None
    distance: float | None = None

    def identifier(self, step):
        anchor = '' if self.anchor is None else ':' + ':'.join(str(round(x * 10)) for x in self.anchor)
        return step['jobStepId'] + ':' + self.direction + anchor

    def matches(self, other):
        if other is None or self.direction != other.direction:
            return False
        if self.anchor is None or other.anchor is None:
            return self.anchor == other.anchor
        return math.dist(self.anchor, other.anchor) < .30


def speech_params(direction, speed_wpm):
    return {'action': 'SPEAK', 'prompt_key': 'NAVIGATION_' + direction,
            'data': {'text': PHRASES[direction], 'language_code': 'vi',
                     'speed_wpm': speed_wpm, 'volume': 70}}


async def prepare_navigation_audio(worker):
    """Warm a bounded in-memory cache without touching ALSA or startup readiness."""
    config = worker.cfg.get('speech', {})
    if (not config.get('enabled') or config.get('engine') != 'be-tts'
            or not worker.cfg.get('navigation_session')
            or 'INTERACT' not in worker.cfg.get('enabled_step_types', [])):
        return
    from .cloud_speech import download
    cache = worker.navigation_audio
    for direction in ('LEFT', 'RIGHT', 'STRAIGHT', 'NEAR', 'UTURN', 'WAITING'):
        if direction in cache:
            continue
        try:
            raw = await asyncio.wait_for(download(config, speech_params(direction, 150)['data']), 12.)
        except Exception as exc:
            logging.debug('NAVIGATION_GUIDANCE_PREPARE kind=%s error=%s', direction, type(exc).__name__)
        else:
            if len(raw) <= 256 * 1024:
                cache[direction] = raw


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class GuidancePath:
    """Simplified speech corners, measured along the original plan."""
    def __init__(self, source):
        self.points = []
        self.lengths = []
        for value in source:
            point = (value['xM'], value['yM'])
            distance = math.dist(self.points[-1], point) if self.points else 0.
            if self.points and distance < .001:
                continue
            self.points.append(point)
            self.lengths.append((self.lengths[-1] if self.lengths else 0.) + distance)
        self.corners = []
        if len(self.points) >= 2:
            self._find_corners()

    @staticmethod
    def projection(a, b, point):
        dx, dy = (b[0] - a[0], b[1] - a[1])
        squared = dx * dx + dy * dy
        fraction = max(0., min(1., ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / squared)) if squared else 0.
        return fraction, math.dist(point, (a[0] + fraction * dx, a[1] + fraction * dy))

    def _find_corners(self):
        last = len(self.points) - 1
        keep, pending = {0, last}, [(0, last)]
        while pending:
            lo, hi = pending.pop()
            if hi - lo < 2:
                continue
            index = max(range(lo + 1, hi), key=lambda i:
                        self.projection(self.points[lo], self.points[hi], self.points[i])[1])
            if self.projection(self.points[lo], self.points[hi], self.points[index])[1] > .035:
                keep.add(index)
                pending.extend(((lo, index), (index, hi)))
        indices = sorted(keep)
        for a, b, c in zip(indices, indices[1:], indices[2:]):
            incoming = self.heading(self.points[a], self.points[b])
            outgoing = self.heading(self.points[b], self.points[c])
            angle = math.degrees(angle_difference(outgoing, incoming))
            if abs(angle) >= 35:
                direction = 'UTURN' if abs(angle) >= 150 else 'LEFT' if angle > 0 else 'RIGHT'
                self.corners.append((b, direction))

    @staticmethod
    def heading(a, b):
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def cue(self, pose, speed):
        if len(self.points) < 2:
            return None
        point = (pose['x_m'], pose['y_m'])
        yaw = pose['yaw_rad']
        best = (math.inf, 0., 0., 0.)
        for index, (a, b) in enumerate(zip(self.points, self.points[1:])):
            fraction, offset = self.projection(a, b, point)
            heading = angle_difference(self.heading(a, b), yaw)
            # Nearby segments can intersect on a tight replan. Break ties by heading.
            score = offset + .02 * (1 - math.cos(heading))
            if score < best[0]:
                along = self.lengths[index] + fraction * (self.lengths[index + 1] - self.lengths[index])
                best = (score, offset, along, heading)
        _, offset, along, heading = best
        if offset > .25:
            return None
        turn_lead = max(.35, min(.90, speed * 2.4))
        for index, direction in self.corners:
            distance = self.lengths[index] - along
            if distance <= .06:
                continue
            if distance <= turn_lead:
                return Cue(direction, self.points[index], distance)
            if distance >= .55 and abs(heading) <= math.pi / 4:
                return Cue('STRAIGHT', self.points[index], distance)
            return None
        remaining = self.lengths[-1] - along
        if .25 <= remaining <= .55:
            return Cue('NEAR', None, remaining)
        if remaining > .55 and abs(heading) <= math.pi / 4:
            return Cue('STRAIGHT', self.points[-1], remaining)
        return None


async def run_navigation_guidance(worker, step):
    try:
        await NavigationGuidance(worker, step).run()
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        logging.warning('NAVIGATION_GUIDANCE_UNAVAILABLE %s', type(exc).__name__)


class NavigationGuidance:
    MAX_PHRASE_FINISH_SEC = 5.

    def __init__(self, worker, step):
        self.worker = worker
        self.step = step
        self.context = dict(worker.cfg['navigation_session'])
        self.initialization_revision = None
        self.last_localization_reason = None
        self.last_pose_good = -math.inf
        self.playback_started_at = None
        self.path_source = self.path = None
        self.announced = []
        self.attempted = set()

    def context_valid(self):
        adapter = self.worker.adapter
        active = adapter.active_navigation_context or {}
        return (self.worker.connection_ready and self.worker.active is self.step
                and not getattr(self.worker, 'shutting_down', False)
                and not getattr(self.worker, 'session_fault', False)
                and not getattr(adapter, 'motion_uncertain', False)
                and all(self.worker.cfg.get('navigation_session', {}).get(key) == self.context[key]
                        for key in ('session_id', 'map_id', 'map_asset_hash'))
                and active.get('initialization_revision') is not None
                and (self.initialization_revision is None
                     or active['initialization_revision'] == self.initialization_revision))

    def current_cue(self):
        adapter = self.worker.adapter
        waiting = getattr(adapter, 'navigation_obstacle_wait', None)
        if waiting:
            return Cue('WAITING', None) if time.monotonic() - waiting['since'] >= 1. else None
        if not getattr(adapter, 'navigation_guidance_tracking', False):
            return None
        source = getattr(adapter, 'navigation_guidance_path', None)
        revision = adapter.active_navigation_context['initialization_revision']
        if source is None or any(source.get(key) != value for key, value in (
                ('jobId', self.step['jobId']), ('jobStepId', self.step['jobStepId']),
                ('sessionId', self.context['session_id']), ('mapId', self.context['map_id']),
                ('mapAssetHash', self.context['map_asset_hash']), ('initializationRevision', revision))):
            return None
        state = adapter.telemetry()
        self.last_localization_reason = state.get('localization', {}).get('reason')
        pose = state.get('map_pose', {})
        value = pose.get('value') or {}
        if not (pose.get('fresh') and pose['age_sec'] <= 1.5 and state.get('localization', {}).get('ready')
                and value.get('map_id') == self.context['map_id']
                and value.get('navigation_session_id') == self.context['session_id']
                and value.get('initialization_revision') == revision):
            return None
        if self.path_source is not source:
            self.path = GuidancePath(source['points'])
            self.path_source = source
        self.initialization_revision = revision
        self.last_pose_good = time.monotonic()
        sample = getattr(adapter, 'odom_twist_sample', None)
        limit = self.step['params']['max_speed_mps']
        speed = min(limit, max(.12, math.hypot(*sample[0][:2]))) if sample and self.last_pose_good - sample[1] <= .5 else limit
        return self.path.cue(value, speed)

    def can_play(self, cue, source):
        from .media_runtime import audio_reserved
        if audio_reserved() or not self.context_valid():
            return False
        latest = self.current_cue()
        if self.playback_started_at is None:
            return cue.matches(latest)
        now = time.monotonic()
        if now - self.playback_started_at > self.MAX_PHRASE_FINISH_SEC:
            return False
        if cue.matches(latest):
            return True
        adapter = self.worker.adapter
        if (cue.direction == 'WAITING' or getattr(adapter, 'navigation_obstacle_wait', None)
                or not getattr(adapter, 'navigation_guidance_tracking', False)
                or source is not getattr(adapter, 'navigation_guidance_path', None)):
            return False
        # The same path can advance to the next cue during a short phrase.
        return (now - self.last_pose_good <= .1
                or (self.last_localization_reason in TRANSIENT_LOCALIZATION
                    and now - self.last_pose_good <= self.MAX_PHRASE_FINISH_SEC))

    async def play(self, cue, cue_id, source):
        config = self.worker.cfg['speech']
        player = self.worker.adapter.speech
        self.playback_started_at = None
        started = time.monotonic()

        def playback_started():
            self.playback_started_at = time.monotonic()
            self.announced.append((cue, source, self.playback_started_at))
            latest = self.current_cue()
            remaining_cm = round(latest.distance * 100) if latest and latest.distance is not None else None
            logging.info('NAVIGATION_GUIDANCE_STARTED kind=%s delay_ms=%d ahead_cm=%s', cue.direction,
                         round((self.playback_started_at - started) * 1000), remaining_cm)

        def guard():
            return self.can_play(cue, source)

        try:
            raw = self.worker.navigation_audio.get(cue.direction)
            if raw is not None:
                await player.play_wav(raw, volume=70, playback_guard=guard, on_playback_start=playback_started)
                result = {'completed': True, 'action': 'SPEAK',
                          'prompt_key': 'NAVIGATION_' + cue.direction, 'source': 'AUDIO_CLIENT',
                          'actor_account_id': None, 'completed_at': utcnow(),
                          'values': {'playback_completed': True, 'audible_verified': False, 'engine': 'be-tts',
                                     'language_code': 'vi'}}
            else:
                if config.get('engine') == 'be-tts':
                    from .speech import SpeechPlayer
                    player = SpeechPlayer({**config, 'engine': 'espeak-ng'})
                result = await player.speak(speech_params(cue.direction, 180),
                                            playback_guard=guard, on_playback_start=playback_started)
            self.worker.store.guidance_result(self.step, cue_id, 'COMPLETED', result)
        except asyncio.CancelledError:
            self.worker.store.guidance_result(self.step, cue_id, 'CANCELLED', {'reason': 'GUIDANCE_NO_LONGER_CURRENT'})
            raise
        except StepError as exc:
            status = 'CANCELLED' if exc.code == 'NAVIGATION_GUIDANCE_EXPIRED' else 'FAILED'
            self.worker.store.guidance_result(self.step, cue_id, status, {'errorCode': exc.code})
            if status == 'FAILED':
                logging.warning('NAVIGATION_GUIDANCE_AUDIO_FAILED %s', exc.code)
        except Exception as exc:
            self.worker.store.guidance_result(self.step, cue_id, 'FAILED', {'errorCode': type(exc).__name__})
            logging.warning('NAVIGATION_GUIDANCE_AUDIO_FAILED %s', type(exc).__name__)

    async def run(self):
        from .media_runtime import audio_reserved
        speech = None
        cue = source = None
        try:
            while True:
                if speech is not None and not speech.done():
                    if not self.can_play(cue, source):
                        speech.cancel()
                        await asyncio.gather(speech, return_exceptions=True)
                elif self.context_valid() and not audio_reserved():
                    cue = self.current_cue()
                    if cue:
                        source = getattr(self.worker.adapter, 'navigation_guidance_path', None)
                        cue_id = cue.identifier(self.step)
                        now = time.monotonic()
                        duplicate = any(old.matches(cue) and old_source is not source and now - when <= 12.
                                        for old, old_source, when in self.announced)
                        if cue_id not in self.attempted and not duplicate:
                            self.attempted.add(cue_id)
                            if self.worker.store.guidance_start(self.step, cue_id):
                                speech = asyncio.create_task(self.play(cue, cue_id, source))
                await asyncio.sleep(.1)
        finally:
            if speech is not None and not speech.done():
                speech.cancel()
                await asyncio.gather(speech, return_exceptions=True)
