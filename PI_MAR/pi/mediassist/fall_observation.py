"""Relay early AI candidates; BE alone owns patrol suspension and its lease.

No motor commands, depth/TF gate, second inference or per-frame HTTP callbacks.
"""
import asyncio
import hashlib
import json
import logging
import math
import time
from uuid import UUID, uuid4


REASONS = {'LOW_HORIZONTAL_POSTURE', 'PARTIAL_LOW_HORIZONTAL_POSTURE', 'END_ON_LOWER_BODY'}
TERMINAL = {'CLEAR', 'INCIDENT', 'INCONCLUSIVE', 'EXPIRED', 'CANCELLED', 'PREEMPTED'}


def bbox(value):
    if (not isinstance(value, list) or len(value) != 4
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)
            or min(value) < 0 or min(value[2:]) <= 0
            or value[0] + value[2] > 1.000001 or value[1] + value[3] > 1.000001):
        return None
    return value


def overlap(a, b):
    width = max(0., min(a[0]+a[2], b[0]+b[2])-max(a[0], b[0]))
    height = max(0., min(a[1]+a[3], b[1]+b[3])-max(a[1], b[1]))
    intersection = width * height
    return intersection / max(1e-9, a[2]*a[3]+b[2]*b[3]-intersection)


def report_identity(result, provider_session):
    """Only the person that produced the event owns its reporting cooldown."""
    person = result.get('person')
    if not isinstance(person, dict):
        candidate = result.get('floor_candidate')
        person = candidate.get('person') if isinstance(candidate, dict) else None
    if not isinstance(person, dict) or type(person.get('track_id')) is not int:
        return None
    try:revision = str(UUID(person.get('tracking_revision')))
    except (ValueError, TypeError, AttributeError):return None
    return provider_session, revision, person['track_id']


class FallObservation:
    def __init__(self, observer):
        self.observer = observer
        self.enabled = observer.config.get('observation_pause_enabled', True)
        self.task = None
        self.body = None
        self.box = None
        self.fresh_after = math.inf
        self.latest = None
        self.reporting = False
        self.incident_reported = False
        self.cooldowns = []
        self.key = None
        self.next_candidate = 0.

    async def request(self, path, body=None):
        raw = await asyncio.to_thread(self.observer.be.request, path, body, timeout=2.)
        value = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
        if not isinstance(value, dict):
            raise ValueError('INVALID_OBSERVATION_RESPONSE')
        return value

    def matched(self, result):
        if self.box is None or not isinstance(result, dict):
            return None
        overlay = result.get('overlay')
        if not isinstance(overlay, dict):return None
        items = overlay.get('detections', [])
        if not isinstance(items, list):return None
        ranked = sorted(((overlap(self.box, box), item) for item in items if isinstance(item, dict)
                         and (box := bbox(item.get('bbox'))) is not None),
                        key=lambda value: value[0], reverse=True)
        if not ranked or ranked[0][0] < .25:
            return None
        if len(ranked) > 1 and ranked[1][0] >= ranked[0][0] - .10:
            return None
        return ranked[0][1]

    def offer(self, worker, result, captured_at, captured_mono, provider_session):
        if not self.enabled:
            return
        now = time.monotonic()
        match = self.matched(result)
        if self.body is not None and captured_mono >= self.fresh_after:
            self.latest = (captured_mono, match.get('state') if match else None)
            if match is not None:self.box = bbox(match['bbox'])
        if self.task is not None or now < self.next_candidate:
            return
        candidate = result.get('observation_candidate')
        step = worker.active
        if (not isinstance(candidate, dict) or type(candidate.get('version')) is not int or candidate.get('version') != 1
                or not isinstance(candidate.get('reason'), str) or candidate.get('reason') not in REASONS
                or candidate.get('phase') != 'OBSERVATION_CANDIDATE'
                or type(candidate.get('track_id')) is not int or candidate['track_id'] < 0
                or not step or step.get('stepType') != 'MOVE' or step.get('params', {}).get('mode') != 'PATROL'
                or not worker.connection_ready or worker.shutting_down or worker.session_fault
                or not 0 <= now-captured_mono <= 1.5):
            return
        duration = candidate.get('continuous_duration_sec')
        box = bbox(candidate.get('bbox'))
        quality = candidate.get('visual_quality', {})
        if (type(duration) not in (int, float) or not math.isfinite(duration) or duration < .5
                or box is None or not isinstance(quality, dict)):
            return
        try:tracking_revision = str(UUID(candidate.get('tracking_revision')))
        except (ValueError, TypeError, AttributeError):return
        for key, minimum in (('person_confidence', .4), ('min_keypoint_confidence', .5)):
            confidence = quality.get(key)
            if type(confidence) not in (int, float) or not math.isfinite(confidence) or not minimum <= confidence <= 1:
                return
        key = provider_session, tracking_revision, candidate['track_id']
        self.cooldowns = [(until, old) for until, old in self.cooldowns if until > now][-64:]
        if any(key == old for _, old in self.cooldowns):
            return
        manual = worker.manual_context()
        if manual is not None and manual.manual_drive.busy(manual.gamepad):
            return
        capture_id = hashlib.sha256((provider_session + ':' + tracking_revision
                                    + ':' + captured_at).encode()).hexdigest()
        self.body = dict(observationId=str(uuid4()), monitoringSessionId=self.observer.monitoring_session,
                         jobId=step['jobId'], jobStepId=step['jobStepId'], operationKey=step['operationKey'],
                         detectedAt=captured_at, trackId=candidate['track_id'], trackingRevision=tracking_revision,
                         candidateReason=candidate['reason'], continuousDurationSec=min(duration,30.), captureId=capture_id)
        self.box = box
        self.key = key
        self.latest = None
        self.reporting = self.incident_reported = False
        self.fresh_after = math.inf
        self.task = asyncio.create_task(self.run(worker, dict(self.body)))

    def report_id(self, result, captured_mono):
        person = result.get('person')
        person_box = bbox(person.get('bbox')) if isinstance(person, dict) else None
        match = self.matched(result)
        if (self.body is None or captured_mono < self.fresh_after
                or self.observer.camera_moving() or match is None or person_box is None
                or overlap(person_box, match['bbox']) < .5
                or person.get('track_id') is not None and person['track_id'] != match.get('track_id')):
            return None
        self.reporting = True
        return self.body['observationId']

    def report_finished(self, observation_id, success):
        if self.body is not None and self.body['observationId'] == observation_id:
            self.reporting = False
            self.incident_reported = success

    async def run(self, worker, body):
        path = '/api/v1/pi/fall-observations/' + body['observationId']
        deadline = time.monotonic() + 20.
        stop_seen = observe_started = None
        completed = False
        try:
            # Same id on retry: a lost HTTP response must never acquire two holds.
            for attempt in range(2):
                try:
                    value = await self.request('/api/v1/pi/fall-observations', body)
                    break
                except Exception:
                    if attempt:raise
                    await asyncio.sleep(.5)
            logging.info('FALL_OBSERVATION_REQUESTED observation=%s step=%s', body['observationId'], body['jobStepId'])
            while time.monotonic() < deadline:
                if value.get('state') in TERMINAL or self.incident_reported:
                    completed = True
                    return
                now = time.monotonic()
                if value.get('stopConfirmedAt') is not None:
                    if stop_seen is None:
                        stop_seen = now
                        self.fresh_after = now + .5
                    active = worker.active
                    manual = worker.manual_context()
                    preempted = (active is not None and active.get('jobStepId') != body['jobStepId']
                                 or manual is not None and manual.manual_drive.busy(manual.gamepad))
                    if preempted or worker.shutting_down or worker.session_fault:
                        await self.request(path + '/complete', {'outcome': 'INCONCLUSIVE'})
                        completed = True
                        return
                    moving = self.observer.camera_moving()
                    if moving and observe_started is not None:
                        # The verification window must be continuously stationary;
                        # new motion never extends the BE's bounded lease.
                        observe_started = None
                        self.fresh_after = now + .5
                    if now >= self.fresh_after and not moving and observe_started is None:
                        observe_started = now
                        logging.info('FALL_OBSERVATION_STOPPED observation=%s', body['observationId'])
                    if observe_started is not None and now-observe_started >= 3.:
                        # A full fall event may be in flight. Its atomic BE commit
                        # replaces this hold with INCIDENT before releasing it.
                        if self.reporting:
                            await asyncio.sleep(.25)
                            continue
                        latest = self.latest
                        clear = latest is not None and now-latest[0] <= 1.5 and latest[1] in (None, 'PERSON')
                        outcome = 'CLEAR' if clear else 'INCONCLUSIVE'
                        await self.request(path + '/complete', {'outcome': outcome})
                        logging.info('FALL_OBSERVATION_COMPLETED observation=%s outcome=%s', body['observationId'], outcome)
                        completed = True
                        return
                await asyncio.sleep(.5)
                value = await self.request(path)
        except asyncio.CancelledError:
            raise
        except Exception as error:
            # Leave cleanup to BE's durable expiry; never resume locally.
            logging.info('FALL_OBSERVATION_UNAVAILABLE code=%s', type(error).__name__)
        finally:
            now = time.monotonic()
            if self.key is not None:self.cooldowns.append((now+20., self.key))
            self.next_candidate = now + (10. if completed else 15.)
            self.body = self.box = self.latest = None
            self.key = None
            self.fresh_after = math.inf
            self.task = None

    async def close(self):
        task = self.task
        if task is not None:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
