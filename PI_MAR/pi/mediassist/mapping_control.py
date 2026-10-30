"""Short, ordered mapping velocity leases. Pure Python; no hardware access."""
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker
from .contracts import StepError, parse_timestamp

MAX_LINEAR = .30
MAX_ANGULAR = 1.0
MAX_PACKET_AGE = 1.501
# Allow modest BE/Pi clock offset without increasing durationMs or replaying
# expired commands. This is a future timestamp tolerance, not a motion lease.
MAX_FUTURE_CLOCK_SKEW_SEC = 1.0

SCHEMA = json.loads((Path(__file__).resolve().parents[2]/'handoff/mapping-live.schema.json').read_text())
VALIDATOR = Draft202012Validator(SCHEMA['oneOf'][0], format_checker=FormatChecker())

class MappingControlLease:
    def __init__(self):
        self.sequence = {}
        self.velocity = (0., 0.)
        self.deadline = 0.

    def stop(self):
        self.velocity = (0., 0.)
        self.deadline = 0.

    def accept(self, message, robot_id, transport_id, state, now=None, mono=None):
        invalid = next(VALIDATOR.iter_errors(message), None)
        if invalid:
            raise StepError('INVALID_MAPPING_CONTROL',
                            'schema field=' + '.'.join(map(str, invalid.path)) + ' rule=' + str(invalid.validator))
        if not all(math.isfinite(message[k]) for k in ('linearX','angularZ')):
            raise StepError('INVALID_MAPPING_CONTROL')
        if message['robotId'] != robot_id or message['transportSessionId'] != transport_id:
            raise StepError('MAPPING_TRANSPORT_MISMATCH')
        if message['mappingSessionId'] != state.get('session_id') or message['jobId'] != state.get('job_id') or not state.get('session_open'):
            raise StepError('MAPPING_SESSION_MISMATCH')
        now = now or datetime.now(timezone.utc)
        mono = time.monotonic() if mono is None else mono
        try:
            issued = parse_timestamp(message['issuedAt'])
            expires = parse_timestamp(message['expiresAt'])
        except ValueError:
            raise StepError('INVALID_MAPPING_CONTROL_TIMESTAMP') from None
        if not 0 < (expires-issued).total_seconds() <= MAX_PACKET_AGE or (issued-now).total_seconds() > MAX_FUTURE_CLOCK_SKEW_SEC or expires <= now:
            raise StepError('MAPPING_CONTROL_EXPIRED_OR_CLOCK_SKEW')
        key = (transport_id,state['session_id'])
        if message['sequenceNo'] <= self.sequence.get(key,0):
            raise StepError('MAPPING_CONTROL_OUT_OF_ORDER')
        x,z = float(message['linearX']),float(message['angularZ'])
        if (x or z) and state.get('state') != 'MAPPING':
            raise StepError('MAPPING_NOT_READY')
        self.sequence = {key:message['sequenceNo']}
        self.velocity = (x,z)
        self.deadline = mono + min(message['durationMs']/1000.,(expires-now).total_seconds()) if (x or z) else 0.

    def current(self, mono=None):
        mono = time.monotonic() if mono is None else mono
        return self.velocity if mono < self.deadline else (0.,0.)


class VelocityRamp:
    """Smooth live nonzero setpoints; an expired/zero command always stops now."""
    def __init__(self, linear_accel=.25, angular_accel=.8):
        self.linear_accel, self.angular_accel = linear_accel, angular_accel
        self.reset()

    def reset(self):
        self.value, self.at = (0., 0.), None

    def update(self, target, now):
        if not any(target):
            self.reset()
            return (0., 0.)
        dt = .05 if self.at is None else max(0., min(.1, now - self.at))
        self.at = now
        def approach(previous, desired, limit):
            return previous + max(-limit, min(limit, desired - previous))
        self.value = (approach(self.value[0], target[0], self.linear_accel * dt),
                      approach(self.value[1], target[1], self.angular_accel * dt))
        return self.value


class HolonomicRamp:
    """Rate limit the translation vector; a reduced clearance limit takes effect now."""
    def __init__(self,linear_accel=.4,angular_accel=1.2):
        self.linear_accel=linear_accel;self.angular_accel=angular_accel;self.reset()
    def reset(self):self.value=(0.,0.,0.);self.at=None
    def update(self,target,now):
        if not any(target):self.reset();return self.value
        dt=.05 if self.at is None else max(0.,min(.1,now-self.at));self.at=now
        # Never coast through a new obstacle limit while decelerating.
        previous=tuple(min(max(v,0.),t) if t>=0 else max(min(v,0.),t) for v,t in zip(self.value,target))
        dx,dy=target[0]-previous[0],target[1]-previous[1]
        scale=min(1.,self.linear_accel*dt/max(__import__('math').hypot(dx,dy),1e-9))
        z=previous[2]+max(-self.angular_accel*dt,min(self.angular_accel*dt,target[2]-previous[2]))
        self.value=(previous[0]+dx*scale,previous[1]+dy*scale,z);return self.value
