"""Short, ordered mapping velocity leases. Pure Python; no hardware access."""
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker
from .contracts import StepError

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
        if next(VALIDATOR.iter_errors(message), None):
            raise StepError('INVALID_MAPPING_CONTROL')
        if not all(math.isfinite(message[k]) for k in ('linearX','angularZ')):
            raise StepError('INVALID_MAPPING_CONTROL')
        if message['robotId'] != robot_id or message['transportSessionId'] != transport_id:
            raise StepError('MAPPING_TRANSPORT_MISMATCH')
        if message['mappingSessionId'] != state.get('session_id') or message['jobId'] != state.get('job_id') or not state.get('session_open'):
            raise StepError('MAPPING_SESSION_MISMATCH')
        now = now or datetime.now(timezone.utc)
        mono = time.monotonic() if mono is None else mono
        issued = datetime.fromisoformat(message['issuedAt'].replace('Z','+00:00'))
        expires = datetime.fromisoformat(message['expiresAt'].replace('Z','+00:00'))
        if not 0 < (expires-issued).total_seconds() <= .751 or (issued-now).total_seconds() > .1 or expires <= now:
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
