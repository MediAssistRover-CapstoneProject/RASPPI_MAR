"""Local mailbox for transport-validated MQTT authorization, not pose evidence."""
import json,os,time
from pathlib import Path
from .contracts import parse_timestamp

MAX_AGE_SEC=15.

def mailbox(cfg):
    status=cfg.get('localization',{}).get('status_file')
    return Path(status).with_name('mqtt-authorization.json') if status and cfg.get('navigation_session') else None

def receive(cfg,descriptor):
    destination=mailbox(cfg)
    if destination is None:return
    if not isinstance(descriptor,dict):raise ValueError('Invalid session authorization')
    observed=parse_timestamp(descriptor['observedAt']).timestamp()
    age=time.time()-observed
    if not -5.<=age<=MAX_AGE_SEC:raise ValueError('Stale session authorization')
    try:
        previous=json.loads(destination.read_text(encoding='utf-8'))
        if parse_timestamp(previous['descriptor']['observedAt']).timestamp()>=observed:return
    except (OSError,ValueError,TypeError,KeyError):pass
    # Called only after MqttSession checked the robot and current transport.
    # Preserve a CLOSED/invalid descriptor too: the reader must revoke, not
    # continue using a previously good grant after an explicit revocation.
    # A delayed packet only carries its remaining lifetime, not a new 15 s.
    # A small wall-clock offset is tolerated; expiry still uses monotonic time.
    value={'received_monotonic':time.monotonic()-max(0.,age),'descriptor':descriptor}
    destination.parent.mkdir(parents=True,exist_ok=True)
    temporary=destination.with_suffix('.new')
    temporary.write_text(json.dumps(value,allow_nan=False),encoding='utf-8')
    temporary.chmod(0o600);os.replace(temporary,destination)

def read(cfg):
    path=mailbox(cfg)
    if path is None:return None
    try:
        if path.stat().st_size>65536:return None
        value=json.loads(path.read_text(encoding='utf-8'))
        at=value['received_monotonic'];age=time.monotonic()-at
        if type(at) not in (float,int) or not 0<=age<=MAX_AGE_SEC:return None
        return at,value['descriptor']
    except (OSError,ValueError,TypeError,KeyError):return None
