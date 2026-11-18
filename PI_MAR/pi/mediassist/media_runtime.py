"""Small tmpfs audio reservation shared by the managed media and voice processes."""
import json
import math
import os
from pathlib import Path
import stat
import tempfile
import time
from datetime import datetime, timezone
from contextlib import contextmanager

OVERLAY_TOPIC = '/mediassist/ai/overlay'
OVERLAY_DATA_TOPIC = 'mar.ai.overlay.v1'
OVERLAY_MAX_AGE_SEC = 3.


def overlay_value(value, now=None):
    """Bound optional display metadata; it never authorizes motion or an alert."""
    if not isinstance(value,dict) or value.get('status') not in ('OK','PAUSED','DEGRADED'):
        raise ValueError('INVALID_OVERLAY_STATUS')
    stamp=value.get('captured_at')
    if not isinstance(stamp,str) or not 16<=len(stamp)<=40:raise ValueError('INVALID_OVERLAY_TIME')
    captured=datetime.fromisoformat(stamp.replace('Z','+00:00'))
    if captured.utcoffset() is None:raise ValueError('INVALID_OVERLAY_TIME')
    age=(time.time() if now is None else now)-captured.timestamp()
    if not -.5<=age<=OVERLAY_MAX_AGE_SEC:raise ValueError('STALE_OVERLAY')
    width,height=value.get('width'),value.get('height')
    if any(type(n) is not int or not 32<=n<=640 for n in (width,height)):raise ValueError('INVALID_OVERLAY_SIZE')
    raw=value.get('detections')
    if not isinstance(raw,list) or len(raw)>8:raise ValueError('INVALID_OVERLAY_DETECTIONS')
    boxes=[]
    for detection in raw:
        if not isinstance(detection,dict):raise ValueError('INVALID_OVERLAY_BOX')
        box=detection.get('bbox');confidence=detection.get('confidence');track=detection.get('track_id')
        if (not isinstance(box,list) or len(box)!=4
                or any(type(n) not in (int,float) or not math.isfinite(n) or not 0<=n<=1 for n in box)
                or box[2]<=0 or box[3]<=0 or box[0]+box[2]>1.000001 or box[1]+box[3]>1.000001
                or type(confidence) not in (int,float) or not math.isfinite(confidence) or not 0<=confidence<=1
                or track is not None and (type(track) is not int or not 0<=track<=1_000_000_000)
                or detection.get('state') not in ('PERSON','OBSERVING','SUSPECTED_FALL','PERSON_ON_FLOOR')):
            raise ValueError('INVALID_OVERLAY_BOX')
        # Round endpoints, then derive size. Rounding x and width separately
        # can turn a valid edge box into x+width=1.0001, rejected on the next hop.
        left,top=round(box[0],4),round(box[1],4)
        right,bottom=round(min(1.,box[0]+box[2]),4),round(min(1.,box[1]+box[3]),4)
        if right<=left or bottom<=top:continue
        boxes.append(dict(track_id=track,bbox=[left,top,round(right-left,4),round(bottom-top,4)],
                          confidence=round(confidence,3),state=detection['state']))
    return dict(captured_at=stamp,width=width,height=height,status=value['status'],
                detections=boxes if value['status']=='OK' else [])


def overlay_packet(value, session_id, sequence, now=None):
    """Stay below the lossy data-channel MTU; keep the most relevant boxes."""
    clean=overlay_value(value,now)
    ranking={'SUSPECTED_FALL':0,'PERSON_ON_FLOOR':1,'OBSERVING':2,'PERSON':3}
    clean['detections'].sort(key=lambda d:(ranking[d['state']],-d['confidence']))
    packet=dict(v=1,media_session_id=session_id,seq=sequence,**clean)
    while True:
        encoded=json.dumps(packet,separators=(',',':'),allow_nan=False).encode()
        if len(encoded)<=1200:return encoded
        if not packet['detections']:raise ValueError('OVERLAY_TOO_LARGE')
        packet['detections'].pop()


def use_media_packages():
    """Keep LiveKit's dependencies out of the vendor ROS Python environment."""
    import sys
    packages = Path(__file__).resolve().parents[1] / '.venv' / 'media-site-packages'
    if packages.is_dir():
        sys.path.insert(0, str(packages))


def state_path():
    return Path('/dev/shm')/('mar-media-'+str(os.getuid())+'.json')


@contextmanager
def bridge_lock():
    import fcntl
    path=Path('/dev/shm')/('mar-media-worker-'+str(os.getuid())+'.lock')
    fd=os.open(path,os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
    try:
        info=os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid():raise RuntimeError('MEDIA_LOCK_OWNER_MISMATCH')
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('MEDIA_ALREADY_RUNNING') from None
        yield
    finally:os.close(fd)


def audio_reserved():
    if os.name!='posix':return False
    path=state_path()
    try:
        info=path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid() or info.st_size>2048:return False
        value=json.loads(path.read_text())
        if value.get('mode')!='ASSIST' or value.get('expiresMonotonic',0)<time.monotonic():return False
        current=Path('/proc',str(int(value['pid'])),'stat').read_text().rsplit(')',1)[1].split()
        return current[0]!='Z' and current[19]==value['startTicks']
    except (OSError,ValueError,TypeError,KeyError,IndexError):return False


class Reservation:
    def __init__(self,session):
        self.path=state_path();self.session=session
        self.ticks=Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19]

    def refresh(self):
        value=dict(pid=os.getpid(),startTicks=self.ticks,mode='ASSIST',sessionId=self.session,
                   expiresMonotonic=time.monotonic()+4.)
        fd,name=tempfile.mkstemp(prefix='.mar-media-',dir=self.path.parent)
        try:
            with os.fdopen(fd,'w') as f:json.dump(value,f)
            os.replace(name,self.path)
        finally:Path(name).unlink(missing_ok=True)

    def close(self):
        try:
            value=json.loads(self.path.read_text())
            if value.get('pid')==os.getpid() and value.get('sessionId')==self.session:self.path.unlink(missing_ok=True)
        except (OSError,ValueError):pass


def configuration(worker):
    from .contracts import StepError
    from urllib.parse import urlsplit
    value=worker.setdefault('media',{})
    for key,default in dict(enabled=False,url='wss://media.mediassistrover.duckdns.org',
                            audio_input='auto',audio_output='auto',force_relay=False).items():value.setdefault(key,default)
    uri=urlsplit(value['url'])
    if type(value['enabled']) is not bool or type(value['force_relay']) is not bool or uri.scheme!='wss' or not uri.hostname or uri.username or uri.password or uri.query or uri.fragment or uri.path not in ('','/'):
        raise StepError('INVALID_MEDIA_CONFIG')
    for key in ('audio_input','audio_output'):
        if not isinstance(value[key],(str,int)) or isinstance(value[key],bool):raise StepError('INVALID_MEDIA_AUDIO_DEVICE')
    return value
