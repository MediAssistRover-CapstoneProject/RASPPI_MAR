"""Small tmpfs audio reservation shared by the managed media and voice processes."""
import json
import os
from pathlib import Path
import stat
import tempfile
import time
from contextlib import contextmanager


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
