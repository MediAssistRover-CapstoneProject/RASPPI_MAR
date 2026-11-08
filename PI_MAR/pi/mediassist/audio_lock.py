"""Cooperative ALSA ownership across the worker and the explicit chat client."""
import asyncio
import os
import stat
from .contracts import StepError


class AudioLock:
    def __init__(self,priority=False):
        self.local=asyncio.Lock()
        self.fd=None
        self.priority=priority

    async def __aenter__(self):
        await self.local.acquire()
        try:
            from .media_runtime import audio_reserved
            if not self.priority and audio_reserved():raise StepError('AUDIO_RESERVED_FOR_SOS')
            if os.name=='posix':
                import fcntl
                path='/tmp/mar-audio-'+str(os.getuid())+'.lock'
                self.fd=os.open(path,os.O_RDWR|os.O_CREAT|os.O_CLOEXEC|os.O_NOFOLLOW,0o600)
                info=os.fstat(self.fd)
                if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid():
                    raise StepError('AUDIO_LOCK_OWNER_MISMATCH')
                while True:
                    try:fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB);break
                    except BlockingIOError:
                        if not self.priority and audio_reserved():raise StepError('AUDIO_RESERVED_FOR_SOS')
                        await asyncio.sleep(.05)
            return self
        except BaseException:
            if self.fd is not None:os.close(self.fd);self.fd=None
            self.local.release()
            raise

    async def __aexit__(self,*args):
        try:
            if self.fd is not None:
                import fcntl
                fcntl.flock(self.fd,fcntl.LOCK_UN);os.close(self.fd);self.fd=None
        finally:self.local.release()
