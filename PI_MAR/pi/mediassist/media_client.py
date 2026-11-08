"""HTTPS descriptors/SSE only. Tokens and audio/video never go through MQTT/SQLite."""
import asyncio
import json
import logging
import threading
import urllib.error
import urllib.request
from .be_token import upload_token
from .voice_client import VoiceClient


class MediaClient:
    def __init__(self,worker):
        self.worker=worker;self.http=VoiceClient(worker);self.stop=threading.Event();self.response=None

    def request(self,path,body=None):
        data=json.dumps(body).encode() if body is not None else None
        status,_,raw=self.http.request(path,data,'application/json' if data else None,timeout=5)
        if status!=200:raise RuntimeError('MEDIA_HTTP_'+str(status))
        if len(raw)>65536:raise RuntimeError('MEDIA_RESPONSE_TOO_LARGE')
        return json.loads(raw)

    async def current(self):
        return await asyncio.to_thread(self.request,'/api/v1/pi/media-session')

    async def credentials(self,session):
        return await asyncio.to_thread(self.request,'/api/v1/pi/media-sessions/'+session+'/join-token',{})

    async def report(self,session,state,reason=None):
        return await asyncio.to_thread(self.request,'/api/v1/pi/media-sessions/'+session+'/state',
                                       dict(state=state,**({'reason':reason} if reason else {})))

    def listen(self,loop,queue):
        def notify():
            if queue.full():queue.get_nowait()
            queue.put_nowait(True)
        delay=1.
        while not self.stop.is_set():
            try:
                token=upload_token(self.worker['mapping'],self.worker['robot_id'])
                req=urllib.request.Request(self.http.origin+'/api/v1/pi/media-session/events',headers={
                    'Authorization':'Bearer '+token,'Accept':'text/event-stream'})
                with self.http.opener.open(req,timeout=20) as response:
                    self.response=response
                    if not response.headers.get('Content-Type','').startswith('text/event-stream'):
                        raise RuntimeError('MEDIA_EVENT_TYPE_INVALID')
                    delay=1.
                    while not self.stop.is_set():
                        line=response.readline(8193)
                        if not line:break
                        if len(line)>8192:raise RuntimeError('MEDIA_EVENT_TOO_LARGE')
                        if line.startswith(b'data:'):loop.call_soon_threadsafe(notify)
            except urllib.error.HTTPError as e:
                if e.code in (401,403):
                    try:upload_token(self.worker['mapping'],self.worker['robot_id'],force=True)
                    except Exception:pass
            except Exception as e:
                if not self.stop.is_set():logging.warning('MEDIA_EVENTS_RETRY code=%s',getattr(e,'code',type(e).__name__))
            finally:self.response=None
            self.stop.wait(delay);delay=min(15.,delay*2)

    def close(self):
        self.stop.set()
        if self.response is not None:self.response.close()
