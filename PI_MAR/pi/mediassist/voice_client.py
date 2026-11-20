"""Bounded voice HTTPS client shared by commissioning tools and the worker."""
import array
import hashlib
import http.client
import io
import json
import logging
import math
from pathlib import Path
import ssl
import socket
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import wave
from .be_token import NoRedirect,upload_token
from .contracts import StepError

MAX_WAV=4*1024*1024
NAVIGATION_REASONS=frozenset({
    'VOICE_NAVIGATION_SESSION_NOT_READY','NAVIGATION_SESSION_NOT_LOCALIZED',
    'NAVIGATION_SESSION_CLOSED_OR_CHANGED','NAVIGATION_SESSION_CLOSED_OR_EXPIRED',
    'NAVIGATION_SESSION_MAP_OR_ID_MISMATCH','NAVIGATION_SESSION_PROCESS_CHANGED'})


class _VoiceResponse:
    """Release a keepalive connection only after its complete response was read."""
    def __init__(self, pool, connection, response):
        self.pool, self.connection, self.response = pool, connection, response
        self.status, self.headers = response.status, response.headers
        self.closed = False

    def read(self, amount=None):
        return self.response.read(amount)

    def readline(self, limit=-1):
        # MediaClient shares this authenticated transport for bounded SSE lines.
        return self.response.readline(limit)

    def getcode(self):
        return self.status

    def close(self):
        if self.closed:
            return
        self.closed = True
        reusable = self.response.isclosed() and not self.response.will_close
        self.response.close()
        self.pool.release(self.connection, reusable)

    def abort(self):
        # Interrupt a pending read on cancellation; never return a partial stream
        # to the pool or drain it while the workflow waits for the microphone.
        if self.closed:
            return
        self.closed = True
        if self.connection.sock is not None:
            try:
                self.connection.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.connection.close()
        self.response.close()
        self.pool.release(self.connection, False)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class VoiceHTTPS:
    """Small origin-bound stdlib HTTPS pool. Failed POSTs are never replayed.

    Two connections allow ordered history uploads alongside speech without
    opening an unbounded socket pool. No redirects or environment proxies.
    """
    def __init__(self, origin, context, capacity=2):
        self.origin = origin
        self.uri = urllib.parse.urlsplit(origin)
        self.context = context
        self.capacity = capacity
        self.idle = []
        self.active = set()
        self.leased = 0
        self.condition = threading.Condition()
        self.closed = False

    def acquire(self, timeout):
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.leased >= self.capacity and not self.closed:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('Voice HTTPS pool wait timed out')
                self.condition.wait(remaining)
            if self.closed:
                raise OSError('Voice HTTPS client closed')
            self.leased += 1
            while self.idle:
                connection, returned = self.idle.pop()
                if time.monotonic()-returned < 30:
                    connection._voice_cancelled=False
                    self.active.add(connection)
                    return connection
                connection.close()
            connection=http.client.HTTPSConnection(self.uri.hostname, self.uri.port,
                                                    context=self.context, timeout=timeout)
            connection._voice_cancelled=False
            self.active.add(connection)
            return connection

    def release(self, connection, reusable):
        with self.condition:
            self.leased -= 1
            self.active.discard(connection)
            if reusable and not self.closed and not connection._voice_cancelled:
                self.idle.append((connection, time.monotonic()))
            else:
                connection.close()
            self.condition.notify()

    def open(self, request, timeout=25):
        uri = urllib.parse.urlsplit(request.full_url)
        if uri.scheme+'://'+uri.netloc != self.origin or uri.fragment:
            raise StepError('VOICE_HTTPS_ORIGIN_REQUIRED')
        connection = self.acquire(timeout)
        try:
            connection.timeout = timeout
            if connection.sock is not None:
                connection.sock.settimeout(timeout)
            headers = dict(request.header_items())
            headers['Connection'] = 'keep-alive'
            path = urllib.parse.urlunsplit(('', '', uri.path or '/', uri.query, ''))
            connection.request(request.get_method(), path, body=request.data, headers=headers)
            if connection._voice_cancelled:
                raise OSError('Voice HTTPS request cancelled')
            response = connection.getresponse()
            if connection._voice_cancelled:
                response.close()
                raise OSError('Voice HTTPS request cancelled')
            if response.status >= 400 or 300 <= response.status < 400 and response.status != 304:
                raw = response.read(8193)
                status, reason, response_headers = response.status, response.reason, response.headers
                response.close()
                self.release(connection, False)
                connection = None
                raise urllib.error.HTTPError(request.full_url, status, reason, response_headers, io.BytesIO(raw))
            return _VoiceResponse(self, connection, response)
        except BaseException:
            if connection is not None:
                self.release(connection, False)
            raise

    def abort_pending(self):
        with self.condition:
            for connection in self.active:
                connection._voice_cancelled=True
                if connection.sock is not None:
                    try:connection.sock.shutdown(socket.SHUT_RDWR)
                    except OSError:pass
                connection.close()

    def close(self):
        with self.condition:
            self.closed = True
            for connection, _ in self.idle:
                connection.close()
            self.idle.clear()
            self.condition.notify_all()
        self.abort_pending()


def describe_audio(raw):
    if len(raw)>MAX_WAV:raise StepError('VOICE_AUDIO_TOO_LARGE')
    try:
        with wave.open(io.BytesIO(raw),'rb') as wav:
            channels,width,rate,frames=wav.getnchannels(),wav.getsampwidth(),wav.getframerate(),wav.getnframes()
            if wav.getcomptype()!='NONE' or width!=2 or channels!=1 or not 8000<=rate<=48000:raise ValueError()
            pcm=wav.readframes(frames)
            # Some TTS providers return RIFF streaming headers with unknown
            # length (0xffffffff). The bounded downloaded body owns its length.
            if frames>=0x7fffffff:frames=len(pcm)//(channels*width)
            if len(pcm)!=frames*channels*width or frames<=0:raise ValueError()
    except (wave.Error,EOFError,ValueError):raise StepError('VOICE_INVALID_WAV') from None
    samples=array.array('h',pcm)
    if sys.byteorder!='little':samples.byteswap()
    rms=math.sqrt(sum((x/32768.)**2 for x in samples)/len(samples))
    return dict(channels=channels,bits=width*8,sample_rate=rate,duration_sec=round(frames/rate,3),
                rms=round(rms,5),peak=round(max(abs(x) for x in samples)/32768.,5))


class VoiceClient:
    def __init__(self,cfg,verbose=False):
        self.verbose=verbose
        self.cfg=cfg;self.auth=cfg['mapping'];uri=urllib.parse.urlsplit(self.auth['upload_base_url'])
        if uri.scheme!='https' or not uri.hostname or uri.username or uri.password or uri.query or uri.fragment:
            raise StepError('VOICE_HTTPS_ORIGIN_REQUIRED')
        self.origin=uri.scheme+'://'+uri.netloc
        context=ssl.create_default_context(cafile=self.auth.get('upload_ca_file') or None)
        context.minimum_version=ssl.TLSVersion.TLSv1_2
        self.opener=VoiceHTTPS(self.origin,context)
        self.profile=None

    def close(self):
        self.opener.close()

    def speech_profile(self):
        headers = {'If-None-Match': '"'+self.profile['revision']+'"'} if self.profile else None
        status, _, raw = self.request('/api/v1/pi/speech/profile',timeout=8,headers=headers)
        if status != 304:
            value = self.value(raw)
            if (not isinstance(value, dict) or not isinstance(value.get('revision'), str)
                    or not isinstance(value.get('voice'), str)
                    or not isinstance(value.get('instructions'), str)
                    or type(value.get('speedMultiplier')) not in (int, float)):
                raise StepError('VOICE_SPEECH_PROFILE_INVALID')
            self.profile=value
        if self.profile is None:
            raise StepError('VOICE_SPEECH_PROFILE_INVALID')
        return dict(self.profile)

    def request(self,path,data=None,content_type=None,timeout=50,idempotency_key=None,headers=None):
        request_id=str(uuid.uuid4());started=time.monotonic();extra_headers=headers or {}
        for attempt in range(2):
            token=upload_token(self.auth,self.cfg['robot_id'],force=attempt==1)
            request_headers={'Authorization':'Bearer '+token,'X-Request-ID':request_id,
                             'X-Correlation-ID':request_id,**extra_headers}
            if content_type:request_headers['Content-Type']=content_type
            if idempotency_key:request_headers['Idempotency-Key']=idempotency_key
            req=urllib.request.Request(self.origin+path,data=data,headers=request_headers)
            try:
                with self.opener.open(req,timeout=timeout) as response:
                    raw=response.read(MAX_WAV+1)
                    if len(raw)>MAX_WAV:raise StepError('VOICE_RESPONSE_TOO_LARGE')
                    logging.info('VOICE_BE_HTTP request_id=%s path=%s status=%d roundtrip_ms=%d',
                                 request_id,path.split('?',1)[0],response.status,
                                 round((time.monotonic()-started)*1000))
                    return response.status,response.headers,raw
            except urllib.error.HTTPError as error:
                if error.code in (401,403) and attempt==0:continue
                raw=error.read(8192)
                try:
                    value=json.loads(raw);value=value.get('data',value)
                    code=value.get('code') or value.get('error') or 'VOICE_HTTP_'+str(error.code)
                    if str(value.get('message','')).startswith('STT_'):code=value['message'].split(':',1)[0]
                    reason=str(value.get('message','')).split(':',1)[0]
                    if (error.code==422 and path.startswith('/api/v1/pi/voice/navigation-')
                            and reason in NAVIGATION_REASONS):code=reason
                except (ValueError,AttributeError):code='VOICE_HTTP_'+str(error.code)
                raise StepError(str(code),'BE returned HTTP '+str(error.code),retryable=error.code>=500) from None

    @staticmethod
    def value(raw):
        value=json.loads(raw)
        return value.get('data',value) if isinstance(value,dict) else value

    def status(self):
        _,_,raw=self.request('/api/v1/pi/speech/status',timeout=15)
        return self.value(raw)

    def transcribe_result(self,raw,language):
        if language not in ('vi','en','auto'):raise StepError('VOICE_LANGUAGE_UNSUPPORTED')
        info=describe_audio(raw)
        if info['sample_rate']!=16000 or not .2<=info['duration_sec']<=20:
            raise StepError('VOICE_STT_REQUIRES_PCM16_MONO_16000HZ_MAX_20_SECONDS')
        if info['peak']==0.:raise StepError('VOICE_CAPTURE_IS_SILENT')
        boundary='mar-'+uuid.uuid4().hex
        data=(('--'+boundary+'\r\nContent-Disposition: form-data; name="file"; filename="capture.wav"\r\nContent-Type: audio/wav\r\n\r\n').encode()+raw+
              ('\r\n--'+boundary+'--\r\n').encode())
        start=time.monotonic()
        _,_,result=self.request('/api/v1/pi/speech/transcribe?languageCode='+language,data,'multipart/form-data; boundary='+boundary)
        value=self.value(result)
        if value.get('state')!='READY' or not value.get('text'):raise StepError('VOICE_STT_RESPONSE_INVALID')
        value['pi_request_ms']=round((time.monotonic()-start)*1000)
        if self.verbose:print('STT',json.dumps(value,ensure_ascii=False),flush=True)
        return value

    def transcribe(self,raw,language):
        return self.transcribe_result(raw,language)['text']

    def synthesize(self,text,language):
        if not 1<=len(text)<=500:raise StepError('VOICE_TTS_TEXT_REQUIRES_1_TO_500_CHARACTERS')
        body=json.dumps(dict(text=text,languageCode=language,speedWpm=150),ensure_ascii=False).encode()
        start=time.monotonic();deadline=start+75
        while True:
            left=deadline-time.monotonic()
            if left<=0:raise StepError('VOICE_TTS_WAIT_TIMEOUT')
            status,headers,raw=self.request('/api/v1/pi/speech/synthesize',body,'application/json',timeout=min(20,left))
            if status==200:
                if not headers.get('Content-Type','').lower().startswith('audio/wav'):raise StepError('VOICE_TTS_RESPONSE_INVALID')
                expected=headers.get('X-Audio-SHA256')
                if expected and hashlib.sha256(raw).hexdigest()!=expected:raise StepError('VOICE_TTS_HASH_MISMATCH')
                info=describe_audio(raw)
                if self.verbose:print('TTS',json.dumps({**info,'pi_request_ms':round((time.monotonic()-start)*1000)}),flush=True)
                return raw
            if status!=202:raise StepError('VOICE_TTS_NOT_READY')
            try:wait=max(.2,min(2.,float(headers.get('Retry-After','1'))))
            except ValueError:wait=1.
            time.sleep(min(wait,max(0.,deadline-time.monotonic())))


