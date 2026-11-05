"""Bounded voice HTTPS client shared by commissioning tools and the worker."""
import array
import hashlib
import io
import json
import math
from pathlib import Path
import ssl
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import wave
from .be_token import NoRedirect,upload_token
from .contracts import StepError

MAX_WAV=4*1024*1024


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
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect(),urllib.request.HTTPSHandler(context=context))

    def request(self,path,data=None,content_type=None,timeout=50,idempotency_key=None):
        for attempt in range(2):
            token=upload_token(self.auth,self.cfg['robot_id'],force=attempt==1)
            headers={'Authorization':'Bearer '+token}
            if content_type:headers['Content-Type']=content_type
            if idempotency_key:headers['Idempotency-Key']=idempotency_key
            req=urllib.request.Request(self.origin+path,data=data,headers=headers)
            try:
                with self.opener.open(req,timeout=timeout) as response:
                    raw=response.read(MAX_WAV+1)
                    if len(raw)>MAX_WAV:raise StepError('VOICE_RESPONSE_TOO_LARGE')
                    return response.status,response.headers,raw
            except urllib.error.HTTPError as error:
                if error.code in (401,403) and attempt==0:continue
                raw=error.read(8192)
                try:
                    value=json.loads(raw);value=value.get('data',value)
                    code=value.get('code') or value.get('error') or 'VOICE_HTTP_'+str(error.code)
                    if str(value.get('message','')).startswith('STT_'):code=value['message'].split(':',1)[0]
                except (ValueError,AttributeError):code='VOICE_HTTP_'+str(error.code)
                raise StepError(str(code),'BE returned HTTP '+str(error.code)) from None

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


