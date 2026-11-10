"""Opt-in robot voice conversation. User PCM/WAV stays in RAM; BE stores text history."""
import asyncio
import array
import io
import functools
import json
import logging
import math
import os
from pathlib import Path
import shutil
import ssl
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import wave
from contextlib import contextmanager
from .audio_lock import AudioLock
from .be_token import NoRedirect
from .contracts import StepError
from .speech import SpeechPlayer,audio_device
from .store import utcnow
from .voice_capture import recording_cue
from .voice_client import VoiceClient
from .voice_actions import VoiceActions,words

INTRO='Xin chào, bạn cần MAR giúp gì không?'
GOODBYE='Phiên trò chuyện đã kết thúc. Khi cần hỗ trợ, hãy gọi MAR.'
HISTORY_LOCK=threading.RLock()


def goodbye(text):
    return words(text) in ('tam biet','ket thuc','ket thuc hoi thoai','dung tro chuyen',
        'khong can nua','thoi khong can nua','goodbye','bye','end conversation','stop chatting','no thank you')


def serialized(method):
    @functools.wraps(method)
    def call(self,*args,**kwargs):
        with self.lock:return method(self,*args,**kwargs)
    return call


class AiClient:
    def __init__(self,path):
        path=Path(path);self.config=json.loads(path.read_text())
        self.marker=path.parent/'voice-chat-session.json'
        uri=urllib.parse.urlsplit(self.config['base_url'])
        if uri.scheme!='https' or not uri.hostname or uri.username or uri.password or uri.query or uri.fragment or uri.path not in ('','/'):
            raise StepError('VOICE_AI_HTTPS_REQUIRED')
        self.base=uri.scheme+'://'+uri.netloc
        self.token=Path(self.config['token_file']).read_text().strip()
        if len(self.token)<20 or any(c.isspace() for c in self.token):raise StepError('VOICE_AI_TOKEN_INVALID')
        context=ssl.create_default_context(cafile=self.config['ca_file']);context.minimum_version=ssl.TLSVersion.TLSv1_2
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect(),urllib.request.HTTPSHandler(context=context))

    def request(self,path,body=None):
        req=urllib.request.Request(self.base+path,data=None if body is None else json.dumps(body,ensure_ascii=False).encode(),
            headers={'Authorization':'Bearer '+self.token,'Content-Type':'application/json'})
        try:
            with self.opener.open(req,timeout=25) as r:raw=r.read(65537)
        except urllib.error.HTTPError as e:
            raise StepError('VOICE_AI_HTTP_'+str(e.code)) from None
        except OSError:raise StepError('VOICE_AI_UNREACHABLE') from None
        if len(raw)>65536:raise StepError('VOICE_AI_RESPONSE_TOO_LARGE')
        result=json.loads(raw)
        if not isinstance(result,dict):raise StepError('VOICE_AI_INVALID_RESPONSE')
        return result

    def ask(self,session_id,text,history,destinations=None,hospital_information_id=None):
        turn=str(uuid.uuid4())
        body=dict(session_id=session_id,turn_id=turn,text=text,language='vi',history=history[-8:],destinations=destinations or [])
        if hospital_information_id:body['hospital_information_id']=hospital_information_id
        result=self.request('/api/robot/chat',body)
        if (result.get('session_id')!=session_id or result.get('turn_id')!=turn
                or not isinstance(result.get('answer'),str) or not 1<=len(result['answer'])<=2000
                or not isinstance(result.get('speech_text'),str) or not 1<=len(result['speech_text'])<=500):
            raise StepError('VOICE_AI_INVALID_RESPONSE')
        return result

    def detect_wake(self,pcm):
        req=urllib.request.Request(self.base+'/api/robot/wake',data=pcm,headers={
            'Authorization':'Bearer '+self.token,'Content-Type':'application/octet-stream',
            'X-Audio-Format':'pcm_s16le_16000_mono'})
        try:
            with self.opener.open(req,timeout=8) as response:raw=response.read(4097)
        except urllib.error.HTTPError as error:raise StepError('VOICE_WAKE_HTTP_'+str(error.code)) from None
        except OSError:raise StepError('VOICE_AI_UNREACHABLE') from None
        if len(raw)>4096:raise StepError('VOICE_AI_RESPONSE_TOO_LARGE')
        value=json.loads(raw)
        if not isinstance(value,dict) or type(value.get('detected')) is not bool:raise StepError('VOICE_WAKE_INVALID_RESPONSE')
        if value.get('phrase')!=self.config.get('wake',{}).get('phrase','M A'):
            raise StepError('VOICE_WAKE_PHRASE_MISMATCH')
        return value['detected']


class History:
    """Only session IDs/counters are retained locally, never user audio or transcript."""
    def __init__(self,be,marker):
        self.be=be;self.marker=marker;self.state=None;self.lock=HISTORY_LOCK

    def save(self):
        fd,temp=tempfile.mkstemp(prefix='.voice-session-',dir=self.marker.parent)
        try:
            with os.fdopen(fd,'w') as stream:json.dump(self.state,stream);stream.flush();os.fsync(stream.fileno())
            os.replace(temp,self.marker)
        finally:
            if os.path.exists(temp):os.unlink(temp)

    def envelope(self,kind,payload,message_id=None):
        self.state['sequence']+=1;self.save();message=message_id or str(uuid.uuid4())
        return dict(protocolVersion='1.0',messageId=message,operationKey='voice-'+message,correlationId=self.state['id'],
            robotId=self.be.cfg['robot_id'],producerSessionId=self.state['producer'],messageType=kind,
            occurredAt=utcnow(),sequenceNo=self.state['sequence'],payload=payload)

    def post(self,path,envelope):
        raw=json.dumps(envelope,ensure_ascii=False).encode()
        # Reuse the exact message identity on an ambiguous network response.
        for attempt in range(2):
            try:
                _,_,body=self.be.request(path,raw,'application/json',timeout=8)
                value=self.be.value(body)
                if value.get('accepted') is not True:raise StepError('VOICE_HISTORY_ACK_REJECTED')
                return
            except (OSError,urllib.error.URLError):
                if attempt:raise

    @serialized
    def recover(self):
        if not self.marker.exists():return
        self.state=json.loads(self.marker.read_text())
        try:self.close('FAILED','Interrupted before normal conversation shutdown')
        except StepError as exc:
            if 'HTTP 404' not in str(exc):raise
            self.marker.unlink(missing_ok=True);self.state=None

    @serialized
    def start(self):
        self.state=dict(id=str(uuid.uuid4()),producer=str(uuid.uuid4()),sequence=0,commands=0,started_at=utcnow())
        self.save()
        self.post('/api/v1/pi/voice/interactions',self.envelope('VOICE_INTERACTION_STARTED',
            dict(languageCode='vi-VN',startedAt=self.state['started_at']),self.state['id']))
        return self.state['id']

    @serialized
    def record(self,question,reply,result,received_at,status='EXECUTED',request_id=None):
        self.state['commands']+=1
        params=dict(answer_text=reply,sources=result.get('sources',[]),speech_playback_completed=True,
            physical_action_executed=False)
        if result.get('model'):params['ai_model']=result['model']
        if result.get('action'):params['action_proposed']=result['action']
        payload=dict(voiceInteractionId=self.state['id'],serviceRequestId=request_id,sequenceNo=self.state['commands'],
            rawText=question,normalizedText=question.strip().casefold(),intentCode=result.get('intent','UNKNOWN'),
            confidenceScore=max(0.,min(1.,float(result.get('confidence',0.)))),commandType='VOICE_QUERY',parameters=params,
            status=status,receivedAt=received_at,processedAt=utcnow())
        self.post('/api/v1/pi/voice/commands',self.envelope('VOICE_COMMAND',payload))

    @serialized
    def close(self,status='COMPLETED',summary=None):
        if not self.state:return
        sid=self.state['id']
        payload=dict(status=status,endedAt=utcnow(),detectedIntent=None,
            transcriptSummary=summary or 'Robot voice chat; turns='+str(self.state['commands']),audioUri=None)
        # Preserve the exact completion on a lost ACK, including its status.
        # Recovery must not try to change an already COMPLETED row to FAILED.
        event=self.state.get('completion') or self.envelope('VOICE_INTERACTION_COMPLETED',payload)
        self.state['completion']=event;self.save()
        self.post('/api/v1/pi/voice/interactions/'+sid+'/completion',event)
        self.marker.unlink(missing_ok=True);self.state=None


class HistoryWriter:
    """Ordered, bounded text-only uploads; never hold the microphone for a DB ACK."""
    def __init__(self, history, capacity=20):
        self.history = history
        self.queue = asyncio.Queue(maxsize=capacity)
        self.error = None
        self.task = asyncio.create_task(self.run())

    def record(self, *args):
        if self.error:
            raise StepError('VOICE_HISTORY_UNAVAILABLE') from self.error
        try:
            self.queue.put_nowait(args)
        except asyncio.QueueFull:
            raise StepError('VOICE_HISTORY_QUEUE_FULL') from None

    async def run(self):
        while True:
            item = await self.queue.get()
            if item is None:
                return
            if self.error is None:
                try:
                    await asyncio.to_thread(self.history.record, *item)
                except Exception as error:
                    self.error = error
                    logging.warning('VOICE_HISTORY_UPLOAD_FAILED code=%s',
                                    getattr(error, 'code', type(error).__name__))

    async def finish(self):
        await self.queue.put(None)
        await self.task
        if self.error:
            raise self.error


def rms(raw):
    samples=array.array('h',raw)
    if sys.byteorder!='little':samples.byteswap()
    return math.sqrt(sum(x*x for x in samples)/max(1,len(samples)))/32768.


async def stop_process(proc):
    if proc.returncode is None:
        try:proc.terminate()
        except ProcessLookupError:pass
    try:await asyncio.wait_for(proc.communicate(),1)
    except asyncio.TimeoutError:
        try:proc.kill()
        except ProcessLookupError:pass
        await proc.communicate()


async def listen(player,start_timeout=8,max_seconds=15,threshold=.008):
    """Read ALSA raw PCM to bounded RAM, finish after 0.8 s of silence."""
    recorder=shutil.which('arecord');output=shutil.which('aplay')
    if not recorder or not output:raise StepError('VOICE_ALSA_TOOLS_MISSING')
    device=audio_device(player.cfg);block_bytes=960;rate=16000
    async with player.lock:
        with tempfile.TemporaryFile() as diagnostic:
            proc=await asyncio.create_subprocess_exec(recorder,'-q','-D',device,'-t','raw','-f','S16_LE','-r',str(rate),'-c','1',
                stdin=asyncio.subprocess.DEVNULL,stdout=asyncio.subprocess.PIPE,stderr=diagnostic)
            armed=asyncio.Event()

            async def capture():
                blocks=[];lead=[];started=None;speech_at=None;voiced=0;quiet=0
                while True:
                    try:block=await asyncio.wait_for(proc.stdout.readexactly(block_bytes),1.5)
                    except (asyncio.IncompleteReadError,asyncio.TimeoutError):raise StepError('VOICE_MIC_STREAM_FAILED') from None
                    # ALSA is already open during the cue. Discard the robot's
                    # own beep instead of opening the mic after the user starts.
                    if not armed.is_set():continue
                    active=rms(block)>=threshold;now=time.monotonic()
                    if started is None:started=now
                    if speech_at is None:
                        lead.append(block);lead=lead[-10:]
                        if active:speech_at=now;blocks=list(lead);voiced=1
                        elif now-started>=start_timeout:return None
                    else:
                        blocks.append(block);voiced+=int(active);quiet=0 if active else quiet+1
                        if (quiet>=27 and voiced>=12) or now-speech_at>=max_seconds:
                            return b''.join(blocks)[-rate*2*max_seconds:] if voiced>=12 else None

            recording=asyncio.create_task(capture())
            try:
                with tempfile.TemporaryDirectory(prefix='mar-voice-cue-') as folder:
                    cue=Path(folder)/'ready.wav';cue.write_bytes(recording_cue())
                    await player.process([output,'-q','-D',device,str(cue)])
                await asyncio.sleep(.10)
                armed.set()
                print('VOICE_LISTENING: nói sau tiếng bíp.',flush=True)
                pcm=await recording
            finally:
                if not recording.done():recording.cancel()
                await asyncio.gather(recording,return_exceptions=True)
                await stop_process(proc)
    if pcm is None:return None
    raw=io.BytesIO()
    with wave.open(raw,'wb') as w:w.setnchannels(1);w.setsampwidth(2);w.setframerate(rate);w.writeframes(pcm)
    return raw.getvalue()


async def say(player,text,log_transcripts=True):
    if log_transcripts:print('MAR:',text,flush=True)
    await player.speak(dict(action='SPEAK',prompt_key='VOICE_CHAT',data=dict(text=text[:500],language_code='vi',volume=70,speed_wpm=150)))


async def wait_wake(port):
    import serial
    print('VOICE_WAKE_LISTENING:',port,'— wake phrase is configured in WonderEcho firmware.',flush=True)
    with serial.Serial(port,115200,timeout=0) as source:
        source.reset_input_buffer() # discard wake events received during a job
        buffer=b''
        while True:
            buffer=(buffer+source.read(64))[-128:]
            if b'\xaa\x55\x03\x00\xfb' in buffer:return
            await asyncio.sleep(.05)


async def wait_for_wake(worker,ai):
    settings=ai.config.get('wake',{})
    if not isinstance(settings,dict):raise StepError('VOICE_WAKE_CONFIG_INVALID')
    mode=settings.get('mode','serial')
    if mode=='serial':return await wait_wake(ai.config.get('wake_serial','/dev/wonderecho'))
    if mode!='local_ai':raise StepError('VOICE_WAKE_MODE_INVALID')
    from .voice_wake import wait_phrase
    await wait_phrase(SpeechPlayer(worker['speech']),ai,settings)


@contextmanager
def controller_lock():
    """One owner for both serial wake detection and the conversation microphone."""
    if os.name=='posix' and os.geteuid()==0:raise StepError('VOICE_CHAT_RUN_AS_UBUNTU')
    import fcntl
    fd=os.open('/tmp/mar-voice-chat-'+str(os.getuid())+'.lock',os.O_CREAT|os.O_RDWR|os.O_CLOEXEC|os.O_NOFOLLOW,0o600)
    try:
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise StepError('VOICE_CHAT_ALREADY_RUNNING') from None
        yield
    finally:
        fcntl.flock(fd,fcntl.LOCK_UN);os.close(fd)


async def run_chat(worker,config_path,start_now=False,once=False,max_turns=10):
    with controller_lock():
        return await conversation(worker,config_path,start_now,once,max_turns)


async def conversation(worker,config_path,start_now=True,once=True,max_turns=10,
                       initial_catalog=None,before_dispatch=None,log_transcripts=True,start_timeout=8):
    """Called only while the CLI or managed voice daemon owns controller_lock."""
    ai=AiClient(config_path);be=VoiceClient(worker);player=SpeechPlayer(worker['speech']);history=History(be,ai.marker)
    actions=VoiceActions(be,ai.config,ai.marker,before_dispatch)
    stt_language=ai.config.get('stt_language','auto')
    if stt_language not in ('vi','en','auto'):raise StepError('VOICE_LANGUAGE_UNSUPPORTED')
    # History is independent of MQTT and motor execution.
    health=await asyncio.to_thread(ai.request,'/health')
    if health.get('state')!='READY':raise StepError('VOICE_AI_NOT_READY')
    await asyncio.to_thread(history.recover)
    while True:
        if not start_now:await wait_for_wake(worker,ai)
        start_now=False
        actions.pending=None
        sid=await asyncio.to_thread(history.start);print('VOICE_SESSION:',sid,flush=True)
        status='COMPLETED';turns=[];dispatched=False;ended=False
        writer=HistoryWriter(history,max_turns);confirmation_retries=0
        try:
            catalog=initial_catalog if initial_catalog is not None else await asyncio.to_thread(actions.conversation_catalog)
            initial_catalog=None
            candidates=catalog.get('endpoints',[])[:100]
            hospital_information_id=catalog.get('navigationSession',{}).get('hospitalInformationId')
            await say(player,INTRO,log_transcripts)
            for _ in range(max_turns):
                confirming=actions.pending is not None
                timeout=worker.get('voice_chat',{}).get('confirmation_timeout_sec',20) if confirming else start_timeout
                raw=await listen(player,start_timeout=timeout)
                if raw is None:
                    if confirming and confirmation_retries<1:
                        confirmation_retries+=1
                        logging.info('VOICE_CONFIRMATION_RETRY: no speech captured')
                        await say(player,'Tôi chưa nghe rõ xác nhận. Sau tiếng bíp, bạn nói xác nhận hoặc hủy.',log_transcripts)
                        continue
                    break
                confirmation_retries=0
                received=utcnow()
                turn_started=time.monotonic()
                try:text=await asyncio.to_thread(be.transcribe,raw,stt_language)
                except StepError as error:
                    if error.code not in ('STT_NO_RELIABLE_SPEECH','STT_TRANSCRIPT_INVALID','VOICE_CAPTURE_IS_SILENT'):
                        raise
                    print('VOICE_RETRY:',error.code,flush=True)
                    await say(player,'Tôi chưa nghe rõ. Bạn vui lòng nói lại sau tiếng bíp.',log_transcripts)
                    continue
                finally:del raw
                stt_done=time.monotonic()
                if log_transcripts:print('USER:',text,flush=True)
                request_id=None
                ending=goodbye(text)
                handled=None if ending else await asyncio.to_thread(actions.consume,text)
                if ending:
                    actions.pending=None;reply=GOODBYE
                    result=dict(intent='FAREWELL',confidence=1.,sources=[],end_session=True)
                elif handled:
                    reply,request_id=handled
                    dispatched=bool(request_id)
                    result=dict(intent='ESCORT_CONFIRMATION',confidence=1.,sources=[],end_session=bool(request_id))
                else:
                    result=await asyncio.to_thread(ai.ask,sid,text,turns,candidates,hospital_information_id)
                    reply=result['speech_text']
                    if result.get('action'):
                        reply=await asyncio.to_thread(actions.prepare,result['action'],sid)
                answer_done=time.monotonic()
                await say(player,reply,log_transcripts)
                writer.record(text,reply,result,received,
                    'RESOLVED' if result.get('action') or request_id else 'EXECUTED',request_id)
                logging.info('VOICE_TURN_TIMING stt_ms=%d answer_ms=%d playback_ms=%d confirming=%s',
                    round((stt_done-turn_started)*1000),round((answer_done-stt_done)*1000),
                    round((time.monotonic()-answer_done)*1000),confirming)
                turns.extend([dict(role='user',content=text[:1000]),dict(role='assistant',content=reply[:1000])]);turns=turns[-8:]
                if result.get('end_session'):ended=True;break
            else:print('VOICE_SESSION_MAX_TURNS',flush=True)
            if not dispatched and not ended:await say(player,GOODBYE,log_transcripts)
        except asyncio.CancelledError:
            status='CANCELLED';raise
        except BaseException:
            status='FAILED';raise
        finally:
            async def complete_history():
                final_status=status
                try:await writer.finish()
                except Exception:final_status='FAILED'
                await asyncio.to_thread(history.close,final_status)
            try:await asyncio.shield(complete_history())
            except Exception as exc:print('VOICE_HISTORY_CLOSE_PENDING:',type(exc).__name__,flush=True)
            turns.clear();actions.pending=None
        # The independent BE job now owns MOVE and arrival SPEAK. Leave
        # the audio device free and do not cancel the job on chat shutdown.
        if once or dispatched:return dispatched
