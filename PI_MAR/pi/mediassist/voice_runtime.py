"""Managed wake/chat companion; BE arbitrates patrol interruption and motion."""
import asyncio
import json
import logging
import os
from pathlib import Path,PurePosixPath
import stat
import tempfile
import time
from .contracts import StepError


def configuration(worker):
    value=worker.setdefault('voice_chat',{})
    for key,default in dict(enabled=False,config_file='/home/ubuntu/mar-secrets/voice-ai.json',
                            idle_settle_sec=2.,start_timeout_sec=12,confirmation_timeout_sec=20,max_turns=10,
                            patrol_pause_timeout_sec=20.).items():
        value.setdefault(key,default)
    if type(value['enabled']) is not bool:raise StepError('INVALID_VOICE_CHAT_CONFIG')
    if not isinstance(value['config_file'],str) or not (Path(value['config_file']).is_absolute()
                                                        or PurePosixPath(value['config_file']).is_absolute()):
        raise StepError('INVALID_VOICE_CHAT_CONFIG','absolute config_file required')
    for key,low,high in (('idle_settle_sec',.5,5),('start_timeout_sec',5,30),('confirmation_timeout_sec',10,30),
                         ('max_turns',1,20),('patrol_pause_timeout_sec',5,60)):
        v=value[key]
        if type(v) not in (int,float) or not low<=v<=high or (key=='max_turns' and type(v) is not int):
            raise StepError('INVALID_VOICE_CHAT_CONFIG',key)
    return value


def enabled(worker):
    return (worker.get('voice_chat',{}).get('enabled') is True
            and bool(worker.get('navigation_session')) and not worker.get('mapping',{}).get('enabled',True))


def state_path(worker):
    # tmpfs: updates never write patient data or add SD-card I/O.
    from uuid import UUID
    robot=str(UUID(worker['robot_id']))
    directory=Path('/dev/shm')/('mar-voice-'+str(os.getuid()))
    directory.mkdir(mode=0o700,exist_ok=True)
    info=directory.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid!=os.getuid():raise StepError('VOICE_STATE_OWNER_MISMATCH')
    return directory/(robot+'.json')


class StatePublisher:
    def __init__(self,worker):
        self.worker=worker;self.path=state_path(worker)
        self.pid=os.getpid();self.ticks=Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19]
        self.failed=False

    def publish(self,status,connected):
        try:
            session=self.worker['navigation_session']
            value=dict(pid=self.pid,startTicks=self.ticks,status=status,connected=bool(connected),
                       sessionId=session['session_id'],mapId=session['map_id'],
                       expiresMonotonic=time.monotonic()+max(7.,2*self.worker['heartbeat_sec']+1))
            fd,name=tempfile.mkstemp(prefix='.state-',dir=self.path.parent)
            try:
                with os.fdopen(fd,'w') as stream:json.dump(value,stream)
                os.replace(name,self.path)
            finally:Path(name).unlink(missing_ok=True)
            self.failed=False
        except (OSError,ValueError,KeyError):
            if not self.failed:logging.warning('VOICE_STATE_UNAVAILABLE: motion execution is unchanged')
            self.failed=True


class StateReader:
    def __init__(self,worker):
        self.path=state_path(worker);self.session=worker['navigation_session']['session_id']

    def read(self):
        try:
            info=self.path.lstat()
            if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid() or info.st_size>4096:return {}
            value=json.loads(self.path.read_text())
            if value.get('sessionId')!=self.session or value.get('expiresMonotonic',0)<=time.monotonic():return {}
            current=Path('/proc',str(int(value['pid'])),'stat').read_text().rsplit(')',1)[1].split()
            if current[0]=='Z' or current[19]!=value['startTicks']:return {}
            return value
        except (OSError,ValueError,TypeError,KeyError,IndexError):return {}


class VoiceRuntime:
    def __init__(self,worker,reader=None):
        self.worker=worker;self.options=configuration(worker)
        self.reader=reader or StateReader(worker);self.handoff=False;self.last_state=None
        self.wake_admission={};self.admission_at=0.;self.admission_checked_at=-float('inf')
        self.admission_interaction=None
        self.lease_task=None

    def report(self,state):
        if state!=self.last_state:
            logging.info('VOICE_STATE %s',state);self.last_state=state

    def available(self,wake=False):
        from .media_runtime import audio_reserved
        if audio_reserved():return False
        value=self.reader.read()
        patrol_wake=(wake and time.monotonic()-self.admission_at<3.
                     and self.wake_admission.get('wakeAllowed') is True
                     and bool(self.wake_admission.get('patrolJobId')))
        return bool(value.get('connected') and (value.get('status')=='IDLE'
                    or (self.handoff or patrol_wake) and value.get('status')=='BUSY'))

    async def refresh_availability(self,interaction_id=None):
        """At most one admission GET per second; only the BE admits a patrol wake."""
        now=time.monotonic()
        if now-self.admission_checked_at<1.:
            return self.wake_admission
        self.admission_checked_at=now
        try:
            value=await asyncio.to_thread(self.be_client.availability,interaction_id)
        except (StepError,OSError,ValueError):
            self.wake_admission={};self.admission_at=0.
            return {}
        self.wake_admission=value;self.admission_at=time.monotonic();self.admission_interaction=interaction_id
        return value

    async def keep_interaction(self,interaction_id,ready):
        """Lease failure yields the microphone; the BE expires an abandoned hold."""
        deadline=None
        while True:
            try:
                value=await asyncio.to_thread(self.be_client.interaction_heartbeat,interaction_id)
                if value.get('active') is not True:raise StepError('VOICE_INTERACTION_NO_LONGER_ACTIVE')
                lease=value['leaseSeconds'];deadline=time.monotonic()+lease
                ready.set()
            except (OSError,StepError) as error:
                if (getattr(error,'code',None)=='VOICE_INTERACTION_NO_LONGER_ACTIVE'
                        or deadline is None or time.monotonic()+3.>=deadline):
                    raise
                logging.info('VOICE_LEASE_RETRY code=%s',getattr(error,'code',type(error).__name__))
                await asyncio.sleep(2.);continue
            await asyncio.sleep(min(10.,lease/3.))

    def check_lease(self):
        if self.lease_task is not None and self.lease_task.done():
            self.lease_task.result()
            raise StepError('VOICE_INTERACTION_LEASE_ENDED')

    async def await_conversation_ready(self,interaction_id):
        deadline=time.monotonic()+self.options['patrol_pause_timeout_sec']
        while time.monotonic()<deadline:
            from .media_runtime import audio_reserved
            self.check_lease()
            if not self.reader.read().get('connected') or audio_reserved():return False
            view=await self.refresh_availability(interaction_id)
            # A pre-start admission cannot authorize a conversation. Only an
            # exact interaction query after START and local stop can do so.
            if (self.admission_interaction==interaction_id and view.get('conversationReady') is True
                    and self.available()):return True
            if view and view.get('wakeAllowed') is False and view.get('state') not in ('WAITING_FOR_STOP','PATROL_STOPPING'):
                logging.info('VOICE_ADMISSION_WAIT state=%s reason=%s',view.get('state'),view.get('reason'))
            await asyncio.sleep(.1)
        raise StepError('VOICE_PATROL_STOP_TIMEOUT')

    def submitting(self):
        # The confirmed HTTP request may create a job before its reply returns.
        # Finish that handoff; cancellation must never replay or cancel the job.
        self.handoff=True;self.report('HANDOFF')
        def rejected():
            # A definite HTTP rejection created no job. Another job must once
            # again preempt the chat while this consent waits for readiness.
            self.handoff=False;self.report('CHATTING')
        return rejected

    async def guarded(self,operation,wake=False):
        task=asyncio.create_task(operation)
        try:
            while not task.done():
                if wake and self.reader.read().get('status')=='BUSY':
                    await self.refresh_availability()
                elif not wake and self.lease_task is not None and not self.handoff and self.admission_interaction:
                    view=await self.refresh_availability(self.admission_interaction)
                    # A confirmed dispatch may begin while admission is in flight.
                    if not self.handoff and not task.done() and view.get('conversationReady') is not True:
                        self.report('YIELD_TO_PRIORITY')
                        task.cancel();await asyncio.gather(task,return_exceptions=True)
                        return False,None
                self.check_lease()
                if not self.available(wake):
                    self.report('YIELD_TO_WORKFLOW')
                    task.cancel();await asyncio.gather(task,return_exceptions=True)
                    return False,None
                await asyncio.wait({task},timeout=.1)
            return True,task.result()
        finally:
            if not task.done():task.cancel()
            await asyncio.gather(task,return_exceptions=True)

    async def run(self):
        from .voice_chat import controller_lock
        config=Path(self.options['config_file'])
        # One HTTP pool per companion, shared by wake and its conversation.
        # Reopen only after local configuration changes, while no capture runs.
        self.ai_client=None;self.be_client=None;self.config_revision=None;self.prewarm_task=None
        # Own the wake listener and chat for the lifetime of the companion.
        with controller_lock():
            try:
                await self.run_available(config)
            finally:
                for client in (self.ai_client,self.be_client):
                    if client is not None:client.close()
                if self.prewarm_task is not None:
                    self.prewarm_task.cancel()
                    await asyncio.gather(self.prewarm_task,return_exceptions=True)

    async def prepare_audio(self,ai,be):
        """Warm only the public greeting while the wake listener keeps recording."""
        try:
            health,profile=await asyncio.gather(asyncio.to_thread(ai.health),
                                               asyncio.to_thread(be.speech_profile))
            if health.get('state')!='READY' or health.get('direct_speech') is not True:return
            ai.supports_stream=health.get('speech_stream') is True
            ai.speech_profile(profile)
            await asyncio.to_thread(ai.prewarm)
            logging.info('VOICE_PREWARM_READY profile=%s stream=%s',profile['revision'],ai.supports_stream)
        except asyncio.CancelledError:raise
        except Exception as error:
            logging.info('VOICE_PREWARM_DEFERRED code=%s',getattr(error,'code',type(error).__name__))

    async def run_available(self,config):
        from .voice_chat import wait_for_wake,conversation,AiClient,History
        from .voice_actions import VoiceActions
        from .voice_client import VoiceClient
        idle_since=None
        try:
            while True:
                try:
                    self.handoff=False
                    if self.be_client is None:self.be_client=VoiceClient(self.worker)
                    state=self.reader.read()
                    if state.get('connected') and state.get('status')=='BUSY':
                        await self.refresh_availability()
                    if not self.available(wake=True):
                        idle_since=None;self.report('WAIT_ROBOT_IDLE');await asyncio.sleep(.25);continue
                    if idle_since is None:idle_since=time.monotonic()
                    if time.monotonic()-idle_since<self.options['idle_settle_sec']:
                        await asyncio.sleep(.1);continue
                    revision=(config.stat().st_mtime_ns,config.stat().st_size)
                    if self.ai_client is None or revision!=self.config_revision:
                        if self.prewarm_task is not None:
                            self.prewarm_task.cancel()
                            await asyncio.gather(self.prewarm_task,return_exceptions=True)
                        if self.ai_client is not None:self.ai_client.close()
                        self.ai_client=AiClient(config);self.config_revision=revision
                        self.prewarm_task=None
                    ai=self.ai_client;settings=ai.config
                    if self.prewarm_task is None:
                        self.prewarm_task=asyncio.create_task(self.prepare_audio(ai,self.be_client))
                    self.report('WAIT_WAKE')
                    woke,_=await self.guarded(wait_for_wake(self.worker,ai),wake=True)
                    idle_since=None
                    if not woke:continue
                    # The live catalog supplies the current profile. An older
                    # background warm-up must never replace it after wake.
                    if self.prewarm_task is not None and not self.prewarm_task.done():
                        ai.cancel_pending()
                        self.prewarm_task.cancel()
                        await asyncio.gather(self.prewarm_task,return_exceptions=True)
                    self.report('CHECK_WAKE')
                    history=History(self.be_client,ai.marker)
                    history_status='FAILED';start_task=None
                    try:
                        await asyncio.to_thread(history.recover)
                        start_task=asyncio.create_task(asyncio.to_thread(history.start,lease_supported=True))
                        sid=await asyncio.shield(start_task)
                        ready=asyncio.Event()
                        self.lease_task=asyncio.create_task(self.keep_interaction(sid,ready))
                        first=asyncio.create_task(ready.wait())
                        try:
                            await asyncio.wait({first,self.lease_task},return_when=asyncio.FIRST_COMPLETED)
                            self.check_lease()
                        finally:
                            first.cancel();await asyncio.gather(first,return_exceptions=True)
                        self.report('WAIT_PATROL_STOP')
                        if not await self.await_conversation_ready(sid):continue
                        actions=VoiceActions(self.be_client,settings,config.parent/'voice-chat-session.json')
                        checked,catalog=await self.guarded(asyncio.to_thread(actions.catalog))
                        if not checked:continue
                        if catalog.get('robotAvailable') is not True:
                            self.report('WAIT_BE_JOB');continue
                        if catalog.get('navigationSession',{}).get('sessionId')!=self.worker['navigation_session']['session_id']:
                            raise StepError('VOICE_SESSION_CHANGED')
                        self.report('CHATTING')
                        completed,_=await self.guarded(conversation(self.worker,config,max_turns=self.options['max_turns'],
                            initial_catalog=catalog,before_dispatch=self.submitting,log_transcripts=False,
                            start_timeout=self.options['start_timeout_sec'],ai_client=ai,be_client=self.be_client,
                            history=history))
                        history_status='COMPLETED' if completed else 'CANCELLED'
                    except asyncio.CancelledError:
                        history_status='CANCELLED';raise
                    finally:
                        # A cancelled await cannot cancel an in-flight START POST.
                        # Join it before completion so a late success cannot leak
                        # an ACTIVE hold after the local cleanup has finished.
                        if start_task is not None:
                            try:await asyncio.shield(start_task)
                            except Exception:pass
                        if self.lease_task is not None:
                            self.lease_task.cancel();await asyncio.gather(self.lease_task,return_exceptions=True)
                            self.lease_task=None
                        try:await asyncio.shield(asyncio.to_thread(history.close,history_status))
                        except Exception as error:
                            logging.warning('VOICE_HISTORY_CLOSE_PENDING code=%s',getattr(error,'code',type(error).__name__))
                        self.wake_admission={};self.admission_at=0.
                    self.report('WAIT_ROBOT_IDLE');await asyncio.sleep(1)
                except asyncio.CancelledError:raise
                except Exception as error:
                    # AI/DNS/USB errors never terminate the motion stack.
                    logging.warning('VOICE_RETRY code=%s',getattr(error,'code',type(error).__name__))
                    idle_since=None;self.report('RETRY');await asyncio.sleep(10)
        finally:
            self.report('STOPPED')


async def run_daemon(worker):
    if not enabled(worker):raise StepError('VOICE_REQUIRES_ENABLED_NAVIGATION_SESSION')
    await VoiceRuntime(worker).run()
