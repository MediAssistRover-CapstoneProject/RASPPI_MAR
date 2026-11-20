"""Managed wake/chat companion. No ROS node, MQTT client, audio archive or DB polling."""
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
                            idle_settle_sec=2.,start_timeout_sec=12,confirmation_timeout_sec=20,max_turns=10).items():
        value.setdefault(key,default)
    if type(value['enabled']) is not bool:raise StepError('INVALID_VOICE_CHAT_CONFIG')
    if not isinstance(value['config_file'],str) or not (Path(value['config_file']).is_absolute()
                                                        or PurePosixPath(value['config_file']).is_absolute()):
        raise StepError('INVALID_VOICE_CHAT_CONFIG','absolute config_file required')
    for key,low,high in (('idle_settle_sec',.5,5),('start_timeout_sec',5,30),('confirmation_timeout_sec',10,30),('max_turns',1,20)):
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

    def report(self,state):
        if state!=self.last_state:
            logging.info('VOICE_STATE %s',state);self.last_state=state

    def available(self):
        from .media_runtime import audio_reserved
        if audio_reserved():return False
        value=self.reader.read()
        return bool(value.get('connected') and (value.get('status')=='IDLE'
                    or self.handoff and value.get('status')=='BUSY'))

    def submitting(self):
        # The confirmed HTTP request may create a job before its reply returns.
        # Finish that handoff; cancellation must never replay or cancel the job.
        self.handoff=True;self.report('HANDOFF')
        def rejected():
            # A definite HTTP rejection created no job. Another job must once
            # again preempt the chat while this consent waits for readiness.
            self.handoff=False;self.report('CHATTING')
        return rejected

    async def guarded(self,operation):
        task=asyncio.create_task(operation)
        try:
            while not task.done():
                if not self.available():
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
        from .voice_chat import wait_for_wake,conversation,AiClient
        from .voice_actions import VoiceActions
        from .voice_client import VoiceClient
        idle_since=None
        try:
            while True:
                try:
                    self.handoff=False
                    if not self.available():
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
                    if self.be_client is None:self.be_client=VoiceClient(self.worker)
                    ai=self.ai_client;settings=ai.config
                    if self.prewarm_task is None:
                        self.prewarm_task=asyncio.create_task(self.prepare_audio(ai,self.be_client))
                    self.report('WAIT_WAKE')
                    woke,_=await self.guarded(wait_for_wake(self.worker,ai))
                    idle_since=None
                    if not woke:continue
                    # The live catalog supplies the current profile. An older
                    # background warm-up must never replace it after wake.
                    if self.prewarm_task is not None and not self.prewarm_task.done():
                        ai.cancel_pending()
                        self.prewarm_task.cancel()
                        await asyncio.gather(self.prewarm_task,return_exceptions=True)
                    self.report('CHECK_WAKE')
                    actions=VoiceActions(self.be_client,settings,config.parent/'voice-chat-session.json')
                    checked,catalog=await self.guarded(asyncio.to_thread(actions.catalog))
                    if not checked:continue
                    # Check the whole BE job, not only the currently active Pi
                    # step: server-side WAIT/branches and inter-step gaps matter.
                    if catalog.get('robotAvailable') is not True:
                        self.report('WAIT_BE_JOB');await asyncio.sleep(1);continue
                    if catalog.get('navigationSession',{}).get('sessionId')!=self.worker['navigation_session']['session_id']:
                        raise StepError('VOICE_SESSION_CHANGED')
                    self.report('CHATTING')
                    await self.guarded(conversation(self.worker,config,max_turns=self.options['max_turns'],
                        initial_catalog=catalog,before_dispatch=self.submitting,log_transcripts=False,
                        start_timeout=self.options['start_timeout_sec'],ai_client=ai,be_client=self.be_client))
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
