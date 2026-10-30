"""Temporary BE map context, verified HTTPS controls, and MOVE binding. No motor access."""
import copy,json,math,ssl,threading,time,urllib.request,urllib.error
from datetime import datetime,timezone
from pathlib import Path
from urllib.parse import urlsplit
from uuid import UUID
from .contracts import StepError,parse_timestamp
from .be_token import NoRedirect,upload_token

def configuration(cfg,hardware):
    value=cfg.get('navigation_session')
    if value is None:return None
    try:
        UUID(value['session_id']);UUID(value['map_id'])
        if value['map_id']!=hardware['map_id'] or value['map_asset_hash']!=hardware['map_asset_sha256']:raise ValueError()
        if parse_timestamp(value['expires_at']).tzinfo is None:raise ValueError()
        if cfg['mapping']['enabled'] or not cfg.get('localization',{}).get('enabled'):raise ValueError()
    except (KeyError,TypeError,ValueError):raise StepError('INVALID_NAVIGATION_SESSION') from None
    return value

def expired(context,now=None):
    return parse_timestamp(context['expires_at']) <= (now or datetime.now(timezone.utc))

def summary(context):return {k:context[k] for k in ('session_id','map_id','map_asset_hash','expires_at')}

def validate_descriptor(value,robot_id,expected=None):
    try:
        UUID(value['sessionId']);UUID(value['mapId'])
        if value['robotId']!=robot_id or expected and value['sessionId']!=expected:raise ValueError()
        if value['state'] in ('NONE','CLOSED','EXPIRED'):raise ValueError()
        if parse_timestamp(value['expiresAt'])<=datetime.now(timezone.utc):raise ValueError()
        import re
        if not re.fullmatch('[a-f0-9]{64}',value['mapAssetHash']):raise ValueError()
        init=value['initialization'];mode=init['mode'];revision=init['revision']
        if mode not in ('MANUAL','SEED','GLOBAL') or type(revision) is not int or revision<0:raise ValueError()
        if mode=='SEED':
            p=init['pose']
            if any(type(p[k]) not in (int,float) or not math.isfinite(p[k]) for k in ('x_m','y_m','yaw_rad')) or abs(p['yaw_rad'])>math.pi:raise ValueError()
        elif init.get('pose') is not None:raise ValueError()
        return copy.deepcopy(value)
    except (KeyError,ValueError,TypeError):raise StepError('NAVIGATION_SESSION_CLOSED_OR_INVALID') from None

class SessionClient:
    def __init__(self,cfg):
        self.cfg=cfg;self.auth=cfg['mapping'];uri=urlsplit(self.auth['upload_base_url'])
        if uri.scheme!='https' or not uri.hostname or uri.username or uri.password:raise StepError('HTTPS_BE_ORIGIN_REQUIRED')
        self.origin=uri.scheme+'://'+uri.netloc
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect(),
            urllib.request.HTTPSHandler(context=ssl.create_default_context(cafile=self.auth.get('upload_ca_file'))))
    def request(self,path,body=None,limit=65536):
        for attempt in range(2):
            token=upload_token(self.auth,self.cfg['robot_id'],force=attempt==1)
            req=urllib.request.Request(self.origin+path,data=json.dumps(body).encode() if body is not None else None,
                headers={'Authorization':'Bearer '+token,**({'Content-Type':'application/json'} if body is not None else {})})
            try:
                with self.opener.open(req,timeout=4) as response:raw=response.read(limit+1)
                if len(raw)>limit:raise StepError('SESSION_RESPONSE_TOO_LARGE')
                return raw
            except urllib.error.HTTPError as error:
                if error.code in (401,403) and attempt==0:continue
                # Retain only recognized, non-secret protocol reasons. These two
                # responses can race the MQTT heartbeat/STEP_STARTED commit.
                code='NAVIGATION_SESSION_HTTP_'+str(error.code)
                try:message=json.loads(error.read(4096)).get('message','')
                except (ValueError,TypeError):message=''
                transient=message in ('Navigation plan session or localization changed',
                                      'Navigation plan does not belong to the active MOVE step')
                raise StepError(code,retryable=error.code==422 and transient) from None
        raise StepError('NAVIGATION_SESSION_AUTH_FAILED')
    def get(self,expected=None):return validate_descriptor(json.loads(self.request('/api/v1/pi/navigation-session')),self.cfg['robot_id'],expected)

class SessionPoller:
    """HTTPS never blocks the ROS executor. Fail closed when authorization evidence ages."""
    def __init__(self,cfg):
        self.client=SessionClient(cfg);self.context=cfg['navigation_session'];self.lock=threading.Lock();self.value=None;self.at=-math.inf
        self.error='WAITING_FOR_SESSION_AUTHORIZATION';self.quit=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        while not self.quit.is_set():
            try:
                value=self.client.get(self.context['session_id'])
                if value['mapId']!=self.context['map_id'] or value['mapAssetHash']!=self.context['map_asset_hash']:raise StepError('NAVIGATION_SESSION_MAP_CHANGED')
                with self.lock:self.value=value;self.at=time.monotonic();self.error=None
            except Exception as e:
                with self.lock:self.value=None;self.error=getattr(e,'code','NAVIGATION_SESSION_UNREACHABLE')
            self.quit.wait(1.)
    def current(self):
        with self.lock:value=copy.deepcopy(self.value);age=time.monotonic()-self.at;error=self.error
        if expired(self.context):return None,'NAVIGATION_SESSION_EXPIRED'
        if value is None or age>5:return None,error or 'NAVIGATION_SESSION_AUTHORIZATION_STALE'
        return value,None
    def close(self):self.quit.set()

def command_context(command,cfg,hardware,adapter,require_localization=True):
    """Freeze map, session, initialization and profile for exactly this MOVE acceptance."""
    context=cfg.get('navigation_session');received=command.get('navigationContext')
    if not context:
        if received is not None:raise StepError('UNEXPECTED_NAVIGATION_CONTEXT')
        return True
    if expired(context) or not isinstance(received,dict):raise StepError('NAVIGATION_SESSION_REQUIRED')
    for key in ('session_id','map_id','map_asset_hash'):
        if received.get(key)!=context[key]:raise StepError('NAVIGATION_SESSION_MISMATCH')
    local=adapter.telemetry().get('localization',{})
    age=local.get('age_sec')
    ready=(local.get('ready') is True and type(age) in (int,float) and math.isfinite(age) and 0<=age<=1.25)
    if require_localization and not ready:raise StepError('LOCALIZATION_NOT_READY')
    if local.get('initialization_revision')!=received.get('initialization_revision'):raise StepError('LOCALIZATION_REVISION_CHANGED')
    deployed=getattr(adapter,'navigation_profile_hash',None)
    if deployed is None or received.get('profile_hash')!=deployed:raise StepError('NAVIGATION_PROFILE_NOT_SYNCED')
    return ready
