"""BE device-token refresh for map upload. Credentials stay in private Pi files."""
import base64
import json
import os
from pathlib import Path
import ssl
import time
import urllib.request
from urllib.parse import urlsplit
import uuid
from .contracts import StepError


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self,*args,**kwargs):return None


def write_private(path,text):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.'+uuid.uuid4().hex+'.new')
    try:
        fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w',encoding='utf-8') as stream:
            stream.write(text.rstrip()+'\n');stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,path);path.chmod(0o600)
    finally:temporary.unlink(missing_ok=True)


def remaining_seconds(token):
    # A scheduling hint only. The BE still verifies signature, scope and revocation.
    try:
        raw=token.split('.')[1]
        claims=json.loads(base64.urlsafe_b64decode(raw+'='*(-len(raw)%4)))
        expires=claims['exp']
        return float(expires)-time.time() if type(expires) in (int,float) else None
    except (ValueError,KeyError,IndexError,TypeError):return None


def request_device_token(base_url,ca_file,robot_code,secret,robot_id,timeout=15):
    uri=urlsplit(base_url)
    if uri.scheme!='https' or not uri.hostname or uri.username or uri.password or uri.query or uri.fragment:
        raise StepError('INVALID_DEVICE_TOKEN_URL')
    origin=uri.scheme+'://'+uri.netloc
    context=ssl.create_default_context(cafile=str(ca_file) if ca_file else None)
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect(),urllib.request.HTTPSHandler(context=context))
    request=urllib.request.Request(origin+'/api/v1/pi/auth/token',
        data=json.dumps({'robotCode':robot_code,'deviceSecret':secret}).encode(),
        headers={'Content-Type':'application/json'},method='POST')
    with opener.open(request,timeout=timeout) as response:
        raw=response.read(65537)
    if len(raw)>65536:raise StepError('TOKEN_RESPONSE_TOO_LARGE')
    result=json.loads(raw)
    if (str(result.get('robotId','')).lower()!=str(robot_id).lower()
            or str(result.get('tokenType','')).lower()!='bearer'
            or not isinstance(result.get('accessToken'),str) or len(result['accessToken'])<16):
        raise StepError('TOKEN_IDENTITY_MISMATCH')
    return result


def upload_token(cfg,robot_id,force=False):
    path=Path(cfg['upload_token_file'])
    token=path.read_text(encoding='utf-8').strip() if path.is_file() else ''
    remaining=remaining_seconds(token)
    secret_file=cfg.get('device_secret_file')
    if secret_file and (force or not token or remaining is None or remaining<90):
        try:
            secret=Path(secret_file).read_text(encoding='utf-8').strip()
            if not secret or not cfg.get('robot_code'):raise ValueError('Device credentials missing')
            result=request_device_token(cfg['upload_base_url'],cfg.get('upload_ca_file'),cfg['robot_code'],secret,robot_id)
            token=result['accessToken'];write_private(path,token)
        except Exception:
            raise StepError('MAP_UPLOAD_AUTH_REFRESH_FAILED','Could not obtain a BE device token; inspect HTTPS/CA and device credentials') from None
    elif remaining is not None and remaining<=0:
        raise StepError('MAP_UPLOAD_TOKEN_EXPIRED','Refresh BE device token before SAVE')
    if not token:raise StepError('MAP_UPLOAD_TOKEN_REQUIRED')
    return token
