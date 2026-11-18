"""Resolve a voice destination against BE data; dispatch only after explicit consent."""
import asyncio
import json
import logging
import math
import re
import time
import unicodedata
import uuid
import ssl
from pathlib import Path
from .be_token import write_private
from .contracts import StepError
from .navigation_session import validate_descriptor


INVITATION = 'Mời bạn di chuyển theo tôi.'
READINESS_ERRORS=frozenset({'VOICE_NAVIGATION_SESSION_NOT_READY','NAVIGATION_SESSION_NOT_LOCALIZED'})
SESSION_ERRORS=frozenset({'NAVIGATION_SESSION_CLOSED_OR_CHANGED','NAVIGATION_SESSION_CLOSED_OR_EXPIRED',
    'NAVIGATION_SESSION_CLOSED_OR_INVALID','NAVIGATION_SESSION_MAP_OR_ID_MISMATCH','NAVIGATION_SESSION_PROCESS_CHANGED'})
WAITING_REPLY='Tôi đang chờ cập nhật định vị hoặc kết nối trong phiên hiện tại. Tôi chưa gửi nhiệm vụ; bạn có thể nói xác nhận lại hoặc hủy.'
CHANGED_REPLY='Phiên bản đồ hoặc vị trí khởi tạo đã thay đổi. Tôi chưa gửi nhiệm vụ; bạn vui lòng nhờ nhân viên kiểm tra.'


def session_context(session):
    return {key:session[key] for key in ('sessionId','mapId','mapAssetHash')} | {
        'revision':session['initialization']['revision']}


def words(text):
    text=unicodedata.normalize('NFD',text.casefold().replace('đ','d'))
    return ' '.join(''.join(c if c.isalnum() or c.isspace() else ' ' for c in text
                            if unicodedata.category(c)!='Mn').split())


class VoiceActions:
    """The model proposes a catalog endpoint, never coordinates or a workflow graph."""
    def __init__(self,be,config,marker,before_dispatch=None):
        self.be=be;self.enabled=config.get('actions_enabled') is True
        self.before_dispatch=before_dispatch
        self.marker=Path(marker).with_name('voice-action-pending.json');self.pending=None
        self.speed=config.get('navigation_speed_mps', 0.35)
        if type(self.speed) not in (int,float) or not math.isfinite(self.speed) or not 0.10 <= self.speed <= 0.35:
            raise StepError('VOICE_NAVIGATION_SPEED_REQUIRES_0P10_TO_0P35')
        self.ready_timeout=config.get('navigation_ready_timeout_sec',6.)
        if (type(self.ready_timeout) not in (int,float) or not math.isfinite(self.ready_timeout)
                or not .5<=self.ready_timeout<=15.):raise StepError('INVALID_VOICE_NAVIGATION_READY_TIMEOUT')

    def get(self,path,timeout=10):
        _,_,raw=self.be.request(path,timeout=timeout)
        return self.be.value(raw)

    def catalog(self,timeout=10):
        value=self.get('/api/v1/pi/voice/navigation-destinations',timeout)
        value['navigationSession']=validate_descriptor(value.get('navigationSession'),self.be.cfg['robot_id'])
        if not isinstance(value.get('endpoints'),list):raise StepError('VOICE_CATALOG_INVALID')
        return value

    def conversation_catalog(self):
        if not self.enabled:return {}
        try:return self.catalog()
        except (StepError,OSError):return {}

    def prepare(self,action,interaction_id):
        self.pending=None
        if not self.enabled:
            return 'Tôi đang ở chế độ thử hội thoại. Chức năng dẫn đường bằng giọng nói chưa được bật.'
        if self.marker.exists():
            return 'Một yêu cầu trước đang chờ kiểm tra trên hệ thống. Bạn vui lòng nhờ nhân viên kiểm tra trước khi tạo lượt mới.'
        label=action.get('destination_label') if action.get('kind')=='NAVIGATE' else action.get('room_code') if action.get('kind')=='ESCORT' else None
        if not isinstance(label,str) or not 1<=len(label)<=160:
            return 'Tôi chưa xác định được điểm đến. Bạn hãy nói dẫn tôi tới điểm và tên điểm đã lưu.'
        try:catalog=self.catalog()
        except StepError as error:
            if error.code in SESSION_ERRORS:
                return 'Chưa có phiên bản đồ đang mở. Bạn vui lòng mở phiên navigation trước.'
            raise
        session=catalog['navigationSession']
        if catalog.get('robotAvailable') is False:
            return 'Tôi đang thực hiện một nhiệm vụ. Bạn vui lòng chờ nhiệm vụ kết thúc.'
        expected=self.be.cfg.get('navigation_session')
        if expected and any(session.get(field)!=expected[key] for field,key in (
                ('sessionId','session_id'),('mapId','map_id'),('mapAssetHash','map_asset_hash'))):
            return CHANGED_REPLY
        if session['initialization']['revision']<=0:
            return 'Phiên bản đồ chưa có vị trí khởi tạo. Bạn vui lòng nhờ nhân viên định vị robot trong phiên hiện tại.'
        if catalog.get('workflowPrepared') is not True:
            return 'Bạn hãy bấm chuẩn bị workflow voice trên trang map trước khi thử dẫn đường.'
        endpoint_id=action.get('endpoint_id')
        endpoints=[p for p in catalog['endpoints'] if p.get('mapId')==session['mapId']
                   and (p.get('endpointId')==endpoint_id if endpoint_id else
                        any(words(p.get(k) or '')==words(label) for k in ('endpointCode','endpointName','roomCode','roomName')))]
        if len(endpoints)!=1:
            labels=', '.join(str(p.get('endpointCode')) for p in catalog['endpoints'][:10])
            return ('Tôi chưa xác định được một điểm đến duy nhất. Các điểm trên map là '+labels+'. Bạn hãy nói rõ điểm cần tới.')[:500]
        point=endpoints[0]
        destination=point.get('spokenLabel') or point.get('roomName')
        if not destination:
            name=(point.get('endpointName') or point['endpointCode']).strip()
            destination='điểm '+name.upper() if re.fullmatch(r'[A-Za-z]|[0-9]+',name) else name
        body=dict(navigationSessionId=session['sessionId'],endpointId=point['endpointId'],
                  maxSpeedMps=self.speed,voiceInteractionId=interaction_id)
        self.pending=dict(body=body,key=str(uuid.uuid4()),expires=time.monotonic()+60,
                          context=session_context(session),invited=False)
        return f'Bạn muốn tôi dẫn đến {destination} phải không? Hãy nói xác nhận hoặc hủy nhé.'

    async def consume(self,text,invite):
        """Return None for a normal query; otherwise (spoken reply, request ID)."""
        if self.pending is None:return None
        pending=self.pending
        if time.monotonic()>pending['expires']:
            self.pending=None
            return 'Yêu cầu xác nhận đã hết hạn. Bạn hãy nói lại điểm đến nếu vẫn cần dẫn đường.',None
        command=words(text)
        if command in ('huy','huy dan duong','khong','khong dong y','khong xac nhan','cancel','cancel navigation','no','do not proceed'):
            self.pending=None
            return 'Đã hủy yêu cầu dẫn đường. Tôi chưa tạo nhiệm vụ di chuyển.',None
        if command not in ('xac nhan','dong y','dong y dan duong','xac nhan dan duong',
                           'vang','da','dung roi','ok','okay','confirm','yes','yes confirm','go ahead','proceed'):
            return None
        if self.marker.exists():
            self.pending=None
            return 'Yêu cầu đã gửi đang chờ kiểm tra kết quả. Tôi chưa tạo thêm nhiệm vụ.',None
        deadline=min(pending['expires'],time.monotonic()+self.ready_timeout)
        rejections=0
        async def wait():
            remaining=deadline-time.monotonic()
            if remaining<=0:return False
            await asyncio.sleep(min(.5,remaining))
            return time.monotonic()<deadline
        def waiting():
            if time.monotonic()>=pending['expires']:
                self.pending=None
                return 'Yêu cầu xác nhận đã hết hạn. Bạn hãy nói lại điểm đến nếu vẫn cần dẫn đường.',None
            return WAITING_REPLY,None
        while True:
            remaining=deadline-time.monotonic()
            if remaining<=0:return waiting()
            try:
                # A timed-out GET can finish harmlessly in its worker thread.
                # Never cancel/retry a POST whose outcome could be ambiguous.
                catalog=await asyncio.wait_for(asyncio.to_thread(self.catalog,timeout=min(2.,remaining)),remaining)
            except asyncio.TimeoutError:return waiting()
            except (StepError,OSError) as error:
                if isinstance(error,StepError) and error.code in SESSION_ERRORS:
                    self.pending=None;return CHANGED_REPLY,None
                reason=getattr(error,'reason',error)
                transient=(isinstance(error,StepError) and (error.code in READINESS_ERRORS or error.retryable)
                           or isinstance(reason,OSError) and not isinstance(reason,ssl.SSLError))
                if not transient:raise
                logging.info('VOICE_NAVIGATION_WAIT session=%s reason=%s',pending['context']['sessionId'],
                             getattr(error,'code',type(error).__name__))
                if await wait():continue
                return waiting()
            session=catalog['navigationSession']
            if session_context(session)!=pending['context']:
                self.pending=None;return CHANGED_REPLY,None
            if catalog.get('robotAvailable') is False:
                return 'Robot vừa nhận một nhiệm vụ khác. Tôi chưa gửi yêu cầu dẫn đường mới.',None
            if session.get('state')!='READY':
                logging.info('VOICE_NAVIGATION_WAIT session=%s state=%s revision=%s',session['sessionId'],
                             session['state'],session['initialization']['revision'])
                if await wait():continue
                return waiting()
            if not any(p.get('endpointId')==pending['body']['endpointId'] and p.get('mapId')==session['mapId'] for p in catalog['endpoints']):
                self.pending=None
                return 'Điểm đến đã bị xóa hoặc ngưng sử dụng. Tôi chưa gửi nhiệm vụ di chuyển.',None
            if catalog.get('workflowPrepared') is not True:
                return 'Workflow dẫn đường chưa sẵn sàng. Tôi chưa gửi nhiệm vụ di chuyển.',None
            if 'dispatch_body' not in pending:
                pending['workflow_owns_invite']=catalog.get('workflowOwnsInvite') is True
                pending['dispatch_body']=dict(pending['body'])
                if 'workflowOwnsInvite' in catalog:
                    pending['dispatch_body']['workflowOwnsInvite']=pending['workflow_owns_invite']
            body=pending['dispatch_body'];workflow_owns_invite=pending['workflow_owns_invite']
            if not workflow_owns_invite and not pending['invited']:
                await invite(INVITATION);pending['invited']=True
                # Legacy TTS can outlast readiness polling. Recheck the same
                # context after playback without repeating its invitation.
                deadline=min(pending['expires'],time.monotonic()+self.ready_timeout)
                continue
            remaining=pending['expires']-time.monotonic()
            if remaining<=0:return waiting()
            # Keep the same receipt guard, body and key across definite readiness
            # rejections. Network/5xx/invalid receipts retain the durable guard.
            write_private(self.marker,json.dumps(dict(idempotency_key=pending['key'],body=body)))
            self.pending=None
            rejected=None
            posted_at=time.monotonic()
            try:
                if self.before_dispatch:rejected=self.before_dispatch()
                _,_,raw=await asyncio.to_thread(self.be.request,'/api/v1/pi/voice/navigation-requests',json.dumps(body).encode(),
                    'application/json',timeout=min(15.,remaining),idempotency_key=pending['key'])
            except StepError as error:
                if error.retryable:raise  # HTTP 5xx may have committed before failing.
                definite=error.code in READINESS_ERRORS|SESSION_ERRORS|{
                    'BUSINESS_RULE_VIOLATION','CONFLICT','CONCURRENT_UPDATE',
                    'UNAUTHORIZED','FORBIDDEN','NOT_FOUND','VALIDATION_ERROR'}
                if definite and callable(rejected):rejected()
                if error.code in READINESS_ERRORS:
                    self.marker.unlink(missing_ok=True);self.pending=pending;rejections+=1
                    # The HTTP operation has its own bounded receipt timeout;
                    # only catalog polling/waiting spends the readiness budget.
                    deadline=min(pending['expires'],deadline+time.monotonic()-posted_at)
                    logging.info('VOICE_NAVIGATION_WAIT session=%s reason=%s attempt=%s',
                                 pending['context']['sessionId'],error.code,rejections)
                    if rejections<2 and await wait():continue
                    return waiting()
                if error.code in SESSION_ERRORS:
                    self.marker.unlink(missing_ok=True);return CHANGED_REPLY,None
                if error.code in ('BUSINESS_RULE_VIOLATION','CONFLICT','CONCURRENT_UPDATE',
                                  'UNAUTHORIZED','FORBIDDEN','NOT_FOUND','VALIDATION_ERROR'):
                    self.marker.unlink(missing_ok=True)
                raise
            response=self.be.value(raw)
            parent=response.get('serviceRequest') or response.get('request') or response
            request_id=parent.get('serviceRequestId') if isinstance(parent,dict) else None
            if not request_id:raise StepError('VOICE_ACTION_RECEIPT_PENDING')
            uuid.UUID(request_id)
            self.marker.unlink(missing_ok=True)
            print('VOICE_NAVIGATION_REQUEST',request_id,'JOBS',','.join(j['jobId'] for j in response.get('jobs',[]) if j.get('jobId')),flush=True)
            return 'Đã nhận yêu cầu dẫn đường.' if workflow_owns_invite else INVITATION,request_id
