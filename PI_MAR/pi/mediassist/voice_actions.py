"""Resolve a voice destination against BE data; dispatch only after explicit consent."""
import json
import math
import time
import unicodedata
import uuid
from pathlib import Path
from .be_token import write_private
from .contracts import StepError
from .navigation_session import validate_descriptor


def words(text):
    text=unicodedata.normalize('NFD',text.casefold().replace('đ','d'))
    return ' '.join(''.join(c if c.isalnum() or c.isspace() else ' ' for c in text
                            if unicodedata.category(c)!='Mn').split())


class VoiceActions:
    """The model proposes a catalog endpoint, never coordinates or a workflow graph."""
    def __init__(self,be,config,marker):
        self.be=be;self.enabled=config.get('actions_enabled') is True
        self.marker=Path(marker).with_name('voice-action-pending.json');self.pending=None
        self.speed=config.get('navigation_speed_mps',.2)
        if type(self.speed) not in (int,float) or not math.isfinite(self.speed) or not .1<=self.speed<=.3:
            raise StepError('VOICE_NAVIGATION_SPEED_REQUIRES_0P10_TO_0P30')

    def get(self,path):
        _,_,raw=self.be.request(path,timeout=10)
        return self.be.value(raw)

    def catalog(self):
        value=self.get('/api/v1/pi/voice/navigation-destinations')
        value['navigationSession']=validate_descriptor(value.get('navigationSession'),self.be.cfg['robot_id'])
        if not isinstance(value.get('endpoints'),list):raise StepError('VOICE_CATALOG_INVALID')
        return value

    def candidates(self):
        if not self.enabled:return []
        try:return self.catalog()['endpoints'][:100]
        except (StepError,OSError):return []

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
            if error.code=='BUSINESS_RULE_VIOLATION':
                return 'Chưa có phiên bản đồ đang mở. Bạn vui lòng mở phiên navigation trước.'
            raise
        session=catalog['navigationSession']
        if session.get('state')!='READY':
            return 'Phiên bản đồ của tôi chưa sẵn sàng dẫn đường. Bạn vui lòng nhờ nhân viên mở phiên trước.'
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
        body=dict(navigationSessionId=session['sessionId'],endpointId=point['endpointId'],
                  maxSpeedMps=self.speed,voiceInteractionId=interaction_id)
        self.pending=dict(body=body,key=str(uuid.uuid4()),expires=time.monotonic()+60,
                          session_id=session['sessionId'],point_name=point.get('endpointName') or point['endpointCode'])
        return 'Bạn muốn tôi tới điểm '+self.pending['point_name']+'. Hãy nói xác nhận để bắt đầu, hoặc hủy.'

    def consume(self,text):
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
        if command not in ('xac nhan','dong y','dong y dan duong','xac nhan dan duong','confirm','yes','yes confirm','go ahead','proceed'):
            self.pending=None
            return None
        self.pending=None
        catalog=self.catalog();session=catalog['navigationSession']
        if session.get('state')!='READY' or session.get('sessionId')!=pending['session_id']:
            return 'Phiên bản đồ đã thay đổi. Tôi chưa gửi nhiệm vụ; bạn vui lòng nhờ nhân viên kiểm tra.',None
        if not any(p.get('endpointId')==pending['body']['endpointId'] and p.get('mapId')==session['mapId'] for p in catalog['endpoints']):
            return 'Điểm đến đã bị xóa hoặc ngưng sử dụng. Tôi chưa gửi nhiệm vụ di chuyển.',None
        # Persist only IDs and request parameters before dispatch. An ambiguous
        # response must not cause a second job with a new idempotency key.
        write_private(self.marker,json.dumps(dict(idempotency_key=pending['key'],body=pending['body'])))
        try:
            _,_,raw=self.be.request('/api/v1/pi/voice/navigation-requests',json.dumps(pending['body']).encode(),
                'application/json',timeout=15,idempotency_key=pending['key'])
        except StepError as error:
            # Only definite request rejection clears the receipt guard.
            # Keep it on HTTP 5xx, malformed receipts and network timeouts.
            if error.code in ('BUSINESS_RULE_VIOLATION','CONFLICT','CONCURRENT_UPDATE',
                              'UNAUTHORIZED','FORBIDDEN','NOT_FOUND','VALIDATION_ERROR'):
                self.marker.unlink(missing_ok=True)
            raise
        response=self.be.value(raw)
        parent=response.get('serviceRequest') or response.get('request') or response
        request_id=parent.get('serviceRequestId') if isinstance(parent,dict) else None
        if not request_id:
            raise StepError('VOICE_ACTION_RECEIPT_PENDING')
        uuid.UUID(request_id)
        self.marker.unlink(missing_ok=True)
        print('VOICE_NAVIGATION_REQUEST',request_id,'JOBS',','.join(j['jobId'] for j in response.get('jobs',[]) if j.get('jobId')),flush=True)
        return 'Hệ thống đã nhận yêu cầu tới điểm '+pending['point_name']+'. Robot sẽ báo khi di chuyển thành công.',request_id
