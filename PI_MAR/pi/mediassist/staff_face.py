"""Capture fresh camera frames only inside a BE-scoped face verification window."""
import asyncio
import base64
import json
import logging
import time
from datetime import datetime, timezone
from uuid import UUID

from .staff_qr import StaffQrScanner
from .navigation_session import SessionClient, parse_timestamp
from .camera_preview import encode_jpeg
from .face_guide import state_path, write_state

CAPABILITY = 'staff-face-verify-v1'
MESSAGES = {
    'CENTER': 'Vui lòng nhìn thẳng vào camera để xác thực khuôn mặt.',
    'NO_FACE': 'Chưa thấy khuôn mặt. Vui lòng nhìn vào camera.',
    'MULTIPLE_FACES': 'Vui lòng chỉ một người đứng trước camera.',
    'LOW_QUALITY': 'Vui lòng đứng gần hơn, giữ yên và nhìn thẳng vào camera.',
    'FACE_OUTSIDE_FRAME': 'Vui lòng đưa toàn bộ khuôn mặt vào trong khung.',
    'CAMERA_UNAVAILABLE': 'Camera chưa sẵn sàng. Vui lòng chờ hoặc báo nhân viên hỗ trợ.',
    'NO_MATCH': 'Khuôn mặt chưa khớp với nhân viên được giao. Vui lòng thử lại.',
    'VERIFIED': 'Xác thực thành công. Đang chờ mở ngăn.',
}


class StaffFaceScanner(StaffQrScanner):
    # Reuse the ROS-owned camera subscription lifecycle, without requiring a QR decoder.
    def __init__(self, worker):
        self.worker = worker
        self.camera = {**worker.cfg['mapping']['camera'], 'rate_hz': 2, 'sample_every_frame': False}
        if not self.camera.get('enabled'):
            raise ValueError('FACE_CAMERA_DISABLED')
        self.client = SessionClient(worker.cfg)
        self.guide_path = state_path(worker.cfg)
        self.preview = None
        self.hint_task = None
        from .speech import playback_volume
        self.guidance = {'messages': MESSAGES, 'language_code': 'vi', 'volume': playback_volume(worker.cfg.get('speech',{}))}

    def guide(self, session, reason):
        path = getattr(self, 'guide_path', None)
        if path is None:
            return
        try:
            if reason == 'VERIFIED':
                expiry = time.time() + 3
            elif reason == 'IDLE':
                expiry = None
            else:
                expiry = parse_timestamp(session.staff_face_scan[0]['expiresAt']).timestamp()
            write_state(path, reason, expiry)
        except (OSError, KeyError, TypeError, ValueError, AttributeError) as exc:
            logging.warning('STAFF_FACE_GUIDE_UNAVAILABLE code=%s', type(exc).__name__)

    async def play_hint(self, session, window, hint):
        from .media_runtime import audio_reserved

        def available():
            return self.window(session) == window and not audio_reserved()

        if not available():
            return
        try:
            await self.worker.adapter.speech.speak(
                {'action': 'SPEAK', 'prompt_key': 'STAFF_FACE_' + hint,
                 'data': {'text': self.guidance['messages'][hint],
                          'language_code': self.guidance['language_code'],
                          'volume': self.guidance['volume']}},
                playback_guard=available)
            logging.info('STAFF_FACE_HINT kind=%s', hint)
        except Exception as error:
            logging.debug('STAFF_FACE_HINT_UNAVAILABLE code=%s', getattr(error, 'code', type(error).__name__))

    def window(self, session):
        value = getattr(session, 'staff_face_scan', None)
        if (not value or not self.worker.connection_ready or self.worker.shutting_down
                or self.worker.session_fault or session.failure or self.worker.status() != 'IDLE'
                or not self.worker.adapter.stationary_for_handover()):
            return None
        window, received = value
        try:
            if time.monotonic() - received > 7 or parse_timestamp(window['expiresAt']) <= datetime.now(timezone.utc):
                return None
            return tuple(str(UUID(window[k])) for k in ('jobId', 'authJobStepId', 'sessionId'))
        except (KeyError, TypeError, ValueError, AttributeError):
            return None

    async def run(self, session):
        selected = verified = None
        next_submit = next_hint = started = next_guide = 0.
        guide_reason = 'WAITING'
        last_frame = None
        try:
            while True:
                window = self.window(session)
                if window != selected:
                    if window is None and selected != verified:
                        self.guide(session, 'IDLE')
                    await self.stop_hint()
                    await self.set_preview(False)
                    selected, verified = window, None
                    started = time.monotonic()
                    last_frame = None
                    next_hint = started
                    next_guide = started + 2
                    guide_reason = 'WAITING'
                    if window is not None:
                        self.guide(session, guide_reason)
                if window is None or window == verified:
                    await asyncio.sleep(.15)
                    continue
                if time.monotonic() >= next_guide:
                    self.guide(session, guide_reason)
                    next_guide = time.monotonic() + 2
                if self.preview is None:
                    try:
                        await self.set_preview(True)
                    except Exception as exc:
                        guide_reason = 'CAMERA_UNAVAILABLE'
                        self.guide(session, guide_reason)
                        logging.info('STAFF_FACE_CAMERA_RETRY code=%s', type(exc).__name__)
                        await asyncio.sleep(1)
                        continue
                if self.hint_task is not None and self.hint_task.done():
                    await self.stop_hint()
                if time.monotonic() >= next_hint and self.hint_task is None:
                    self.hint_task = asyncio.create_task(self.play_hint(session, window, 'CENTER'))
                    next_hint = time.monotonic() + 10
                sample = self.preview.take_sample(.5)
                if (sample is None or sample[6] < started or sample[6] == last_frame
                        or time.monotonic() < next_submit):
                    if sample is None and time.monotonic() - started > 5 and last_frame is None and guide_reason != 'CAMERA_UNAVAILABLE':
                        guide_reason = 'CAMERA_UNAVAILABLE'
                        self.guide(session, guide_reason)
                    await asyncio.sleep(.1)
                    continue
                if guide_reason == 'CAMERA_UNAVAILABLE':
                    guide_reason = 'WAITING'
                    self.guide(session, guide_reason)
                last_frame = sample[6]
                try:
                    frame = await asyncio.to_thread(encode_jpeg, sample, {
                        'max_width': 960, 'max_height': 720, 'jpeg_quality': 85, 'max_jpeg_bytes': 512 * 1024})
                except Exception as exc:
                    logging.info('STAFF_FACE_FRAME_RETRY code=%s', type(exc).__name__)
                    await asyncio.sleep(1)
                    continue
                if frame is None or self.window(session) != window:
                    await asyncio.sleep(.1)
                    continue
                next_submit = time.monotonic() + 1
                try:
                    raw = await asyncio.to_thread(self.client.request,
                        f'/api/v1/pi/jobs/{window[0]}/face-auth/verify',
                        {'authJobStepId': window[1], 'sessionId': window[2], 'transportSessionId': session.id,
                         'image': base64.b64encode(frame['jpeg']).decode('ascii')}, 4096, 6)
                    result = json.loads(raw)
                    result = result.get('data', result)
                    if self.window(session) != window:
                        continue
                    if result.get('verified') is True:
                        verified = window
                        self.guide(session, 'VERIFIED')
                        await self.stop_hint()
                        await self.set_preview(False)
                        self.hint_task = asyncio.create_task(self.play_hint(session, window, 'VERIFIED'))
                        logging.info('STAFF_FACE_VERIFIED job=%s step=%s', window[0], window[1])
                    elif result.get('reason') in MESSAGES:
                        guide_reason = result['reason']
                        self.guide(session, guide_reason)
                        next_guide = time.monotonic() + 2
                        if time.monotonic() >= next_hint - 5:
                            await self.stop_hint()
                            self.hint_task = asyncio.create_task(self.play_hint(session, window, guide_reason))
                            next_hint = time.monotonic() + 10
                except Exception as exc:
                    logging.info('STAFF_FACE_RETRY code=%s', getattr(exc, 'code', type(exc).__name__))
                await asyncio.sleep(.1)
        finally:
            if selected != verified:
                self.guide(session, 'IDLE')
            await self.stop_hint()
            await self.set_preview(False)
