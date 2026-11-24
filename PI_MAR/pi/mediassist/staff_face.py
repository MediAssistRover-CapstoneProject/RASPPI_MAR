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

CAPABILITY = 'staff-face-verify-v1'
MESSAGES = {
    'CENTER': 'Vui lòng nhìn thẳng vào camera để xác thực khuôn mặt.',
    'NO_FACE': 'Chưa thấy khuôn mặt. Vui lòng nhìn vào camera.',
    'MULTIPLE_FACES': 'Vui lòng chỉ một người đứng trước camera.',
    'LOW_QUALITY': 'Vui lòng đứng gần hơn, giữ yên và nhìn thẳng vào camera.',
    'NO_MATCH': 'Khuôn mặt chưa khớp với nhân viên được giao. Vui lòng thử lại.',
}


class StaffFaceScanner(StaffQrScanner):
    # Reuse the ROS-owned camera subscription lifecycle, without requiring a QR decoder.
    def __init__(self, worker):
        self.worker = worker
        self.camera = {**worker.cfg['mapping']['camera'], 'rate_hz': 2, 'sample_every_frame': False}
        if not self.camera.get('enabled'):
            raise ValueError('FACE_CAMERA_DISABLED')
        self.client = SessionClient(worker.cfg)
        self.preview = None
        self.hint_task = None
        from .speech import playback_volume
        self.guidance = {'messages': MESSAGES, 'language_code': 'vi', 'volume': playback_volume(worker.cfg.get('speech',{}))}

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
        next_submit = next_hint = started = 0.
        last_frame = None
        try:
            while True:
                window = self.window(session)
                if window != selected:
                    await self.stop_hint()
                    await self.set_preview(False)
                    selected, verified = window, None
                    started = time.monotonic()
                    last_frame = None
                    next_hint = started
                if window is None or window == verified:
                    await asyncio.sleep(.15)
                    continue
                if self.preview is None:
                    try:
                        await self.set_preview(True)
                    except Exception as exc:
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
                    await asyncio.sleep(.1)
                    continue
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
                        await self.stop_hint()
                        await self.set_preview(False)
                        logging.info('STAFF_FACE_VERIFIED job=%s step=%s', window[0], window[1])
                    elif result.get('reason') in MESSAGES and time.monotonic() >= next_hint - 5:
                        await self.stop_hint()
                        self.hint_task = asyncio.create_task(self.play_hint(session, window, result['reason']))
                        next_hint = time.monotonic() + 10
                except Exception as exc:
                    logging.info('STAFF_FACE_RETRY code=%s', getattr(exc, 'code', type(exc).__name__))
                await asyncio.sleep(.1)
        finally:
            await self.stop_hint()
            await self.set_preview(False)
