"""Read staff phone QR codes from the existing camera during a BE AUTHENTICATE window."""
import asyncio
from datetime import datetime, timezone
import hashlib
import logging
import re
import time
from uuid import UUID

from .camera_preview import CameraPreview, frame_pixels
from .contracts import StepError, parse_timestamp
from .navigation_session import SessionClient

STAFF_QR = re.compile(rb'MARSTAFF1\.[A-Za-z0-9_-]{64}')
CAPABILITIES = ('staff-qr-scan-v1', 'secure-unlock-v1')
QR_HINTS = {
    'vi': {
        'CENTER': 'Vui lòng đưa mã QR vào giữa khung hình camera.',
        'CLOSER': 'Vui lòng đưa điện thoại gần camera hơn một chút.',
        'STEADY': 'Vui lòng giữ điện thoại thẳng và yên để tôi quét mã.',
        'ONE_CODE': 'Vui lòng đưa một mã QR vào khung hình.',
    },
    'en': {
        'CENTER': 'Please center the QR code in front of the camera.',
        'CLOSER': 'Please bring the phone a little closer to the camera.',
        'STEADY': 'Please hold the phone upright and steady so I can scan the code.',
        'ONE_CODE': 'Please show one QR code at a time.',
    },
}


def guidance_configuration(worker):
    defaults = dict(enabled=True, language_code='vi', delay_sec=2.5, repeat_sec=7.,
                    min_code_fraction=.15, edge_fraction=.05, volume=70, messages={})
    raw = worker.get('staff_qr_guidance', {})
    if not isinstance(raw, dict) or set(raw) - set(defaults):
        raise StepError('INVALID_STAFF_QR_GUIDANCE')
    cfg = {**defaults, **raw}
    language = cfg['language_code']
    if (type(cfg['enabled']) is not bool or not isinstance(language, str) or language not in QR_HINTS
            or type(cfg['volume']) is not int or not 0 <= cfg['volume'] <= 100):
        raise StepError('INVALID_STAFF_QR_GUIDANCE')
    for key, low, high in (('delay_sec', 1, 15), ('repeat_sec', 4, 30),
                           ('min_code_fraction', .05, .5), ('edge_fraction', .01, .2)):
        if type(cfg[key]) not in (int, float) or not low <= cfg[key] <= high:
            raise StepError('INVALID_STAFF_QR_GUIDANCE', key)
    messages = cfg['messages']
    if (not isinstance(messages, dict) or set(messages) - set(QR_HINTS[language])
            or any(not isinstance(text, str) or not text.strip() or len(text) > 160 for text in messages.values())):
        raise StepError('INVALID_STAFF_QR_GUIDANCE', 'messages')
    return {**cfg, 'messages': {**QR_HINTS[language], **messages}}


def framing_hint(gray, detector, guidance):
    import cv2
    import numpy as np
    try:
        detected, corners = detector.detect(gray)
    except cv2.error:
        return 'CENTER'
    if not detected or corners is None:
        return 'CENTER'
    points = np.asarray(corners).reshape(-1, 2)
    if len(points) != 4 or not np.isfinite(points).all():
        return 'CENTER'
    height, width = gray.shape
    left, top = points.min(axis=0)
    right, bottom = points.max(axis=0)
    edge = guidance['edge_fraction']
    if left < edge * width or right > (1 - edge) * width or top < edge * height or bottom > (1 - edge) * height:
        return 'CENTER'
    # Apparent size is a framing cue. The QR on a phone can be zoomed,
    # so RGB bounds alone do not establish physical distance.
    side = min(right - left, bottom - top)
    return 'CLOSER' if side < guidance['min_code_fraction'] * min(width, height) else 'STEADY'


def inspect_staff_qr(sample, detector=None, guidance=None):
    started = time.perf_counter()
    import cv2
    import numpy as np
    from pyzbar.pyzbar import ZBarSymbol, decode

    image = frame_pixels(sample)
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = decode(np.ascontiguousarray(gray), symbols=[ZBarSymbol.QRCODE])
    matched = [item for item in results if STAFF_QR.fullmatch(item.data)]
    matches = {item.data for item in matched}
    details = dict(decode_ms=round((time.perf_counter()-started)*1000,2), count=len(matches))
    if len(matches) == 1:
        rect = matched[0].rect
        details['bbox'] = [int(rect.left),int(rect.top),int(rect.width),int(rect.height)]
    elif detector is not None:
        details['hint'] = 'ONE_CODE' if len(matches) > 1 else framing_hint(gray, detector, guidance)
    # Reject ambiguous frames. The camera never decides whether a proof is valid.
    return (matches.pop().decode('ascii') if len(matches) == 1 else None), details


def decode_staff_qr(sample):
    return inspect_staff_qr(sample)[0]


class StaffQrScanner:
    def __init__(self, worker):
        import cv2
        from pyzbar import pyzbar

        if not callable(pyzbar.decode) or not callable(cv2.cvtColor):
            raise StepError('QR_DECODER_UNAVAILABLE')
        self.worker = worker
        self.camera = {**worker.cfg['mapping']['camera'], 'rate_hz': 4, 'sample_every_frame': False}
        if not self.camera.get('enabled'):
            raise StepError('QR_CAMERA_DISABLED')
        self.client = SessionClient(worker.cfg)
        self.preview = None
        self.last_status = None
        self.next_status = 0.
        self.guidance = guidance_configuration(worker.cfg)
        self.detector = (cv2.QRCodeDetector() if self.guidance['enabled']
                         and worker.cfg.get('speech', {}).get('enabled') else None)
        self.hint_task = None

    def report(self, status, window, **metrics):
        now = time.monotonic()
        if status == self.last_status or now < self.next_status:
            return
        self.last_status = status
        self.next_status = now + 2.
        logging.info('STAFF_QR_STATUS job=%s step=%s status=%s metrics=%s',
                     window[0],window[1],status,metrics)

    def window(self, session):
        if (not self.worker.connection_ready or self.worker.shutting_down or self.worker.session_fault
                or session.failure or not session.staff_qr_scan or self.worker.status() != 'IDLE'
                or not self.worker.adapter.stationary_for_handover()):
            return None
        value, received = session.staff_qr_scan
        if time.monotonic() - received > 7:
            return None
        try:
            if parse_timestamp(value['expiresAt']) <= datetime.now(timezone.utc):
                return None
            return str(UUID(value['jobId'])), str(UUID(value['authJobStepId']))
        except (ValueError, TypeError, KeyError, AttributeError):
            return None

    def close_preview(self):
        if self.preview is not None:
            preview, self.preview = self.preview, None
            self.worker.adapter.node.destroy_subscription(preview.subscription)

    async def stop_hint(self):
        if self.hint_task is not None:
            task, self.hint_task = self.hint_task, None
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    async def play_hint(self, session, window, hint):
        from .media_runtime import audio_reserved

        def available():
            return self.window(session) == window and not audio_reserved()

        if not available():
            return
        data = dict(text=self.guidance['messages'][hint], language_code=self.guidance['language_code'],
                    volume=self.guidance['volume'])
        try:
            await self.worker.adapter.speech.speak(
                dict(action='SPEAK', prompt_key='STAFF_QR_' + hint, data=data), playback_guard=available)
            logging.info('STAFF_QR_HINT kind=%s', hint)
        except Exception as error:
            logging.debug('STAFF_QR_HINT_UNAVAILABLE code=%s', getattr(error, 'code', type(error).__name__))

    async def run(self, session):
        from .media_runtime import audio_reserved
        selected = verified = last_digest = None
        next_submit = last_submit = 0.
        window_started = next_hint = 0.
        first_frame = False
        try:
            while True:
                window = self.window(session)
                if window != selected:
                    self.close_preview()
                    await self.stop_hint()
                    selected = window
                    last_digest = None
                    window_started = time.monotonic()
                    next_hint = window_started + self.guidance['delay_sec']
                    first_frame = False
                    self.last_status = None
                    self.next_status = 0.
                if self.hint_task is not None and (self.hint_task.done() or audio_reserved()):
                    await self.stop_hint()
                if not window or window == verified:
                    await asyncio.sleep(.15)
                    continue
                if self.preview is None:
                    self.preview = CameraPreview(self.worker.adapter, self.camera, lambda: self.preview is not None)
                    logging.info('STAFF_QR_SCAN_READY job=%s step=%s', *window)
                sample = self.preview.take_sample(.5)
                if sample is None or time.monotonic() < next_submit:
                    if sample is None:self.report('WAIT_CAMERA_FRAME',window)
                    await asyncio.sleep(.10)
                    continue
                frame_age_ms = round(max(0.,time.monotonic()-sample[6])*1000)
                if not first_frame:
                    first_frame = True
                    logging.info('STAFF_QR_FRAME_READY job=%s step=%s frame_age_ms=%d window_elapsed_ms=%d',
                                 *window,frame_age_ms,round((time.monotonic()-window_started)*1000))
                try:
                    detector = self.detector if (time.monotonic() >= next_hint
                        and self.hint_task is None and not audio_reserved()) else None
                    proof, metrics = await asyncio.to_thread(inspect_staff_qr, sample, detector, self.guidance)
                except Exception as error:
                    self.report('FRAME_DECODE_FAILED',window,error=type(error).__name__)
                    await asyncio.sleep(.15)
                    continue
                self.report('QR_DETECTED' if proof else 'QR_AMBIGUOUS' if metrics['count']>1 else 'QR_NOT_VISIBLE',
                            window,frame_age_ms=frame_age_ms,**metrics)
                if proof and self.window(session) == window:
                    await self.stop_hint()
                    now = time.monotonic()
                    digest = hashlib.sha256(proof.encode('ascii')).digest()
                    if digest != last_digest or now - last_submit >= 2.:
                        next_submit = now + .75
                        last_digest, last_submit = digest, now
                        submitted = time.monotonic()
                        try:
                            await asyncio.to_thread(self.client.request,
                                f'/api/v1/pi/jobs/{window[0]}/staff-phone-qr/verify',
                                {'authJobStepId': window[1], 'transportSessionId': session.id, 'proof': proof},
                                1024)
                        except (StepError, OSError) as exc:
                            logging.info('STAFF_QR_NOT_ACCEPTED code=%s verify_ms=%d',
                                getattr(exc, 'code', type(exc).__name__),round((time.monotonic()-submitted)*1000))
                        else:
                            verified = window
                            self.close_preview()
                            logging.info('STAFF_QR_VERIFIED job=%s step=%s decode_ms=%.2f verify_ms=%d window_elapsed_ms=%d',
                                *window,metrics['decode_ms'],round((time.monotonic()-submitted)*1000),
                                round((time.monotonic()-window_started)*1000))
                elif metrics.get('hint') and self.window(session) == window:
                    next_hint = time.monotonic() + self.guidance['repeat_sec']
                    self.hint_task = asyncio.create_task(self.play_hint(session, window, metrics['hint']))
                await asyncio.sleep(.10)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            logging.warning('STAFF_QR_SCAN_UNAVAILABLE %s', type(exc).__name__)
        finally:
            self.close_preview()
            await self.stop_hint()
