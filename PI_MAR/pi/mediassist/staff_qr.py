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


def decode_staff_qr(sample):
    import cv2
    import numpy as np
    from pyzbar.pyzbar import ZBarSymbol, decode

    image = frame_pixels(sample)
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = decode(np.ascontiguousarray(gray), symbols=[ZBarSymbol.QRCODE])
    matches = {item.data for item in results if STAFF_QR.fullmatch(item.data)}
    # Reject ambiguous frames. The camera never decides whether a proof is valid.
    return matches.pop().decode('ascii') if len(matches) == 1 else None


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

    async def run(self, session):
        selected = verified = last_digest = None
        next_submit = last_submit = 0.
        try:
            while True:
                window = self.window(session)
                if window != selected:
                    self.close_preview()
                    selected = window
                    last_digest = None
                if not window or window == verified:
                    await asyncio.sleep(.15)
                    continue
                if self.preview is None:
                    self.preview = CameraPreview(self.worker.adapter, self.camera, lambda: self.preview is not None)
                    logging.info('STAFF_QR_SCAN_READY job=%s step=%s', *window)
                sample = self.preview.take_sample(.5)
                if sample is None or time.monotonic() < next_submit:
                    await asyncio.sleep(.10)
                    continue
                proof = await asyncio.to_thread(decode_staff_qr, sample)
                if proof and self.window(session) == window:
                    now = time.monotonic()
                    digest = hashlib.sha256(proof.encode('ascii')).digest()
                    if digest != last_digest or now - last_submit >= 2.:
                        next_submit = now + .75
                        last_digest, last_submit = digest, now
                        try:
                            await asyncio.to_thread(self.client.request,
                                f'/api/v1/pi/jobs/{window[0]}/staff-phone-qr/verify',
                                {'authJobStepId': window[1], 'transportSessionId': session.id, 'proof': proof},
                                1024)
                        except (StepError, OSError) as exc:
                            logging.info('STAFF_QR_NOT_ACCEPTED code=%s', getattr(exc, 'code', type(exc).__name__))
                        else:
                            verified = window
                            self.close_preview()
                            logging.info('STAFF_QR_VERIFIED job=%s step=%s', *window)
                await asyncio.sleep(.10)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            logging.warning('STAFF_QR_SCAN_UNAVAILABLE %s', type(exc).__name__)
        finally:
            self.close_preview()
