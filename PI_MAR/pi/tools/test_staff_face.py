"""Face scan scope and camera lifecycle tests; no ROS, motors or network."""
import asyncio
import sys
import time
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import AsyncMock, Mock, patch
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.staff_face import StaffFaceScanner


class FaceScannerTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.worker = N(connection_ready=True, shutting_down=False, session_fault=False,
                        status=lambda: 'IDLE', adapter=N(stationary_for_handover=lambda: True))
        self.scanner = StaffFaceScanner.__new__(StaffFaceScanner)
        self.scanner.worker = self.worker
        self.session = N(id=str(uuid4()), failure=False, staff_face_scan=({
            'jobId': str(uuid4()), 'authJobStepId': str(uuid4()), 'sessionId': str(uuid4()),
            'expiresAt': (datetime.now(timezone.utc)+timedelta(seconds=120)).isoformat()}, time.monotonic()))

    def test_scoped_window_and_stale_heartbeat(self):
        self.assertEqual(len(self.scanner.window(self.session)), 3)
        self.session.staff_face_scan = (self.session.staff_face_scan[0], time.monotonic()-8)
        self.assertIsNone(self.scanner.window(self.session))

    def test_expired_cancelled_moving_and_disconnected(self):
        self.worker.connection_ready = False
        self.assertIsNone(self.scanner.window(self.session))
        self.worker.connection_ready = True
        self.worker.adapter.stationary_for_handover = lambda: False
        self.assertIsNone(self.scanner.window(self.session))
        self.worker.adapter.stationary_for_handover = lambda: True
        self.session.staff_face_scan[0]['expiresAt'] = datetime.now(timezone.utc).isoformat()
        self.assertIsNone(self.scanner.window(self.session))

    def test_qr_window_does_not_enable_face(self):
        self.session.staff_qr_scan = self.session.staff_face_scan
        self.session.staff_face_scan = None
        self.assertIsNone(self.scanner.window(self.session))

    async def exercise(self, result, seconds=.3, encode_failure=False, open_failure=False):
        scanner = self.scanner
        scanner.preview = None
        scanner.hint_task = None
        scanner.stop_hint = AsyncMock()
        scanner.play_hint = AsyncMock()
        preview = N(take_sample=lambda age: (b'pixels', 100, 100, 300, 'bgr8', 'now', time.monotonic()))
        failed_open = False
        async def set_preview(enabled):
            nonlocal failed_open
            if enabled and open_failure and not failed_open:
                failed_open = True
                raise RuntimeError('camera unavailable')
            scanner.preview = preview if enabled else None
        scanner.set_preview = AsyncMock(side_effect=set_preview)
        scanner.client = N(request=Mock(return_value=result))
        with patch('mediassist.staff_face.encode_jpeg',
                   side_effect=[ValueError('invalid frame'), {'jpeg': b'jpeg'}] if encode_failure else None,
                   return_value={'jpeg': b'jpeg'}):
            task = asyncio.create_task(scanner.run(self.session))
            await asyncio.sleep(seconds)
            self.assertFalse(task.done(), 'Scanner stopped unexpectedly')
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self.assertIsNone(scanner.preview)
        return scanner.client.request

    async def test_success_stops_submissions_and_closes_camera(self):
        request = await self.exercise(b'{"verified":true}', 1.2)
        self.assertEqual(request.call_count, 1)
        body = request.call_args.args[1]
        self.assertEqual(body['sessionId'], self.session.staff_face_scan[0]['sessionId'])
        self.assertNotIn('matched', body)

    async def test_mismatch_rate_limited(self):
        request = await self.exercise(b'{"verified":false,"reason":"NO_MATCH"}', .5)
        self.assertEqual(request.call_count, 1)

    async def test_encoder_recovers_without_sending_invalid_frame(self):
        request = await self.exercise(b'{"verified":true}', 1.4, encode_failure=True)
        self.assertEqual(request.call_count, 1)

    async def test_camera_open_recovers(self):
        request = await self.exercise(b'{"verified":true}', 1.4, open_failure=True)
        self.assertEqual(request.call_count, 1)


if __name__ == '__main__':
    unittest.main()
