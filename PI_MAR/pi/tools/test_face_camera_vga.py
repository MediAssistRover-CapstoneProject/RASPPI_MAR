"""VGA RGB frames at 15 FPS; no ROS installation or camera required."""
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.camera_preview import CameraPreview, encode_jpeg


class VgaCameraTests(unittest.TestCase):
    def test_rgb_padding_jpeg_preserves_resolution_and_channels(self):
        import cv2
        import numpy as np
        # Red RGB pixels and 8 padding bytes per row, as a ROS driver may emit.
        rows = np.zeros((480, 640 * 3 + 8), np.uint8)
        rows[:, :640 * 3:3] = 255
        sample = (rows.tobytes(), 640, 480, rows.shape[1], 'rgb8', 'now', 10.)
        frame = encode_jpeg(sample, dict(max_width=960, max_height=720,
                                        jpeg_quality=85, max_jpeg_bytes=512 * 1024))
        self.assertEqual((frame['width'], frame['height']), (640, 480))
        decoded = cv2.imdecode(np.frombuffer(frame['jpeg'], np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (480, 640, 3))
        self.assertGreater(int(decoded[240, 320, 2]), 240)
        self.assertLess(int(decoded[240, 320, 0]), 10)

    def test_15fps_source_sampled_without_backlog_and_stale_rejected(self):
        preview = object.__new__(CameraPreview)
        preview.cfg = {'rate_hz': 2, 'sample_every_frame': False}
        preview.active = lambda: True
        preview.last_sample = 0.
        preview.error = None
        preview.lock = threading.Lock()
        preview.sample = None
        msg = N(encoding='rgb8', width=640, height=480, step=1920,
                data=bytes(640 * 480 * 3))
        accepted = []
        preview.on_receive = lambda w, h, timestamp: accepted.append(timestamp)
        with patch('mediassist.camera_preview.time.monotonic') as clock:
            for index in range(30):
                clock.return_value = 10 + index / 15
                preview.on_image(msg)
            self.assertEqual(len(accepted), 4)
            sample = preview.take_sample(.5)
            self.assertEqual(sample[-1], accepted[-1])
            self.assertIsNone(preview.take_sample(.5))
            clock.return_value = 13
            preview.on_image(msg)
            clock.return_value = 13.6
            self.assertIsNone(preview.take_sample(.5))


if __name__ == '__main__':
    unittest.main()
