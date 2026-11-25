"""Guide status expires closed and never contains face/job/credential data."""
import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from http.server import ThreadingHTTPServer
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.face_guide import PAGE, current_state, handler_for, write_state


class FaceGuideTests(unittest.TestCase):
    def test_active_and_expired_status(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'face-guide.json'
            self.assertEqual(current_state(path)['reason'], 'IDLE')
            write_state(path, 'WAITING', time.time() + 10)
            self.assertEqual(current_state(path), {'active': True, 'reason': 'WAITING'})
            self.assertEqual(set(json.loads(path.read_text())), {'reason', 'updatedAt', 'expiresAt'})
            write_state(path, 'VERIFIED', time.time() - 1)
            self.assertEqual(current_state(path), {'active': False, 'reason': 'IDLE'})

    def test_page_uses_same_frame_as_ai_and_loopback_stream(self):
        self.assertIn('left:20%;top:12%;width:60%;height:76%', PAGE)
        self.assertIn('FACE_OUTSIDE_FRAME', PAGE)

    def test_http_page_and_status(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'face-guide.json'
            server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(path, 'http://127.0.0.1:8080/stream'))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                base = 'http://127.0.0.1:' + str(server.server_address[1])
                with urlopen(base + '/') as response:
                    body = response.read().decode('utf-8')
                    self.assertIn('const STREAM_URL="http://127.0.0.1:8080/stream"', body)
                    self.assertIn('connect-src', response.headers['Content-Security-Policy'])
                with urlopen(base + '/status') as response:
                    self.assertEqual(json.load(response), {'active': False, 'reason': 'IDLE'})
                write_state(path, 'WAITING', time.time() + 10)
                with urlopen(base + '/status') as response:
                    self.assertEqual(json.load(response), {'active': True, 'reason': 'WAITING'})
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
