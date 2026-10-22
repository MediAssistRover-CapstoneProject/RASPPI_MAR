"""Real localhost HTTP, no ROS/camera hardware or robot movement."""
import json
import base64
from pathlib import Path
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import urllib.error
import urllib.request
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.camera_stream import Frames, CameraServer, handler_for, EncodePacer
from mediassist.camera_preview import CameraPreview, encode_frame, encode_jpeg


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.frames=Frames('/camera/rgb')
        self.server=CameraServer(('127.0.0.1',0),handler_for(self.frames))
        self.thread=threading.Thread(target=lambda:self.server.serve_forever(poll_interval=.02),daemon=True)
        self.thread.start()
        self.base='http://127.0.0.1:'+str(self.server.server_address[1])
        self.http=urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def tearDown(self):
        self.frames.close();self.server.shutdown();self.server.server_close();self.thread.join(2.)

    def test_index_and_health_report_configured_topic(self):
        with self.http.open(self.base+'/',timeout=2) as r:
            self.assertIn(b'/stream?topic=',r.read())
        with self.http.open(self.base+'/health',timeout=2) as r:
            body=json.load(r)
            self.assertEqual(body['topic'],'/camera/rgb')
            self.assertIn('BEST_EFFORT',body['qos'])
            self.assertFalse(body['frameFresh'])
            self.assertEqual(body['viewers'],0)

    def test_snapshot_activates_capture_and_returns_shared_jpeg(self):
        def producer():
            deadline=time.monotonic()+1
            while not self.frames.viewing() and time.monotonic()<deadline:time.sleep(.005)
            self.frames.publish(b'jpeg-test','2026-10-21T00:00:00Z')
        thread=threading.Thread(target=producer);thread.start()
        with self.http.open(self.base+'/snapshot?topic=/camera/rgb',timeout=2) as r:
            self.assertEqual(r.headers['Content-Type'],'image/jpeg')
            self.assertEqual(r.read(),b'jpeg-test')
        thread.join(1.)

    def test_stream_has_multipart_jpeg_and_closes_on_shutdown(self):
        self.frames.publish(b'jpeg-test','now')
        with self.http.open(self.base+'/stream?topic=/camera/rgb',timeout=2) as r:
            self.assertIn('multipart/x-mixed-replace',r.headers['Content-Type'])
            self.assertIn(b'Content-Type: image/jpeg',r.read(64))

    def test_no_topic_frames_is_an_explicit_503(self):
        self.frames.next=lambda *a,**k:None
        with self.assertRaises(urllib.error.HTTPError) as caught:self.http.open(self.base+'/snapshot',timeout=2)
        self.assertEqual(caught.exception.code,503)
        self.assertIn(b'CAMERA_TOPIC_NO_FRAMES',caught.exception.read())

    def test_foreign_topic_and_unbounded_viewers_are_rejected(self):
        with self.assertRaises(urllib.error.HTTPError) as caught:self.http.open(self.base+'/stream?topic=/other',timeout=2)
        self.assertEqual(caught.exception.code,400)
        for _ in range(4):self.assertTrue(self.frames.enter())
        with self.assertRaises(urllib.error.HTTPError) as caught:self.http.open(self.base+'/stream',timeout=2)
        self.assertEqual(caught.exception.code,503)
        for _ in range(4):self.frames.leave()

    def test_stale_frame_is_not_returned_as_live(self):
        self.frames.publish(b'jpeg-test','now')
        revision,_,data,captured=self.frames.frame
        self.frames.frame=(revision,time.monotonic()-3,data,captured)
        self.assertIsNone(self.frames.next(-1,timeout=.01))

    def test_latest_frame_replaces_backlog_and_health_reports_real_encode_rate(self):
        with patch('mediassist.camera_stream.time.monotonic',side_effect=[10.,10.1,10.2]):
            for value in (b'first',b'second',b'latest'):self.frames.publish(value,'now',encode_ms=2.5)
        with patch('mediassist.camera_stream.time.monotonic',return_value=10.2):
            self.assertEqual(self.frames.next(0)[2],b'latest')
            status=self.frames.status()
        self.assertAlmostEqual(status['encodedFps'],10.)
        self.assertEqual(status['targetFps'],15.)
        self.assertEqual(status['lastEncodeMs'],2.5)

    def test_pacer_limits_encoding_and_never_replays_a_stall_backlog(self):
        pacer=EncodePacer(15.)
        self.assertTrue(pacer.due(0.));pacer.encoded(0.)
        self.assertFalse(pacer.due(.06));self.assertTrue(pacer.due(.067))
        pacer.encoded(2.)
        self.assertFalse(pacer.due(2.01));self.assertTrue(pacer.due(2.067))

    def test_health_separates_source_rate_encode_rate_and_receive_age(self):
        frames=Frames('/camera',target_fps=20.,max_frame_age=.35)
        for stamp in (10.,10.05,10.10):frames.observe_source(640,480,stamp)
        with patch('mediassist.camera_stream.time.monotonic',side_effect=[10.02,10.12]):
            frames.publish(b'first','now',2.,received_at=10.,width=320,height=240)
            frames.publish(b'latest','now',3.,received_at=10.10,width=320,height=240)
        with patch('mediassist.camera_stream.time.monotonic',return_value=10.12):
            status=frames.status()
        self.assertEqual(status['sourceReceivedFps'],20.)
        self.assertEqual(status['encodedFps'],10.)
        self.assertEqual(status['sourceSize'],[640,480])
        self.assertEqual(status['streamSize'],[320,240])
        self.assertEqual(status['lastReceiveToEncodeMs'],20.)
        self.assertEqual(status['lastJpegBytes'],6)

    def test_recently_encoded_but_old_input_is_not_served_as_live(self):
        frames=Frames('/camera',max_frame_age=.35)
        with patch('mediassist.camera_stream.time.monotonic',return_value=10.):
            frames.publish(b'old','now',received_at=9.)
        with patch('mediassist.camera_stream.time.monotonic',return_value=10.):
            self.assertFalse(frames.status()['frameFresh'])
            self.assertIsNone(frames.next(-1,timeout=0.))

    def test_stream_sample_is_latest_even_when_source_clock_jitters(self):
        preview=object.__new__(CameraPreview)
        preview.cfg={'rate_hz':15.,'sample_every_frame':True}
        preview.active=lambda:True;preview.last_sample=0.;preview.error=None
        preview.lock=threading.Lock();preview.sample=None
        def image(value):return SimpleNamespace(width=1,height=1,step=3,encoding='rgb8',data=value)
        with patch('mediassist.camera_preview.time.monotonic',side_effect=[10.,10.06]):
            preview.on_image(image(b'123'));preview.on_image(image(b'456'))
        self.assertEqual(preview.sample[0],b'456')
        preview.cfg['sample_every_frame']=False
        with patch('mediassist.camera_preview.time.monotonic',return_value=10.065):
            preview.on_image(image(b'789'))
        self.assertEqual(preview.sample[0],b'456','MQTT preview keeps its original rate limit')

    def test_mqtt_wrapper_keeps_base64_contract_while_stream_uses_raw_jpeg(self):
        frame={'jpeg':b'jpeg-bytes','captured_at':'now','width':320,'height':240}
        with patch('mediassist.camera_preview.encode_jpeg',return_value=dict(frame)):
            payload=encode_frame(None,{})
        self.assertNotIn('jpeg',payload)
        self.assertEqual(payload['encoding'],'jpeg-base64')
        self.assertEqual(base64.b64decode(payload['data']),b'jpeg-bytes')

    def test_real_rgb_encoder_keeps_color_size_and_mqtt_payload_compatible(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest('OpenCV/NumPy are supplied by ROS on Pi; install them for image-codec tests')
        rgb=np.zeros((240,320,3),dtype=np.uint8);rgb[:,:,0]=255
        sample=(rgb.tobytes(),320,240,960,'rgb8','captured',10.)
        cfg={'max_width':160,'max_height':120,'jpeg_quality':50,'max_jpeg_bytes':65536}
        direct=encode_jpeg(sample,cfg)
        mqtt=encode_frame(sample,cfg)
        self.assertEqual(base64.b64decode(mqtt['data']),direct['jpeg'])
        self.assertEqual((direct['width'],direct['height']),(160,120))
        decoded=cv2.imdecode(np.frombuffer(direct['jpeg'],dtype=np.uint8),cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape,(120,160,3))
        self.assertGreater(int(decoded[60,80,2]),240)
        self.assertLess(int(decoded[60,80,0]),10)
        self.assertEqual(mqtt['captured_at'],'captured')


if __name__=='__main__':unittest.main(verbosity=2)
