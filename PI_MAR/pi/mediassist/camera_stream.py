"""Read-only LAN MJPEG from the configured ROS camera; no motor/MQTT access."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import html
import json
import logging
import socket
import threading
import time
from urllib.parse import parse_qs, quote, urlsplit


class Frames:
    def __init__(self, topic, max_clients=4, target_fps=15., max_frame_age=2.):
        self.topic, self.max_clients = topic, max_clients
        self.target_fps = target_fps
        self.published = deque(maxlen=151)
        self.received = deque(maxlen=151)
        self.source_size = self.output_size = None
        self.input_age = 0.
        self.jpeg_bytes = None
        self.client_write_ms = None
        self.max_frame_age = max_frame_age
        self.encode_ms = None
        self.condition = threading.Condition()
        self.clients = 0
        self.revision = 0
        self.frame = None
        self.closed = False

    def enter(self):
        with self.condition:
            if self.closed or self.clients >= self.max_clients:
                return False
            self.clients += 1
            return True

    def leave(self):
        with self.condition:
            self.clients = max(0, self.clients - 1)

    def viewing(self):
        with self.condition:
            return self.clients > 0 and not self.closed

    def observe_source(self, width, height, received_at):
        with self.condition:
            self.received.append(received_at)
            self.source_size = [width, height]

    def record_write(self, elapsed_ms):
        with self.condition:
            self.client_write_ms = round(elapsed_ms, 1)

    def publish(self, jpeg, captured_at, encode_ms=None, received_at=None, width=None, height=None):
        if not jpeg or len(jpeg) > 65536:
            raise ValueError('JPEG must be between 1 byte and 64 KiB')
        with self.condition:
            self.revision += 1
            self.frame = (self.revision, time.monotonic(), bytes(jpeg), captured_at)
            self.input_age = max(0., self.frame[1]-received_at) if received_at is not None else 0.
            self.output_size = [width,height] if width is not None and height is not None else None
            self.jpeg_bytes = len(jpeg)
            self.published.append(self.frame[1])
            self.encode_ms = encode_ms
            self.condition.notify_all()

    def next(self, revision, timeout=5.):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.closed:
                if self.frame and self.frame[0] > revision and time.monotonic() - self.frame[1] + self.input_age <= self.max_frame_age:
                    return self.frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
        return None

    def status(self):
        with self.condition:
            now = time.monotonic()
            age = None if not self.frame else round(now - self.frame[1] + self.input_age, 3)
            def rate(stamps):
                recent = [stamp for stamp in stamps if now-stamp <= 5.]
                return round((len(recent)-1)/(recent[-1]-recent[0]),1) if len(recent)>1 and recent[-1]>recent[0] else 0.
            return {'topic': self.topic, 'qos': 'BEST_EFFORT / VOLATILE / depth=1',
                    'viewers': self.clients, 'frameRevision': self.revision,
                    'targetFps':self.target_fps,'encodedFps':rate(self.published),'lastEncodeMs':self.encode_ms,
                    'sourceReceivedFps':rate(self.received),'sourceSize':self.source_size,'streamSize':self.output_size,
                    'lastJpegBytes':self.jpeg_bytes,'lastReceiveToEncodeMs':round(self.input_age*1000,1) if self.frame else None,
                    'lastClientWriteMs':self.client_write_ms,'maxFrameAgeSeconds':self.max_frame_age,
                    'frameAgeSeconds': age, 'frameFresh': age is not None and age <= self.max_frame_age}

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


def handler_for(frames):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(5.)
            self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            # Bound per-viewer TCP backlog; slow viewers do not queue more JPEGs in Python.
            self.connection.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,16384)

        def log_message(self, *_):
            pass

        def reply(self, code, body, content_type='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if len(self.path) > 2048:
                self.reply(414, b'{"error":"URL_TOO_LONG"}'); return
            uri = urlsplit(self.path)
            if uri.path == '/health':
                self.reply(200, json.dumps(frames.status()).encode()); return
            if uri.path == '/':
                topic = html.escape(frames.topic)
                stream = '/stream?topic=' + quote(frames.topic, safe='')
                body = ('<!doctype html><meta charset="utf-8"><title>Robot camera</title>'
                        '<h1>Robot camera</h1><p>Topic: ' + topic + '</p>'
                        '<p>BEST_EFFORT; no JPEG encoding until a viewer connects.</p>'
                        '<p><a href="/health">Camera status</a></p>'
                        '<img alt="Waiting for ROS camera frames" src="' + stream + '">').encode()
                self.reply(200, body, 'text/html; charset=utf-8'); return
            if uri.path not in ('/stream', '/snapshot'):
                self.reply(404, b'{"error":"NOT_FOUND"}'); return
            topic = parse_qs(uri.query).get('topic', [frames.topic])[0]
            if topic != frames.topic:
                self.reply(400, json.dumps({'error':'WRONG_CAMERA_TOPIC','topic':frames.topic}).encode()); return
            if not frames.enter():
                self.reply(503, b'{"error":"CAMERA_VIEWER_LIMIT"}'); return
            try:
                frame = frames.next(-1)
                if frame is None:
                    self.reply(503, b'{"error":"CAMERA_TOPIC_NO_FRAMES","hint":"Check ROS topic, encoding and camera driver"}')
                    return
                if uri.path == '/snapshot':
                    self.reply(200, frame[2], 'image/jpeg'); return
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Accel-Buffering', 'no')
                self.send_header('Connection', 'close')
                self.end_headers()
                while frame is not None:
                    started = time.monotonic()
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                     str(len(frame[2])).encode() + b'\r\n\r\n' + frame[2] + b'\r\n')
                    self.wfile.flush()
                    frames.record_write((time.monotonic()-started)*1000)
                    frame = frames.next(frame[0], timeout=3.)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
            finally:
                frames.leave()
    return Handler


class CameraServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class EncodePacer:
    """Bound encoding rate without queuing stale frames or catching up after a stall."""
    def __init__(self, fps):
        self.interval = 1./fps
        self.deadline = 0.

    def due(self, now):
        return now >= self.deadline

    def encoded(self, started):
        self.deadline = started + self.interval


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    from .worker_config import load
    cfg, _ = load(args.config)
    camera = dict(cfg['mapping']['camera'])
    camera['rate_hz'] = camera['stream_rate_hz']
    for key in ('max_width','max_height','jpeg_quality'):
        camera[key] = camera['stream_'+key]
    # Sample the latest source frame independently of the encoder cadence. Two
    # nearly equal 15 Hz clocks otherwise drop alternating camera messages.
    camera['sample_every_frame'] = True
    frames = Frames(camera['topic'],target_fps=camera['stream_rate_hz'],max_frame_age=camera['stream_max_frame_age_sec'])
    pacer = EncodePacer(camera['stream_rate_hz'])
    server = CameraServer((camera['stream_bind'], camera['stream_port']), handler_for(frames))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    import rclpy
    from types import SimpleNamespace
    from .camera_preview import CameraPreview, encode_jpeg
    import cv2
    cv2.setNumThreads(1)
    try:
        import os
        os.nice(5)
    except (AttributeError,OSError):
        pass
    rclpy.init()
    node = rclpy.create_node('mediassist_camera_stream')
    preview = CameraPreview(SimpleNamespace(node=node), camera, frames.viewing, frames.observe_source)
    print('CAMERA_HTTP_READY port=' + str(server.server_address[1]) + ' topic=' + camera['topic'] +
          ' target_fps=' + str(camera['stream_rate_hz']) + ' QoS=BEST_EFFORT; waiting for viewers/ROS frames', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.005)
            if not frames.viewing() or not pacer.due(time.monotonic()):
                continue
            with preview.lock:
                sample = preview.sample
                preview.sample = None
            if sample and frames.viewing() and time.monotonic() - sample[-1] <= camera['stream_max_frame_age_sec']:
                try:
                    started = time.monotonic()
                    pacer.encoded(started)
                    encoded = encode_jpeg(sample, camera)
                    if encoded:
                        frames.publish(encoded['jpeg'], encoded['captured_at'],
                                       round((time.monotonic()-started)*1000,1),
                                       received_at=sample[-1],width=encoded['width'],height=encoded['height'])
                except Exception as exc:
                    logging.warning('Camera encoding failed: %s', type(exc).__name__)
    except KeyboardInterrupt:
        pass
    finally:
        frames.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
