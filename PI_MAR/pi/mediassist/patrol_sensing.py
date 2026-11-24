"""Stationary restricted-zone observation from a commissioned RGB-D camera."""
import asyncio
from collections import deque
from datetime import datetime, timezone
import math
from pathlib import Path
import threading
import time

from .contracts import StepError
from .navigation_zones import signed_distance


def configuration(ai, camera):
    provider = ai.get('restricted_zone')
    if provider is None:
        return None
    if (not ai['enabled'] or not camera['enabled'] or not isinstance(provider, dict)
            or provider.get('registered_depth_verified') is not True):
        raise StepError('INVALID_RESTRICTED_ZONE_CONFIG')
    for key, limit in (('model_version', 100), ('producer_ref', 255), ('model_path', 1024),
                       ('depth_topic', 255), ('camera_info_topic', 255)):
        value = provider.get(key)
        if not isinstance(value, str) or not value.strip() or len(value) > limit:
            raise StepError('INVALID_RESTRICTED_ZONE_CONFIG', key)
    model = Path(provider['model_path'])
    if not model.is_absolute() or model.suffix.lower() != '.onnx':
        raise StepError('INVALID_RESTRICTED_ZONE_CONFIG', 'model_path')
    for key in ('depth_topic', 'camera_info_topic'):
        if not provider[key].startswith('/') or any(c.isspace() for c in provider[key]):
            raise StepError('INVALID_RESTRICTED_ZONE_CONFIG', key)
    frame = provider.get('registered_depth_frame')
    if frame is not None and (not isinstance(frame, str) or not frame or len(frame) > 255
                              or frame.startswith('/') or any(c.isspace() for c in frame)):
        raise StepError('INVALID_RESTRICTED_ZONE_CONFIG', 'registered_depth_frame')
    size = provider.setdefault('input_size', 416)
    if type(size) is not int or not 320 <= size <= 640 or size % 32:
        raise StepError('INVALID_RESTRICTED_ZONE_CONFIG', 'input_size')
    floor = provider.setdefault('floor_z_m', 0.)
    if type(floor) not in (int, float) or not math.isfinite(floor) or abs(floor) > 5.:
        raise StepError('INVALID_RESTRICTED_ZONE_CONFIG', 'floor_z_m')
    if type(provider.setdefault('rgb_rectified', False)) is not bool:
        raise StepError('INVALID_RESTRICTED_ZONE_CONFIG', 'rgb_rectified')
    return provider


class PoseDetector:
    def __init__(self, path, size):
        if not Path(path).is_file():
            raise StepError('AI_MODEL_UNAVAILABLE')
        try:
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            options.add_session_config_entry('session.intra_op.allow_spinning', '0')
            self.session = ort.InferenceSession(path, sess_options=options,
                                                providers=['CPUExecutionProvider'])
            inputs = self.session.get_inputs()
            metadata = self.session.get_modelmeta().custom_metadata_map
            if (len(inputs) != 1 or inputs[0].type != 'tensor(float)' or len(inputs[0].shape) != 4
                    or metadata.get('task') != 'pose' or metadata.get('kpt_shape') != '[17, 3]'
                    or any(isinstance(actual, int) and actual != expected
                           for actual, expected in zip(inputs[0].shape, (1, 3, size, size)))):
                raise ValueError('Expected a YOLOv8n pose export without integrated NMS')
            self.input_name, self.size = inputs[0].name, size
        except Exception as exc:
            raise StepError('AI_MODEL_UNAVAILABLE', type(exc).__name__) from exc

    def ankles(self, image):
        import cv2
        import numpy as np

        height, width = image.shape[:2]
        scale = self.size / max(height, width)
        resized = cv2.resize(image, (round(width * scale), round(height * scale)))
        if resized.ndim == 2:
            resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
        h, w = resized.shape[:2]
        x, y = (self.size - w) // 2, (self.size - h) // 2
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[y:y+h, x:x+w] = resized[:, :, ::-1]
        tensor = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None], dtype=np.float32) / 255.
        output = self.session.run(None, {self.input_name: tensor})
        if len(output) != 1 or output[0].ndim != 3 or output[0].shape[:2] != (1, 56):
            raise StepError('AI_MODEL_OUTPUT_UNSUPPORTED')
        rows = output[0][0].T
        if not np.isfinite(rows).all():
            raise StepError('AI_MODEL_OUTPUT_UNSUPPORTED')
        rows = rows[rows[:, 4] >= .4]
        if len(rows) == 0:
            return []
        if len(rows) > 256 or (rows[:, 2:4] <= 0).any():
            raise StepError('AI_MODEL_OUTPUT_UNAVAILABLE')
        boxes = rows[:, :4].copy()
        boxes[:, :2] -= boxes[:, 2:] / 2
        indexes = cv2.dnn.NMSBoxes(boxes.tolist(), rows[:, 4].tolist(), .39, .45)
        people = rows[np.asarray(indexes, dtype=int).reshape(-1), 5:].reshape(-1, 17, 3)[:, 15:17].copy()
        people[:, :, 0] = (people[:, :, 0] - x) * width / w
        people[:, :, 1] = (people[:, :, 1] - y) * height / h
        return people


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class Frames:
    def __init__(self):
        self.lock = threading.Lock()
        self.images = [deque(maxlen=4), deque(maxlen=4)]
        self.info = None

    def receive(self, msg, kind):
        with self.lock:
            if kind == 2:
                self.info = msg
            else:
                self.images[kind].append(msg)

    def pair(self, after):
        with self.lock:
            if self.info is None:
                return None
            depths = [sample for sample in self.images[1] if stamp_ns(sample) > after]
            if not depths:
                return None
            for rgb in reversed(self.images[0]):
                stamp = stamp_ns(rgb)
                if stamp <= after:
                    continue
                depth = min(depths, key=lambda sample: abs(stamp_ns(sample) - stamp))
                if abs(stamp_ns(depth) - stamp) <= 40_000_000:
                    return rgb, depth, self.info
        return None


def calibration(info, rgb, rectified):
    import numpy as np

    if (info.header.frame_id != rgb.header.frame_id or (info.width, info.height) != (rgb.width, rgb.height)
            or info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
            or info.roi.x_offset or info.roi.y_offset or info.roi.width not in (0, rgb.width)
            or info.roi.height not in (0, rgb.height)):
        raise StepError('AI_CAMERA_CALIBRATION_UNAVAILABLE')
    if rectified:
        projection = np.asarray(info.p).reshape(3, 4)
        if np.any(projection[:, 3]):
            raise StepError('AI_CAMERA_CALIBRATION_UNAVAILABLE')
        matrix, distortion = projection[:, :3].copy(), None
    else:
        matrix = np.asarray(info.k).reshape(3, 3)
        distortion = np.asarray(info.d)
        lengths = {'plumb_bob': (0, 4, 5), 'rational_polynomial': (8, 12, 14), '': (0,)}
        if len(distortion) not in lengths.get(info.distortion_model, ()) or not np.isfinite(distortion).all():
            raise StepError('AI_CAMERA_CALIBRATION_UNAVAILABLE')
        if not len(distortion):
            distortion = None
    if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise StepError('AI_CAMERA_CALIBRATION_UNAVAILABLE')
    return matrix, distortion


def camera_transform(transform):
    import numpy as np

    t, q = transform.translation, transform.rotation
    offset = np.asarray((t.x, t.y, t.z), dtype=float)
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not np.isfinite(offset).all() or not .99 <= norm <= 1.01:
        raise StepError('AI_CAMERA_TF_UNAVAILABLE')
    x, y, z, w = (value / norm for value in (q.x, q.y, q.z, q.w))
    rotation = np.array(((1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)),
                         (2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)),
                         (2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y))))
    return rotation, offset


def depth_image(depth, rgb, registered_frame):
    import numpy as np

    encodings = {'16UC1': ('u2', .001), '32FC1': ('f4', 1.)}
    if (depth.encoding not in encodings or depth.header.frame_id != registered_frame
            or (depth.width, depth.height) != (rgb.width, rgb.height)):
        raise StepError('AI_REGISTERED_DEPTH_UNAVAILABLE')
    code, scale = encodings[depth.encoding]
    dtype = np.dtype(('>' if depth.is_bigendian else '<') + code)
    if depth.step < depth.width * dtype.itemsize or len(depth.data) != depth.height * depth.step:
        raise StepError('AI_REGISTERED_DEPTH_UNAVAILABLE')
    values = np.ndarray((depth.height, depth.width), dtype=dtype, buffer=depth.data,
                        strides=(depth.step, dtype.itemsize))
    return values.astype(np.float32) * scale


def depth_at(depth, u, v):
    import numpy as np

    if not (math.isfinite(u) and math.isfinite(v)):
        return None
    x, y = round(u), round(v)
    h, w = depth.shape
    if not (2 <= x < w - 2 and 2 <= y < h - 2):
        return None
    patch = depth[y-2:y+3, x-2:x+3]
    valid = patch[(patch >= .2) & (patch <= 4.)]
    if len(valid) < 15:
        return None
    median = float(np.median(valid))
    return median if np.count_nonzero(np.abs(valid - median) <= .08) >= 15 else None


def zone_coverage_points(polygon, floor):
    min_x, max_x = min(p[0] for p in polygon), max(p[0] for p in polygon)
    min_y, max_y = min(p[1] for p in polygon), max(p[1] for p in polygon)
    nx, ny = max(1, math.ceil((max_x-min_x)/.2)), max(1, math.ceil((max_y-min_y)/.2))
    if nx * ny > 625:
        return None
    vertices = [(*p, floor) for p in polygon]
    for ix in range(nx):
        for iy in range(ny):
            x = min_x + (ix+.5) * (max_x-min_x) / nx
            y = min_y + (iy+.5) * (max_y-min_y) / ny
            if signed_distance((x, y), polygon) <= 0:
                vertices.append((x, y, floor))
    return vertices


def zone_visible(pair, transform, coverage, config):
    """A negative shared observation is valid only if the whole zone floor is visible."""
    import cv2
    import numpy as np
    if coverage is None:return False
    rgb,depth,info,_=pair
    values=depth_image(depth,rgb,rgb.header.frame_id)
    matrix,distortion=calibration(info,rgb,config['rgb_rectified'])
    rotation,offset=camera_transform(transform)
    points=(np.asarray(coverage)-offset)@rotation
    if (points[:,2]<.2).any() or (points[:,2]>4.).any():return False
    pixels=cv2.projectPoints(points,np.zeros(3),np.zeros(3),matrix,distortion)[0].reshape(-1,2)
    for (u,v),point in zip(pixels,points):
        depth=depth_at(values,u,v)
        if depth is None or abs(depth-point[2])>.18:return False
    return True


def classify(detector, rgb, depth, info, transform, polygon, coverage, config):
    import cv2
    import numpy as np
    from .camera_preview import frame_pixels

    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(rgb.encoding)
    if (not channels or not 0 < rgb.width * rgb.height <= 1_000_000
            or rgb.step < rgb.width * channels or len(rgb.data) != rgb.step * rgb.height
            or len(rgb.data) > 4_000_000 or len(depth.data) > 4_000_000):
        raise StepError('AI_CAMERA_FORMAT_UNSUPPORTED')
    frame = frame_pixels((rgb.data, rgb.width, rgb.height, rgb.step, rgb.encoding, None, None))
    depths = depth_image(depth, rgb, config.get('registered_depth_frame') or rgb.header.frame_id)
    matrix, distortion = calibration(info, rgb, config['rgb_rectified'])
    rotation, offset = camera_transform(transform)
    unknown = False
    for ankles in detector.ankles(frame):
        locations = []
        for u, v, confidence in ankles:
            if confidence < .45:
                continue
            z = depth_at(depths, u, v)
            if z is None:
                continue
            pixels = np.asarray([[[u, v]]], dtype=np.float64)
            x, y = cv2.undistortPoints(pixels, matrix, distortion).reshape(2)
            point = rotation @ (x*z, y*z, z) + offset
            if np.isfinite(point).all() and abs(point[2] - config['floor_z_m']) <= .30:
                locations.append(signed_distance(point[:2], polygon))
        if any(gap < -.08 for gap in locations):
            return True
        if len(locations) != 2 or any(gap <= .08 for gap in locations):
            unknown = True
    if unknown or coverage is None:
        return None
    # A view of only part of the zone cannot be reported as clear.
    points = (np.asarray(coverage) - offset) @ rotation
    if (points[:, 2] < .2).any() or (points[:, 2] > 4.).any():
        return None
    image_points = cv2.projectPoints(points, np.zeros(3), np.zeros(3), matrix, distortion)[0].reshape(-1, 2)
    for (u, v), (_, _, expected) in zip(image_points, points):
        z = depth_at(depths, u, v)
        if z is None or abs(z-expected) > .18:
            return None
    return False


class RestrictedZoneObserver:
    def __init__(self, adapter, config, rgb_topic):
        self.adapter, self.config, self.rgb_topic = adapter, config, rgb_topic
        self.detector = self.work = None

    def load_detector(self):
        if self.detector is None:
            self.detector = PoseDetector(self.config['model_path'], self.config['input_size'])

    async def run(self, deadline, function, *args):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise StepError('AI_DATA_UNAVAILABLE', retryable=True)
        if self.work is not None and not self.work.done():
            raise StepError('AI_PROVIDER_BUSY', retryable=True)
        self.work = asyncio.create_task(asyncio.to_thread(function, *args))
        # Retain at most one inference even when a job cancels while ORT is running.
        self.work.add_done_callback(lambda task: task.exception() if not task.cancelled() else None)
        try:
            return await asyncio.wait_for(asyncio.shield(self.work), remaining)
        except asyncio.TimeoutError as exc:
            raise StepError('AI_DATA_UNAVAILABLE', 'RGB-D inference exceeded the observation window', retryable=True) from exc

    def revision(self):
        state = self.adapter.telemetry()
        localization = state.get('localization', {})
        odom = state.get('odom_speed', {})
        if (localization.get('ready') is True and 0 <= localization.get('age_sec', math.inf) <= 1.25
                and state.get('map_pose', {}).get('fresh') is True and 0 <= odom.get('age_sec', math.inf) <= .5
                and 0 <= odom.get('value', math.inf) <= .01):
            return localization['session_id'], localization['initialization_revision']
        return None

    async def observe(self, params):
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import TransformException

        adapter, cfg = self.adapter, self.config
        window = params.get('window_sec', 0)
        if window < 1.:
            raise StepError('AI_OBSERVATION_WINDOW_REQUIRED')
        profile = getattr(adapter, 'navigation_profile_hash', None)
        zones = getattr(adapter, 'zones', None)
        polygon = next((zone['points'] for zone in zones.zones if zone['kind'] == 'KEEP_OUT'
                        and zone.get('zone_id') == params['zone_id']), None) if profile and zones else None
        if polygon is None or any(abs(v) > 100000. for point in polygon for v in point):
            raise StepError('AI_RESTRICTED_ZONE_NOT_SYNCED', retryable=True)
        deadline = time.monotonic() + window
        node = adapter.node
        frame_id = adapter.cfg['ros']['map_frame']
        subscriptions = []
        previous, consecutive, revision = None, 0, None
        reason = 'No synchronized RGB-D sample'
        frames = Frames()
        coverage = zone_coverage_points(polygon, cfg['floor_z_m'])
        try:
            await self.run(deadline, self.load_detector)
            after = node.get_clock().now().nanoseconds
            for kind, (msg_type, topic) in enumerate(((Image, self.rgb_topic), (Image, cfg['depth_topic']),
                                                       (CameraInfo, cfg['camera_info_topic']))):
                subscriptions.append(node.create_subscription(msg_type, topic,
                    lambda msg, kind=kind: frames.receive(msg, kind), qos_profile_sensor_data))
            while time.monotonic() < deadline:
                await asyncio.sleep(.05)
                current = self.revision()
                if current is None or current != revision:
                    previous, consecutive, revision = None, 0, current
                    after = node.get_clock().now().nanoseconds
                    reason = 'Waiting for a stable stationary map pose'
                    continue
                pair = frames.pair(after)
                if pair is None:
                    continue
                rgb, depth, info = pair
                stamp = stamp_ns(rgb)
                after = stamp + 150_000_000
                if not -.05 <= (node.get_clock().now().nanoseconds - stamp) / 1e9 <= .60:
                    previous, consecutive = None, 0
                    reason = 'RGB-D sample timestamp is stale'
                    continue
                try:
                    tf = adapter.tf_buffer.lookup_transform(frame_id, rgb.header.frame_id, Time.from_msg(rgb.header.stamp))
                    result = await self.run(deadline, classify, self.detector, rgb, depth, info,
                                            tf.transform, polygon, coverage, cfg)
                except TransformException:
                    previous, consecutive = None, 0
                    reason = 'Camera TF unavailable at the RGB timestamp'
                    continue
                except StepError as exc:
                    if exc.code not in ('AI_CAMERA_CALIBRATION_UNAVAILABLE', 'AI_REGISTERED_DEPTH_UNAVAILABLE',
                                        'AI_CAMERA_TF_UNAVAILABLE', 'AI_CAMERA_FORMAT_UNSUPPORTED'):
                        raise
                    previous, consecutive = None, 0
                    reason = exc.code
                    continue
                age = (node.get_clock().now().nanoseconds - stamp) / 1e9
                if (self.revision() != revision or adapter.navigation_profile_hash != profile
                        or not 0 <= age <= min(1.5, adapter.worker['ai']['max_age_sec'])):
                    previous, consecutive = None, 0
                    reason = 'Pose, profile or image changed during inference'
                    continue
                if result is None:
                    previous, consecutive = None, 0
                    reason = 'Zone coverage or person location is uncertain'
                    continue
                consecutive = consecutive + 1 if result is previous else 1
                previous = result
                if consecutive == 2:
                    return {'sensing_type': 'RESTRICTED_ZONE', 'map_id': adapter.cfg['map_id'],
                            'zone_id': params['zone_id'], 'frame_id': frame_id, 'model_version': cfg['model_version'],
                            'producer_ref': cfg['producer_ref'],
                            'captured_at': datetime.fromtimestamp(stamp / 1e9, timezone.utc).isoformat(),
                            'metrics': {'intrusion_detected': result}, 'evidence_ref': None}
            raise StepError('AI_DATA_UNAVAILABLE', reason, retryable=True)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
