"""Measure fall keypoints in registered depth before selecting a reference frame."""
from collections import deque
import math
from types import SimpleNamespace
from threading import Lock

from .contracts import StepError
from .patrol_sensing import calibration, camera_transform, depth_at, depth_image, stamp_ns


MAX_SKEW_NS = 40_000_000


def configuration(config):
    if config is None:
        return None
    if not isinstance(config, dict) or type(config.get('enabled')) is not bool:
        raise StepError('INVALID_PATROL_FALL_DEPTH_CONFIG')
    if not config['enabled']:
        return None
    if config.get('registered_depth_verified') is not True:
        raise StepError('PATROL_FALL_DEPTH_REGISTRATION_UNVERIFIED')
    for key in ('depth_topic', 'rgb_info_topic', 'depth_info_topic'):
        value = config.get(key)
        if not isinstance(value, str) or not value.startswith('/') or len(value) < 2 or any(c.isspace() for c in value):
            raise StepError('INVALID_PATROL_FALL_DEPTH_CONFIG', key)
    floor = config.get('floor_z_m', 0.)
    near_floor = config.get('near_floor_max_height_m', .45)
    end_on_height = config.get('end_on_max_height_m', .30)
    rectified = config.get('rgb_rectified', False)
    if (type(floor) not in (float, int) or not math.isfinite(floor) or abs(floor) > 5 or type(rectified) is not bool
            or type(near_floor) not in (float, int) or not math.isfinite(near_floor) or not .15 <= near_floor <= .45
            or type(end_on_height) not in (float, int) or not math.isfinite(end_on_height) or not .15 <= end_on_height <= .45):
        raise StepError('INVALID_PATROL_FALL_DEPTH_CONFIG')
    return {**config, 'floor_z_m': floor, 'rgb_rectified': rectified,
            'near_floor_max_height_m': near_floor, 'end_on_max_height_m': end_on_height}


class DepthFrames:
    """Subscribe only while the robot is observing, with bounded sensor history."""
    def __init__(self, node, config):
        self.node, self.config = node, config
        self.lock = Lock()
        self.frames = [deque(maxlen=6) for _ in range(3)]
        self.subscriptions = []

    def start(self):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        topics = ((Image, 'depth_topic'), (CameraInfo, 'rgb_info_topic'), (CameraInfo, 'depth_info_topic'))
        for index, (kind, topic) in enumerate(topics):
            self.subscriptions.append(self.node.create_subscription(
                kind, self.config[topic], lambda msg, i=index: self.receive(i, msg), qos_profile_sensor_data))

    def receive(self, index, msg):
        # Bound memory before retaining an untrusted or reconfigured ROS image.
        if index == 0 and (not 0 < msg.width * msg.height <= 640 * 480
                           or len(msg.data) > 640 * 480 * 4):
            return
        with self.lock:
            self.frames[index].append(msg)

    def for_rgb(self, header, width, height):
        if header is None or not header.frame_id:
            return None
        rgb = SimpleNamespace(header=header, width=width, height=height)
        stamp = stamp_ns(rgb)
        if stamp <= 0:
            return None
        with self.lock:
            matches = [min((msg for msg in frames if msg.header.frame_id == header.frame_id),
                           key=lambda msg: abs(stamp_ns(msg) - stamp), default=None)
                       for frames in self.frames]
        if any(msg is None or abs(stamp_ns(msg) - stamp) > MAX_SKEW_NS for msg in matches):
            return None
        return (rgb, *matches)

    def close(self):
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()


def measure_person(pair, keypoints, config, diagnostics=None):
    import cv2
    import numpy as np

    def unavailable(reason, **metrics):
        if diagnostics is not None:diagnostics.update(reason=reason, **metrics)
        return None
    rgb, depth, rgb_info, depth_info = pair
    skew_ns = abs(stamp_ns(depth) - stamp_ns(rgb))
    if skew_ns > MAX_SKEW_NS:return unavailable('DEPTH_FRAME_SKEW', skew_ms=round(skew_ns/1e6,3))
    if not 0 < rgb.width * rgb.height <= 1_000_000 or len(depth.data) > 4_000_000:
        return unavailable('DEPTH_FRAME_INVALID')
    matrix, distortion = calibration(rgb_info, rgb, config['rgb_rectified'])
    depth_matrix, depth_distortion = calibration(depth_info, rgb, config['rgb_rectified'])
    if (not np.allclose(matrix, depth_matrix, rtol=1e-4, atol=1e-6)
            or (distortion is None) != (depth_distortion is None)
            or distortion is not None and (distortion.shape != depth_distortion.shape
                                           or not np.allclose(distortion, depth_distortion, atol=1e-6))):
        return unavailable('DEPTH_INTRINSICS_MISMATCH')
    values = depth_image(depth, rgb, rgb.header.frame_id)
    if not isinstance(keypoints, list) or len(keypoints)!=2:return unavailable('DEPTH_KEYPOINT_INVALID')
    camera_points = []
    used_pixels = []
    for keypoint in keypoints:
        if not isinstance(keypoint, dict):return unavailable('DEPTH_KEYPOINT_INVALID')
        x, y = keypoint.get('x'), keypoint.get('y')
        if any(type(v) not in (float, int) or not math.isfinite(v) or not 0 <= v < 1 for v in (x, y)):
            return unavailable('DEPTH_KEYPOINT_INVALID')
        u, v = x * rgb.width, y * rgb.height
        if any(math.hypot(u - a, v - b) < 5. for a, b in used_pixels):
            return unavailable('DEPTH_ANCHORS_OVERLAP')
        z = depth_at(values, u, v)
        if z is None:
            px,py=round(u),round(v)
            metrics=dict(anchor_index=len(camera_points))
            if 2<=px<rgb.width-2 and 2<=py<rgb.height-2:
                patch=values[py-2:py+3,px-2:px+3];valid=patch[(patch>=.2)&(patch<=4.)]
                metrics['valid_pixels']=int(len(valid))
                metrics['consistent_pixels']=int(np.count_nonzero(np.abs(valid-np.median(valid))<=.08)) if len(valid) else 0
            return unavailable('DEPTH_KEYPOINT_UNAVAILABLE',**metrics)
        ray_x, ray_y = cv2.undistortPoints(np.array([[[u, v]]], dtype=float), matrix, distortion).reshape(2)
        point = (ray_x * z, ray_y * z, z)
        if np.isfinite(point).all():
            camera_points.append(point)
            used_pixels.append((u, v))
    if len(camera_points) != 2:
        return unavailable('DEPTH_KEYPOINT_UNAVAILABLE', measured_anchors=len(camera_points))
    if diagnostics is not None:diagnostics.update(reason='READY', measured_anchors=2)
    return camera_points, skew_ns


def project_person(measurement, transform, floor_z, *, end_on=False, max_height=.30, diagnostics=None, hip_midpoint=False):
    def unavailable(reason):
        if diagnostics is not None:diagnostics['reason']=reason
        return None
    camera_points, skew_ns = measurement
    rotation, offset = camera_transform(transform)
    hip, support = (rotation @ point + offset for point in camera_points)
    span=math.hypot(*(hip-support)[:2])
    if span > (1.2 if end_on else .85) or abs(camera_points[0][2] - camera_points[1][2]) > (1.2 if end_on else .65):
        return unavailable('DEPTH_BODY_SPAN_INVALID')
    heights = (hip[2] - floor_z, support[2] - floor_z)
    if min(heights) < -.15:
        return unavailable('DEPTH_BELOW_FLOOR')
    if end_on:
        if span < .25:return unavailable('DEPTH_BODY_SPAN_INVALID')
        if abs(heights[0]-heights[1]) > .18 or max(heights) > max_height:
            return unavailable('PERSON_NOT_NEAR_FLOOR')
    # Two hips locate the pelvis midpoint; hip+knee/ankle keeps the hip as the
    # location anchor. Both original heights still corroborate floor proximity.
    # Preserve a reliable above-floor measurement so callers can reject it.
    height_m = max(heights)
    location=(hip+support)/2 if hip_midpoint and not end_on else hip
    optical_depth=(camera_points[0][2]+camera_points[1][2])/2 if hip_midpoint and not end_on else camera_points[0][2]
    return dict(xM=round(float(location[0]), 4), yM=round(float(location[1]), 4),
                rangeM=round(float(optical_depth), 4),
                depthSkewMs=round(skew_ns / 1e6, 3), floorHeightM=round(float(height_m), 4))


def project_ground(measurement, transform, floor_z):
    """Two observed ankles locate a standing person, independently of fall posture rules."""
    import numpy as np
    points, skew = measurement
    rotation, offset = camera_transform(transform)
    ground = [rotation @ point + offset for point in points]
    heights = [float(point[2]-floor_z) for point in ground]
    if (not np.isfinite(ground).all() or any(not -.15 <= z <= .30 for z in heights)
            or math.hypot(*(ground[0]-ground[1])[:2]) > 1.2):
        return None
    anchor = (ground[0]+ground[1])/2
    return dict(xM=round(float(anchor[0]),4), yM=round(float(anchor[1]),4),
                rangeM=round(float((points[0][2]+points[1][2])/2),4),
                depthSkewMs=round(skew/1e6,3), floorHeightM=round(max(heights),4))
