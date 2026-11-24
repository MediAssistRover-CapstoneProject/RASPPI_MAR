"""Bounded camera observation; inference stays on the server, independent of motion."""
import asyncio
import http.client
import json
import logging
import math
import re
import socket
import ssl
import threading
import time
import urllib.error
from pathlib import Path
from urllib.parse import urlsplit
from uuid import uuid4

from .camera_preview import CameraPreview, encode_jpeg
from .contracts import StepError
from .fall_depth import DepthFrames, configuration as depth_configuration, measure_person, project_person
from .navigation_session import SessionClient


def configuration(ai, camera):
    config = ai.get('patrol_fall')
    if config is None:
        return None
    if not isinstance(config, dict) or type(config.get('enabled')) is not bool:
        raise StepError('INVALID_PATROL_FALL_CONFIG')
    mode = config.get('mode', 'CHECKPOINT')
    if mode not in ('CHECKPOINT', 'CONTINUOUS'):
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'mode')
    if not config['enabled']:
        return None
    origin = config.get('base_url')
    if not isinstance(origin, str):
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'base_url')
    try:
        uri = urlsplit(origin)
        uri.port
    except ValueError:
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'base_url') from None
    if (not camera.get('enabled') or uri.scheme != 'https' or not uri.hostname
            or uri.path not in ('', '/') or uri.username or uri.password or uri.query or uri.fragment):
        raise StepError('INVALID_PATROL_FALL_CONFIG')
    for key in ('token_file', 'ca_file'):
        value = config.get(key)
        if key == 'ca_file' and value is None:
            continue
        if not isinstance(value, str) or not Path(value).is_absolute():
            raise StepError('INVALID_PATROL_FALL_CONFIG', key)
    rate = config.get('rate_hz', 4)
    if type(rate) not in (int, float) or not 2 <= rate <= 5:
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'rate_hz')
    memory = config.get('min_available_memory_mb', 512)
    cooldown = config.get('cooldown_sec', 60)
    request_timeout = config.get('request_timeout_sec', 1.5)
    if type(memory) is not int or not 128 <= memory <= 4096:
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'min_available_memory_mb')
    if type(cooldown) not in (int, float) or not math.isfinite(cooldown) or not 20 <= cooldown <= 600:
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'cooldown_sec')
    if type(request_timeout) not in (int, float) or not math.isfinite(request_timeout) or not .5 <= request_timeout <= 2.:
        raise StepError('INVALID_PATROL_FALL_CONFIG', 'request_timeout_sec')
    from .restricted_monitor import configuration as restricted_configuration
    restricted=restricted_configuration(config.get('restricted_monitor'))
    if restricted['enabled'] and (mode!='CONTINUOUS' or not (config.get('depth') or {}).get('enabled')):
        raise StepError('SHARED_RESTRICTED_REQUIRES_CONTINUOUS_DEPTH')
    return {**config, 'mode': mode, 'base_url': uri.scheme + '://' + uri.netloc, 'rate_hz': rate,
            'min_available_memory_mb': memory, 'cooldown_sec': cooldown, 'request_timeout_sec': request_timeout,
            'depth': depth_configuration(config.get('depth')), 'restricted_monitor':restricted}


def read_token(path):
    try:
        token = Path(path).read_text().strip()
    except (OSError, UnicodeError):
        raise StepError('PATROL_FALL_TOKEN_UNAVAILABLE') from None
    if len(token) < 32 or not token.isascii() or any(c.isspace() for c in token):
        raise StepError('PATROL_FALL_TOKEN_INVALID')
    return token


class Capture:
    """ROS owns creation/destruction; at most one RGB and six bounded depth frames."""
    def __init__(self, observer, active):
        self.observer, self.active = observer, active
        self.preview = self.depth = None

    async def set_enabled(self, enabled):
        adapter = self.observer.adapter
        executor = adapter.node.executor
        if executor is None or not adapter.thread.is_alive():
            if enabled:
                raise StepError('PATROL_FALL_ROS_UNAVAILABLE')
            self.preview = self.depth = None
            return
        waiting = True

        def change():
            try:
                if enabled and waiting and self.preview is None:
                    cfg = self.observer.config.get('depth')
                    if cfg is not None:
                        self.depth = DepthFrames(adapter.node, cfg)
                        self.depth.start()
                    self.preview = CameraPreview(adapter, self.observer.camera, self.active,
                                                 retain_header=cfg is not None)
                elif not enabled:
                    if self.preview is not None:
                        adapter.node.destroy_subscription(self.preview.subscription)
                        self.preview = None
                    if self.depth is not None:
                        self.depth.close()
                        self.depth = None
            except Exception as error:
                return error
            return None

        task = executor.create_task(change)
        try:
            error = await adapter.future(task, timeout=2.)
        finally:
            waiting = False
        if error is not None:
            raise error


def enough_memory(minimum_mb):
    """Pressure only suspends optional inference; never limit the motion worker's RSS."""
    try:
        for row in Path('/proc/meminfo').read_text().splitlines():
            if row.startswith('MemAvailable:'):
                return int(row.split()[1]) >= minimum_mb * 1024
    except (OSError, ValueError):
        pass
    return False


def provider_error_code(error):
    """Never include tokens, request bodies, or untrusted provider text in logs."""
    if isinstance(error, urllib.error.HTTPError):
        return 'HTTP_' + str(error.code)
    if isinstance(error, StepError):
        return error.code
    reason = error.reason if isinstance(error, urllib.error.URLError) else error
    if isinstance(reason, ssl.SSLCertVerificationError):
        return 'TLS_CERTIFICATE_VERIFY_FAILED'
    if isinstance(reason, ssl.SSLError):
        return 'TLS_ERROR'
    if isinstance(reason, socket.gaierror):
        return 'DNS_LOOKUP_FAILED'
    if isinstance(reason, TimeoutError):
        return 'PROVIDER_TIMEOUT'
    if isinstance(reason, http.client.HTTPException):
        return 'HTTP_TRANSPORT_ERROR'
    if isinstance(reason, ValueError):
        return 'INVALID_PROVIDER_RESPONSE'
    return 'PROVIDER_NETWORK_ERROR'


class PatrolFallObserver:
    def __init__(self, adapter, config, topic):
        self.adapter = adapter
        self.config = config
        self.camera = dict(topic=topic, rate_hz=config['rate_hz'], max_width=640,
                           max_height=480, jpeg_quality=55, max_jpeg_bytes=256 * 1024,
                           source_max_pixels=640 * 480, source_max_bytes=640 * 480 * 4)
        read_token(config['token_file'])
        self.be = SessionClient(adapter.worker)
        self.work = None
        self.monitoring_session = str(uuid4())
        self.monitor_worker = None
        self.monitor_state = None
        self.ground_warning_at = 0.
        self.location_reason = 'NOT_EVALUATED'
        self.overlay_warning_at = 0.
        self.overlay_published = self.overlay_rejected = 0
        self.last_people = self.last_tracked = None
        self.last_suspected = False
        self.motion_until = 0.
        self.monitor_counts = dict(submitted=0, ok=0, failed=0, no_frame=0, dropped=0)
        self.health_at = 0.
        self.error_at = 0.
        self.last_provider_error = None
        self.last_provider_ms = 0
        self.last_provider_algorithm = 'UNKNOWN'
        tls = ssl.create_default_context(cafile=config.get('ca_file'))
        tls.minimum_version = ssl.TLSVersion.TLSv1_2
        self.provider_tls = tls
        self.provider_connection = None
        self.provider_lock = threading.Lock()
        self.overlay_publisher = None
        if hasattr(adapter, 'node'):
            from std_msgs.msg import String
            from .media_runtime import OVERLAY_TOPIC
            self.overlay_string = String
            self.overlay_publisher = adapter.node.create_publisher(String, OVERLAY_TOPIC, 1)
        self.overlay_status_at = 0.
        from .restricted_monitor import SharedRestrictedMonitor
        self.restricted=SharedRestrictedMonitor(self)
        self.restricted_enabled=self.restricted.enabled

    def display_overlay(self, result=None, status='OK'):
        """Latest-only ROS metadata. No network await or image conversion here."""
        if self.overlay_publisher is None:return
        from datetime import datetime, timezone
        from .media_runtime import overlay_value
        now=time.monotonic()
        if status!='OK' and now-self.overlay_status_at<1.:return
        try:
            value=dict(result.get('overlay',{})) if result else {}
            if status!='OK':
                value=dict(captured_at=datetime.now(timezone.utc).isoformat(),width=640,height=480,detections=[])
            value['status']=status
            clean=overlay_value(value)
            self.overlay_publisher.publish(self.overlay_string(data=json.dumps(clean,separators=(',',':'))))
            self.overlay_published = getattr(self, 'overlay_published', 0) + 1
            self.overlay_status_at=now
        except Exception as error:
            # Keep video/fall decisions independent, but expose a missing or
            # stale provider overlay instead of silently hiding every box.
            self.overlay_rejected = getattr(self, 'overlay_rejected', 0) + 1
            if now >= getattr(self, 'overlay_warning_at', 0.):
                detail = str(error)
                code = ('PROVIDER_OVERLAY_MISSING' if result and 'overlay' not in result else
                        detail if isinstance(error, ValueError) and re.fullmatch(r'[A-Z_]{1,64}',detail) else type(error).__name__)
                logging.info('FALL_OVERLAY_UNAVAILABLE code=%s', code)
                self.overlay_warning_at = now + 30.

    def current_revision(self):
        state = self.adapter.telemetry()
        pose = state.get('map_pose', {})
        localization = state.get('localization', {})
        if pose.get('fresh') is True and localization.get('ready') is True:
            return localization.get('session_id'), localization.get('initialization_revision')
        return None

    def camera_moving(self):
        twist = self.adapter.odom_twist_sample
        now = time.monotonic()
        moving = (not twist or now - twist[1] > .5
                  or math.hypot(*twist[0][:2]) > .01 or abs(twist[0][2]) > .03)
        if moving:
            self.motion_until = now + .5
        # A delayed image acquired just before braking still belongs to a moving camera.
        return moving or now < self.motion_until

    def stationary_revision(self):
        return None if self.camera_moving() else self.current_revision()

    def analyze(self, jpeg, session, token, captured_at=None, moving=False):
        headers = {'Authorization': 'Bearer ' + token, 'Content-Type': 'image/jpeg', 'X-Fall-Session': session}
        if captured_at is not None:
            headers.update({'X-Fall-Captured-At': captured_at, 'X-Fall-Camera-Moving': str(moving).lower()})
        provider_url = self.config['base_url'] + '/v1/fall/frame'
        # One bounded request at a time, reusing verified HTTPS. Opening a new
        # DNS/TCP/TLS connection for every image adds jitter to temporal tracking.
        uri = urlsplit(self.config['base_url'])
        with self.provider_lock:
            try:
                if self.provider_connection is None:
                    self.provider_connection = http.client.HTTPSConnection(
                        uri.hostname, uri.port or 443, timeout=self.config.get('request_timeout_sec', 1.5),
                        context=self.provider_tls)
                connection = self.provider_connection
                connection.request('POST', '/v1/fall/frame', body=jpeg, headers=headers)
                response = connection.getresponse()
                raw = response.read(8193)
                if response.status != 200:
                    if response.status == 422 and len(raw) <= 8192:
                        try:
                            detail = json.loads(raw)
                            code = detail.get('detail') if isinstance(detail,dict) else None
                        except (ValueError,UnicodeError):
                            code = None
                        if code in ('FALL_CAPTURE_TIME_NOT_FRESH','FALL_FRAME_OUT_OF_ORDER',
                                    'INVALID_FALL_CAPTURE_TIME','INVALID_FALL_SESSION',
                                    'INVALID_FRAME_DIMENSIONS','INVALID_JPEG'):
                            raise StepError(code)
                    raise urllib.error.HTTPError(provider_url, response.status,
                                                 'Fall provider rejected request', response.headers, None)
                if response.will_close or len(raw) > 8192:
                    connection.close()
                    self.provider_connection = None
            except Exception:
                if self.provider_connection is not None:
                    self.provider_connection.close()
                    self.provider_connection = None
                raise
        if len(raw) > 8192:
            raise ValueError('fall response is too large')
        value = json.loads(raw)
        if not isinstance(value, dict) or type(value.get('suspected_fall')) is not bool:
            raise ValueError('fall response is invalid')
        if value['suspected_fall']:
            confidence = value.get('confidence')
            algorithm = value.get('algorithm')
            if (type(confidence) not in (int, float) or not 0 <= confidence <= 1
                    or not isinstance(algorithm, str) or not 1 <= len(algorithm) <= 100):
                raise ValueError('fall detection data is invalid')
            kind = value.get('detection_kind', 'FALL_TRANSITION')
            if kind not in ('FALL_TRANSITION', 'PERSON_ON_FLOOR'):
                raise ValueError('fall detection kind is invalid')
            value['detection_kind'] = kind
            # Old providers ignore motion headers; do not trust their image-speed
            # transitions while our camera is moving.
            if moving and kind == 'FALL_TRANSITION':
                value = {**value, 'suspected_fall': False}
        return value

    def observation_location(self, pair, result, size, captured_at, revision, moving=False, stationary=True):
        """A depth-only end-on candidate can request review only after 3D proof."""
        if result.get('suspected_fall'):
            return result, self.patient_location(pair,result,size,captured_at,revision,stationary)
        candidate=result.get('floor_candidate')
        if not isinstance(candidate,dict):return None,None
        self.location_reason='DEPTH_FLOOR_CANDIDATE_INVALID'
        if (candidate.get('reason')!='END_ON_LOWER_BODY'
                or candidate.get('ground_confirmation_required') is not True
                or candidate.get('visual_review_eligible') is not False):return None,None
        confidence=candidate.get('confidence');person=candidate.get('person');algorithm=result.get('algorithm')
        if (type(confidence) not in (int,float) or not math.isfinite(confidence) or not 0<=confidence<=1
                or not isinstance(algorithm,str) or not 1<=len(algorithm)<=100
                or not isinstance(person,dict)
                or person.get('keypoint_roles') not in (['hip','ankle'],['hip','knee'])):return None,None
        box=person.get('bbox');points=person.get('keypoints')
        if (not isinstance(box,list) or len(box)!=4
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in box)
                or min(box)<0 or min(box[2:])<=0 or box[0]+box[2]>1.000001 or box[1]+box[3]>1.000001
                or not isinstance(points,list) or len(points)!=2):return None,None
        for point in points:
            if not isinstance(point,dict):return None,None
            x,y=point.get('x'),point.get('y')
            if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (x,y))
                    or not box[0]<=x<=box[0]+box[2] or not box[1]<=y<=box[1]+box[3]):return None,None
        if moving or self.camera_moving():
            self.location_reason='DEPTH_FLOOR_CAMERA_MOVING';return None,None
        proposal={**result,'detection_kind':'PERSON_ON_FLOOR','confidence':confidence,
                  'person':person,'visual_review_eligible':False,'_depth_only_candidate':True}
        position=self.patient_location(pair,proposal,size,captured_at,revision,stationary=True)
        if position is None:return None,None
        track_id=candidate.get('track_id');overlay=result.get('overlay')
        if (type(track_id) is int and isinstance(overlay,dict) and isinstance(overlay.get('detections'),list)):
            proposal['overlay']={**overlay,'detections':[
                {**item,'state':'PERSON_ON_FLOOR'} if isinstance(item,dict) and item.get('track_id')==track_id else item
                for item in overlay['detections']]}
        return {**proposal,'suspected_fall':True},position

    def patient_location(self, pair, result, size, captured_at, revision, stationary=True):
        floor_posture = result.get('detection_kind') == 'PERSON_ON_FLOOR'
        end_on=result.get('_depth_only_candidate') is True
        self.ground_height_m = None
        self.location_diagnostics = {}
        self.location_reason = 'LOCALIZATION_NOT_READY'
        if end_on and (revision is None or not stationary):return None
        if end_on and self.camera_moving():
            self.location_reason='DEPTH_FLOOR_CAMERA_MOVING';return None
        if revision is None and (pair is None or not floor_posture):
            return None
        self.location_reason = 'DEPTH_PAIR_UNAVAILABLE'
        if pair is None:
            return None
        self.location_reason = 'PERSON_GEOMETRY_UNAVAILABLE'
        person = result.get('person')
        if not isinstance(person, dict) or (person.get('image_width'), person.get('image_height')) != size:
            return None
        keypoints = person.get('keypoints')
        if not isinstance(keypoints, list) or len(keypoints) != 2:
            return None
        from rclpy.time import Time
        from tf2_ros import TransformException

        rgb = pair[0]
        ros = self.adapter.cfg['ros']
        stamp = Time.from_msg(rgb.header.stamp)
        try:
            measurement = measure_person(pair, keypoints, self.config['depth'],self.location_diagnostics)
        except (StepError, ValueError) as error:
            self.location_reason = getattr(error,'code','DEPTH_PROJECTION_INVALID')
            return None
        if measurement is None:
            self.location_reason = self.location_diagnostics.get('reason','DEPTH_PROJECTION_UNAVAILABLE')
            return None

        projection=dict(end_on=end_on,max_height=min(.45,(self.config.get('depth') or {}).get('end_on_max_height_m',.30)),
                        diagnostics=self.location_diagnostics,
                        hip_midpoint=person.get('keypoint_roles')==['hip','hip'])
        base = ros.get('base_frame')
        if floor_posture and isinstance(base, str) and base.rsplit('/', 1)[-1] == 'base_footprint':
            # The calibrated base_footprint plane is independent of AMCL/map.
            try:
                transform = self.adapter.tf_buffer.lookup_transform(base, rgb.header.frame_id, stamp)
                local = project_person(measurement, transform.transform, 0.,**projection)
            except (TransformException, StepError, ValueError):
                local = None
                self.location_diagnostics['reason']='BASE_CAMERA_TF_UNAVAILABLE'
            if local is not None:
                self.ground_height_m = local['floorHeightM']
                if not self.ground_valid(result, local):
                    return None
            elif end_on:
                self.location_reason=self.location_diagnostics.get('reason','DEPTH_PROJECTION_UNAVAILABLE')
                return None
        elif end_on:
            self.location_reason='BASE_FLOOR_FRAME_UNAVAILABLE';return None
        self.location_reason = 'LOCALIZATION_NOT_READY'
        if revision is None:
            return None

        try:
            transform = self.adapter.tf_buffer.lookup_transform(ros['map_frame'], rgb.header.frame_id, stamp)
            position = project_person(measurement, transform.transform, self.config['depth']['floor_z_m'],**projection)
        except TransformException:
            self.location_reason = 'MAP_CAMERA_TF_UNAVAILABLE'
            return None
        except (StepError, ValueError):
            self.location_reason = 'DEPTH_PROJECTION_INVALID'
            return None
        if position is None:
            self.location_reason = self.location_diagnostics.get('reason','DEPTH_PROJECTION_UNAVAILABLE')
            return None
        if floor_posture:
            # Keep local height evidence even if the map plane disagrees.
            # In that case staff can review the camera without a map point.
            map_height = position['floorHeightM']
            observed_in_base = self.ground_height_m is not None
            if not observed_in_base:
                self.ground_height_m = map_height
            if not self.ground_valid(result, position):
                if observed_in_base:
                    self.location_reason = 'MAP_FLOOR_DISAGREES'
                return None
            position['floorHeightM'] = self.ground_height_m
        elif position['floorHeightM'] > .65:
            self.location_reason = 'PERSON_NOT_NEAR_FLOOR'
            return None
        context = self.adapter.worker.get('navigation_session', {})
        session_id, init_revision = revision
        live = self.stationary_revision() if stationary else self.current_revision()
        if (live != revision or context.get('session_id') != session_id
                or type(init_revision) is not int or init_revision < 1):
            self.location_reason = 'NAVIGATION_CONTEXT_CHANGED'
            return None
        self.location_reason = 'READY'
        return dict(**position, capturedAt=captured_at, navigationSessionId=session_id,
                    mapId=context['map_id'], mapAssetHash=context['map_asset_hash'], initializationRevision=init_revision)

    def ground_valid(self, result, position):
        if result.get('detection_kind', 'FALL_TRANSITION') != 'PERSON_ON_FLOOR':
            return True
        height = position.get('floorHeightM') if position else None
        limit = (self.config.get('depth') or {}).get('near_floor_max_height_m', .45)
        valid = type(height) in (int, float) and math.isfinite(height) and -.15 <= height <= limit
        if not valid and position is not None:
            self.location_reason = 'PERSON_NOT_NEAR_FLOOR'
        return valid

    def visual_floor_review(self, result, position, moving):
        if (result.get('_depth_only_candidate') is True or result.get('detection_kind') != 'PERSON_ON_FLOOR'
                or position is not None or moving):
            return False
        height = self.ground_height_m
        if height is not None:
            return self.ground_valid(result, {'floorHeightM': height})
        return result.get('visual_review_eligible') is True

    async def report(self, step, result, captured_at, patient_location=None):
        body = dict(eventId=str(uuid4()), jobStepId=step['jobStepId'], detectedAt=captured_at,
                    confidence=result['confidence'], algorithm=result['algorithm'],
                    detectionKind=result.get('detection_kind', 'FALL_TRANSITION'))
        if patient_location is not None:
            body['patientLocation'] = patient_location
        try:
            await asyncio.wait_for(asyncio.to_thread(self.be.request, '/api/v1/pi/patrol-fall-events', body), 5.)
        except (StepError, OSError, asyncio.TimeoutError):
            logging.warning('PATROL_FALL_REPORT_UNCONFIRMED event=%s', body['eventId'])
            raise StepError('PATROL_FALL_REPORT_UNCONFIRMED') from None
        logging.warning('PATROL_FALL_REVIEW_REQUIRED event=%s rgbd_position_submitted=%s',
                        body['eventId'], patient_location is not None)
        raise StepError('PATROL_FALL_REVIEW_REQUIRED')

    async def observe(self, step):
        from .media_runtime import audio_reserved

        if self.work is not None and not self.work.done():
            try:
                await asyncio.wait_for(asyncio.shield(self.work), 2.)
            except asyncio.TimeoutError:
                raise StepError('PATROL_FALL_PROVIDER_BUSY') from None
            except (OSError, ValueError, http.client.HTTPException):
                pass
        token = read_token(self.config['token_file'])
        window = step['params']['window_sec']
        deadline = time.monotonic() + window
        capture = Capture(self, lambda: not audio_reserved())
        successful = 0
        revision = None
        session = None
        first_frame_after = math.inf
        try:
            await capture.set_enabled(True)
            while time.monotonic() < deadline:
                await asyncio.sleep(.05)
                if audio_reserved():
                    raise StepError('PATROL_INTERRUPTED_BY_SOS')
                live = self.stationary_revision()
                if live != revision:
                    revision, session = live, str(uuid4())
                    first_frame_after = time.monotonic()
                    successful = 0
                if live is None:
                    continue
                sample, header = capture.preview.take_sample_with_header(.35)
                if not sample or sample[-1] < first_frame_after:
                    continue
                pair = capture.depth.for_rgb(header, sample[1], sample[2]) if capture.depth is not None else None
                if capture.depth is not None and pair is None:
                    # Color may be delivered shortly before its same-timestamp depth message.
                    await asyncio.sleep(.035)
                    pair = capture.depth.for_rgb(header, sample[1], sample[2])
                try:
                    def evaluate():
                        frame = encode_jpeg(sample, self.camera)
                        if frame is None:
                            return None, None
                        return self.analyze(frame['jpeg'], session, token, frame['captured_at']), (frame['width'], frame['height'])

                    self.work = asyncio.create_task(asyncio.to_thread(evaluate))
                    self.work.add_done_callback(lambda task: task.exception() if not task.cancelled() else None)
                    result, size = await asyncio.wait_for(asyncio.shield(self.work), max(.01, deadline - time.monotonic()))
                except asyncio.TimeoutError:
                    break
                except (OSError, ValueError, http.client.HTTPException):
                    continue
                if (not result or time.monotonic() >= deadline or audio_reserved() or self.stationary_revision() != revision
                        or time.monotonic() - sample[-1] > 1.2):
                    continue
                successful += 1
                self.display_overlay(result)
                # The checkpoint owns inference while continuous observation
                # yields. Feed that same result to its restricted-zone consumer.
                worker = self.monitor_worker
                if self.restricted_enabled and worker is not None and worker.active is step:
                    self.restricted.update(worker, result, pair, size, sample[-2], revision)
                if result['suspected_fall'] or isinstance(result.get('floor_candidate'),dict):
                    result,position=self.observation_location(pair,result,size,sample[-2],revision)
                    if result is None:continue
                    if result.get('_depth_only_candidate'):self.display_overlay(result)
                    if self.ground_valid(result, position):
                        await self.report(step, result, sample[-2], position)
            if successful < 3:
                raise StepError('PATROL_FALL_DATA_UNAVAILABLE')
            logging.info('PATROL_FALL_WINDOW_COMPLETED samples=%d step=%s', successful, step['jobStepId'])
            return successful
        finally:
            await capture.set_enabled(False)

    def state(self, name):
        if name != self.monitor_state:
            logging.info('FALL_MONITOR_STATE %s', name)
            self.monitor_state = name

    def provider_outcome(self, started, result=None, error=None):
        self.last_provider_ms = round((time.monotonic() - started) * 1000)
        if error is None:
            self.monitor_counts['ok'] += 1
            algorithm = result.get('algorithm') if isinstance(result, dict) else None
            if isinstance(algorithm, str) and re.fullmatch(r'[A-Za-z0-9_.:-]{1,100}', algorithm):
                self.last_provider_algorithm = algorithm
            overlay=result.get('overlay') if isinstance(result,dict) else None
            detections=overlay.get('detections') if isinstance(overlay,dict) else None
            self.last_people=len(detections) if isinstance(detections,list) else None
            self.last_tracked=sum(d.get('track_id') is not None for d in detections if isinstance(d,dict)) if isinstance(detections,list) else None
            self.last_suspected=result.get('suspected_fall') is True if isinstance(result,dict) else False
            if self.last_provider_error is not None:
                logging.info('FALL_AI_RECOVERED previous=%s elapsed_ms=%d',
                             self.last_provider_error, self.last_provider_ms)
            self.last_provider_error = None
            return
        self.monitor_counts['failed'] += 1
        code = provider_error_code(error)
        now = time.monotonic()
        if code != self.last_provider_error or now >= self.error_at:
            logging.warning('FALL_AI_REQUEST_FAILED code=%s elapsed_ms=%d', code, self.last_provider_ms)
            self.error_at = now + 15.
        self.last_provider_error = code

    def monitor_health(self, preview):
        now = time.monotonic()
        if now < self.health_at:
            return
        self.health_at = now + 30.
        count = self.monitor_counts
        logging.info('FALL_MONITOR_HEALTH submitted=%d ok=%d failed=%d no_frame=%d dropped=%d '
                     'last_ms=%d algorithm=%s preview_error=%s depth=%s people=%s tracked=%s '
                     'suspected=%s overlay_sent=%d overlay_rejected=%d location_reason=%s location_metrics=%s',
                     count['submitted'], count['ok'], count['failed'], count['no_frame'], count['dropped'],
                     self.last_provider_ms, self.last_provider_algorithm,
                     getattr(preview, 'error', None), self.config.get('depth') is not None,
                     self.last_people,self.last_tracked,self.last_suspected,
                     self.overlay_published,self.overlay_rejected,self.location_reason,
                     json.dumps(getattr(self,'location_diagnostics',{}),separators=(',',':')))

    async def monitor(self, worker):
        """Optional task; failures/reporting never become a MOVE result or stop wheels."""
        self.monitor_worker = worker
        session = self.monitoring_session
        permitted = False
        capture = Capture(self, lambda: permitted)
        memory_ok = True
        next_memory = next_sample = cooldown_until = 0.
        revision = object()
        first_frame_after = time.monotonic()
        pending = None
        pending_deadline = next_report = 0.
        report_attempts, report_error = 0, 'NONE'
        try:
            await capture.set_enabled(True)
            while True:
                await asyncio.sleep(.05)
                now = time.monotonic()
                self.monitor_health(capture.preview)
                if now >= next_memory:
                    memory_ok = enough_memory(self.config['min_available_memory_mb'])
                    next_memory = now + 5.
                qr = getattr(worker, 'staff_qr', None)
                # ASSIST reserves microphone/speakers, not the shared RGB feed.
                # Keep camera analysis available during a staff SOS video call.
                face = getattr(worker, 'staff_face', None)
                priority = ((qr is not None and qr.preview is not None)
                            or (face is not None and face.preview is not None))
                checkpoint = bool(worker.active and worker.active.get('stepType') == 'EXECUTE_SENSING'
                                  and worker.active.get('params', {}).get('fall_detection'))
                permitted = (worker.connection_ready and not worker.shutting_down and not worker.session_fault
                             and memory_ok and not priority and not checkpoint)
                if pending is not None:
                    if now >= pending_deadline:
                        logging.warning('FALL_MONITOR_REPORT_EXPIRED event=%s attempts=%d last_code=%s',
                                        pending['eventId'], report_attempts, report_error)
                        pending = None
                        cooldown_until = now + self.config['cooldown_sec']
                    elif worker.connection_ready and now >= next_report:
                        report_attempts += 1
                        try:
                            await asyncio.to_thread(self.be.request, '/api/v1/pi/fall-events', pending)
                            logging.warning('FALL_MONITOR_REVIEW_REQUESTED event=%s kind=%s attempts=%d',
                                            pending['eventId'], pending['detectionKind'], report_attempts)
                            pending = None
                            cooldown_until = time.monotonic() + self.config['cooldown_sec']
                        except (StepError, OSError, ValueError) as error:
                            # Only internal codes and local evidence flags: never
                            # log an HTTP response body, token or exception text.
                            code = provider_error_code(error)
                            report_error = code if re.fullmatch(r'[A-Z0-9_]{1,80}', code) else 'REPORT_FAILED'
                            logging.warning('FALL_MONITOR_REPORT_RETRY event=%s attempt=%d code=%s kind=%s '
                                            'rgbd_position_submitted=%s location_reason=%s',
                                            pending['eventId'], report_attempts, report_error,
                                            pending['detectionKind'], 'patientLocation' in pending,
                                            getattr(self, 'location_reason', 'UNKNOWN'))
                            next_report = time.monotonic() + 2.
                if not permitted:
                    self.display_overlay(status='PAUSED')
                    self.state('MEMORY_PRESSURE' if not memory_ok else 'YIELD' if priority or checkpoint else
                               'WAIT_CONNECTION')
                    first_frame_after = now
                    continue
                self.state('REPORT_PENDING' if pending is not None else 'OBSERVING')
                if now < next_sample or self.work is not None and not self.work.done():
                    continue
                next_sample = now + 1. / self.config['rate_hz']
                live = self.current_revision()
                # A brief localization pause does not start a new image track.
                # A map position still requires matching fresh localization;
                # an image-only transition can request staff review without it.
                if live is not None and live != revision:
                    revision, session = live, str(uuid4())
                    first_frame_after = now
                sample, header = capture.preview.take_sample_with_header(.35)
                if sample is None or sample[-1] < first_frame_after:
                    self.monitor_counts['no_frame'] += 1
                    continue
                moving = self.camera_moving()
                pair = capture.depth.for_rgb(header, sample[1], sample[2]) if capture.depth is not None else None
                if capture.depth is not None and pair is None:
                    await asyncio.sleep(.035)
                    pair = capture.depth.for_rgb(header, sample[1], sample[2])
                started = time.monotonic()
                self.monitor_counts['submitted'] += 1
                try:
                    token = read_token(self.config['token_file'])

                    def evaluate():
                        frame = encode_jpeg(sample, self.camera)
                        if frame is None:
                            return None, None
                        return self.analyze(frame['jpeg'], session, token, frame['captured_at'], moving), (frame['width'], frame['height'])

                    self.work = asyncio.create_task(asyncio.to_thread(evaluate))
                    self.work.add_done_callback(lambda task: task.exception() if not task.cancelled() else None)
                    result, size = await asyncio.wait_for(asyncio.shield(self.work),
                                                         self.config.get('request_timeout_sec', 1.5) + .5)
                    if result is None:
                        self.monitor_counts['dropped'] += 1
                        continue
                    self.provider_outcome(started, result=result)
                    self.display_overlay(result)
                    if self.restricted_enabled and time.monotonic()-sample[-1]<=1.5:
                        try:self.restricted.update(worker,result,pair,size,sample[-2],live)
                        except (StepError,ValueError,TypeError) as error:
                            if time.monotonic()>=self.restricted.warning_at:
                                logging.info('RESTRICTED_OBSERVATION_UNAVAILABLE code=%s',getattr(error,'code',type(error).__name__))
                                self.restricted.warning_at=time.monotonic()+15.
                    qr = getattr(worker, 'staff_qr', None)
                    checkpoint = bool(worker.active and worker.active.get('stepType') == 'EXECUTE_SENSING'
                                      and worker.active.get('params', {}).get('fall_detection'))
                    if (not result or not (result['suspected_fall'] or isinstance(result.get('floor_candidate'),dict))
                            or self.current_revision() not in (None, revision)
                            or time.monotonic() - sample[-1] > 2.5
                            or time.monotonic() < cooldown_until or pending is not None
                            or not worker.connection_ready or worker.shutting_down or worker.session_fault
                            or checkpoint or qr is not None and qr.preview is not None):
                        if result and result.get('suspected_fall'):
                            self.monitor_counts['dropped'] += 1
                        continue
                    result,position=self.observation_location(pair,result,size,sample[-2],live,moving,stationary=False)
                    if result is None:
                        if time.monotonic()>=self.ground_warning_at:
                            logging.info('FALL_MONITOR_GROUND_UNAVAILABLE reason=%s',self.location_reason)
                            self.ground_warning_at=time.monotonic()+15.
                        continue
                    if result.get('_depth_only_candidate'):self.display_overlay(result)
                    visual_review = self.visual_floor_review(result, position, moving or self.camera_moving())
                    if not self.ground_valid(result, position) and not visual_review:
                        if time.monotonic() >= self.ground_warning_at:
                            logging.info('%s reason=%s floor_height_m=%s depth_enabled=%s visual_review_eligible=%s',
                                'FALL_MONITOR_GROUND_REJECTED' if self.location_reason == 'PERSON_NOT_NEAR_FLOOR'
                                else 'FALL_MONITOR_GROUND_UNAVAILABLE',
                                getattr(self, 'location_reason', 'UNKNOWN'),
                                self.ground_height_m, self.config.get('depth') is not None,
                                result.get('visual_review_eligible') is True)
                            self.ground_warning_at = time.monotonic() + 15.
                        continue
                    pending = dict(eventId=str(uuid4()), monitoringSessionId=self.monitoring_session,
                                   detectedAt=sample[-2], confidence=result['confidence'], algorithm=result['algorithm'],
                                   detectionKind=result.get('detection_kind', 'FALL_TRANSITION'))
                    if position is not None:
                        pending['patientLocation'] = position
                    if visual_review:
                        pending['visualReview'] = True
                    logging.info('FALL_MONITOR_REPORT_QUEUED event=%s kind=%s visual_review=%s '
                                 'rgbd_position_submitted=%s location_reason=%s',
                                 pending['eventId'], pending['detectionKind'], visual_review, position is not None,
                                 self.location_reason)
                    pending_deadline, next_report = time.monotonic() + 10., 0.
                    report_attempts, report_error = 0, 'NONE'
                except (StepError, OSError, ValueError, http.client.HTTPException, asyncio.TimeoutError) as error:
                    self.display_overlay(status='DEGRADED')
                    self.provider_outcome(started, error=error)
                    self.state('PROVIDER_RETRY')
                    await asyncio.sleep(1.)
        finally:
            permitted = False
            if self.monitor_worker is worker:
                self.monitor_worker = None
            await self.restricted.close()
            self.display_overlay(status='PAUSED')
            await capture.set_enabled(False)
            if self.work is not None and not self.work.done():
                try:
                    await asyncio.wait_for(asyncio.shield(self.work), 2.)
                except Exception:
                    pass
            self.state('STOPPED')
