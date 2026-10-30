"""Mapping session, local dead-man teleoperation and portable map artifacts."""
import asyncio
import base64
import hashlib
import json
import math
import os
import signal
import tarfile
import threading
import time
import urllib.request
import urllib.error
import zlib
from datetime import datetime,timezone
from pathlib import Path
from .contracts import StepError, canonical
from .store import utcnow
from .worker_config import NoRedirect


class MappingManager:
    def __init__(self, adapter, cfg):
        self.adapter, self.cfg = adapter, cfg
        self.root = Path(cfg['storage_dir'])
        self.root.mkdir(parents=True, exist_ok=True)
        self.state_file = self.root/'session.json'
        self.state = json.loads(self.state_file.read_text()) if self.state_file.exists() else {}
        if self.state.get('session_open'):
            self.state['state'] = 'RECOVERY_REQUIRED'
            self.persist()
        self.lock = threading.Lock()
        self.grid = None
        self.revision = 0
        self.last_sent = 0
        self.last_preview = 0
        self.last_graph_check = 0
        self.manual = (0., 0., 0.)
        self.accept_after_ns=0
        from .mapping_control import MappingControlLease, VelocityRamp
        self.remote_control = MappingControlLease()
        pad_config=cfg.get('gamepad',{})
        self.ramp = VelocityRamp(pad_config.get('linear_accel',.25),pad_config.get('angular_accel',.8))
        from .autonomous_mapping import AutoMapper, configuration
        self.auto = AutoMapper(configuration(cfg))
        from .frontier_mapping import FrontierWorker
        self.exploration=FrontierWorker(self.auto.cfg)
        self.navigator=None
        if self.auto.cfg['enabled'] and self.auto.cfg['exploration_mode']=='frontier' and self.auto.cfg['frontier_controller']=='nav2':
            from .mapping_navigator import MappingNavigator
            self.navigator=MappingNavigator(adapter,self.exploration)
        self.manual_assist_reason='IDLE'
        self.auto_selected = False
        self.auto_scan = (None, 0.)
        self.state['autonomy'] = self.auto_status()
        self.camera = None
        self.process = None
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import String
        from geometry_msgs.msg import Twist
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        adapter.node.create_subscription(OccupancyGrid, cfg['map_topic'], self.on_map, qos)
        adapter.node.create_subscription(String, '/mediassist/mapping/manual', self.on_manual, 1)
        if self.auto.cfg['enabled']:
            from sensor_msgs.msg import LaserScan
            adapter.node.create_subscription(LaserScan, adapter.cfg['ros']['scan'], self.on_auto_scan, qos)
        # One worker output, selected between workflow navigation and mapping/manual.
        self.velocity = adapter.navigation_velocity_output
        from std_srvs.srv import Trigger
        self.reset_client=adapter.node.create_client(Trigger,cfg['reset_service'])
        self.gamepad = None
        self.manual_drive = None
        self.manual_inhibited = False
        if cfg.get('control_source') == 'gamepad':
            from .gamepad import GamepadInput, configuration
            from sensor_msgs.msg import Joy
            self.gamepad = GamepadInput(configuration(cfg))
            if self.gamepad.cfg['layout']=='vendor_mecanum':
                from .manual_drive import ManualDrive
                self.manual_drive=ManualDrive(self.gamepad.cfg)
            self.gamepad_status = adapter.node.create_publisher(String, '/mediassist/gamepad/status', 1)
            self.gamepad_report_at, self.gamepad_report = 0., None
            adapter.node.create_subscription(Joy, cfg['gamepad']['topic'], self.on_gamepad, qos)
        if cfg.get('camera',{}).get('enabled') and cfg['camera'].get('mqtt_preview_enabled',cfg['camera'].get('stream_backend')!='web_video_server'):
            from .camera_preview import CameraPreview
            self.camera = CameraPreview(adapter,cfg['camera'],lambda:self.state.get('state')=='MAPPING')

    def auto_status(self):
        status={**self.auto.status(time.monotonic()), 'control_selected':self.auto_selected,
                'manual_assist_reason':getattr(self,'manual_assist_reason','IDLE'),
                'profile_max_duration_sec':self.auto.cfg['max_duration_sec']}
        pad=self.cfg.get('gamepad',{})
        status['manual_control']={'source':self.cfg.get('control_source','local'),
            'layout':pad.get('layout','legacy'),'session_required':pad.get('session_required',True),
            'forward_mps':pad.get('forward_mps',.10),'angular_rps':pad.get('angular_rps',.35),
            'continuous':pad.get('max_run_sec',3.)==0}
        if self.auto.cfg['enabled'] and getattr(self,'exploration',None):
            status['exploration']=self.exploration.tracker.status()
        return status

    @property
    def active(self):
        return self.state.get('session_open',False)

    def persist(self):
        temporary = self.state_file.with_suffix('.new')
        with temporary.open('w', encoding='utf-8') as f:
            f.write(canonical(self.state)); f.flush(); os.fsync(f.fileno())
        os.replace(temporary, self.state_file)

    def on_map(self, msg):
        stamp=msg.header.stamp
        source_ns=int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
        if source_ns<getattr(self,'accept_after_ns',0):return
        info = msg.info
        if not (0 < info.width*info.height <= self.cfg['max_cells'] and
                len(msg.data) == info.width*info.height and math.isfinite(info.resolution) and info.resolution > 0):
            return
        if msg.header.frame_id != self.adapter.cfg['ros']['map_frame']:
            return
        p, q = info.origin.position, info.origin.orientation
        vals = (p.x,p.y,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(x) for x in vals):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        snapshot = {'width':info.width, 'height':info.height, 'resolution':info.resolution,
                    'origin':[p.x,p.y,yaw], 'frame_id':msg.header.frame_id,
                    'captured_at':datetime.fromtimestamp(source_ns/1e9,timezone.utc).isoformat() if source_ns else utcnow(), 'data':msg.data.tobytes()}
        with self.lock:
            if source_ns<self.accept_after_ns:return
            self.grid = (snapshot,time.monotonic())
            self.revision += 1

    def on_gamepad(self, msg):
        # Driver and worker share the container clock. Do not refresh delayed DDS samples.
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        age = (self.adapter.node.get_clock().now().nanoseconds - stamp) / 1e9
        if not -.1 <= age <= self.gamepad.cfg['timeout_sec']:
            self.gamepad.reject('JOY_SOURCE_STALE')
            return
        self.gamepad.feed(msg.axes, time.monotonic(), stamp)

    def on_auto_scan(self, msg):
        from .autonomous_mapping import scan_view
        from rclpy.time import Time
        age=0.
        try:
            age=(self.adapter.node.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
            if not 0<=age<=self.auto.cfg['scan_max_age_sec']:return
            if msg.header.frame_id==self.adapter.cfg['ros']['base_frame']: transform=(0.,0.,0.)
            else:
                tf=self.adapter.tf_buffer.lookup_transform(self.adapter.cfg['ros']['base_frame'],msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
                q=tf.rotation
                norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
                normal=1-2*(q.x*q.x+q.y*q.y)
                if not .99<=norm<=1.01 or abs(normal)<.98:raise ValueError('Non-planar lidar transform')
                transform=(tf.translation.x,tf.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),1 if normal>0 else -1)
            view=scan_view(msg,transform,self.auto.cfg)
        except Exception:
            view={'valid':False,'reason':'LIDAR_TF_NOT_READY'}
        with self.lock:self.auto_scan=(view,time.monotonic()-max(0.,age))

    def gamepad_velocity(self, ready, idle):
        now = time.monotonic()
        active = self.active and self.state.get('state') == 'MAPPING'
        reason, allowed = 'WAIT_READY', bool(active and ready and idle and not self.adapter.motion_uncertain)
        if allowed:
            publishers = self.adapter.node.get_publishers_info_by_topic(self.cfg['gamepad']['topic'])
            allowed = len(publishers) == 1 and publishers[0].node_name == 'mediassist_gamepad_joy'
            if not allowed: reason = 'JOY_PUBLISHER_COUNT'
        if allowed:
            allowed = all(self.adapter.cfg['motion'][k] for k in
                          ('commissioned', 'cart_footprint_validated', 'watchdog_validated'))
            if not allowed: reason = 'MOTION_NOT_COMMISSIONED'
        if allowed:
            allowed = self.adapter.health()['sensors_ready']
            if not allowed: reason = 'SENSORS_NOT_READY'
        if allowed:
            odom = self.adapter.telemetry().get('odom_speed')
            allowed = bool(odom and odom['age_sec'] <= .5)
            if not allowed: reason = 'ODOM_NOT_READY'
        velocity = self.gamepad.current(self.state.get('session_id') if active else None, allowed, now, reason)
        status = self.gamepad.status()
        if status != self.gamepad_report or now - self.gamepad_report_at >= 1.:
            from std_msgs.msg import String
            self.gamepad_status.publish(String(data=json.dumps(status)))
            if status != self.gamepad_report:
                self.adapter.node.get_logger().info('GAMEPAD ' + status['reason'])
            self.gamepad_report, self.gamepad_report_at = status, now
        return velocity

    def on_manual(self, msg):
        try:
            if len(msg.data) > 512:
                return
            p = json.loads(msg.data)
            if self.state.get('state') != 'MAPPING' or p['mapping_session_id'] != self.state['session_id']:
                return
            x, z = p['linear_x'], p['angular_z']
            if x==0 and z==0 and getattr(self,'auto',None):
                self.auto.stop('OPERATOR_STOP');self.auto_selected=False
                self.state['autonomy']=self.auto_status()
                self.remote_control.stop()
                if self.gamepad:self.gamepad.stop()
                with self.lock:self.manual=(0.,0.,time.monotonic())
                return
            if self.cfg.get('control_source','local') != 'local':
                if x == 0 and z == 0:
                    self.remote_control.stop()
                    if getattr(self, 'gamepad', None): self.gamepad.stop()
                    if getattr(self,'auto',None):self.auto.stop('OPERATOR_STOP');self.auto_selected=False
                return
            if type(x) not in (int,float) or type(z) not in (int,float) or not all(math.isfinite(v) for v in (x,z)):
                return
            if abs(x) > .30 or abs(z) > 1.0:
                return
            with self.lock:
                self.manual = (float(x),float(z),time.monotonic())
        except (ValueError,KeyError,TypeError):
            return

    def receive_control(self, message, transport_id, ready, idle):
        auto=getattr(self,'auto',None)
        options=message.get('autonomy')
        if options is not None:
            if message.get('linearX')!=0 or message.get('angularZ')!=0:raise StepError('AUTONOMY_VELOCITY_MUST_BE_ZERO')
            self.remote_control.accept(message,self.adapter.worker['robot_id'],transport_id,self.state)
            if options['action']!='STOP':
                if not ready or not idle or self.adapter.motion_uncertain or self.state.get('state')!='MAPPING':raise StepError('MAPPING_ROBOT_BUSY')
                if not auto or not auto.cfg['enabled']:raise StepError('AUTONOMY_NOT_CONFIGURED')
                if not all(self.adapter.cfg['motion'][k] for k in ('commissioned','cart_footprint_validated','watchdog_validated')):raise StepError('MOTION_NOT_COMMISSIONED')
                telemetry=self.adapter.telemetry()
                if not self.adapter.health()['sensors_ready'] or telemetry.get('odom_speed',{}).get('age_sec',99)>.5:raise StepError('SENSORS_NOT_READY')
                sample=telemetry.get('map_pose',{})
                if not sample.get('fresh'):raise StepError('MAPPING_POSE_NOT_READY')
                self.adapter._check_graph()
                if self.gamepad:
                    with self.gamepad.lock:
                        if any(abs(v)>.02 for v in self.gamepad.axes):raise StepError('RETURN_GAMEPAD_TO_NEUTRAL')
                view,at=self.auto_scan
                if not view or not view.get('valid') or time.monotonic()-at>auto.cfg['scan_max_age_sec']:
                    raise StepError('AUTONOMY_LIDAR_NOT_READY')
            else:sample={}
            if auto is None:raise StepError('AUTONOMY_NOT_CONFIGURED')
            from .contracts import parse_timestamp
            remaining=(parse_timestamp(message['expiresAt'])-datetime.now(timezone.utc)).total_seconds()
            auto.accept(options,time.monotonic(),sample.get('value'),min(message['durationMs']/1000.,remaining))
            self.auto_selected=options['action']!='STOP'
            if self.gamepad:self.gamepad.stop()
            self.state['autonomy']=self.auto_status()
            if options['action']!='RENEW':self.persist()
            return
        if auto and not message.get('linearX') and not message.get('angularZ'):
            # A normal web STOP releases autonomous ownership, including its latched terminal state.
            self.remote_control.accept(message,self.adapter.worker['robot_id'],transport_id,self.state)
            auto.stop('OPERATOR_STOP');self.auto_selected=False
            if self.gamepad:self.gamepad.stop()
            self.state['autonomy']=self.auto_status();self.persist()
            return
        if getattr(self,'auto_selected',False):raise StepError('AUTONOMY_OWNS_CONTROL')
        if self.cfg.get('control_source') == 'gamepad':
            if message.get('linearX') or message.get('angularZ'):
                raise StepError('MAPPING_CONTROL_SOURCE_GAMEPAD')
            self.remote_control.accept(message,self.adapter.worker['robot_id'],transport_id,self.state)
            self.gamepad.stop()
            return
        if self.cfg.get('control_source','local') != 'mqtt':raise StepError('MAPPING_CONTROL_SOURCE_LOCAL')
        moving = bool(message.get('linearX') or message.get('angularZ'))
        if moving:
            if not ready or not idle or self.adapter.motion_uncertain:raise StepError('MAPPING_ROBOT_BUSY')
            if not all(self.adapter.cfg['motion'][k] for k in ('commissioned','cart_footprint_validated','watchdog_validated')):
                raise StepError('MOTION_NOT_COMMISSIONED')
            if not self.adapter.health()['sensors_ready']:raise StepError('SENSORS_NOT_READY')
            odom=self.adapter.telemetry().get('odom_speed')
            if not odom or odom['age_sec']>.5:raise StepError('ODOM_NOT_READY')
            self.adapter._check_graph()
        self.remote_control.accept(message,self.adapter.worker['robot_id'],transport_id,self.state)

    def snapshot(self):
        with self.lock:
            item, rev = self.grid, self.revision
        if not item or time.monotonic()-item[1] > self.cfg['max_map_age_sec']:
            raise StepError('MAPPING_MAP_NOT_FRESH')
        return item[0], rev

    async def tick(self, ready, idle):
        from geometry_msgs.msg import Twist
        y=0.
        navigator=getattr(self,'navigator',None)
        if navigator and navigator.active and (not self.active or not self.auto_selected or self.auto.phase!='RUNNING'):
            await navigator.stop()
        if getattr(self,'manual_drive',None) and not getattr(self,'auto_selected',False):
            self.manual_drive.tick(self,ready,idle)
            self.state['autonomy']=self.auto_status()
            return
        gamepad_velocity = self.gamepad_velocity(ready, idle) if self.cfg.get('control_source') == 'gamepad' and not getattr(self,'auto_selected',False) else None
        if not self.active:
            return
        with self.lock:
            x, z, captured = self.manual
        if getattr(self,'auto_selected',False):
            x,z,fresh=0.,0.,True
        elif gamepad_velocity is not None:
            x,z = gamepad_velocity
            fresh = True
        elif self.cfg.get('control_source','local') == 'mqtt':
            x,z = self.remote_control.current()
            fresh = True
        else:
            fresh = time.monotonic()-captured < .25
        moving = (ready and idle and not self.adapter.motion_uncertain and self.state.get('state') == 'MAPPING'
                  and fresh and self.adapter.health()['sensors_ready'])
        if not moving:self.remote_control.stop()
        if moving:
            if time.monotonic()-self.last_graph_check >= .2:
                self.adapter._check_graph()
                self.last_graph_check = time.monotonic()
            odom = self.adapter.telemetry().get('odom_speed')
            if not odom or odom['age_sec'] > .5:
                moving = False
        auto=getattr(self,'auto',None)
        if auto and getattr(self,'auto_selected',False):
            telemetry=self.adapter.telemetry();sample=telemetry.get('map_pose',{})
            with self.lock:view,at=self.auto_scan
            old_phase=auto.phase
            with self.lock:grid=self.grid;map_revision=self.revision
            if not grid or time.monotonic()-grid[1]>min(2.,self.cfg['max_map_age_sec']):auto.stop('MAPPING_MAP_STALE')
            if self.gamepad:
                with self.gamepad.lock:override=any(abs(v)>.15 for v in self.gamepad.axes)
                if override:
                    auto.stop('GAMEPAD_OVERRIDE');self.gamepad.stop('RETURN_NEUTRAL')
            gate=telemetry.get('gate',{}).get('value')
            if isinstance(gate,str):
                try:gate=json.loads(gate)
                except (ValueError,TypeError):gate={'reason':'INVALID_STATUS'}
            now=time.monotonic();pose=sample.get('value') if sample.get('fresh') else None
            plan_result=None;plan_error=None;heading=None
            if auto.cfg['exploration_mode']=='frontier' and auto.phase=='RUNNING' and grid and pose:
                from .frontier_mapping import guidance
                plan_result,plan_error=self.exploration.poll(grid[0],pose,map_revision,now,auto.run_id,now-auto.start_at,auto.travel)
                heading=guidance(plan_result,pose) if not navigator else None
            # Frontier availability chooses the NEXT goal. It must not cancel
            # an accepted Nav2 route merely because the scan now observes its
            # destination. Nav2 still checks the live route and footprint.
            continue_goal=bool(navigator and navigator.active and not plan_error and plan_result
                and plan_result.get('seed_body_clear') is True
                and plan_result['reason'] in ('FRONTIERS_UNREACHABLE','OBSERVED_AND_VISITED_REACHABLE_SPACE','NO_SIGNIFICANT_REACHABLE_FRONTIER'))
            hold=auto.cfg['exploration_mode']=='frontier' and not continue_goal and (plan_result is None or plan_result['state'] in ('NO_FRONTIERS','BLOCKED'))
            reactive=auto.cfg['exploration_mode']=='reactive' and auto.cfg['vehicle']=='rigid'
            x,z=auto.current(now,view,now-at,pose,moving,'SENSORS_OR_SESSION_NOT_READY',gate,heading,hold or bool(navigator) or reactive)
            rear_blocked=auto.reason=='REPLANNING_REAR_BLOCKED' and auto.phase=='RUNNING'
            if rear_blocked and navigator and navigator.active:
                target=dict(navigator.target)
                await navigator.stop()
                self.exploration.exclude_goal(target,now,ttl=15.)
                plan_result=None
            authorized=auto.phase=='RUNNING' and now<auto.deadline
            if reactive and authorized:
                from .lidar_reactive import LidarReactive
                if not hasattr(self,'lidar_reactive'):self.lidar_reactive=LidarReactive()
                (x,y,z),auto.reason=self.lidar_reactive.current(view,auto.cfg,max(0.,now-at),now,auto.run_id)
                # Observe coverage without using a cached grid to authorize motion.
                # All actual velocities are bounded from the current raw scan.
                if grid and pose and getattr(self,'exploration',None):
                    result,error=self.exploration.poll(grid[0],pose,map_revision,now,auto.run_id,now-auto.start_at,auto.travel)
                    if result and result['state']=='NO_FRONTIERS' and self.exploration.tracker.complete:
                        from .frontier_mapping import source_signature
                        if result['source_signature']==source_signature(grid[0]):
                            x,z=auto.stop('COVERAGE_COMPLETE');y=0.
            if authorized and auto.cfg['exploration_mode']=='frontier':
                if rear_blocked:x,y,z=0.,0.,0.
                elif plan_error:x,z=auto.stop(plan_error)
                elif plan_result is None:x,z=0.,0.;auto.reason='PLANNING_FRONTIERS'
                elif plan_result['state']=='BLOCKED' and not continue_goal:
                    if plan_result['reason']=='FRONTIERS_UNREACHABLE' and not self.exploration.no_progress(now,True):
                        x,z=0.,0.;auto.reason='REOBSERVING_FRONTIERS'
                    elif plan_result.get('seed_block_kind') in ('unknown','boundary') and now-auto.start_at<8.:
                        x,z=0.,0.;auto.reason='WAITING_FOR_MAP_AROUND_BODY'
                    else:x,z=auto.stop(plan_result['reason'])
                elif plan_result['state']=='NO_FRONTIERS' and not continue_goal:
                    x,z=0.,0.;auto.reason='CONFIRMING_COVERAGE'
                    if self.exploration.tracker.complete:
                        from .frontier_mapping import source_signature
                        if plan_result['source_signature']==source_signature(grid[0]):x,z=auto.stop('COVERAGE_COMPLETE')
                elif self.exploration.no_progress(now,bool(heading and heading['hold'])):
                    x,z=auto.stop('FRONTIER_NO_PROGRESS')
                elif navigator:
                    try:
                        (x,y,z),auto.reason=await navigator.tick(plan_result,pose,now)
                        from .autonomous_mapping import nav2_assist
                        limited=nav2_assist((x,y,z),view,auto.cfg,max(0.,now-at))
                        if limited!=(x,y,z):auto.reason='FOLLOW_FRONTIER_NAV2_SLOWED'
                        x,y,z=limited
                    except Exception as error:
                        x,z=auto.stop(getattr(error,'code','NAV2_MAPPING_FAILED'));y=0.
            if not authorized or auto.phase!='RUNNING':x,y,z=0.,0.,0.
            if navigator and navigator.active and (auto.phase!='RUNNING' or hold):await navigator.stop()
            self.state['autonomy']=self.auto_status()
            if old_phase!=auto.phase:
                self.persist()
                if auto.phase=='STOPPED':
                    self.adapter.node.get_logger().info('AUTONOMY_STOP '+json.dumps({
                        'reason':auto.reason,'front_clearance_m':view.get('front_m') if view else None,
                        'nearest_lidar_m':view.get('nearest_lidar_m') if view else None,
                        'lidar_stop_distance_m':auto.cfg['lidar_stop_distance_m'],
                        'footprint':auto.cfg['footprint'],'plan_reason':plan_result.get('reason') if plan_result else None}))
                    self.save_autonomy_diagnostic(plan_result,pose,view)
        elif auto and auto.cfg['enabled'] and auto.cfg['manual_lidar_assist'] and moving:
            from .autonomous_mapping import manual_assist
            with self.lock:view,at=self.auto_scan
            (x,z),self.manual_assist_reason=manual_assist((x,z),view,time.monotonic()-at,auto.cfg)
            self.state['autonomy']=self.auto_status()
        msg = Twist()
        auto_cap = auto.cfg['speed_mps'] if auto and getattr(self,'auto_selected',False) else .30
        cap = min(auto_cap, self.adapter.cfg['max_speed_mps'])
        if moving:
            target = (max(-cap,min(x,cap)),
                      max(-self.adapter.cfg['motion']['max_angular_speed_rps'],
                          min(z, self.adapter.cfg['motion']['max_angular_speed_rps'])))
            if (navigator or auto and auto.cfg['exploration_mode']=='reactive') and getattr(self,'auto_selected',False):
                from .mapping_control import HolonomicRamp
                if not hasattr(self,'holonomic_ramp'):self.holonomic_ramp=HolonomicRamp()
                if not self.adapter.cfg['motion'].get('allow_strafe',False):y=0.
                scale=min(1.,cap/max(math.hypot(target[0],y),1e-9))
                msg.linear.x,msg.linear.y,msg.angular.z=self.holonomic_ramp.update((target[0]*scale,y*scale,target[1]),time.monotonic())
            else:
                if hasattr(self,'holonomic_ramp'):self.holonomic_ramp.reset()
                msg.linear.x, msg.angular.z = self.ramp.update(target, time.monotonic())
        else:
            self.ramp.reset()
            if hasattr(self,'holonomic_ramp'):self.holonomic_ramp.reset()
        self.velocity.publish(msg)
        self.adapter.lease_message(bool(moving and (x or y or z)), cap)

    async def preview(self):
        rate = min(self.state.get('preview_rate_hz',0.2), self.cfg.get('max_preview_rate_hz',0.2))
        if self.state.get('state') != 'MAPPING' or time.monotonic()-self.last_preview < 1/rate:
            return None
        try:
            snap, rev = self.snapshot()
        except StepError:
            return None
        if rev == self.last_sent:
            return None
        self.last_preview = time.monotonic()
        encoded = await asyncio.to_thread(lambda: base64.b64encode(zlib.compress(snap['data'],1)).decode('ascii'))
        if len(encoded) > self.cfg.get('max_preview_bytes',131072):
            return None
        self.last_sent = rev
        # Pair the marker with this OccupancyGrid's timestamp. A latest TF read
        # after PNG compression can belong to a later SLAM correction/robot pose.
        pose = self.preview_pose(snap)
        # Pose belongs to the live session, not the previously commissioned map UUID.
        if pose.get('value'):
            # The outer payload already identifies the SLAM session. Worker
            # telemetry also includes mapping_session_id; neither identity field
            # belongs in common.pose, which accepts only these three numbers.
            pose = {**pose, 'value':{k:pose['value'][k] for k in ('x_m','y_m','yaw_rad')}}
        return {**{k:v for k,v in snap.items() if k!='data'}, 'revision':rev,
                'mapping_session_id':self.state['session_id'], 'pose':pose,
                'encoding':'zlib-base64-int8-row-major-bottom-up', 'data':encoded}

    def preview_pose(self,snapshot):
        try:
            from rclpy.time import Time
            stamp=datetime.fromisoformat(snapshot['captured_at'].replace('Z','+00:00')).timestamp()
            transform=self.adapter.tf_buffer.lookup_transform(snapshot['frame_id'],
                self.adapter.cfg['ros']['base_frame'],Time(nanoseconds=round(stamp*1e9)))
            t,q=transform.transform.translation,transform.transform.rotation
            value=dict(x_m=t.x,y_m=t.y,yaw_rad=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            if not all(math.isfinite(v) for v in value.values()):raise ValueError('Invalid map pose')
            return {'value':value,'age_sec':0.,'fresh':True}
        except Exception:
            return {'value':None,'age_sec':None,'fresh':False}

    def ensure_scope(self, step):
        if self.state.get('session_id') != step['params']['mapping_session_id'] or self.state.get('job_id') != step['jobId']:
            raise StepError('MAPPING_SESSION_MISMATCH')

    async def new_slam_session(self):
        from std_srvs.srv import Trigger
        deadline=time.monotonic()+8.
        while not self.reset_client.service_is_ready():
            if time.monotonic()>deadline:raise StepError('SLAM_RESET_SERVICE_UNAVAILABLE','Update Pi launch and restart run; old SLAM graph was not reused')
            await asyncio.sleep(.1)
        for attempt in range(3):
            response=await self.adapter.future(self.reset_client.call_async(Trigger.Request()),timeout=20)
            if response.success:
                generation=json.loads(response.message)['generation']
                with self.lock:
                    self.accept_after_ns=self.adapter.node.get_clock().now().nanoseconds
                    self.grid=None;self.revision=0;self.last_sent=0;self.last_preview=0
                self.adapter.node.get_logger().info('MAPPING_NEW_GRAPH '+str(generation))
                return
            if response.message!='SLAM_RESET_REQUIRES_STOPPED_GATE':break
            await asyncio.sleep(.2)
        raise StepError('SLAM_RESET_FAILED',response.message)

    async def perform(self, step):
        p = step['params']
        action = p['action']
        if action == 'START':
            if self.active:
                raise StepError('MAPPING_ALREADY_ACTIVE')
            if getattr(self,'navigator',None):await self.navigator.stop()
            self.ramp.reset()
            if getattr(self,'auto',None):
                from .autonomous_mapping import AutoMapper
                self.auto=AutoMapper(self.auto.cfg);self.auto_selected=False;self.auto_scan=(None,0.)
                if getattr(self,'exploration',None):self.exploration.reset(None)
            # Existing navigation must be stopped by the local operator first.
            if self.adapter.nav.server_is_ready() and not getattr(self,'navigator',None):
                raise StepError('STOP_NAVIGATION_BEFORE_MAPPING')
            self.remote_control.stop()
            if not await self.adapter.stop():raise StepError('PHYSICAL_STOP_UNCONFIRMED')
            self.state = {'session_id':p['mapping_session_id'], 'job_id':step['jobId'],
                          'state':'STARTING','session_open':True,'preview_rate_hz':min(p.get('preview_rate_hz',0.2),self.cfg.get('max_preview_rate_hz',0.2))}
            self.persist()
            with self.lock:
                self.grid = None
                self.manual=(0.,0.,0.)
            if self.cfg['launch_argv']:
                log = (self.root/'slam.log').open('ab')
                try:
                    self.process = await asyncio.create_subprocess_exec(*self.cfg['launch_argv'],
                        stdout=log,stderr=log,start_new_session=True)
                finally:
                    log.close()
            if self.cfg.get('fresh_session',True):
                await asyncio.sleep(.2)
                await self.new_slam_session()
            deadline = time.monotonic()+self.cfg['start_timeout_sec']
            while True:
                if self.process and self.process.returncode is not None:
                    raise StepError('SLAM_PROCESS_EXITED')
                try:
                    self.snapshot()
                    if self.adapter.telemetry()['map_pose']['fresh']:
                        break
                except StepError:
                    pass
                if time.monotonic() > deadline:
                    raise StepError('SLAM_MAP_OR_TF_NOT_READY')
                await asyncio.sleep(.1)
            self.adapter._check_graph()
            if not await self.adapter.stop():
                raise StepError('PHYSICAL_STOP_UNCONFIRMED')
            self.state['state'] = 'MAPPING'
            if getattr(self,'auto',None):self.state['autonomy']=self.auto_status()
            self.persist()
            return self.result({'state':'MAPPING','preview_ready':True})
        self.ensure_scope(step)
        if action == 'STOP':
            await self.abort('STOPPED')
            return self.result({'state':self.state['state'],'stopped':True})
        if self.state.get('state') not in ('MAPPING','SAVED_LOCAL','SAVED'):
            raise StepError('MAPPING_NOT_READY_TO_SAVE')
        artifact = p['artifact_id']
        if self.state.get('artifact_id') not in (None,artifact):
            raise StepError('ARTIFACT_ID_MISMATCH')
        self.remote_control.stop()
        self.state['state'] = 'SAVING'
        if getattr(self,'auto',None):
            self.auto.stop('SAVING_MAP');self.auto_selected=False
            self.state['autonomy']=self.auto_status()
        self.persist()
        with self.lock:
            self.manual = (0.,0.,0.)
        if not await self.adapter.stop():
            raise StepError('PHYSICAL_STOP_UNCONFIRMED')
        folder = self.root/self.state['session_id']/artifact
        if not ((folder/'manifest.json').exists() and (folder/'map.tar.gz').exists()):
            snap, _ = self.snapshot()
            await asyncio.to_thread(self.save_files, folder, snap, artifact)
        metadata = json.loads((folder/'manifest.json').read_text())
        self.state.update(state='SAVED_LOCAL',artifact_id=artifact,artifact_dir=str(folder))
        self.persist()
        uploaded = False
        if self.cfg['upload_base_url']:
            uploaded = await asyncio.to_thread(self.upload, folder, metadata)
        self.state['state'] = 'SAVED' if uploaded else 'SAVED_LOCAL'
        self.persist()
        return self.result({'state':self.state['state'],'saved_locally':True,'uploaded':uploaded,**metadata})

    def save_autonomy_diagnostic(self,plan_result,pose,view):
        """Persist exact planner inputs locally on STOP; floorplan never added to heartbeat."""
        try:
            with self.lock:grid=self.grid;revision=self.revision
            if not grid:return
            snap=grid[0];result={k:v for k,v in (plan_result or {}).items() if k not in ('planning_grid','path')}
            report={'observed_at':utcnow(),'run_id':self.auto.run_id,'mapping_session_id':self.state.get('session_id'),
                'map_revision':revision,'pose':pose,'pose_jump':getattr(self.auto,'pose_jump',None),
                'scan_view':view,'autonomy':self.auto.cfg,'plan':result,
                'snapshot':{**snap,'data':base64.b64encode(zlib.compress(snap['data'])).decode()}}
            folder=self.root/'diagnostics';folder.mkdir(exist_ok=True)
            (folder/(self.auto.run_id+'.json')).write_text(json.dumps(report,allow_nan=False))
        except Exception:self.adapter.node.get_logger().warning('AUTONOMY_DIAGNOSTIC_WRITE_FAILED')

    def save_files(self, folder, snap, artifact):
        import yaml
        folder.mkdir(parents=True,exist_ok=True)
        width, height = snap['width'],snap['height']
        raw = snap['data']
        # Trinary PGM: 0 occupied >=65, 254 free <=25, 205 unknown/intermediate.
        lut = bytes(254 if v <= 25 else 0 if 65 <= v <= 100 else 205 for v in range(256))
        pixels = raw.translate(lut)
        pgm = b'P5\n%d %d\n255\n' % (width,height) + b''.join(
            pixels[row*width:(row+1)*width] for row in range(height-1,-1,-1))
        (folder/'map.pgm').write_bytes(pgm)
        text = yaml.safe_dump({'image':'map.pgm','mode':'trinary','resolution':snap['resolution'],
             'origin':snap['origin'],'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False).encode()
        (folder/'map.yaml').write_bytes(text)
        metadata = {'artifact_id':artifact,'asset_hash':hashlib.sha256(text+b'\x00'+pgm).hexdigest(),
            'width_px':width,'height_px':height,'resolution_m_per_pixel':snap['resolution'],
            'width_m':width*snap['resolution'],'height_m':height*snap['resolution'],
            'origin_x_m':snap['origin'][0],'origin_y_m':snap['origin'][1],
            'origin_yaw_rad':snap['origin'][2],'frame_id':snap['frame_id']}
        (folder/'manifest.json').write_text(canonical(metadata),encoding='utf-8')
        with tarfile.open(folder/'map.tar.gz','w:gz',format=tarfile.USTAR_FORMAT) as tar:
            for name in ('map.yaml','map.pgm','manifest.json'):
                tar.add(folder/name,arcname=name)

    def upload(self, folder, metadata):
        import ssl
        url = self.cfg['upload_base_url'].rstrip('/')+'/'+self.state['session_id']+'/artifacts/'+metadata['artifact_id']
        from .be_token import upload_token
        robot_id=getattr(self.adapter,'worker',{}).get('robot_id') if hasattr(self,'adapter') else None
        token=upload_token(self.cfg,robot_id)
        body = (folder/'map.tar.gz').read_bytes()
        context = ssl.create_default_context(cafile=self.cfg.get('upload_ca_file') or None)
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect(),
                                            urllib.request.HTTPSHandler(context=context))
        request = urllib.request.Request(url,data=body,method='PUT',headers={
            'Authorization':'Bearer '+token,'Content-Type':'application/gzip',
            'X-Asset-SHA256':metadata['asset_hash'],'X-Archive-SHA256':hashlib.sha256(body).hexdigest()})
        try:
            with opener.open(request,timeout=15) as response:
                receipt = json.loads(response.read(65536))
        except urllib.error.HTTPError as exc:
            if exc.code in (401,403) and self.cfg.get('device_secret_file'):
                token=upload_token(self.cfg,robot_id,force=True)
                request.remove_header('Authorization');request.add_header('Authorization','Bearer '+token)
                try:
                    with opener.open(request,timeout=15) as response:
                        receipt=json.loads(response.read(65536))
                except urllib.error.HTTPError as retry:
                    raise StepError('MAP_UPLOAD_UNAUTHORIZED' if retry.code in (401,403) else 'MAP_UPLOAD_HTTP_ERROR',
                                    'Map upload HTTP '+str(retry.code)+'; local artifact preserved') from None
                if (receipt.get('artifactId')!=metadata['artifact_id'] or receipt.get('assetHash')!=metadata['asset_hash'] or receipt.get('stored') is not True):
                    raise StepError('MAP_UPLOAD_RECEIPT_MISMATCH')
                return True
            code = 'MAP_UPLOAD_UNAUTHORIZED' if exc.code in (401,403) else 'MAP_UPLOAD_HTTP_ERROR'
            raise StepError(code, 'Map upload HTTP ' + str(exc.code) + '; local artifact preserved') from None
        if (receipt.get('artifactId') != metadata['artifact_id'] or receipt.get('assetHash') != metadata['asset_hash']
            or receipt.get('stored') is not True):
            raise StepError('MAP_UPLOAD_RECEIPT_MISMATCH')
        return True

    def result(self, metrics):
        return {'sensing_type':'MAPPING','mapping_session_id':self.state['session_id'],
                'metrics':metrics,'captured_at':utcnow(),'evidence_ref':None}

    async def abort(self, reason):
        if not self.state:
            return
        previous_inhibit=getattr(self,'manual_inhibited',False)
        self.manual_inhibited=True
        self.remote_control.stop()
        if self.gamepad:self.gamepad.stop('RETURN_NEUTRAL')
        if getattr(self,'auto',None):
            self.auto.stop(reason);self.auto_selected=False
            self.state['autonomy']=self.auto_status()
        needs_reconciliation = self.state.get('state') == 'RECOVERY_REQUIRED'
        with self.lock:
            self.manual = (0.,0.,0.)
        stopped = await self.adapter.stop()
        if self.process and self.process.returncode is None:
            try:
                os.killpg(self.process.pid,signal.SIGINT)
                await asyncio.wait_for(self.process.wait(),5)
            except asyncio.TimeoutError:
                os.killpg(self.process.pid,signal.SIGKILL)
                await self.process.wait()
        self.process = None
        self.state['session_open'] = not stopped or needs_reconciliation
        if needs_reconciliation:
            self.state['state'] = 'RECOVERY_REQUIRED'
        elif not stopped or self.state.get('state') not in ('SAVED','SAVED_LOCAL'):
            self.state['state'] = reason if stopped else 'RECOVERY_REQUIRED'
        self.persist()
        self.manual_inhibited=previous_inhibit
        if not stopped:
            raise StepError('PHYSICAL_STOP_UNCONFIRMED')
