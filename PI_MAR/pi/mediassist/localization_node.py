"""AMCL bootstrap + evidence monitor. Does not publish velocity or drive a search rotation."""
import argparse
import json
import math
import os
from pathlib import Path
import time
from collections import deque
from .localization import QualityGate,GridMatcher,angle,particle_support,bootstrap_verdict
from .map_tools import map_fingerprint


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class LocalizationNode:
    def __init__(self,cfg,hardware,rclpy):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid,Odometry
        from nav2_msgs.msg import ParticleCloud
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from lifecycle_msgs.srv import GetState
        from rcl_interfaces.srv import GetParameters
        from std_srvs.srv import Empty
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
        from tf2_ros import Buffer,TransformListener
        self.rclpy=rclpy;self.node=rclpy.create_node('mediassist_localization')
        self.cfg=cfg['localization'];self.hw=hardware;self.gate=QualityGate(self.cfg);self.grid_topic=cfg['mapping']['map_topic']
        # Use the same measured body that creates this session's Nav2 footprint.
        self.footprint=cfg['mapping'].get('autonomy',{}).get('footprint')
        from .navigation_setup import footprint_rectangle
        if not self.footprint:raise RuntimeError('LOCALIZATION_FOOTPRINT_REQUIRED')
        footprint_rectangle(**self.footprint)
        self.map_yaml=Path(cfg['navigation_runtime']['map_yaml']);self.meta=json.loads((self.map_yaml.parent/'metadata.json').read_text())
        if map_fingerprint(self.map_yaml)!=self.cfg['map_asset_hash']:raise RuntimeError('LOCALIZATION_MAP_HASH_MISMATCH')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self.node)
        self.grid=None;self.matcher=None;self.pose=None;self.scan=None;self.odom=None;self.particles=[]
        self.scans=deque(maxlen=12)
        self.map_file_ok=False;self.file_verified_at=-1e9;self.active={};self.pending={}
        self.boot_at=None;self.boot_ros=math.inf;self.initialized=False;self.ever_ready=False
        self.last_poll=-1e9;self.last_nomotion=-1e9;self.error='WAITING_FOR_MAP';self.last_reason=None
        self.session_context=cfg.get('navigation_session');self.initialization_revision=0;self.authorization_error=None
        self.session_poller=None
        if self.session_context:
            from .navigation_session import SessionPoller
            self.session_poller=SessionPoller(cfg)
        self.output=self.node.create_publisher(String,'/mediassist/localization/status',10)
        self.initial=self.node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(OccupancyGrid,cfg['mapping']['map_topic'],self.on_map,qos)
        self.node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,qos)
        self.node.create_subscription(ParticleCloud,'/particle_cloud',self.on_particles,qos_profile_sensor_data)
        self.node.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.on_initial,10)
        self.node.create_subscription(LaserScan,hardware['ros']['scan'],self.on_scan,qos_profile_sensor_data)
        self.node.create_subscription(Odometry,hardware['ros']['odom'],lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
        self.states={name:self.node.create_client(GetState,'/'+name+'/get_state') for name in ('amcl','map_server')}
        self.parameters=self.node.create_client(GetParameters,'/map_server/get_parameters')
        self.global_reset=self.node.create_client(Empty,'/reinitialize_global_localization')
        self.nomotion=self.node.create_client(Empty,'/request_nomotion_update')
        self.node.create_timer(.25,self.tick)

    def seconds(self,stamp):return stamp.sec+stamp.nanosec/1e9
    def ros_now(self):return self.node.get_clock().now().nanoseconds/1e9
    def on_map(self,msg):
        m=self.meta;o=msg.info.origin;self.grid=None;self.matcher=None
        if (not all(math.isfinite(v) for v in (msg.info.resolution,o.position.x,o.position.y,yaw(o.orientation)))
                or msg.header.frame_id!=self.hw['ros']['map_frame'] or msg.info.width!=m['width_px'] or msg.info.height!=m['height_px']
                or abs(msg.info.resolution-m['resolution_m_per_pixel'])>1e-6
                or abs(o.position.x-m['origin_x_m'])>1e-6 or abs(o.position.y-m['origin_y_m'])>1e-6
                or abs(angle(yaw(o.orientation)-m['origin_yaw_rad']))>1e-6):
            self.error='LOADED_MAP_GEOMETRY_MISMATCH';return
        try:self.matcher=GridMatcher(msg.info.width,msg.info.height,msg.info.resolution,
                                    (o.position.x,o.position.y,yaw(o.orientation)),msg.data,self.cfg['match_distance_m'])
        except ValueError:self.error='LOADED_MAP_INVALID';return
        self.grid=msg

    def on_scan(self,msg):
        self.scan=msg;self.scans.append(msg)

    def on_pose(self,msg):
        if msg.header.frame_id==self.hw['ros']['map_frame'] and self.seconds(msg.header.stamp)>=self.boot_ros:self.pose=msg

    def on_particles(self,msg):
        if (msg.header.frame_id==self.hw['ros']['map_frame'] and self.seconds(msg.header.stamp)>=self.boot_ros
                and len(msg.particles)<=20000):
            self.particles=[(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation),p.weight) for p in msg.particles]

    def on_initial(self,msg):
        if msg.header.frame_id!=self.hw['ros']['map_frame']:return
        self.boot_at=time.monotonic();self.boot_ros=self.seconds(msg.header.stamp)
        self.pose=None;self.particles=[];self.gate.reset();self.ever_ready=False;self.initialized=True

    def call(self,key,client,request,done):
        if key in self.pending or not client.service_is_ready():return False
        future=client.call_async(request);self.pending[key]=(future,time.monotonic())
        def complete(f):
            if self.pending.get(key,(None,))[0] is not f:return
            self.pending.pop(key,None)
            try:done(f.result())
            except Exception:self.error='ROS_SERVICE_FAILED_'+key
        future.add_done_callback(complete);return True

    def poll_services(self,now):
        from lifecycle_msgs.srv import GetState
        from rcl_interfaces.srv import GetParameters
        for key,(future,at) in list(self.pending.items()):
            if now-at>3:
                self.pending.pop(key,None);future.cancel()
                if key in self.states:self.active.pop(key,None)
        if now-self.last_poll<1:return
        self.last_poll=now
        for name,client in self.states.items():
            if not client.service_is_ready():self.active.pop(name,None)
            self.call(name,client,GetState.Request(),lambda r,n=name:self.active.update({n:(r.current_state.id==3,time.monotonic())}))
        def verified(response):
            self.map_file_ok=False
            try:
                loaded=Path(response.values[0].string_value).resolve(strict=True)
                self.map_file_ok=loaded==self.map_yaml.resolve(strict=True) and map_fingerprint(loaded)==self.cfg['map_asset_hash']
                self.file_verified_at=time.monotonic()
            except (OSError,ValueError,KeyError,IndexError):pass
        self.call('map_file',self.parameters,GetParameters.Request(names=['yaml_filename']),verified)

    def lifecycle_ready(self,now):return all(self.active.get(n,(False,-1e9))[0] and now-self.active[n][1]<3 for n in self.states)
    def map_ready(self,now):
        publishers=self.node.get_publishers_info_by_topic(self.grid_topic)
        return (self.grid is not None and self.map_file_ok and now-self.file_verified_at<3
                and len(publishers)==1 and publishers[0].node_name=='map_server')

    def bootstrap(self,now):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_srvs.srv import Empty
        if not self.map_ready(now) or not self.lifecycle_ready(now):return
        if self.authorization_error:return
        if self.cfg['mode']=='manual' and not self.initialized:self.error='WAITING_FOR_INITIAL_POSE';return
        if not self.initialized:
            if self.odom is None or abs(self.ros_now()-self.seconds(self.odom.header.stamp))>.6:return
            velocity=self.odom.twist.twist
            if not all(math.isfinite(v) for v in (velocity.linear.x,velocity.linear.y,velocity.angular.z)) or max(abs(velocity.linear.x),abs(velocity.linear.y),abs(velocity.angular.z))>.01:
                self.error='STOP_ROBOT_BEFORE_INITIALIZATION';return
            if self.cfg['mode']=='seed':
                if self.initial.get_subscription_count()<1:return
                p=self.cfg['initial_pose'];u,v=self.matcher.index(p['x_m'],p['y_m'])
                if not (0<=u<self.matcher.w and 0<=v<self.matcher.h and 0<=self.grid.data[v*self.matcher.w+u]<25):
                    self.error='INITIAL_POSE_NOT_IN_KNOWN_FREE_SPACE';return
                msg=PoseWithCovarianceStamped();msg.header.frame_id=self.hw['ros']['map_frame'];msg.header.stamp=self.node.get_clock().now().to_msg()
                msg.pose.pose.position.x=p['x_m'];msg.pose.pose.position.y=p['y_m']
                msg.pose.pose.orientation.z=math.sin(p['yaw_rad']/2);msg.pose.pose.orientation.w=math.cos(p['yaw_rad']/2)
                msg.pose.covariance[0]=msg.pose.covariance[7]=.25**2;msg.pose.covariance[35]=.35**2
                self.on_initial(msg);self.initial.publish(msg);self.error='INITIAL_POSE_SENT'
            else:
                def initialized(response):
                    self.boot_at=time.monotonic();self.initialized=True;self.error='GLOBAL_LOCALIZATION_REQUESTED'
                if 'global' not in self.pending:
                    # The first post-reset scan/cloud may arrive before the service response callback.
                    self.boot_ros=self.ros_now();self.pose=None;self.particles=[];self.gate.reset()
                    self.call('global',self.global_reset,Empty.Request(),initialized)
        # Request updates only when AMCL has not already supplied a new pose.
        # Normal motion updates remain AMCL's job. Repeated 4 Hz forced updates
        # of the same stationary geometry can impoverish the particle cloud.
        pose_age=self.ros_now()-self.seconds(self.pose.header.stamp) if self.pose is not None else math.inf
        # A temporary quality loss must not leave bootstrap's .8 s freshness
        # check waiting for a 1 s forced update forever. Request recovery updates
        # promptly, without resetting AMCL or publishing another initial pose.
        threshold=1.0 if self.gate.tracking else .35
        # AMCL already updates from moving scans. Forcing a no-motion update
        # during travel repeatedly resamples the same particle cloud.
        velocity=self.odom.twist.twist if self.odom is not None else None
        stationary=(velocity is not None and
                    all(math.isfinite(v) for v in (velocity.linear.x,velocity.linear.y,velocity.angular.z)) and
                    max(abs(velocity.linear.x),abs(velocity.linear.y),abs(velocity.angular.z))<=.01 and
                    abs(self.ros_now()-self.seconds(self.odom.header.stamp))<=.6)
        if self.initialized and stationary and pose_age>=threshold and now-self.last_nomotion>=.5:
            if self.call('nomotion',self.nomotion,Empty.Request(),lambda r:None):self.last_nomotion=now

    def evidence(self,now):
        from rclpy.time import Time
        s=dict(map_verified=self.map_ready(now),amcl_active=self.lifecycle_ready(now),tf_fresh=False,body_clear=False)
        ros=self.ros_now()
        if self.odom is not None:
            velocity=self.odom.twist.twist
            if all(math.isfinite(v) for v in (velocity.linear.x,velocity.linear.y,velocity.angular.z)):
                s['odom_age_sec']=ros-self.seconds(self.odom.header.stamp)
        if self.pose is not None:
            p=self.pose.pose.pose;s.update(amcl_pose=dict(x_m=p.position.x,y_m=p.position.y,yaw_rad=yaw(p.orientation)),
                                         covariance=list(self.pose.pose.covariance),pose_age_sec=ros-self.seconds(self.pose.header.stamp))
        if self.scan is None:return s
        # Current TF transports the last AMCL correction with odometry.
        try:
            transform=self.tf.lookup_transform(self.hw['ros']['map_frame'],self.hw['ros']['base_frame'],Time())
            age=ros-self.seconds(transform.header.stamp);p=transform.transform.translation
            s.update(tf_fresh=-.5<=age<=1.,tf_pose_age_sec=max(0.,age),
                     pose=dict(x_m=p.x,y_m=p.y,yaw_rad=yaw(transform.transform.rotation)))
            if self.grid is not None:
                from .footprint import body_clear
                g=self.grid;o=g.info.origin
                snapshot=dict(width=g.info.width,height=g.info.height,resolution=g.info.resolution,
                    origin=[o.position.x,o.position.y,yaw(o.orientation)],data=g.data)
                body=body_clear(snapshot,s['pose'],self.footprint)
                s.update(body_clear=body['clear'],footprint_kind=body['kind'],footprint_cell=body['cell'])
        except Exception as exc:
            s['tf_error']=type(exc).__name__
        if self.pose is not None:
            try:
                aligned=self.tf.lookup_transform(self.hw['ros']['map_frame'],self.hw['ros']['base_frame'],Time.from_msg(self.pose.header.stamp))
                a=aligned.transform.translation
                s['amcl_tf_pose']=dict(x_m=a.x,y_m=a.y,yaw_rad=yaw(aligned.transform.rotation))
            except Exception:
                # Never substitute current TF for a missing historical alignment.
                s['tf_fresh']=False
        s['particle_support']=particle_support(self.particles,s.get('amcl_pose',s.get('pose',{}))) if s.get('amcl_pose') else 0.
        # Latest scan often arrives before its matching odom TF. Use the newest
        # transformable scan in a bounded queue, never Time() for a past scan.
        for scan in reversed(self.scans):
            age=ros-self.seconds(scan.header.stamp)
            if not 0<=age<=self.cfg['max_sensor_age_sec']:continue
            try:
                sensor=self.tf.lookup_transform(self.hw['ros']['map_frame'],scan.header.frame_id,Time.from_msg(scan.header.stamp))
            except Exception as exc:
                s['scan_tf_error']=type(exc).__name__;continue
            from .geometry_diagnostics import planar_transform,scan_points
            try:
                transform=planar_transform(sensor.transform);points=[]
                step=max(1,math.ceil(len(scan.ranges)/240))
                for _,distance,x,y in scan_points(scan,transform,step):
                    if not max(.05,scan.range_min)<distance<min(scan.range_max*.99,12.):continue
                    points.append((x,y))
                s.update(scan_age_sec=age,scan_sequence=(scan.header.stamp.sec,scan.header.stamp.nanosec))
                if self.matcher:s.update(self.matcher.score(points))
                s.pop('scan_tf_error',None);break
            except ValueError as exc:
                s['scan_tf_error']=str(exc);break
        return s

    def tick(self):
        from std_msgs.msg import String
        now=time.monotonic()
        if self.session_poller:
            descriptor,self.authorization_error=self.session_poller.current()
            if descriptor:
                initialization=descriptor['initialization'];revision=initialization['revision']
                if revision!=self.initialization_revision:
                    self.initialization_revision=revision;self.cfg['mode']=initialization['mode'].lower()
                    if initialization['mode']=='SEED':self.cfg['initial_pose']=initialization['pose']
                    self.initialized=self.ever_ready=False;self.boot_at=None;self.boot_ros=math.inf
                    self.pose=None;self.particles=[];self.gate=QualityGate(self.cfg);self.error='INITIALIZATION_REQUESTED'
                    if 'global' in self.pending:
                        future,_=self.pending.pop('global');future.cancel()
        self.poll_services(now);self.bootstrap(now)
        sample=self.evidence(now);status=self.gate.update(now,sample)
        status=bootstrap_verdict(status,self.initialized,self.ever_ready,
            now-self.boot_at if self.boot_at is not None else 0.,self.cfg['bootstrap_timeout_sec'],self.error)
        if status['ready']:self.ever_ready=True
        if self.authorization_error:
            status.update(ready=False,reason=self.authorization_error);self.gate.reset()
        pose=sample.get('pose')
        if pose and not all(math.isfinite(v) for v in pose.values()):pose=None
        status.update(map_id=self.cfg['map_id'],map_asset_hash=self.cfg['map_asset_hash'],frame_id=self.hw['ros']['map_frame'],
                      observed_at_sec=self.ros_now(),pose=pose,pose_age_sec=sample.get('tf_pose_age_sec'),
                      amcl_pose_age_sec=sample.get('pose_age_sec'),mode=self.cfg['mode'])
        if self.session_context:status.update(session_id=self.session_context['session_id'],initialization_revision=self.initialization_revision)
        raw=json.dumps(status,allow_nan=False);self.output.publish(String(data=raw))
        if status['reason']!=self.last_reason:
            self.node.get_logger().info('LOCALIZATION '+status['reason']+' metrics='+json.dumps(status.get('metrics',{}),allow_nan=False));self.last_reason=status['reason']
        destination=Path(self.cfg['status_file']);destination.parent.mkdir(parents=True,exist_ok=True)
        temporary=destination.with_suffix('.new');temporary.write_text(raw+'\n');os.replace(temporary,destination)

def main():
    import rclpy
    from .worker_config import load
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True);args=p.parse_args()
    cfg,hardware=load(args.config)
    if not cfg.get('localization',{}).get('enabled') or cfg['mapping']['enabled']:raise RuntimeError('NAVIGATION_LOCALIZATION_CONFIG_REQUIRED')
    if cfg['localization']['map_id']!=hardware['map_id'] or cfg['localization']['map_asset_hash']!=hardware['map_asset_sha256']:
        raise RuntimeError('LOCALIZATION_MAP_BINDING_MISMATCH')
    rclpy.init();guard=LocalizationNode(cfg,hardware,rclpy)
    try:rclpy.spin(guard.node)
    except KeyboardInterrupt:pass
    finally:
        if guard.session_poller:guard.session_poller.close()
        guard.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
