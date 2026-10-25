"""Read config, raw LiDAR, TF, map and localization status. Never publishes or commands motion."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'pi'))
from mediassist.worker_config import load
from mediassist.autonomous_mapping import configuration,scan_view
from mediassist.geometry_diagnostics import planar_transform,scan_summary,map_summary,nav_footprints


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--seconds',type=float,default=6.)
    p.add_argument('--config-only',action='store_true')
    args=p.parse_args()
    if not 2<=args.seconds<=20:p.error('--seconds must be 2..20')
    cfg,hardware=load(args.config);auto=configuration(cfg['mapping'])
    body=auto.get('footprint') or dict(front=.12,rear=.12,left=.10,right=.10)
    loc=cfg.get('localization',{});nav=cfg.get('navigation_runtime',{})
    report=dict(mapping_enabled=cfg['mapping']['enabled'],autonomy_enabled=auto['enabled'],
                body_bounds_m=body,lidar_stop_distance_m=auto['lidar_stop_distance_m'],
                body_stop_margin_m=auto['stop_margin_m'],map_id=hardware.get('map_id'),
                localization={k:loc.get(k) for k in ('enabled','mode','map_id','initial_pose')},
                navigation_map_configured=bool(nav.get('map_yaml')),navigation_parameters_configured=bool(nav.get('nav2_params')))
    if nav.get('nav2_params'):
        import yaml
        try:report['nav2_config_footprints']=nav_footprints(yaml.safe_load(Path(nav['nav2_params']).read_text()) or {})
        except (OSError,ValueError,yaml.YAMLError):report['nav2_config_error']='NAV2_PARAMS_UNREADABLE'
    if not args.config_only:read_ros(cfg,hardware,auto,body,report,args.seconds)
    print(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))


def read_ros(cfg,hardware,auto,body,report,seconds):
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid,Odometry
    from std_msgs.msg import String
    from geometry_msgs.msg import PolygonStamped
    rclpy.init();node=rclpy.create_node('mar_geometry_readonly');buffer=Buffer();listener=TransformListener(buffer,node)
    samples={}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def remember(name):return lambda msg:samples.__setitem__(name,msg)
    node.create_subscription(LaserScan,hardware['ros']['scan'],remember('scan'),qos_profile_sensor_data)
    node.create_subscription(Odometry,hardware['ros']['odom'],remember('odom'),qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid,cfg['mapping']['map_topic'],remember('map'),qos)
    for name,topic in (('localization_status','/mediassist/localization/status'),('safety_state','/mediassist/safety_state')):
        node.create_subscription(String,topic,remember(name),10)
    for name in ('local_costmap','global_costmap'):
        node.create_subscription(PolygonStamped,'/'+name+'/published_footprint',remember(name),10)
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def age(stamp):return (node.get_clock().now().nanoseconds-(stamp.sec*1000000000+stamp.nanosec))/1e9
    try:
        until=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        report['ros_nodes']=sorted('/'.join((namespace.rstrip('/'),name)) for name,namespace in node.get_node_names_and_namespaces())
        scan=samples.get('scan');grid=samples.get('map')
        for name in ('localization_status','safety_state'):
            if name in samples:
                try:report[name]=json.loads(samples[name].data)
                except (ValueError,TypeError):report[name]={'error':'INVALID_JSON'}
            else:report[name]={'error':'TOPIC_NOT_RECEIVED'}
        if scan is None:report['scan_error']='SCAN_NOT_RECEIVED'
        else:
            publishers=node.get_publishers_info_by_topic(hardware['ros']['scan'])
            report['scan_source']=dict(topic=hardware['ros']['scan'],frame=scan.header.frame_id,age_sec=round(age(scan.header.stamp),3),
                beams=len(scan.ranges),range_min_m=scan.range_min,range_max_m=scan.range_max,
                publishers=[i.node_name for i in publishers])
            try:
                transform=(0.,0.,0.,1) if scan.header.frame_id==hardware['ros']['base_frame'] else planar_transform(
                    buffer.lookup_transform(hardware['ros']['base_frame'],scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform)
                report['base_from_lidar']=dict(x_m=transform[0],y_m=transform[1],yaw_rad=transform[2],angle_direction=transform[3])
                report['raw_scan']=scan_summary(scan,transform,body)
                report['autonomy_scan']=scan_view(scan,transform,{**auto,'footprint':body})
            except Exception as error:report['scan_transform_error']=type(error).__name__
        if 'odom' in samples:
            odom=samples['odom'];twist=odom.twist.twist
            report['odom']=dict(age_sec=round(age(odom.header.stamp),3),speed_mps=math.hypot(twist.linear.x,twist.linear.y),angular_rps=twist.angular.z)
        try:
            # Use map timestamp for SLAM so the raster and base pose describe the same instant.
            stamp=Time.from_msg(grid.header.stamp) if grid and cfg['mapping']['enabled'] else Time()
            tf=buffer.lookup_transform(hardware['ros']['map_frame'],hardware['ros']['base_frame'],stamp)
            pose=dict(x_m=tf.transform.translation.x,y_m=tf.transform.translation.y,yaw_rad=yaw(tf.transform.rotation))
            report['map_tf_pose']=dict(**pose,tf_age_sec=round(age(tf.header.stamp),3),quality_verified=False)
            if grid:
                origin=grid.info.origin
                snapshot=dict(width=grid.info.width,height=grid.info.height,resolution=grid.info.resolution,
                    origin=[origin.position.x,origin.position.y,yaw(origin.orientation)],frame_id=grid.header.frame_id,
                    data=bytes(v%256 for v in grid.data))
                report['map_age_sec']=round(age(grid.header.stamp),3)
                report['map_body']=map_summary(snapshot,pose,body)
                if cfg['mapping']['enabled']:
                    from mediassist.frontier_mapping import plan
                    result=plan(snapshot,pose,{**auto,'footprint':body})
                    report['frontier_plan']={k:result[k] for k in ('state','reason','frontier_count','reachable_free_area_m2',
                        'planner_cell_m','clearance_radius_m','seed_clearance_m','body_radius_m',
                        'seed_obstacle_clearance_m','seed_unknown_clearance_m') if k in result}
        except Exception as error:report['map_pose_error']=type(error).__name__
        for name in ('local_costmap','global_costmap'):
            if name not in samples:continue
            polygon=samples[name]
            report[name+'_live']=dict(frame=polygon.header.frame_id,age_sec=round(age(polygon.header.stamp),3),
                                      points=[[p.x,p.y] for p in polygon.polygon.points])
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
