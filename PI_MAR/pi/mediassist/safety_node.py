"""Independent ROS velocity gate and motor-stop watchdog; no ROS imports on desktop."""
import argparse
import json
import time
import math
import signal
from .contracts import read_json
from .safety import gate_velocity


def bench_pause_allowed(cfg, motor_watchdog):
    return (not motor_watchdog and cfg['max_speed_mps'] <= .03 and not cfg['allowed_workflows']
            and cfg['navigation'].get('operator_test_notes', '').startswith('TEMPORARY WHEELS-OFF-GROUND'))


def imu_motion_ready(ready, observed_at, now):
    return ready is True and 0 <= now-observed_at <= 1.5


def spin_and_stop(ros, node, cleanup):
    """Keep the ROS context alive until the final zero command is published."""
    stopping = [False]
    previous = {}

    def request_stop(*_):
        stopping[0] = True

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, request_stop)
        while ros.ok() and not stopping[0]:
            ros.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if ros.ok():
                cleanup()
            else:
                node.get_logger().error('ROS_CONTEXT_CLOSED: final zero command could not be sent')
        finally:
            try:
                node.destroy_node()
            finally:
                try:
                    if ros.ok():
                        ros.shutdown()
                finally:
                    for sig, handler in previous.items():
                        signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--motor-watchdog", action="store_true")
    parser.add_argument("--bench-pause", action="store_true",
                        help="Bench profile only: SIGUSR1 pauses gate output for five seconds")
    args = parser.parse_args()
    from .configuration import load_config
    cfg = load_config(args.config)
    if args.bench_pause and not bench_pause_allowed(cfg, args.motor_watchdog):
        parser.error('--bench-pause requires a dedicated wheels-off-ground profile, without workflows')
    pause_until = [0.]
    if args.bench_pause:
        signal.signal(signal.SIGUSR1, lambda *_: pause_until.__setitem__(0, time.monotonic() + 5.))
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Range, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    # The default ROS SIGINT handler closes the context before cleanup can publish.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("mediassist_motor_watchdog" if args.motor_watchdog else "mediassist_safety")
    if args.motor_watchdog:
        from ros_robot_controller_msgs.msg import MotorState, MotorsState
        from nav_msgs.msg import Odometry
        from .watchdog import stop_required
        heartbeat, safe_at, odom_at, moving = [0.0], [0.0], [0.0], [False]
        # Send zero into the chassis subscription too: direct motor zeros alone could race
        # the chassis timer repeating its previous velocity after the gate process dies.
        velocity_zero = node.create_publisher(Twist, cfg["ros"]["safe_cmd_vel"], 10)
        def safe_seen(msg):
            safe_at[0] = time.monotonic()
            moving[0] = any((msg.linear.x, msg.linear.y, msg.angular.z))
        node.create_subscription(Twist, cfg["ros"]["safe_cmd_vel"], safe_seen, 10)
        node.create_subscription(Odometry, cfg["ros"]["odom"], lambda m: odom_at.__setitem__(0, time.monotonic()), qos_profile_sensor_data)
        pub = node.create_publisher(MotorsState, "/ros_robot_controller/set_motor", 10)
        node.create_subscription(String, "/mediassist/safety_state", lambda m: heartbeat.__setitem__(0, time.monotonic()), 10)
        def zero_motors():
            velocity_zero.publish(Twist())
            msg = MotorsState()
            for servo_id in range(1, 5):
                motor = MotorState()
                motor.id, motor.rps = servo_id, 0.0
                msg.data.append(motor)
            pub.publish(msg)
        node.create_timer(0.05, lambda: zero_motors() if stop_required(time.monotonic(), heartbeat[0], moving[0], safe_at[0], odom_at[0]) else None)
        cleanup = zero_motors
    else:
        state = {"cmd": (0.0, 0.0, 0.0), "command_at": 0.0, "lease_at": 0.0, "scan_at": 0.0,
                 "left": (None, 0.0), "right": (None, 0.0), "odom_at": 0.0, "max_speed": cfg["max_speed_mps"],
                 "rear_reason":"REAR_LIDAR_NOT_READY","rear_at":0.,'braking_view':None,'braking_at':0.,
                 "imu_ready":False,"imu_at":0.}
        from .rear_scan import chassis_guard,rear_clearance
        from tf2_ros import Buffer,TransformListener
        from rclpy.time import Time
        rear_tf=Buffer();rear_listener=TransformListener(rear_tf,node)
        pub = node.create_publisher(Twist, cfg["ros"]["safe_cmd_vel"], 10)
        status = node.create_publisher(String, "/mediassist/safety_state", 10)
        def command(msg):
            state["cmd"] = (msg.linear.x, msg.linear.y, msg.angular.z)
            state["command_at"] = time.monotonic()
        def lease(msg):
            try:
                data = json.loads(msg.data)
                cap = float(data["max_speed_mps"])
                if data["enabled"] is True and 0 < cap <= cfg["max_speed_mps"]:
                    state["lease_at"], state["max_speed"] = time.monotonic(), cap
                else: state["lease_at"] = 0.0
            except (ValueError, KeyError, TypeError): state["lease_at"] = 0.0
        def sample(side, msg):
            state[side] = (msg.range, time.monotonic())
        node.create_subscription(Twist, cfg["ros"]["raw_cmd_vel"], command, 10)
        node.create_subscription(String, cfg["ros"]["lease"], lease, 10)
        for side in ("left", "right"):
            node.create_subscription(Range, cfg["ros"][side + "_range"], lambda m, s=side: sample(s, m), qos_profile_sensor_data)
        def scan_seen(msg):
            if not msg.ranges:return
            state['scan_at']=time.monotonic()
            braking=cfg['motion'].get('lidar_braking')
            if not cfg['motion'].get('allow_reverse',False) and not braking:return
            state['braking_view']=None
            try:
                stamp=Time.from_msg(msg.header.stamp)
                age=(node.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
                if not 0<=age<=cfg['motion']['sensor_max_age_sec']:
                    state['rear_reason']='STALE_REAR_LIDAR';return
                state['rear_at']=time.monotonic()
                if msg.header.frame_id==cfg['ros']['base_frame']:
                    transform=(0.,0.,0.)
                else:
                    tf=rear_tf.lookup_transform(cfg['ros']['base_frame'],msg.header.frame_id,Time()).transform
                    q=tf.rotation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    normal=1-2*(q.x*q.x+q.y*q.y)
                    if abs(normal)<.98:raise ValueError('Non-planar lidar transform')
                    transform=(tf.translation.x,tf.translation.y,yaw)
                if braking:
                    from .autonomous_mapping import scan_view
                    direction=1 if msg.header.frame_id==cfg['ros']['base_frame'] or normal>0 else -1
                    state['braking_view']=scan_view(msg,(*transform,direction),braking)
                    state['braking_at']=time.monotonic()-age
                state['rear_reason']=rear_clearance(msg,transform,cfg['motion'].get('rear_stop_distance_m',.3))
                state['rear_at']=time.monotonic()
            except Exception:
                state['rear_reason']='REAR_LIDAR_TF_NOT_READY'
        node.create_subscription(LaserScan, cfg["ros"]["scan"], scan_seen, qos_profile_sensor_data)
        node.create_subscription(Odometry, cfg["ros"]["odom"], lambda m: state.__setitem__("odom_at", time.monotonic()), qos_profile_sensor_data)
        def imu_status_seen(msg):
            try:
                state['imu_ready']=json.loads(msg.data).get('state')=='READY'
            except (ValueError,TypeError,AttributeError):
                state['imu_ready']=False
            state['imu_at']=time.monotonic()
        node.create_subscription(String,'/mediassist/imu_status',imu_status_seen,10)
        last_report=[None,0.]
        def tick():
            if time.monotonic() < pause_until[0]:
                return  # Bench fault injection only; independent watchdog must stop motors.
            cmd, reason = gate_velocity(state["cmd"], [state["left"], state["right"]], time.monotonic(),
                                       state["command_at"], state["lease_at"], state["scan_at"], cfg["motion"], state["max_speed"])
            if time.monotonic() - state["odom_at"] > cfg["motion"]["sensor_max_age_sec"]:
                cmd, reason = (0.0, 0.0, 0.0), "STALE_ODOM"
            if not imu_motion_ready(state['imu_ready'],state['imu_at'],time.monotonic()):
                cmd,reason=(0.0,0.0,0.0),'IMU_NOT_READY'
            cmd,chassis_reason=chassis_guard(cmd,cfg['motion'],state['rear_reason'],time.monotonic()-state['rear_at'])
            if chassis_reason:reason=chassis_reason
            if cfg['motion'].get('lidar_braking') and any(cmd):
                from .motion_braking import limit_velocity
                cmd,braking_reason=limit_velocity(cmd,state['braking_view'],time.monotonic()-state['braking_at'],cfg['motion']['lidar_braking'])
                if braking_reason:reason=braking_reason
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = cmd
            pub.publish(msg)
            now=time.monotonic()
            if reason!=last_report[0] and now-last_report[1]>=.5:
                node.get_logger().info('GATE '+reason+' requested='+str(state['cmd'])+' applied='+str(cmd))
                last_report[:]=[reason,now]
            status.publish(String(data=json.dumps({"reason": reason, "moving_command": any(cmd),
                "requested_velocity":state['cmd'],"applied_velocity":cmd,
                "command_age_sec":round(time.monotonic()-state['command_at'],3),
                "lease_age_sec":round(time.monotonic()-state['lease_at'],3),"rear_reason":state['rear_reason']})))
        node.create_timer(0.05, tick)
        cleanup = lambda: pub.publish(Twist())
    spin_and_stop(rclpy, node, cleanup)


if __name__ == "__main__": main()
