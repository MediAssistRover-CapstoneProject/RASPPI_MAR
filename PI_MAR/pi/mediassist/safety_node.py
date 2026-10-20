"""Independent ROS velocity gate and motor-stop watchdog; no ROS imports on desktop."""
import argparse
import json
import time
import signal
from .contracts import read_json
from .safety import gate_velocity


def bench_pause_allowed(cfg, motor_watchdog):
    return (not motor_watchdog and cfg['max_speed_mps'] <= .03 and not cfg['allowed_workflows']
            and cfg['navigation'].get('operator_test_notes', '').startswith('TEMPORARY WHEELS-OFF-GROUND'))


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
                 "left": (None, 0.0), "right": (None, 0.0), "odom_at": 0.0, "max_speed": cfg["max_speed_mps"]}
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
        node.create_subscription(LaserScan, cfg["ros"]["scan"], lambda m: state.__setitem__("scan_at", time.monotonic()) if m.ranges else None, qos_profile_sensor_data)
        node.create_subscription(Odometry, cfg["ros"]["odom"], lambda m: state.__setitem__("odom_at", time.monotonic()), qos_profile_sensor_data)
        def tick():
            if time.monotonic() < pause_until[0]:
                return  # Bench fault injection only; independent watchdog must stop motors.
            cmd, reason = gate_velocity(state["cmd"], [state["left"], state["right"]], time.monotonic(),
                                       state["command_at"], state["lease_at"], state["scan_at"], cfg["motion"], state["max_speed"])
            if time.monotonic() - state["odom_at"] > cfg["motion"]["sensor_max_age_sec"]:
                cmd, reason = (0.0, 0.0, 0.0), "STALE_ODOM"
            # Current cart profile disallows strafe/reverse until a separate model is commissioned.
            if cmd[0] < 0 or abs(cmd[1]) > 0.001:
                cmd, reason = (0.0, 0.0, 0.0), "CART_MOTION_NOT_SUPPORTED"
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = cmd
            pub.publish(msg)
            status.publish(String(data=json.dumps({"reason": reason, "moving_command": any(cmd)})))
        node.create_timer(0.05, tick)
        cleanup = lambda: pub.publish(Twist())
    spin_and_stop(rclpy, node, cleanup)


if __name__ == "__main__": main()
