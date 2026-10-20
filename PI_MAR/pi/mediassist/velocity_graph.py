"""Shared topology checks for runtime and startup; no ROS imports or motion."""
from .contracts import StepError


def check_velocity_graph(node, ros):
    safe = ros['safe_cmd_vel']
    publishers = node.get_publishers_info_by_topic(safe)
    if sorted(e.node_name for e in publishers) != ['mediassist_motor_watchdog', 'mediassist_safety']:
        actual = sorted(e.node_name for e in publishers)
        raise StepError('UNSAFE_VELOCITY_GRAPH',
                        f'UNSAFE_VELOCITY_GRAPH topic={safe} publishers={actual!r}; '
                        'expected exactly mediassist_motor_watchdog and mediassist_safety')
    drivers = [(name, ns) for name, ns in node.get_node_names_and_namespaces() if name == 'odom_publisher']
    if len(drivers) != 1:
        raise StepError('CHASSIS_COUNT_INVALID')
    if not any(e.node_name == 'odom_publisher' for e in node.get_subscriptions_info_by_topic(safe)):
        raise StepError('CHASSIS_NOT_GATED')
    name, namespace = drivers[0]
    for topic, types in node.get_subscriber_names_and_types_by_node(name, namespace):
        if not any(t in ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped') for t in types):
            continue
        if topic == safe:
            continue
        # Vendor driver has three subscriptions. The other two must be isolated
        # and have NO publishers; otherwise even their renamed topics bypass gate.
        if topic in ('/mediassist/unused_cmd_vel', '/mediassist/unused_app_cmd_vel'):
            if not node.get_publishers_info_by_topic(topic):
                continue
        raise StepError('CHASSIS_BYPASS_DETECTED', topic)
    motors = node.get_publishers_info_by_topic('/ros_robot_controller/set_motor')
    if any(e.node_name not in ('odom_publisher', 'mediassist_motor_watchdog') for e in motors):
        raise StepError('DIRECT_MOTOR_PUBLISHER_DETECTED')
    if not any(e.node_name == 'mediassist_motor_watchdog' for e in motors):
        raise StepError('MOTOR_WATCHDOG_MISSING')
