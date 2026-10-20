"""Read-only ROS probe. Checks samples and TF, not merely topic names."""
import math
import time


class Probe:
    def __init__(self, hardware, mapping):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from sensor_msgs.msg import Range, LaserScan
        from nav_msgs.msg import Odometry, OccupancyGrid
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        self.rclpy, self.hardware, self.mapping = rclpy, hardware, mapping
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('mediassist_stack_probe')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.samples = {}
        def sample(key, valid):
            if valid:
                self.samples[key] = time.monotonic()
            else:
                self.samples.pop(key, None)
        for side in ('left', 'right'):
            self.node.create_subscription(Range, hardware['ros'][side + '_range'],
                lambda msg, k=side: sample(k, math.isfinite(msg.range) and .02 <= msg.range <= 4.), qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, hardware['ros']['scan'],
            lambda m: sample('scan', len(m.ranges) > 0), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, hardware['ros']['odom'],
            lambda m: sample('odom', math.isfinite(m.twist.twist.linear.x) and math.isfinite(m.twist.twist.angular.z)), qos_profile_sensor_data)
        self.node.create_subscription(String, '/mediassist/safety_state', lambda m: sample('gate', bool(m.data)), 10)
        # VOLATILE forces a newly published map, not just a retained old map.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE if mapping.get('static_map') else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if mapping.get('static_map') else DurabilityPolicy.VOLATILE)
        self.node.create_subscription(OccupancyGrid, mapping['map_topic'], lambda m: sample('map',
            m.header.frame_id == hardware['ros']['map_frame'] and
            0 < m.info.width * m.info.height <= mapping['max_cells'] and
            len(m.data) == m.info.width * m.info.height and math.isfinite(m.info.resolution) and m.info.resolution > 0), qos)

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=.1)

    def graph(self):
        from mediassist.velocity_graph import check_velocity_graph
        check_velocity_graph(self.node, self.hardware['ros'])

    def missing(self):
        now = time.monotonic()
        age = self.hardware['motion']['sensor_max_age_sec']
        limits = {'left': age, 'right': age, 'scan': age, 'odom': age, 'gate': 1., 'map': self.mapping['max_map_age_sec']}
        missing = [key for key, limit in limits.items() if now - self.samples.get(key, -1e9) > limit]
        if self.mapping.get('static_map') and 'map' in self.samples:
            missing=[key for key in missing if key!='map']
        try:
            ros = self.hardware['ros']
            transform = self.buffer.lookup_transform(ros['map_frame'], ros['base_frame'], self.rclpy.time.Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            if not -1 <= self.node.get_clock().now().nanoseconds / 1e9 - stamp <= 2:
                missing.append('TF_STALE')
        except Exception:
            missing.append('TF_map_base')
        try:
            self.graph()
        except Exception as exc:
            missing.append(getattr(exc, 'code', str(exc)))
        return missing

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
