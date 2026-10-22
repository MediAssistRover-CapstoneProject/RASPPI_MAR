"""Fixed angular bins for MS200 SLAM only. Raw scans and motion guards stay unchanged."""
import argparse
from array import array
import math
from pathlib import Path
import struct
import subprocess
import sys
import time

TOPIC = '/mediassist/scan_slam'


def configuration(mapping):
    value = mapping.get('scan_normalization', {'enabled': False})
    if not isinstance(value, dict) or type(value.get('enabled', False)) is not bool:
        raise ValueError('INVALID_SCAN_NORMALIZATION')
    # One tested grid, rather than guessing the first rotating driver's sample count.
    if value.get('bins', 360) != 360 or type(value.get('bins', 360)) is not int:
        raise ValueError('SCAN_NORMALIZATION_REQUIRES_360_BINS')
    return {'enabled': value.get('enabled', False), 'bins': 360, 'topic': TOPIC}


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def normalize(scan):
    """Bin measured angles, retain nearest obstacles, leave unmeasured directions NaN."""
    count = len(scan.ranges)
    geometry = (scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min, scan.range_max)
    if (not 256 <= count <= 4096 or not all(math.isfinite(v) for v in geometry)
            or scan.angle_increment <= 0 or not 0 <= scan.range_min < scan.range_max
            or not math.radians(350) <= (scan.angle_max-scan.angle_min) <= 2*math.pi+.02):
        raise ValueError('INVALID_MS200_SCAN_GEOMETRY')
    endpoint = scan.angle_min + (count-1)*scan.angle_increment
    if abs(endpoint-scan.angle_max) > max(.0002, scan.angle_increment*.1):
        raise ValueError('SCAN_LENGTH_ANGLE_MISMATCH')
    ranges = array('f', [math.nan])*360
    has_intensity = len(scan.intensities) == count
    intensities = array('f', [0.])*360 if has_intensity else array('f')
    for index, reading in enumerate(scan.ranges):
        if math.isnan(reading) or reading == -math.inf or reading < scan.range_min:
            continue
        if math.isfinite(reading) and reading > scan.range_max:
            continue
        angle = (scan.angle_min + index*scan.angle_increment) % (2*math.pi)
        cell = int(math.floor(angle*360/(2*math.pi)+.5)) % 360
        if math.isnan(ranges[cell]) or reading < ranges[cell]:
            ranges[cell] = reading
            if has_intensity:
                intensity = scan.intensities[index]
                intensities[cell] = intensity if math.isfinite(intensity) else 0.
    increment = f32(2*math.pi/360)
    return {'angle_min': 0., 'angle_max': f32(359*increment), 'angle_increment': increment,
            'time_increment': 0., 'ranges': ranges, 'intensities': intensities}


class ManagedNormalizer:
    """Owned child of the SLAM session manager, reused across fresh mapping graphs."""
    def __init__(self, config, logfile, process_tools):
        self.config, self.logfile, self.process_tools = config, Path(logfile), process_tools
        self.process, self.identity = None, None

    def start(self):
        with self.logfile.open('ab') as output:
            self.process = subprocess.Popen([sys.executable, '-m', 'mediassist.scan_normalizer',
                '--config', str(self.config)], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        self.identity = self.process_tools[0](self.process.pid)
        if self.identity is None:
            raise RuntimeError('SCAN_NORMALIZER_START_FAILED')

    def check(self):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('SCAN_NORMALIZER_EXITED: see scan-normalizer.log')

    def stop(self):
        if self.process is not None:
            if self.identity is not None:
                self.process_tools[1](self.identity, grace=3.)
            self.process.wait(timeout=2.)
            self.process, self.identity = None, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    args = parser.parse_args()
    from .worker_config import load
    cfg, hardware = load(args.config)
    opts = configuration(cfg['mapping'])
    if not opts['enabled']:
        raise ValueError('SCAN_NORMALIZATION_NOT_ENABLED')
    source = hardware['ros']['scan']
    if source == TOPIC:
        raise ValueError('SCAN_NORMALIZATION_FEEDBACK_LOOP')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    node = rclpy.create_node('mediassist_scan_normalizer')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    publisher = node.create_publisher(LaserScan, TOPIC, qos)
    counts, reported, latest = [0, 0], [0.], [None]
    def receive(msg):
        try:
            stamp = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
            age = (node.get_clock().now().nanoseconds-stamp)/1e9
            if not -.1 <= age <= hardware['motion']['sensor_max_age_sec']:
                raise ValueError('SOURCE_SCAN_STALE')
            if latest[0] is not None and stamp <= latest[0]:
                raise ValueError('SOURCE_SCAN_OUT_OF_ORDER')
            converted = normalize(msg)
            result = LaserScan()
            result.header = msg.header  # Never make an old source scan appear fresh.
            result.range_min, result.range_max, result.scan_time = msg.range_min, msg.range_max, msg.scan_time
            for name, value in converted.items():
                setattr(result, name, value)
            publisher.publish(result)
            latest[0] = stamp
            counts[0] += 1
            if time.monotonic()-reported[0] >= 5:
                node.get_logger().info('SCAN_NORMALIZED raw='+str(len(msg.ranges))+' fixed=360 accepted='+str(counts[0])+' rejected='+str(counts[1]))
                reported[0] = time.monotonic()
        except (ValueError, TypeError, OverflowError) as exc:
            counts[1] += 1
            if time.monotonic()-reported[0] >= 5:
                node.get_logger().warning('SCAN_REJECTED '+str(exc))
                reported[0] = time.monotonic()
    node.create_subscription(LaserScan, source, receive, qos)
    node.get_logger().info('SCAN_NORMALIZER_READY '+source+' -> '+TOPIC+' bins=360; raw safety input unchanged')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
