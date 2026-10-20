#!/usr/bin/env python3
"""Two US100 TRIG/ECHO sensors on Pi 5; Range plus explicit read status.

Only valid echoes become Range measurements. NO_ECHO is never clear space.
This node does not command motors or inject obstacles into a Nav2 costmap.
"""
import argparse
import json
import math
import os
import pathlib
import stat
import sys
import threading
import time
from dataclasses import dataclass

# Direct execution puts scripts/, not pi/, on sys.path.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))


PROFILE_PINS = {'left': (17, 18), 'right': (22, 23)}


def pulse_metres(rise_ns, fall_ns, speed=343.0):
    if rise_ns is None or fall_ns is None or fall_ns <= rise_ns:
        return None
    return (fall_ns - rise_ns) * 1e-9 * speed / 2.0


def _device_key(path):
    resolved = path.resolve(strict=True)
    info = resolved.stat()
    if stat.S_ISCHR(info.st_mode):
        return ('device', info.st_rdev)
    return ('path', str(resolved))


def choose_chip(lg, requested=None):
    """Choose one RP1, counting gpiochip4 -> gpiochip0 only once."""
    candidates, errors, observed, seen = [], [], [], set()
    paths = sorted(pathlib.Path('/dev').glob('gpiochip*'),
                   key=lambda p: (p.is_symlink(), p.name))
    for path in paths:
        if not path.name[8:].isdigit():
            continue
        number = int(path.name[8:])
        if requested is not None and number != requested:
            continue
        handle = None
        try:
            key = _device_key(path)
            if key in seen:
                continue
            seen.add(key)
            handle = lg.gpiochip_open(number)
            if handle < 0:
                raise RuntimeError('lgpio open returned ' + str(handle))
            info = lg.gpio_get_chip_info(handle)
            if not isinstance(info, (list, tuple)) or len(info) != 4 or info[0] != 0:
                raise RuntimeError('Cannot read chip info: ' + repr(info))
            label = info[3]
            if isinstance(label, bytes):
                label = label.decode('utf-8', errors='replace')
            label = label.rstrip('\0')
            observed.append(path.name + '=' + label)
            if label == 'pinctrl-rp1':
                candidates.append(number)
        except Exception as exc:
            try:
                info = path.stat()
                detail = f' gid={info.st_gid} mode={stat.S_IMODE(info.st_mode):o}'
            except OSError:
                detail = ''
            errors.append(f'{path}{detail}: {exc}')
        finally:
            if handle is not None and handle >= 0:
                lg.gpiochip_close(handle)
    if len(candidates) == 1:
        return candidates[0]
    identity = f'uid={os.getuid()} groups={os.getgroups()}' if hasattr(os, 'getuid') else ''
    if len(candidates) > 1:
        reason = 'More than one RP1 device; select --gpiochip from ' + str(candidates)
    else:
        reason = 'No accessible pinctrl-rp1 device'
    raise RuntimeError(reason + '. ' + identity + '. Found: '
                       + (', '.join(observed) or 'none opened') + '. '
                       + '; '.join(errors)
                       + '. Check /dev/gpiochip* and group access; run '
                       + 'tools/fix_gpio_access.sh on Pi HOST, then open a NEW '
                       + 'docker exec session as ubuntu. Do not use sudo to run this node.')


def pins_for(side, config=None):
    expected = PROFILE_PINS[side]
    if config is None:
        return expected
    profile = config['us100']
    pins = (profile[side]['trig_bcm'], profile[side]['echo_bcm'])
    if profile['mode'] != 'TRIG_ECHO' or pins != expected:
        raise ValueError('WIRING_PROFILE_MISMATCH: ' + side)
    return pins


class EchoCapture:
    def __init__(self):
        self.lock = threading.Lock()
        self.event = threading.Event()
        self.active = False
        self.rise = None
        self.distance = None
        self.not_before = 0

    def arm(self):
        with self.lock:
            self.rise = self.distance = None
            self.event.clear()
            self.not_before = time.monotonic_ns()
            self.active = True

    def edge(self, chip, gpio, level, tick):
        with self.lock:
            if not self.active or tick < self.not_before:
                return
            if level == 1 and self.rise is None:
                self.rise = tick
            elif level == 0 and self.rise is not None:
                self.distance = pulse_metres(self.rise, tick)
                self.active = False
                self.event.set()

    def finish(self):
        with self.lock:
            self.active = False
            return self.distance


@dataclass(frozen=True)
class Sample:
    status: str
    distance_m: float | None = None


class US100Channel:
    """One channel, owned by one process. Close before starting another reader."""
    def __init__(self, lg, handle, side, trig, echo):
        self.lg, self.handle = lg, handle
        self.side, self.trig, self.echo = side, trig, echo
        self.claimed, self.callback = [], None
        self.capture = EchoCapture()
        try:
            for pin in (trig, echo):
                info = lg.gpio_get_line_info(handle, pin)
                if info[0] != 0 or info[2] & 1 or info[4]:
                    raise RuntimeError(f'GPIO_BUSY: BCM{pin}; line_info={info}. '
                                       'Stop the other reader; do not run two US100 processes.')
            lg.gpio_claim_output(handle, trig, 0)
            self.claimed.append(trig)
            lg.gpio_claim_alert(handle, echo, lg.BOTH_EDGES)
            self.claimed.append(echo)
            self.callback = lg.callback(handle, echo, lg.BOTH_EDGES, self.capture.edge)
        except Exception:
            self.close()
            raise

    def measure(self):
        lg, handle = self.lg, self.handle
        if lg.gpio_read(handle, self.echo) != 0:
            return Sample('ECHO_HIGH_BEFORE_TRIGGER')
        self.capture.arm()
        try:
            # One >=10 us trigger; edge timestamps are nanoseconds, not microseconds.
            result = lg.tx_pulse(handle, self.trig, 10, 10, pulse_cycles=1)
            if result < 0:
                raise RuntimeError('TRIGGER_FAILED: ' + str(result))
            self.capture.event.wait(0.035)
        finally:
            distance = self.capture.finish()
        if distance is None:
            return Sample('NO_ECHO')
        if not 0.02 <= distance <= 4.0:
            return Sample('OUT_OF_RANGE')
        return Sample('OK', distance)

    def close(self):
        if self.callback is not None:
            self.callback.cancel()
            self.callback = None
        for pin in reversed(self.claimed):
            try:
                if pin == self.trig:
                    self.lg.tx_pulse(self.handle, pin, 0, 0)
                    self.lg.gpio_write(self.handle, pin, 0)
                self.lg.gpio_free(self.handle, pin)
            except Exception:
                pass
        self.claimed.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpiochip', type=int, default=None)
    parser.add_argument('--config', type=pathlib.Path, default=None)
    parser.add_argument('--fov-deg', type=float, default=None,
                        help='Configured full cone angle; validate on hardware')
    args = parser.parse_args()
    cfg = None
    if args.config:
        from mediassist.configuration import load_config
        cfg = load_config(args.config)
        args.fov_deg = cfg['us100']['fov_deg'] if args.fov_deg is None else args.fov_deg
    if args.fov_deg is None or not math.isfinite(args.fov_deg) or not 0 < args.fov_deg <= 180:
        parser.error('fov-deg must be in (0, 180]')
    import lgpio as lg
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Range
    from std_msgs.msg import String

    rclpy.init()
    node = Node('us100_dual')
    handle, channels = None, []
    try:
        number = choose_chip(lg, args.gpiochip)
        handle = lg.gpiochip_open(number)
        node.get_logger().info('RP1 gpiochip' + str(number))
        for side in ('left', 'right'):
            trig, echo = pins_for(side, cfg)
            channel = US100Channel(lg, handle, side, trig, echo)
            channels.append((channel, None))
            topic = cfg['ros'][side + '_range'] if cfg else '/ultrasonic/' + side + '/range'
            pub = node.create_publisher(Range, topic, qos_profile_sensor_data)
            channels[-1] = (channel, pub)
            node.get_logger().info(f'{side}: TRIG BCM{trig}, ECHO BCM{echo} -> {topic}')
        status_pub = node.create_publisher(String, '/ultrasonic/status', 10)
        node.get_logger().info('US100 READY; /ultrasonic/status reports each attempt; '
                               'Range is published only for a valid echo.')
        time.sleep(0.1)
        index, failures = 0, {'left': 0, 'right': 0}

        def sample():
            nonlocal index
            channel, pub = channels[index]
            index = 1 - index
            stamp = node.get_clock().now().to_msg()
            result = channel.measure()
            status = String()
            status.data = json.dumps({'side': channel.side, 'status': result.status,
                                      'distance_m': result.distance_m,
                                      'stamp_sec': stamp.sec,
                                      'stamp_nanosec': stamp.nanosec}, allow_nan=False)
            status_pub.publish(status)
            if result.status != 'OK':
                failures[channel.side] += 1
                if failures[channel.side] % 20 == 1:
                    node.get_logger().warning(channel.side + ': ' + result.status
                                               + '; not a clear-path measurement')
                return
            failures[channel.side] = 0
            msg = Range()
            msg.header.stamp, msg.header.frame_id = stamp, 'us100_' + channel.side
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = math.radians(args.fov_deg)
            msg.min_range, msg.max_range, msg.range = 0.02, 4.0, result.distance_m
            pub.publish(msg)

        # Alternate sides every 80 ms: each side ~6.25 Hz, no simultaneous pings.
        node.create_timer(0.08, sample)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for channel, _ in channels:
            channel.close()
        if handle is not None:
            lg.gpiochip_close(handle)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
