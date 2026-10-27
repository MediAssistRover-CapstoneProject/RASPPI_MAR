"""Gyro bias from stationary evidence; keep the vendor's accelerometer calibration.

The vendor apply_calib node averages its first 100 gyro samples unconditionally.
Here it runs with calibrate_gyros=false. Only this node estimates/subtracts bias,
before the unchanged vendor complementary filter and EKF. No motor commands.
"""
from collections import deque
import json
import math
import statistics
import time


class StationaryGyro:
    WINDOW = 3.0

    def __init__(self):
        self.phase, self.reason = 'CALIBRATING', 'KEEP_ROBOT_STILL'
        self.bias = None
        self.samples = deque()
        self.command_at = self.scan_at = -math.inf
        self.command_still = False
        self.anchor = None
        self.scan_still_since = None
        self.last_stamp = None
        self.evaluate_at = 0.

    def command(self, values, now):
        self.command_at = now
        self.command_still = all(math.isfinite(v) and abs(v) <= .005 for v in values)
        if not self.command_still:
            self.samples.clear(); self.scan_still_since = None; self.anchor = None

    def scan(self, ranges, now):
        # Fixed angular bins; compare only common measured rays, never inf/NaN as free space.
        self.scan_at = now
        if self.anchor is None or len(self.anchor) != len(ranges):
            self.anchor = list(ranges); self.scan_still_since = now; return
        pairs = [(x,y) for x,y in zip(self.anchor,ranges)
                 if math.isfinite(x) and math.isfinite(y) and x > .1 and y > .1]
        differences = sorted(abs(x-y) for x,y in pairs)
        relative = sorted(abs(x-y)/max(1.,(x+y)/2) for x,y in pairs)
        # Distant rays vary with the MS200's angular sampling. Require a stable
        # near scene and relative agreement in the far scene, not 6 cm at 10 m.
        stable = (len(differences) >= 90 and statistics.median(differences) <= .025
                  and relative[int(.9*(len(relative)-1))] <= .04)
        if not stable:
            self.anchor = list(ranges); self.scan_still_since = now; self.samples.clear()

    def feed(self, gyro, acceleration, now, stamp):
        if (len(gyro) != 3 or len(acceleration) != 3 or
                not all(math.isfinite(v) for v in (*gyro,*acceleration,stamp))):
            self.samples.clear(); self.reason = 'IMU_INVALID'; return None
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.samples.clear(); self.reason = 'IMU_TIMESTAMP_NOT_INCREASING'; return None
        self.last_stamp = stamp
        if self.phase == 'FAULT': return None
        stationary = (self.command_still and 0 <= now-self.command_at <= .4 and
                      0 <= now-self.scan_at <= .4 and self.scan_still_since is not None)
        if not stationary:
            self.samples.clear()
            if self.bias is None: self.reason = 'WAITING_FOR_STATIONARY_SCAN_AND_COMMAND'
        else:
            self.samples.append((now,tuple(gyro),tuple(acceleration)))
            while self.samples and now-self.samples[0][0] > self.WINDOW+.1: self.samples.popleft()
            if (now >= self.evaluate_at and len(self.samples) >= 100 and now-self.samples[0][0] >= self.WINDOW
                    and now-self.scan_still_since >= self.WINDOW):
                self.evaluate_at=now+.5
                axes = list(zip(*(s[1] for s in self.samples)))
                means = tuple(statistics.mean(x) for x in axes)
                quiet = (all(statistics.pstdev(x) <= .01 for x in axes) and
                         all(statistics.pstdev(x) <= .30 for x in zip(*(s[2] for s in self.samples))))
                if self.bias is None:
                    if quiet and all(abs(x) <= .20 for x in means):
                        self.bias = means; self.phase = 'READY'; self.reason = 'STATIONARY_CALIBRATED'
                    else: self.reason = 'WAITING_FOR_STABLE_IMU'
                elif quiet and any(abs(x-b) > .04 for x,b in zip(means,self.bias)):
                    # Never change a running map's inertial reference silently.
                    self.phase = 'FAULT'; self.reason = 'IMU_STATIONARY_DRIFT'; return None
        if self.bias is None: return None
        return tuple(x-b for x,b in zip(gyro,self.bias))

    def status(self):
        return dict(state=self.phase, reason=self.reason, gyro_bias=self.bias,
                    calibration_window_sec=self.WINDOW, samples=len(self.samples))


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu,LaserScan
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from .scan_normalizer import normalize
    rclpy.init(); node=rclpy.create_node('mediassist_imu_stability')
    check=StationaryGyro(); received=[-math.inf]; output=[-math.inf]; last_reason=[None]
    publisher=node.create_publisher(Imu,'/imu_corrected',10)
    status=node.create_publisher(String,'/mediassist/imu_status',10)
    def imu(msg):
        now=time.monotonic(); stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=node.get_clock().now().nanoseconds/1e9-stamp
        if not 0 <= age <= .4: return
        received[0]=now
        g,a=msg.angular_velocity,msg.linear_acceleration
        corrected=check.feed((g.x,g.y,g.z),(a.x,a.y,a.z),now,stamp)
        if corrected is not None:
            msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=corrected
            publisher.publish(msg); output[0]=now
    def scan(msg):
        age=node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        if not 0 <= age <= .4: return
        try: check.scan(normalize(msg)['ranges'],time.monotonic())
        except (ValueError,TypeError): pass
    node.create_subscription(Imu,'/mediassist/imu_accel_calibrated',imu,10)
    node.create_subscription(LaserScan,'/scan_raw',scan,qos_profile_sensor_data)
    node.create_subscription(Twist,'/mediassist/cmd_vel_safe',lambda m:
        check.command((m.linear.x,m.linear.y,m.angular.z),time.monotonic()),10)
    def report():
        result=check.status()
        if check.phase=='READY' and time.monotonic()-output[0]>.5:
            result.update(state='WAITING',reason='IMU_SOURCE_STALE')
        status.publish(String(data=json.dumps(result)))
        key=(result['state'],result['reason'])
        if key != last_reason[0]:
            node.get_logger().info('IMU_STABILITY '+json.dumps(result)); last_reason[0]=key
    node.create_timer(.5,report)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
