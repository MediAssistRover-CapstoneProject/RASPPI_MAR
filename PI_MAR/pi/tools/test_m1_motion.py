"""M1 reverse guard and signed ramps. No motors, GPIO or ROS are touched."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.rear_scan import rear_clearance,chassis_guard
from mediassist.mapping_control import VelocityRamp


class M1MotionTests(unittest.TestCase):
    def scan(self):
        return SimpleNamespace(angle_min=-math.pi,angle_increment=math.pi/180,range_min=.05,range_max=10.,ranges=[1.]*360)

    def test_rear_clear_and_front_obstacle_does_not_block_backing_away(self):
        scan=self.scan();scan.ranges[180]=.1
        self.assertEqual(rear_clearance(scan,(0.,0.,0.),.3),'CLEAR')

    def test_rear_obstacle_and_rotated_lidar_use_base_direction(self):
        scan=self.scan();scan.ranges[0]=.1
        self.assertEqual(rear_clearance(scan,(0.,0.,0.),.3),'REAR_OBSTACLE')
        self.assertEqual(rear_clearance(scan,(0.,0.,math.pi),.3),'CLEAR')
        scan.ranges[180]=.1
        self.assertEqual(rear_clearance(scan,(0.,0.,math.pi),.3),'REAR_OBSTACLE')

    def test_invalid_or_missing_rear_coverage_is_not_clear(self):
        scan=self.scan();scan.ranges=[float('nan')]*360
        self.assertEqual(rear_clearance(scan,(0.,0.,0.),.3),'REAR_LIDAR_NOT_READY')
        scan=self.scan();scan.angle_min=-math.pi/2;scan.ranges=scan.ranges[:180]
        self.assertEqual(rear_clearance(scan,(0.,0.,0.),.3),'REAR_LIDAR_NOT_READY')
        scan.ranges=[]
        self.assertEqual(rear_clearance(scan,(0.,0.,0.),.3),'REAR_LIDAR_INVALID')

    def test_valid_no_hit_is_distinct_from_zero_or_nan(self):
        scan=self.scan();scan.ranges=[float('inf')]*360
        self.assertEqual(rear_clearance(scan,(0.,0.,0.),.3),'CLEAR')
        scan.ranges=[0.]*360
        self.assertEqual(rear_clearance(scan,(0.,0.,0.),.3),'REAR_LIDAR_NOT_READY')

    def test_reverse_needs_m1_profile_and_fresh_rear_guard(self):
        profile={'chassis_model':'MENTORPI_M1_MECANUM','allow_reverse':True,'max_reverse_speed_mps':.1,'sensor_max_age_sec':.6}
        self.assertEqual(chassis_guard((-.15,0.,0.),profile,'CLEAR',.1),((-.1,0.,0.),None))
        self.assertEqual(chassis_guard((-.1,0.,0.),profile,'CLEAR',.7)[1],'STALE_REAR_LIDAR')
        self.assertEqual(chassis_guard((-.1,0.,0.),profile,'REAR_OBSTACLE',.1)[1],'REAR_OBSTACLE')
        self.assertEqual(chassis_guard((-.1,0.,0.),{},'CLEAR',.1)[1],'REVERSE_NOT_ENABLED')
        self.assertEqual(chassis_guard((.15,0.,.45),profile,'REAR_OBSTACLE',10.),((.15,0.,.45),None))
        self.assertEqual(chassis_guard((0.,.1,0.),profile,'CLEAR',.1)[1],'STRAFE_NOT_ENABLED')

    def test_signed_ramp_and_direction_change_cross_zero_without_jump(self):
        ramp=VelocityRamp()
        previous=(0.,0.)
        for i in range(30):
            value=ramp.update((-.1,-.45),i*.05)
            self.assertLessEqual(abs(value[0]-previous[0]),.01251)
            self.assertLessEqual(abs(value[1]-previous[1]),.04001)
            previous=value
        self.assertEqual(previous,(-.1,-.45))
        value=ramp.update((.1,.45),1.5)
        self.assertLess(value[0],0);self.assertLess(value[1],0)
        self.assertEqual(ramp.update((0.,0.),1.55),(0.,0.))


if __name__=='__main__':unittest.main()
