"""Geometry regressions using physical points and rotated maps; no ROS/motors/network."""
from contextlib import nullcontext
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools'),str(ROOT/'pi/tools')]
from mediassist.geometry_diagnostics import planar_transform,scan_points,scan_summary,map_summary,nav_footprints
import configure_autonomy_geometry as tool

BODY=dict(front=.12,rear=.12,left=.10,right=.10)


class GeometryTests(unittest.TestCase):
    def transform(self,q):return N(translation=N(x=.1,y=.2),rotation=N(x=q[0],y=q[1],z=q[2],w=q[3]))
    def test_vendor_lidar_yaw_pi_turns_front_ray_toward_base_rear(self):
        tf=planar_transform(self.transform((0.,0.,1.,0.)))
        scan=N(ranges=[1.],angle_min=0.,angle_increment=.01,range_min=.05,range_max=8.)
        point=list(scan_points(scan,tf))[0]
        self.assertAlmostEqual(point[2],-.9);self.assertAlmostEqual(point[3],.2)
    def test_upside_down_planar_mount_reverses_angle_direction(self):
        tf=planar_transform(self.transform((1.,0.,0.,0.)))
        scan=N(ranges=[1.],angle_min=math.pi/2,angle_increment=.01,range_min=.05,range_max=8.)
        point=list(scan_points(scan,tf))[0]
        self.assertAlmostEqual(point[2],.1);self.assertAlmostEqual(point[3],-.8)
    def test_amcl_quality_matcher_uses_the_same_actual_sensor_transform(self):
        from mediassist.localization_node import LocalizationNode
        guard=object.__new__(LocalizationNode)
        guard.map_ready=Mock(return_value=True);guard.lifecycle_ready=Mock(return_value=True)
        guard.ros_now=Mock(return_value=10.);guard.odom=None;guard.pose=None;guard.particles=[]
        guard.hw={'ros':{'map_frame':'map','base_frame':'base_footprint'}}
        stamp=N(sec=10,nanosec=0)
        guard.scan=N(header=N(stamp=stamp,frame_id='lidar_frame'),ranges=[1.],angle_min=math.pi/2,
                     angle_increment=.01,range_min=.05,range_max=8.)
        guard.tf=Mock();guard.matcher=Mock();guard.matcher.score.return_value={'scan_match':1.}
        guard.tf.lookup_transform.side_effect=[N(header=N(stamp=stamp),transform=self.transform((0.,0.,0.,1.))),
                                              N(transform=self.transform((1.,0.,0.,0.)))]
        class Time:
            @staticmethod
            def from_msg(stamp):return stamp
        with patch.dict(sys.modules,{'rclpy.time':N(Time=Time)}):result=guard.evidence(10.)
        point=guard.matcher.score.call_args.args[0][0]
        self.assertAlmostEqual(point[0],.1);self.assertAlmostEqual(point[1],-.8)
        self.assertEqual(result['scan_match'],1.)
    def test_tilted_and_non_unit_transforms_are_rejected(self):
        for q in ((0.,0.,0.,2.),(math.sin(.3),0.,0.,math.cos(.3))):
            with self.assertRaises(ValueError):planar_transform(self.transform(q))
    def test_scan_distinguishes_raw_minimum_from_body_relative_distance(self):
        scan=N(ranges=[.08,.3,float('nan'),float('inf')],angle_min=0.,angle_increment=.1,range_min=.05,range_max=8.)
        result=scan_summary(scan,(-.01,0.,0.,1),BODY)
        self.assertEqual(result['finite_hits'],2);self.assertEqual(result['hits_inside_body'],1)
        self.assertEqual(result['nearest_lidar_m'],.08);self.assertEqual(result['near_hits'][0]['base_x_m'],.07)
    def test_map_occupied_under_body_is_reported_without_clearing_the_map(self):
        raw=bytearray([0]*400);raw[10*20+10]=100;raw[10*20+11]=255
        snap=dict(width=20,height=20,resolution=.05,origin=[0.,0.,0.],data=bytes(raw))
        result=map_summary(snap,dict(x_m=.525,y_m=.525,yaw_rad=0.),BODY)
        self.assertEqual(result['occupied_cells_inside_body'],1)
        self.assertEqual(result['seed_cell'],100);self.assertEqual(snap['data'],bytes(raw))
        self.assertEqual(result['nearest_occupied_cells'][0]['distance_to_base_m'],0.)
    def test_map_origin_rotation_and_robot_yaw_are_both_applied(self):
        raw=bytearray([0]*100);raw[4*10+4]=100
        snap=dict(width=10,height=10,resolution=.1,origin=[2.,3.,math.pi/2],data=bytes(raw))
        result=map_summary(snap,dict(x_m=1.55,y_m=3.45,yaw_rad=-math.pi/2),BODY)
        self.assertEqual(result['occupied_cells_inside_body'],1)
        self.assertAlmostEqual(result['nearest_occupied_cells'][0]['distance_to_base_m'],0.)
    def test_nav2_radius_is_reported_separately_from_autonomy_polygon(self):
        parameters={'local_costmap':{'local_costmap':{'ros__parameters':{'robot_radius':.05}}}}
        self.assertEqual(nav_footprints(parameters)['local_costmap']['robot_radius'],.05)
        self.assertIsNone(nav_footprints(parameters)['global_costmap']['footprint'])
    def test_geometry_only_configuration_preserves_all_other_modes_and_hardware(self):
        from test_camera_config import CameraConfigTests
        with tempfile.TemporaryDirectory() as folder:
            path,_,hardware=CameraConfigTests().fixture(folder)
            before=tool.load(path)[0];hw_before=hardware.read_bytes()
            expected=copy.deepcopy(before);expected['mapping'].pop('autonomy',None)
            with patch.object(tool,'stopped_configuration',return_value=nullcontext()):result=tool.configure(path,True)
            auto=result['mapping'].pop('autonomy')
            self.assertEqual(auto['footprint'],BODY);self.assertEqual(auto['lidar_stop_distance_m'],.1)
            self.assertFalse(auto['enabled'])
            self.assertEqual(result,expected);self.assertEqual(hardware.read_bytes(),hw_before)


if __name__=='__main__':unittest.main()
