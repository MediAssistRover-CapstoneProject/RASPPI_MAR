"""Offline regression for the opt-in 0.8 m/s mapping and MOVE profile."""
import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'pi'), str(ROOT / 'tools'), str(ROOT / 'pi/tools')]
from configure_motion_speed import tune
from mediassist.autonomous_mapping import configuration, nav2_assist
from mediassist.motion_braking import limit_velocity
from mediassist.navigation_setup import nav2_params


class MotionSpeedTests(unittest.TestCase):
    def profile(self):
        hw=json.loads((ROOT/'pi/config/site.example.json').read_text(encoding='utf-8'))
        hw['motion'].update(chassis_model='MENTORPI_M1_MECANUM',allow_strafe=True,
                            allow_reverse=True,max_reverse_speed_mps=.25,rear_stop_distance_m=.3,
                            commissioned=True,cart_footprint_validated=True,watchdog_validated=True,
                            max_angular_speed_rps=.8)
        autonomy=configuration({'autonomy':dict(enabled=True,footprint=dict(
            front=.12,rear=.12,left=.1,right=.1))})
        raw={'mapping':{'autonomy':autonomy,'upload_base_url':'https://unchanged'}}
        return raw, hw

    def test_mapping_move_and_braking_are_configured_together(self):
        raw, hw = self.profile()
        original = copy.deepcopy((raw, hw))
        configured, hardware = tune(raw, hw, .8, .8, .8)
        self.assertEqual((raw, hw), original)
        self.assertEqual(configured['mapping']['autonomy']['speed_mps'], .8)
        self.assertEqual(configured['mapping']['autonomy']['max_reverse_mps'], .8)
        self.assertEqual(hardware['max_speed_mps'], .8)
        self.assertEqual(hardware['navigation']['max_speed_mps'], .8)
        self.assertEqual(hardware['motion']['max_reverse_speed_mps'], .8)
        self.assertEqual(hardware['motion']['rear_stop_distance_m'], .18)
        self.assertEqual(configured['mapping']['autonomy']['turn_clearance_m'], .20)
        self.assertTrue(hardware['motion']['lidar_braking']['include_points'])
        self.assertEqual(configured['mapping']['upload_base_url'], raw['mapping']['upload_base_url'])

    def test_lidar_caps_both_directions_and_stops_on_stale_scan(self):
        _, hw = tune(*self.profile(), .8, .8, .8)
        profile = hw['motion']['lidar_braking']
        view = dict(valid=True, nearest_lidar_m=1., rotation_clear=True,
                    obstacle_points=[(.6, 0.), (-.6, 0.)])
        forward, reason = limit_velocity((.8, 0., 0.), view, .1, profile)
        reverse, _ = limit_velocity((-.8, 0., 0.), view, .1, profile)
        self.assertIsNone(reason)
        self.assertLess(forward[0], .8)
        self.assertLess(abs(reverse[0]), .8)
        self.assertEqual(limit_velocity((.8, 0., 0.), view, .5, profile)[0], (0., 0., 0.))
        self.assertEqual(limit_velocity((.8, 0., 0.), {**view, 'nearest_lidar_m':.05}, .1, profile)[0], (0., 0., 0.))
        self.assertEqual(limit_velocity((0., 0., .8), {**view, 'rotation_clear':False}, .1, profile)[0], (0., 0., 0.))
        near={**view,'nearest_lidar_m':.17,'obstacle_points':[(.17,0.)]}
        self.assertEqual(limit_velocity((.8,0.,0.),near,.1,profile)[0],(0.,0.,0.))

    def test_nav2_speed_uses_braking_distance_and_body_margin(self):
        configured, _ = tune(*self.profile(), .8, .8, .8)
        cfg = configured['mapping']['autonomy']
        view = dict(front_m=.05,back_m=1.,lateral_left_m=1.,lateral_right_m=1.)
        self.assertEqual(nav2_assist((.8,0.,0.),view,cfg,.1),(0.,0.,0.))
        self.assertLess(nav2_assist((.8,0.,0.),{**view,'front_m':.1},cfg,.1)[0],.15)
        self.assertAlmostEqual(nav2_assist((.8,0.,0.),{**view,'front_m':1.},cfg,.1)[0],.8)

    def test_navigation_params_allow_0_8_mecanum_and_reverse(self):
        base={'amcl':{'ros__parameters':{}},'velocity_smoother':{'ros__parameters':{}},
              'planner_server':{'ros__parameters':{'GridBased':{}}}}
        for name in ('local_costmap','global_costmap'):
            base[name]={name:{'ros__parameters':{'inflation_layer':{}}}}
        controller={'controller_server':{'ros__parameters':{
            'FollowPath':{'plugin':'dwb_core::DWBLocalPlanner','critics':['BaseObstacle']},
            'progress_checker':{},'general_goal_checker':{}}}}
        params=nav2_params(base, controller, [], .8, .8, mecanum=True, fast_navigation=True)
        follow=params['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual((follow['min_vel_x'], follow['max_vel_x'], follow['max_vel_y']), (-.8,.8,0.))
        self.assertEqual(follow['vy_samples'],1)
        self.assertEqual(params['velocity_smoother']['ros__parameters']['min_velocity'], [-.8,0.,-.8])
        self.assertIn('mar_keepout',params['global_costmap']['global_costmap']['ros__parameters']['filters'])
        self.assertEqual(params['global_costmap']['global_costmap']['ros__parameters']['mar_keepout']['filter_info_topic'],'/mediassist/keepout_info')
        self.assertEqual(params['local_costmap']['local_costmap']['ros__parameters']['mar_keepout']['filter_info_topic'],'/mediassist/keepout_local_info')

    def test_three_centimetre_margin_preserves_braking(self):
        configured, hardware=tune(*self.profile(),.8,.8,.8,.03)
        self.assertEqual(configured['mapping']['autonomy']['stop_margin_m'],.03)
        self.assertAlmostEqual(hardware['motion']['rear_stop_distance_m'],.16)


if __name__ == '__main__':
    unittest.main()
