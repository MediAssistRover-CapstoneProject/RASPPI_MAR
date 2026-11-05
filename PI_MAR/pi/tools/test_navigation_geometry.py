"""Rigid rear cargo: map reuse, rear collision and shared Nav2/braking bounds. No ROS."""
import copy
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'pi'), str(ROOT/'tools')]
from configure_navigation_geometry import tune
from mediassist.contracts import StepError
from mediassist.footprint import body_clear
from mediassist.motion_braking import limit_velocity
from mediassist.navigation_geometry import session_geometry
from mediassist.navigation_setup import nav2_params, footprint_rectangle


BARE = dict(front=.109, rear=.104, left=.086, right=.086)


def fixtures():
    cfg = {'mapping': {'enabled': True, 'autonomy': {'vehicle': 'rigid', 'footprint': dict(BARE)}},
           'hardware_profile': '/unchanged/site.json', 'token_url': 'https://be.example.test/token'}
    hw = {'motion': {'chassis_model': 'MENTORPI_M1_MECANUM', 'rear_stop_distance_m': .13,
                    'lidar_braking': {'footprint': dict(BARE), 'stop_margin_m': .02,
                                     'scan_max_age_sec': .4, 'lidar_stop_distance_m': .05,
                                     'speed_mps': .6}}, 'max_speed_mps': .6,
          'navigation': {'max_speed_mps': .6}}
    return cfg, hw


class CargoGeometryTests(unittest.TestCase):
    def test_mapping_and_hardware_base_unchanged_across_repeated_sessions(self):
        cfg, hw = fixtures()
        before = copy.deepcopy((cfg, hw))
        configured, _ = tune(cfg, hw, .10, .15)
        self.assertEqual((cfg, hw), before)
        self.assertEqual(configured['mapping'], cfg['mapping'])
        for _ in range(2):
            runtime, hardware, evidence = session_geometry(configured, hw)
            self.assertAlmostEqual(evidence['effective_footprint_m']['rear'], .204)
            self.assertEqual(runtime['mapping']['autonomy']['footprint'], hardware['motion']['lidar_braking']['footprint'])
            self.assertEqual(hardware['navigation'], hw['navigation'])
            self.assertEqual(hardware['max_speed_mps'], hw['max_speed_mps'])
            self.assertEqual(runtime['token_url'], cfg['token_url'])
            self.assertEqual(runtime['hardware_profile'], cfg['hardware_profile'])
            self.assertGreaterEqual(hardware['motion']['rear_stop_distance_m'], .224)
        self.assertEqual(configured['mapping']['autonomy']['footprint'], BARE)
        self.assertEqual(hw['motion']['lidar_braking']['footprint'], BARE)

    def test_bare_override_and_legacy_default(self):
        cfg, hw = fixtures()
        self.assertEqual(session_geometry(cfg, hw)[2]['profile'], 'bare')
        configured, _ = tune(cfg, hw, .10, .15)
        runtime, hardware, evidence = session_geometry(configured, hw, 'bare')
        self.assertEqual(evidence['profile'], 'bare')
        self.assertEqual(runtime['mapping']['autonomy']['footprint'], BARE)
        self.assertEqual(hardware['motion']['lidar_braking']['footprint'], BARE)

    def test_invalid_dimensions_and_unconfigured_cargo_rejected(self):
        cfg, hw = fixtures()
        for length, width in ((-.1, .15), (.1, 0.), (math.nan, .15), (.1, True), (.1, math.inf), (1., .15)):
            with self.subTest(length=length, width=width), self.assertRaises(StepError):
                tune(cfg, hw, length, width)
        with self.assertRaisesRegex(StepError, 'CARGO_DIMENSIONS_REQUIRED'):
            session_geometry(cfg, hw, 'cargo')

    def test_articulated_or_missing_lidar_profile_is_not_silently_treated_as_rigid(self):
        cfg, hw = fixtures()
        cfg['mapping']['autonomy']['vehicle'] = 'articulated'
        with self.assertRaisesRegex(StepError, 'RIGID_CARGO_PROFILE_REQUIRED'):
            tune(cfg, hw, .10, .15)
        cfg['mapping']['autonomy']['vehicle'] = 'rigid'
        del hw['motion']['lidar_braking']
        with self.assertRaisesRegex(StepError, 'CARGO_LIDAR_BRAKING_REQUIRED'):
            tune(cfg, hw, .10, .15)

    def test_generated_runtime_cannot_add_extension_twice(self):
        cfg, hw = fixtures()
        configured, _ = tune(cfg, hw, .10, .15)
        configured['navigation_session'] = {'session_id': 'already-running'}
        with self.assertRaises(StepError) as error:
            session_geometry(configured, hw)
        self.assertEqual(error.exception.code, 'USE_BASE_GEOMETRY_CONFIG')

    def test_rear_obstacle_clear_for_bare_robot_is_blocked_with_cargo(self):
        cfg, hw = fixtures()
        configured, _ = tune(cfg, hw, .10, .15)
        extent = session_geometry(configured, hw)[2]['effective_footprint_m']
        for yaw, cell in ((0., (42, 50)), (math.pi/2, (50, 42))):
            data = bytearray(100*100)
            data[cell[1]*100 + cell[0]] = 100
            grid = dict(width=100, height=100, resolution=.02, origin=[0., 0., 0.], data=bytes(data))
            pose = dict(x_m=1., y_m=1., yaw_rad=yaw)
            with self.subTest(yaw=yaw):
                self.assertTrue(body_clear(grid, pose, BARE)['clear'])
                self.assertFalse(body_clear(grid, pose, extent)['clear'])

    def test_loaded_rear_braking_stops_before_box_reaches_obstacle(self):
        cfg, hw = fixtures()
        configured, _ = tune(cfg, hw, .10, .15)
        hardware = session_geometry(configured, hw)[1]
        view = dict(valid=True, nearest_lidar_m=.22, rotation_clear=True, obstacle_points=[(-.22, 0.)])
        speed = (-.15, 0., 0.)
        self.assertLess(limit_velocity(speed, view, .02, hw['motion']['lidar_braking'])[0][0], 0.)
        self.assertEqual(limit_velocity(speed, view, .02, hardware['motion']['lidar_braking'])[0], (0., 0., 0.))

    def test_nav2_both_costmaps_and_padding_match_loaded_bounds(self):
        cfg, hw = fixtures()
        configured, _ = tune(cfg, hw, .10, .15)
        extent = session_geometry(configured, hw)[2]['effective_footprint_m']
        polygon = footprint_rectangle(**extent)
        base = {'amcl': {'ros__parameters': {}}, 'velocity_smoother': {'ros__parameters': {}},
                'planner_server': {'ros__parameters': {'GridBased': {}}}}
        for name in ('global_costmap', 'local_costmap'):
            base[name] = {name: {'ros__parameters': {'inflation_layer': {}}}}
        controller = {'controller_server': {'ros__parameters': {
            'FollowPath': {'plugin': 'dwb_core::DWBLocalPlanner', 'critics': ['BaseObstacle']},
            'progress_checker': {}, 'general_goal_checker': {}}}}
        options = dict(mecanum=True, fast_navigation=True, position_then_spin=True,
                       footprint_planner=True, forward_only=True, path_tracking=True)
        params = nav2_params(base, controller, polygon, .6, .8, **options, full_footprint_search=True)
        self.assertEqual(params['planner_server']['ros__parameters']['GridBased']['plugin'],
                         'nav2_smac_planner/SmacPlannerHybrid')
        self.assertEqual(params['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'],.05)
        follow=params['controller_server']['ros__parameters']['FollowPath']
        self.assertGreaterEqual(follow['min_lookahead_dist'],.25)
        self.assertLessEqual(follow['max_lookahead_dist'],.5)
        self.assertTrue(follow['use_collision_detection'])
        bare_params = nav2_params(base, controller, footprint_rectangle(**BARE), .6, .8, **options)
        self.assertEqual(bare_params['planner_server']['ros__parameters']['GridBased']['plugin'],
                         'nav2_smac_planner/SmacPlanner2D')
        import json
        for name in ('global_costmap', 'local_costmap'):
            costmap = params[name][name]['ros__parameters']
            self.assertEqual(json.loads(costmap['footprint']), polygon)
            self.assertEqual(costmap['footprint_padding'], .02)

    def test_session_generator_writes_consistent_private_runtime_without_changing_base(self):
        import json
        import os
        import run_navigation_session as runner
        cfg, hw = fixtures()
        configured, _ = tune(cfg, hw, .10, .15)
        configured.update(robot_id='robot', enabled_step_types=['WAIT'])
        hw.update(capabilities=[], ros={'map_frame': 'map'})
        hw['motion'].update(allow_strafe=True, max_angular_speed_rps=.8)
        hw['navigation']['planning_timeout_sec'] = 8.
        before = copy.deepcopy((configured, hw))
        descriptor = dict(sessionId='session', mapId='map', mapAssetHash='a'*64, expiresAt='2099-01-01T00:00:00Z')
        client = Mock()
        client.get.return_value = descriptor
        client.request.return_value = b'{}'
        base = {'amcl': {'ros__parameters': {}}, 'velocity_smoother': {'ros__parameters': {}},
                'planner_server': {'ros__parameters': {'GridBased': {}}}}
        for name in ('global_costmap', 'local_costmap'):
            base[name] = {name: {'ros__parameters': {'inflation_layer': {}}}}
        controller = {'controller_server': {'ros__parameters': {
            'FollowPath': {'plugin': 'dwb_core::DWBLocalPlanner', 'critics': ['BaseObstacle']},
            'progress_checker': {}, 'general_goal_checker': {}}}}
        read_text = Path.read_text
        def read(path, *args, **kwargs):
            if path.as_posix().endswith('/navigation/config/nav2_params.yaml'):
                return json.dumps(base)
            if path.as_posix().endswith('/navigation/config/nav2_controller_dwb.yaml'):
                return json.dumps(controller)
            return read_text(path, *args, **kwargs)
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ):
            with patch.object(runner, 'ROOT', Path(folder)), patch.object(runner, 'load', return_value=(configured, hw)), \
                 patch.object(runner, 'SessionClient', return_value=client), \
                 patch.object(runner, 'verify_bundle', return_value=({'map.yaml': b'image: map.pgm\n'}, {'asset_hash': 'a'*64, 'frame_id': 'map'})), \
                 patch.object(runner, 'validate_profile', return_value={'endpoints': {}}), \
                 patch.object(runner, 'nav_parameters', side_effect=lambda params, hardware: params), \
                 patch.object(Path, 'is_file', return_value=True), patch.object(Path, 'read_text', new=read):
                root, _, _ = runner.prepare(Path(folder)/'base.json', 'session')
            runtime = json.loads((root/'worker.json').read_text())
            hardware = json.loads((root/'hardware.json').read_text())
            import yaml
            params = yaml.safe_load((root/'nav2.yaml').read_text())
            self.assertFalse(runtime['mapping']['enabled'])
            self.assertTrue(runtime['navigation_runtime']['full_footprint_search'])
            extent = runtime['mapping']['autonomy']['footprint']
            self.assertAlmostEqual(extent['rear'], .204)
            self.assertEqual(extent, hardware['motion']['lidar_braking']['footprint'])
            for name in ('global_costmap', 'local_costmap'):
                self.assertEqual(json.loads(params[name][name]['ros__parameters']['footprint']), footprint_rectangle(**extent))
        self.assertEqual((configured, hw), before)


if __name__ == '__main__':
    unittest.main()
