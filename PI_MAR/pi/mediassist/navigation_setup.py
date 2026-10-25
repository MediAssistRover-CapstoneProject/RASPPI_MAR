"""Offline navigation preparation; measured dimensions are supplied by the operator."""
import copy
import json
import math
from .configuration import validate_config


def footprint_rectangle(front, rear, left, right):
    values = (front, rear, left, right)
    if any(type(v) not in (int, float) or not math.isfinite(v) or not 0.05 <= v <= 5 for v in values):
        raise ValueError("Each measured extent must be finite and within 0.05..5 metres")
    return [[front, left], [-rear, left], [-rear, -right], [front, -right]]


def nav2_params(base, controller, footprint, speed=0.05, angular=0.3, mecanum=False, fast_mapping=False):
    limit = 1.00 if fast_mapping and mecanum else .25
    yaw_limit = .8 if fast_mapping and mecanum else .6
    if not 0 < speed <= limit or not 0 < angular <= yaw_limit:
        raise ValueError(f"Navigation limits: 0 < linear <= {limit} m/s; 0 < angular <= {yaw_limit} rad/s; hardware caps also apply")
    result = copy.deepcopy(base)
    result.update(copy.deepcopy(controller))
    result['amcl']['ros__parameters']['set_initial_pose'] = False
    result['amcl']['ros__parameters']['scan_topic'] = '/scan_raw'
    for name in ('local_costmap', 'global_costmap'):
        costmap = result[name][name]['ros__parameters']
        costmap.pop('robot_radius', None)
        costmap['footprint'] = json.dumps(footprint)
        costmap['footprint_padding'] = 0.03
        costmap['inflation_layer']['inflation_radius'] = max(0.3, costmap['inflation_layer'].get('inflation_radius', 0.3))
    ctl = result['controller_server']['ros__parameters']
    follow = ctl['FollowPath']
    if follow.get('plugin') != 'dwb_core::DWBLocalPlanner':
        raise ValueError('Expected installed Hiwonder DWB profile; do not replace an unknown controller silently')
    lateral = speed if mecanum else 0.0
    follow.update(min_vel_x=0.0, min_vel_y=-lateral, max_vel_x=speed, max_vel_y=lateral,
                  max_speed_xy=speed, max_vel_theta=angular, min_speed_theta=0.0,
                  trans_stopped_velocity=0.01, vy_samples=5 if mecanum else 1,xy_goal_tolerance=.05)
    if mecanum:
        # Keep the extra Y search bounded on Pi; do not multiply the vendor's
        # 20x20 sampling by seven while also publishing trajectory debug data.
        follow.update(vx_samples=10,vtheta_samples=12,debug_trajectory_details=False)
    # DWB's centre-point BaseObstacle critic alone does not check the cart polygon.
    follow['critics'] = ['ObstacleFootprint' if x == 'BaseObstacle' else x for x in follow['critics']]
    follow.pop('BaseObstacle.scale', None)
    follow['ObstacleFootprint.scale'] = 1.0
    ctl['progress_checker'].update(required_movement_radius=0.1, movement_time_allowance=20.0)
    ctl['general_goal_checker'].update(xy_goal_tolerance=.05,yaw_goal_tolerance=.10)
    ctl.update(min_y_velocity_threshold=.001)
    follow.update(acc_lim_x=.40,decel_lim_x=-.6,acc_lim_y=.40 if mecanum else 0.,
                  decel_lim_y=-.6 if mecanum else 0.,acc_lim_theta=1.2,decel_lim_theta=-1.8)
    result['velocity_smoother']['ros__parameters'].update(
        max_velocity=[speed, lateral, angular], min_velocity=[0.0, -lateral, -angular],
        scale_velocities=True, smoothing_frequency=20.,
        max_accel=[.40,.40 if mecanum else 0.,1.2],max_decel=[-.6,-.6 if mecanum else 0.,-1.8],velocity_timeout=.25)
    result['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    result['planner_server']['ros__parameters']['GridBased']['tolerance'] = .05
    return result


def add_group(cfg, group_id, endpoint_ids, revision=1):
    cfg = copy.deepcopy(cfg)
    if not group_id or type(revision) is not int or revision < 1:
        raise ValueError('Non-empty group ID and positive revision required')
    if not 1 <= len(endpoint_ids) <= cfg['navigation']['max_candidates'] or len(endpoint_ids) != len(set(endpoint_ids)):
        raise ValueError('Invalid or duplicate group members')
    if not set(endpoint_ids) <= set(cfg['endpoints']):
        raise ValueError('Group contains an endpoint absent from this map')
    cfg['endpoint_groups'][group_id] = {'map_id': cfg['map_id'], 'revision': revision, 'endpoint_ids': endpoint_ids}
    return validate_config(cfg)
