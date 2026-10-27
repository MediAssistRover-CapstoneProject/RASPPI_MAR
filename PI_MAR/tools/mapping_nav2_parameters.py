"""Generate session-local Nav2 parameters from installed Hiwonder YAML, without editing vendor files."""
import copy
from pathlib import Path
import yaml
from mediassist.navigation_setup import nav2_params,footprint_rectangle

def mapping_motion_parameters(params,auto,hardware):
    follow=params['controller_server']['ros__parameters']['FollowPath']
    if auto.get('reverse_enabled'):
        if hardware['motion'].get('allow_reverse') is not True:raise RuntimeError('HARDWARE_REVERSE_NOT_ENABLED')
        reverse=min(.1,auto['max_reverse_mps'],hardware['motion']['max_reverse_speed_mps'])
        follow['min_vel_x']=-reverse
        params['velocity_smoother']['ros__parameters']['min_velocity'][0]=-reverse
    # Mapping uses a 360-degree scan. A viewpoint does not need a forced final spin.
    params['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance']=3.141592653589793
    follow['critics']=[name for name in follow['critics'] if name!='RotateToGoal']
    follow['GoalAlign.scale']=0.;follow['PathAlign.scale']=0.
    return params


def create(mapping,hardware,destination):
    auto=mapping['autonomy'];extent=auto['footprint']
    if auto['vehicle']!='rigid':raise RuntimeError('NAV2_MAPPING_REQUIRES_RIGID_BODY_PROFILE')
    folder=Path('/home/ubuntu/ros2_ws/src/navigation/config')
    base=yaml.safe_load((folder/'nav2_params.yaml').read_text());controller=yaml.safe_load((folder/'nav2_controller_dwb.yaml').read_text())
    params=nav2_params(base,controller,footprint_rectangle(**extent),min(auto['speed_mps'],hardware['max_speed_mps']),
        min(auto['angular_rps'],hardware['motion']['max_angular_speed_rps']),
        mecanum=hardware['motion'].get('chassis_model')=='MENTORPI_M1_MECANUM' and hardware['motion'].get('allow_strafe') is True,
        fast_mapping=True)
    mapping_motion_parameters(params,auto,hardware)
    # A frontier is an observation viewpoint, not a docking pose. Navfn may
    # reject its exact grid cell at the edge of a newly observed doorway even
    # though a nearby free cell is reachable. Keep collision checks in Nav2.
    params['planner_server']['ros__parameters']['GridBased']['tolerance']=.12
    for name in ('local_costmap','global_costmap'):
        c=params[name][name]['ros__parameters'];c.update(global_frame=hardware['ros']['map_frame'] if name=='global_costmap' else 'odom',
            robot_base_frame=hardware['ros']['base_frame'],resolution=mapping['slam_resolution_m'])
        for layer in ('obstacle_layer','voxel_layer'):
            if layer in c:
                for source in c[layer].get('observation_sources','').split():
                    if source in c[layer]:c[layer][source].update(topic=hardware['ros']['scan'],clearing=True,marking=True)
    params['bt_navigator']['ros__parameters'].update(global_frame=hardware['ros']['map_frame'],robot_base_frame=hardware['ros']['base_frame'],odom_topic=hardware['ros']['odom'])
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True)
    temporary=destination.with_suffix('.new');temporary.write_text(yaml.safe_dump(params,sort_keys=False));temporary.replace(destination)
    return destination
