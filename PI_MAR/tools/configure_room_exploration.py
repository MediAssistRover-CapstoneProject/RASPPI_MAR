"""Opt in to Nav2 room exploration while stopped. Does not start nodes or send goals."""
import argparse,copy,json,math
from contextlib import nullcontext
from pathlib import Path
from configure_mapping import ROOT,load,stopped_configuration,write_configuration
from mediassist.autonomous_mapping import configuration

def tune(cfg,hardware,speed=.25,reverse=.15,angular=.8,escape_min=.10,escape_max=.15,observation_rotation=True,lidar_avoidance=True):
    if type(speed) not in (int,float) or not math.isfinite(speed) or not .05<=speed<=.8:raise ValueError('Room exploration speed: .05..0.80 m/s')
    if type(reverse) not in (int,float) or not math.isfinite(reverse) or not .02<=reverse<=.8:raise ValueError('Reverse: .02..0.80 m/s')
    if type(angular) not in (int,float) or not math.isfinite(angular) or not .1<=angular<=1.2:raise ValueError('Angular: .10..1.20 rad/s')
    motion=hardware['motion']
    if motion.get('chassis_model')!='MENTORPI_M1_MECANUM' or not motion.get('allow_strafe') or not motion.get('allow_reverse'):
        raise ValueError('COMMISSIONED_BARE_MECANUM_WITH_REVERSE_REQUIRED')
    if not all(motion.get(k) is True for k in ('commissioned','cart_footprint_validated','watchdog_validated')):
        raise ValueError('EXISTING_COMMISSIONING_REQUIRED')
    result=copy.deepcopy(cfg);auto=configuration(result['mapping'])
    if not auto['enabled'] or auto['vehicle']!='rigid':raise ValueError('REMOVE_TRAILER_AND_CONFIGURE_RIGID_FOOTPRINT_FIRST')
    if any(auto['footprint'].get(k,10)>v for k,v in dict(front=.15,rear=.15,left=.13,right=.13).items()):
        raise ValueError('EXISTING_BARE_BODY_FOOTPRINT_REQUIRED')
    auto.update(exploration_mode='frontier',frontier_controller='nav2',visit_known_space=True,visit_radius_m=.4,
                reverse_enabled=True,max_reverse_mps=min(reverse,speed,motion['max_reverse_speed_mps']),
                control_pause_sec=2.,speed_mps=min(speed,hardware['max_speed_mps']),
                angular_rps=min(angular,motion['max_angular_speed_rps']),slow_clearance_m=max(.65,.8 if speed>.25 else .65,1.9*speed if speed>.5 else 0.),
                scan_max_age_sec=min(.4,auto['scan_max_age_sec']),
                frontier_escape_min_m=escape_min,frontier_escape_max_m=escape_max,
                frontier_observation_rotation=observation_rotation,frontier_observation_max_rad=math.tau,
                frontier_lidar_avoidance=lidar_avoidance,
                max_duration_sec=600,frontier_no_progress_sec=30.)
    braking=motion.get('lidar_braking')
    if isinstance(braking,dict) and braking.get('near_slow_margin_m') is not None:
        auto['near_slow_margin_m']=braking['near_slow_margin_m']
    configuration(result['mapping']);return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--trailer-removed',action='store_true',required=True);p.add_argument('--speed',type=float,default=.25);p.add_argument('--apply',action='store_true')
    p.add_argument('--reverse',type=float,default=.15);p.add_argument('--angular',type=float,default=.8)
    p.add_argument('--escape-min',type=float,default=.10);p.add_argument('--escape-max',type=float,default=.15)
    p.add_argument('--no-observation-rotation',dest='observation_rotation',action='store_false',
                   help='Disable bounded observation turns during frontier recovery')
    p.add_argument('--no-lidar-avoidance',dest='lidar_avoidance',action='store_false',
                   help='Disable vendor-style LiDAR recovery preferences; retain collision stops')
    a=p.parse_args();config=a.config.resolve(strict=True)
    with stopped_configuration(config) if a.apply else nullcontext():
        _,hardware=load(config);cfg=json.loads(config.read_text(encoding='utf-8-sig'));result=tune(cfg,hardware,a.speed,a.reverse,a.angular,a.escape_min,a.escape_max,a.observation_rotation,a.lidar_avoidance)
        if a.apply:result=write_configuration(config,result,hardware)
    print('ROOM_EXPLORATION_APPLIED' if a.apply else 'PLAN_ONLY; stop stack and add --apply')
    print(json.dumps(result['mapping']['autonomy'],indent=2))
    print('Footprint, motor commissioning, camera, network, device credentials and map journal preserved. No goal sent.')
if __name__=='__main__':main()
