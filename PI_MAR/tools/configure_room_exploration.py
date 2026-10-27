"""Opt in to Nav2 room exploration while stopped. Does not start nodes or send goals."""
import argparse,copy,json,math
from contextlib import nullcontext
from pathlib import Path
from configure_mapping import ROOT,load,stopped_configuration,write_configuration
from mediassist.autonomous_mapping import configuration

def tune(cfg,hardware,speed=.25):
    if type(speed) not in (int,float) or not math.isfinite(speed) or not .05<=speed<=.25:raise ValueError('Room test speed: .05..0.25 m/s')
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
                reverse_enabled=True,max_reverse_mps=min(.1,motion['max_reverse_speed_mps']),
                control_pause_sec=2.,speed_mps=min(speed,hardware['max_speed_mps']),
                angular_rps=min(.6,motion['max_angular_speed_rps']),slow_clearance_m=.65,
                max_duration_sec=600,frontier_no_progress_sec=30.)
    configuration(result['mapping']);return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--trailer-removed',action='store_true',required=True);p.add_argument('--speed',type=float,default=.25);p.add_argument('--apply',action='store_true')
    a=p.parse_args();config=a.config.resolve(strict=True)
    with stopped_configuration(config) if a.apply else nullcontext():
        _,hardware=load(config);cfg=json.loads(config.read_text(encoding='utf-8-sig'));result=tune(cfg,hardware,a.speed)
        if a.apply:result=write_configuration(config,result,hardware)
    print('ROOM_EXPLORATION_APPLIED' if a.apply else 'PLAN_ONLY; stop stack and add --apply')
    print(json.dumps(result['mapping']['autonomy'],indent=2))
    print('Footprint, motor commissioning, camera, network, device credentials and map journal preserved. No goal sent.')
if __name__=='__main__':main()
