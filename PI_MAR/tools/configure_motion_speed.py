"""Configure matched mapping/MOVE speed ceilings while the stack is stopped."""
import argparse,copy,json,math
from contextlib import nullcontext
from pathlib import Path
from configure_mapping import ROOT,load,stopped_configuration,write_configuration
from mediassist.autonomous_mapping import configuration
from mediassist.configuration import validate_config

def tune(raw,hardware,speed,reverse,angular,stop_margin=.05):
    if type(stop_margin) not in (int,float) or not math.isfinite(stop_margin) or not .03<=stop_margin<=.10:
        raise ValueError('Stop margin must be .03..0.10 m from the body')
    for name,value,low,high in (('speed',speed,.05,.8),('reverse',reverse,.02,.8),('angular',angular,.1,1.2)):
        if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high:raise ValueError('Invalid '+name)
    if reverse>speed:raise ValueError('Reverse exceeds translation cap')
    hw=copy.deepcopy(hardware);motion=hw['motion'];auto=copy.deepcopy(raw['mapping']['autonomy'])
    if any(motion.get(key) is not True for key in ('commissioned','cart_footprint_validated','watchdog_validated','allow_reverse','allow_strafe')):
        raise ValueError('BARE_MECANUM_COMMISSIONING_REQUIRED')
    if motion.get('chassis_model')!='MENTORPI_M1_MECANUM' or auto.get('vehicle')!='rigid' or not auto.get('enabled'):
        raise ValueError('BARE_RIGID_MECANUM_REQUIRED')
    if angular>motion['max_angular_speed_rps']:raise ValueError('ANGULAR_HARDWARE_LIMIT_EXCEEDED')
    if any(auto['footprint'].get(k,10)>v for k,v in dict(front=.15,rear=.15,left=.13,right=.13).items()):
        raise ValueError('BARE_BODY_FOOTPRINT_REQUIRED')
    auto.update(speed_mps=speed,max_reverse_mps=reverse,angular_rps=angular,reverse_enabled=True,
                exploration_mode='frontier',frontier_controller='nav2',visit_known_space=True,
                stop_margin_m=stop_margin,turn_clearance_m=.20,
                slow_clearance_m=max(auto['slow_clearance_m'],1.9*speed),scan_max_age_sec=min(.4,auto['scan_max_age_sec']))
    candidate=copy.deepcopy(raw);candidate['mapping']['autonomy']=auto;configuration(candidate['mapping'])
    hw['max_speed_mps']=max(speed,hw['max_speed_mps']);motion['max_reverse_speed_mps']=reverse
    # The old 0.30 m centre-to-obstacle rear threshold leaves about 18 cm
    # behind the bare chassis. Keep roughly 6 cm beyond its measured rear edge.
    motion['rear_stop_distance_m']=max(.15,auto['footprint']['rear']+stop_margin+.01)
    motion['lidar_braking']={**copy.deepcopy(auto),'include_points':True}
    motion['lidar_braking']['angular_rps']=motion['max_angular_speed_rps']
    hw['navigation']['max_speed_mps']=speed
    validate_config(hw)
    return candidate,hw

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--trailer-removed',action='store_true',required=True)
    p.add_argument('--speed',type=float,required=True);p.add_argument('--reverse',type=float,required=True)
    p.add_argument('--angular',type=float,default=.8);p.add_argument('--apply',action='store_true')
    p.add_argument('--stop-margin',type=float,default=.05,help='Nominal body clearance; braking starts earlier according to speed')
    a=p.parse_args();path=a.config.resolve(strict=True)
    with stopped_configuration(path) if a.apply else nullcontext():
        _,hw=load(path);raw=json.loads(path.read_text(encoding='utf-8-sig'))
        candidate,tuned=tune(raw,hw,a.speed,a.reverse,a.angular,a.stop_margin)
        if a.apply:
            candidate=write_configuration(path,candidate,hw,motion_options={
                'max_speed_mps':tuned['max_speed_mps'],'motion':tuned['motion'],'navigation':tuned['navigation']})
    print('MOTION_SPEED_APPLIED' if a.apply else 'PLAN_ONLY')
    print('HARDWARE_PROFILE:',candidate['hardware_profile'])
    print('MAPPING_AND_MOVE:',json.dumps(dict(speed_mps=a.speed,reverse_mps=a.reverse,angular_rps=a.angular)))
if __name__=='__main__':main()
