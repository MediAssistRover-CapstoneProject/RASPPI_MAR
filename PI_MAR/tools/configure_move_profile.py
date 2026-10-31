"""Set the saved-map MOVE ceiling; preserve exploration, joystick, TLS and journals."""
import argparse,copy,json,math
from pathlib import Path
from contextlib import nullcontext
from configure_mapping import ROOT,load,stopped_configuration,write_configuration
from mediassist.configuration import validate_config

def tune(hardware,speed):
    if type(speed) not in (int,float) or not math.isfinite(speed) or not .05<=speed<=.8:
        raise ValueError('MOVE speed must be .05..0.80 m/s')
    if speed>hardware['max_speed_mps']:raise ValueError('MOVE exceeds hardware speed cap')
    result=copy.deepcopy(hardware);result['navigation']['max_speed_mps']=speed
    return validate_config(result)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--speed',type=float,default=.6);p.add_argument('--apply',action='store_true');a=p.parse_args()
    path=a.config.resolve(strict=True)
    with stopped_configuration(path) if a.apply else nullcontext():
        _,hw=load(path);raw=json.loads(path.read_text(encoding='utf-8-sig'));tuned=tune(hw,a.speed)
        if a.apply:
            raw=write_configuration(path,raw,hw,motion_options={'motion':hw['motion'],
                'max_speed_mps':hw['max_speed_mps'],'navigation':tuned['navigation']})
    print('MOVE_PROFILE_APPLIED' if a.apply else 'PLAN_ONLY')
    print('MOVE_MAX_MPS:',a.speed,'; new navigation sessions use forward only')
    print('Mapping and joystick speed unchanged; hardware profile:',raw['hardware_profile'])
if __name__=='__main__':main()
