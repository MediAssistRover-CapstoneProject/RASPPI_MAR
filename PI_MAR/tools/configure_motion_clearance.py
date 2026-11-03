"""Change bare-body clearance without changing speeds, identities or network/TLS."""
import argparse,copy,json,math
from contextlib import nullcontext
from pathlib import Path
from configure_mapping import ROOT,load,stopped_configuration,write_configuration
from mediassist.autonomous_mapping import configuration
from mediassist.configuration import validate_config

def tune(raw,hardware,stop=.02,slow=.05):
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (stop,slow)) or not .02<=stop<slow<=.10:
        raise ValueError('Require .02 <= stop < slow <= .10 metres from body')
    candidate=copy.deepcopy(raw);hw=copy.deepcopy(hardware);motion=hw['motion']
    auto=candidate['mapping']['autonomy'];f=auto['footprint']
    if (motion.get('chassis_model')!='MENTORPI_M1_MECANUM' or auto.get('vehicle')!='rigid'
            or not auto.get('enabled') or not motion.get('lidar_braking')
            or any(motion.get(k) is not True for k in ('commissioned','cart_footprint_validated','watchdog_validated'))
            or any(not .05<=f.get(k,10)<=v for k,v in dict(front=.15,rear=.15,left=.13,right=.13).items())):
        raise ValueError('COMMISSIONED_BARE_MECANUM_REQUIRED')
    auto['stop_margin_m']=stop
    configuration(candidate['mapping'])
    # Preserve the separately commissioned manual/MOVE angular and speed caps.
    motion['lidar_braking'].update(stop_margin_m=stop,near_slow_margin_m=slow)
    motion['rear_stop_distance_m']=f['rear']+stop
    validate_config(hw)
    return candidate,hw

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--trailer-removed',action='store_true',required=True)
    p.add_argument('--stop',type=float,default=.02);p.add_argument('--slow',type=float,default=.05)
    p.add_argument('--apply',action='store_true');a=p.parse_args();path=a.config.resolve(strict=True)
    with stopped_configuration(path) if a.apply else nullcontext():
        _,hardware=load(path);raw=json.loads(path.read_text(encoding='utf-8-sig'))
        candidate,hw=tune(raw,hardware,a.stop,a.slow)
        if a.apply:write_configuration(path,candidate,hardware,motion_options={'max_speed_mps':hw['max_speed_mps'],'motion':hw['motion']})
    print('CLEARANCE_APPLIED' if a.apply else 'PLAN_ONLY')
    print('Body stop:',a.stop,'m; near slowdown:',a.slow,'m; speed-dependent braking retained')

if __name__=='__main__':main()
