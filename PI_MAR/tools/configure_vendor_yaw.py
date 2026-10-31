"""Match the installed Hiwonder joystick launch ceiling; configure MOVE separately."""
import argparse,copy,json,math
from contextlib import nullcontext
from pathlib import Path
from configure_mapping import ROOT,load,stopped_configuration,write_configuration
from mediassist.configuration import validate_config
from mediassist.gamepad import configuration


def tune(raw,hardware,manual=2.,move=1.2):
    if any(type(v) not in (int,float) or not math.isfinite(v) or not .1<=v<=2. for v in (manual,move)):
        raise ValueError('Yaw ceilings must be .1..2.0 rad/s')
    if move>manual:raise ValueError('MOVE cannot exceed the hardware/manual ceiling')
    cfg=copy.deepcopy(raw);hw=copy.deepcopy(hardware);m=hw['motion']
    if m.get('chassis_model')!='MENTORPI_M1_MECANUM' or cfg['mapping']['autonomy'].get('vehicle')!='rigid':
        raise ValueError('BARE_RIGID_MECANUM_REQUIRED')
    if not m.get('lidar_braking') or any(not m.get(k) for k in ('commissioned','cart_footprint_validated','watchdog_validated')):
        raise ValueError('COMMISSIONED_LIDAR_BRAKING_REQUIRED')
    gamepad=configuration(cfg['mapping'])
    if gamepad['layout']!='vendor_mecanum':raise ValueError('VENDOR_MECANUM_LAYOUT_REQUIRED')
    gamepad.update(angular_rps=manual,angular_accel=3.)
    m['max_angular_speed_rps']=manual;m['lidar_braking']['angular_rps']=manual
    hw['navigation']['max_angular_speed_rps']=move
    validate_config(hw);configuration(cfg['mapping'])
    return cfg,hw


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--trailer-removed',action='store_true',required=True)
    p.add_argument('--manual',type=float,default=2.);p.add_argument('--move',type=float,default=1.2)
    p.add_argument('--apply',action='store_true');a=p.parse_args();path=a.config.resolve(strict=True)
    with stopped_configuration(path) if a.apply else nullcontext():
        _,hw=load(path);raw=json.loads(path.read_text(encoding='utf-8-sig'))
        cfg,tuned=tune(raw,hw,a.manual,a.move)
        if a.apply:write_configuration(path,cfg,hw,motion_options={k:tuned[k] for k in ('max_speed_mps','motion','navigation')})
    print('VENDOR_YAW_APPLIED' if a.apply else 'PLAN_ONLY',json.dumps(dict(manual_rps=a.manual,move_rps=a.move,mapping_rps=cfg['mapping']['autonomy']['angular_rps'])))
if __name__=='__main__':main()
