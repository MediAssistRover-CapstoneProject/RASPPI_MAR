"""Opt in to bounded LiDAR exploration; preserve credentials, commissioning and motion limits."""
import argparse
from contextlib import nullcontext
import copy
import json
from pathlib import Path
from configure_mapping import ROOT, load, stopped_configuration, write_configuration
from mediassist.autonomous_mapping import configuration


def configure(path, enabled, footprint=None, speed=.10, angular=.30, duration=180, vehicle='rigid', turn_radius=.60, apply=False, exploration='reactive', manual_assist=False):
    path=Path(path).resolve(strict=True)
    with stopped_configuration(path) if apply else nullcontext():
        cfg,hardware=load(path)
        before=copy.deepcopy(cfg)
        auto=configuration(cfg['mapping'])
        auto['enabled']=enabled
        auto['exploration_mode']=exploration
        auto['manual_lidar_assist']=bool(enabled and manual_assist)
        if enabled:
            if footprint is None:raise ValueError('Measure front/rear/left/right from base_footprint in metres first')
            auto.update(vehicle=vehicle,min_turn_radius_m=turn_radius,footprint=footprint,speed_mps=speed,angular_rps=angular,max_duration_sec=duration)
            if vehicle=='articulated':
                import math
                radius=math.hypot(max(footprint['front'],footprint['rear']),max(footprint['left'],footprint['right']))+auto['stop_margin_m']
                auto['turn_clearance_m']=max(.65,radius+.15-footprint['front'])
                auto['slow_clearance_m']=auto['turn_clearance_m']+.35
                if exploration=='frontier':auto['blocked_timeout_sec']=20.
            if speed>hardware['max_speed_mps'] or angular>hardware['motion']['max_angular_speed_rps']:
                raise ValueError('Requested autonomous speed exceeds the existing hardware profile; no files changed')
            cfg['mapping']['enabled']=True
        configuration(cfg['mapping'])
        if apply and cfg!=before:cfg=write_configuration(path,cfg,hardware)
        print('APPLIED' if apply else 'PLAN_ONLY: add --apply after checking the measured geometry and turning radius')
        print('AUTONOMY:', 'ENABLED' if enabled else 'DISABLED')
        print(json.dumps(auto,indent=2))
        print('Manual input:',cfg['mapping']['control_source'],'; commissioning flags and US100 policy unchanged')
        return cfg


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    select=p.add_mutually_exclusive_group(required=True)
    select.add_argument('--enable',action='store_true');select.add_argument('--disable',action='store_true')
    for side in ('front','rear','left','right'):p.add_argument('--'+side,type=float)
    p.add_argument('--speed',type=float,default=.10)
    p.add_argument('--angular',type=float,default=.30)
    p.add_argument('--duration',type=int,default=180)
    p.add_argument('--vehicle',choices=('rigid','articulated'),default='rigid')
    p.add_argument('--exploration',choices=('reactive','frontier'),default='reactive')
    p.add_argument('--manual-lidar-assist',action='store_true',help='Slow/stop gamepad and web input near obstacles; does not steer for the operator')
    p.add_argument('--turn-radius',type=float,help='Measured minimum forward turning radius, metres; required for articulated trailer')
    p.add_argument('--apply',action='store_true',help='Write validated config with backup; default prints plan only')
    a=p.parse_args()
    if a.enable and a.vehicle=='articulated' and a.turn_radius is None:p.error('Measure --turn-radius with the real trailer first')
    footprint={side:getattr(a,side) for side in ('front','rear','left','right')}
    if a.enable and any(v is None for v in footprint.values()):p.error('--enable needs all four measured footprint extents')
    configure(a.config,a.enable,footprint,a.speed,a.angular,a.duration,a.vehicle,a.turn_radius or .60,a.apply,a.exploration,a.manual_lidar_assist)


if __name__=='__main__':main()
