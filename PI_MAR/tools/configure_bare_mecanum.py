"""Opt-in bare MentorPi Mecanum preset. No trailer; credentials and motion test flags preserved."""
import argparse
import copy
from pathlib import Path
from configure_mapping import ROOT,load,stopped_configuration,write_configuration
from mediassist.gamepad import configuration as gamepad_config
from mediassist.autonomous_mapping import configuration as auto_config


def configure(path,source='gamepad',linear=.25,angular=.8,auto_speed=.18,apply=False):
    if source not in ('gamepad','mqtt') or not .05<=linear<=.30 or not .2<=angular<=1.0 or not .02<=auto_speed<=min(.25,linear):
        raise ValueError('Use manual .05..30 m/s, yaw .2..1.0 rad/s; autonomous speed <= manual speed')
    from contextlib import nullcontext
    path=Path(path).resolve(strict=True)
    with stopped_configuration(path) if apply else nullcontext():
        cfg,hardware=load(path);cfg=copy.deepcopy(cfg)
        m=cfg['mapping'];m['enabled']=True;m['control_source']=source
        p=gamepad_config(m)
        p.update(layout='vendor_mecanum',linear_axis=1,lateral_axis=0,angular_axis=2,
                 session_required=False,max_run_sec=0.,forward_mps=linear,reverse_mps=linear,
                 lateral_mps=linear,angular_rps=angular,linear_accel=.8,angular_accel=2.5)
        gamepad_config(m)
        a=auto_config(m)
        a.update(enabled=True,vehicle='rigid',exploration_mode='frontier',frontier_controller='nav2',manual_lidar_assist=False,
                 speed_mps=auto_speed,angular_rps=min(.6,angular),max_duration_sec=300,
                 footprint={'front':.12,'rear':.12,'left':.10,'right':.10},
                 lidar_stop_distance_m=.10,turn_clearance_m=.35,slow_clearance_m=.65)
        auto_config(m)
        options={'max_speed_mps':linear,'motion':{
            'chassis_model':'MENTORPI_M1_MECANUM','allow_reverse':True,'allow_strafe':True,
            'max_reverse_speed_mps':linear,'rear_stop_distance_m':.30,
            'max_angular_speed_rps':angular,'us100_required':False}}
        if apply:cfg=write_configuration(path,cfg,hardware,motion_options=options)
        print('BARE_MECANUM_APPLIED' if apply else 'PLAN_ONLY: add --apply while the robot stack is stopped')
        print('Trailer removed. Body bounds: front/rear .12 m, left/right .10 m; all-around LiDAR stop .10 m.')
        print('Input:',source,'; left stick x/y translation, right stick yaw (axis 2); continuous fresh input.')
        print('Manual:',linear,'m/s;',angular,'rad/s; autonomous:',auto_speed,'m/s, frontier.')
        print('US100 diagnostic only. LiDAR/odom freshness, gate, watchdog and commissioning flags retained.')
        print('Camera backend, credentials, maps, endpoints and journals preserved.')
        return cfg


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--trailer-removed',action='store_true',required=True)
    p.add_argument('--source',choices=('gamepad','mqtt'),default='gamepad')
    p.add_argument('--linear',type=float,default=.25)
    p.add_argument('--angular',type=float,default=.8)
    p.add_argument('--auto-speed',type=float,default=.18)
    p.add_argument('--apply',action='store_true')
    a=p.parse_args()
    configure(a.config,a.source,a.linear,a.angular,a.auto_speed,a.apply)


if __name__=='__main__':main()
