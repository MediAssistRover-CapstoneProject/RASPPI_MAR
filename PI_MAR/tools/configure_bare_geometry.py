"""Bare MentorPi Mecanum geometry from the 212 x 171 mm drawing and vendor URDF.

The URDF origin is about 3 mm behind the body centre. Outward-rounded mesh
bounds: front .109, rear .104, left/right .086 metres. No trailer is included.
"""
import argparse
import copy
from contextlib import nullcontext
from pathlib import Path
from configure_mapping import ROOT, load, stopped_configuration, write_configuration
from mediassist.autonomous_mapping import configuration
from mediassist.navigation_setup import footprint_rectangle


def tune(cfg, hardware):
    if hardware['motion'].get('chassis_model') != 'MENTORPI_M1_MECANUM':
        raise ValueError('BARE_MENTORPI_MECANUM_REQUIRED')
    result=copy.deepcopy(cfg)
    auto=configuration(result['mapping'])
    if auto['vehicle'] != 'rigid':
        raise ValueError('REMOVE_TRAILER_AND_SELECT_RIGID_PROFILE_FIRST')
    extent=dict(front=.109,rear=.104,left=.086,right=.086)
    footprint_rectangle(**extent)
    auto['footprint']=extent
    configuration(result['mapping'])
    motion=copy.deepcopy(hardware['motion'])
    # Same body in manual safety, autonomous exploration and saved-map Nav2.
    if isinstance(motion.get('lidar_braking'),dict):
        motion['lidar_braking']['footprint']=copy.deepcopy(extent)
    return result,dict(motion=motion,max_speed_mps=hardware['max_speed_mps'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--trailer-removed',action='store_true',required=True)
    p.add_argument('--apply',action='store_true')
    a=p.parse_args();path=a.config.resolve(strict=True)
    with stopped_configuration(path) if a.apply else nullcontext():
        cfg,hardware=load(path);candidate,motion=tune(cfg,hardware)
        if a.apply:write_configuration(path,candidate,hardware,motion_options=motion)
    print('BARE_GEOMETRY_APPLIED' if a.apply else 'PLAN_ONLY')
    print('Footprint from base_footprint: front=.109 rear=.104 left=.086 right=.086 m')
    print('Vendor body 212 x 171 mm; URDF bounds rounded outward, origin offset retained.')
    print('Speed, braking margins, Nav2 padding, credentials, HTTPS/TLS, maps and journals preserved.')


if __name__=='__main__':main()
