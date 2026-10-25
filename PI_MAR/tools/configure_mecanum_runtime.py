"""Tune an already commissioned bare Mecanum; never replace credentials, maps or journals."""
import argparse,copy,json
from pathlib import Path
from contextlib import nullcontext
from configure_mapping import ROOT,load,stopped_configuration,write_configuration
from mediassist.worker_config import camera_config
from mediassist.autonomous_mapping import configuration

def tune(cfg,hardware,speed=.18,angular=.6):
    m=hardware['motion']
    if m.get('chassis_model')!='MENTORPI_M1_MECANUM' or not m.get('allow_strafe'):
        raise RuntimeError('CONFIGURE_MECANUM_HARDWARE_FIRST')
    if not all(m.get(k) for k in ('commissioned','cart_footprint_validated','watchdog_validated')):
        raise RuntimeError('EXISTING_COMMISSIONING_REQUIRED')
    if not .02<=speed<=min(.25,hardware['max_speed_mps']) or not .1<=angular<=min(.6,m['max_angular_speed_rps']):
        raise ValueError('Requested speed exceeds validated hardware or autonomous limits')
    result=copy.deepcopy(cfg);mapping=result['mapping'];auto=configuration(mapping)
    if auto['vehicle']!='rigid':raise RuntimeError('TRAILER_PROFILE_NOT_SUPPORTED_BY_THIS_PRESET')
    extent=auto.get('footprint',{})
    if not extent or any(extent.get(k,10)>v for k,v in dict(front=.15,rear=.15,left=.13,right=.13).items()):
        raise RuntimeError('EXISTING_BARE_BODY_FOOTPRINT_REQUIRED: dimensions are never shrunk automatically')
    auto.update(enabled=True,frontier_controller='nav2',exploration_mode='frontier',speed_mps=speed,
                angular_rps=angular,slow_clearance_m=max(.45,auto['turn_clearance_m']+.1),frontier_no_progress_sec=20.)
    configuration(mapping)
    camera=camera_config(mapping)
    camera.update(stream_backend='web_video_server',stream_port=8080,stream_bind='0.0.0.0',stream_enabled=True,
                  stream_max_width=640,stream_max_height=480,stream_jpeg_quality=60,mqtt_preview_enabled=False)
    camera_config(mapping)
    mapping['scan_normalization']={'enabled':True,'bins':360}
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--trailer-removed',action='store_true',required=True)
    p.add_argument('--speed',type=float,default=.18);p.add_argument('--angular',type=float,default=.6)
    p.add_argument('--apply',action='store_true');a=p.parse_args();path=a.config.resolve(strict=True)
    with stopped_configuration(path) if a.apply else nullcontext():
        cfg,hardware=load(path);updated=tune(cfg,hardware,a.speed,a.angular)
        if a.apply:
            from mediassist.vendor_camera import vendor_executable,check_port_available
            vendor_executable();check_port_available()
            write_configuration(path,updated,hardware)
    print('MECANUM_RUNTIME_APPLIED' if a.apply else 'PLAN_ONLY; add --apply while stopped')
    print('Autonomous translation vector <=',a.speed,'m/s; yaw <=',a.angular,'rad/s; Y enabled in Nav2.')
    print('Camera: vendor 8080, 640x480 quality60; Python MQTT RGB subscriber disabled.')
    print('SLAM: fixed 360 angular bins; raw LiDAR safety/AMCL input unchanged; physical scan frequency unchanged.')
    print('Hardware flags, geometry, manual control, TLS, IPs, secrets and saved maps preserved.')

if __name__=='__main__':main()
