"""Configure mapping and the explicit US100 side threshold; preserve calibration and secrets."""
import argparse
import copy
from contextlib import nullcontext
import json
import os
import shutil
import sys
from datetime import datetime,timezone
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from mediassist.worker_config import load,mapping_upload_base
from mediassist.configuration import validate_config


def write_configuration(config,cfg,hardware,stop_distance=None,motion_options=None):
    """Publish a validated profile/config pair; preserve motion flags, credentials and calibration."""
    config=Path(config)
    candidate=copy.deepcopy(cfg)
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
    temporary=config.with_suffix('.mapping.new')
    created=None
    applied=False
    try:
        if stop_distance is not None or motion_options:
            tuned=copy.deepcopy(hardware)
            if motion_options:
                tuned['motion'].update(motion_options['motion'])
                tuned['max_speed_mps']=motion_options['max_speed_mps']
                if 'navigation' in motion_options:tuned['navigation']=copy.deepcopy(motion_options['navigation'])
            if stop_distance is not None:
                tuned['motion']['stop_distance_m']=stop_distance
            validate_config(tuned)
            if tuned!=hardware:
                created=config.parent/('site.motion.'+stamp+'.json')
                with created.open('x',encoding='utf-8') as stream:
                    stream.write(json.dumps(tuned,ensure_ascii=False,indent=2)+'\n')
                    stream.flush();os.fsync(stream.fileno())
                created.chmod(0o600)
                candidate['hardware_profile']=str(created.resolve())
        temporary.write_text(json.dumps(candidate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        temporary.chmod(0o600)
        load(temporary)
        backup=config.with_suffix('.'+stamp+'.bak')
        shutil.copy2(config,backup)
        os.replace(temporary,config)
        applied=True
        config.chmod(0o600)
        return candidate
    finally:
        temporary.unlink(missing_ok=True)
        if created is not None and not applied:created.unlink(missing_ok=True)


def stopped_configuration(config):
    if sys.platform!='linux':return nullcontext()
    from robot_stack import processes,conflict,locked
    if any(conflict(p) for p in processes().values()):
        raise RuntimeError('STOP_ROBOT_STACK_FIRST: stop run/bench and the standalone worker before editing configuration')
    return locked(config.parent.parent/'state/stack',config)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--enable-mapping',action='store_true')
    p.add_argument('--control-source',choices=['mqtt','local'],default='mqtt')
    p.add_argument('--preview-rate-hz',type=float,default=1.)
    p.add_argument('--enable-camera',action='store_true')
    p.add_argument('--camera-topic',default='/ascamera/camera_publisher/rgb0/image')
    p.add_argument('--camera-rate-hz',type=float,default=.5)
    p.add_argument('--camera-stream-fps',type=float,help='LAN MJPEG FPS, 1..30; source camera must publish at least this fast')
    p.add_argument('--fresh-map',action='store_true',help='Restart only the owned SLAM engine for each mapping START; keep saved maps and hardware running')
    p.add_argument('--slam-resolution-m',type=float,help='SLAM grid cell size in metres, 0.01..0.10; default .025')
    p.add_argument('--m1-manual',action='store_true',help='Bare MentorPi M1: forward .15, reverse .10 m/s, yaw .5 rad/s, rear LiDAR guard; preserves commissioning flags')
    p.add_argument('--slam-launch-argv-json',type=Path)
    p.add_argument('--upload-base-url')
    p.add_argument('--upload-token-file',type=Path)
    p.add_argument('--upload-ca-file',type=Path)
    p.add_argument('--stop-distance-m',type=float,
                   help='US100 side threshold in metres, 0.025..2; keeps motion flags and speed unchanged')
    a=p.parse_args()
    a.config=a.config.resolve(strict=True)
    with stopped_configuration(a.config):
        configure(a)


def configure(a):
    cfg,hardware=load(a.config)
    m=cfg['mapping']
    if a.enable_mapping:m['enabled']=True
    if m['enabled'] and not all(hardware['motion'][k] for k in ('commissioned','cart_footprint_validated','watchdog_validated')):
        print('Motion remains inhibited; stationary scan/preview/save may run.')
    m['control_source']=a.control_source;m['max_preview_rate_hz']=a.preview_rate_hz
    if a.enable_camera:m['camera']['enabled']=True
    m['camera']['topic']=a.camera_topic;m['camera']['rate_hz']=a.camera_rate_hz
    stream_fps=getattr(a,'camera_stream_fps',None)
    if stream_fps is not None:m['camera']['stream_rate_hz']=stream_fps
    if getattr(a,'fresh_map',False):m['fresh_session']=True
    if getattr(a,'slam_resolution_m',None) is not None:m['slam_resolution_m']=a.slam_resolution_m
    if a.slam_launch_argv_json:m['launch_argv']=json.loads(a.slam_launch_argv_json.read_text())
    for name in ('upload_base_url','upload_token_file','upload_ca_file'):
        value=getattr(a,name)
        if name=='upload_base_url' and value is not None:value=mapping_upload_base(value)
        if value is not None:m[name]=str(value.resolve(strict=True)) if isinstance(value,Path) else value
    motion_options=None
    if getattr(a,'m1_manual',False):
        if os.environ.get('MACHINE_TYPE') not in (None,'','MentorPi_Mecanum'):
            raise RuntimeError('M1_PROFILE_REQUIRES_MECANUM: check MACHINE_TYPE before configuring this model')
        motion_options={'max_speed_mps':.15,'motion':{'chassis_model':'MENTORPI_M1_MECANUM',
            'allow_reverse':True,'max_reverse_speed_mps':.10,'rear_stop_distance_m':.30,'max_angular_speed_rps':.50}}
    cfg=write_configuration(a.config,cfg,hardware,a.stop_distance_m,motion_options)
    print('Updated:',a.config,'; mapping:',m['enabled'],'; control:',m['control_source'])
    print('Map maximum Hz:',m['max_preview_rate_hz'],'; camera enabled:',m['camera']['enabled'],'; camera Hz:',m['camera']['rate_hz'])
    print('Camera topic:',m['camera']['topic'])
    print('Camera LAN stream target FPS:',m['camera']['stream_rate_hz'])
    print('Fresh map per START:',m['fresh_session'],'; SLAM resolution (m/cell):',m['slam_resolution_m'])
    print('Map upload collection:',m.get('upload_base_url'))
    print('Active hardware profile:',cfg['hardware_profile'])
    print('US100 side stop distance (m):',hardware['motion']['stop_distance_m'] if a.stop_distance_m is None else a.stop_distance_m)
    print('Motion flags unchanged:',json.dumps({key:hardware['motion'][key] for key in
        ('commissioned','cart_footprint_validated','watchdog_validated')}))
    if motion_options:print('M1 manual: forward <=.15, reverse <=.10 m/s, yaw <=.50 rad/s; rear LiDAR guard enabled. Motion flags preserved.')
    else:print('Speed limits unchanged; this threshold does not change Nav2 footprint or goal tolerance.')
    if not m.get('upload_base_url'):print('No upload URL: preview/local save only; WF_MAPPING COMMIT cannot complete until HTTPS upload is configured.')
if __name__=='__main__':main()
