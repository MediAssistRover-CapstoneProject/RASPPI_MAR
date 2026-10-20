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


def write_configuration(config,cfg,hardware,stop_distance=None):
    """Publish a validated profile/config pair; preserve motion flags, credentials and calibration."""
    config=Path(config)
    candidate=copy.deepcopy(cfg)
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
    temporary=config.with_suffix('.mapping.new')
    created=None
    applied=False
    try:
        if stop_distance is not None:
            tuned=copy.deepcopy(hardware)
            tuned['motion']['stop_distance_m']=stop_distance
            validate_config(tuned)
            if tuned['motion']['stop_distance_m']!=hardware['motion']['stop_distance_m']:
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
    if a.slam_launch_argv_json:m['launch_argv']=json.loads(a.slam_launch_argv_json.read_text())
    for name in ('upload_base_url','upload_token_file','upload_ca_file'):
        value=getattr(a,name)
        if name=='upload_base_url' and value is not None:value=mapping_upload_base(value)
        if value is not None:m[name]=str(value.resolve(strict=True)) if isinstance(value,Path) else value
    cfg=write_configuration(a.config,cfg,hardware,a.stop_distance_m)
    print('Updated:',a.config,'; mapping:',m['enabled'],'; control:',m['control_source'])
    print('Map maximum Hz:',m['max_preview_rate_hz'],'; camera enabled:',m['camera']['enabled'],'; camera Hz:',m['camera']['rate_hz'])
    print('Camera topic:',m['camera']['topic'])
    print('Map upload collection:',m.get('upload_base_url'))
    print('Active hardware profile:',cfg['hardware_profile'])
    print('US100 side stop distance (m):',hardware['motion']['stop_distance_m'] if a.stop_distance_m is None else a.stop_distance_m)
    print('Motion flags unchanged:',json.dumps({key:hardware['motion'][key] for key in
        ('commissioned','cart_footprint_validated','watchdog_validated')}))
    print('Speed limits unchanged; this threshold does not change Nav2 footprint or goal tolerance.')
    if not m.get('upload_base_url'):print('No upload URL: preview/local save only; WF_MAPPING COMMIT cannot complete until HTTPS upload is configured.')
if __name__=='__main__':main()
