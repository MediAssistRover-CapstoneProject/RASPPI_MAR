"""Preserve MQTT credentials/journals/calibration; configure speech and stationary mapping."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from mediassist.worker_config import load


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--mode',choices=('speech','mapping'),required=True)
    p.add_argument('--audio-device',default='auto',help='auto selects exactly one USB-Audio card; or plughw:CARD=<id>,DEV=0')
    p.add_argument('--camera',action='store_true')
    p.add_argument('--camera-topic',default='/ascamera/camera_publisher/rgb0/image')
    args=p.parse_args()
    cfg,hardware=load(args.config)
    cfg['speech']={**cfg.get('speech',{}),'enabled':True,'device':args.audio_device}
    cfg['enabled_step_types']=list(dict.fromkeys([*cfg['enabled_step_types'],'INTERACT','WAIT','EXECUTE_SENSING']))
    cfg['mapping']['enabled']=args.mode=='mapping'
    cfg['mapping']['control_source']='mqtt'
    # The supervisor owns the vendor SLAM launch, not the step worker.
    cfg['mapping']['launch_argv']=[]
    cfg['mapping']['max_preview_rate_hz']=1
    cfg['mapping']['camera']['enabled']=args.camera
    cfg['mapping']['camera']['topic']=args.camera_topic
    temporary=args.config.with_suffix('.demo.new')
    try:
        temporary.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        load(temporary)
        backup=args.config.with_suffix('.'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')+'.bak')
        shutil.copy2(args.config,backup)
        os.replace(temporary,args.config)
        args.config.chmod(0o600)
    finally:
        temporary.unlink(missing_ok=True)
    print('Config:',args.config)
    print('Speech enabled; ALSA device:',args.audio_device)
    print('Mapping:',cfg['mapping']['enabled'],'; camera:',args.camera)
    print('Motion flags preserved:',json.dumps(hardware['motion'],ensure_ascii=False))
    if not cfg['mapping'].get('upload_base_url'):
        print('Map upload is not configured: local save works; BE map COMMIT requires an HTTPS upload endpoint.')

if __name__=='__main__':main()
