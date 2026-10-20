"""Write a Pi MQTT profile; preserve existing hardware calibration."""
import argparse
import json
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from uuid import UUID
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from mediassist.worker_config import load


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hardware-profile',type=Path,required=True)
    p.add_argument('--robot-id',required=True)
    p.add_argument('--broker-host',required=True)
    p.add_argument('--broker-port',type=int,default=8883)
    p.add_argument('--password-file',type=Path,required=True)
    p.add_argument('--ca-file',type=Path)
    p.add_argument('--state-path',type=Path,required=True,help='Reuse existing WSS worker journal if deployed')
    p.add_argument('--output',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--compartment-id',default='CART_MAIN')
    for opt in ('move','servo','commissioning-unlock','ai-provider','mapping'):
        p.add_argument('--enable-'+opt,action='store_true')
    p.add_argument('--slam-launch-argv-json',type=Path,help='Local JSON array; omitted means SLAM started in another terminal')
    p.add_argument('--mapping-control-source',choices=['local','mqtt'],default='mqtt')
    p.add_argument('--enable-camera',action='store_true')
    p.add_argument('--camera-topic',default='/ascamera/camera_publisher/rgb0/image')
    p.add_argument('--preview-rate-hz',type=float,default=1.0)
    p.add_argument('--upload-base-url')
    p.add_argument('--upload-token-file',type=Path)
    p.add_argument('--upload-ca-file',type=Path)
    args=p.parse_args()
    UUID(args.robot_id)
    hwpath=args.hardware_profile.resolve(strict=True)
    hardware=json.loads(hwpath.read_text(encoding='utf-8'))
    legacy=Path(hardware['state_path'])
    if not legacy.is_absolute(): legacy=hwpath.parent.parent/legacy
    enabled=['EXECUTE_SENSING','WAIT']
    if args.enable_move:
        if not all(hardware['motion'][k] for k in ('commissioned','cart_footprint_validated','watchdog_validated')):
            p.error('Motion commissioning flags are not complete; do not change flags without checking the robot.')
    if args.enable_move:
        UUID(hardware['map_id'])
        for eid in hardware['endpoints']: UUID(eid)
        if not hardware.get('map_asset_sha256') or 'nav2' not in hardware['capabilities']:
            p.error('MOVE requires a commissioned map hash and nav2 capability; endpoints synchronize from BE.')
        enabled.append('MOVE')
    if args.enable_servo:
        if not hardware['servo']['commissioned']: p.error('Servo is not marked commissioned.')
        enabled.append('CONTROL_HARDWARE')
    if args.enable_commissioning_unlock and not args.enable_servo:
        p.error('Unlock requires --enable-servo')
    if args.upload_base_url and not args.upload_token_file:
        p.error('Upload URL requires --upload-token-file')
    if args.state_path.resolve()==legacy.resolve():
        p.error('Worker SQLite and legacy SQLite must be different files.')
    cfg={'transport':'mqtt5','robot_id':args.robot_id,
         'hardware_profile':str(hwpath),'legacy_state_path':str(legacy.resolve()),
         'state_path':str(args.state_path.resolve()),
         'tls_ca_file':str(args.ca_file.resolve(strict=True)) if args.ca_file else None,
         'mqtt':{'host':args.broker_host,'port':args.broker_port,'username':args.robot_id,
                 'password_file':str(args.password_file.resolve(strict=True)),'be_timeout_sec':15,'max_publish_per_sec':8},
         'heartbeat_sec':3,'ack_timeout_sec':15,'connect_timeout_sec':10,
         'enabled_step_types':enabled,'compartments':{args.compartment_id:'cargo_lock'},
         'unlock_policy':'COMMISSIONING_ONLY' if args.enable_commissioning_unlock else 'DISABLED',
         'ai':{'enabled':args.enable_ai_provider,'topic':'/mediassist/ai/observations','max_age_sec':2},
         'mapping':{'enabled':args.enable_mapping,'map_topic':'/map','control_source':args.mapping_control_source,
                    'camera':{'enabled':args.enable_camera,'topic':args.camera_topic,'rate_hz':.5,
                              'max_width':320,'max_height':240,'jpeg_quality':50,'max_jpeg_bytes':65536},
                    'storage_dir':str((ROOT/'pi/state/maps').resolve()),'max_cells':4000000,
                    'max_map_age_sec':10,'start_timeout_sec':45,'max_preview_rate_hz':args.preview_rate_hz,'max_preview_bytes':131072,
                    'launch_argv':json.loads(args.slam_launch_argv_json.read_text()) if args.slam_launch_argv_json else [],
                    'upload_base_url':args.upload_base_url,
                    'upload_token_file':str(args.upload_token_file.resolve(strict=True)) if args.upload_token_file else None,
                    'upload_ca_file':str(args.upload_ca_file.resolve(strict=True)) if args.upload_ca_file else None}}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    temporary=args.output.with_suffix('.new')
    temporary.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    load(temporary)
    if args.output.exists():
        shutil.copy2(args.output,args.output.with_suffix('.'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')+'.bak'))
    os.replace(temporary,args.output)
    args.output.chmod(0o600)
    print('Written',args.output)
    print('Enabled',enabled,'mapping=',args.enable_mapping)
    print('Entry: cd',ROOT/'pi','; .venv/bin/python -m mediassist.worker_cli run --config',args.output)

if __name__=='__main__': main()
