#!/usr/bin/env python3
"""Inspect current config, reconcile one old Job, or obtain the existing HTTPS upload token."""
import argparse
import getpass
import json
import sqlite3
import sys
from contextlib import closing
from pathlib import Path

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['inspect', 'reconcile', 'token'])
    p.add_argument('--config', type=Path, default=Path('/home/ubuntu/mediassist-mqtt-2026-10-08/pi/config/mqtt.worker.json'))
    p.add_argument('--job-id', default='f2145e0f-f480-4ead-a46b-04746fa07558')
    p.add_argument('--reason')
    p.add_argument('--apply', action='store_true')
    p.add_argument('--base-url')
    p.add_argument('--ca-file', type=Path)
    p.add_argument('--robot-code', default='mar-v1')
    p.add_argument('--token-file', type=Path, default=Path('/home/ubuntu/mar-secrets/be-device-token.txt'))
    p.add_argument('--remember-device-secret',action='store_true',help='Save secret in a private Pi file and enable automatic map-upload token refresh; stop stack first')
    p.add_argument('--device-secret-file',type=Path,default=Path('/home/ubuntu/mar-secrets/be-device-secret.txt'))
    a = p.parse_args()
    sys.path.insert(0, str(a.config.resolve().parent.parent))
    cfg = json.loads(a.config.read_text(encoding='utf-8-sig'))
    hw_path = Path(cfg['hardware_profile'])
    hw = json.loads(hw_path.read_text(encoding='utf-8-sig'))
    legacy = Path(cfg['legacy_state_path'])
    if not legacy.is_absolute():
        raise SystemExit('legacy_state_path must be absolute; do not guess a database.')
    if a.action in ('inspect', 'reconcile'):
        print(json.dumps({'robot_id':cfg['robot_id'], 'hardware_profile':str(hw_path),
            'state_path':cfg['state_path'], 'legacy_state_path':str(legacy),
            'broker_host':cfg['mqtt']['host'], 'broker_port':cfg['mqtt']['port'],
            'broker_ca':cfg.get('tls_ca_file'), 'password_file_exists':Path(cfg['mqtt']['password_file']).is_file(),
            'enabled_step_types':cfg['enabled_step_types'], 'motion':hw['motion'], 'ros':hw['ros'],
            'mapping':cfg.get('mapping')}, indent=2))
        if legacy.exists():
            with closing(sqlite3.connect(legacy.as_uri()+'?mode=ro', uri=True)) as db:
                print('Legacy jobs:', db.execute('SELECT id,status FROM jobs').fetchall())
        else:
            print('Legacy DB does not exist; no DB was created.')
    if a.action == 'reconcile':
        if not a.apply:
            print('Read-only. Add --apply --reason after confirming the robot is stopped and this Job is absent/closed on BE.')
            return
        if not a.reason or not a.reason.strip():
            p.error('--reason is required')
        from mediassist.cli import reconcile_legacy
        print(json.dumps(reconcile_legacy(hw_path, a.job_id, a.reason, legacy), indent=2))
    if a.action == 'token':
        if not a.base_url or not a.base_url.startswith('https://'):
            p.error('--base-url must be the HTTPS URL reachable FROM PI')
        from contextlib import nullcontext
        from mediassist.be_token import request_device_token,write_private
        from configure_mapping import stopped_configuration,write_configuration
        with stopped_configuration(a.config) if a.remember_device_secret else nullcontext():
            secret=getpass.getpass('BE device secret (not MQTT password): ')
            result=request_device_token(a.base_url,a.ca_file,a.robot_code,secret,cfg['robot_id'])
            write_private(a.token_file,result['accessToken'])
            if a.remember_device_secret:
                write_private(a.device_secret_file,secret)
                from mediassist.worker_config import load,mapping_upload_base
                full,hardware=load(a.config)
                full['mapping'].update(device_secret_file=str(a.device_secret_file.resolve()),robot_code=a.robot_code,
                    upload_token_file=str(a.token_file.resolve()),upload_base_url=mapping_upload_base(a.base_url),
                    upload_ca_file=str(a.ca_file.resolve()) if a.ca_file else None)
                write_configuration(a.config,full,hardware)
                print('Map upload automatic token refresh enabled; secret saved privately, not printed.')
        print('Token saved:', a.token_file, '; expires:', result['expiresAt'])
        print('No credential was printed. Obtain a fresh token before a later SAVE if expired.')

if __name__ == '__main__':
    main()
