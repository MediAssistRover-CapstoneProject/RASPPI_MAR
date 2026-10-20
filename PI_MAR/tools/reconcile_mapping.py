"""Offline acknowledgement after physically stopping robot and old SLAM process."""
import argparse
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from mediassist.cli import instance_lock
from mediassist.worker_config import load

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--robot-and-slam-stopped',action='store_true',required=True)
    p.add_argument('--reason',required=True)
    a=p.parse_args()
    if not a.reason.strip(): p.error('A reason is required')
    cfg,_=load(a.config)
    with instance_lock(cfg['legacy_state_path']),instance_lock(cfg['state_path']):
        target=Path(cfg['mapping']['storage_dir'])/'session.json'
        if not target.exists(): raise SystemExit('No mapping journal')
        state=json.loads(target.read_text())
        saved = state.get('artifact_dir') and (Path(state['artifact_dir'])/'map.tar.gz').is_file()
        state.update(state='SAVED_LOCAL' if saved else 'OPERATOR_RECONCILED',
                     session_open=False,reconciliation_reason=a.reason)
        temp=target.with_suffix('.new'); temp.write_text(json.dumps(state),encoding='utf-8'); temp.replace(target)
        print('Mapping journal reconciled; reconcile interrupted JobStep and BE separately if required.')

if __name__=='__main__': main()
