"""Offline acknowledgement after physically stopping robot and old SLAM process."""
import argparse
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from mediassist.cli import instance_lock
from mediassist.worker_config import load
from mediassist.step_store import StepStore

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--robot-and-slam-stopped',action='store_true',required=True)
    p.add_argument('--reason',required=True)
    p.add_argument('--job-step-id',help='Interrupted/failed MAPPING operation to reconcile with BE')
    a=p.parse_args()
    if not a.reason.strip(): p.error('A reason is required')
    cfg,_=load(a.config)
    with instance_lock(cfg['legacy_state_path']),instance_lock(cfg['state_path']):
        target=Path(cfg['mapping']['storage_dir'])/'session.json'
        if not target.exists(): raise SystemExit('No mapping journal')
        state=json.loads(target.read_text())
        if a.job_step_id:
            store=StepStore(cfg['state_path'],cfg['robot_id'])
            try:
                row=store.db.execute('SELECT body FROM worker_steps WHERE step_id=?',(a.job_step_id,)).fetchone()
                step=json.loads(row[0]) if row else {}
                if (step.get('jobId')!=state.get('job_id') or step.get('stepType')!='EXECUTE_SENSING'
                        or step.get('params',{}).get('sensing_type')!='MAPPING'):
                    raise SystemExit('Operation does not match this mapping journal; nothing reconciled')
                store.reconcile(a.job_step_id,a.reason,allow_terminal_failed_mapping=True)
            finally:
                store.close()
        saved = state.get('artifact_dir') and (Path(state['artifact_dir'])/'map.tar.gz').is_file()
        state.update(state='SAVED_LOCAL' if saved else 'OPERATOR_RECONCILED',
                     session_open=False,reconciliation_reason=a.reason)
        temp=target.with_suffix('.new'); temp.write_text(json.dumps(state),encoding='utf-8'); temp.replace(target)
        print('Mapping journal closed; local artifacts preserved. Restart worker to deliver operator receipt.')
        print('BE still requires staff recovery review; no Pi physicalStopConfirmed flag was invented.')

if __name__=='__main__': main()
