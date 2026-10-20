"""Select offline speech or BE-hosted OpenAI synthesis; preserve robot/MQTT/motion configuration."""
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
    p.add_argument('--engine',choices=('espeak-ng','be-tts'),required=True)
    p.add_argument('--base-url')
    p.add_argument('--token-file')
    p.add_argument('--ca-file')
    p.add_argument('--wait-timeout',type=int,default=45)
    a=p.parse_args()
    cfg,hw=load(a.config)
    speech=cfg.setdefault('speech',{})
    speech.update(enabled=True,engine=a.engine)
    speech.setdefault('device','auto')
    if a.engine=='be-tts':
        if not a.base_url or not a.token_file:p.error('be-tts requires --base-url and --token-file')
        speech.update(synthesize_url=a.base_url.rstrip('/')+'/api/v1/pi/speech/synthesize',
            token_file=a.token_file,ca_file=a.ca_file,wait_timeout_sec=a.wait_timeout)
    cfg['enabled_step_types']=list(dict.fromkeys([*cfg['enabled_step_types'],'INTERACT']))
    temp=a.config.with_suffix('.tts.new')
    try:
        temp.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        load(temp)
        shutil.copy2(a.config,a.config.with_suffix('.'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')+'.bak'))
        os.replace(temp,a.config);a.config.chmod(0o600)
    finally:temp.unlink(missing_ok=True)
    print('Speech engine:',a.engine)
    print('Config saved:',a.config)
    print('MQTT, GPIO, servo calibration and motion flags preserved. Restart the existing stack once.')

if __name__=='__main__':main()
