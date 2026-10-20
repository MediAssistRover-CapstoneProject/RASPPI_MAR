"""Inspect Linux audio or explicitly play one local sentence; never sends a motor command."""
import argparse
import asyncio
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from mediassist.speech import SpeechPlayer,audio_device
from mediassist.contracts import StepError


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--speak',action='store_true',help='Play a local fixed sentence after inspection')
    a=p.parse_args();cfg=json.loads(a.config.read_text(encoding='utf-8-sig'))
    speech=cfg.get('speech',{})
    print('Configured audio:',json.dumps(speech))
    print('espeak-ng:',shutil.which('espeak-ng'));print('aplay:',shutil.which('aplay'))
    cards=Path('/proc/asound/cards')
    print(cards.read_text() if cards.exists() else 'NO /proc/asound/cards')
    for path in sorted(Path('/dev/snd').glob('*')):
        if path.is_file() or not path.is_dir():print('Device:',path,'readable:',os.access(path,os.R_OK),'writable:',os.access(path,os.W_OK))
    if shutil.which('aplay'):subprocess.run(['aplay','-l'],check=False,timeout=5)
    try:print('Selected device:',audio_device(speech))
    except StepError as e:
        print('AUDIO_NOT_READY:',e.code);raise SystemExit(2)
    if a.speak:
        result=asyncio.run(SpeechPlayer(speech).speak({'prompt_key':'LOCAL_AUDIO_CHECK','action':'SPEAK','data':{'text':'Xin chào. Kiểm tra loa của robot.','language_code':'vi'}}))
        print(json.dumps(result,ensure_ascii=False,indent=2))
    else:print('Inspection only; use --speak to play a local sentence.')


if __name__=='__main__':main()
