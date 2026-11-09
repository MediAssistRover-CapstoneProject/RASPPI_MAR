"""Enable the optional supervised WebRTC client while stopped; no audio/video recording."""
import argparse
import json
import os
from pathlib import Path
import stat
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools')]
from mediassist.worker_config import load
from mediassist.media_runtime import configuration,use_media_packages
from configure_mapping import stopped_configuration
from configure_camera import atomic_write


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--url',required=True,help='Public WSS origin of the media EC2')
    mode=p.add_mutually_exclusive_group(required=True);mode.add_argument('--enable',action='store_true');mode.add_argument('--disable',action='store_true')
    p.add_argument('--audio-input',default='auto');p.add_argument('--audio-output',default='auto')
    p.add_argument('--force-relay',action='store_true',help='Use TURN only; normal mode already falls back to TURN automatically')
    p.add_argument('--apply',action='store_true');a=p.parse_args()
    cfg,_=load(a.config);media=configuration(cfg)
    media.update(enabled=a.enable,url=a.url,audio_input=int(a.audio_input) if a.audio_input.isdigit() else a.audio_input,
                 audio_output=int(a.audio_output) if a.audio_output.isdigit() else a.audio_output,force_relay=a.force_relay)
    configuration(cfg)
    if a.enable:
        use_media_packages()
        from livekit import rtc
        from livekit.rtc.media_devices import MediaDevices
        import sounddevice
        import cv2
        import numpy
        if not numpy.__version__.startswith('1.'):raise RuntimeError('ROS_NUMPY_1_REQUIRED: use pi/requirements-media.txt')
    if a.apply:
        if sys.platform!='linux':raise RuntimeError('Apply inside MentorPi')
        with stopped_configuration(a.config):
            raw=a.config.read_bytes();candidate=json.loads(raw)
            candidate['media']=media
            atomic_write(a.config,(json.dumps(candidate,ensure_ascii=False,indent=2)+'\n').encode(),stat.S_IMODE(a.config.stat().st_mode))
            try:load(a.config)
            except BaseException:
                atomic_write(a.config,raw,stat.S_IMODE(a.config.stat().st_mode));raise
        print('MEDIA_CONFIG_APPLIED; restart the existing stack once. No backup/test files created.')
    else:print('MEDIA_CONFIG_CHECK_OK; no files changed. Add --apply after stopping the stack.')
    print('URL',media['url'],'enabled',media['enabled'],'relay_only',media['force_relay'])


if __name__=='__main__':main()
