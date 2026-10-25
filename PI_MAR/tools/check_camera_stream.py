"""Measure the actual local MJPEG pipeline. Read-only; starts a temporary video viewer, no robot commands."""
import argparse
import json
import threading
import time
import urllib.request
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from mediassist.worker_config import camera_config
from mediassist.vendor_camera import stream_path


def jpeg_starts(tail,data):
    joined=tail+data
    return joined.count(b'\xff\xd8'),joined[-1:]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[1]/'pi/config/mqtt.worker.json')
    p.add_argument('--seconds',type=int,default=10,choices=range(5,31))
    a=p.parse_args()
    cfg=camera_config(json.loads(a.config.read_text(encoding='utf-8-sig'))['mapping'])
    base='http://127.0.0.1:'+str(cfg.get('stream_port',8081))
    http=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    end=time.monotonic()+a.seconds
    count=0;errors=[];response=None
    def read():
        nonlocal count,response
        try:
            with http.open(base+stream_path(cfg),timeout=3) as r:
                response=r;tail=b''
                while time.monotonic()<end:
                    data=r.read1(65536)
                    if not data:break
                    received,tail=jpeg_starts(tail,data)
                    count+=received
        except Exception as e:errors.append(type(e).__name__+': '+str(e))
    thread=threading.Thread(target=read,daemon=True);thread.start()
    started=time.monotonic()
    try:
        while time.monotonic()<end:
            time.sleep(1.)
            health={'backend':cfg['stream_backend']}
            if cfg['stream_backend']=='python':
                with http.open(base+'/health',timeout=3) as r:health.update(json.load(r))
            health['localHttpReceivedFps']=round(count/max(.1,time.monotonic()-started),1)
            print(json.dumps(health,ensure_ascii=False),flush=True)
    finally:
        if response is not None:response.close()
        thread.join(1.)
    if errors:print('CAMERA_HTTP_ERRORS:', '; '.join(errors))
    if not count:raise SystemExit('CAMERA_NO_FRAMES: inspect pi/state/stack/camera.log, ROS topic, driver and source timestamps')
    print('These are local Pi rates. Wi-Fi and browser display latency must also be checked on the laptop.')


if __name__=='__main__':main()
