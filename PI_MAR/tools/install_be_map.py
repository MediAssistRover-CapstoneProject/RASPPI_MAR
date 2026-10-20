"""Download an assigned map/profile from BE while the worker is stopped. Never starts motion."""
import argparse
import copy
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import shutil
import ssl
import sys
import urllib.request
from urllib.parse import urlparse
from uuid import UUID
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'pi'))
from mediassist.worker_config import load,NoRedirect
from mediassist.configuration import validate_config
from mediassist.navigation_profile import validate_profile
from mediassist.map_sync import verify_bundle

def save(path,data):
    temp=path.with_suffix('.new');temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    os.replace(temp,path);path.chmod(0o600)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--base-url',required=True);p.add_argument('--map-id',required=True)
    p.add_argument('--token-file',type=Path,required=True);p.add_argument('--ca-file',type=Path)
    a=p.parse_args();UUID(a.map_id)
    uri=urlparse(a.base_url)
    if uri.scheme!='https' or not uri.hostname or uri.username or uri.password or uri.path not in ('','/') or uri.query or uri.fragment:p.error('Use an HTTPS BE origin')
    if Path('/proc').is_dir():
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit() or proc.name==str(os.getpid()):continue
            try:cmd=(proc/'cmdline').read_bytes().split(b'\0')
            except OSError:continue
            if b'mediassist.worker_cli' in cmd:p.error('Stop robot.zsh run / MQTT worker before installing a map')
    cfg,hardware=load(a.config)
    token=a.token_file.read_text(encoding='utf-8-sig').strip()
    ctx=ssl.create_default_context(cafile=str(a.ca_file) if a.ca_file else None)
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect(),urllib.request.HTTPSHandler(context=ctx))
    def get(path,limit):
        req=urllib.request.Request(a.base_url.rstrip('/')+path,headers={'Authorization':'Bearer '+token})
        with opener.open(req,timeout=15) as r:body=r.read(limit+1)
        if len(body)>limit:raise ValueError('BE response exceeds limit')
        return body
    files,meta=verify_bundle(get('/api/v1/pi/maps/'+a.map_id+'/bundle',6*1024*1024),a.map_id)
    manifest=json.loads(get('/api/v1/pi/catalog/navigation-profile',2*1024*1024))
    hw=copy.deepcopy(hardware)
    hw.update(map_id=a.map_id,map_asset_sha256=meta['asset_hash'],endpoints={},endpoint_groups={})
    if hw['ros']['map_frame']!=meta['frame_id']:raise ValueError('BE map frame differs from hardware map_frame')
    message={**manifest,'messageType':'NAV_PROFILE','robotId':cfg['robot_id']}
    profile=validate_profile(message,cfg['robot_id'],hw)
    # Declare the installed navigation software without changing motion commissioning.
    hw['endpoints']=profile['endpoints']
    hw['capabilities']=sorted(set(hw['capabilities'])|{'nav2'})
    validate_config(hw)
    root=ROOT/'pi/state/maps/installed'/a.map_id
    root.mkdir(parents=True,exist_ok=True)
    for name,raw in files.items():
        dest=root/name
        if dest.exists() and dest.read_bytes()!=raw:raise ValueError('Existing map differs; import a new map version instead')
        dest.write_bytes(raw)
    hardware_file=root/'hardware.json';save(hardware_file,hw)
    cfg['hardware_profile']=str(hardware_file)
    cfg['navigation_runtime']={'map_yaml':str(root/'map.yaml')}
    cfg['mapping']['enabled']=False
    cfg['enabled_step_types']=list(dict.fromkeys([*cfg['enabled_step_types'],'MOVE']))
    shutil.copy2(a.config,a.config.with_suffix('.'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')+'.bak'))
    save(a.config,cfg)
    print('Installed:',root/'map.yaml')
    print('Map SHA256:',meta['asset_hash'],'; endpoints:',len(hw['endpoints']))
    print('No movement, no ROS map load and no profile ACK sent. Start navigation mode and set initial pose next.')

if __name__=='__main__':main()
