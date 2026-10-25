"""Run a temporary saved-map session; never edits the base worker/hardware configuration."""
import argparse,copy,json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'pi'))
import yaml
from mediassist.navigation_session import SessionClient,summary
from mediassist.map_sync import verify_bundle
from mediassist.navigation_profile import validate_profile
from mediassist.navigation_setup import nav2_params,footprint_rectangle
from mediassist.be_token import write_private
from mediassist.worker_config import load
from configure_mapping import stopped_configuration
from configure_localization import nav_parameters

def prepare(config,session_id=None):
    cfg,hw=load(config);client=SessionClient(cfg);descriptor=client.get(session_id)
    session_id=descriptor['sessionId'];map_id=descriptor['mapId']
    import uuid
    root=ROOT/'pi/state/navigation-sessions'/session_id/uuid.uuid4().hex;root.mkdir(parents=True,mode=0o700,exist_ok=False)
    context=dict(session_id=session_id,map_id=map_id,map_asset_hash=descriptor['mapAssetHash'],expires_at=descriptor['expiresAt'])
    files,meta=verify_bundle(client.request('/api/v1/pi/maps/'+map_id+'/bundle',limit=6*1024*1024),map_id)
    if meta['asset_hash']!=context['map_asset_hash'] or meta['frame_id']!=hw['ros']['map_frame']:raise RuntimeError('SESSION_MAP_HASH_OR_FRAME_MISMATCH')
    for name,raw in files.items():(root/name).write_bytes(raw)
    hardware=copy.deepcopy(hw);hardware.update(map_id=map_id,map_asset_sha256=meta['asset_hash'],endpoints={},endpoint_groups={})
    hardware['navigation_session_id']=session_id;hardware['capabilities']=sorted(set(hardware['capabilities'])|{'nav2'})
    manifest=json.loads(client.request('/api/v1/pi/catalog/navigation-profile',limit=2*1024*1024))
    profile=validate_profile({**manifest,'messageType':'NAV_PROFILE','robotId':cfg['robot_id']},cfg['robot_id'],hardware,allow_empty=True)
    hardware['endpoints']=profile['endpoints']
    # Preserve real commissioning evidence and hardware speed limits.
    extent=cfg['mapping']['autonomy'].get('footprint')
    if not extent:raise RuntimeError('CONFIGURE_MEASURED_FOOTPRINT_FIRST')
    polygon=footprint_rectangle(**extent)
    base=yaml.safe_load(Path('/home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml').read_text())
    controller=yaml.safe_load(Path('/home/ubuntu/ros2_ws/src/navigation/config/nav2_controller_dwb.yaml').read_text())
    params=nav_parameters(nav2_params(base,controller,polygon,
        speed=min(.25,cfg['mapping']['autonomy']['speed_mps'],hw['max_speed_mps']),
        angular=min(.6,cfg['mapping']['autonomy']['angular_rps'],hw['motion']['max_angular_speed_rps']),
        mecanum=hw['motion'].get('chassis_model')=='MENTORPI_M1_MECANUM' and hw['motion'].get('allow_strafe') is True),hardware)
    (root/'nav2.yaml').write_text(yaml.safe_dump(params,sort_keys=False),encoding='utf-8')
    write_private(root/'hardware.json',json.dumps(hardware,indent=2))
    runtime=copy.deepcopy(cfg);runtime['hardware_profile']=str(root/'hardware.json');runtime['navigation_session']=context
    runtime['stack_state_dir']=str(ROOT/'pi/state/stack')
    runtime['navigation_runtime']=dict(map_yaml=str(root/'map.yaml'),nav2_params=str(root/'nav2.yaml'),localization_only=False)
    runtime['localization']=dict(enabled=True,mode='manual',map_id=map_id,map_asset_hash=meta['asset_hash'],status_file=str(root/'localization.json'))
    runtime['mapping']['enabled']=False
    runtime['enabled_step_types']=list(dict.fromkeys([*runtime['enabled_step_types'],'MOVE']))
    # Reuse the durable operation journal; old operations must still be reconciled.
    write_private(root/'worker.json',json.dumps(runtime,indent=2))
    load(root/'worker.json')
    return root,client,descriptor

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json');p.add_argument('--session-id')
    a=p.parse_args()
    with stopped_configuration(a.config):root,client,descriptor=prepare(a.config,a.session_id)
    print('NAVIGATION_SESSION:',descriptor['sessionId'],'MAP:',descriptor['mapId'])
    print('Base configuration preserved. Waiting for initial pose/global localization from the map UI; no goal sent.')
    process=None
    try:
        process=subprocess.Popen(['zsh',str(ROOT/'tools/robot.zsh'),'run','--config',str(root/'worker.json'),'--mode','navigation','--nav2-params',str(root/'nav2.yaml')],cwd=ROOT)
        try:code=process.wait()
        except KeyboardInterrupt:
            # The terminal also delivers SIGINT to the supervisor. Keep waiting
            # for its STOP sequence before closing the BE session.
            print('Waiting for robot supervisor STOP before closing the map session...')
            while process.poll() is None:
                try:process.wait(timeout=1)
                except (subprocess.TimeoutExpired,KeyboardInterrupt):continue
            code=process.returncode
    finally:
        # The supervisor completes STOP before returning. Unresolved Jobs prevent BE closure.
        try:client.request('/api/v1/pi/navigation-session/close',body={'sessionId':descriptor['sessionId']})
        except Exception:print('Session closure not confirmed; inspect BE Job recovery or session expiry. No base configuration changed.')
    raise SystemExit(code)
if __name__=='__main__':main()
