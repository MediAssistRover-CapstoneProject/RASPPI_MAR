"""Run a temporary saved-map session; never edits the base worker/hardware configuration."""
import argparse,copy,json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'pi'))
import yaml
from mediassist.navigation_session import SessionClient,summary
from mediassist.map_sync import verify_bundle
from mediassist.navigation_profile import validate_profile
from mediassist.navigation_setup import nav2_params,footprint_rectangle
from mediassist.navigation_geometry import session_geometry
from mediassist.be_token import write_private
from mediassist.worker_config import load
from configure_mapping import stopped_configuration
from configure_localization import nav_parameters

def prepare(config,session_id=None,arrival_mode='POSITION_THEN_SPIN',load_profile=None,voice_chat=None,patrol_fall=None,fall_monitor=None,travel_policy='FORWARD_PATH',restricted_monitor=None):
    if arrival_mode not in ('POSITION_ONLY','POSITION_THEN_SPIN'):raise ValueError('Unsupported arrival mode')
    if travel_policy not in ('FORWARD_PATH','HOLONOMIC'):raise ValueError('Unsupported travel policy')
    position_only=arrival_mode=='POSITION_ONLY'
    native=ROOT/'pi/native/navigation_bt/build'
    if not (native/'libmar_commit_navigation_path_bt_node.so').is_file():
        raise RuntimeError('BUILD_NAVIGATION_BT_FIRST: bash tools/build_navigation_bt.sh')
    prefix=native/'install'
    if (travel_policy=='FORWARD_PATH' and (not (native/'libmar_forward_pivot_navigation.so').is_file()
            or not (prefix/'share/ament_index/resource_index/packages/mar_navigation_bt').is_file())):
        raise RuntimeError('BUILD_FORWARD_PIVOT_NAVIGATION_FIRST: bash tools/build_navigation_bt.sh')
    os.environ['LD_LIBRARY_PATH']=str(native)+os.pathsep+os.environ.get('LD_LIBRARY_PATH','')
    os.environ['AMENT_PREFIX_PATH']=str(prefix)+os.pathsep+os.environ.get('AMENT_PREFIX_PATH','')
    cfg,hw=load(config)
    if voice_chat is not None:cfg['voice_chat']['enabled']=voice_chat
    if patrol_fall is not None:
        settings=cfg['ai'].setdefault('patrol_fall', {})
        settings['enabled']=patrol_fall
        if patrol_fall:settings['mode']='CHECKPOINT'
    if fall_monitor is not None:
        settings=cfg['ai'].setdefault('patrol_fall', {})
        settings['enabled']=fall_monitor
        if fall_monitor:settings['mode']='CONTINUOUS'
    if restricted_monitor is not None:
        settings=cfg['ai'].setdefault('patrol_fall',{})
        settings.setdefault('restricted_monitor',{})['enabled']=restricted_monitor
        if restricted_monitor:settings.update(enabled=True,mode='CONTINUOUS')
    if cfg.get('ai', {}).get('patrol_fall', {}).get('enabled'):
        from mediassist.patrol_fall import configuration as fall_configuration, read_token
        settings = fall_configuration(cfg['ai'], cfg['mapping']['camera'])
        read_token(settings['token_file'])
    cfg,hw,geometry=session_geometry(cfg,hw,load_profile)
    mecanum=hw['motion'].get('chassis_model')=='MENTORPI_M1_MECANUM' and hw['motion'].get('allow_strafe') is True
    # Routine travel faces the path; chassis capability alone must not select
    # fixed-yaw strafing. Collision-checked escape XY keeps its hardware limits.
    holonomic=travel_policy=='HOLONOMIC'
    if holonomic and not (mecanum and hw['motion'].get('lidar_braking') is not None):
        raise ValueError('Holonomic travel requires the Mecanum chassis and lidar braking')
    client=SessionClient(cfg);descriptor=client.get(session_id)
    session_id=descriptor['sessionId'];map_id=descriptor['mapId']
    import uuid
    root=ROOT/'pi/state/navigation-sessions'/session_id/uuid.uuid4().hex;root.mkdir(parents=True,mode=0o700,exist_ok=False)
    context=dict(session_id=session_id,map_id=map_id,map_asset_hash=descriptor['mapAssetHash'],expires_at=descriptor['expiresAt'])
    files,meta=verify_bundle(client.request('/api/v1/pi/maps/'+map_id+'/bundle',limit=6*1024*1024),map_id)
    if meta['asset_hash']!=context['map_asset_hash'] or meta['frame_id']!=hw['ros']['map_frame']:raise RuntimeError('SESSION_MAP_HASH_OR_FRAME_MISMATCH')
    for name,raw in files.items():(root/name).write_bytes(raw)
    hardware=copy.deepcopy(hw);hardware.update(map_id=map_id,map_asset_sha256=meta['asset_hash'],endpoints={},endpoint_groups={})
    hardware['navigation_session_id']=session_id;hardware['capabilities']=sorted(set(hardware['capabilities'])|{'nav2'})
    hardware['navigation']['planning_timeout_sec']=max(8.,hardware['navigation']['planning_timeout_sec'])
    manifest=json.loads(client.request('/api/v1/pi/catalog/navigation-profile',limit=2*1024*1024))
    profile=validate_profile({**manifest,'messageType':'NAV_PROFILE','robotId':cfg['robot_id']},cfg['robot_id'],hardware,allow_empty=True)
    hardware['endpoints']=profile['endpoints']
    # Preserve real commissioning evidence and hardware speed limits.
    extent=cfg['mapping']['autonomy'].get('footprint')
    if not extent:raise RuntimeError('CONFIGURE_MEASURED_FOOTPRINT_FIRST')
    polygon=footprint_rectangle(**extent)
    base=yaml.safe_load(Path('/home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml').read_text())
    controller=yaml.safe_load(Path('/home/ubuntu/ros2_ws/src/navigation/config/nav2_controller_dwb.yaml').read_text())
    if not Path('/opt/ros/humble/lib/libnav2_smac_planner.so').is_file():raise RuntimeError('SMAC_PLANNER_NOT_INSTALLED')
    reverse=hw['motion']['max_reverse_speed_mps'] if hw['motion'].get('allow_reverse') else 0.
    required='libdwb_core.so' if holonomic else 'libnav2_regulated_pure_pursuit_controller.so'
    if not Path('/opt/ros/humble/lib',required).is_file():raise RuntimeError('NAVIGATION_CONTROLLER_NOT_INSTALLED: '+required)
    params=nav_parameters(nav2_params(base,controller,polygon,
        speed=min(hw['navigation'].get('max_speed_mps',.25),hw['max_speed_mps']),
        angular=min(2. if mecanum else .6,hw['navigation'].get('max_angular_speed_rps',.8 if mecanum else .6),hw['motion']['max_angular_speed_rps']),
        mecanum=mecanum,fast_navigation=mecanum and hw['motion'].get('lidar_braking') is not None,
        position_then_spin=not position_only,footprint_planner=True,forward_only=not holonomic,
        position_only=position_only,path_tracking=not holonomic,
        full_footprint_search=not holonomic and (mecanum or geometry['profile']=='cargo'),
        holonomic_move=holonomic,reverse_limit=reverse),hardware)
    if not holonomic and mecanum:
        # Retain the saved map's door detail in the footprint planner rather
        # than quantizing a 2.5 cm survey into the vendor's 5 cm default grid.
        resolution=float(meta['resolution_m_per_pixel'])
        for name in ('global_costmap','local_costmap'):
            params[name][name]['ros__parameters']['resolution']=resolution
    (root/'nav2.yaml').write_text(yaml.safe_dump(params,sort_keys=False),encoding='utf-8')
    write_private(root/'hardware.json',json.dumps(hardware,indent=2))
    runtime=copy.deepcopy(cfg);runtime['hardware_profile']=str(root/'hardware.json');runtime['navigation_session']=context
    runtime['stack_state_dir']=str(ROOT/'pi/state/stack')
    runtime['navigation_runtime']=dict(map_yaml=str(root/'map.yaml'),nav2_params=str(root/'nav2.yaml'),localization_only=False,
        arrival_mode=arrival_mode,travel_policy=travel_policy,controller='HOLONOMIC_DWB' if holonomic else 'REGULATED_PURSUIT',
        footprint_planner=True,forward_only=not holonomic,preserve_path_on_replan=True,geometry=geometry,
        full_footprint_search=not holonomic and (mecanum or geometry['profile']=='cargo'))
    runtime['navigation_runtime'].update(native_turn_checked=not holonomic and mecanum)
    runtime['navigation_runtime'].update(obstacle_wait_sec=15.,obstacle_retry_interval_sec=1.)
    runtime['navigation_runtime'].update(localization_wait_sec=30.,localization_retry_count=2)
    status_root=Path('/dev/shm')/('mar-localization-'+str(os.getuid()))
    if status_root.is_symlink() or (status_root.exists() and (not status_root.is_dir() or status_root.stat().st_uid!=os.getuid())):
        raise RuntimeError('LOCALIZATION_RAM_OWNER_MISMATCH')
    status_root.mkdir(mode=0o700,exist_ok=True)
    runtime['localization']={**runtime.get('localization',{}),
        'enabled':True,'mode':'manual','map_id':map_id,
        'map_asset_hash':meta['asset_hash'],'status_file':str(status_root/(session_id+'-'+root.name+'.json'))}
    runtime['mapping']['enabled']=False
    runtime['enabled_step_types']=list(dict.fromkeys([*runtime['enabled_step_types'],'MOVE']))
    # Reuse the durable operation journal; old operations must still be reconciled.
    write_private(root/'worker.json',json.dumps(runtime,indent=2))
    load(root/'worker.json')
    return root,client,descriptor

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json');p.add_argument('--session-id')
    p.add_argument('--arrival-mode',choices=['POSITION_ONLY','POSITION_THEN_SPIN'],default='POSITION_THEN_SPIN')
    p.add_argument('--travel-policy',choices=['FORWARD_PATH','HOLONOMIC'],default='FORWARD_PATH',
                   help='Travel policy for this new session: face the path by default; HOLONOMIC permits routine strafing')
    p.add_argument('--load-profile',choices=['bare','cargo'],help='Override saved-map load geometry; mapping configuration is unchanged')
    p.add_argument('--voice-chat',action=argparse.BooleanOptionalAction,default=None,
                   help='Override the managed idle wake/chat listener for this session only')
    p.add_argument('--patrol-fall',action=argparse.BooleanOptionalAction,default=None,
                   help='Enable or disable stationary fall observation for patrol checkpoints in this session')
    p.add_argument('--fall-monitor',action=argparse.BooleanOptionalAction,default=None,
                   help='Enable or disable background fall observation during MOVE/IDLE for this session')
    p.add_argument('--restricted-monitor',action=argparse.BooleanOptionalAction,default=None,
                   help='Reuse EC2 pose results and registered depth for patrol KEEP_OUT monitoring; requires the updated BE/AI')
    a=p.parse_args()
    if a.patrol_fall is not None and a.fall_monitor is not None:
        p.error('Choose either --patrol-fall or --fall-monitor')
    with stopped_configuration(a.config):root,client,descriptor=prepare(a.config,a.session_id,a.arrival_mode,a.load_profile,a.voice_chat,a.patrol_fall,a.fall_monitor,a.travel_policy,a.restricted_monitor)
    print('NAVIGATION_SESSION:',descriptor['sessionId'],'MAP:',descriptor['mapId'])
    print('Base configuration preserved. Waiting for initial pose/global localization from the map UI; no goal sent.')
    navigation=json.loads((root/'worker.json').read_text())['navigation_runtime']
    print('ARRIVAL_MODE:',a.arrival_mode,'TRAVEL_POLICY:',navigation['travel_policy'],'CONTROLLER:',navigation['controller'])
    print('NAVIGATION_GEOMETRY:',json.dumps(navigation['geometry']))
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
