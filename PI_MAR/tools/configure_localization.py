"""Bind localization to an installed map and create a separate Nav2 configuration. No motor commands."""
import argparse
import copy
import json
from pathlib import Path
import sys
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'pi'))
from mediassist.worker_config import load
from mediassist.localization import configuration,GridMatcher
from mediassist.map_tools import map_fingerprint
from configure_mapping import stopped_configuration,write_configuration

def nav_parameters(base,hardware):
    output=copy.deepcopy(base)
    amcl=output.setdefault('amcl',{}).setdefault('ros__parameters',{})
    amcl.update(robot_model_type='nav2_amcl::OmniMotionModel',set_initial_pose=False,save_pose_rate=-1.,always_reset_initial_pose=True,
                tf_broadcast=True,transform_tolerance=.3,scan_topic=hardware['ros']['scan'],
                base_frame_id=hardware['ros']['base_frame'],global_frame_id=hardware['ros']['map_frame'],
                odom_frame_id='odom',update_min_d=.05,update_min_a=.05)
    return output

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    p.add_argument('--mode',choices=('seed','global'),required=True)
    p.add_argument('--x',type=float);p.add_argument('--y',type=float);p.add_argument('--yaw',type=float)
    p.add_argument('--nav2-params',type=Path,required=True,help='Existing vendor/generated Nav2 YAML to copy')
    p.add_argument('--output-nav2-params',type=Path,required=True,help='New file; existing files are never overwritten')
    a=p.parse_args();a.config=a.config.resolve(strict=True)
    if a.mode=='seed' and any(v is None for v in (a.x,a.y,a.yaw)):p.error('Seed mode requires --x --y --yaw in map metres/radians')
    if a.mode=='global' and any(v is not None for v in (a.x,a.y,a.yaw)):p.error('Global mode does not accept a guessed initial pose')
    with stopped_configuration(a.config):
        cfg,hw=load(a.config)
        if cfg['mapping']['enabled']:p.error('Install the assigned navigation map first; mapping must be disabled')
        map_yaml=Path(cfg.get('navigation_runtime',{}).get('map_yaml','')).resolve(strict=True)
        meta=json.loads((map_yaml.parent/'metadata.json').read_text())
        if map_fingerprint(map_yaml)!=hw['map_asset_sha256'] or meta['map_id']!=hw['map_id']:p.error('Installed map identity/hash differs')
        guard=configuration(dict(enabled=True,mode=a.mode,map_id=hw['map_id'],map_asset_hash=hw['map_asset_sha256'],
                                 status_file=str((a.config.parent.parent/'state/stack/localization.json').resolve()),
                                 **({'initial_pose':dict(x_m=a.x,y_m=a.y,yaw_rad=a.yaw)} if a.mode=='seed' else {})))
        if a.mode=='seed':
            geometry=GridMatcher(meta['width_px'],meta['height_px'],meta['resolution_m_per_pixel'],
                                 (meta['origin_x_m'],meta['origin_y_m'],meta['origin_yaw_rad']),[0]*(meta['width_px']*meta['height_px']))
            u,v=geometry.index(a.x,a.y)
            if not 0<=u<geometry.w or not 0<=v<geometry.h:p.error('Initial pose is outside this map raster')
        base=yaml.safe_load(a.nav2_params.read_text())
        if not isinstance(base,dict):p.error('Nav2 parameters must be a YAML object')
        output=a.output_nav2_params.resolve();output.parent.mkdir(parents=True,exist_ok=True)
        with output.open('x',encoding='utf-8') as stream:yaml.safe_dump(nav_parameters(base,hw),stream,sort_keys=False)
        cfg['localization']=guard;cfg['navigation_runtime']['nav2_params']=str(output)
        try:write_configuration(a.config,cfg,hw)
        except Exception:output.unlink(missing_ok=True);raise
    print('Localization configured:',a.mode,'map:',hw['map_id'])
    print('Nav2 parameters:',output)
    print('No motion or initial-pose command sent. Next:')
    print('zsh tools/robot.zsh run --mode navigation --nav2-params '+str(output))

if __name__=='__main__':main()
