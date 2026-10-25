"""Read current configuration and last mapping failure; never starts drivers or motion."""
import argparse,json,sys,time,base64,zlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools')]
from mediassist.footprint import body_clear

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json');a=p.parse_args()
 c=json.loads(a.config.read_text(encoding='utf-8-sig'));m=c['mapping'];h=json.loads(Path(c['hardware_profile']).read_text())
 print('MAPPING_MODE',m['autonomy']['exploration_mode']);print('SPEED_LIMITS',m['autonomy']['speed_mps'],m['autonomy']['angular_rps']);print('UPLOAD_ORIGIN',m['upload_base_url'])
 print('HARDWARE_PROFILE',c['hardware_profile']);print('FOOTPRINT',json.dumps(m['autonomy']['footprint']))
 state=Path(m['storage_dir'])/'session.json'
 if state.is_file():
  s=json.loads(state.read_text());print('LOCAL_MAPPING_STATE',json.dumps({k:s.get(k) for k in ('state','session_open','job_id','session_id')}))
  print('LAST_AUTONOMY',json.dumps(s.get('autonomy',{})))
 folder=Path(m['storage_dir'])/'diagnostics'
 files=sorted(folder.glob('*.json'),key=lambda x:x.stat().st_mtime,reverse=True)
 if files:
  path=files[0];d=json.loads(path.read_text());s=d['snapshot'];s['data']=zlib.decompress(base64.b64decode(s['data']))
  print('LAST_DIAGNOSTIC',path,'age_sec',round(time.time()-path.stat().st_mtime));print('LAST_SLAM_POSE',json.dumps(d['pose']))
  print('BODY_MAP_INTERSECTION',json.dumps(body_clear(s,d['pose'],d['autonomy']['footprint'])))
 print('Saved-map pose requires a NEW live navigation session + READY AMCL; an old diagnostic or SLAM pose is not a current saved-map localization.')
 print('MS200 vendor SDK supports 5..15 Hz; launch motor_speed=15 does not prove measured 15 Hz. Measure /scan_raw while drivers run.')

if __name__=='__main__':main()
