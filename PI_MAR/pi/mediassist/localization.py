"""ROS-independent localization evidence. A fresh TF alone is not a localization proof."""
import math
from .contracts import StepError

DEFAULTS = dict(enabled=False, mode='global', max_sensor_age_sec=.6, max_pose_age_sec=.8,
                max_xy_std_m=.15, max_yaw_std_rad=.20, min_scan_match=.60,
                min_known_fraction=.50, min_valid_beams=40, match_distance_m=.12,min_particle_support=.85,
                stable_sec=3., stable_scans=8, bootstrap_timeout_sec=60.,
                max_tf_amcl_distance_m=.35, max_tf_amcl_yaw_rad=.4)

def configuration(raw):
    cfg={**DEFAULTS, **raw}
    if type(cfg['enabled']) is not bool or cfg['mode'] not in ('global','seed','manual'):
        raise StepError('INVALID_LOCALIZATION_CONFIG')
    bounds={'max_sensor_age_sec':(.1,2), 'max_pose_age_sec':(.2,1), 'max_xy_std_m':(.02,.5),
            'max_yaw_std_rad':(.03,.6), 'min_scan_match':(.3,1), 'min_known_fraction':(.1,1),
            'min_valid_beams':(10,1000), 'match_distance_m':(.02,.3), 'stable_sec':(1,15),
            'stable_scans':(3,100), 'bootstrap_timeout_sec':(10,300),
            'max_tf_amcl_distance_m':(.05,.6), 'max_tf_amcl_yaw_rad':(.05,.7)}
    bounds['min_particle_support']=(.7,1)
    for key,(lo,hi) in bounds.items():
        value=cfg[key]
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            raise StepError('INVALID_LOCALIZATION_CONFIG',key)
    for key in ('stable_scans','min_valid_beams'):
        if type(cfg[key]) is not int:raise StepError('INVALID_LOCALIZATION_CONFIG',key)
    if cfg['enabled']:
        if not cfg.get('map_id') or not cfg.get('map_asset_hash'):raise StepError('LOCALIZATION_MAP_REQUIRED')
        if cfg['mode']=='seed':
            pose=cfg.get('initial_pose',{})
            if any(type(pose.get(k)) not in (int,float) or not math.isfinite(pose[k]) for k in ('x_m','y_m','yaw_rad')):
                raise StepError('INITIAL_POSE_REQUIRED')
            if abs(pose['yaw_rad'])>math.pi:raise StepError('INVALID_INITIAL_YAW')
    return cfg

def angle(value):return math.atan2(math.sin(value),math.cos(value))

def covariance_std(cov):
    if len(cov)!=36 or any(not math.isfinite(v) for v in cov):raise ValueError('covariance')
    x,y,a=cov[0],cov[7],cov[35];xy,xa,ya=cov[1],cov[5],cov[11]
    if min(x,y,a)<=0 or any(abs(cov[i]-cov[j])>1e-6 for i,j in ((1,6),(5,30),(11,31))):raise ValueError('covariance')
    if min(x*y-xy*xy,x*a-xa*xa,y*a-ya*ya)<-1e-10:raise ValueError('covariance')
    if x*y*a+2*xy*xa*ya-x*ya*ya-y*xa*xa-a*xy*xy < -1e-10:raise ValueError('covariance')
    return math.sqrt((x+y+math.sqrt((x-y)**2+4*xy*xy))/2),math.sqrt(a)

class QualityGate:
    def __init__(self,cfg):self.cfg=configuration(cfg);self.reset()
    def reset(self):
        self.since=None;self.count=0;self.last_scan=None;self.previous_pose=None
        self.had_lock=False;self.last_good_at=None;self.recovering=False;self.tracking=False
    def update(self,now,sample):
        c=self.cfg;reasons=[];metrics={}
        for key,code in (('map_verified','MAP_NOT_VERIFIED'),('amcl_active','AMCL_NOT_ACTIVE'),('tf_fresh','TF_NOT_READY')):
            if sample.get(key) is not True:reasons.append(code)
        for key,limit in (('scan_age_sec',c['max_sensor_age_sec']),('odom_age_sec',c['max_sensor_age_sec']),('pose_age_sec',c['max_pose_age_sec'])):
            age=sample.get(key)
            if type(age) not in (int,float) or not math.isfinite(age) or not 0<=age<=limit:reasons.append(key.upper()+'_STALE')
        try:
            xy,yaw=covariance_std(sample.get('covariance',[]));metrics.update(xy_std_m=xy,yaw_std_rad=yaw)
            if xy>c['max_xy_std_m'] or yaw>c['max_yaw_std_rad']:reasons.append('AMCL_UNCERTAIN')
        except (ValueError,TypeError):reasons.append('AMCL_COVARIANCE_INVALID')
        try:
            p,a=sample['pose'],sample['amcl_pose']
            if any(not math.isfinite(p[k]) or not math.isfinite(a[k]) for k in ('x_m','y_m','yaw_rad')):raise ValueError()
            if math.hypot(p['x_m']-a['x_m'],p['y_m']-a['y_m'])>c['max_tf_amcl_distance_m'] or abs(angle(p['yaw_rad']-a['yaw_rad']))>c['max_tf_amcl_yaw_rad']:reasons.append('TF_AMCL_DISAGREE')
            if self.previous_pose:
                old,at=self.previous_pose;dt=max(0,now-at)
                if math.hypot(p['x_m']-old['x_m'],p['y_m']-old['y_m'])>.3+.35*dt or abs(angle(p['yaw_rad']-old['yaw_rad']))>.4+.6*dt:reasons.append('POSE_JUMP')
            self.previous_pose=(dict(p),now)
        except (KeyError,TypeError,ValueError):reasons.append('POSE_INVALID')
        for key,minimum in (('scan_match',c['min_scan_match']),('known_fraction',c['min_known_fraction']),('valid_beams',c['min_valid_beams'])):
            value=sample.get(key)
            if type(value) not in (int,float) or not math.isfinite(value) or value<minimum:reasons.append(key.upper()+'_LOW')
            if type(value) in (int,float) and math.isfinite(value):metrics[key]=value
        if c['mode']=='global':
            support=sample.get('particle_support',0.)
            if not math.isfinite(support) or support<c['min_particle_support']:reasons.append('PARTICLES_NOT_CONVERGED')
            metrics['particle_support']=support if math.isfinite(support) else None
        if reasons:
            transient={'SCAN_AGE_SEC_STALE','ODOM_AGE_SEC_STALE','POSE_AGE_SEC_STALE',
                       'SCAN_MATCH_LOW','KNOWN_FRACTION_LOW','VALID_BEAMS_LOW'}
            self.recovering=bool(self.had_lock and self.last_good_at is not None and now-self.last_good_at<=2.
                                 and set(reasons)<=transient)
            if not self.recovering:self.had_lock=False
            self.tracking=False
            self.since=None;self.count=0;self.last_scan=None
            return dict(ready=False,reason=reasons[0],reasons=reasons,stable_scans=0,metrics=metrics)
        if sample.get('scan_sequence')!=self.last_scan:
            self.last_scan=sample.get('scan_sequence');self.count+=1
            if self.since is None:self.since=now
        if self.recovering and now-self.last_good_at>2.:
            self.recovering=False;self.had_lock=False
        needed_scans,needed_sec=(2,.5) if self.recovering else (c['stable_scans'],c['stable_sec'])
        ready=self.tracking or (self.count>=needed_scans and self.since is not None and now-self.since>=needed_sec)
        if ready:self.had_lock=True;self.last_good_at=now;self.recovering=False;self.tracking=True
        return dict(ready=ready,reason='LOCALIZED' if ready else 'VERIFYING',reasons=[],stable_scans=self.count,metrics=metrics)

class GridMatcher:
    """Checks laser hit endpoints against occupied cells; never interprets unknown as free."""
    def __init__(self,width,height,resolution,origin,cells,distance=.12):
        if width<1 or height<1 or width*height>4_000_000 or len(cells)!=width*height or resolution<=0:raise ValueError('grid')
        self.w,self.h,self.r,self.origin,self.cells=width,height,resolution,origin,cells
        # Build once per loaded map. Each scan endpoint is then an O(1) lookup,
        # rather than a Python search through hundreds of neighbouring cells.
        from .frontier_mapping import squared_distances
        if any(v>=65 for v in cells):
            distances=squared_distances(bytearray(v>=65 for v in cells),width,height,boundary=False)
            self.near_occupied=bytes(d*resolution*resolution<=distance*distance+1e-12 for d in distances)
        else:self.near_occupied=bytes(width*height)
    def index(self,x,y):
        ox,oy,yaw=self.origin;c,s=math.cos(yaw),math.sin(yaw);dx,dy=x-ox,y-oy
        return math.floor((c*dx+s*dy)/self.r),math.floor((-s*dx+c*dy)/self.r)
    def score(self,points):
        matched=known=0;total=len(points)
        for x,y in points:
            u,v=self.index(x,y)
            if not 0<=u<self.w or not 0<=v<self.h:continue
            if self.cells[v*self.w+u]>=0:known+=1
            if self.near_occupied[v*self.w+u]:matched+=1
        return dict(scan_match=matched/total if total else 0.,known_fraction=known/total if total else 0.,valid_beams=total)


def bootstrap_verdict(status,initialized,ever_ready,elapsed,timeout,error):
    """Timeout requests operator help; later fresh, converged evidence can still pass."""
    if not initialized:return {**status,'ready':False,'reason':error}
    if not status['ready'] and not ever_ready and elapsed>timeout:
        return {**status,'ready':False,'reason':'BOOTSTRAP_TIMEOUT_NEEDS_INITIAL_POSE'}
    return status

def particle_support(particles,pose):
    """Weighted support near the reported hypothesis; multiple global hypotheses are not ready."""
    if len(particles)<50:return 0.
    total=near=0.
    for x,y,a,weight in particles:
        if not all(math.isfinite(v) for v in (x,y,a,weight)) or weight<0:return 0.
        total+=weight
        if math.hypot(x-pose['x_m'],y-pose['y_m'])<=.5 and abs(angle(a-pose['yaw_rad']))<=.5:near+=weight
    return near/total if total>0 else 0.

def guarded_pose(status,received_age,ros_now,map_id,asset_hash,frame_id='map'):
    empty={'value':None,'age_sec':None,'fresh':False}
    try:
        if not 0<=received_age<=.75 or status.get('ready') is not True:return empty
        if status['map_id']!=map_id or status['map_asset_hash']!=asset_hash or status['frame_id']!=frame_id:return empty
        if not 0<=ros_now-status['observed_at_sec']<=1.:return empty
        p=status['pose']
        if any(type(p[k]) not in (int,float) or not math.isfinite(p[k]) for k in ('x_m','y_m','yaw_rad')):return empty
        age=status['pose_age_sec']+received_age
        if not 0<=age<=1.:return empty
        return dict(value=dict(map_id=map_id,**{k:p[k] for k in ('x_m','y_m','yaw_rad')}),age_sec=age,fresh=True)
    except (KeyError,TypeError,ValueError):return empty
