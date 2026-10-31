"""Bounded mapping motion with optional frontier guidance. No ROS or motor access here."""
import math
from .contracts import StepError


def configuration(mapping):
    cfg = mapping.setdefault('autonomy', {})
    if not isinstance(cfg,dict):raise StepError('INVALID_AUTONOMY_CONFIG')
    defaults = dict(enabled=False, vehicle='rigid', min_turn_radius_m=.60, speed_mps=.10, angular_rps=.30, stop_margin_m=.10,
                    turn_clearance_m=.40, slow_clearance_m=.70, max_duration_sec=180,
                    return_radius_m=.25, min_return_sec=30., min_return_travel_m=3.,
                    scan_max_age_sec=.4, blocked_timeout_sec=8.,exploration_mode='reactive',
                    manual_lidar_assist=False,frontier_min_length_m=.20,coverage_stable_sec=15.,
                    coverage_min_duration_sec=30.,coverage_min_travel_m=0.,coverage_min_area_m2=.5,
                    frontier_no_progress_sec=15.,lidar_stop_distance_m=.10,frontier_controller='nav2',
                    visit_known_space=False,visit_radius_m=.6,reverse_enabled=False,max_reverse_mps=.1,control_pause_sec=0.)
    for key, value in defaults.items(): cfg.setdefault(key, value)
    if cfg['vehicle'] not in ('rigid','articulated'):raise StepError('INVALID_AUTONOMY_VEHICLE')
    if cfg['frontier_controller'] not in ('nav2','reactive'):raise StepError('INVALID_FRONTIER_CONTROLLER')
    if cfg['exploration_mode'] not in ('reactive','frontier') or type(cfg['manual_lidar_assist']) is not bool:
        raise StepError('INVALID_EXPLORATION_CONFIG')
    if type(cfg.get('enabled')) is not bool: raise StepError('INVALID_AUTONOMY_CONFIG')
    if any(type(cfg[k]) is not bool for k in ('visit_known_space','reverse_enabled')):raise StepError('INVALID_AUTONOMY_CONFIG')
    if cfg['reverse_enabled'] and (cfg['vehicle']!='rigid' or cfg['frontier_controller']!='nav2' or cfg['exploration_mode']!='frontier'):
        raise StepError('AUTONOMOUS_REVERSE_REQUIRES_RIGID_NAV2')
    # The final manual/MOVE braking profile also covers the vendor's 2 rad/s
    # joystick ceiling. Rigid exploration has an independent 1.2 rad/s limit.
    braking_yaw=(2. if cfg.get('include_points') is True else 1.2) if cfg['vehicle']=='rigid' else .8
    for key, low, high in (('speed_mps', .02, 1.00), ('angular_rps', .10, braking_yaw),
                          ('stop_margin_m', .03, .4), ('turn_clearance_m', .2, 1.),
                          ('slow_clearance_m', .3, 2.), ('max_duration_sec', 10, 600),
                          ('return_radius_m', .15, .5), ('min_return_sec', 10, 300),
                          ('min_return_travel_m', 1, 50), ('scan_max_age_sec', .1, .6),
                          ('blocked_timeout_sec', 2, 20), ('min_turn_radius_m', .5, 2.),
                          ('frontier_min_length_m',.10,.5),('coverage_stable_sec',10,60),
                          ('coverage_min_duration_sec',20,120),('coverage_min_travel_m',0.,5.),
                          ('coverage_min_area_m2',.3,10.),('frontier_no_progress_sec',10,60),('lidar_stop_distance_m',.05,.50),
                          ('visit_radius_m',.3,1.5),('max_reverse_mps',.02,.8),('control_pause_sec',0.,2.)):
        value = cfg[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise StepError('INVALID_AUTONOMY_CONFIG', key)
    if not cfg['stop_margin_m'] < cfg['turn_clearance_m'] < cfg['slow_clearance_m']:
        raise StepError('INVALID_AUTONOMY_CLEARANCES')
    if max(cfg['speed_mps'],cfg['max_reverse_mps']) > .25:
        if cfg['vehicle'] != 'rigid' or (cfg['exploration_mode']=='frontier' and cfg['frontier_controller'] != 'nav2'):
            raise StepError('FAST_MAPPING_REQUIRES_RIGID_NAV2')
        fast=max(cfg['speed_mps'],cfg['max_reverse_mps'])
        if cfg['slow_clearance_m'] < (.80 if fast <= .5 else 1.9*fast) or cfg['scan_max_age_sec'] > .4:
            raise StepError('FAST_MAPPING_REQUIRES_EARLY_SLOWDOWN_AND_FRESH_SCAN')
    if cfg['enabled']:
        footprint = cfg.get('footprint', {})
        if any(type(footprint.get(k)) not in (int,float) or not math.isfinite(footprint[k]) or
               not .05 <= footprint[k] <= 1.5 for k in ('front','rear','left','right')):
            raise StepError('MEASURED_AUTONOMY_FOOTPRINT_REQUIRED')
    return cfg


def scan_view(scan, transform, cfg):
    """Measure a base-frame corridor and full rotation envelope using raw scan rays."""
    try:
        tx, ty, yaw = transform[:3]
        direction=transform[3] if len(transform)==4 else 1
        if direction not in (-1,1):raise ValueError()
        if (not all(math.isfinite(v) for v in (tx,ty,yaw,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max))
                or scan.angle_increment == 0 or not 30 <= len(scan.ranges) <= 10000
                or not 0 <= scan.range_min < scan.range_max):
            raise ValueError()
        f = cfg['footprint']; margin = cfg['stop_margin_m']
        radius = max(math.hypot(f[x], f[y]) for x in ('front','rear') for y in ('left','right')) + margin
        if scan.range_max < max(1., radius + .5): raise ValueError()
        bins, valid_bins, points = set(), set(), []
        nearest_lidar=scan.range_max
        selected = valid = 0
        for index, distance in enumerate(scan.ranges):
            a = direction*(scan.angle_min + index * scan.angle_increment) + yaw
            slot = int(((a + math.pi) % (2 * math.pi)) / (2 * math.pi) * 36)
            bins.add(slot); selected += 1
            if math.isinf(distance) and distance > 0:
                valid_bins.add(slot); valid += 1; continue
            if not math.isfinite(distance) or not max(.01, scan.range_min) <= distance <= scan.range_max: continue
            nearest_lidar=min(nearest_lidar,distance)
            valid_bins.add(slot); valid += 1
            points.append((tx + distance * math.cos(a), ty + distance * math.sin(a)))
        if len(bins) < 34 or len(valid_bins) < 33 or valid < selected * .8:
            return {'valid': False, 'reason': 'LIDAR_COVERAGE_NOT_READY'}
        front = scan.range_max - f['front']
        left = right = scan.range_max
        lateral_left=lateral_right=back=scan.range_max
        rotation_clear = True
        for x, y in points:
            d = math.hypot(x, y)
            if d < radius: rotation_clear = False
            # Rear returns must not become a negative FRONT clearance.
            # The separate all-around minimum below still guards near returns.
            if x >= 0 and -f['right'] - margin <= y <= f['left'] + margin:
                front = min(front, x - f['front'])
            if x <= 0 and -f['right']-margin <= y <= f['left']+margin:
                back=min(back,-x-f['rear'])
            if -f['rear']-margin <= x <= f['front']+margin:
                if y>=0:lateral_left=min(lateral_left,y-f['left'])
                else:lateral_right=min(lateral_right,-y-f['right'])
            a = math.atan2(y, x)
            if math.radians(20) <= a <= math.radians(120): left = min(left, d)
            if math.radians(-120) <= a <= math.radians(-20): right = min(right, d)
        arcs={}
        # For an articulated trailer, check a conservative full-body circle
        # along each forward arc. footprint must bound ALL trailer angles.
        if cfg['vehicle']=='articulated':
            r=cfg['min_turn_radius_m']; v=min(.06,cfg['speed_mps'])
            for sign,label in ((1.,'arc_left_clear'),(-1.,'arc_right_clear')):
                centers=[(r*math.sin(v/r*t),sign*r*(1-math.cos(v/r*t))) for t in (.0,.5,1.,1.5,2.)]
                arcs[label]=all(math.hypot(x-cx,y-cy)>radius for x,y in points for cx,cy in centers)
        return dict(**arcs,**({'obstacle_points':points} if cfg.get('include_points') else {}),valid=True, reason='CLEAR', front_m=front, left_m=left, right_m=right,
                    lateral_left_m=lateral_left,lateral_right_m=lateral_right,back_m=back,
                    nearest_lidar_m=nearest_lidar,lidar_stop_distance_m=cfg['lidar_stop_distance_m'],
                    rotation_clear=rotation_clear)
    except (ValueError,TypeError,AttributeError,KeyError):
        return {'valid': False, 'reason': 'LIDAR_INVALID'}


def braking_speed_cap(clearance_m, scan_age_sec, stop_margin_m):
    """Cap speed so a 0.6 m/s² stop leaves the requested body clearance."""
    if not all(math.isfinite(v) for v in (clearance_m, scan_age_sec, stop_margin_m)) or scan_age_sec < 0:
        return 0.
    remaining=max(0.,clearance_m-stop_margin_m)
    reaction=scan_age_sec+.35
    return max(0.,math.sqrt((.6*reaction)**2+1.2*remaining)-.6*reaction)


class AutoMapper:
    def __init__(self, cfg):
        self.cfg = cfg
        self.run_id = None; self.phase = 'IDLE'; self.reason = 'NOT_STARTED'
        self.start_at = self.deadline = 0.; self.last_pose = self.start_pose = None
        self.travel = 0.; self.departed = False; self.turn = 0.; self.turn_at = None
        self.stop_on_return = False; self.duration = cfg['max_duration_sec']
        self.last_now = self.end_at = None
        self.save_on_complete=False
        self.pose_jump=None
        self.gate_blocked_at=None

    def accept(self, message, now, pose, lease_sec):
        self.last_now=now
        action, run = message['action'], message['runId']
        save_on_complete=message.get('saveOnComplete',False)
        if type(save_on_complete) is not bool:raise StepError('INVALID_AUTONOMY_SAVE_OPTION')
        if action == 'STOP':
            if self.run_id is not None and run != self.run_id: raise StepError('AUTONOMY_RUN_MISMATCH')
            self.stop('OPERATOR_STOP'); return
        if not self.cfg['enabled']: raise StepError('AUTONOMY_NOT_CONFIGURED')
        if action == 'START':
            if self.run_id == run:
                if self.phase != 'RUNNING': raise StepError('AUTONOMY_RUN_FINISHED')
                if (message['maxDurationSec'],message['stopOnReturn'],save_on_complete) != (self.duration,self.stop_on_return,self.save_on_complete):
                    raise StepError('AUTONOMY_RUN_OPTIONS_CHANGED')
                if now >= self.deadline+self.cfg.get('control_pause_sec',0.):
                    self.stop('CONTROL_LEASE_EXPIRED'); raise StepError('AUTONOMY_RUN_FINISHED')
            else:
                if self.phase == 'RUNNING': raise StepError('AUTONOMY_ALREADY_RUNNING')
                if not pose or not all(math.isfinite(pose.get(k,math.nan)) for k in ('x_m','y_m','yaw_rad')):
                    raise StepError('MAPPING_POSE_NOT_READY')
                if not 10 <= message['maxDurationSec'] <= self.cfg['max_duration_sec']:
                    raise StepError('AUTONOMY_DURATION_EXCEEDS_PROFILE')
                self.run_id, self.start_at = run, now
                self.start_pose = {k:pose[k] for k in ('x_m','y_m','yaw_rad')}; self.last_pose = (dict(pose),now)
                self.travel = 0.; self.departed = False; self.turn = 0.; self.turn_at = None
                self.duration, self.stop_on_return = message['maxDurationSec'],message['stopOnReturn']
                self.save_on_complete=save_on_complete
                self.phase, self.reason = 'RUNNING','STARTED'
                self.end_at=None
                self.pose_jump=None
                self.gate_blocked_at=None
        elif action == 'RENEW':
            if run != self.run_id or self.phase != 'RUNNING': raise StepError('AUTONOMY_NOT_RUNNING')
            if (message['maxDurationSec'],message['stopOnReturn'],save_on_complete) != (self.duration,self.stop_on_return,self.save_on_complete):
                raise StepError('AUTONOMY_RUN_OPTIONS_CHANGED')
            if now >= self.deadline+self.cfg.get('control_pause_sec',0.):
                self.stop('CONTROL_LEASE_EXPIRED'); raise StepError('AUTONOMY_RUN_FINISHED')
        else: raise StepError('INVALID_AUTONOMY_ACTION')
        self.deadline = now + min(.9, max(0., lease_sec))

    def stop(self, reason):
        if self.phase == 'RUNNING' or self.phase == 'IDLE' and reason == 'OPERATOR_STOP':
            self.phase, self.reason = 'STOPPED',reason
            self.end_at=self.last_now
        self.turn_at = None
        self.gate_blocked_at=None
        return (0.,0.)

    def current(self, now, view, age, pose, allowed, blocked_reason='NOT_READY', gate=None, guidance=None, hold=False):
        self.last_now=now
        if self.phase != 'RUNNING': return (0.,0.)
        if now >= self.deadline:
            if now-self.deadline < self.cfg.get('control_pause_sec',0.):
                self.reason='WAITING_FOR_CONTROL_LEASE';return (0.,0.)
            return self.stop('CONTROL_LEASE_EXPIRED')
        if now - self.start_at >= self.duration: return self.stop('TIME_LIMIT')
        if not allowed: return self.stop(blocked_reason)
        if not view or not view.get('valid') or not 0 <= age <= self.cfg['scan_max_age_sec']:
            return self.stop(view.get('reason','LIDAR_NOT_READY') if view and not view.get('valid') else 'LIDAR_STALE')
        if view.get('nearest_lidar_m',math.inf)<=self.cfg['lidar_stop_distance_m']:
            return self.stop('LIDAR_OBSTACLE_WITHIN_10CM' if self.cfg['lidar_stop_distance_m']==.10 else 'LIDAR_OBSTACLE_WITHIN_STOP_DISTANCE')
        if not pose or not all(math.isfinite(pose.get(k,math.nan)) for k in ('x_m','y_m','yaw_rad')):
            return self.stop('MAPPING_POSE_NOT_READY')
        if gate is not None and (not isinstance(gate,dict) or gate.get('reason') not in ('CLEAR','NO_LEASE','STALE_COMMAND','TURN_ENVELOPE_BLOCKED')):
            if not isinstance(gate,dict):return self.stop('GATE_INVALID_STATUS')
            if gate.get('reason')=='REAR_OBSTACLE':
                if self.gate_blocked_at is None:self.gate_blocked_at=now
                # A vetoed reverse candidate is a reason to choose another goal,
                # not permission to finish the whole exploration run.
                self.reason='REPLANNING_REAR_BLOCKED';return (0.,0.)
            return self.stop('GATE_' + str(gate.get('reason','UNKNOWN')))
        self.gate_blocked_at=None
        previous, at = self.last_pose; dt = max(0.,now-at)
        delta = math.hypot(pose['x_m']-previous['x_m'],pose['y_m']-previous['y_m'])
        limit=.20 + max(.4,1.5*self.cfg['speed_mps'])*dt
        if delta > limit:
            self.pose_jump=dict(previous=previous,current=dict(pose),delta_m=delta,dt_sec=dt,limit_m=limit)
            return self.stop('MAP_POSE_JUMP')
        self.travel += delta; self.last_pose = (dict(pose),now)
        distance = math.hypot(pose['x_m']-self.start_pose['x_m'],pose['y_m']-self.start_pose['y_m'])
        self.departed |= distance > self.cfg['return_radius_m']*2
        yaw_error = abs(math.atan2(math.sin(pose['yaw_rad']-self.start_pose['yaw_rad']),math.cos(pose['yaw_rad']-self.start_pose['yaw_rad'])))
        if (self.stop_on_return and self.departed and now-self.start_at >= self.cfg['min_return_sec']
                and self.travel >= self.cfg['min_return_travel_m'] and distance <= self.cfg['return_radius_m'] and yaw_error <= .35):
            return self.stop('RETURNED_TO_START')
        if hold:
            self.reason='WAITING_FOR_MAP_PLAN';return (0.,0.)
        if guidance and guidance['hold']:
            # MS200 observes 360 degrees. Do not rotate a trailer just to face
            # the frontier after reaching the stand-off observation point.
            self.reason='OBSERVING_FRONTIER';return (0.,0.)
        front = view['front_m']
        if front <= self.cfg['stop_margin_m']: return self.stop('FRONT_OBSTACLE_TOO_CLOSE')
        if front <= self.cfg['turn_clearance_m'] or self.turn_at is not None:
            if not view['rotation_clear']: return self.stop('ROTATION_ENVELOPE_BLOCKED')
            if self.turn_at is None:
                self.turn_at = now; self.turn = 1. if view['left_m'] >= view['right_m'] else -1.
                if self.cfg['vehicle']=='articulated':
                    left,right=view.get('arc_left_clear',False),view.get('arc_right_clear',False)
                    if not left and not right:return self.stop('TRAILER_NO_CLEAR_FORWARD_ARC')
                    if not left:self.turn=-1.
                    elif not right:self.turn=1.
            if now-self.turn_at >= self.cfg['blocked_timeout_sec']: return self.stop('TURN_TIMEOUT')
            if now-self.turn_at < .5 or front < self.cfg['turn_clearance_m']+.15:
                self.reason = 'TURN_LEFT' if self.turn > 0 else 'TURN_RIGHT'
                if self.cfg['vehicle']=='articulated':
                    key='arc_left_clear' if self.turn>0 else 'arc_right_clear'
                    if not view.get(key,False):return self.stop('TRAILER_FORWARD_ARC_BLOCKED')
                    speed=min(.06,self.cfg['speed_mps'])
                    return (speed,self.turn*min(self.cfg['angular_rps'],speed/self.cfg['min_turn_radius_m']))
                return (0.,self.turn*self.cfg['angular_rps'])
            self.turn_at = None
        self.reason = 'FORWARD'
        safe_speed=min(self.cfg['speed_mps'],braking_speed_cap(front,age,self.cfg['stop_margin_m']))
        if guidance:
            error=math.atan2(math.sin(guidance['yaw']-pose['yaw_rad']),math.cos(guidance['yaw']-pose['yaw_rad']))
            if guidance['hold'] and abs(error)<.15:
                self.reason='OBSERVING_FRONTIER';return (0.,0.)
            angular=max(-self.cfg['angular_rps'],min(self.cfg['angular_rps'],error*1.2))
            if self.cfg['vehicle']=='articulated':
                if abs(angular)>.01:
                    key='arc_left_clear' if angular>0 else 'arc_right_clear'
                    if not view.get(key,False):return self.stop('TRAILER_FORWARD_ARC_BLOCKED')
                speed=min(safe_speed,.06 if abs(error)>.15 else self.cfg['speed_mps'])
                angular=max(-speed/self.cfg['min_turn_radius_m'],min(angular,speed/self.cfg['min_turn_radius_m']))
            else:
                if abs(angular)>.01 and not view['rotation_clear']:return self.stop('ROTATION_ENVELOPE_BLOCKED')
                speed=0. if abs(error)>.35 or guidance['hold'] else safe_speed*max(0.,math.cos(error))
            self.reason='FOLLOW_FRONTIER_PATH'
            return (speed,angular)
        return (safe_speed,0.)

    def status(self, now):
        return dict(vehicle=self.cfg['vehicle'],available=self.cfg['enabled'],run_id=self.run_id,state=self.phase,reason=self.reason,
                    finished=self.phase=='STOPPED',coverage_complete=self.phase=='STOPPED' and self.reason=='COVERAGE_COMPLETE',
                    exploration_mode=self.cfg['exploration_mode'],manual_lidar_assist=self.cfg['manual_lidar_assist'],
                    elapsed_sec=round(max(0.,(self.end_at if self.end_at is not None else now)-self.start_at),1) if self.run_id else 0.,
                    travel_m=round(self.travel,3),start_pose=self.start_pose,
                    max_duration_sec=self.duration,stop_on_return=self.stop_on_return,save_on_complete=self.save_on_complete)


def manual_assist(velocity,view,age,cfg):
    """Scale/stop manual input; do not choose an unexpected avoidance direction."""
    x,z=velocity
    if not (x or z):return (0.,0.),'IDLE'
    if not view or not view.get('valid') or not 0<=age<=cfg['scan_max_age_sec']:
        return (0.,0.),'MANUAL_LIDAR_NOT_READY'
    if x>0:
        x=min(x,braking_speed_cap(view['front_m'],age,cfg['stop_margin_m']))
    if z and not view['rotation_clear']:return (0.,0.),'MANUAL_TURN_BLOCKED'
    if cfg['vehicle']=='articulated' and z:
        if x<=0:return (0.,0.),'TRAILER_FORWARD_ARC_REQUIRED'
        key='arc_left_clear' if z>0 else 'arc_right_clear'
        if not view.get(key,False):return (0.,0.),'TRAILER_FORWARD_ARC_BLOCKED'
        z=max(-x/cfg['min_turn_radius_m'],min(z,x/cfg['min_turn_radius_m']))
    return (x,z),'MANUAL_SLOWED' if (x,z)!=velocity else 'CLEAR'


def nav2_assist(velocity,view,cfg,scan_age=0.):
    """Bound a Nav2 candidate and slow forward motion near the body corridor.

    Nav2 checks its oriented trajectory; a blocked full turning circle must not
    veto a valid turn. The caller has already checked scan age and hard stops.
    """
    legacy=len(velocity)==2
    x,y,z=(velocity[0],0.,velocity[1]) if legacy else velocity
    if not all(math.isfinite(v) for v in (x,y,z)):return (0.,0.) if legacy else (0.,0.,0.)
    reverse=cfg.get('reverse_enabled',False) and cfg.get('vehicle')=='rigid'
    x=max(-cfg.get('max_reverse_mps',.1),x) if reverse else max(0.,x)
    # DWB can add millimetres-per-second of reverse drift to a sideways
    # command.  Treat that as zero before the independent rear guard sees it;
    # meaningful reverse motion is still checked against the rear sector.
    if abs(x)<.005:x=0.
    # A rear/side point inside the full turning circle does not veto a clear
    # translation corridor. Keep the Nav2 translation and remove only yaw.
    if not view.get('rotation_clear',True):z=0.
    # A configured maximum is never permission to outrun sensor latency and
    # the .6 m/s^2 Nav2 deceleration profile. Bound each translation corridor.
    def braking_cap(gap):
        return braking_speed_cap(gap,scan_age,cfg['stop_margin_m'])
    longitudinal_gap=view.get('back_m',0.) if x<0 else view['front_m']
    x=math.copysign(min(abs(x),braking_cap(longitudinal_gap)),x)
    if y:
        cap=braking_cap(view.get('lateral_left_m' if y>0 else 'lateral_right_m',0.))
        y=math.copysign(min(abs(y),cap),y)
    scale=min(1.,cfg['speed_mps']/max(math.hypot(x,y),1e-9))
    result=(x*scale,y*scale,max(-cfg['angular_rps'],min(z,cfg['angular_rps'])))
    return (result[0],result[2]) if legacy else result
