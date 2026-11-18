"""Frontier goals use Humble Nav2; its velocities stay isolated until selected by MAR."""
import math,time
from pathlib import Path
from .contracts import StepError
from .footprint import cached_body_clear as body_clear


def frontier_replan_delay(cfg):
    # The active action replans at 1 Hz. Allow one full update after braking
    # vetoes its velocity before replacing the target.
    return max(1.5,cfg.get('frontier_blocked_replan_sec',1.5))


def translation_sweep_clear(snapshot,pose,view,footprint,margin,translation,zones=None):
    """Check a short fixed-yaw segment against observed grid and raw scan."""
    points=view.get('obstacle_points')
    dx,dy=translation;length=math.hypot(dx,dy)
    if points is None or not 0.<length<=.150000001:
        return False
    steps=max(1,math.ceil(length/.01));bound=length/steps/2.
    # Half-step padding covers the space between sampled body polygons.
    extent={name:value+margin+bound for name,value in footprint.items()}
    yaw=pose['yaw_rad'];c,s=math.cos(yaw),math.sin(yaw);path=[];padded_clear=True;initial_clear=None
    for i in range(steps+1):
        x,y=dx*i/steps,dy*i/steps
        clear=not any(-extent['rear']<=px-x<=extent['front'] and -extent['right']<=py-y<=extent['left'] for px,py in points)
        sample=dict(x_m=pose['x_m']+c*x-s*y,y_m=pose['y_m']+s*x+c*y,yaw_rad=yaw)
        clear=clear and body_clear(snapshot,sample,extent)['clear']
        if initial_clear is None:initial_clear=clear
        if not clear:
            if initial_clear:return False # not an existing overlap: ordinary collision rejection
            padded_clear=False
        path.append((sample['x_m'],sample['y_m'],yaw))
    if not padded_clear:
        # An existing padding overlap must not imprison a physically clear
        # chassis. Permit only a fixed-yaw retreat that leaves every initial
        # contact, never a rotation, physical overlap or new contact.
        from .cargo_recovery import padding_escape_clear
        obstacles=[(pose['x_m']+c*x-s*y,pose['y_m']+s*x+c*y) for x,y in points]
        if not padding_escape_clear(snapshot,path,footprint,obstacles,zones,padding=margin+bound):return False
        # Occupied-cell overlap must not hide unknown cells in the same body
        # check. Keep the full unknown/boundary reserve at every sample.
        raw=snapshot['data']
        class UnknownOnly:
            def __getitem__(self,index):
                value=raw[index]
                return 0 if 65<=value<=100 else value
        # A virtual view avoids copying a potentially large map in the lease loop.
        known={**snapshot,'data':UnknownOnly()}
        for x,y,a in path:
            if not body_clear(known,dict(x_m=x,y_m=y,yaw_rad=a),extent)['clear']:return False
        last=dict(x_m=path[-1][0],y_m=path[-1][1],yaw_rad=yaw)
        if not body_clear(snapshot,last,extent)['clear']:return False
        if any(-extent['rear']<=px-dx<=extent['front'] and -extent['right']<=py-dy<=extent['left'] for px,py in points):return False
    if zones is not None:
        try:zones.validate_path(path)
        except StepError:return False
    return True


def lateral_sweep_clear(snapshot,pose,view,footprint,margin,distance):
    return translation_sweep_clear(snapshot,pose,view,footprint,margin,(0.,distance))


def rotation_sweep_clear(snapshot,pose,view,footprint,margin,angle,zones=None):
    """Check every oriented body along an observation turn, including unknown."""
    points=view.get('obstacle_points') if view else None
    if not view or view.get('valid') is not True or points is None or not 0.<abs(angle)<=math.tau+1.e-8:
        return False
    radius=math.hypot(max(footprint['front'],footprint['rear'])+margin,
                      max(footprint['left'],footprint['right'])+margin)
    steps=max(1,math.ceil(radius*abs(angle)/.01));bound=radius*abs(angle)/steps/2.
    extent={name:value+margin+bound for name,value in footprint.items()};path=[]
    for i in range(steps+1):
        a=angle*i/steps;c,s=math.cos(a),math.sin(a)
        if any(-extent['rear']<=c*x+s*y<=extent['front'] and -extent['right']<=-s*x+c*y<=extent['left'] for x,y in points):return False
        sample={**pose,'yaw_rad':pose['yaw_rad']+a}
        if not body_clear(snapshot,sample,extent)['clear']:return False
        path.append((pose['x_m'],pose['y_m'],sample['yaw_rad']))
    if zones is not None:
        try:zones.validate_path(path)
        except StepError:return False
    return True


class MappingNavigator:
    def __init__(self,adapter,exploration):
        self.adapter=adapter;self.exploration=exploration;self.target=None;self.result=None;self.started=0.
        self.best_distance=math.inf;self.progress_at=0.;self.velocity_missing_at=None;self.progress_pose=None
        self.path=();self.side=None;self.side_pending=False;self.side_attempts=[];self.side_run=None
        self.velocity_blocked_at=None
        self.side_progress=None
        self.observation=None;self.observation_attempts=[];self.observation_failure=None
        self.lidar_reactive=None
        self.recovery_wait_at=None;self.recovery_reason=None
    @property
    def active(self):return self.target is not None or self.side is not None or self.side_pending or self.observation is not None
    async def stop(self):
        manual_active=self.side is not None or self.observation is not None
        self.side=None;self.side_pending=False
        self.side_progress=None
        self.observation=None
        self.velocity_blocked_at=None
        self.recovery_wait_at=None;self.recovery_reason=None
        if self.target is None and not manual_active:return
        try:
            if not await self.adapter.stop():raise StepError('GOAL_STOP_UNCONFIRMED')
        finally:self.target=None;self.result=None

    def note_velocity_limit(self,requested,applied,now):
        """A blocked translation replans; an ordinary in-place alignment does not."""
        wanted=math.hypot(*requested[:2])>.01
        blocked=math.hypot(*applied[:2])<=.005
        if self.target is not None and wanted and blocked:
            if self.velocity_blocked_at is None:self.velocity_blocked_at=now
        else:self.velocity_blocked_at=None

    def start_sideways(self,run,pose,now,snapshot,view,odom_speed):
        """Try one of eight bounded translations; yaw stays fixed until replanning."""
        motion=self.adapter.cfg.get('motion',{});cfg=self.exploration.cfg
        if cfg['vehicle']!='rigid' or motion.get('chassis_model')!='MENTORPI_M1_MECANUM':
            self.side_pending=False;self.recovery_reason='RECOVERY_UNSUPPORTED'
            return
        if self.side_run!=run:
            self.side_run=run;self.side_attempts=[];self.observation_attempts=[];self.observation_failure=None
            self.recovery_wait_at=None
        if self.target is not None or self.side is not None or self.observation is not None:return
        self.side_attempts=[p for p in self.side_attempts if now-p[2]<30.]
        x,y,yaw=pose['x_m'],pose['y_m'],pose['yaw_rad']
        odom=getattr(self.adapter,'odom_pose_sample',None)
        twist=getattr(self.adapter,'odom_twist_sample',None)
        if (not odom or not -.05<=now-odom[1]<=.35
                or not twist or not -.05<=now-twist[1]<=.35 or math.hypot(*twist[0][:2])>.02
                or abs(twist[0][2])>.03 or getattr(self.adapter,'motion_uncertain',False)
                or odom_speed.get('age_sec',math.inf)>.35 or abs(odom_speed.get('value',math.inf))>.02):
            if self.recovery_wait_at is None:self.recovery_wait_at=now
            self.side_pending=now-self.recovery_wait_at<2.
            self.recovery_reason='WAITING_FOR_RECOVERY_STOP' if self.side_pending else 'RECOVERY_STOP_TIMEOUT'
            return
        self.side_pending=False;self.recovery_wait_at=None
        if (not view or not view.get('valid')
                or view.get('nearest_lidar_m',0.)<=cfg['lidar_stop_distance_m']):
            self.recovery_reason='RECOVERY_SCAN_NOT_READY';return
        point=None
        if self.path:
            nearest=min(range(len(self.path)),key=lambda i:math.hypot(x-self.path[i][0],y-self.path[i][1]))
            point=next(((px,py) for px,py in self.path[nearest+1:] if math.hypot(px-x,py-y)>=.12),None)
        desired_yaw=math.atan2(point[1]-y,point[0]-x) if point else yaw
        # Reuse the vendor-style hysteretic side choice only as a preference.
        # MAR still checks the complete known map/scan sweep and owns velocity.
        if (cfg.get('frontier_lidar_avoidance',True)
                and all(key in view for key in ('front_m','left_m','right_m'))
                and view['front_m']<=cfg.get('turn_clearance_m',.2)):
            if self.lidar_reactive is None:
                from .lidar_reactive import LidarReactive
                self.lidar_reactive=LidarReactive()
            preference,reason=self.lidar_reactive.current(view,cfg,cfg.get('scan_max_age_sec',.4),now,run)
            length=math.hypot(*preference[:2])
            if reason=='LIDAR_SIDE_ESCAPE' and length>.005:
                direction=(preference[0]/length,preference[1]/length)
                if self.start_translation(pose,now,snapshot,view,odom[0],[direction]):return
            elif reason=='LIDAR_TURN_TO_CLEAR_SIDE' and preference[2]:
                desired_yaw=yaw+math.copysign(math.pi/2,preference[2])
        desired=(0.,0.)
        if point is not None:
            dx,dy=point[0]-x,point[1]-y;c,s=math.cos(yaw),math.sin(yaw)
            forward,side=c*dx+s*dy,-s*dx+c*dy;norm=math.hypot(forward,side)
            if norm>1.e-6:desired=(forward/norm,side/norm)
        diagonal=math.sqrt(.5)
        directions=[(1.,0.),(0.,1.),(0.,-1.),(-1.,0.),
                    (diagonal,diagonal),(diagonal,-diagonal),(-diagonal,diagonal),(-diagonal,-diagonal)]
        directions=[(a,b) for a,b in directions
                    if (a>=0. or motion.get('allow_reverse') is True and cfg.get('reverse_enabled'))
                    and (not b or motion.get('allow_strafe') is True)]
        # A missing/unreachable frontier path is not a missing escape direction.
        # Prefer increasing physical clearance from nearby scan returns, using
        # path alignment only as a tie-breaker. Sweeps still authorize motion.
        f=cfg['footprint']
        def gap(px,py):
            return math.hypot(max(-f['rear']-px,px-f['front'],0.),max(-f['right']-py,py-f['left'],0.))
        nearby=[(px,py) for px,py in view.get('obstacle_points',()) if gap(px,py)<.25]
        baseline=min((gap(px,py) for px,py in nearby),default=0.)
        length=cfg.get('frontier_escape_min_m',.1)
        def rank(d):
            gain=min((gap(px-d[0]*length,py-d[1]*length) for px,py in nearby),default=baseline)-baseline
            return (-gain,-(d[0]*desired[0]+d[1]*desired[1]))
        directions.sort(key=rank)
        # Crossing a doorway with a fixed heading generally needs less room
        # than turning the entire chassis. Observe by rotating only if none
        # of the clearance-checked translations is available.
        if not self.start_translation(pose,now,snapshot,view,odom[0],directions):
            self.start_observation(pose,now,snapshot,view,odom[0],desired_yaw)

    def start_translation(self,pose,now,snapshot,view,odom,directions):
        cfg=self.exploration.cfg;motion=self.adapter.cfg.get('motion',{})
        if len(self.side_attempts)>=4:
            self.recovery_reason='RECOVERY_ATTEMPT_BUDGET';return False
        x,y=pose['x_m'],pose['y_m']
        margin=cfg['stop_margin_m']
        for direction in directions:
            if direction[0]<0. and not (motion.get('allow_reverse') is True and cfg.get('reverse_enabled')):continue
            if direction[1] and motion.get('allow_strafe') is not True:continue
            if any(math.hypot(x-px,y-py)<.20 and a*direction[0]+b*direction[1]>.99
                   for px,py,_,a,b in self.side_attempts):continue
            for length in (cfg.get('frontier_escape_max_m',.15),cfg.get('frontier_escape_min_m',.10)):
                translation=tuple(v*length for v in direction)
                if translation_sweep_clear(snapshot,pose,view,cfg['footprint'],margin,translation,
                                           getattr(self.adapter,'zones',None)):
                    self.side=(now,dict(pose),odom,translation)
                    self.side_progress=(0.,now)
                    self.side_attempts.append((x,y,now,*direction))
                    self.velocity_blocked_at=None
                    self.recovery_reason='MAPPING_REPOSITIONING'
                    return True
        self.recovery_reason='RECOVERY_NO_CLEAR_TRANSLATION'
        return False

    def start_observation(self,pose,now,snapshot,view,odom,desired_yaw):
        cfg=self.exploration.cfg
        if not cfg.get('frontier_observation_rotation',True):return False
        self.observation_attempts=[p for p in self.observation_attempts if now-p[2]<30.]
        if any(math.hypot(pose['x_m']-x,pose['y_m']-y)<.25 for x,y,_ in self.observation_attempts):return False
        failed=self.observation_failure
        if failed is not None and now-failed[2]<1. and math.hypot(pose['x_m']-failed[0],pose['y_m']-failed[1])<.05:return False
        maximum=min(math.tau,cfg.get('frontier_observation_max_rad',math.tau))
        delta=math.atan2(math.sin(desired_yaw-pose['yaw_rad']),math.cos(desired_yaw-pose['yaw_rad']))
        sign=(1. if delta>0 else -1.) if abs(delta)>.15 else (1. if view.get('left_m',0.)>=view.get('right_m',0.) else -1.)
        candidates=[]
        if abs(delta)>.15:candidates.append(math.copysign(min(abs(delta),maximum),delta))
        candidates.append(sign*maximum)
        candidates.extend((sign*min(math.pi/2,maximum),-sign*min(math.pi/2,maximum)))
        zones=getattr(self.adapter,'zones',None)
        for angle in candidates:
            if abs(angle)<.15:continue
            if rotation_sweep_clear(snapshot,pose,view,cfg['footprint'],cfg['stop_margin_m'],angle,zones):
                self.observation=dict(started=now,origin=dict(pose),odom_origin=odom,
                    previous_yaw=odom[2],previous_at=now,accumulated=0.,target=angle,best=0.,progress_at=now)
                self.observation_attempts.append((pose['x_m'],pose['y_m'],now));self.observation_failure=None
                self.velocity_blocked_at=None
                self.recovery_reason='MAPPING_OBSERVING_ROTATION'
                return True
        self.observation_failure=(pose['x_m'],pose['y_m'],now)
        return False

    def observation_velocity(self,pose,now,snapshot,view):
        state=self.observation;cfg=self.exploration.cfg
        def stop():self.observation=None;return 0.,0.,0.
        if state is None:return 0.,0.,0.
        odom=getattr(self.adapter,'odom_pose_sample',None);twist=getattr(self.adapter,'odom_twist_sample',None)
        if (not odom or not -.05<=now-odom[1]<=.35 or not twist or not -.05<=now-twist[1]<=.35
                or math.hypot(*twist[0][:2])>.03 or getattr(self.adapter,'motion_uncertain',False)
                or not view or not view.get('valid') or view.get('nearest_lidar_m',0.)<=cfg['lidar_stop_distance_m']):return stop()
        origin=state['origin'];o=state['odom_origin'];position=odom[0]
        wrap=lambda a:math.atan2(math.sin(a),math.cos(a))
        if (math.hypot(position[0]-o[0],position[1]-o[1])>.025
                or math.hypot(pose['x_m']-origin['x_m'],pose['y_m']-origin['y_m'])>.04
                or abs(wrap((pose['yaw_rad']-origin['yaw_rad'])-(position[2]-o[2])))>.15):return stop()
        step=wrap(position[2]-state['previous_yaw']);dt=max(0.,now-state['previous_at'])
        if abs(step)>.15+cfg['angular_rps']*dt:return stop()
        state['accumulated']+=step;state['previous_yaw']=position[2];state['previous_at']=now
        sign=math.copysign(1.,state['target']);progress=sign*state['accumulated'];remaining=abs(state['target'])-progress
        if progress<-.03 or remaining<=.03:return stop()
        cap=min(.8,cfg['angular_rps'])
        if now-state['started']>min(90.,3.+1.5*abs(state['target'])/max(.1,cap)):return stop()
        if progress>=state['best']+.02:state['best']=progress;state['progress_at']=now
        elif now-state['progress_at']>1.5:return stop()
        rate=min(cap,max(.1,remaining))
        horizon=cfg.get('scan_max_age_sec',.4)+.35+rate/2.4
        angle=sign*min(remaining,rate*horizon)
        if not rotation_sweep_clear(snapshot,pose,view,cfg['footprint'],cfg['stop_margin_m'],angle,
                                    getattr(self.adapter,'zones',None)):return stop()
        return 0.,0.,sign*rate

    def sideways_velocity(self,pose,now,snapshot,view):
        if self.observation is not None:return self.observation_velocity(pose,now,snapshot,view)
        if self.side is None:return 0.,0.,0.
        started,origin,odom_origin,translation=self.side
        odom=getattr(self.adapter,'odom_pose_sample',None)
        def stop():
            self.side=None
            self.side_progress=None
            return 0.,0.,0.
        if not odom or not -.05<=now-odom[1]<=.35:return stop()
        dx,dy=odom[0][0]-odom_origin[0],odom[0][1]-odom_origin[1]
        c,s=math.cos(odom_origin[2]),math.sin(odom_origin[2])
        forward,side=c*dx+s*dy,-s*dx+c*dy
        map_dx,map_dy=pose['x_m']-origin['x_m'],pose['y_m']-origin['y_m']
        map_yaw=origin['yaw_rad'];odom_yaw=odom[0][2]-odom_origin[2]
        map_forward=math.cos(map_yaw)*map_dx+math.sin(map_yaw)*map_dy
        map_side=-math.sin(map_yaw)*map_dx+math.cos(map_yaw)*map_dy
        yaw_error=pose['yaw_rad']-map_yaw
        distance=math.hypot(*translation);ux,uy=translation[0]/distance,translation[1]/distance
        progress=forward*ux+side*uy;cross=-forward*uy+side*ux
        if (abs(cross)>.025 or math.hypot(map_forward-forward,map_side-side)>.04 or progress<-.001
                or abs(math.atan2(math.sin(yaw_error),math.cos(yaw_error)))>.10
                or abs(math.atan2(math.sin(odom_yaw),math.cos(odom_yaw)))>.10):return stop()
        remaining=distance-progress
        if remaining<=.008:return stop()
        cfg=self.exploration.cfg
        speed_cap=min(.045,cfg['speed_mps'])
        if ux<0:
            speed_cap=min(speed_cap,cfg.get('max_reverse_mps',.1)/abs(ux),
                          self.adapter.cfg['motion'].get('max_reverse_speed_mps',.1)/abs(ux))
        timeout=min(12.,2.+distance/max(.005,min(.02,speed_cap)))
        if now-started>timeout:return stop()
        best,at=self.side_progress if self.side_progress is not None else (0.,started)
        if progress>=best+.005:self.side_progress=(progress,now)
        elif now-at>1.5:return stop()
        if (not view or not view.get('valid')
                or view.get('nearest_lidar_m',0.)<=cfg['lidar_stop_distance_m']):return stop()
        if not translation_sweep_clear(snapshot,pose,view,cfg['footprint'],cfg['stop_margin_m'],
                                       (ux*remaining,uy*remaining),getattr(self.adapter,'zones',None)):
            return stop()
        speed=min(speed_cap,max(.02,remaining))
        return ux*speed,uy*speed,0.
    async def tick(self,plan,pose,now):
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        if self.result is not None and self.result.done():
            terminal=self.result.result();target=self.target
            if not await self.adapter.stop():raise StepError('GOAL_STOP_UNCONFIRMED')
            self.target=None;self.result=None
            self.velocity_blocked_at=None
            self.exploration.exclude_goal(target,now,ttl=5. if terminal.status==4 else 20.,
                                          radius=.12 if terminal.status==4 else .25)
            self.side_pending=terminal.status==6
            return (0.,0.,0.),'OBSERVING_FRONTIER' if terminal.status==4 else 'REPLANNING_UNREACHABLE_FRONTIER'
        if self.target is None:
            goal=plan['goal']
            if math.hypot(goal['x_m']-pose['x_m'],goal['y_m']-pose['y_m'])<.06:
                self.exploration.exclude_goal(goal,now,ttl=5.,radius=.08);return (0.,0.,0.),'OBSERVING_FRONTIER'
            if not self.adapter.nav.server_is_ready():raise StepError('NAV2_MAPPING_NOT_READY')
            msg=PoseStamped();msg.header.frame_id=self.adapter.cfg['ros']['map_frame'];msg.header.stamp=self.adapter.node.get_clock().now().to_msg()
            msg.pose.position.x=float(goal['x_m']);msg.pose.position.y=float(goal['y_m'])
            msg.pose.orientation.z=math.sin(goal['yaw_rad']/2);msg.pose.orientation.w=math.cos(goal['yaw_rad']/2)
            request=NavigateToPose.Goal();request.pose=msg
            request.behavior_tree=str(Path(__file__).resolve().parents[1]/'config/navigate_no_recovery.xml')
            self.path=tuple(plan.get('path',()))
            self.adapter.clear_navigation_velocity()
            sent=self.adapter.nav.send_goal_async(request)
            try:handle=await self.adapter.future(sent,timeout=2)
            except BaseException:sent.add_done_callback(self.adapter._late_cancel);raise
            if not handle.accepted:
                self.exploration.exclude_goal(goal,now,ttl=20.,radius=.25)
                self.side_pending=True
                return (0.,0.,0.),'REPLANNING_REJECTED_FRONTIER'
            self.adapter.goal=handle;self.target=dict(goal);self.result=handle.get_result_async();self.started=now
            self.recovery_reason=None
            self.velocity_blocked_at=None
            self.path=tuple(plan.get('path',()))
            self.best_distance=math.hypot(goal['x_m']-pose['x_m'],goal['y_m']-pose['y_m']);self.progress_at=now;self.velocity_missing_at=None
            self.progress_pose=dict(pose)
        distance=math.hypot(self.target['x_m']-pose['x_m'],self.target['y_m']-pose['y_m'])
        # A path around a wall can initially lead away from the goal. Measure
        # actual translation as well as goal distance; retain a total timeout
        # so oscillation cannot keep an unreachable goal alive indefinitely.
        moved=self.progress_pose is not None and math.hypot(pose['x_m']-self.progress_pose['x_m'],pose['y_m']-self.progress_pose['y_m'])>=.05
        if distance<self.best_distance-.04 or moved:
            self.best_distance=min(self.best_distance,distance);self.progress_at=now;self.progress_pose=dict(pose)
        cfg=getattr(self.exploration,'cfg',{})
        if (self.velocity_blocked_at is not None
                and now-self.velocity_blocked_at>=frontier_replan_delay(cfg)):
            target=dict(self.target);await self.stop();self.exploration.exclude_goal(target,now,ttl=10.,radius=.15)
            self.side_pending=True
            return (0.,0.,0.),'REPLANNING_BLOCKED_FRONTIER'
        if (now-self.progress_at>cfg.get('frontier_stall_sec',8.)
                or now-self.started>cfg.get('frontier_goal_timeout_sec',60.)):
            target=dict(self.target);await self.stop();self.exploration.exclude_goal(target,now,ttl=20.,radius=.25)
            self.side_pending=True
            return (0.,0.,0.),'REPLANNING_STALLED_FRONTIER'
        velocity,fresh=self.adapter.navigation_velocity()
        if not fresh:
            if self.velocity_missing_at is None:self.velocity_missing_at=now
            if now-self.velocity_missing_at>3. and now-self.started>5.:
                target=dict(self.target);await self.stop();self.exploration.exclude_goal(target,now,ttl=20.,radius=.25)
                return (0.,0.,0.),'REPLANNING_STALE_NAV2'
            return (0.,0.,0.),'WAITING_FOR_NAV2_VELOCITY'
        self.velocity_missing_at=None
        return velocity,'FOLLOW_FRONTIER_NAV2' if fresh else 'WAITING_FOR_NAV2_VELOCITY'
