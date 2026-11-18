"""Bounded, collision-checked repositioning inside MOVE, never during preview."""
import asyncio
import math
import time
from .contracts import StepError
from .footprint import body_clear, cached_body_clear, NAVIGATION_PADDING_M


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


def turn_options(current,target):
    delta=wrap(target-current)
    return (0.,) if abs(delta)<=.10 else (delta,delta-math.copysign(2*math.pi,delta))


def turn_in_direction(current,target,preferred_delta):
    short=wrap(target-current)
    if abs(short)<=.10 or preferred_delta==0 or short*preferred_delta>=0:return short
    return short-math.copysign(2*math.pi,short)


def blocked_motion_reason(state):
    gate=state.get('gate',{}).get('value',{})
    if isinstance(gate,str):
        import json
        try:gate=json.loads(gate)
        except ValueError:return None
    if not isinstance(gate,dict):return None
    reason=gate.get('reason')
    if reason not in ('TURN_ENVELOPE_BLOCKED','BRAKING_OBSTACLE','REAR_OBSTACLE','LIDAR_BRAKING_STOP'):return None
    applied=gate.get('applied_velocity')
    if not isinstance(applied,(list,tuple)) or len(applied)!=3:return None
    return reason if all(type(v) in (int,float) and abs(v)<.005 for v in applied) else None


def maneuver_points(origin,distance,yaw,lateral=0.,turn_delta=None,extended=False):
    max_axis,max_length=(.200000001,.200000001) if extended else (.100000001,.141422)
    if not (abs(distance)<=max_axis and abs(lateral)<=max_axis and math.hypot(distance,lateral)<=max_length):
        raise StepError('CARGO_ESCAPE_DISTANCE_INVALID')
    x,y,a=origin['x_m'],origin['y_m'],origin['yaw_rad']
    end=(x-distance*math.cos(a)-lateral*math.sin(a),y-distance*math.sin(a)+lateral*math.cos(a),a)
    n=max(1,math.ceil(math.hypot(distance,lateral)/.01))
    points=[(x+(end[0]-x)*i/n,y+(end[1]-y)*i/n,a) for i in range(n+1)]
    delta=wrap(yaw-a) if turn_delta is None else turn_delta
    if not math.isfinite(delta) or abs(delta)>2*math.pi+.001:raise StepError('CARGO_ESCAPE_TURN_INVALID')
    n=max(1,math.ceil(abs(delta)/.03))
    points.extend((end[0],end[1],a+delta*i/n) for i in range(1,n+1))
    return points




def maneuver_clear(grid,points,extent,obstacles,zones=None):
    radius=math.hypot(max(extent['front'],extent['rear']),max(extent['left'],extent['right']))
    f={k:v+NAVIGATION_PADDING_M+max(.005,.015*radius) for k,v in extent.items()}
    for x,y,a in points:
        if not cached_body_clear(grid,dict(x_m=x,y_m=y,yaw_rad=a),f)['clear']:return False
        c,s=math.cos(a),math.sin(a)
        for ox,oy in obstacles:
            dx,dy=ox-x,oy-y;rx,ry=c*dx+s*dy,-s*dx+c*dy
            if -f['rear']<=rx<=f['front'] and -f['right']<=ry<=f['left']:return False
    if zones is not None:
        try:zones.validate_path(points)
        except StepError:return False
    return True


def padding_escape_clear(grid,points,extent,obstacles,zones=None,*,padding=None):
    """Leave an existing padding overlap, never a physical/map/keepout collision.

    Only translation away from every initial contact is allowed until the full
    padded body is clear. It cannot skim toward a different wall or rotate the
    cargo through the padding. Unknown cells and map edges are not relaxed.
    """
    from .navigation_zones import body_gap
    from .footprint import overlap
    pad=NAVIGATION_PADDING_M+.005 if padding is None else padding
    if type(pad) not in (int,float) or not math.isfinite(pad) or pad<0:return False
    padded={k:v+pad for k,v in extent.items()}
    radius=math.hypot(max(extent['front'],extent['rear'])+pad,max(extent['left'],extent['right'])+pad)
    r=grid['resolution'];ox,oy,a=grid['origin'];c,s=math.cos(a),math.sin(a)
    local=[((x-ox)*c+(y-oy)*s, -(x-ox)*s+(y-oy)*c) for x,y,_ in points]
    bounds=(math.floor((min(x for x,y in local)-radius)/r),math.ceil((max(x for x,y in local)+radius)/r),
            math.floor((min(y for x,y in local)-radius)/r),math.ceil((max(y for x,y in local)+radius)/r))
    polygons=[]
    for row in range(max(0,bounds[2]),min(grid['height'],bounds[3]+1)):
        for col in range(max(0,bounds[0]),min(grid['width'],bounds[1]+1)):
            value=grid['data'][row*grid['width']+col]
            if 0<=value<=25:continue
            poly=[(ox+c*x*r-s*y*r,oy+s*x*r+c*y*r) for x,y in
                  ((col,row),(col+1,row),(col+1,row+1),(col,row+1))]
            polygons.append(poly)
    polygons.extend([[(x,y)] for x,y in obstacles])
    # Padding may already overlap an obstacle at the starting pose. Sampled
    # distances alone cannot prove that a thin obstacle misses a body corner
    # between samples. The convex hull of this fixed-yaw translation's body
    # corners covers the physical body continuously, without extra samples.
    # Later turns retain the ordinary padded checks once the overlap is left.
    vertices=[];initial_yaw=points[0][2]
    for x,y,yaw in points:
        if abs(wrap(yaw-initial_yaw))>1e-6:break
        ca,sa=math.cos(yaw),math.sin(yaw)
        vertices.extend((x+ca*dx-sa*dy,y+sa*dx+ca*dy) for dx,dy in
                        ((extent['front'],extent['left']),(-extent['rear'],extent['left']),
                         (-extent['rear'],-extent['right']),(extent['front'],-extent['right'])))
    vertices=sorted(set(vertices))
    def half_hull(sequence):
        result=[]
        for p in sequence:
            while len(result)>1:
                a,b=result[-2:]
                if (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])>0.:break
                result.pop()
            result.append(p)
        return result
    swept=half_hull(vertices)[:-1]+half_hull(reversed(vertices))[:-1]
    if any(overlap(swept,polygon) for polygon in polygons):return False
    initial=None;previous=None;clean=False;initial_yaw=points[0][2]
    for x,y,yaw in points:
        pose=dict(x_m=x,y_m=y,yaw_rad=yaw)
        physical=cached_body_clear(grid,pose,extent)
        if not physical['clear']:return False
        body=cached_body_clear(grid,pose,padded)
        if not body['clear'] and body['kind']!='occupied':return False
        near={i:body_gap(pose,extent,p) for i,p in enumerate(polygons)
              if min(math.hypot(px-x,py-y) for px,py in p)<=radius+r*1.42}
        if any(gap<=.001 for gap in near.values()):return False
        contacts={i for i in near if body_gap(pose,padded,polygons[i])<=1e-8}
        if initial is None:
            initial=contacts;previous={i:near[i] for i in initial}
            if not initial:return False # ordinary full-padding validation owns this case
        if contacts:
            if clean or not contacts<=initial or abs(wrap(yaw-initial_yaw))>1e-6:return False
            for i in initial:
                gap=near.get(i,math.inf)
                if gap+1e-6<previous[i]:return False
                previous[i]=gap
        else:clean=True
    if not clean:return False
    if zones is not None:
        try:zones.validate_path(points)
        except StepError:return False
    return True


class CargoRecovery:
    async def _wait_localization_recovery(self,endpoint_id,reason,seconds,attempt):
        """Pause this accepted MOVE; keep its identity/deadline and never reset AMCL."""
        if not await self.stop():raise StepError('GOAL_STOP_UNCONFIRMED')
        started=time.monotonic();deadline=started+seconds
        self.navigation_localization_wait=dict(since=started,reason=reason,attempt=attempt)
        self.node.get_logger().warning('NAV_WAIT_LOCALIZATION endpoint='+endpoint_id+
            ' reason='+reason+' attempt='+str(attempt)+' wait_sec='+str(seconds))
        try:
            while True:
                self.hold_navigation_velocity();self._check_graph()
                state=self.telemetry()
                # Frozen map/session/profile/revision remain authoritative.
                # A missing quality sample can recover; identity changes cannot.
                ready=self._navigation_context(allow_localization_pause=True,state=state)
                if (ready and state.get('map_pose',{}).get('fresh')
                        and self.health(state).get('sensors_ready')):
                    self.node.get_logger().info('NAV_LOCALIZATION_RECOVERED endpoint='+endpoint_id+
                        ' elapsed_sec='+str(round(time.monotonic()-started,2)))
                    return
                self.navigation_diagnostic(state,'WAIT_LOCALIZATION')
                if time.monotonic()>=deadline:raise StepError('LOCALIZATION_RECOVERY_TIMEOUT')
                await asyncio.sleep(.1)
        finally:self.navigation_localization_wait=None

    async def _retry_localization(self,action,p,endpoint_id,meta,replan=False):
        """No new BE command/ACK, step replay or hardware action during recovery."""
        runtime=getattr(self,'worker',{}).get('navigation_runtime',{})
        seconds=runtime.get('localization_wait_sec',30.)
        limit=runtime.get('localization_retry_count',2)
        if (type(seconds) not in (int,float) or not math.isfinite(seconds) or not .5<=seconds<=60
                or type(limit) is not int or not 0<=limit<=3):
            raise StepError('INVALID_LOCALIZATION_RECOVERY_CONFIG')
        needs_plan=False
        while True:
            try:
                if needs_plan:await self._plan_with_cargo_recovery_once(p,endpoint_id,meta)
                return await action()
            except StepError as exc:
                if exc.code not in ('LOCALIZATION_LOST','LOCALIZATION_LOST_AT_GOAL',
                                    'LOCALIZATION_NOT_READY','MAP_TF_NOT_READY','SENSORS_NOT_READY'):raise
                used=getattr(self,'localization_recovery_attempts',0)
                if used>=limit:
                    raise StepError('LOCALIZATION_RECOVERY_EXHAUSTED',
                                    'LOCALIZATION_RECOVERY_EXHAUSTED: '+exc.code) from exc
                self.localization_recovery_attempts=used+1
                await self._wait_localization_recovery(endpoint_id,exc.code,seconds,used+1)
                # The previous Nav2 goal was canceled with stop confirmation.
                # Recompute from the current pose, not from an old motor command.
                needs_plan=replan

    async def _wait_obstacle_path(self,p,endpoint_id,meta,blocked=False):
        """Wait/replan on live RAM costmaps inside the original MOVE deadline."""
        runtime=getattr(self,'worker',{}).get('navigation_runtime',{})
        seconds=runtime.get('obstacle_wait_sec',15.)
        interval=runtime.get('obstacle_retry_interval_sec',1.)
        if (type(seconds) not in (int,float) or not math.isfinite(seconds) or not 0<=seconds<=30
                or type(interval) not in (int,float) or not .1<=interval<=2):
            raise StepError('INVALID_OBSTACLE_WAIT_CONFIG')
        previous=getattr(self,'navigation_obstacle_wait',None)
        deadline=None
        if blocked:
            deadline=time.monotonic()+seconds
            self.navigation_obstacle_wait=dict(since=time.monotonic(),reason='WAITING_FOR_PATH')
        try:
            if blocked:await asyncio.sleep(min(.5,seconds))
            while True:
                try:
                    if deadline is None:return await self._plan(p,endpoint_id,meta)
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise StepError('PATH_UNAVAILABLE')
                    return await asyncio.wait_for(self._plan(p,endpoint_id,meta),remaining)
                except asyncio.TimeoutError:
                    raise StepError('PATH_UNAVAILABLE') from None
                except StepError as error:
                    if error.code!='PATH_UNAVAILABLE':raise
                    if deadline is None:
                        deadline=time.monotonic()+seconds
                        self.navigation_obstacle_wait=dict(since=time.monotonic(),reason='WAITING_FOR_PATH')
                        self.node.get_logger().warning('NAV_WAITING_FOR_PATH wait_sec='+str(seconds))
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise
                    self.lease_message(False)
                    await asyncio.sleep(min(interval,remaining))
        finally:
            self.navigation_obstacle_wait=previous

    async def _move_with_path_recovery(self,p,endpoint_id,meta):
        return await self._retry_localization(
            lambda:self._move_with_path_recovery_once(p,endpoint_id,meta),p,endpoint_id,meta,replan=True)

    async def _move_with_path_recovery_once(self,p,endpoint_id,meta):
        """Bounded local retries for obstacles and one small arrival correction.

        Cancellation, sensor loss and an unconfirmed stop never enter the
        obstacle branch. Localization has its own stopped retry above.
        The same step deadline and endpoint remain in force;
        there is no new BE job, AMCL reset or replay of an earlier motor command.
        """
        obstacle_retries=0;arrival_corrected=False
        while True:
            try:return await self._move_one(p,endpoint_id)
            except StepError as exc:
                if exc.code not in ('NAV_ABORTED','NAV_GATE_BLOCKED','NAV2_COMMAND_STALE',
                                    'ENDPOINT_POSITION_NOT_CONFIRMED','CARGO_REVERSE_BLOCKED'):raise
                failure_code=exc.code
                arrival_correction=exc.code=='ENDPOINT_POSITION_NOT_CONFIRMED'
                if arrival_correction:
                    if arrival_corrected:raise
                    arrival_corrected=True
                else:
                    if obstacle_retries>=2:raise
                    obstacle_retries+=1
            await self._wait_stationary()
            state=self.telemetry()
            if not self._navigation_context(allow_localization_pause=True,state=state):
                raise StepError('LOCALIZATION_LOST')
            if arrival_correction:
                pose=state.get('map_pose',{})
                if not pose.get('fresh'):raise StepError('LOCALIZATION_LOST')
                current=pose['value'];target=self.cfg['endpoints'][endpoint_id]
                error=math.hypot(current['x_m']-target['x_m'],current['y_m']-target['y_m'])
                # Never recover a large pose jump or declare an unreached
                # endpoint successful merely because Nav2 returned success.
                if error>.25:raise StepError('ENDPOINT_POSITION_NOT_CONFIRMED')
                self.node.get_logger().warning('ARRIVAL_POSITION_REPLANNING endpoint='+endpoint_id+' error_m='+str(error)+' attempt=1')
            else:self.node.get_logger().warning('NAVIGATION_BLOCKED_REPLANNING endpoint='+endpoint_id+' attempt='+str(obstacle_retries)+' reason='+failure_code)
            await self._prerequisites(p)
            # RPP can reject a colliding arc before emitting any velocity: the
            # independent gate then correctly reports CLEAR, not BLOCKED. A
            # global replan from that same pose alone can repeat the deadlock.
            # Reposition once, including on a bare Mecanum chassis, only after
            # cancelling/stopping the old goal and checking a virtual route.
            runtime=getattr(self,'worker',{}).get('navigation_runtime',{})
            escape=(not arrival_correction and runtime.get('map_yaml')
                    and runtime.get('geometry',{}).get('profile') in ('bare','cargo')
                    and not getattr(self,'cargo_escape_used',False))
            if arrival_correction:
                await self._plan_with_cargo_recovery(p,endpoint_id,meta)
            elif failure_code=='CARGO_REVERSE_BLOCKED':
                # The final reposition was already stopped. Retry the planned
                # approach from the actual pose, without another escape.
                await self._wait_obstacle_path(p,endpoint_id,meta,blocked=True)
            elif escape and (failure_code in ('NAV_GATE_BLOCKED','NAV_ABORTED') or obstacle_retries>=2):
                try:await self._plan_with_cargo_recovery(p,endpoint_id,meta,force_escape=True)
                except StepError as error:
                    if error.code not in ('NAV_RECOVERY_NO_CLEAR_MANEUVER','CARGO_REVERSE_BLOCKED'):raise
                    # A passing person can disappear on the next costmap
                    # update. Never force a maneuver merely to avoid failure.
                    self.node.get_logger().warning(error.code+'; replan without repositioning')
                    if error.code=='CARGO_REVERSE_BLOCKED':await self._wait_stationary()
                    await self._wait_obstacle_path(p,endpoint_id,meta,blocked=True)
            else:
                try:await self._wait_obstacle_path(p,endpoint_id,meta,blocked=True)
                except StepError as error:
                    if error.code!='PATH_UNAVAILABLE' or not escape:raise
                    await self._plan_with_cargo_recovery(p,endpoint_id,meta,force_escape=True)

    async def _align_arrival_heading(self,p,endpoint_id):
        await self._cargo_prepare_final_heading(p,endpoint_id)
        try:
            return await self._finish_heading(p,endpoint_id)
        except StepError as exc:
            runtime=getattr(self,'worker',{}).get('navigation_runtime',{})
            if (exc.code!='FINAL_ALIGNMENT_BLOCKED' or runtime.get('geometry',{}).get('profile')!='cargo'
                    or getattr(self,'cargo_arrival_reposition_used',False)):
                raise
        # Nav2 has stopped the previous Spin. Confirm a fresh arrival pose
        # before looking for a short translation and one alternate turn.
        await self._wait_stationary()
        await self._wait_arrival_pose(endpoint_id)
        self.node.get_logger().warning('CARGO_FINAL_ALIGNMENT_REPOSITION endpoint='+endpoint_id)
        await self._cargo_prepare_final_heading(p,endpoint_id,require_reposition=True)
        return await self._finish_heading(p,endpoint_id)

    async def _cargo_prepare_final_heading(self,p,endpoint_id,require_reposition=False):
        runtime=getattr(self,'worker',{}).get('navigation_runtime',{})
        if runtime.get('geometry',{}).get('profile')!='cargo':return
        self.cargo_final_turn_sign=None
        _,origin=self._cargo_pose();target=self.cfg['endpoints'][endpoint_id]
        if abs(wrap(target['yaw_rad']-origin['yaw_rad']))<=.10:return
        for distance,lateral in self._cargo_offsets():
            if require_reposition and not (distance or lateral):continue
            if (distance or lateral) and getattr(self,'cargo_arrival_reposition_used',False):continue
            for delta in turn_options(origin['yaw_rad'],target['yaw_rad']):
                points=maneuver_points(origin,distance,target['yaw_rad'],lateral,delta)
                if math.hypot(points[-1][0]-target['x_m'],points[-1][1]-target['y_m'])>.10:continue
                clear,_=self._cargo_clear(origin,distance,target['yaw_rad'],lateral=lateral,turn_delta=delta)
                if not clear:continue
                self.cargo_final_turn_sign=math.copysign(1.,delta)
                if distance or lateral:
                    self.cargo_arrival_reposition_used=True
                    self.node.get_logger().info('CARGO_FINAL_REPOSITION distance_m='+str(distance)+' lateral_m='+str(lateral))
                    await self._cargo_reposition(distance,lateral,p)
                    await self._wait_stationary()
                    _,actual=self._cargo_pose()
                    if abs(wrap(target['yaw_rad']-actual['yaw_rad']))<=.10:return
                    # The real stopped pose can differ from the simulated escape.
                    for actual_delta in turn_options(actual['yaw_rad'],target['yaw_rad']):
                        clear,_=self._cargo_clear(actual,0.,target['yaw_rad'],lateral=0.,turn_delta=actual_delta)
                        if clear:
                            self.cargo_final_turn_sign=math.copysign(1.,actual_delta)
                            return
                    raise StepError('FINAL_ALIGNMENT_BLOCKED')
                return
        raise StepError('FINAL_ALIGNMENT_BLOCKED')

    def _cargo_offsets(self):
        motion=self.cfg['motion']
        mecanum=motion.get('chassis_model')=='MENTORPI_M1_MECANUM'
        strafe=mecanum and motion.get('allow_strafe')
        reverse=mecanum and motion.get('allow_reverse')
        directions=[(0.,1.),(0.,-1.)] if strafe else []
        if reverse:directions.append((1.,0.))
        directions.append((-1.,0.))
        if strafe:
            diagonal=math.sqrt(.5)
            for longitudinal in ((-diagonal,diagonal) if reverse else (-diagonal,)):
                directions.extend([(longitudinal,diagonal),(longitudinal,-diagonal)])
        # Visit every allowed direction at 6 cm before trying 10 cm. Both
        # diagonal components share the same distance budget as axial moves.
        return [(0.,0.)]+[(r*x,r*y) for r in (.06,.10) for x,y in directions]

    def _cargo_pose(self):
        state=self.telemetry();self._check_graph()
        if not self._navigation_context(allow_localization_pause=True,state=state):raise StepError('LOCALIZATION_NOT_READY')
        if not state.get('map_pose',{}).get('fresh'):raise StepError('LOCALIZATION_LOST')
        if state.get('gate',{}).get('age_sec',99.)>.6:raise StepError('SAFETY_GATE_LOST')
        return state,state['map_pose']['value']

    def _cargo_saved_grid(self):
        from pathlib import Path
        from .map_tools import load_trinary_grid
        path=self.worker['navigation_runtime']['map_yaml'];key=(path,Path(path).stat().st_mtime_ns)
        cached=getattr(self,'cargo_grid',None)
        if cached is None or cached[0]!=key:
            cached=(key,load_trinary_grid(path))
            self.cargo_grid=cached
        return cached[1]

    def _cargo_map_allows_turn(self,pose,yaw_rate):
        # Match the rotational stopping horizon used by the braking gate.
        # This checks static cells that the live scan may not currently see.
        horizon=.35+abs(yaw_rate)/2.4
        sweep=maneuver_points(pose,0.,pose['yaw_rad'],turn_delta=yaw_rate*horizon)
        extent=self.worker['mapping']['autonomy']['footprint']
        return maneuver_clear(self._cargo_saved_grid(),sweep,extent,())

    def _cargo_clear(self,origin,distance,yaw,lateral=0.,turn_delta=None,extended=False):
        from .autonomous_mapping import scan_view
        from .geometry_diagnostics import planar_transform
        from rclpy.time import Time
        sample=getattr(self,'scan_message_sample',None)
        if sample is None:raise StepError('CARGO_ESCAPE_SCAN_NOT_READY')
        scan=sample[0]
        age=(self.node.get_clock().now().nanoseconds-(scan.header.stamp.sec*1000000000+scan.header.stamp.nanosec))/1e9
        motion=self.cfg['motion'];profile=motion['lidar_braking']
        if not 0<=age<=profile['scan_max_age_sec']:raise StepError('CARGO_ESCAPE_SCAN_NOT_READY')
        try:
            transform=(0.,0.,0.,1) if scan.header.frame_id==self.cfg['ros']['base_frame'] else planar_transform(
                self.tf_buffer.lookup_transform(self.cfg['ros']['base_frame'],scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform)
            view=scan_view(scan,transform,profile)
        except Exception as exc:raise StepError('CARGO_ESCAPE_SCAN_NOT_READY') from exc
        if not view.get('valid') or 'obstacle_points' not in view:raise StepError('CARGO_ESCAPE_SCAN_NOT_READY')
        points=maneuver_points(origin,distance,yaw,lateral,turn_delta,extended=extended)
        if distance>0:
            from .rear_scan import rear_clearance
            if rear_clearance(scan,transform[:3],motion.get('rear_stop_distance_m',.3))!='CLEAR':
                return False,points
        if distance or lateral:
            # Do not select a geometrically clear maneuver that the independent
            # braking gate cannot begin. Search another direction instead.
            from .motion_braking import limit_velocity
            length=math.hypot(distance,lateral)
            initial=(-.05*distance/length,.05*lateral/length,0.)
            limited,_=limit_velocity(initial,view,age,profile)
            if math.hypot(limited[0],limited[1])<.005:return False,points
        c,s=math.cos(origin['yaw_rad']),math.sin(origin['yaw_rad'])
        obstacles=[(origin['x_m']+c*x-s*y,origin['y_m']+s*x+c*y) for x,y in view['obstacle_points']]
        grid=self._cargo_saved_grid()
        extent=self.worker['mapping']['autonomy']['footprint'];zones=getattr(self,'zones',None)
        clear=maneuver_clear(grid,points,extent,obstacles,zones)
        if not clear and (distance or lateral):
            clear=padding_escape_clear(grid,points,extent,obstacles,zones)
        return clear,points

    async def _cargo_reverse(self,distance,p):
        return await self._cargo_translate(distance,0.,p)

    async def _cargo_reposition(self,distance,lateral,p):
        if lateral:return await self._cargo_translate(distance,lateral,p)
        return await self._cargo_reverse(distance,p)

    async def _cargo_translate(self,distance,lateral,p):
        from geometry_msgs.msg import Twist
        if distance>0 and not self.cfg['motion'].get('allow_reverse'):raise StepError('CARGO_REVERSE_NOT_ENABLED')
        if lateral and not (self.cfg['motion'].get('allow_strafe') and self.cfg['motion'].get('chassis_model')=='MENTORPI_M1_MECANUM'):
            raise StepError('STRAFE_NOT_ENABLED')
        length=math.hypot(distance,lateral)
        if not 0<length<=.141422:raise StepError('CARGO_ESCAPE_DISTANCE_INVALID')
        ux,uy=-distance/length,lateral/length
        _,origin=self._cargo_pose();sample=getattr(self,'odom_pose_sample',None)
        if sample is None or len(sample[0])<3 or time.monotonic()-sample[1]>.5:raise StepError('STALE_ODOM')
        odom_origin=sample[0];started=time.monotonic();speed=min(.05,p['max_speed_mps'])
        if ux<0:speed=min(speed,self.cfg['motion'].get('max_reverse_speed_mps',.1)/(-ux))
        self.clear_navigation_velocity()
        try:
            while True:
                state,pose=self._cargo_pose();sample=getattr(self,'odom_pose_sample',None)
                if sample is None or time.monotonic()-sample[1]>.5:raise StepError('STALE_ODOM')
                dx,dy=sample[0][0]-odom_origin[0],sample[0][1]-odom_origin[1]
                c,s=math.cos(odom_origin[2]),math.sin(odom_origin[2])
                bx,by=c*dx+s*dy,-s*dx+c*dy
                travelled=ux*bx+uy*by
                if travelled<-.01 or abs(-uy*bx+ux*by)>.025:raise StepError('CARGO_ESCAPE_DIRECTION_INVALID')
                if travelled>=length-.005:return
                if time.monotonic()-started>length/max(speed,.01)+2.:raise StepError('CARGO_REVERSE_BLOCKED')
                if abs(wrap(pose['yaw_rad']-origin['yaw_rad']))>.15:raise StepError('CARGO_ESCAPE_HEADING_CHANGED')
                remaining=max(0.,length-travelled)/length
                clear,_=self._cargo_clear(pose,distance*remaining,pose['yaw_rad'],lateral=lateral*remaining)
                if not clear:raise StepError('CARGO_REVERSE_BLOCKED')
                cap=speed
                if getattr(self,'zones',None) is not None:cap=self.zones.directional_cap(state['map_pose'],speed,(ux*speed,uy*speed,0.))
                if cap<=0.:raise StepError('CARGO_REVERSE_BLOCKED_BY_ZONE')
                speed_now=min(speed,cap)
                if ux and abs(ux*speed_now)<.005:raise StepError('CARGO_REVERSE_BLOCKED')
                msg=Twist();msg.linear.x=ux*speed_now;msg.linear.y=uy*speed_now
                self.navigation_velocity_output.publish(msg)
                self.lease_message(True,speed_now);await asyncio.sleep(.05)
        finally:
            if not await self.stop():raise StepError('GOAL_STOP_UNCONFIRMED')

    async def _cargo_rotate(self,yaw,p,turn_delta=None):
        from nav2_msgs.action import Spin
        _,pose=self._cargo_pose();delta=wrap(yaw-pose['yaw_rad']) if turn_delta is None else turn_delta
        if abs(delta)<=.10:return
        # A disabled lease confirms command ownership, not stopped wheels.
        # Finish the translation before checking a turn from the actual pose.
        await self._wait_stationary()
        _,pose=self._cargo_pose()
        delta=(wrap(yaw-pose['yaw_rad']) if turn_delta is None
               else turn_in_direction(pose['yaw_rad'],yaw,turn_delta))
        if abs(delta)<=.10:return
        clear,_=self._cargo_clear(pose,0.,yaw,turn_delta=delta)
        if not clear:raise StepError('CARGO_ESCAPE_TURN_BLOCKED')
        await self._wait_navigation_server(self.spin,'FINAL_ALIGNMENT_UNAVAILABLE')
        goal=Spin.Goal();goal.target_yaw=float(delta);goal.time_allowance.sec=math.ceil(max(8.,abs(delta)/.2+4.))
        self.clear_navigation_velocity();sent=self.spin.send_goal_async(goal);received=False;start=time.monotonic()
        odom_origin=getattr(self,'odom_pose_sample',None)
        try:
            self.goal=await self.future(sent);received=True
            if not self.goal.accepted:raise StepError('CARGO_ESCAPE_TURN_REJECTED')
            result=self.goal.get_result_async()
            while not result.done():
                if time.monotonic()-start>goal.time_allowance.sec+2:raise StepError('CARGO_ESCAPE_TURN_TIMEOUT')
                state,_=self._cargo_pose()
                odom=getattr(self,'odom_pose_sample',None)
                if (odom_origin and odom and time.monotonic()-odom[1]<.5
                        and math.hypot(odom[0][0]-odom_origin[0][0],odom[0][1]-odom_origin[0][1])>.03):raise StepError('CARGO_ESCAPE_DRIFT')
                if self.publish_navigation_velocity(p['max_speed_mps'],rotation_only=True,state=state):self.lease_message(True,p['max_speed_mps'])
                else:self.lease_message(False)
                await asyncio.sleep(.05)
            status=result.result().status;self.goal=None
            if status!=4:raise StepError('CARGO_ESCAPE_TURN_BLOCKED')
        finally:
            if not received:sent.add_done_callback(self._late_cancel)
            if not await self.stop():raise StepError('GOAL_STOP_UNCONFIRMED')

    async def _plan_with_cargo_recovery(self,p,endpoint_id,meta,force_escape=False):
        return await self._retry_localization(
            lambda:self._plan_with_cargo_recovery_once(p,endpoint_id,meta,force_escape),p,endpoint_id,meta)

    async def _plan_with_cargo_recovery_once(self,p,endpoint_id,meta,force_escape=False):
        runtime=getattr(self,'worker',{}).get('navigation_runtime',{})
        extended=(runtime.get('map_yaml') and (runtime.get('full_footprint_search')
                                               or runtime.get('controller')=='HOLONOMIC_DWB')
                  and runtime.get('geometry',{}).get('profile')=='cargo' and 'endpoint_group_id' not in p)
        if not force_escape:
            try:
                if (extended or runtime.get('geometry',{}).get('profile')=='bare'
                        or 'endpoint_group_id' in p):return await self._plan(p,endpoint_id,meta)
                return await self._wait_obstacle_path(p,endpoint_id,meta)
            except StepError as exc:
                if (exc.code!='PATH_UNAVAILABLE' or runtime.get('geometry',{}).get('profile') not in ('bare','cargo')
                        or 'endpoint_group_id' in p):raise
        elif runtime.get('geometry',{}).get('profile') not in ('bare','cargo'):raise StepError('NAV_GATE_BLOCKED')
        if getattr(self,'cargo_escape_used',False):
            if extended and not force_escape:return await self._wait_obstacle_path(p,endpoint_id,meta,blocked=True)
            raise StepError('CARGO_ESCAPE_LIMIT_REACHED')
        await self._wait_stationary()
        _,origin=self._cargo_pose();target=self.cfg['endpoints'][endpoint_id]
        bearing=math.atan2(target['y_m']-origin['y_m'],target['x_m']-origin['x_m'])
        headings=[]
        for a in (bearing,origin['yaw_rad']+math.pi/2,origin['yaw_rad']-math.pi/2,origin['yaw_rad']):
            if not any(abs(wrap(a-b))<.1 for b in headings):headings.append(a)
        selected=None;deadline=time.monotonic()+5.;probes=0
        offsets=self._cargo_offsets()
        moving_offsets=offsets[1:]
        near=moving_offsets[:len(moving_offsets)//2]
        far=moving_offsets[len(near):]
        heading=origin['yaw_rad']
        turns=[y for y in headings if abs(wrap(y-heading))>.10]
        candidates=[(d,l,heading) for d,l in near]
        if not force_escape:candidates.extend((0.,0.,y) for y in turns)
        # Try a longer translation and a turned start together, after the
        # short translation probes have covered all allowed directions.
        for (d,l),(short_d,short_l) in zip(far,near):
            candidates.append((d,l,heading))
            candidates.extend((short_d,short_l,y) for y in turns)
        candidates.extend((d,l,y) for d,l in far for y in turns)
        offset_counts={};probe_limit=len(near)+4
        for distance,lateral,yaw in candidates:
            if time.monotonic()>deadline or probes>=probe_limit:break
            await asyncio.sleep(0)
            if offset_counts.get((distance,lateral),0)>=2:continue
            if not (distance or lateral) and abs(wrap(yaw-origin['yaw_rad']))<=.15:continue
            for delta in turn_options(origin['yaw_rad'],yaw):
                clear,points=self._cargo_clear(origin,distance,yaw,lateral=lateral,turn_delta=delta)
                if not clear:continue
                remaining=deadline-time.monotonic()
                if remaining<=0.:break
                x,y,a=points[-1];probes+=1
                offset_counts[(distance,lateral)]=offset_counts.get((distance,lateral),0)+1
                try:
                    await asyncio.wait_for(self._plan(p,endpoint_id,meta,
                        start_override=dict(x_m=x,y_m=y,yaw_rad=a),probe=True),remaining)
                except asyncio.TimeoutError:break
                except StepError as exc:
                    if exc.code in ('PATH_UNAVAILABLE','NAVIGATION_PATH_INTERSECTS_KEEPOUT'):break
                    raise
                selected=(distance,lateral,yaw,delta);break
            if selected is not None:break
        staged=selected is None and extended
        if staged:
            selected=await self._probe_extended_cargo_escape(p,endpoint_id,meta)
            if selected is None and not force_escape:
                return await self._wait_obstacle_path(p,endpoint_id,meta,blocked=True)
        if selected is None:raise StepError('NAV_RECOVERY_NO_CLEAR_MANEUVER' if force_escape else 'PATH_UNAVAILABLE')
        try:return await self._execute_cargo_escape(p,endpoint_id,meta,selected,staged)
        except StepError as exc:
            if not extended or exc.code not in ('PATH_UNAVAILABLE','NAV_RECOVERY_NO_CLEAR_MANEUVER',
                                                'CARGO_REVERSE_BLOCKED','CARGO_ESCAPE_TURN_BLOCKED'):raise
            self.node.get_logger().warning('CARGO_ESCAPE_STOPPED reason='+exc.code)
            await self._wait_stationary()
            return await self._wait_obstacle_path(p,endpoint_id,meta,blocked=True)

    async def _probe_extended_cargo_escape(self,p,endpoint_id,meta):
        """Test wider straight escapes and both turn directions before moving."""
        _,origin=self._cargo_pose();heading=origin['yaw_rad']
        target=self.cfg['endpoints'][endpoint_id]
        bearing=math.atan2(target['y_m']-origin['y_m'],target['x_m']-origin['x_m'])
        turns=[bearing]+[heading+offset*math.pi/4 for offset in (1,-1,2,-2,3,-3,4)]
        directions=[(d/.10,l/.10) for d,l in self._cargo_offsets() if math.isclose(math.hypot(d,l),.10)]
        # A different starting yaw may unlock the path in a doorway without a
        # translation. Then probe every 16 cm direction before going farther.
        candidates=[(0.,0.,yaw) for yaw in turns if abs(wrap(yaw-heading))>.10]
        candidates.extend((r*d,r*l,heading) for r in (.16,.20) for d,l in directions)
        candidates.extend((r*d,r*l,yaw) for yaw in turns for d,l in directions for r in (.16,.20))
        deadline=time.monotonic()+6.;probes=0;tested=set()
        for distance,lateral,yaw in candidates:
            if time.monotonic()>=deadline or probes>=16:break
            await asyncio.sleep(0)
            for delta in turn_options(heading,yaw):
                if time.monotonic()>=deadline:return None
                clear,points=self._cargo_clear(origin,distance,yaw,lateral=lateral,turn_delta=delta,extended=True)
                if not clear:continue
                x,y,a=points[-1];key=(round(x,4),round(y,4),round(math.cos(a),4),round(math.sin(a),4))
                if key in tested:break
                remaining=deadline-time.monotonic()
                if remaining<=0.:return None
                tested.add(key);probes+=1
                try:
                    await asyncio.wait_for(self._plan(p,endpoint_id,meta,
                        start_override=dict(x_m=x,y_m=y,yaw_rad=a),probe=True),remaining)
                except asyncio.TimeoutError:return None
                except StepError as exc:
                    if exc.code in ('PATH_UNAVAILABLE','NAVIGATION_PATH_INTERSECTS_KEEPOUT',
                                    'ENDPOINT_FOOTPRINT_NOT_CLEAR'):break
                    raise
                return distance,lateral,yaw,delta
        return None

    async def _execute_cargo_escape(self,p,endpoint_id,meta,selected,staged):
        distance,lateral,yaw,delta=selected
        self.cargo_escape_used=True
        label='CARGO_EXTENDED_ESCAPE_SELECTED' if staged else 'CARGO_ESCAPE_SELECTED'
        self.node.get_logger().info(label+' distance_m='+str(distance)+
            ' lateral_m='+str(lateral)+' turn_rad='+str(delta))
        if staged and (distance or lateral):
            distance*=.5;lateral*=.5
            await self._cargo_reposition(distance,lateral,p)
            await self._wait_stationary()
            try:return await self._plan(p,endpoint_id,meta)
            except StepError as exc:
                if exc.code!='PATH_UNAVAILABLE':raise
            _,actual=self._cargo_pose()
            remaining=turn_in_direction(actual['yaw_rad'],yaw,delta)
            clear,_=self._cargo_clear(actual,distance,yaw,lateral=lateral,turn_delta=remaining)
            if not clear:raise StepError('NAV_RECOVERY_NO_CLEAR_MANEUVER')
            await self._cargo_reposition(distance,lateral,p)
            await self._wait_stationary()
        elif distance or lateral:
            await self._cargo_reposition(distance,lateral,p)
        # Keep the selected turn direction and commit from the actual pose.
        _,actual=self._cargo_pose()
        remaining=turn_in_direction(actual['yaw_rad'],yaw,delta)
        await self._cargo_rotate(yaw,p,turn_delta=remaining)
        return await self._plan(p,endpoint_id,meta)
