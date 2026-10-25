"""Conservative reachable-frontier planning on real OccupancyGrid data. No motor access."""
import asyncio
from collections import deque
import hashlib
import math


def source_signature(snapshot):
    geometry=tuple(snapshot[k] for k in ('width','height','resolution','origin','frame_id'))
    return hashlib.sha256(repr(geometry).encode()+snapshot['data']).hexdigest()


def neighbors(i,w,h,diagonal=False):
    x,y=i%w,i//w
    for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
        if 0<=x+dx<w and 0<=y+dy<h:yield (y+dy)*w+x+dx
    if diagonal:
        for dx,dy in ((1,1),(1,-1),(-1,1),(-1,-1)):
            if 0<=x+dx<w and 0<=y+dy<h:yield (y+dy)*w+x+dx


def squared_distances(cells,w,h,boundary=True):
    """Exact Euclidean distances to blocked cell centres, in squared cells, O(w*h)."""
    def line(values):
        n=len(values);sites=[0]*n;cuts=[-math.inf]+[math.inf]*n;k=0
        for q in range(1,n):
            previous=sites[k]
            cross=((values[q]+q*q)-(values[previous]+previous*previous))/(2*(q-previous))
            while cross<=cuts[k]:
                k-=1;previous=sites[k]
                cross=((values[q]+q*q)-(values[previous]+previous*previous))/(2*(q-previous))
            k+=1;sites[k]=q;cuts[k]=cross;cuts[k+1]=math.inf
        result=[0]*n;k=0
        for q in range(n):
            while cuts[k+1]<q:k+=1
            previous=sites[k];result[q]=(q-previous)**2+values[previous]
        return result
    far=(w+h+1)**2
    distance=[0 if cells[i] or boundary and (i%w in (0,w-1) or i//w in (0,h-1)) else far for i in range(w*h)]
    for x in range(w):
        column=line([distance[y*w+x] for y in range(h)])
        for y,value in enumerate(column):distance[y*w+x]=value
    for y in range(h):distance[y*w:(y+1)*w]=line(distance[y*w:(y+1)*w])
    return distance


def visible(grid,start,end):
    """Supercover ray through safe planner cells, including both sides of a corner."""
    ox,oy,a=grid['origin'];c,s=math.cos(a),math.sin(a);cell=grid['cell'];w,h=grid['width'],grid['height']
    def point(p):
        dx,dy=p[0]-ox,p[1]-oy
        return ((c*dx+s*dy)/cell,(-s*dx+c*dy)/cell)
    x,y=point(start);ex,ey=point(end);ix,iy=math.floor(x),math.floor(y);gx,gy=math.floor(ex),math.floor(ey)
    dx,dy=ex-x,ey-y;sx=1 if dx>0 else -1;sy=1 if dy>0 else -1
    tx=((ix+(1 if sx>0 else 0))-x)/dx if dx else math.inf
    ty=((iy+(1 if sy>0 else 0))-y)/dy if dy else math.inf
    stepx=abs(1/dx) if dx else math.inf;stepy=abs(1/dy) if dy else math.inf
    def free(u,v):return 0<=u<w and 0<=v<h and grid['safe'][v*w+u]
    for _ in range(w+h+2):
        if not free(ix,iy):return False
        if ix==gx and iy==gy:return True
        if abs(tx-ty)<1e-10:
            if not free(ix+sx,iy) or not free(ix,iy+sy):return False
            ix+=sx;iy+=sy;tx+=stepx;ty+=stepy
        elif tx<ty:ix+=sx;tx+=stepx
        else:iy+=sy;ty+=stepy
    return False


def plan(snapshot,pose,cfg):
    """Unknown is not free. Inflate the full swept-body bound, including map edges."""
    width,height,res=snapshot['width'],snapshot['height'],snapshot['resolution']
    origin=snapshot['origin'];raw=snapshot['data']
    if (type(width) is not int or type(height) is not int or not 0<width*height<=4_000_000
            or len(raw)!=width*height or not math.isfinite(res) or res<=0
            or len(origin)!=3 or not all(math.isfinite(v) for v in origin)
            or not all(math.isfinite(pose.get(k,math.nan)) for k in ('x_m','y_m','yaw_rad'))):
        raise ValueError('INVALID_EXPLORATION_GRID')
    # Coarsen only the planner; the SLAM map and saved artifacts stay unchanged.
    stride=max(1,math.ceil(width/300),math.ceil(height/300))
    w,h=math.ceil(width/stride),math.ceil(height/stride);cell=res*stride;n=w*h
    cells=bytearray(n)  # 0 known free <=25, 1 unknown/uncertain, 2 occupied >=65
    for y in range(height):
        for x in range(width):
            value=raw[y*width+x];i=(y//stride)*w+x//stride
            if 65<=value<=100:cells[i]=2
            elif not 0<=value<=25 and cells[i]!=2:cells[i]=1
    c,s=math.cos(origin[2]),math.sin(origin[2])
    dx,dy=pose['x_m']-origin[0],pose['y_m']-origin[1]
    px,py=(c*dx+s*dy)/cell,(-s*dx+c*dy)/cell
    signature=hashlib.sha256(repr((width,height,res,origin)).encode()+cells).hexdigest()
    result=dict(state='BLOCKED',reason='MAP_POSE_OUTSIDE_FREE',frontier_count=0,
                frontier_cells=0,ignored_frontier_cells=0,reachable_free_area_m2=0.,
                signature=signature,source_signature=source_signature(snapshot),path=[],goal=None)
    if not (0<=px<w and 0<=py<h):return result
    seed=int(py)*w+int(px)
    if cells[seed]!=0:return result
    # Raw connectivity defines the room. Unknown outside closed walls is excluded.
    reached={seed};q=deque([seed]);frontiers=set()
    while q:
        i=q.popleft();x,y=i%w,i//w
        adjacent=list(neighbors(i,w,h))
        if x in (0,w-1) or y in (0,h-1) or any(cells[j]==1 for j in adjacent):frontiers.add(i)
        for j in adjacent:
            if cells[j]==0 and j not in reached:reached.add(j);q.append(j)
    result['reachable_free_area_m2']=round(len(reached)*cell*cell,3)
    clusters=[];remaining=set(frontiers)
    while remaining:
        i=min(remaining);remaining.remove(i);group=[i];q=deque([i])
        while q:
            for j in neighbors(q.popleft(),w,h,True):
                if j in remaining:remaining.remove(j);group.append(j);q.append(j)
        if len(group)*cell>=cfg['frontier_min_length_m']:clusters.append(group)
        else:result['ignored_frontier_cells']+=len(group)
    result.update(frontier_count=len(clusters),frontier_cells=sum(map(len,clusters)))
    footprint=cfg['footprint']
    body_radius=max(math.hypot(footprint[a],footprint[b]) for a in ('front','rear') for b in ('left','right'))
    radius=body_radius+cfg['stop_margin_m']
    # The robot and obstacle each occupy an entire cell, not only their centres.
    # Two half diagonals preserve clearance for every point of a traversed cell.
    clearance_cells=radius/cell+math.sqrt(2)
    padding=math.ceil(clearance_cells)
    distance=squared_distances(bytearray(1 if value==2 else 0 for value in cells),w,h,boundary=False)
    unknown=squared_distances(bytearray(1 if value==1 else 0 for value in cells),w,h)
    threshold=clearance_cells**2
    # Unknown remains forbidden to the entire rotating body. It is not a measured
    # obstacle and does not gain the additional obstacle stand-off buffer.
    unknown_threshold=(body_radius/cell+math.sqrt(2))**2
    safe=bytes(1 if cells[i]==0 and distance[i]>threshold and unknown[i]>unknown_threshold else 0 for i in range(n))
    obstacle_clearance=max(0.,math.sqrt(distance[seed])*cell-math.sqrt(2)*cell)
    unknown_clearance=max(0.,math.sqrt(unknown[seed])*cell-math.sqrt(2)*cell)
    result.update(planner_cell_m=cell,clearance_radius_m=radius,body_radius_m=body_radius,
                  seed_clearance_m=min(obstacle_clearance,unknown_clearance),
                  seed_obstacle_clearance_m=obstacle_clearance,seed_unknown_clearance_m=unknown_clearance)
    nav2=cfg.get('frontier_controller')=='nav2'
    from .footprint import body_clear
    actual=body_clear(snapshot,pose,footprint)
    result.update(planning_backend='nav2' if nav2 else 'reactive',seed_body_clear=actual['clear'],seed_block_kind=actual['kind'])
    if not actual['clear'] or not nav2 and not safe[seed]:
        result['reason']='ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP';return result
    if not clusters:
        result.update(state='NO_FRONTIERS',reason='NO_SIGNIFICANT_REACHABLE_FRONTIER')
        return result
    parents={seed:None};cost={seed:0};q=deque([seed])
    while q:
        i=q.popleft()
        for j in neighbors(i,w,h):
            # Nav2 validates the whole route with the actual oriented footprint.
            # This raw-connectivity path only ranks candidate observation goals;
            # it must never be forwarded as a direct velocity authorization.
            if (cells[j]==0 if nav2 else safe[j]) and j not in parents:
                parents[j]=i;cost[j]=cost[i]+1;q.append(j)
    best=None
    # Prefer large frontiers, but never declare completion when some are blocked.
    for group in sorted(clusters,key=lambda g:(-len(g),min(g)))[:32]:
        q=deque(sorted(group));source={i:i for i in group};depth=0;candidates=[]
        # Frontier search uses four-neighbour (Manhattan) steps while inflation
        # uses Euclidean distance, so a diagonal stand-off needs up to sqrt(2) more steps.
        while q and depth<=math.ceil(padding*math.sqrt(2))+3+(math.ceil(.25/cell) if nav2 else 0) and not candidates:
            for _ in range(len(q)):
                i=q.popleft()
                if i in parents and safe[i]:
                    excluded=cfg.get('_excluded_goals',[])
                    ux=(i%w+.5)*cell;uy=(i//w+.5)*cell;wx=origin[0]+c*ux-s*uy;wy=origin[1]+s*ux+c*uy
                    if not any(math.hypot(wx-g[0],wy-g[1])<.20 for g in excluded):
                        candidates.append(i);continue
                for j in neighbors(i,w,h):
                    if cells[j]==0 and j not in source:source[j]=source[i];q.append(j)
            depth+=1
        if candidates:
            goal=min(candidates,key=lambda i:(cost[i],i))
            score=cost[goal]*cell-.05*min(len(group),20)
            if best is None or score<best[0]:best=(score,goal,source[goal])
    if best is None:
        result['reason']='FRONTIERS_UNREACHABLE' if cfg.get('_excluded_goals') else 'FRONTIERS_BLOCKED_BY_FOOTPRINT';return result
    _,goal,frontier=best
    def world(i):
        x=(min(width,(i%w+1)*stride)+i%w*stride)*res*.5
        y=(min(height,(i//w+1)*stride)+i//w*stride)*res*.5
        return (origin[0]+c*x-s*y,origin[1]+s*x+c*y)
    path=[];i=goal
    while i is not None:path.append(world(i));i=parents[i]
    path.reverse();gx,gy=world(goal);fx,fy=world(frontier)
    result.update(state='FRONTIERS_PRESENT',reason='APPROACH_FRONTIER',path=path,
                  goal=dict(x_m=gx,y_m=gy,yaw_rad=math.atan2(fy-gy,fx-gx)),
                  planning_grid=dict(width=w,height=h,cell=cell,origin=origin,
                      safe=safe))
    return result


class Tracker:
    def __init__(self,cfg):
        self.cfg=cfg;self.result=None;self.since=None;self.samples=0;self.revision=None
        self.complete=False;self.progress_signature=None;self.progress_at=None

    def observe(self,result,now,revision,elapsed,travel):
        previous=self.result;self.result=result
        if result['state']=='NO_FRONTIERS':
            if previous is None or previous['state']!='NO_FRONTIERS' or previous['signature']!=result['signature']:
                self.since=now;self.samples=0;self.revision=None
            if revision!=self.revision:self.samples+=1;self.revision=revision
            self.complete=(elapsed>=self.cfg['coverage_min_duration_sec'] and travel>=self.cfg['coverage_min_travel_m']
                           and result['reachable_free_area_m2']>=self.cfg['coverage_min_area_m2']
                           and self.samples>=3 and now-self.since>=self.cfg['coverage_stable_sec'])
        else:
            self.since=None;self.samples=0;self.complete=False
        if self.progress_signature!=result['signature']:
            self.progress_signature=result['signature'];self.progress_at=now

    def status(self):
        r=self.result or {}
        return dict(enabled=True,state='COMPLETE' if self.complete else r.get('state','PLANNING'),
                    reason=r.get('reason','WAITING_FOR_PLAN'),frontier_count=r.get('frontier_count',0),
                    frontier_cells=r.get('frontier_cells',0),ignored_frontier_cells=r.get('ignored_frontier_cells',0),
                    reachable_free_area_m2=r.get('reachable_free_area_m2',0.),goal=r.get('goal'),
                    **{k:r[k] for k in ('planning_backend','planner_cell_m','clearance_radius_m','body_radius_m',
                        'seed_obstacle_clearance_m','seed_unknown_clearance_m','seed_body_clear','seed_block_kind') if k in r})


class FrontierWorker:
    """Planning runs outside the MQTT/lease loop; stale plans never drive."""
    def __init__(self,cfg):
        self.cfg=cfg;self.run=None;self.task=None;self.tracker=Tracker(cfg)
        self.submitted_at=self.observed_at=0.;self.pending_revision=None
        self.hold_at=None;self.hold_signature=None
        self.excluded=[];self.replan=False

    def reset(self,run):
        if self.task:self.task.cancel()
        self.run=run;self.task=None;self.tracker=Tracker(self.cfg)
        self.submitted_at=self.observed_at=0.
        self.hold_at=None;self.hold_signature=None
        self.excluded=[];self.replan=False

    def exclude_goal(self,goal,now,ttl=30.):
        self.excluded.append((goal['x_m'],goal['y_m'],now+ttl));self.excluded=self.excluded[-64:]
        # Do not corrupt the timestamp of a plan still running in the executor.
        # Discard that result and submit a new plan with the exclusion applied.
        self.replan=True
        self.tracker.result=None

    def no_progress(self,now,holding):
        signature=self.tracker.progress_signature
        if not holding or self.hold_at is None or self.hold_signature!=signature:
            self.hold_at=now if holding else None;self.hold_signature=signature
            return False
        return now-self.hold_at>=self.cfg['frontier_no_progress_sec']

    def poll(self,snapshot,pose,revision,now,run,elapsed,travel):
        if self.run!=run:self.reset(run)
        if self.task and self.task.done():
            try:result=self.task.result()
            except Exception:
                self.task=None;return None,'EXPLORATION_PLAN_FAILED'
            self.task=None;self.observed_at=now
            if now-self.submitted_at>3.:return None,'EXPLORATION_PLAN_TIMEOUT'
            if not self.replan:self.tracker.observe(result,now,self.pending_revision,elapsed,travel)
        if self.task and now-self.submitted_at>3.:return None,'EXPLORATION_PLAN_TIMEOUT'
        if self.task is None and (self.replan or self.tracker.result is None or now-self.submitted_at>=1.):
            self.replan=False
            self.pending_revision=revision;self.submitted_at=now
            self.excluded=[g for g in self.excluded if now<g[2]]
            options={**self.cfg,'_excluded_goals':[(x,y) for x,y,_ in self.excluded]}
            self.task=asyncio.create_task(asyncio.to_thread(plan,snapshot,dict(pose),options))
        if self.tracker.result and now-self.observed_at>3.:return None,'EXPLORATION_PLAN_STALE'
        return self.tracker.result,None


def guidance(result,pose):
    if not result or result['state']!='FRONTIERS_PRESENT':return None
    gx,gy=result['goal']['x_m'],result['goal']['y_m']
    distance=math.hypot(gx-pose['x_m'],gy-pose['y_m'])
    tolerance=min(.06,max(.02,result.get('planner_cell_m',.1)*.75))
    if distance<tolerance:return dict(yaw=result['goal']['yaw_rad'],hold=True)
    # Use a waypoint ahead on this newly planned path, never a straight line
    # through walls to the final frontier.
    path=result['path'];nearest=min(range(len(path)),key=lambda i:math.hypot(path[i][0]-pose['x_m'],path[i][1]-pose['y_m']))
    point=path[-1]
    previous=None;grid=result.get('planning_grid')
    for p in path[nearest:]:
        if grid and not visible(grid,(pose['x_m'],pose['y_m']),p):
            if previous is None:return dict(yaw=pose['yaw_rad'],hold=True)
            break
        point=p
        previous=p
        if math.hypot(p[0]-pose['x_m'],p[1]-pose['y_m'])>=.25:break
    return dict(yaw=math.atan2(point[1]-pose['y_m'],point[0]-pose['x_m']),hold=False)
