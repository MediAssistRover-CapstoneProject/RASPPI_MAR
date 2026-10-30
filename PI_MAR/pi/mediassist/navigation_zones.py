"""Metric zone rules and latched Nav2 keepout masks for a saved map."""
import math
from .contracts import StepError


def rules(raw):
    if not isinstance(raw,list) or len(raw)>256:raise StepError('INVALID_NAV_ZONES')
    result=[]
    for zone in raw:
        if not isinstance(zone,dict):raise StepError('INVALID_NAV_ZONES')
        kind=zone.get('zone_type');polygon=zone.get('polygon')
        if isinstance(polygon,dict):polygon=polygon.get('points')
        if kind not in ('KEEP_OUT','SPEED') or not isinstance(polygon,list) or not 3<=len(polygon)<=256:
            raise StepError('INVALID_NAV_ZONE_POLYGON')
        points=[]
        for point in polygon:
            if not isinstance(point,(list,tuple)) or len(point)!=2 or any(type(v) not in (float,int) or not math.isfinite(v) for v in point):
                raise StepError('INVALID_NAV_ZONE_POLYGON')
            points.append(tuple(point))
        area=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1])))
        if area<1e-8:raise StepError('INVALID_NAV_ZONE_POLYGON')
        limit=zone.get('speed_limit_mps')
        if kind=='SPEED' and (type(limit) not in (float,int) or not math.isfinite(limit) or limit<=0):
            raise StepError('INVALID_NAV_ZONE_SPEED')
        result.append(dict(kind=kind,points=points,speed=limit))
    return result


def distance(point, polygon):
    x,y=point;inside=False;nearest=math.inf
    for (ax,ay),(bx,by) in zip(polygon,polygon[1:]+polygon[:1]):
        dx,dy=bx-ax,by-ay;t=max(0.,min(1.,((x-ax)*dx+(y-ay)*dy)/max(dx*dx+dy*dy,1e-20)))
        nearest=min(nearest,math.hypot(x-ax-t*dx,y-ay-t*dy))
        if (ay>y)!=(by>y) and x<(bx-ax)*(y-ay)/(by-ay)+ax:inside=not inside
    return 0. if inside else nearest


def body_gap(pose,extent,polygon):
    """Distance from an oriented rectangle to a simple zone polygon."""
    c,s=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
    body=[(pose['x_m']+c*x-s*y,pose['y_m']+s*x+c*y) for x,y in
          [(extent['front'],extent['left']),(-extent['rear'],extent['left']),
           (-extent['rear'],-extent['right']),(extent['front'],-extent['right'])]]
    gap=min([distance(p,polygon) for p in body]+[distance(p,body) for p in polygon])
    if gap==0.:return 0.
    def side(a,b,p):return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
    for a,b in zip(body,body[1:]+body[:1]):
        for u,v in zip(polygon,polygon[1:]+polygon[:1]):
            if side(a,b,u)*side(a,b,v)<0 and side(u,v,a)*side(u,v,b)<0:return 0.
    return gap


def compact_path(points,limit=256,clear=None):
    """Preserve bends/cusps instead of connecting uniform samples across zones."""
    if len(points)<2:raise StepError('PATH_UNAVAILABLE')
    if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in points):raise StepError('NAVIGATION_PLAN_INVALID')
    keep={0,len(points)-1};stack=[(0,len(points)-1)]
    while stack:
        lo,hi=stack.pop()
        if hi-lo<2:continue
        a,b=points[lo],points[hi];dx,dy=b[0]-a[0],b[1]-a[1];norm=dx*dx+dy*dy
        da=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]));worst=-1.;split=(lo+hi)//2
        for i in range(lo+1,hi):
            p=points[i];t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/norm)) if norm else (i-lo)/(hi-lo)
            deviation=math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
            yaw=abs(math.atan2(math.sin(p[2]-a[2]-t*da),math.cos(p[2]-a[2]-t*da)))
            score=max(deviation/.01,yaw/.05)
            if score>worst:worst,split=score,i
        safe=True
        if clear is not None:
            try:clear([a,b])
            except StepError:safe=False
        if worst<=1. and safe:continue
        keep.add(split)
        if len(keep)>limit:raise StepError('NAVIGATION_PLAN_PREVIEW_TOO_COMPLEX')
        stack.extend([(lo,split),(split,hi)])
    return [points[i] for i in sorted(keep)]


def keepout_data(grid,zones,radius,soft_margin=0.,soft_floor_radius=0.):
    w,h=grid['width'],grid['height'];res=grid['resolution'];ox,oy,yaw=grid['origin']
    c,s=math.cos(yaw),math.sin(yaw);data=[0]*(w*h)
    polygons=[z['points'] for z in zones if z['kind']=='KEEP_OUT']
    if not polygons:return data
    # Include a cell's half diagonal, so polygon boundaries cannot disappear.
    padding=radius+res/math.sqrt(2.)
    for poly in polygons:
        local=[((x-ox)*c+(y-oy)*s,-(x-ox)*s+(y-oy)*c) for x,y in poly]
        extent=padding+soft_floor_radius+soft_margin
        x0=max(0,math.floor((min(x for x,y in local)-extent)/res))
        x1=min(w,math.ceil((max(x for x,y in local)+extent)/res))
        y0=max(0,math.floor((min(y for x,y in local)-extent)/res))
        y1=min(h,math.ceil((max(y for x,y in local)+extent)/res))
        for row in range(y0,y1):
            for col in range(x0,x1):
                if data[row*w+col]==100:continue
                x,y=(col+.5)*res,(row+.5)*res;p=(ox+c*x-s*y,oy+s*x+c*y)
                gap=distance(p,poly)-padding
                if gap<=0.:data[row*w+col]=100
                elif soft_margin>0. and gap<soft_margin+soft_floor_radius:
                    # Traversable preference, never enlarge the forbidden core.
                    data[row*w+col]=max(data[row*w+col],max(1,round(98*(1-max(0.,gap-soft_floor_radius)/soft_margin))))
    return data


class NavigationZones:
    def __init__(self,node,frame,footprint):
        from nav_msgs.msg import OccupancyGrid
        from nav2_msgs.msg import CostmapFilterInfo
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.node,self.frame=node,frame;self.zones=[];self.footprint=dict(footprint);self.se2=False
        self.radius=math.hypot(max(footprint['front'],footprint['rear']),max(footprint['left'],footprint['right']))+.03
        self.mask=node.create_publisher(OccupancyGrid,'/mediassist/keepout_mask',qos)
        self.info=node.create_publisher(CostmapFilterInfo,'/mediassist/keepout_info',qos)
        # Navfn plans a centre point; DWB checks the actual polygon. Sharing
        # the inflated mask with DWB would count the robot footprint twice.
        self.local_mask=node.create_publisher(OccupancyGrid,'/mediassist/keepout_local_mask',qos)
        self.local_info=node.create_publisher(CostmapFilterInfo,'/mediassist/keepout_local_info',qos)

    def apply(self,raw,map_yaml,footprint_planner=False):
        from nav_msgs.msg import OccupancyGrid
        from nav2_msgs.msg import CostmapFilterInfo
        from .map_tools import load_trinary_grid
        zones=rules(raw);grid=load_trinary_grid(map_yaml)
        msg=OccupancyGrid();msg.header.frame_id=self.frame;msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.info.width=grid['width'];msg.info.height=grid['height'];msg.info.resolution=float(grid['resolution'])
        x,y,yaw=grid['origin'];msg.info.origin.position.x=float(x);msg.info.origin.position.y=float(y)
        msg.info.origin.orientation.z=math.sin(yaw/2);msg.info.origin.orientation.w=math.cos(yaw/2)
        self.se2=footprint_planner
        # SE2 collision checks the oriented body. A high but traversable halo
        # forces Smac's full-footprint check near zones without disc inflation.
        msg.data=keepout_data(grid,zones,0. if self.se2 else self.radius,soft_margin=.25,
            soft_floor_radius=self.radius+.04 if self.se2 else 0.)
        info=CostmapFilterInfo();info.header=msg.header;info.type=0
        info.filter_mask_topic='/mediassist/keepout_mask';info.base=0.;info.multiplier=1.
        self.mask.publish(msg);self.info.publish(info)
        import copy
        local=copy.deepcopy(msg);local.data=keepout_data(grid,zones,0.)
        self.zone_grid={**grid,'data':list(local.data)}
        local_info=copy.deepcopy(info);local_info.filter_mask_topic='/mediassist/keepout_local_mask'
        self.local_mask.publish(local);self.local_info.publish(local_info);self.zones=zones

    def validate_path(self,points):
        """Check every segment, not just the downsampled points sent to BE.

        SE2 checks the oriented padded body; legacy point planners use a disc.
        Sampling adds half a step to the margin, bounding the unchecked gap.
        """
        keepouts=[z for z in self.zones if z['kind']=='KEEP_OUT']
        if not keepouts:return
        if getattr(self,'se2',False):
            extent={k:v+.03 for k,v in self.footprint.items()};previous=None
            for point in points:
                if len(point)!=3 or not all(math.isfinite(v) for v in point):raise StepError('NAVIGATION_PLAN_INVALID')
                a=previous or point;dx,dy=point[0]-a[0],point[1]-a[1]
                da=math.atan2(math.sin(point[2]-a[2]),math.cos(point[2]-a[2]))
                movement=math.hypot(dx,dy)+self.radius*abs(da)
                steps=max(1,math.ceil(movement/.01))
                for i in range(steps+1):
                    p=dict(x_m=a[0]+dx*i/steps,y_m=a[1]+dy*i/steps,yaw_rad=a[2]+da*i/steps)
                    if any(body_gap(p,extent,z['points'])<=movement/steps/2 for z in keepouts):
                        raise StepError('NAVIGATION_PATH_INTERSECTS_KEEPOUT')
                previous=point
            return
        previous=None
        for point in points:
            x,y=point[:2]
            if not all(math.isfinite(v) for v in (x,y)):raise StepError('NAVIGATION_PLAN_INVALID')
            a=previous or point;length=math.hypot(x-a[0],y-a[1]);count=max(1,math.ceil(length/.01))
            margin=self.radius+length/count/2
            for i in range(count+1):
                p=(a[0]+(x-a[0])*i/count,a[1]+(y-a[1])*i/count)
                if any(distance(p,z['points'])<=margin for z in keepouts):
                    raise StepError('NAVIGATION_PATH_INTERSECTS_KEEPOUT')
            previous=point

    def preview_path(self,points):
        return compact_path(points,clear=self.validate_path)

    def speed_cap(self,pose,speed):
        if not self.zones:return speed
        if not pose or not pose.get('fresh'):return 0.
        p=pose['value'];point=(p['x_m'],p['y_m'])
        for zone in self.zones:
            if zone['kind']!='SPEED':continue
            gap=distance(point,zone['points'])-self.radius
            if getattr(self,'se2',False):
                gap=body_gap(p,self.footprint,zone['points'])-.03
            if gap<=speed*.45+max(0.,speed*speed-zone['speed']**2)/1.2:
                speed=min(speed,zone['speed'])
        return speed

    def cap(self,pose,speed):
        speed=self.speed_cap(pose,speed)
        if not self.zones or speed<=0:return speed
        p=pose['value'];point=(p['x_m'],p['y_m'])
        for zone in self.zones:
            if zone['kind']!='KEEP_OUT':continue
            gap=distance(point,zone['points'])-self.radius
            if getattr(self,'se2',False):gap=body_gap(p,self.footprint,zone['points'])-.03
            from .autonomous_mapping import braking_speed_cap
            speed=min(speed,braking_speed_cap(max(0.,gap),.1,0.))
        return speed
