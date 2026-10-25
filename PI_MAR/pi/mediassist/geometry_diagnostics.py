"""Read-only scan/map geometry summaries. No ROS, configuration writes or motor output."""
import math


def planar_transform(transform):
    q=transform.rotation
    values=(transform.translation.x,transform.translation.y,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values):raise ValueError('INVALID_LIDAR_TF')
    norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    normal=1-2*(q.x*q.x+q.y*q.y)
    if not .99<=norm<=1.01 or abs(normal)<.98:raise ValueError('NON_PLANAR_LIDAR_TF')
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return (transform.translation.x,transform.translation.y,yaw,1 if normal>0 else -1)


def scan_points(scan,transform,step=1):
    tx,ty,yaw=transform[:3]
    direction=transform[3] if len(transform)==4 else 1
    for i in range(0,len(scan.ranges),step):
        distance=scan.ranges[i]
        if not math.isfinite(distance) or not max(.01,scan.range_min)<=distance<=scan.range_max:continue
        angle=yaw+direction*(scan.angle_min+i*scan.angle_increment)
        yield (i,distance,tx+distance*math.cos(angle),ty+distance*math.sin(angle))


def scan_summary(scan,transform,footprint):
    count=0;inside=0;near=[];nearest=None
    for i,d,x,y in scan_points(scan,transform):
        count+=1
        if nearest is None or d<nearest:nearest=d
        if -footprint['rear']<=x<=footprint['front'] and -footprint['right']<=y<=footprint['left']:inside+=1
        if d<=.4:near.append(dict(index=i,range_m=round(d,4),base_x_m=round(x,4),base_y_m=round(y,4)))
    return dict(finite_hits=count,nearest_lidar_m=nearest,hits_inside_body=inside,
                near_hits=sorted(near,key=lambda p:p['range_m'])[:12])


def map_summary(snapshot,pose,footprint):
    w,h,res=snapshot['width'],snapshot['height'],snapshot['resolution']
    ox,oy,oa=snapshot['origin'];c,s=math.cos(oa),math.sin(oa)
    pc,ps=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
    inside=0;near=[];radius=math.hypot(max(footprint['front'],footprint['rear']),max(footprint['left'],footprint['right']))
    dx,dy=pose['x_m']-ox,pose['y_m']-oy
    u,v=math.floor((c*dx+s*dy)/res),math.floor((-s*dx+c*dy)/res)
    pad=math.ceil(max(radius,.5)/res)+2
    for row in range(max(0,v-pad),min(h,v+pad+1)):
        for col in range(max(0,u-pad),min(w,u+pad+1)):
            value=snapshot['data'][row*w+col]
            if not 65<=value<=100:continue
            mx,my=(col+.5)*res,(row+.5)*res
            wx,wy=ox+c*mx-s*my,oy+s*mx+c*my
            dx,dy=wx-pose['x_m'],wy-pose['y_m']
            x,y=pc*dx+ps*dy,-ps*dx+pc*dy
            if -footprint['rear']<=x<=footprint['front'] and -footprint['right']<=y<=footprint['left']:inside+=1
            near.append(dict(distance_to_base_m=round(math.hypot(x,y),4),base_x_m=round(x,4),base_y_m=round(y,4)))
    seed=snapshot['data'][v*w+u] if 0<=u<w and 0<=v<h else None
    return dict(seed_cell=seed,occupied_cells_inside_body=inside,
                nearest_occupied_cells=sorted(near,key=lambda p:p['distance_to_base_m'])[:8])


def nav_footprints(parameters):
    result={}
    for name in ('local_costmap','global_costmap'):
        cfg=parameters.get(name,{}).get(name,{}).get('ros__parameters',{})
        result[name]={k:cfg.get(k) for k in ('footprint','robot_radius','footprint_padding','robot_base_frame')}
    return result
