"""Raster/convex-polygon intersection for the actual oriented body; no ROS or motion."""
import math

# Extra clearance beyond each physical body edge; shared by Nav2, zone sweep
# validation and endpoint validation. Grid discretization can add more space.
NAVIGATION_PADDING_M=.03

def overlap(a,b):
    for polygon in (a,b):
        for i,p in enumerate(polygon):
            q=polygon[(i+1)%len(polygon)];axis=(-(q[1]-p[1]),q[0]-p[0])
            pa=[x*axis[0]+y*axis[1] for x,y in a];pb=[x*axis[0]+y*axis[1] for x,y in b]
            if max(pa)<min(pb)-1e-10 or max(pb)<min(pa)-1e-10:return False
    return True

def body_clear(snapshot,pose,extent):
    r=snapshot['resolution'];ox,oy,a=snapshot['origin'];c,s=math.cos(a),math.sin(a)
    yaw=pose['yaw_rad'];cy,sy=math.cos(yaw),math.sin(yaw)
    polygon=[]
    for x,y in ((extent['front'],extent['left']),(-extent['rear'],extent['left']),(-extent['rear'],-extent['right']),(extent['front'],-extent['right'])):
        wx=pose['x_m']+cy*x-sy*y-ox;wy=pose['y_m']+sy*x+cy*y-oy
        polygon.append(((c*wx+s*wy)/r,(-s*wx+c*wy)/r))
    w,h=snapshot['width'],snapshot['height'];raw=snapshot['data'];unknown=None
    for y in range(math.floor(min(v[1] for v in polygon)),math.floor(max(v[1] for v in polygon))+1):
        for x in range(math.floor(min(v[0] for v in polygon)),math.floor(max(v[0] for v in polygon))+1):
            if not overlap(polygon,[(x,y),(x+1,y),(x+1,y+1),(x,y+1)]):continue
            if not(0<=x<w and 0<=y<h):unknown=('boundary',[x,y]);continue
            value=raw[y*w+x]
            if 65<=value<=100:return dict(clear=False,kind='occupied',cell=[x,y])
            if not 0<=value<=25:unknown=('unknown',[x,y])
    return dict(clear=unknown is None,kind=unknown[0] if unknown else 'clear',cell=unknown[1] if unknown else None)
