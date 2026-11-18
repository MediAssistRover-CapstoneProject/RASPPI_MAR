"""Raster/convex-polygon intersection for the actual oriented body; no ROS or motion."""
import math

# Extra clearance beyond each physical body edge; shared by Nav2, zone sweep
# validation and endpoint validation. Grid discretization can add more space.
NAVIGATION_PADDING_M=.01

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
    x0=math.floor(min(v[0] for v in polygon));x1=math.floor(max(v[0] for v in polygon))
    y0=math.floor(min(v[1] for v in polygon));y1=math.floor(max(v[1] for v in polygon))
    for y in range(y0,y1+1):
        for x in range(x0,x1+1):
            inside=0<=x<w and 0<=y<h
            if inside:
                value=raw[y*w+x]
                # Free floor cannot invalidate the body, whether it intersects
                # the polygon or not. Reserve exact SAT for blocked/boundary
                # cells while preserving their original scan order and reason.
                if 0<=value<=25:continue
            if not overlap(polygon,[(x,y),(x+1,y),(x+1,y+1),(x,y+1)]):continue
            if not inside:unknown=('boundary',[x,y]);continue
            if 65<=value<=100:return dict(clear=False,kind='occupied',cell=[x,y])
            if not 0<=value<=25:unknown=('unknown',[x,y])
    return dict(clear=unknown is None,kind=unknown[0] if unknown else 'clear',cell=unknown[1] if unknown else None)


def cached_body_clear(grid,pose,extent):
    """Fast free boxes on an immutable grid snapshot; exact SAT near walls."""
    from array import array
    raw=grid['data'];w,h=grid['width'],grid['height']
    if not isinstance(raw,bytes):return body_clear(grid,pose,extent)
    cache=grid.get('_body_blocked_prefix')
    if cache is None or cache[0] is not raw or cache[1:3]!=(w,h):
        prefix=array('I',[0])*((w+1)*(h+1));stride=w+1
        for row in range(h):
            count=0;off=(row+1)*stride;above=row*stride
            for col in range(w):
                count+=not 0<=raw[row*w+col]<=25
                prefix[off+col+1]=prefix[above+col+1]+count
        cache=(raw,w,h,prefix);grid['_body_blocked_prefix']=cache
    prefix=cache[3];r=grid['resolution'];ox,oy,a=grid['origin'];c,s=math.cos(a),math.sin(a)
    cy,sy=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad']);points=[]
    for x,y in ((extent['front'],extent['left']),(-extent['rear'],extent['left']),(-extent['rear'],-extent['right']),(extent['front'],-extent['right'])):
        dx=pose['x_m']+cy*x-sy*y-ox;dy=pose['y_m']+sy*x+cy*y-oy
        points.append(((c*dx+s*dy)/r,(-s*dx+c*dy)/r))
    x0=math.floor(min(x for x,y in points));x1=math.floor(max(x for x,y in points))+1
    y0=math.floor(min(y for x,y in points));y1=math.floor(max(y for x,y in points))+1
    if 0<=x0<x1<=w and 0<=y0<y1<=h:
        stride=w+1
        count=prefix[y1*stride+x1]-prefix[y0*stride+x1]-prefix[y1*stride+x0]+prefix[y0*stride+x0]
        if count==0:return dict(clear=True,kind='clear',cell=None)
    return body_clear(grid,pose,extent)
