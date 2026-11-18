"""Final scan-based translation cap shared by mapping, MOVE and manual sources."""
import math


def swept_turn_clear(velocity,points,footprint,margin,age):
    """Check the commanded arc until braking, rather than a full 360 circle."""
    x,y,w=velocity
    horizon=age+.35+max(math.hypot(x,y)/.6,abs(w)/1.2)/2.
    # Margin expands each edge, so its corner radius is not physical_radius
    # plus margin. Use the expanded corner for both filtering and sampling.
    radius=math.hypot(max(footprint['front'],footprint['rear'])+margin,
                      max(footprint['left'],footprint['right'])+margin)
    travel=(math.hypot(x,y)+radius*abs(w))*horizon
    steps=max(1,math.ceil(travel/.01))
    if steps>256:return False
    bound=travel/steps/2.
    # The sample bound expands both rectangle axes. Its corner can move by
    # sqrt(2)*bound; a radial +bound filter would discard those sample corners.
    relevant=[(a,b) for a,b in points if math.hypot(a,b)<=radius+math.hypot(x,y)*horizon+math.sqrt(2)*bound]
    for i in range(steps+1):
        t=horizon*i/steps;a=w*t;c,s=math.cos(a),math.sin(a)
        if abs(w)<1e-9:dx,dy=x*t,y*t
        else:dx,dy=(x*s+y*(c-1))/w,(x*(1-c)+y*s)/w
        for px,py in relevant:
            rx,ry=c*(px-dx)+s*(py-dy),-s*(px-dx)+c*(py-dy)
            if (-footprint['rear']-margin-bound<=rx<=footprint['front']+margin+bound
                    and -footprint['right']-margin-bound<=ry<=footprint['left']+margin+bound):return False
    return True


def limit_turn(velocity,view,age,profile):
    if not velocity[2]:return velocity,None
    points=view.get('obstacle_points')
    if profile.get('vehicle')!='rigid' or points is None:
        return (velocity,None) if view.get('rotation_clear',False) else ((0.,0.,0.),'TURN_ENVELOPE_BLOCKED')
    def clear(scale):return swept_turn_clear(tuple(v*scale for v in velocity),points,profile['footprint'],profile['stop_margin_m'],age)
    if clear(1.):return velocity,None
    lo,hi=0.,1.
    for _ in range(7):
        mid=(lo+hi)/2
        if clear(mid):lo=mid
        else:hi=mid
    result=tuple(v*lo for v in velocity)
    if math.hypot(result[0],result[1])<=.01 and abs(result[2])<.1:return (0.,0.,0.),'TURN_ENVELOPE_BLOCKED'
    # Preserve the curve while slowing. Never remove yaw and drive straight.
    return result,'TURN_SWEEP_SLOWDOWN' if lo>0. else 'TURN_ENVELOPE_BLOCKED'


def translation_entry(point,direction,footprint,margin):
    enter,leave=0.,math.inf
    for value,axis,low,high in ((point[0],direction[0],-footprint['rear']-margin,footprint['front']+margin),
                               (point[1],direction[1],-footprint['right']-margin,footprint['left']+margin)):
        if abs(axis)<1e-12:
            if not low<=value<=high:return math.inf
        else:
            first,last=sorted(((value-high)/axis,(value-low)/axis))
            enter=max(enter,first);leave=min(leave,last)
    return enter if enter<=leave else math.inf


def padding_escape_contacts(points,direction,footprint,margin):
    """Only a straight creep increasing distance from every existing contact.

    Squared distance to a convex rectangle along a translation is convex. A
    positive initial derivative proves it keeps increasing. Points outside
    the initial padding still use full normal braking; physical contact and
    tangential skimming never get this exception. Map/zone sweeps are checked
    independently by the recovery planner.
    """
    contacts=set()
    for index,(x,y) in enumerate(points):
        if not (-footprint['rear']-margin<=x<=footprint['front']+margin
                and -footprint['right']-margin<=y<=footprint['left']+margin):continue
        dx=x-min(footprint['front'],max(-footprint['rear'],x))
        dy=y-min(footprint['left'],max(-footprint['right'],y))
        if math.hypot(dx,dy)<=.001 or -(dx*direction[0]+dy*direction[1])<=1e-8:return None
        contacts.add(index)
    return contacts or None

def limit_velocity(velocity,view,age,profile):
    if (not view or not view.get('valid') or not math.isfinite(age)
            or not 0<=age<=profile['scan_max_age_sec']):
        return (0.,0.,0.),'BRAKING_SCAN_NOT_READY'
    if view.get('nearest_lidar_m',0.)<=profile['lidar_stop_distance_m']:
        return (0.,0.,0.),'LIDAR_OBSTACLE_WITHIN_STOP_DISTANCE'
    x,y,z=velocity;speed=math.hypot(x,y)
    if speed<1e-9:
        return limit_turn((0.,0.,z),view,age,profile)
    if 'obstacle_points' not in view:return (0.,0.,0.),'BRAKING_SCAN_NOT_READY'
    ux,uy=x/speed,y/speed
    f=profile['footprint'];margin=profile['stop_margin_m']
    points=view['obstacle_points'];gap=math.inf
    for point in points:
        # Intersect the point's relative ray with the expanded body. A rotated
        # bounding box in along/cross coordinates also covers empty corners;
        # it can falsely stop a diagonal escape moving away from an obstacle.
        gap=min(gap,translation_entry(point,(ux,uy),f,margin))
    escaping=False
    if gap==0. and profile.get('vehicle')=='rigid' and abs(z)<1e-9 and speed<=.045000001:
        contacts=padding_escape_contacts(points,(ux,uy),f,margin)
        if contacts:
            gap=min((translation_entry(point,(ux,uy),f,margin)
                     for index,point in enumerate(points) if index not in contacts),default=math.inf)
            escaping=True
    # Same 0.6 m/s^2 deceleration as Nav2; include sensing + command latency.
    reaction=age+.35
    cap=max(0.,math.sqrt((.6*reaction)**2+1.2*gap)-.6*reaction)
    # Near-field creep is additional to stopping-distance braking, never a
    # reason to keep full speed until only a few centimetres remain.
    slow=profile.get('near_slow_margin_m',margin)
    if slow>margin:cap=min(cap,profile['speed_mps']*min(1.,gap/(slow-margin)))
    scale=min(1.,cap/speed,profile['speed_mps']/speed)
    if speed*scale<1e-6:scale=0.
    # Preserve curvature while slowing a moving arc. A stationary turn keeps
    # its independent swept-body check above.
    z*=scale
    if speed*scale<=.01:
        from .safety import angular_deadband
        z=angular_deadband(z)
    limited,reason=limit_turn((x*scale,y*scale,z),view,age,profile)
    if not any(limited) and speed>.005 and reason is None:reason='LIDAR_BRAKING_STOP'
    elif any(limited) and escaping and reason is None:reason='LIDAR_PADDING_ESCAPE'
    return limited,reason
