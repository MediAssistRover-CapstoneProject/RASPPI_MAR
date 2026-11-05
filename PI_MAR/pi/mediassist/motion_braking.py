"""Final scan-based translation cap shared by mapping, MOVE and manual sources."""
import math


def swept_turn_clear(velocity,points,footprint,margin,age):
    """Check the commanded arc until braking, rather than a full 360 circle."""
    x,y,w=velocity
    horizon=age+.35+max(math.hypot(x,y)/.6,abs(w)/1.2)/2.
    radius=math.hypot(max(footprint['front'],footprint['rear']),max(footprint['left'],footprint['right']))
    travel=(math.hypot(x,y)+radius*abs(w))*horizon
    steps=max(1,math.ceil(travel/.01))
    if steps>256:return False
    bound=travel/steps/2.
    relevant=[(a,b) for a,b in points if math.hypot(a,b)<=radius+margin+math.hypot(x,y)*horizon+bound]
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
    corners=[(a,b) for a in (-f['rear'],f['front']) for b in (-f['right'],f['left'])]
    front=max(a*ux+b*uy for a,b in corners)
    lo=min(-a*uy+b*ux for a,b in corners)-margin
    hi=max(-a*uy+b*ux for a,b in corners)+margin
    gap=math.inf
    for a,b in view['obstacle_points']:
        along=a*ux+b*uy;cross=-a*uy+b*ux
        if along>=0 and lo<=cross<=hi:gap=min(gap,max(0.,along-front-margin))
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
    return limited,reason
