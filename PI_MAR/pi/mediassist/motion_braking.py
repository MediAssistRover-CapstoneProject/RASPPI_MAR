"""Final scan-based translation cap shared by mapping, MOVE and manual sources."""
import math

def limit_velocity(velocity,view,age,profile):
    if (not view or not view.get('valid') or not math.isfinite(age)
            or not 0<=age<=profile['scan_max_age_sec']):
        return (0.,0.,0.),'BRAKING_SCAN_NOT_READY'
    if view.get('nearest_lidar_m',0.)<=profile['lidar_stop_distance_m']:
        return (0.,0.,0.),'LIDAR_OBSTACLE_WITHIN_STOP_DISTANCE'
    x,y,z=velocity;speed=math.hypot(x,y)
    turn_blocked=bool(z and not view.get('rotation_clear',False))
    if turn_blocked:
        z=0.
    if speed<1e-9:
        return (0.,0.,z), 'TURN_ENVELOPE_BLOCKED' if z!=velocity[2] else None
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
    return (x*scale,y*scale,z),'TURN_ENVELOPE_BLOCKED' if turn_blocked else None
