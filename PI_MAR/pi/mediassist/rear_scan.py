"""Rear-sector guard for manual M1 reverse. No ROS imports or motor commands."""
import math


def rear_clearance(scan, transform, stop_distance):
    """Use base-frame rear rays; missing coverage/invalid echoes never mean clear."""
    try:
        tx,ty,yaw=transform
        if not all(math.isfinite(v) for v in (tx,ty,yaw,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max)):
            return 'REAR_LIDAR_INVALID'
        if scan.angle_increment==0 or not 0 < scan.range_max or not 0 < len(scan.ranges)<=10000:
            return 'REAR_LIDAR_INVALID'
        selected=valid=0
        closest=float('inf')
        for i,distance in enumerate(scan.ranges):
            angle=scan.angle_min+i*scan.angle_increment+yaw
            rear_error=math.atan2(math.sin(angle-math.pi),math.cos(angle-math.pi))
            if abs(rear_error)>math.radians(35):continue
            selected+=1
            if math.isinf(distance) and distance>0 and scan.range_max>stop_distance:
                valid+=1;continue
            if not math.isfinite(distance) or not max(.01,scan.range_min)<=distance<=scan.range_max:continue
            valid+=1
            x,y=tx+distance*math.cos(angle),ty+distance*math.sin(angle)
            closest=min(closest,math.hypot(x,y))
        if closest<stop_distance:return 'REAR_OBSTACLE'
        coverage=abs(scan.angle_increment)*selected
        if selected<5 or coverage<math.radians(60) or valid<selected*.8:
            return 'REAR_LIDAR_NOT_READY'
        return 'CLEAR'
    except (AttributeError,TypeError,ValueError):
        return 'REAR_LIDAR_INVALID'


def chassis_guard(cmd,motion,rear_reason,rear_age):
    if abs(cmd[1])>.001 and not motion.get('allow_strafe',False):return (0.,0.,0.),'STRAFE_NOT_ENABLED'
    if cmd[0]>=0:return cmd,None
    if motion.get('chassis_model')!='MENTORPI_M1_MECANUM' or motion.get('allow_reverse') is not True:
        return (0.,0.,0.),'REVERSE_NOT_ENABLED'
    if rear_age>motion['sensor_max_age_sec']:
        return (0.,0.,0.),'STALE_REAR_LIDAR'
    if rear_reason!='CLEAR':return (0.,0.,0.),rear_reason
    return (max(cmd[0],-motion.get('max_reverse_speed_mps',.1)),cmd[1],cmd[2]),None
