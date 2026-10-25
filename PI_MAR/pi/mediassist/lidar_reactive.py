"""Vendor-style LiDAR escape policy selected inside the single mapping velocity owner."""
from .autonomous_mapping import nav2_assist

class LidarReactive:
    def __init__(self):self.run=None;self.side=1.;self.until=0.

    def current(self,view,cfg,age,now,run):
        if run!=self.run:self.run=run;self.until=0.
        front=view['front_m'];left=view.get('lateral_left_m',view['left_m']);right=view.get('lateral_right_m',view['right_m'])
        margin=cfg['stop_margin_m'];speed=cfg['speed_mps']
        if front>cfg['turn_clearance_m']:
            return nav2_assist((speed,0.,0.),view,cfg,age),'LIDAR_FORWARD'
        if now>=self.until:
            self.side=1. if left>=right else -1.;self.until=now+.5
        chosen=left if self.side>0 else right
        other=right if self.side>0 else left
        if chosen<=margin and other>margin:
            self.side=-self.side;chosen=other
        if view.get('rotation_clear'):
            return (0.,0.,self.side*cfg['angular_rps']),'LIDAR_TURN_TO_CLEAR_SIDE'
        if chosen>margin:
            velocity=nav2_assist((0.,self.side*speed,0.),view,cfg,age)
            return velocity,'LIDAR_SIDE_ESCAPE'
        return (0.,0.,0.),'LIDAR_BLOCKED_WAITING_FOR_CLEARANCE'
