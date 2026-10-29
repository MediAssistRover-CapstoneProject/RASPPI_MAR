"""Frontier goals use Humble Nav2; its velocities stay isolated until selected by MAR."""
import math,time
from pathlib import Path
from .contracts import StepError

class MappingNavigator:
    def __init__(self,adapter,exploration):
        self.adapter=adapter;self.exploration=exploration;self.target=None;self.result=None;self.started=0.
        self.best_distance=math.inf;self.progress_at=0.;self.velocity_missing_at=None
    @property
    def active(self):return self.target is not None
    async def stop(self):
        if self.target is None:return
        try:
            if not await self.adapter.stop():raise StepError('GOAL_STOP_UNCONFIRMED')
        finally:self.target=None;self.result=None
    async def tick(self,plan,pose,now):
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        if self.result is not None and self.result.done():
            terminal=self.result.result();target=self.target
            if not await self.adapter.stop():raise StepError('GOAL_STOP_UNCONFIRMED')
            self.target=None;self.result=None
            self.exploration.exclude_goal(target,now,ttl=5. if terminal.status==4 else 30.,
                                          radius=.12 if terminal.status==4 else .35)
            return (0.,0.,0.),'OBSERVING_FRONTIER' if terminal.status==4 else 'REPLANNING_UNREACHABLE_FRONTIER'
        if self.target is None:
            goal=plan['goal']
            if math.hypot(goal['x_m']-pose['x_m'],goal['y_m']-pose['y_m'])<.06:
                self.exploration.exclude_goal(goal,now,ttl=5.,radius=.08);return (0.,0.,0.),'OBSERVING_FRONTIER'
            if not self.adapter.nav.server_is_ready():raise StepError('NAV2_MAPPING_NOT_READY')
            msg=PoseStamped();msg.header.frame_id=self.adapter.cfg['ros']['map_frame'];msg.header.stamp=self.adapter.node.get_clock().now().to_msg()
            msg.pose.position.x=float(goal['x_m']);msg.pose.position.y=float(goal['y_m'])
            msg.pose.orientation.z=math.sin(goal['yaw_rad']/2);msg.pose.orientation.w=math.cos(goal['yaw_rad']/2)
            request=NavigateToPose.Goal();request.pose=msg
            request.behavior_tree=str(Path(__file__).resolve().parents[1]/'config/navigate_no_recovery.xml')
            self.adapter.clear_navigation_velocity()
            sent=self.adapter.nav.send_goal_async(request)
            try:handle=await self.adapter.future(sent,timeout=2)
            except BaseException:sent.add_done_callback(self.adapter._late_cancel);raise
            if not handle.accepted:
                self.exploration.exclude_goal(goal,now,ttl=30.);return (0.,0.,0.),'REPLANNING_REJECTED_FRONTIER'
            self.adapter.goal=handle;self.target=dict(goal);self.result=handle.get_result_async();self.started=now
            self.best_distance=math.hypot(goal['x_m']-pose['x_m'],goal['y_m']-pose['y_m']);self.progress_at=now;self.velocity_missing_at=None
        distance=math.hypot(self.target['x_m']-pose['x_m'],self.target['y_m']-pose['y_m'])
        if distance<self.best_distance-.04:self.best_distance=distance;self.progress_at=now
        if now-self.progress_at>20. or now-self.started>60.:
            target=dict(self.target);await self.stop();self.exploration.exclude_goal(target,now,ttl=30.)
            return (0.,0.,0.),'REPLANNING_STALLED_FRONTIER'
        velocity,fresh=self.adapter.navigation_velocity()
        if not fresh:
            if self.velocity_missing_at is None:self.velocity_missing_at=now
            if now-self.velocity_missing_at>3. and now-self.started>5.:
                target=dict(self.target);await self.stop();self.exploration.exclude_goal(target,now,ttl=30.)
                return (0.,0.,0.),'REPLANNING_STALE_NAV2'
            return (0.,0.,0.),'WAITING_FOR_NAV2_VELOCITY'
        self.velocity_missing_at=None
        return velocity,'FOLLOW_FRONTIER_NAV2' if fresh else 'WAITING_FOR_NAV2_VELOCITY'
