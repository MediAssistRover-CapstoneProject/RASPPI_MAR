"""Local, continuously refreshed Mecanum input; uses the existing velocity gate."""
import json
import math
import time


class ManualDrive:
    def __init__(self, cfg):
        self.cfg=cfg
        self.value=(0.,0.,0.)
        self.at=None
        self.owned=False

    def reset(self):
        self.value=(0.,0.,0.);self.at=None;self.owned=False

    def busy(self, pad, now=None):
        now=time.monotonic() if now is None else now
        with pad.lock:
            return pad.received is not None and 0<=now-pad.received<=pad.cfg['timeout_sec'] and any(abs(v)>.02 for v in pad.axes)

    def ramp(self, target, now, cap):
        if not any(target):
            self.value=(0.,0.,0.);self.at=None;return self.value
        dt=.05 if self.at is None else max(0.,min(.1,now-self.at));self.at=now
        speed=math.hypot(target[0],target[1])
        if speed>cap:target=(target[0]*cap/speed,target[1]*cap/speed,target[2])
        limits=(self.cfg['linear_accel']*dt,)*2+(self.cfg['angular_accel']*dt,)
        self.value=tuple(old+max(-limit,min(limit,new-old)) for old,new,limit in zip(self.value,target,limits))
        # Clamp again after slewing: a diagonal turn must not exceed the vector cap.
        speed=math.hypot(*self.value[:2])
        if speed>cap:self.value=(self.value[0]*cap/speed,self.value[1]*cap/speed,self.value[2])
        return self.value

    def tick(self, manager, ready, idle):
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        adapter,pad=manager.adapter,manager.gamepad
        now=time.monotonic()
        active=manager.active and manager.state.get('state')=='MAPPING'
        local=not self.cfg['session_required']
        allowed=bool(idle and not adapter.motion_uncertain and not getattr(manager,'manual_inhibited',False)
                     and manager.state.get('state')!='RECOVERY_REQUIRED'
                     and (active or local and not manager.active) and (ready or local))
        reason='WAIT_READY'
        if allowed:
            publishers=adapter.node.get_publishers_info_by_topic(self.cfg['topic'])
            allowed=len(publishers)==1 and publishers[0].node_name=='mediassist_gamepad_joy'
            reason='JOY_PUBLISHER_COUNT'
        if allowed:
            allowed=all(adapter.cfg['motion'][k] for k in ('commissioned','cart_footprint_validated','watchdog_validated'))
            reason='MOTION_NOT_COMMISSIONED'
        if allowed:
            allowed=adapter.health()['sensors_ready'];reason='SENSORS_NOT_READY'
        if allowed:
            sample=adapter.telemetry().get('odom_speed',{})
            allowed=sample.get('age_sec',99)<=.5;reason='ODOM_NOT_READY'
        if allowed and now-manager.last_graph_check>=.2:
            try:
                adapter._check_graph();manager.last_graph_check=now
            except Exception:
                allowed=False;reason='VELOCITY_GRAPH_NOT_READY'
        scope='local-manual' if local else manager.state.get('session_id') if active else None
        target=pad.current(scope,allowed,now,reason)
        status={**pad.status(),'mapping_session_id':manager.state.get('session_id') if active else None,
                'local_manual':local,'requested_velocity':list(target)}
        if status!=manager.gamepad_report or now-manager.gamepad_report_at>=1.:
            manager.gamepad_status.publish(String(data=json.dumps(status)))
            if manager.gamepad_report is None or status['reason']!=manager.gamepad_report.get('reason'):
                adapter.node.get_logger().info('GAMEPAD '+status['reason'])
            manager.gamepad_report,manager.gamepad_report_at=status,now
        # During a WF step, the existing action/stop owns the lease. Never
        # continuously publish zeros against an active navigation controller.
        if not idle:
            if self.owned:
                manager.velocity.publish(Twist());adapter.lease_message(False)
            self.reset();return
        cap=min(adapter.cfg['max_speed_mps'],self.cfg['forward_mps'])
        if allowed:
            target=(target[0],target[1],max(-adapter.cfg['motion']['max_angular_speed_rps'],
                                          min(target[2],adapter.cfg['motion']['max_angular_speed_rps'])))
        else:target=(0.,0.,0.)
        applied=self.ramp(target,now,cap)
        msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=applied
        manager.velocity.publish(msg)
        self.owned=bool(any(applied))
        adapter.lease_message(self.owned,cap)
