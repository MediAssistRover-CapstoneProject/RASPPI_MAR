"""Local, continuously refreshed Mecanum input; uses the existing velocity gate."""
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import stat
import time
from .contracts import StepError


class ManualDrive:
    def __init__(self, cfg, control_source='gamepad', device_present=None):
        if control_source not in ('gamepad', 'auto'):
            raise ValueError('Manual drive source must be gamepad or auto')
        self.cfg=cfg
        self.control_source=control_source
        self._device_present=device_present or self._usb_present
        self._source_name='mqtt' if control_source=='auto' else 'gamepad'
        self._handoff_pending=False
        self.mqtt_resumed_at = None
        self.value=(0.,0.,0.)
        self.at=None
        self.owned=False
        self.battery_stopped=False

    @property
    def source_name(self):
        return self._source_name

    @property
    def handoff_pending(self):
        return self._handoff_pending

    def _usb_present(self):
        try:
            return stat.S_ISCHR(Path('/dev/input/js'+str(self.cfg['device_id'])).stat().st_mode)
        except OSError:
            return False

    def select(self, pad, now=None):
        """Reserve manual input for a plugged USB receiver; stale input stays zero.

        The receiver being present does not prove a paired controller is live.
        GamepadInput still requires fresh samples and a neutral arming interval.
        A source transition is latched until tick() stops the previous lease.
        """
        if self.control_source=='gamepad':
            return True
        try:
            present=bool(self._device_present())
        except OSError:
            present=False
        selected='gamepad' if present else 'mqtt'
        if selected!=self._source_name:
            self._source_name=selected
            self._handoff_pending=True
            self.value=(0.,0.,0.);self.at=None
            if present:
                pad.stop('CONTROL_SOURCE_CHANGED_RETURN_NEUTRAL')
            else:
                pad.reject('JOY_DISCONNECTED')
        return selected=='gamepad'

    def reset(self):
        self.value=(0.,0.,0.);self.at=None;self.owned=False

    def battery_allowed(self, adapter):
        """Inhibit only manual input; backend MOVE still owns charger return.

        A stale/unavailable estimate cannot trip or release the latch. Once
        tripped, fresh recovery and the existing neutral gate are both required.
        Read the estimator directly, including when MQTT telemetry is offline.
        """
        estimator=getattr(adapter,'battery',None)
        profile=getattr(estimator,'profile',None)
        if profile is None:
            return not self.battery_stopped
        sample=estimator.snapshot() or {}
        percent,age=sample.get('percent'),sample.get('age_sec')
        if (sample.get('estimate_status')=='ESTIMATED'
                and type(percent) in (int,float) and math.isfinite(percent) and 0<=percent<=100
                and type(age) in (int,float) and math.isfinite(age) and 0<=age<=profile['stale_after_sec']):
            if percent<=profile['manual_critical_stop_percent']:
                self.battery_stopped=True
            elif percent>=profile['manual_rearm_percent']:
                self.battery_stopped=False
        return not self.battery_stopped

    def busy(self, pad, now=None):
        now=time.monotonic() if now is None else now
        selected = self.select(pad, now)
        if self.owned or self.handoff_pending:
            return True
        # A stopped critical-battery stick cannot hold backend charger return.
        if self.battery_stopped:
            return False
        if not selected:
            return False
        with pad.lock:
            return pad.received is not None and 0<=now-pad.received<=pad.cfg['timeout_sec'] and any(abs(v)>.02 for v in pad.axes)

    def ramp(self, target, now, cap):
        if not any(target):
            self.value=(0.,0.,0.);self.at=None;return self.value
        dt=.05 if self.at is None else max(0.,min(.1,now-self.at));self.at=now
        speed=math.hypot(target[0],target[1])
        if speed>cap:target=(target[0]*cap/speed,target[1]*cap/speed,target[2])
        limits=(self.cfg['linear_accel']*dt,)*2+(self.cfg['angular_accel']*dt,)
        self.value=tuple(old+max(-limit,min(limit,new-old)) if new else 0.
                         for old,new,limit in zip(self.value,target,limits))
        # Clamp again after slewing: a diagonal turn must not exceed the vector cap.
        speed=math.hypot(*self.value[:2])
        if speed>cap:self.value=(self.value[0]*cap/speed,self.value[1]*cap/speed,self.value[2])
        return self.value

    def _report(self, manager, target, active, local, now):
        from std_msgs.msg import String
        pad=manager.gamepad
        status={**pad.status(),'source':self.source_name,'configured_source':self.control_source,
                'mapping_session_id':manager.state.get('session_id') if active else None,
                'local_manual':local,'requested_velocity':list(target),
                'source_handoff':self.handoff_pending}
        if status!=manager.gamepad_report or now-manager.gamepad_report_at>=1.:
            manager.gamepad_status.publish(String(data=json.dumps(status)))
            if manager.gamepad_report is None or status['reason']!=manager.gamepad_report.get('reason'):
                manager.adapter.node.get_logger().info('GAMEPAD '+status['reason'])
            manager.gamepad_report,manager.gamepad_report_at=status,now

    def _clear_remote(self, manager):
        remote=getattr(manager,'remote_control',None)
        if remote is not None:remote.stop()
        ramp=getattr(manager,'ramp',None)
        if ramp is not None:ramp.reset()

    def tick(self, manager, ready, idle):
        """Return whether this tick owns output, including a source-change stop."""
        from geometry_msgs.msg import Twist
        adapter,pad=manager.adapter,manager.gamepad
        now=time.monotonic();selected=self.select(pad,now)
        active=manager.active and manager.state.get('state')=='MAPPING'
        local=not self.cfg['session_required']
        if self.handoff_pending:
            # A held HTML pulse from before USB ownership must never resume on
            # unplug. Clear it even when a workflow currently owns the motors.
            self._clear_remote(manager)
            if idle:
                manager.velocity.publish(Twist());adapter.lease_message(False)
                self.reset();self._handoff_pending=False
                if not selected:
                    self.mqtt_resumed_at = datetime.now(timezone.utc)
                self._report(manager,(0.,0.,0.),active,local,now)
                return True
        if not selected and not self.handoff_pending:
            pad.reject('JOY_DISCONNECTED')
            self.reset();self._report(manager,(0.,0.,0.),active,local,now)
            return False
        battery_allowed=self.battery_allowed(adapter)
        allowed=bool(battery_allowed and idle and not adapter.motion_uncertain and not getattr(manager,'manual_inhibited',False)
                     and manager.state.get('state')!='RECOVERY_REQUIRED'
                     and (active or getattr(manager, 'manual_scope', None) or local and not manager.active)
                     and (ready or local))
        reason=('BATTERY_CRITICAL_RETURN_NEUTRAL' if idle and not battery_allowed else
                'WORKFLOW_OWNS_CONTROL' if not idle else 'WAIT_READY')
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
            snapshot=adapter.telemetry()
            sample=snapshot.get('odom_speed',{})
            allowed=sample.get('age_sec',99)<=.5;reason='ODOM_NOT_READY'
            gate=snapshot.get('gate',{}).get('value')
            if isinstance(gate,str):
                try:gate=json.loads(gate)
                except (ValueError,TypeError):gate=None
            if isinstance(gate,dict) and gate.get('reason')=='SOS_MEDIA_HOLD':
                allowed=False;reason='SOS_MEDIA_HOLD'
        if allowed and now-manager.last_graph_check>=.2:
            try:
                adapter._check_graph();manager.last_graph_check=now
            except Exception:
                allowed=False;reason='VELOCITY_GRAPH_NOT_READY'
        scope = getattr(manager, 'manual_scope', None)
        if scope is None:
            scope = 'local-manual' if local else manager.state.get('session_id') if active else None
        target=pad.current(scope,allowed,now,reason)
        if len(target)==2:target=(target[0],0.,target[1])
        # During a WF step, the existing action/stop owns the lease. Never
        # continuously publish zeros against an active navigation controller.
        if not idle:
            self._report(manager,target,active,local,now)
            if self.owned:
                manager.velocity.publish(Twist());adapter.lease_message(False)
            self.reset();return bool(selected or self.handoff_pending)
        cap=min(adapter.cfg['max_speed_mps'],self.cfg['forward_mps'])
        if allowed:
            motion = adapter.cfg['motion']
            x, y, yaw = target
            if x < 0 and motion.get('allow_reverse') is False:
                x = 0.
            if motion.get('allow_strafe') is False:
                y = 0.
            target=(x, y, max(-motion['max_angular_speed_rps'], min(yaw, motion['max_angular_speed_rps'])))
            limit=getattr(manager,'limit_velocity',None)
            if limit is not None:
                try:
                    target,limited_cap=limit(target,cap)
                    cap=min(cap,limited_cap)
                except StepError as error:
                    pad.stop(error.code);target=(0.,0.,0.);allowed=False
        else:target=(0.,0.,0.)
        self._report(manager,target,active,local,now)
        applied=self.ramp(target,now,cap)
        if allowed and limit is not None and any(applied):
            try:
                applied,cap=limit(applied,cap)
                size=math.hypot(*applied[:2])
                scale=min(1.,cap/max(size,1e-9))
                applied=(applied[0]*scale,applied[1]*scale,applied[2])
                self.value=applied
            except StepError as error:
                pad.stop(error.code)
                self.reset()
                applied=(0.,0.,0.)
        msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=applied
        manager.velocity.publish(msg)
        self.owned=bool(any(applied))
        adapter.lease_message(self.owned,cap)
        return True


def attach_gamepad(manager, mapping, callback=None):
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String
    from .gamepad import GamepadInput, configuration

    options = dict(configuration(mapping))
    if mapping.get('control_source') == 'auto' and options['max_run_sec'] == 0:
        # An RF receiver can repeat a held report even after the radio link drops.
        options['max_run_sec'] = 5.
    manager.gamepad = GamepadInput(options)
    manager.gamepad_status = manager.adapter.node.create_publisher(
        String, '/mediassist/gamepad/status', 1)
    manager.gamepad_report_at, manager.gamepad_report = 0., None
    manager.adapter.node.create_subscription(
        Joy, manager.gamepad.cfg['topic'], callback or (lambda msg: receive_gamepad(manager, msg)),
        qos_profile_sensor_data)


def receive_gamepad(manager, msg):
    stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
    age = (manager.adapter.node.get_clock().now().nanoseconds - stamp) / 1e9
    if not -.1 <= age <= manager.gamepad.cfg['timeout_sec']:
        manager.gamepad.reject('JOY_SOURCE_STALE')
        return
    manager.gamepad.feed(msg.axes, time.monotonic(), stamp)


class NavigationManual:
    """Manual input on a saved map while no workflow owns the chassis."""

    active = False

    def __init__(self, adapter, worker):
        self.adapter = adapter
        self.context = worker['navigation_session']
        self.manual_scope = self.context['session_id']
        self.state = {}
        self.velocity = adapter.navigation_velocity_output
        self.manual_inhibited = False
        self.last_graph_check = 0.
        attach_gamepad(self, worker['mapping'])
        self.manual_drive = ManualDrive(self.gamepad.cfg, worker['mapping']['control_source'])

    async def tick(self, ready, idle):
        self.manual_drive.tick(self, ready, idle)

    def limit_velocity(self, target, cap):
        from .navigation_session import command_context

        state = self.adapter.telemetry()
        pose = state.get('map_pose', {})
        current = pose.get('value') or {}
        if not pose.get('fresh') or current.get('navigation_session_id') != self.context['session_id']:
            raise StepError('LOCALIZATION_NOT_READY')
        command_context({'navigationContext': {
            **self.context,
            'initialization_revision': current.get('initialization_revision'),
            'profile_hash': getattr(self.adapter, 'navigation_profile_hash', None),
        }}, self.adapter.worker, self.adapter.cfg, self.adapter, state=state)
        motion = self.adapter.cfg['motion']
        cap = min(cap, self.adapter.cfg['navigation'].get('max_speed_mps', cap))
        x, y, yaw = target
        if x < 0:
            x = max(x, -motion.get('max_reverse_speed_mps', 0.)) if motion.get('allow_reverse') else 0.
        if not motion.get('allow_strafe'):
            y = 0.
        yaw_cap = self.adapter.cfg['navigation'].get('max_angular_speed_rps', motion['max_angular_speed_rps'])
        target = (x, y, max(-yaw_cap, min(yaw, yaw_cap)))
        zones = self.adapter.zones
        if zones is not None:
            cap = zones.directional_cap(pose, cap, target)
        return ((0., 0., 0.) if cap <= 0. else target), cap
