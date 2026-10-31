"""Session-scoped joystick input. No motor publisher, network access or ROS imports."""
import math
import threading

from .contracts import StepError


def configuration(mapping):
    value = mapping.setdefault('gamepad', {})
    if not isinstance(value, dict):
        raise StepError('INVALID_GAMEPAD_CONFIG')
    defaults = {'device_id': 0, 'topic': '/mediassist/gamepad/input',
                'linear_axis': 1, 'angular_axis': 0, 'linear_sign': 1, 'angular_sign': 1,
                'layout': 'legacy', 'session_required': True, 'lateral_axis': 0, 'lateral_sign': 1,
                'lateral_mps': .10, 'linear_accel': .6, 'angular_accel': 2.,
                'deadzone': .15, 'forward_mps': .10, 'reverse_mps': .08,
                'angular_rps': .35, 'timeout_sec': .25, 'neutral_hold_sec': .30, 'max_run_sec': 3.}
    for key, default in defaults.items():
        value.setdefault(key, default)
    if value['layout'] not in ('legacy','vendor_mecanum') or type(value['session_required']) is not bool:
        raise StepError('INVALID_GAMEPAD_CONFIG', 'layout/session_required')
    for key in ('device_id', 'linear_axis', 'angular_axis', 'lateral_axis'):
        if type(value[key]) is not int or not 0 <= value[key] <= 15:
            raise StepError('INVALID_GAMEPAD_CONFIG', key)
    if value['linear_axis'] == value['angular_axis']:
        raise StepError('INVALID_GAMEPAD_CONFIG', 'axes must differ')
    if value['layout']=='vendor_mecanum' and len({value[k] for k in ('linear_axis','angular_axis','lateral_axis')})!=3:
        raise StepError('INVALID_GAMEPAD_CONFIG','three separate axes required')
    for key in ('linear_sign', 'angular_sign', 'lateral_sign'):
        if type(value[key]) is not int or value[key] not in (-1, 1):
            raise StepError('INVALID_GAMEPAD_CONFIG', key)
    if value['topic'] != '/mediassist/gamepad/input':
        raise StepError('INVALID_GAMEPAD_CONFIG', 'dedicated input topic required')
    extended=value['layout']=='vendor_mecanum'
    for key, low, high in (('deadzone', .05, .30), ('forward_mps', .02, .30 if extended else .15),
                           ('reverse_mps', .02, .30 if extended else .10), ('angular_rps', .10, 2.0 if extended else .50),
                           ('lateral_mps', .02, .30),('linear_accel',.25,1.),('angular_accel',.8,3.),
                           ('timeout_sec', .10, .25), ('neutral_hold_sec', .20, 1.), ('max_run_sec', 0., 5.)):
        number = value[key]
        if type(number) not in (int, float) or not math.isfinite(number) or not low <= number <= high:
            raise StepError('INVALID_GAMEPAD_CONFIG', key)
    return value


def driver_argv(mapping):
    cfg = configuration(mapping)
    return ['ros2', 'run', 'joy', 'joy_node', '--ros-args',
            '-r', '__node:=mediassist_gamepad_joy', '-r', '/joy:=' + cfg['topic'],
            '-p', 'device_id:=' + str(cfg['device_id']),
            '-p', 'deadzone:=' + str(cfg['deadzone']), '-p', 'autorepeat_rate:=20.0']


class GamepadInput:
    def __init__(self, cfg):
        self.cfg = dict(cfg)
        self.lock = threading.RLock()
        self.session = None
        self.axes = (0., 0., 0.) if cfg['layout']=='vendor_mecanum' else (0.,0.)
        self.received = None
        self.last_stamp = None
        self.armed = False
        self.neutral_since = None
        self.reason = 'WAIT_MAPPING'
        self.moving_since = None

    def stop(self, reason='WEB_STOP_RETURN_NEUTRAL'):
        with self.lock:
            self.armed = False
            self.neutral_since = None
            self.reason = reason
            self.moving_since = None

    def feed(self, axes, now, stamp=None):
        with self.lock:
            try:
                if not 2 <= len(axes) <= 32 or any(not math.isfinite(v) or abs(v) > 1.001 for v in axes):
                    raise ValueError('axes')
                # Equal/older source stamps cannot keep an old command alive.
                if stamp is not None and self.last_stamp is not None and stamp <= self.last_stamp:
                    raise ValueError('stamp')
                x = axes[self.cfg['linear_axis']] * self.cfg['linear_sign']
                z = axes[self.cfg['angular_axis']] * self.cfg['angular_sign']
                y = axes[self.cfg['lateral_axis']] * self.cfg['lateral_sign'] if self.cfg['layout']=='vendor_mecanum' else None
            except (ValueError, TypeError, IndexError):
                self.received = None
                self.stop('JOY_INVALID')
                return
            if self.received is None or now - self.received > self.cfg['timeout_sec']:
                self.stop('RETURN_NEUTRAL')
            self.axes = tuple(max(-1.,min(1.,v)) for v in ((x,y,z) if y is not None else (x,z)))
            self.received, self.last_stamp = now, stamp

    def reject(self, reason):
        with self.lock:
            self.received = None
            self.stop(reason)

    def current(self, session, allowed, now, blocked_reason='WAIT_READY'):
        with self.lock:
            zero=(0.,)*len(self.axes)
            if session != self.session:
                self.session = session
                self.stop('RETURN_NEUTRAL')
            if not session or not allowed:
                self.stop(blocked_reason if session else 'WAIT_MAPPING')
                return zero
            if self.received is None or not 0 <= now - self.received <= self.cfg['timeout_sec']:
                self.stop('JOY_STALE')
                return zero
            x,z=self.axes[0],self.axes[-1]
            neutral = all(abs(v)<=.02 for v in self.axes)
            if not self.armed:
                if not neutral:
                    self.neutral_since = None
                    return zero
                if self.neutral_since is None:
                    self.neutral_since = now
                if now - self.neutral_since < self.cfg['neutral_hold_sec']:
                    self.reason = 'RETURN_NEUTRAL'
                    return zero
                self.armed = True
            if neutral:
                self.moving_since = None
            elif self.moving_since is None:
                self.moving_since = now
            elif self.cfg['max_run_sec']>0 and now - self.moving_since >= self.cfg['max_run_sec']:
                # An RF dongle may keep autorepeating its last HID state after radio loss.
                self.stop('RUN_LIMIT_RETURN_NEUTRAL')
                return zero
            self.reason = 'READY' if neutral else 'DRIVING'
            vx=0. if abs(x)<=.02 else x*(self.cfg['forward_mps'] if x>0 else self.cfg['reverse_mps'])
            wz=0. if abs(z)<=.02 else z*self.cfg['angular_rps']
            if len(self.axes)==3:
                y=self.axes[1]
                return vx,0. if abs(y)<=.02 else y*self.cfg['lateral_mps'],wz
            return vx,wz

    def status(self):
        with self.lock:
            return {'source': 'gamepad', 'reason': self.reason, 'armed': self.armed,
                    'mapping_session_id': self.session}
