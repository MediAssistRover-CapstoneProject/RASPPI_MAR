"""Enable camera QR delivery after validating the existing MentorPi PWM readback driver."""
import argparse
import json
import os
from pathlib import Path
import stat
import tempfile

from configure_mapping import ROOT, load, stopped_configuration

OLD_NODE = '''    def get_pwm_servo_state(self, msg):
        states = []
        for i in msg.cmd:
            data = PWMServoState()
            if i.get_position:
                state = self.board.pwm_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.pwm_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            states.append(data)
        return [True, states]
'''

NEW_NODE = '''    MAR_BOUNDED_PWM_READBACK = True

    def get_pwm_servo_state(self, request, response):
        # Limit work in the shared controller executor.
        if len(request.cmd) != 1:
            response.success = False
            return response
        item = request.cmd[0]
        state = PWMServoState()
        state.id = [item.id]
        if item.get_position:
            position = self.board.pwm_servo_read_position(item.id)
            if position is None:
                response.success = False
                return response
            state.position = [position]
        if item.get_offset:
            offset = self.board.pwm_servo_read_offset(item.id)
            if offset is None:
                response.success = False
                return response
            state.offset = [offset]
        response.state = [state]
        response.success = bool(item.get_position or item.get_offset)
        return response
'''

OLD_SDK = '''    def pwm_servo_read_and_unpack(self, servo_id, cmd, unpack):
        with self.servo_read_lock:
            self.buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, [cmd, servo_id])
            data = self.pwm_servo_queue.get(block=True)
            servo_id, cmd, info = struct.unpack(unpack, data)
            return info
'''

NEW_SDK = '''    def pwm_servo_read_and_unpack(self, servo_id, cmd, unpack):
        from queue import Empty

        with self.servo_read_lock:
            # Discard a bounded number of late responses before this query.
            for _ in range(8):
                try:
                    self.pwm_servo_queue.get_nowait()
                except Empty:
                    break
            else:
                return None
            self.buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, [cmd, servo_id])
            try:
                data = self.pwm_servo_queue.get(timeout=0.35)
                reply_id, reply_cmd, value = struct.unpack(unpack, data)
            except (Empty, struct.error):
                return None
            return value if reply_id == servo_id and reply_cmd == cmd else None
'''


def driver_changes(workspace):
    source = workspace / 'src/driver/ros_robot_controller/ros_robot_controller'
    installed = list((workspace / 'install/ros_robot_controller/lib').glob('python*/site-packages/ros_robot_controller'))
    if len(installed) != 1:
        raise RuntimeError('Expected one installed MentorPi controller package')
    changes = {}
    for package in (source, installed[0]):
        for name, original, replacement in (
                ('ros_robot_controller_node.py', OLD_NODE, NEW_NODE),
                ('ros_robot_controller_sdk.py', OLD_SDK, NEW_SDK)):
            path = (package / name).resolve(strict=True)
            if not path.is_relative_to(workspace) or not path.is_file():
                raise RuntimeError('Controller file is outside the vendor workspace')
            if path in changes:
                continue
            raw = path.read_bytes()
            code = raw.decode('utf-8')
            if replacement in code:
                continue
            if code.count(original) != 1:
                raise RuntimeError(f'Inspect the installed vendor version before patching {name}')
            candidate = code.replace(original, replacement, 1)
            compile(candidate, str(path), 'exec')
            changes[path] = (raw, candidate.encode('utf-8'))
    return changes


def replace_bytes(path, data):
    mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o644
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            os.fchmod(stream.fileno(), mode)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        Path(name).unlink(missing_ok=True)


def prepare_changes(config, workspace):
    cfg, hardware = load(config)
    if (cfg['compartments'].get('CART_MAIN') != 'cargo_lock' or not hardware['servo']['commissioned']
            or 'CONTROL_HARDWARE' not in cfg['enabled_step_types'] or not cfg['mapping']['camera']['enabled']):
        raise RuntimeError('Commission CART_MAIN, CONTROL_HARDWARE and the camera before enabling staff delivery')
    from pyzbar.pyzbar import ZBarSymbol, decode
    if not callable(decode) or ZBarSymbol.QRCODE is None:
        raise RuntimeError('MentorPi pyzbar/ZBar QR support is unavailable')
    changes = driver_changes(workspace)
    raw = config.read_bytes()
    document = json.loads(raw)
    if document.get('unlock_policy') != 'STAFF_VERIFIED':
        document['unlock_policy'] = 'STAFF_VERIFIED'
        changes[config] = (raw, (json.dumps(document, ensure_ascii=False, indent=2) + '\n').encode('utf-8'))
    return changes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'pi/config/mqtt.worker.json')
    parser.add_argument('--vendor-workspace', type=Path, default=Path('/home/ubuntu/ros2_ws'))
    parser.add_argument('--apply', action='store_true', help='Apply after stopping the robot stack')
    args = parser.parse_args()
    config = args.config.resolve(strict=True)
    workspace = args.vendor_workspace.resolve(strict=True)

    if not args.apply:
        print('Staff delivery preflight passed. Files to update:')
        for path in prepare_changes(config, workspace):
            print(path)
        return
    with stopped_configuration(config):
        changes = prepare_changes(config, workspace)
        written = []
        try:
            for path, (original, candidate) in changes.items():
                if path.read_bytes() != original:
                    raise RuntimeError(f'File changed since preflight: {path}')
                replace_bytes(path, candidate)
                written.append((path, original))
            load(config)
        except BaseException:
            for path, original in reversed(written):
                replace_bytes(path, original)
            raise
    print('Staff delivery configured. Run one navigation session to verify PWM readback.')


if __name__ == '__main__':
    main()
