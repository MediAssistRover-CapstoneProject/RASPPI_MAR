"""Bounded 0.03 m/s gate tests with every wheel lifted; never uses direct motor commands."""
import argparse
import asyncio
import time
import json
import signal
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from mediassist.configuration import load_config
from mediassist.adapters import RemoteServices
from mediassist.contracts import StepError


async def wait_for_graph(check, timeout=12.0, clock=time.monotonic, sleep=asyncio.sleep,
                         required_successes=3, timeout_label='BENCH_GRAPH_TIMEOUT'):
    """Read-only wait; every prerequisite must pass the requested consecutive checks."""
    deadline, stable, last, previous = clock() + timeout, 0, 'not checked', None
    while clock() < deadline:
        try:
            check()
        except StepError as exc:
            stable, last = 0, f'{exc.code}: {exc}'
            if last != previous:
                print('BENCH_GRAPH_WAIT:', last, flush=True)
                previous = last
        else:
            stable += 1
            if stable >= required_successes:
                print('BENCH_GRAPH_READY', flush=True)
                return
        await sleep(0.25)
    raise RuntimeError(timeout_label + ': no nonzero command sent; last failure: ' + last)


def check_bench_inputs(adapter, safe_sample, clock=time.monotonic):
    """Require current measurements and a stationary gate, never substitute missing echoes."""
    adapter._check_graph()
    health, snapshot = adapter.health(), adapter.telemetry()
    age = adapter.cfg['motion']['sensor_max_age_sec']
    missing = []
    for side in ('left', 'right'):
        if not health.get(side + '_fresh'):
            missing.append(side + '_range')
        elif health[side + '_m'] < adapter.cfg['motion']['stop_distance_m']:
            missing.append(side + '_clearance')
    if not health.get('sensors_ready'):
        scan = snapshot.get('scan')
        if not scan or scan.get('value') is not True or scan['age_sec'] > age:
            missing.append('scan')
        if not missing:
            missing.append('sensors_ready')
    odom, gate = snapshot.get('odom_speed'), snapshot.get('gate')
    if not odom or odom['age_sec'] > age or odom.get('value') is None or odom['value'] > .01:
        missing.append('stationary_odom')
    if not gate or gate['age_sec'] > age:
        missing.append('gate_sample')
    if clock() - safe_sample.get('at', float('-inf')) > age or safe_sample.get('value') != [0., 0., 0.]:
        missing.append('zero_safe_velocity')
    if missing:
        raise StepError('BENCH_INPUTS_NOT_READY', 'missing/stale/blocked: ' + ', '.join(missing))


def check_test_sensors(health, stop_distance):
    if not health.get('sensors_ready'):
        raise StepError('BENCH_SENSORS_LOST', 'Sensor data became unavailable during the stop test; no PASS can be recorded')
    if any(health[side + '_m'] < stop_distance for side in ('left','right')):
        raise StepError('BENCH_SIDE_OBSTACLE', 'Side clearance changed during the stop test; repeat after fixing the setup')


def show_diagnostics(adapter, elapsed, safe_sample):
    snapshot = adapter.telemetry()
    wanted = {key: snapshot.get(key) for key in ('gate', 'left', 'right', 'scan', 'odom_speed')}
    wanted['safe_velocity'] = safe_sample.get('value')
    at = safe_sample.get('at')
    wanted['safe_age_sec'] = None if at is None else round(time.monotonic() - at, 3)
    wanted['us100_status'] = {side: {**item['value'], 'age_sec': round(time.monotonic()-item['at'],3)}
                              for side,item in list(safe_sample.get('us100_status', {}).items())}
    print('BENCH_DIAG t=' + str(round(elapsed, 2)) + ' ' + json.dumps(wanted), flush=True)


async def run(args):
    from mediassist.ros_adapter import RosAdapter
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    cfg = load_config(args.config)
    if cfg['allowed_workflows'] or cfg['max_speed_mps'] > 0.03 or 'TEMPORARY WHEELS-OFF-GROUND' not in cfg['navigation'].get('operator_test_notes', ''):
        raise ValueError('Use a dedicated bench-profile; do not use a floor navigation profile')
    adapter = RosAdapter(cfg, RemoteServices())
    raw = adapter.node.create_publisher(Twist, cfg['ros']['raw_cmd_vel'], 10)
    safe_sample = {}
    def safe_seen(msg):
        safe_sample.update(value=[msg.linear.x, msg.linear.y, msg.angular.z], at=time.monotonic())
    adapter.node.create_subscription(Twist, cfg['ros']['safe_cmd_vel'], safe_seen, 10)
    def status_seen(msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict) and data.get('side') in ('left','right'):
                safe_sample.setdefault('us100_status', {})[data['side']] = {
                    'value': {'status': data.get('status'), 'distance_m': data.get('distance_m')},
                    'at': time.monotonic()}
        except (ValueError,TypeError):
            pass
    adapter.node.create_subscription(String, '/ultrasonic/status', status_seen, 10)
    start = None
    try:
        print('BENCH_INPUTS_WAIT: up to 15 seconds; graph and fresh inputs must remain ready for 1 second; no motion yet',flush=True)
        await wait_for_graph(lambda: check_bench_inputs(adapter,safe_sample),
                             timeout=15., required_successes=5, timeout_label='BENCH_INPUTS_TIMEOUT')
        print('BENCH_INPUTS_READY',flush=True)
        show_diagnostics(adapter, -1, safe_sample)
        print('BENCH ONLY: first second=0.03 m/s forward request. Total <=4 seconds.')
        gate = None
        if args.test == 'watchdog':
            if not args.gate_record:
                raise ValueError('watchdog test requires the verified bench gate record')
            from robot_stack import still_same
            gate = json.loads(args.gate_record.read_text())
            if not still_same(gate) or '--bench-pause' not in gate['argv'] or str(args.config.resolve()) not in gate['argv']:
                raise ValueError('Bench gate identity/config does not match')
            print('After 1 second, pause gate output for 5 seconds. Observe watchdog stop the wheels.')
        start = time.monotonic()
        paused = False
        next_diagnostic = 0.0
        while time.monotonic() - start < 4:
            elapsed = time.monotonic() - start
            # A stop caused by sensor loss is not evidence that the injected fault worked.
            check_test_sensors(adapter.health(), cfg['motion']['stop_distance_m'])
            if elapsed >= next_diagnostic:
                show_diagnostics(adapter, elapsed, safe_sample)
                next_diagnostic = elapsed + 0.25
            if gate and elapsed >= 1 and not paused:
                from robot_stack import send_verified, still_same
                if not still_same(gate):
                    raise ValueError('Bench gate exited or PID changed')
                send_verified(gate, signal.SIGUSR1)
                paused = True
                print('GATE_OUTPUT_PAUSED; watch the wheels stop now', flush=True)
            if elapsed < 1 or args.test != 'stale-command':
                command = Twist()
                command.linear.x = 0.03
                raw.publish(command)
            if elapsed < 1 or args.test != 'stale-lease':
                adapter.lease_message(True, 0.03)
            await asyncio.sleep(0.05)
        # Gate resumes only AFTER the test has stopped publishing motion.
        # Final zero/lease cancellation below precedes any resumption.
        print('Request ended. Observe the physical stop; this script does not certify that wheels stopped.')
    finally:
        for _ in range(5):
            raw.publish(Twist())
            adapter.lease_message(False)
            await asyncio.sleep(0.05)
        stopped = await adapter.stop()
        show_diagnostics(adapter, -1 if start is None else time.monotonic()-start, safe_sample)
        adapter.close()
        if not stopped:
            raise RuntimeError('STOP_REQUEST_FAILED: do not mark this test passed')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--wheels-lifted', required=True, action='store_true')
    p.add_argument('--test', required=True, choices=('stale-command', 'stale-lease', 'watchdog'))
    p.add_argument('--gate-record', type=Path)
    args = p.parse_args()
    from mediassist.cli import instance_lock
    from mediassist.store import Store
    cfg = load_config(args.config)
    state = Path(cfg['state_path'])
    state.parent.mkdir(parents=True, exist_ok=True)
    with instance_lock(state):
        store = Store(state, recover=True)
        try:
            if any(s == 'RECOVERY_REQUIRED' for s in store.states().values()):
                raise ValueError('Reconcile the interrupted job before this bench test')
            asyncio.run(run(args))
        finally:
            store.close()


if __name__ == '__main__': main()
