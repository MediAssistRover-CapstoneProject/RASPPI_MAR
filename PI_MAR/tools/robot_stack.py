"""One foreground supervisor for MentorPi mapping; never sends a movement command.

prepare: hardware/SLAM only, commissioning flags are preserved.
run: stack plus worker; stationary mapping is allowed while the motion gate remains inhibited.
inspect: read-only configuration/process report. stop: verified supervisor SIGTERM.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time
from contextlib import contextmanager

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))


def say(message):
    print(time.strftime('%H:%M:%S'), message, flush=True)


def proc(pid, base=Path('/proc')):
    try:
        path = base / str(pid)
        fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        argv = [os.fsdecode(a) for a in (path / 'cmdline').read_bytes().split(b'\0') if a]
        return {'pid': int(pid), 'ppid': int(fields[1]), 'ticks': fields[19], 'argv': argv}
    except (OSError, ValueError, IndexError):
        return None


def processes():
    return {int(p.name): info for p in Path('/proc').iterdir()
            if p.name.isdigit() and (info := proc(int(p.name))) is not None}


def vendor_bringup(info):
    a = info['argv']
    return any(Path(a[i]).name == 'ros2' and a[i+1:i+4] == ['launch', 'bringup', 'bringup.launch.py']
               for i in range(max(0, len(a) - 3)))


def conflict(info):
    a = info['argv']
    if any(x in a for x in ('mediassist.worker_cli', 'mediassist.safety_node', 'mediassist.cli')):
        return True
    if any(Path(x).name in ('us100_dual_ros2.py', 'manual_mapping.py', 'mediassist_demo_slam.launch.py',
                            'mediassist_demo_navigation.launch.py', 'slam.launch.py', 'navigation.launch.py') for x in a):
        return True
    return False


def descendants(root, table):
    selected = {root['pid']: root}
    changed = True
    while changed:
        changed = False
        for pid, info in table.items():
            if pid not in selected and info['ppid'] in selected:
                selected[pid] = info
                changed = True
    return list(selected.values())


def still_same(info, allow_exec=False):
    current = proc(info['pid'])
    return (current is not None and current['ticks'] == info['ticks']
            and (allow_exec or current['argv'] == info['argv']))


def send_verified(info, sig, allow_exec=False):
    if info['pid'] <= 1 or info['pid'] == os.getpid():
        raise RuntimeError('Refusing to signal PID 1/self')
    if still_same(info, allow_exec):
        try:
            os.kill(info['pid'], sig)
        except ProcessLookupError:
            pass


def terminate_tree(root, grace=8., remembered=()):
    # Snapshot descendants BEFORE signaling parent, and verify start ticks again
    # for every signal. Never run the vendor kill-all shell script.
    table = processes()
    current = table.get(root['pid'])
    tree = descendants(current, table) if current and current['ticks'] == root['ticks'] else []
    owned = {(p['pid'], p['ticks']): p for p in [*remembered, *tree]}

    def survivors():
        # A verified child may exec or fork during shutdown. Track descendants
        # while lineage is still visible; a reused PID never qualifies.
        table = processes()
        for p in list(owned.values()):
            current = table.get(p['pid'])
            if current and current['ticks'] == p['ticks']:
                for child in descendants(current, table):
                    owned[(child['pid'], child['ticks'])] = child
        return [p for p in owned.values() if still_same(p, allow_exec=True)]

    send_verified(root, signal.SIGINT, allow_exec=True)
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline and survivors():
        time.sleep(.1)
    for p in reversed(survivors()):
        send_verified(p, signal.SIGTERM, allow_exec=True)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and survivors():
        time.sleep(.1)
    for p in reversed(survivors()):
        send_verified(p, signal.SIGKILL, allow_exec=True)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and survivors():
        time.sleep(.1)
    remaining = survivors()
    if remaining:
        raise RuntimeError('Processes did not stop: ' + ','.join(str(p['pid']) for p in remaining))


def stop_leftover(pid):
    """Stop one explicitly selected leftover component, never a global ROS kill."""
    if pid <= 1 or pid == os.getpid():
        raise RuntimeError('Refusing to stop PID 1/self')
    info = proc(pid)
    if info is None:
        say('PID_ALREADY_STOPPED: ' + str(pid)); return
    if not conflict(info):
        raise RuntimeError('PID_NOT_ROBOT_COMPONENT: ' + str(pid))
    # Only exact project scripts or known Hiwonder launch commands qualify.
    project = str(ROOT)
    if not any(project in arg for arg in info['argv']):
        vendor = Path('/home/ubuntu/ros2_ws/src')
        absolute_launch = any(str(vendor / package / 'launch' / (package + '.launch.py')) in info['argv']
                              for package in ('slam', 'navigation'))
        argv = info['argv']
        package_launch = any(Path(argv[i]).name == 'ros2' and argv[i+1:i+4] in
                             (['launch', 'slam', 'slam.launch.py'], ['launch', 'navigation', 'navigation.launch.py'])
                             for i in range(max(0, len(argv)-3)))
        if not absolute_launch and not package_launch:
            raise RuntimeError('PID_OUTSIDE_PROJECT: inspect its command before stopping it')
    say('STOP_LEFTOVER PID=' + str(pid))
    terminate_tree(info, grace=5.)
    say('LEFTOVER_STOPPED: ' + str(pid))


def validate_mode(mode, cfg, hardware, runtime_mode='mapping'):
    if mode == 'run':
        if runtime_mode == 'navigation':
            if cfg['mapping']['enabled']:raise RuntimeError('MAPPING_MUST_BE_DISABLED_FOR_NAVIGATION')
            if not hardware.get('map_asset_sha256') or not hardware.get('endpoints'):raise RuntimeError('INSTALL_BE_MAP_AND_ENDPOINTS_FIRST')
            if not all(hardware['motion'][k] for k in ('commissioned','cart_footprint_validated','watchdog_validated')):
                raise RuntimeError('MOTION_NOT_COMMISSIONED')
            return
        # MQTT, speech and stationary map preview can run with motion inhibited.
        # Mapping control and the local gate still require all commissioning flags.
        if not cfg['mapping']['enabled']:
            raise RuntimeError('MAPPING_DISABLED: configure_mapping.py --enable-mapping first')
        if cfg['mapping']['launch_argv']:
            raise RuntimeError('SLAM_DOUBLE_LAUNCH: supervisor owns SLAM; set mapping.launch_argv=[]')
        if cfg['mapping']['control_source'] != 'mqtt':
            raise RuntimeError('MAPPING_CONTROL_MUST_BE_MQTT')


def check_journals(cfg):
    for key, table, states in (
        ('legacy_state_path', 'jobs', ('ACCEPTED', 'RUNNING', 'RECOVERY_REQUIRED')),
    ):
        path = Path(cfg[key])
        if path.exists():
            db = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
            try:
                if db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (table,)).fetchone():
                    if db.execute('SELECT 1 FROM jobs WHERE status IN (?,?,?) LIMIT 1', states).fetchone():
                        raise RuntimeError('LEGACY_JOB_REQUIRES_RECONCILIATION')
            finally:
                db.close()
    session = Path(cfg['mapping']['storage_dir']) / 'session.json'
    if session.exists() and json.loads(session.read_text()).get('session_open'):
        raise RuntimeError('MAPPING_RECOVERY_REQUIRED: reconcile interrupted session; do not delete journal')


@contextmanager
def locked(state_dir, config):
    import fcntl
    state_dir.mkdir(parents=True, exist_ok=True)
    record = state_dir / 'supervisor.json'
    with (state_dir / 'supervisor.lock').open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('STACK_ALREADY_RUNNING') from None
        identity = proc(os.getpid())
        record.write_text(json.dumps({**identity, 'config': str(config)}))
        record.chmod(0o600)
        try:
            yield
        finally:
            record.unlink(missing_ok=True)


class Supervisor:
    def __init__(self, directory):
        self.children = []
        self.directory = directory
        self.stopping = False
        self.owned = {}

    def start(self, name, argv):
        log = (self.directory / (name + '.log')).open('ab', buffering=0)
        log.write(('\n=== ' + time.strftime('%Y-%m-%d %H:%M:%S') + ' ===\n').encode())
        env = dict(os.environ)
        env['PYTHONUNBUFFERED'] = '1'
        env['PYTHONPATH'] = str(ROOT / 'pi') + (os.pathsep + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
        try:
            child = subprocess.Popen(argv, cwd=ROOT / 'pi', env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        finally:
            log.close()
        self.children.append((name, child, proc(child.pid)))
        say('START ' + name + ': ' + str(self.directory / (name + '.log')))

    def alive(self):
        table = processes()
        for name, child, identity in self.children:
            if identity:
                saved = self.owned.setdefault(name, {})
                for entry in descendants(identity, table):
                    saved[(entry['pid'], entry['ticks'])] = entry
            if child.poll() is not None:
                raise RuntimeError(name + ' exited code=' + str(child.returncode) + '; see ' + str(self.directory / (name + '.log')))

    def close(self):
        # Worker first, hardware while gate/watchdog remain available, then sensors
        # and finally gate/watchdog. No automatic restart/replay of a failed job.
        order = {'worker': 0, 'slam': 1, 'us100': 2, 'gate': 3, 'watchdog': 4}
        for name, child, identity in sorted(self.children, key=lambda item: order[item[0]]):
            say('STOP ' + name)
            try:
                if identity:
                    terminate_tree(identity, grace=10. if name == 'worker' else 5.,
                                   remembered=self.owned.get(name, {}).values())
                child.wait(timeout=1)
            except Exception as exc:
                say('STOP_ERROR ' + name + ': ' + str(exc))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('inspect', 'prepare', 'bench', 'bench-test', 'enable-mapping', 'run', 'stop'))
    parser.add_argument('--config', type=Path, default=ROOT / 'pi/config/mqtt.worker.json')
    parser.add_argument('--ready-timeout', type=int, default=90)
    parser.add_argument('--mode', choices=('mapping','navigation'), default='mapping')
    parser.add_argument('--nav2-params', type=Path)
    parser.add_argument('--wheels-lifted', action='store_true')
    parser.add_argument('--pid', type=int, help='stop only: explicitly stop one recognized leftover component tree')
    parser.add_argument('--reuse-stop-tests', action='store_true',
                        help='enable-mapping only: compare backup before reusing PASS after mapping/US100 threshold edits')
    parser.add_argument('--test', choices=('stale-command', 'stale-lease', 'watchdog'))
    args = parser.parse_args()
    if args.reuse_stop_tests and args.action != 'enable-mapping':
        parser.error('--reuse-stop-tests is only valid with enable-mapping')
    if args.pid is not None and args.action != 'stop':
        parser.error('--pid is only valid with stop')
    if sys.platform != 'linux' or not Path('/opt/ros/humble').is_dir():
        raise RuntimeError('Run inside MentorPi Ubuntu/ROS Humble container')
    if not 15 <= args.ready_timeout <= 180:
        raise ValueError('ready-timeout must be 15..180 seconds')
    os.umask(0o077)
    config = args.config.resolve(strict=True)
    state_dir = config.parent.parent / 'state/stack'
    if args.action == 'stop':
        if args.pid is not None:
            record = state_dir / 'supervisor.json'
            if record.exists() and still_same(json.loads(record.read_text())):
                raise RuntimeError('SUPERVISOR_RUNNING: use stop without --pid and wait for it to finish first')
            with locked(state_dir, config):
                stop_leftover(args.pid)
            return
        record = state_dir / 'supervisor.json'
        if not record.exists():
            say('STACK_NOT_RUNNING'); return
        identity = json.loads(record.read_text())
        if identity.get('config') != str(config) or str(Path(__file__).resolve()) not in identity.get('argv', []):
            raise RuntimeError('SUPERVISOR_IDENTITY_MISMATCH')
        if not still_same(identity):
            say('STALE_RECORD: no process signaled'); return
        send_verified(identity, signal.SIGTERM)
        say('Stop requested; wait for STOP messages in the run terminal'); return
    from mediassist.worker_config import load
    cfg, hardware = load(config)
    if args.action == 'bench-test':
        from commission_mapping import bench_test
        bench_test(config, args.test, args.wheels_lifted)
        return
    if args.action == 'enable-mapping':
        from commission_mapping import enable_mapping
        enable_mapping(config, args.reuse_stop_tests)
        return
    if args.action == 'bench' and not args.wheels_lifted:
        raise RuntimeError('BENCH_REQUIRES_WHEELS_LIFTED: securely lift ALL driven wheels first')
    table = processes()
    existing = [p for p in table.values() if conflict(p)]
    vendor = [p for p in table.values() if vendor_bringup(p)]
    if args.action == 'inspect':
        print(json.dumps({'hardware_profile': cfg['hardware_profile'], 'motion': hardware['motion'],
            'mapping_enabled': cfg['mapping']['enabled'], 'speech': cfg.get('speech', {}),
            'map_topic': cfg['mapping']['map_topic'],
            'slam_launch_argv': cfg['mapping']['launch_argv'], 'vendor_bringup_pids': [p['pid'] for p in vendor],
            'existing_components': existing}, indent=2))
        return
    if args.mode=='navigation' and args.action!='run':raise RuntimeError('NAVIGATION_MODE_REQUIRES_RUN')
    validate_mode(args.action, cfg, hardware, args.mode)
    if args.mode=='navigation':
        from mediassist.map_tools import map_fingerprint
        map_yaml=Path(cfg.get('navigation_runtime',{}).get('map_yaml','')).resolve(strict=True)
        if map_fingerprint(map_yaml)!=hardware['map_asset_sha256']:raise RuntimeError('INSTALLED_MAP_HASH_MISMATCH')
        if not args.nav2_params or not args.nav2_params.is_file():raise RuntimeError('NAV2_PARAMS_REQUIRED')
    check_journals(cfg)
    if existing:
        raise RuntimeError('EXISTING_COMPONENTS: stop their own terminals first; PIDs=' + ','.join(str(p['pid']) for p in existing))
    vendor_file = Path('/home/ubuntu/ros2_ws/src/slam/launch/slam.launch.py')
    if not vendor_file.is_file():
        raise RuntimeError('VENDOR_SLAM_MISSING: ' + str(vendor_file))
    with locked(state_dir, config):
        supervisor = Supervisor(state_dir)
        bench = None
        hw = cfg['hardware_profile']
        if args.action == 'bench':
            from commission_mapping import create_bench
            hardware, hw, bench = create_bench(config, cfg, hardware, state_dir)
        def stop(signum, frame):
            supervisor.stopping = True
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, stop)
        probe = None
        try:
            for p in vendor:
                say('Stopping verified vendor bringup tree PID=' + str(p['pid']))
                terminate_tree(p)
            if supervisor.stopping:
                return
            # A supervising vendor service may respawn bringup. Do not fight it.
            time.sleep(1)
            if any(vendor_bringup(p) for p in processes().values()):
                raise RuntimeError('VENDOR_BRINGUP_RESPAWNED: inspect service owner; no second driver started')
            python = str(ROOT / 'pi/.venv/bin/python')
            supervisor.start('watchdog', [python, '-m', 'mediassist.safety_node', '--config', hw, '--motor-watchdog'])
            supervisor.start('gate', [python, '-m', 'mediassist.safety_node', '--config', hw] + (['--bench-pause'] if bench else []))
            supervisor.start('us100', [python, str(ROOT / 'pi/scripts/us100_dual_ros2.py'), '--config', hw])
            launch = ['ros2','launch',str(ROOT/'pi/launch/mediassist_demo_slam.launch.py')]
            if args.mode=='navigation':
                launch=['ros2','launch',str(ROOT/'pi/launch/mediassist_demo_navigation.launch.py'),
                        'map:='+str(map_yaml),'params_file:='+str(args.nav2_params.resolve())]
            supervisor.start('slam',launch)
            from stack_probe import Probe
            probe = Probe(hardware, {**cfg['mapping'],'static_map':args.mode=='navigation'})
            deadline, report, stable = time.monotonic() + args.ready_timeout, 0., 0
            while not supervisor.stopping:
                supervisor.alive()
                probe.spin()
                missing = probe.missing()
                if args.action == 'run':
                    missing = [x for x in missing if x not in ('left', 'right')]

                stable = 0 if missing else stable + 1
                if stable >= 5:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('STARTUP_NOT_READY: ' + ', '.join(missing))
                if time.monotonic() >= report:
                    say('WAIT ' + ', '.join(missing)); report = time.monotonic() + 5
            if supervisor.stopping:
                return
            if args.action == 'run':
                supervisor.start('worker', [python, '-m', 'mediassist.worker_cli', 'run', '--config', str(config)])
                say('STACK_READY: map/odom/TF/topology checked; check BE MQTT handshake separately')
                flags = ('commissioned', 'cart_footprint_validated', 'watchdog_validated')
                if not all(hardware['motion'][k] for k in flags) or any(x in probe.missing() for x in ('left', 'right')):
                    say('MOTION_INHIBITED: stationary mapping and speech available; motion still requires ready sensors and commissioning')
            elif bench:
                from commission_mapping import save_json
                gate = next(identity for name, _, identity in supervisor.children if name == 'gate')
                save_json(state_dir / 'bench-gate.json', gate)
                bench['ready'] = True
                save_json(state_dir / 'bench.json', bench)
                say('BENCH_READY: wheels must stay lifted; no worker; run bench-test in another terminal')
            else:
                say('PREPARE_READY: map/sensors/TF/topology checked; NO MQTT worker, NO movement command sent')
            report, graph_bad = 0., 0
            while not supervisor.stopping:
                supervisor.alive()
                probe.spin()
                if time.monotonic() < report:
                    continue
                report = time.monotonic() + 1
                if any(vendor_bringup(p) for p in processes().values()):
                    raise RuntimeError('VENDOR_BRINGUP_RESPAWNED')
                try:
                    probe.graph(); graph_bad = 0
                except Exception as exc:
                    graph_bad += 1
                    say('GRAPH_ERROR ' + str(exc))
                    if graph_bad >= 2:
                        raise RuntimeError('Velocity topology changed; stopping stack')
        finally:
            supervisor.close()
            if probe:
                probe.close()


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        say('ERROR ' + getattr(exc, 'code', str(exc)))
        sys.exit(1)
