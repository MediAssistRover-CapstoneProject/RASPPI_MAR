"""Recorded stop tests and backed-up mapping activation.

Never infers physical success from PREPARE_READY or a zero process exit code.
"""
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
TESTS = ('stale-command', 'stale-lease', 'watchdog')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.new')
    try:
        with temporary.open('x', encoding='utf-8') as handle:
            handle.write(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
            handle.flush()
            os.fsync(handle.fileno())
        temporary.chmod(0o600)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def create_bench(config, cfg, hardware, directory):
    from mediassist.configuration import validate_config
    candidate = copy.deepcopy(hardware)
    candidate['motion'].update(commissioned=True, cart_footprint_validated=True, watchdog_validated=True)
    candidate['max_speed_mps'] = min(.03, hardware['max_speed_mps'])
    candidate['allowed_workflows'] = []
    candidate['capabilities'] = ['sensors']
    candidate['navigation']['operator_test_notes'] = 'TEMPORARY WHEELS-OFF-GROUND TEST; NOT FLOOR COMMISSIONING'
    validate_config(candidate)
    path = directory / 'bench.hardware.json'
    save_json(path, candidate)
    context = {'run_id': str(uuid.uuid4()), 'ready': False, 'worker_config': str(config),
               'worker_sha256': digest(config), 'source_hardware': cfg['hardware_profile'],
               'source_sha256': digest(cfg['hardware_profile']), 'bench_hardware': str(path),
               'bench_sha256': digest(path), 'tests': {}, 'created_at': datetime.now(timezone.utc).isoformat()}
    save_json(directory / 'bench.json', context)
    return candidate, str(path), context


def review_mapping_changes(config, context):
    """Reuse stop-test evidence only across mapping/side-threshold configuration edits."""
    from mediassist.worker_config import load
    require_observations(context)
    backups = sorted(config.parent.glob(config.stem + '*.bak'))
    original = next((p for p in backups if digest(p) == context['worker_sha256']), None)
    if original is None:
        raise RuntimeError('BENCH_CONFIG_BACKUP_MISSING: cannot compare the tested worker configuration')
    before, source = load(original)
    current, hardware = load(config)
    if Path(before['hardware_profile']).resolve() != Path(context['source_hardware']).resolve():
        raise RuntimeError('BENCH_SOURCE_MISMATCH')
    # Compare normalized configurations so newly written default values do not
    # invalidate evidence; never print MQTT credentials or token contents.
    for cfg in (before, current):
        cfg.pop('mapping', None)
        cfg.pop('hardware_profile', None)
    if before != current:
        raise RuntimeError('BENCH_CRITICAL_CONFIG_CHANGED: worker settings outside mapping changed')
    old_distance = source['motion'].pop('stop_distance_m')
    new_distance = hardware['motion'].pop('stop_distance_m')
    if source != hardware:
        raise RuntimeError('BENCH_CRITICAL_CONFIG_CHANGED: ' + ', '.join(changed_paths(source, hardware)))
    return {'tested_worker_sha256': context['worker_sha256'], 'active_worker_sha256': digest(config),
            'tested_config_backup': str(original), 'tested_stop_distance_m': old_distance,
            'active_stop_distance_m': new_distance,
            'scope': 'mapping settings and US100 side threshold only; stop tests reused'}


def changed_paths(before, after, prefix=''):
    """Report changed field names only; never include credentials or values."""
    if isinstance(before, dict) and isinstance(after, dict):
        result = []
        for key in sorted(set(before) | set(after)):
            path = prefix + '.' + key if prefix else key
            if key not in before or key not in after:
                result.append(path)
            else:
                result.extend(changed_paths(before[key], after[key], path))
        return result
    return [] if before == after else [prefix]


def context_for(config, reuse_stop_tests=False):
    directory = config.parent.parent / 'state/stack'
    path = directory / 'bench.json'
    if not path.is_file():
        raise RuntimeError('NO_BENCH_RECORD: run bench --wheels-lifted first')
    context = json.loads(path.read_text())
    if (context.get('worker_config') != str(config)
            or context.get('source_sha256') != digest(context['source_hardware'])
            or context.get('bench_sha256') != digest(context['bench_hardware'])):
        raise RuntimeError('BENCH_CONFIG_CHANGED: inspect changes and perform bench checks for the new profile')
    if context.get('worker_sha256') != digest(config):
        if not reuse_stop_tests:
            raise RuntimeError('BENCH_CONFIG_CHANGED: use enable-mapping --reuse-stop-tests only for mapping/side-threshold edits')
        context['activation_settings_review'] = review_mapping_changes(config, context)
    return directory, context


def activation_snapshot(config):
    """Detect configuration/evidence edits between review and the locked write."""
    cfg = json.loads(config.read_text(encoding='utf-8-sig'))
    record = config.parent.parent / 'state/stack/bench.json'
    return digest(config), digest(cfg['hardware_profile']), digest(record)


def unchanged_activation(config, snapshot, reuse_stop_tests):
    if activation_snapshot(config) != snapshot:
        raise RuntimeError('ACTIVATION_CONFIG_CHANGED: configuration or observations changed before writing')
    context_for(config, reuse_stop_tests)


def bench_test(config, name, wheels_lifted):
    if name not in TESTS or not wheels_lifted:
        raise RuntimeError('Use bench-test --test stale-command|stale-lease|watchdog --wheels-lifted')
    if not sys.stdin.isatty():
        raise RuntimeError('Use interactive docker exec -it; operator observation is required')
    import fcntl
    from robot_stack import still_same
    directory, context = context_for(config)
    with (directory / 'bench-test.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('ANOTHER_BENCH_TEST_IS_RUNNING') from None
        owner = json.loads((directory / 'supervisor.json').read_text())
        gate = json.loads((directory / 'bench-gate.json').read_text())
        if not context.get('ready') or not still_same(owner) or not still_same(gate) or '--bench-pause' not in gate['argv']:
            raise RuntimeError('BENCH_NOT_RUNNING: keep the BENCH_READY terminal open')
        if input('Tat ca banh chu dong da ke chac chan, khong cham san? Go LIFTED: ').strip() != 'LIFTED':
            raise RuntimeError('No movement command sent')
        # Invalidate an older observation before attempting another test.
        context['tests'].pop(name, None)
        save_json(directory / 'bench.json', context)
        print('Ban sap gui lenh quay banh tuong duong 0.03 m/s, toi da 4 giay. Quan sat truc tiep.', flush=True)
        argv = [str(ROOT / 'pi/.venv/bin/python'), str(ROOT / 'pi/tools/bench_motion.py'),
                '--config', context['bench_hardware'], '--wheels-lifted', '--test', name]
        if name == 'watchdog':
            argv += ['--gate-record', str(directory / 'bench-gate.json')]
        child = subprocess.Popen(argv, cwd=ROOT / 'pi', start_new_session=True)
        try:
            rc = child.wait(timeout=35)
        except BaseException:
            # Let bench_motion finally publish zero and cancel its lease.
            import signal
            from robot_stack import proc, send_verified
            info = proc(child.pid)
            if info:
                send_verified(info, signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                if info: send_verified(info, signal.SIGTERM)
            raise
        if rc != 0:
            raise RuntimeError('BENCH_TEST_FAILED: observation not recorded; see error above')
        print('Chi PASS neu banh DA quay luc dau, DUNG som sau khi cat lenh/lease/gate, va khong quay lai.')
        print('Banh khong quay tu dau, chi dung khi het 4 giay, hoac khong dung: go FAIL.')
        answer = input('Ket qua quan sat [PASS/FAIL]: ').strip()
        context['tests'][name] = {'observed_pass': answer == 'PASS',
                                 'observed_at': datetime.now(timezone.utc).isoformat(), 'process_exit': rc}
        save_json(directory / 'bench.json', context)
        if answer != 'PASS':
            raise RuntimeError('OBSERVED_TEST_FAILED: keep production motion disabled')
        print('Da ghi nhan quan sat:', name, '; chua bat mapping/production motion.')


def require_observations(context):
    missing = [name for name in TESTS if context.get('tests', {}).get(name, {}).get('observed_pass') is not True]
    if missing:
        raise RuntimeError('BENCH_OBSERVATIONS_MISSING: ' + ', '.join(missing))


def activated_profiles(cfg, hardware, context):
    from mediassist.configuration import validate_config
    require_observations(context)
    hw = copy.deepcopy(hardware)
    hw['motion'].update(commissioned=True, cart_footprint_validated=True, watchdog_validated=True)
    hw['max_speed_mps'] = min(.05, hw['max_speed_mps'])
    hw['allowed_workflows'] = []
    hw['capabilities'] = ['sensors']
    hw['navigation']['operator_test_notes'] = 'MANUAL_MAPPING: enabled from recorded stop tests'
    hw['navigation']['mapping_commissioning'] = {'bench_run_id': context['run_id'], 'observations': context['tests'],
        'activation_mode': 'recorded_stop_tests', 'geometry_measured_by_activation': False,
        'activated_at': datetime.now(timezone.utc).isoformat()}
    if 'activation_settings_review' in context:
        hw['navigation']['mapping_commissioning']['settings_review'] = copy.deepcopy(context['activation_settings_review'])
    validate_config(hw)
    worker = copy.deepcopy(cfg)
    worker['mapping'].update(enabled=True, control_source='mqtt', launch_argv=[], max_preview_rate_hz=1.)
    worker['mapping'].setdefault('camera', {}).update(enabled=True, topic='/ascamera/camera_publisher/rgb0/image', rate_hz=.5)
    return worker, hw


def enable_mapping(config, reuse_stop_tests=False):
    from robot_stack import processes, conflict, still_same, locked
    from mediassist.worker_config import load
    cfg, hardware = load(config)
    flags = ('commissioned', 'cart_footprint_validated', 'watchdog_validated')
    if (all(hardware['motion'][key] for key in flags) and cfg['mapping']['enabled']
            and cfg['mapping'].get('control_source') == 'mqtt' and not cfg['mapping']['launch_argv']):
        print('MAPPING_ALREADY_ENABLED: motion flags are true; configuration unchanged.')
        print('Hardware profile:', cfg['hardware_profile'])
        return
    directory, context = context_for(config, reuse_stop_tests)
    snapshot = activation_snapshot(config)
    record = directory / 'supervisor.json'
    if record.exists() and still_same(json.loads(record.read_text())):
        raise RuntimeError('STOP_BENCH_FIRST: Ctrl+C and wait for the bench terminal to finish')
    if any(conflict(p) for p in processes().values()):
        raise RuntimeError('STOP_EXISTING_COMPONENTS_FIRST')
    require_observations(context)
    if 'activation_settings_review' in context:
        review = context['activation_settings_review']
        print('Da doi chieu backup: chi mapping va nguong US100 thay doi; giu 3 ket qua PASS.')
        print('Nguong US100 (m):', review['tested_stop_distance_m'], '->', review['active_stop_distance_m'])
    worker, hw = activated_profiles(cfg, hardware, context)
    # Lock prevents a simultaneous run/prepare/configuration activation.
    with locked(directory, config):
        unchanged_activation(config, snapshot, reuse_stop_tests)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
        target = config.parent / ('site.mapping.' + stamp + '.json')
        backup = config.with_name(config.name + '.before-mapping-' + stamp + '.bak')
        save_json(target, hw)
        worker['hardware_profile'] = str(target)
        staged = config.with_name(config.name + '.enable-' + stamp + '.new')
        try:
            save_json(staged, worker)
            load(staged)  # Validate before changing the active config.
            shutil.copy2(config, backup)
            os.replace(staged, config)
        finally:
            staged.unlink(missing_ok=True)
        print('Backup:', backup)
        print('Hardware profile:', target)
        print('MOTION_ENABLED: commissioned=true, cart_footprint_validated=true, watchdog_validated=true')
        print('Mapping enabled; MQTT credentials, state paths, GPIO and servo values preserved.')
        if not worker['mapping'].get('upload_base_url'):
            print('Upload HTTPS chua cau hinh: preview/control duoc, SAVE -> COMMIT_BE chua hoan tat.')
