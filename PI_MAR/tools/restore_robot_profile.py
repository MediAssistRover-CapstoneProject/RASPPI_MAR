"""Rebuild the measured M1 profile from a valid scaffold, without robot motion.

Credentials, device identity and journals stay in the scaffold. Historical
commissioning flags are restored only by an explicit option and never create
bench-test observations. Default invocation is a read-only configuration plan.
"""
import argparse
import copy
import json
import os
import shutil
from contextlib import nullcontext
from datetime import datetime, timezone
from pathlib import Path

from configure_mapping import ROOT, load, stopped_configuration
from mediassist.autonomous_mapping import configuration as autonomy_configuration
from mediassist.configuration import validate_config
from mediassist.gamepad import configuration as gamepad_configuration
from mediassist.localization import configuration as localization_configuration
from mediassist.navigation_geometry import session_geometry
from mediassist.worker_config import camera_config


BARE = dict(front=.109, rear=.104, left=.086, right=.086)


def construct(cfg, hardware, *, mapping_speed, camera_backend, camera_fps=15.,
              move_speed=.35, move_angular=1.2, mapping_angular=1.2,
              manual_speed=.25, manual_angular=2., reverse_speed=.25,
              restore_commissioning=False):
    if cfg.get('navigation_session') or cfg.get('navigation_runtime'):
        raise ValueError('BASE_WORKER_REQUIRED: do not reuse a generated navigation runtime')
    if not .02 <= mapping_speed <= .8 or not .05 <= move_speed <= .8:
        raise ValueError('Invalid mapping or MOVE speed')
    if not .02 <= reverse_speed <= .8 or not .05 <= manual_speed <= .30:
        raise ValueError('Invalid reverse or manual speed')
    if not .1 <= manual_angular <= 2. or not .1 <= move_angular <= manual_angular:
        raise ValueError('Invalid manual or MOVE angular speed')
    if not .1 <= mapping_angular <= min(1.2, manual_angular):
        raise ValueError('Invalid mapping angular speed')
    if camera_backend not in ('python', 'web_video_server'):
        raise ValueError('Invalid camera backend')
    result, hw = copy.deepcopy(cfg), copy.deepcopy(hardware)
    hw['robot_id'] = result['robot_id']
    motion = hw['motion']
    hw['max_speed_mps'] = max(hw['max_speed_mps'], move_speed, mapping_speed,
                              manual_speed, reverse_speed)
    motion.update(chassis_model='MENTORPI_M1_MECANUM', allow_reverse=True,
                  allow_strafe=True, max_reverse_speed_mps=reverse_speed,
                  max_angular_speed_rps=manual_angular, us100_required=False,
                  rear_stop_distance_m=BARE['rear'] + .02)
    for name in ('commissioned', 'cart_footprint_validated', 'watchdog_validated'):
        motion[name] = bool(restore_commissioning)
    hw['servo']['commissioned'] = bool(restore_commissioning)
    hw['navigation'].update(max_speed_mps=move_speed,
                            max_angular_speed_rps=move_angular)
    hw['navigation']['profile_restoration'] = {
        'commissioning_flags_restored': bool(restore_commissioning),
        'fresh_stop_test_evidence_created': False,
        'scope': 'configuration recovery; no motor, servo or ROS command sent'}
    mapping = result['mapping']
    mapping.update(enabled=True, control_source='auto', launch_argv=[], fresh_session=True)
    auto = autonomy_configuration(mapping)
    auto.update(enabled=True, vehicle='rigid', footprint=copy.deepcopy(BARE),
                exploration_mode='frontier', frontier_controller='nav2',
                manual_lidar_assist=False, speed_mps=mapping_speed,
                angular_rps=mapping_angular, reverse_enabled=True,
                max_reverse_mps=min(mapping_speed, reverse_speed),
                stop_margin_m=.02, turn_clearance_m=.20,
                slow_clearance_m=max(.65, .8 if mapping_speed > .25 else .65,
                                     1.9 * mapping_speed),
                scan_max_age_sec=.4, lidar_stop_distance_m=.05,
                visit_known_space=True)
    autonomy_configuration(mapping)
    motion['lidar_braking'] = {**copy.deepcopy(auto), 'include_points': True,
                               'angular_rps': manual_angular,
                               'speed_mps': hw['max_speed_mps'],
                               'max_reverse_mps': reverse_speed,
                               'near_slow_margin_m': .05}
    # The braking profile must satisfy the fastest configured translation cap.
    motion['lidar_braking']['slow_clearance_m'] = max(
        auto['slow_clearance_m'], .8 if hw['max_speed_mps'] > .25 else .65,
        1.9 * hw['max_speed_mps'])
    pad = gamepad_configuration(mapping)
    pad.update(layout='vendor_mecanum', linear_axis=1, lateral_axis=0,
               angular_axis=2, session_required=False, max_run_sec=5.,
               forward_mps=manual_speed, reverse_mps=min(manual_speed, reverse_speed),
               lateral_mps=manual_speed, angular_rps=manual_angular,
               linear_accel=.8, angular_accel=3.)
    gamepad_configuration(mapping)
    camera = mapping['camera']
    camera.update(enabled=True, topic='/ascamera/camera_publisher/rgb0/image',
                  stream_backend=camera_backend, stream_enabled=True,
                  stream_bind='0.0.0.0', stream_port=8080,
                  stream_rate_hz=camera_fps, stream_max_width=640,
                  stream_max_height=480, stream_jpeg_quality=75 if camera_backend == 'web_video_server' else 60,
                  vendor_stream_type='ros_compressed' if camera_backend == 'web_video_server' else 'mjpeg',
                  mqtt_preview_enabled=False)
    camera_config(mapping)
    result['navigation_geometry'] = {
        'default_profile': 'cargo',
        'cargo': {'rear_extension_m': .08, 'width_m': .15}}
    result['localization'] = localization_configuration({
        **result.get('localization', {}), 'enabled': False,
        'min_scan_match': .55, 'tracking_min_scan_match': .55})
    # A fresh base profile has no selected map. The navigation launcher adds
    # MOVE only after downloading and validating the BE session's real bundle.
    result['enabled_step_types'] = list(dict.fromkeys([
        *(kind for kind in result['enabled_step_types'] if kind != 'MOVE'),
        'CONTROL_HARDWARE', 'EXECUTE_SENSING', 'WAIT']))
    result['compartments'] = {**result['compartments'], 'CART_MAIN': 'cargo_lock'}
    result['unlock_policy'] = 'STAFF_VERIFIED'
    result['allow_commissioning_unlock'] = bool(restore_commissioning)
    validate_config(hw)
    _, _, geometry = session_geometry(result, hw, 'cargo')
    return result, hw, geometry


def publish(config, candidate, hardware):
    """Publish a complete immutable hardware version, then its validated worker."""
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
    profile = config.parent / ('site.restored.' + stamp + '.json')
    staged = config.with_name(config.name + '.' + stamp + '.new')
    result = copy.deepcopy(candidate)
    result['hardware_profile'] = str(profile.resolve())
    committed = False
    try:
        for path, document in ((profile, hardware), (staged, result)):
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(document, stream, ensure_ascii=False, indent=2)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
        load(staged)
        shutil.copy2(config, config.with_name(config.name + '.' + stamp + '.bak'))
        os.replace(staged, config)
        config.chmod(0o600)
        committed = True
        return result
    finally:
        staged.unlink(missing_ok=True)
        if not committed:
            profile.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'pi/config/mqtt.worker.json')
    parser.add_argument('--mapping-speed', type=float, required=True,
                        help='Choose explicitly; the last private mapping ceiling was lost')
    parser.add_argument('--camera-backend', choices=('python', 'web_video_server'), required=True)
    parser.add_argument('--camera-fps', type=float, default=15.)
    parser.add_argument('--move-speed', type=float, default=.35)
    parser.add_argument('--move-angular', type=float, default=1.2)
    parser.add_argument('--mapping-angular', type=float, default=1.2)
    parser.add_argument('--manual-speed', type=float, default=.25)
    parser.add_argument('--manual-angular', type=float, default=2.)
    parser.add_argument('--reverse-speed', type=float, default=.25)
    parser.add_argument('--restore-commissioning', action='store_true',
                        help='Explicitly restore prior approved motor/servo flags; does not certify fresh tests')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    path = args.config.resolve(strict=True)
    with stopped_configuration(path) if args.apply else nullcontext():
        cfg, hardware = load(path)
        candidate, restored, geometry = construct(
            cfg, hardware, **{name: getattr(args, name) for name in (
                'mapping_speed', 'camera_backend', 'camera_fps', 'move_speed',
                'move_angular', 'mapping_angular', 'manual_speed', 'manual_angular',
                'reverse_speed', 'restore_commissioning')})
        if args.apply:
            candidate = publish(path, candidate, restored)
    print('ROBOT_PROFILE_RESTORED' if args.apply else 'PLAN_ONLY')
    print(json.dumps({'geometry': geometry, 'move_mps': args.move_speed,
                      'mapping_mps': args.mapping_speed, 'camera_backend': args.camera_backend,
                      'camera_fps': args.camera_fps,
                      'commissioning_flags_restored': args.restore_commissioning,
                      'fresh_stop_test_evidence_created': False}, indent=2))


if __name__ == '__main__':
    main()
