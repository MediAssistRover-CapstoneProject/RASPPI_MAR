"""Change autonomous mapping speed while stopped; preserve manual control, camera and network."""
import argparse
import copy
import json
import math
from contextlib import nullcontext
from pathlib import Path
from configure_mapping import ROOT, load, stopped_configuration, write_configuration
from mediassist.autonomous_mapping import configuration
from mediassist.configuration import validate_config


def tune(cfg, hardware, speed, angular=.6, mode='frontier'):
    if (type(speed) not in (int, float) or not math.isfinite(speed) or not .02 <= speed <= 1.
            or type(angular) not in (int, float) or not math.isfinite(angular) or not .1 <= angular <= 1.2):
        raise ValueError('Use speed .02..1.00 m/s and angular .10..1.20 rad/s')
    motion = hardware['motion']
    if mode not in ('frontier','reactive'):
        raise ValueError('Mode must be frontier or reactive')
    if motion.get('chassis_model') != 'MENTORPI_M1_MECANUM' or motion.get('allow_strafe') is not True:
        raise ValueError('BARE_MECANUM_REQUIRED')
    if not all(motion.get(k) is True for k in ('commissioned', 'cart_footprint_validated', 'watchdog_validated')):
        raise ValueError('EXISTING_COMMISSIONING_REQUIRED')
    if angular > motion['max_angular_speed_rps']:
        raise ValueError('ANGULAR_HARDWARE_LIMIT_EXCEEDED')
    result = copy.deepcopy(cfg)
    auto = configuration(result['mapping'])
    if auto['vehicle'] != 'rigid' or not auto['enabled']:
        raise ValueError('ENABLED_RIGID_MAPPING_REQUIRED')
    extent = auto.get('footprint', {})
    if any(extent.get(k, 10) > maximum for k, maximum in dict(front=.15, rear=.15, left=.13, right=.13).items()):
        raise ValueError('EXISTING_BARE_BODY_FOOTPRINT_REQUIRED')
    auto.update(speed_mps=speed, angular_rps=angular,exploration_mode=mode,
                frontier_controller='nav2' if mode=='frontier' else 'reactive',stop_margin_m=.05,lidar_stop_distance_m=.05)
    if speed > .25:
        auto['slow_clearance_m'] = max(.80 if speed <= .5 else 1.9*speed, auto['slow_clearance_m'])
        auto['scan_max_age_sec'] = .4
    configuration(result['mapping'])
    tuned_hardware = copy.deepcopy(hardware)
    tuned_hardware['max_speed_mps'] = max(speed, hardware['max_speed_mps'])
    validate_config(tuned_hardware)
    return result, tuned_hardware


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'pi/config/mqtt.worker.json')
    parser.add_argument('--trailer-removed', action='store_true', required=True)
    parser.add_argument('--speed', type=float, required=True)
    parser.add_argument('--angular', type=float, default=.6)
    parser.add_argument('--mode',choices=('frontier','reactive'),default='frontier')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    path = args.config.resolve(strict=True)
    with stopped_configuration(path) if args.apply else nullcontext():
        _, hardware = load(path)
        raw = json.loads(path.read_text(encoding='utf-8-sig'))
        candidate, tuned_hardware = tune(raw, hardware, args.speed, args.angular,args.mode)
        if args.apply:
            options = {'max_speed_mps': tuned_hardware['max_speed_mps'], 'motion': {}}
            candidate = write_configuration(path, candidate, hardware, motion_options=options)
    auto = candidate['mapping']['autonomy']
    print('AUTONOMY_SPEED_APPLIED' if args.apply else 'PLAN_ONLY; add --apply with the stack stopped')
    print('CONFIG:', path)
    print('HARDWARE_PROFILE:', candidate['hardware_profile'])
    print('AUTONOMY:', json.dumps({k: auto[k] for k in ('exploration_mode','speed_mps', 'angular_rps', 'slow_clearance_m', 'scan_max_age_sec', 'lidar_stop_distance_m')}))
    print('Manual speeds, reverse limit, footprint, commissioning flags, camera, TLS, maps and journals preserved.')
    print('Configured ceiling only; this command sends no motion and does not certify stopping distance at the new speed.')


if __name__ == '__main__':
    main()
