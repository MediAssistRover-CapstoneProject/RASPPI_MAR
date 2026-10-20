"""Generate separate measured navigation files, never edit the vendor installation."""
import argparse
import copy
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import yaml
from mediassist.contracts import canonical, read_json
from mediassist.configuration import validate_config
from mediassist.navigation_setup import footprint_rectangle, nav2_params, add_group


def save(path, data, as_yaml=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as handle:
        handle.write(yaml.safe_dump(data, allow_unicode=True, sort_keys=False) if as_yaml else canonical(data) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    a = sub.add_parser('nav2-params')
    a.add_argument('--vendor', type=Path, default=Path('/home/ubuntu/ros2_ws/src/navigation/config'))
    for name in ('front', 'rear', 'left', 'right'):
        a.add_argument('--' + name, required=True, type=float, help='Measured extent from base_footprint, metres')
    a.add_argument('--output', type=Path, required=True)
    a = sub.add_parser('bench-profile')
    a.add_argument('--base', type=Path, required=True)
    a.add_argument('--output', type=Path, required=True)
    a.add_argument('--wheels-lifted', action='store_true', required=True)
    a.add_argument('--footprint-reviewed', action='store_true', required=True)
    a = sub.add_parser('group')
    a.add_argument('--base', type=Path, required=True)
    a.add_argument('--id', required=True)
    a.add_argument('--members', nargs='+', required=True)
    a.add_argument('--revision', type=int, default=1)
    a.add_argument('--output', type=Path, required=True)
    a = sub.add_parser('move-input')
    a.add_argument('--base', type=Path, required=True)
    target = a.add_mutually_exclusive_group(required=True)
    target.add_argument('--endpoint')
    target.add_argument('--group')
    a.add_argument('--output', type=Path, required=True)
    a = sub.add_parser('commission')
    a.add_argument('--base', type=Path, required=True)
    a.add_argument('--output', type=Path, required=True)
    a.add_argument('--footprint-reviewed', action='store_true', required=True)
    a.add_argument('--sensors-tested', action='store_true', required=True)
    a.add_argument('--watchdog-tested', action='store_true', required=True)
    a.add_argument('--evidence', required=True, help='Operator test notes; not a claim of automatic verification')
    a = sub.add_parser('manual-profile')
    a.add_argument('--base', type=Path, required=True)
    a.add_argument('--output', type=Path, required=True)
    a.add_argument('--footprint-reviewed', action='store_true', required=True)
    a.add_argument('--sensors-tested', action='store_true', required=True)
    a.add_argument('--watchdog-tested', action='store_true', required=True)
    a.add_argument('--evidence', required=True)
    args = p.parse_args()
    if args.command == 'nav2-params':
        base = yaml.safe_load((args.vendor / 'nav2_params.yaml').read_text())
        controller = yaml.safe_load((args.vendor / 'nav2_controller_dwb.yaml').read_text())
        rect = footprint_rectangle(args.front, args.rear, args.left, args.right)
        save(args.output, nav2_params(base, controller, rect), as_yaml=True)
    else:
        cfg = validate_config(read_json(args.base))
        if args.command == 'group':
            save(args.output, add_group(cfg, args.id, args.members, args.revision))
        elif args.command == 'bench-profile':
            result = copy.deepcopy(cfg)
            result['motion'].update(commissioned=True, cart_footprint_validated=True, watchdog_validated=True)
            result['max_speed_mps'] = 0.03
            result['allowed_workflows'] = []
            result['capabilities'] = ['sensors']
            # Keep the real journal path, including its single-process lock.
            result['state_path'] = cfg['state_path']
            result['navigation']['operator_test_notes'] = 'TEMPORARY WHEELS-OFF-GROUND TEST; NOT FLOOR COMMISSIONING'
            save(args.output, validate_config(result))
        elif args.command == 'move-input':
            result = {'map_id': cfg['map_id'], 'mode': 'NORMAL', 'max_speed_mps': min(0.05, cfg['max_speed_mps'])}
            if args.endpoint:
                if args.endpoint not in cfg['endpoints']: raise ValueError('Unknown endpoint')
                result['endpoint_id'] = args.endpoint
            else:
                group = cfg['endpoint_groups'][args.group]
                result.update(endpoint_group_id=args.group, group_revision=group['revision'],
                              candidate_endpoint_ids=group['endpoint_ids'])
            save(args.output, result)
        elif args.command == 'manual-profile':
            if len(args.evidence.strip()) < 12:
                raise ValueError('Provide concrete operator evidence')
            result = copy.deepcopy(cfg)
            result['motion'].update(commissioned=True, cart_footprint_validated=True, watchdog_validated=True)
            result['max_speed_mps'] = min(0.05, result['max_speed_mps'])
            result['allowed_workflows'] = []
            result['capabilities'] = ['sensors']
            result['navigation']['operator_test_notes'] = 'MANUAL_MAPPING: ' + args.evidence.strip()
            save(args.output, validate_config(result))
        else:
            if not cfg.get('map_asset_sha256') or not cfg['endpoints']:
                raise ValueError('Import the saved map and real endpoints first')
            if len(args.evidence.strip()) < 12:
                raise ValueError('Provide concrete operator evidence')
            result = copy.deepcopy(cfg)
            result['motion'].update(commissioned=True, cart_footprint_validated=True, watchdog_validated=True)
            result['max_speed_mps'] = min(0.05, result['max_speed_mps'])
            result['motion']['max_angular_speed_rps'] = min(0.3, result['motion']['max_angular_speed_rps'])
            result['capabilities'] = sorted(set(result['capabilities']) | {'nav2'})
            result['navigation']['operator_test_notes'] = args.evidence.strip()
            save(args.output, validate_config(result))
    print('Saved:', args.output, '; no ROS command sent; no existing file overwritten.')


if __name__ == '__main__': main()
