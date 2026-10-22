"""Change only the US100 requirement in the active hardware profile; stop stack first."""
import argparse
from pathlib import Path

from configure_mapping import ROOT, load, stopped_configuration, write_configuration


def configure(config, mode):
    if mode not in ('required', 'optional'):
        raise ValueError('mode must be required or optional')
    config = Path(config).resolve(strict=True)
    with stopped_configuration(config):
        cfg, hardware = load(config)
        required = mode == 'required'
        if hardware['motion'].get('us100_required', True) == required:
            return cfg
        return write_configuration(config, cfg, hardware, motion_options={
            'max_speed_mps': hardware['max_speed_mps'],
            'motion': {'us100_required': required},
        })


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'pi/config/mqtt.worker.json')
    parser.add_argument('--mode', required=True, choices=('required', 'optional'))
    args = parser.parse_args()
    cfg = configure(args.config, args.mode)
    print('US100_POLICY:', args.mode.upper())
    print('Hardware profile:', cfg['hardware_profile'])
    if args.mode == 'optional':
        print('US100 readings are diagnostic only; side-obstacle stopping by US100 is disabled.')
        print('LiDAR, lease, command, commissioning and rear-reverse checks remain active.')


if __name__ == '__main__':
    main()
