"""Select exactly one mapping input; preserve credentials, map storage and calibration."""
import argparse
import copy
from pathlib import Path

from configure_mapping import ROOT, load, stopped_configuration, write_configuration
from mediassist.gamepad import configuration


def configure(config, source, device_id=None, linear_sign=None, angular_sign=None, us100=None):
    if source not in ('mqtt', 'gamepad'):
        raise ValueError('source must be mqtt or gamepad')
    config = Path(config).resolve(strict=True)
    with stopped_configuration(config):
        cfg, hardware = load(config)
        previous = copy.deepcopy(cfg)
        cfg['mapping']['control_source'] = source
        pad = configuration(cfg['mapping'])
        for key, value in (('device_id', device_id), ('linear_sign', linear_sign), ('angular_sign', angular_sign)):
            if value is not None:
                pad[key] = value
        configuration(cfg['mapping'])
        options = None
        if us100 is not None:
            if us100 not in ('required', 'optional'):
                raise ValueError('us100 must be required or optional')
            if hardware['motion'].get('us100_required', True) != (us100 == 'required'):
                options = {'max_speed_mps': hardware['max_speed_mps'],
                           'motion': {'us100_required': us100 == 'required'}}
        if cfg != previous or options:
            cfg = write_configuration(config, cfg, hardware, motion_options=options)
        print('MAPPING_CONTROL_SOURCE:', source.upper())
        print('GAMEPAD:', pad)
        print('US100:', us100 or ('required' if hardware['motion'].get('us100_required', True) else 'optional'))
        return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'pi/config/mqtt.worker.json')
    parser.add_argument('--source', required=True, choices=('gamepad', 'mqtt'))
    parser.add_argument('--device-id', type=int)
    parser.add_argument('--linear-sign', type=int, choices=(-1, 1))
    parser.add_argument('--angular-sign', type=int, choices=(-1, 1))
    parser.add_argument('--us100', choices=('required', 'optional'))
    args = parser.parse_args()
    configure(args.config, args.source, args.device_id, args.linear_sign, args.angular_sign, args.us100)


if __name__ == '__main__':
    main()
