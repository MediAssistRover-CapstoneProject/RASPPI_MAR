"""Configure a fixed rear box for saved-map MOVE; mapping stays bare."""
import argparse
import copy
import json
from contextlib import nullcontext
from pathlib import Path
from configure_mapping import ROOT, load, stopped_configuration, write_configuration
from mediassist.navigation_geometry import session_geometry


def tune(cfg, hardware, rear_extension, cargo_width, default_profile='cargo'):
    candidate = copy.deepcopy(cfg)
    if candidate.get('navigation_session') or candidate.get('navigation_runtime', {}).get('geometry'):
        raise ValueError('USE_BASE_MAPPING_CONFIG: configure the base worker, not a generated navigation session')
    candidate['navigation_geometry'] = {
        'default_profile': default_profile,
        'cargo': {'rear_extension_m': rear_extension, 'width_m': cargo_width}}
    _, _, evidence = session_geometry(candidate, hardware, 'cargo')
    return candidate, evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'pi/config/mqtt.worker.json')
    parser.add_argument('--rear-extension', type=float, required=True,
                        help='Extra rear length beyond the bare robot edge, in metres')
    parser.add_argument('--cargo-width', type=float, required=True,
                        help='Width of a rigid box centred behind the chassis, in metres')
    parser.add_argument('--default-profile', choices=['bare', 'cargo'], default='cargo')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    config = args.config.resolve(strict=True)
    with stopped_configuration(config) if args.apply else nullcontext():
        cfg, hw = load(config)
        candidate, evidence = tune(cfg, hw, args.rear_extension, args.cargo_width, args.default_profile)
        if args.apply:
            write_configuration(config, candidate, hw)
    print('NAVIGATION_GEOMETRY_APPLIED' if args.apply else 'PLAN_ONLY')
    print(json.dumps(evidence, indent=2))
    print('Mapping keeps its bare footprint. New saved-map sessions default to:', args.default_profile)
    print('Use --load-profile bare or --load-profile cargo when starting navigation to override.')


if __name__ == '__main__':
    main()
