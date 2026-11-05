"""Select rigid cargo bounds for a saved-map runtime; keep mapping geometry intact."""
import copy
import math
from .contracts import StepError
from .navigation_setup import footprint_rectangle


def configuration(cfg):
    value = cfg.get('navigation_geometry')
    if value is None:
        return {'default_profile': 'bare'}
    if not isinstance(value, dict) or set(value) - {'default_profile', 'cargo'}:
        raise StepError('INVALID_NAVIGATION_GEOMETRY')
    if value.get('default_profile') not in ('bare', 'cargo'):
        raise StepError('INVALID_NAVIGATION_GEOMETRY', 'default_profile')
    cargo = value.get('cargo')
    if cargo is not None:
        if (not isinstance(cargo, dict) or set(cargo) != {'rear_extension_m', 'width_m'}):
            raise StepError('INVALID_NAVIGATION_GEOMETRY', 'cargo')
        for key, low, high in (('rear_extension_m', 0., 1.), ('width_m', .05, 2.)):
            v = cargo[key]
            if type(v) not in (int, float) or not math.isfinite(v) or not low <= v <= high:
                raise StepError('INVALID_NAVIGATION_GEOMETRY', key)
    if value['default_profile'] == 'cargo' and cargo is None:
        raise StepError('CARGO_DIMENSIONS_REQUIRED')
    return value


def session_geometry(cfg, hardware, selected=None):
    """Return private copies for every session; extension is never cumulative.

    Cargo is fixed, centred laterally, and measured from the bare robot's rear
    edge. A bounding rectangle includes its narrower rear box conservatively.
    """
    if cfg.get('navigation_session') or cfg.get('navigation_runtime', {}).get('geometry'):
        raise StepError('USE_BASE_GEOMETRY_CONFIG', 'Do not extend an already generated navigation runtime')
    settings = configuration(cfg)
    selected = settings['default_profile'] if selected is None else selected
    if selected not in ('bare', 'cargo'):
        raise StepError('INVALID_NAVIGATION_GEOMETRY', 'profile')
    runtime, hw = copy.deepcopy(cfg), copy.deepcopy(hardware)
    auto = runtime['mapping']['autonomy']
    extent = copy.deepcopy(auto.get('footprint'))
    if not extent:
        raise StepError('CONFIGURE_MEASURED_FOOTPRINT_FIRST')
    footprint_rectangle(**extent)
    bare = copy.deepcopy(extent)
    if selected == 'cargo':
        cargo = settings.get('cargo')
        if cargo is None:
            raise StepError('CARGO_DIMENSIONS_REQUIRED')
        if (auto.get('vehicle') != 'rigid'
                or hw['motion'].get('chassis_model') != 'MENTORPI_M1_MECANUM'):
            raise StepError('RIGID_CARGO_PROFILE_REQUIRED')
        extent['rear'] += cargo['rear_extension_m']
        extent['left'] = max(extent['left'], cargo['width_m'] / 2)
        extent['right'] = max(extent['right'], cargo['width_m'] / 2)
        if any(v > 1.5 for v in extent.values()):
            raise StepError('INVALID_NAVIGATION_GEOMETRY', 'combined footprint')
    footprint_rectangle(**extent)
    auto['footprint'] = extent
    braking = hw['motion'].get('lidar_braking')
    if selected == 'cargo' and not isinstance(braking, dict):
        raise StepError('CARGO_LIDAR_BRAKING_REQUIRED')
    if isinstance(braking, dict):
        braking['footprint'] = copy.deepcopy(extent)
        # Legacy rear-distance gate measures from base, rather than body edge.
        hw['motion']['rear_stop_distance_m'] = max(
            hw['motion'].get('rear_stop_distance_m', 0.),
            extent['rear'] + braking['stop_margin_m'])
        if hw['motion']['rear_stop_distance_m'] > 1.:
            raise StepError('INVALID_NAVIGATION_GEOMETRY', 'rear stop distance exceeds hardware range')
    evidence = {'profile': selected, 'attachment': 'rigid',
                'bare_footprint_m': bare, 'effective_footprint_m': copy.deepcopy(extent)}
    if selected == 'cargo':
        evidence['cargo'] = copy.deepcopy(settings['cargo'])
    return runtime, hw, evidence
