"""Voltage-only battery estimates; no motor control, files or network access."""
from collections import deque
import math
import re
import statistics
import threading
import time

from .contracts import StepError


def configuration(value):
    if value is None:
        return None
    if not isinstance(value, dict) or type(value.get('enabled')) is not bool:
        raise StepError('INVALID_CONFIG', 'battery.enabled')
    allowed = {'enabled', 'profile_id', 'series_cells', 'parallel_cells',
               'nominal_cell_voltage_v', 'nominal_cell_capacity_mah', 'voltage_gain',
               'voltage_offset_v', 'valid_min_cell_v', 'valid_max_cell_v',
               'stale_after_sec', 'median_samples', 'minimum_samples',
               'filter_time_constant_sec', 'curve_calibrated', 'cell_voltage_curve'}
    if set(value) - allowed:
        raise StepError('INVALID_CONFIG', 'battery unknown field')
    if not value['enabled']:
        return None
    result = dict(value)
    defaults = dict(voltage_gain=1., voltage_offset_v=0., valid_min_cell_v=2.5,
                    valid_max_cell_v=4.3, stale_after_sec=60., median_samples=5,
                    minimum_samples=3, filter_time_constant_sec=15., curve_calibrated=False)
    for key, default in defaults.items():
        result.setdefault(key, default)

    def number(key, low, high, integer=False):
        item = result.get(key)
        if (type(item) not in ((int,) if integer else (int, float))
                or not math.isfinite(item) or not low <= item <= high):
            raise StepError('INVALID_CONFIG', 'battery.' + key)

    if not isinstance(result.get('profile_id'), str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,80}', result['profile_id']):
        raise StepError('INVALID_CONFIG', 'battery.profile_id')
    for key, low, high in (('series_cells', 1, 12), ('parallel_cells', 1, 16),
                           ('nominal_cell_capacity_mah', 100, 30000), ('median_samples', 3, 31),
                           ('minimum_samples', 3, result['median_samples'])):
        number(key, low, high, integer=True)
    for key, low, high in (('nominal_cell_voltage_v', 2., 5.), ('voltage_gain', .8, 1.2),
                           ('voltage_offset_v', -1., 1.), ('valid_min_cell_v', 1., 4.),
                           ('valid_max_cell_v', 3., 5.), ('stale_after_sec', 5., 60.),
                           ('filter_time_constant_sec', .5, 120.)):
        number(key, low, high)
    if result['median_samples'] % 2 != 1 or type(result['curve_calibrated']) is not bool:
        raise StepError('INVALID_CONFIG', 'battery filter/calibration')
    curve = result.get('cell_voltage_curve')
    if not isinstance(curve, list) or not 3 <= len(curve) <= 64:
        raise StepError('INVALID_CONFIG', 'battery.cell_voltage_curve')
    previous = (-math.inf, -math.inf)
    for point in curve:
        if (not isinstance(point, list) or len(point) != 2
                or any(type(x) not in (int, float) or not math.isfinite(x) for x in point)
                or not result['valid_min_cell_v'] <= point[0] <= result['valid_max_cell_v']
                or not 0 <= point[1] <= 100 or point[0] <= previous[0] or point[1] <= previous[1]):
            raise StepError('INVALID_CONFIG', 'battery.cell_voltage_curve point')
        previous = point
    if curve[0][1] != 0 or curve[-1][1] != 100:
        raise StepError('INVALID_CONFIG', 'battery curve must span 0..100 percent')
    result['cell_voltage_curve'] = [tuple(point) for point in curve]
    return result


class BatteryEstimator:
    """Median + time-based EMA and configurable per-cell voltage/SOC curve.

    A voltage curve does not measure consumed charge or individual cell balance.
    Freshness follows sensor arrivals, never heartbeat or API requests.
    """
    def __init__(self, profile=None):
        self.profile = configuration(profile)
        self.lock = threading.Lock()
        self.values = deque(maxlen=self.profile['median_samples'] if self.profile else 5)
        self.raw = self.at = self.filtered = self.filtered_at = None
        self.valid = False

    def update(self, raw, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            previous = self.at
            self.raw, self.at = raw, now
            cfg = self.profile
            if cfg is None:
                return
            volts = raw / 1000. * cfg['voltage_gain'] + cfg['voltage_offset_v'] if type(raw) is int else math.nan
            self.valid = (type(raw) is int and 0 <= raw <= 65535
                          and cfg['valid_min_cell_v'] * cfg['series_cells'] <= volts
                          <= cfg['valid_max_cell_v'] * cfg['series_cells'])
            if (not self.valid or previous is None or now < previous
                    or now - previous > cfg['stale_after_sec']):
                self.values.clear()
                self.filtered = self.filtered_at = None
            if not self.valid:
                return
            self.values.append(volts)
            median = statistics.median(self.values)
            if self.filtered is None:
                self.filtered = median
            else:
                alpha = -math.expm1(-max(0., now - self.filtered_at) / cfg['filter_time_constant_sec'])
                self.filtered += alpha * (median - self.filtered)
            self.filtered_at = now

    def snapshot(self, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            if self.at is None:
                return None
            age = max(0., now - self.at)
            result = dict(value=self.raw, age_sec=age, unit='MILLIVOLT', percent=None,
                          voltage_v=None, estimate_method='UNAVAILABLE', estimate_status='UNCONFIGURED')
            cfg = self.profile
            if cfg is None:
                return result
            result.update(profile_id=cfg['profile_id'], series_cells=cfg['series_cells'],
                          parallel_cells=cfg['parallel_cells'], curve_calibrated=cfg['curve_calibrated'],
                          nominal_capacity_mah=cfg['nominal_cell_capacity_mah'] * cfg['parallel_cells'],
                          nominal_voltage_v=cfg['nominal_cell_voltage_v'] * cfg['series_cells'])
            if now < self.at or age > cfg['stale_after_sec']:
                result['estimate_status'] = 'STALE'
            elif not self.valid:
                result['estimate_status'] = 'INVALID'
            elif len(self.values) < cfg['minimum_samples']:
                result['estimate_status'] = 'WARMING_UP'
            else:
                cell = self.filtered / cfg['series_cells']
                curve = cfg['cell_voltage_curve']
                percent = curve[-1][1] if cell >= curve[-1][0] else curve[0][1]
                for (low_v, low_p), (high_v, high_p) in zip(curve, curve[1:]):
                    if low_v <= cell <= high_v:
                        percent = low_p + (cell - low_v) * (high_p - low_p) / (high_v - low_v)
                        break
                result.update(percent=max(0, min(100, math.floor(percent + .5))),
                              voltage_v=round(self.filtered, 3), estimate_method='VOLTAGE_CURVE',
                              estimate_status='ESTIMATED')
            return result
