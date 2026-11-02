"""Explicit, bounded configuration loading. No assumed hardware calibration."""

from dataclasses import dataclass
import math
from pathlib import Path
import yaml

from .geometry import rigid_transform
from .tracking import Tracker, TrackingParams

MAX_CONFIG_BYTES = 64 * 1024


@dataclass(frozen=True)
class Tag:
    vehicle_id: str
    size_m: float
    opponent_from_tag: object


@dataclass(frozen=True)
class Configuration:
    tracking_frame: str
    tags: dict
    tracking: TrackingParams
    max_error_px: float
    max_range_m: float


def parse_config(raw, require_calibrated=True):
    expected = {'schema_version', 'calibrated', 'tracking_frame', 'tags',
                'tracking', 'max_error_px', 'max_range_m'}
    if not isinstance(raw, dict) or set(raw) != expected:
        raise ValueError('configuration has missing or unknown keys')
    if (type(raw['schema_version']) is not int or raw['schema_version'] != 1
            or type(raw['calibrated']) is not bool):
        raise ValueError('invalid configuration version or calibrated flag')
    if require_calibrated and not raw['calibrated']:
        raise ValueError('calibration is not confirmed; measure mounts before live use')
    if not isinstance(raw['tracking'], dict):
        raise ValueError('tracking must be a mapping')
    try:
        params = TrackingParams(**raw['tracking'])
    except TypeError as error:
        raise ValueError('unknown tracking parameter') from error
    Tracker(raw['tracking_frame'], params)  # validates the frame contract
    for name in ('max_error_px', 'max_range_m'):
        if type(raw[name]) not in (int, float) or not math.isfinite(raw[name]) or raw[name] <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if not isinstance(raw['tags'], list) or not 1 <= len(raw['tags']) <= 32:
        raise ValueError('configure 1..32 tags')
    tags, vehicles = {}, set()
    for entry in raw['tags']:
        if not isinstance(entry, dict) or set(entry) != {
                'id', 'vehicle_id', 'size_m', 'opponent_from_tag'}:
            raise ValueError('tag has missing or unknown keys')
        tag_id, vehicle, size = entry['id'], entry['vehicle_id'], entry['size_m']
        if (type(tag_id) is not int or not 0 <= tag_id < 587 or tag_id in tags
                or not isinstance(vehicle, str) or not 1 <= len(vehicle) <= 64
                or vehicle in vehicles or type(size) not in (int, float)
                or not math.isfinite(size) or size <= 0):
            raise ValueError('invalid tag ID, size or duplicate vehicle; '
                             'one tag per vehicle in v1')
        tags[tag_id] = Tag(vehicle, float(size), rigid_transform(entry['opponent_from_tag']))
        vehicles.add(vehicle)
    return Configuration(raw['tracking_frame'], tags, params,
                         float(raw['max_error_px']), float(raw['max_range_m']))


def load_config(path):
    data = Path(path).read_bytes()
    if len(data) > MAX_CONFIG_BYTES:
        raise ValueError('configuration exceeds 64 KiB')
    try:
        raw = yaml.safe_load(data.decode('utf-8', errors='strict'))
    except (UnicodeError, yaml.YAMLError) as error:
        raise ValueError('configuration must be valid UTF-8 YAML') from error
    return parse_config(raw)
