"""Load strictly validated car hardware profiles."""

import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

from . import identity, paths


class ProfileError(ValueError):
    """A profile is missing or has an invalid field."""


@dataclass(frozen=True)
class LidarSerial:
    port: str
    usb_id: str


@dataclass(frozen=True)
class LidarEthernet:
    ip: str
    port: int
    iface: str
    host_ip: str


@dataclass(frozen=True)
class CameraUVC:
    device: str


@dataclass(frozen=True)
class CameraRealSense:
    usb_id: str
    topic: str


@dataclass(frozen=True)
class CarProfile:
    id: str
    name: str
    lidar: LidarSerial | LidarEthernet
    camera: CameraUVC | CameraRealSense | None
    dashboard_site: str
    hostnames: dict[str, str]
    distinctive_usb_ids: tuple[str, ...]
    full_lock_left_rad: float
    full_lock_right_rad: float


def _mapping(value, required, optional, path, key):
    if not isinstance(value, dict):
        raise ProfileError(f'{path}: {key}: expected mapping')
    for name in value:
        if name not in required and name not in optional:
            raise ProfileError(f'{path}: {key}{name}: unknown key')
    for name in required:
        if name not in value:
            raise ProfileError(f'{path}: {key}{name}: missing key')
    return value


def _string(value, path, key, usb=False):
    if not isinstance(value, str):
        raise ProfileError(f'{path}: {key}: expected string')
    if usb and not re.fullmatch(r'[0-9a-f]{4}:[0-9a-f]{4}', value):
        raise ProfileError(f'{path}: {key}: invalid USB id {value!r}')
    return value


def load_profile(car_id: str, cars_dir: Path | None = None) -> CarProfile:
    directory = cars_dir if cars_dir is not None else paths.cars_dir()
    path = directory / car_id / 'car.yaml'
    try:
        with path.open() as stream:
            data = yaml.safe_load(stream)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ProfileError(f'{path}: profile: {exc}') from exc
    data = _mapping(data, {'name', 'lidar', 'camera', 'dashboard_site', 'distinctive_usb_ids', 'steering'},
                    {'hostnames'}, path, '')
    name = _string(data['name'], path, 'name')
    dashboard = _string(data['dashboard_site'], path, 'dashboard_site')
    steering = _mapping(data['steering'], {'full_lock_rad'}, set(), path, 'steering.')
    locks = _mapping(steering['full_lock_rad'], {'left', 'right'}, set(),
                     path, 'steering.full_lock_rad.')
    for side, sign in (('left', 1), ('right', -1)):
        angle = locks[side]
        if (type(angle) not in (int, float) or not math.isfinite(angle)
                or not 0 < sign * angle < 0.8):
            raise ProfileError(f'{path}: steering.full_lock_rad.{side}: '
                               'expected finite angle with correct sign and magnitude < 0.8 rad')

    lidar = data['lidar']
    _mapping(lidar, {'kind'}, set(lidar) - {'kind'} if isinstance(lidar, dict) else set(), path, 'lidar.')
    kind = lidar['kind']
    if kind == 'serial':
        _mapping(lidar, {'kind', 'port', 'usb_id'}, set(), path, 'lidar.')
        laser = LidarSerial(_string(lidar['port'], path, 'lidar.port'),
                            _string(lidar['usb_id'], path, 'lidar.usb_id', usb=True))
    elif kind == 'ethernet':
        _mapping(lidar, {'kind', 'ip', 'port', 'iface', 'host_ip'}, set(), path, 'lidar.')
        if type(lidar['port']) is not int:
            raise ProfileError(f'{path}: lidar.port: expected integer')
        laser = LidarEthernet(_string(lidar['ip'], path, 'lidar.ip'), lidar['port'],
                              _string(lidar['iface'], path, 'lidar.iface'),
                              _string(lidar['host_ip'], path, 'lidar.host_ip'))
    else:
        raise ProfileError(f'{path}: lidar.kind: invalid kind {kind!r}')

    camera = data['camera']
    _mapping(camera, {'kind'}, set(camera) - {'kind'} if isinstance(camera, dict) else set(), path, 'camera.')
    kind = camera['kind']
    if kind == 'uvc':
        _mapping(camera, {'kind', 'device'}, set(), path, 'camera.')
        cam = CameraUVC(_string(camera['device'], path, 'camera.device'))
    elif kind == 'realsense':
        _mapping(camera, {'kind', 'usb_id', 'topic'}, set(), path, 'camera.')
        cam = CameraRealSense(_string(camera['usb_id'], path, 'camera.usb_id', usb=True),
                              _string(camera['topic'], path, 'camera.topic'))
    elif kind == 'none':
        _mapping(camera, {'kind'}, set(), path, 'camera.')
        cam = None
    else:
        raise ProfileError(f'{path}: camera.kind: invalid kind {kind!r}')

    hostnames = data.get('hostnames', {})
    if not isinstance(hostnames, dict):
        raise ProfileError(f'{path}: hostnames: expected mapping')
    for key, value in hostnames.items():
        _string(key, path, 'hostnames')
        _string(value, path, f'hostnames.{key}')
    usb_ids = data['distinctive_usb_ids']
    if not isinstance(usb_ids, list):
        raise ProfileError(f'{path}: distinctive_usb_ids: expected list')
    ids = tuple(_string(value, path, f'distinctive_usb_ids[{i}]', usb=True)
                for i, value in enumerate(usb_ids))
    return CarProfile(car_id, name, laser, cam, dashboard, hostnames, ids,
                      float(locks['left']), float(locks['right']))


def all_profiles(cars_dir: Path | None = None) -> list[CarProfile]:
    directory = cars_dir if cars_dir is not None else paths.cars_dir()
    return [load_profile(car_id, directory) for car_id in sorted(identity.known_car_ids(directory))]
