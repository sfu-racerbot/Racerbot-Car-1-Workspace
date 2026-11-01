"""Resolve a car ID from the Jetson serial and an optional override."""

import os
import re
from collections.abc import Mapping
from pathlib import Path

import yaml

from . import paths

SERIAL_PATH = '/proc/device-tree/serial-number'
CAR_ID_PATTERN = re.compile(r'^rb[0-9]+$')
ENV_VAR = 'RACERBOT_CAR'


class UnknownCarError(RuntimeError):
    """The current car cannot be identified."""


class CarMismatchError(RuntimeError):
    """A serial and an override select different cars."""


def read_serial(path: str = SERIAL_PATH) -> str | None:
    try:
        serial = Path(path).read_text().strip('\x00 \t\r\n\v\f')
    except (OSError, UnicodeError):
        return None
    return serial or None


def known_car_ids(cars_dir: Path) -> set[str]:
    return {p.name for p in cars_dir.iterdir()
            if p.is_dir() and CAR_ID_PATTERN.fullmatch(p.name)
            and (p / 'car.yaml').is_file()}


class _UniqueLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as exc:
            raise ValueError(f'invalid mapping key {key!r}') from exc
        if duplicate:
            raise ValueError(f'duplicate machine serial {key!r}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def load_machines(path: Path, known_ids: set[str]) -> dict[str, str]:
    with path.open() as stream:
        data = yaml.load(stream, Loader=_UniqueLoader)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping')
    for serial, car_id in data.items():
        if not isinstance(serial, str):
            raise ValueError(f'{path}: {serial!r}: quote the serial')
        if not isinstance(car_id, str) or not CAR_ID_PATTERN.fullmatch(car_id):
            raise ValueError(f'{path}: {serial!r}: invalid car id {car_id!r}')
        if car_id not in known_ids:
            raise ValueError(f'{path}: {serial!r}: unknown car id {car_id!r}')
    return data


def resolve_car_id(env: Mapping[str, str], serial: str | None,
                   machines: dict[str, str], known_ids: set[str]) -> str:
    override = env.get(ENV_VAR, '')
    if serial in machines:
        car_id = machines[serial]
        if override and override != car_id:
            raise CarMismatchError(
                f'serial {serial} identifies {car_id}, but {ENV_VAR} is {override}; '
                f'unset {ENV_VAR}')
        return car_id
    if override:
        if override in known_ids:
            return override
        raise UnknownCarError(f'unknown {ENV_VAR} {override!r}; known ids: {sorted(known_ids)}')
    if serial is not None:
        raise UnknownCarError(f'unknown serial {serial}; add to cars/machines.yaml:\n"{serial}": <car id>')
    raise UnknownCarError(f'no Jetson serial was found; set {ENV_VAR} for a dev box or the simulator')


def current_car_id() -> str:
    directory = paths.cars_dir()
    ids = known_car_ids(directory)
    return resolve_car_id(os.environ, read_serial(), load_machines(directory / 'machines.yaml', ids), ids)
