"""Locate car profiles and per-car data without creating directories."""

import os
from pathlib import Path

DATA_KINDS = ('racerbot_runs', 'racerbot_auto', 'racerbot_sim', 'imu_calibration')


def cars_dir() -> Path:
    override = os.environ.get('RACERBOT_CARS_DIR')
    if override:
        return Path(override)
    source = Path(__file__).resolve().parents[1] / 'cars'
    if source.is_dir():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return (Path(get_package_share_directory('racerbot_car')) / 'cars').resolve()
    except (ImportError, LookupError):
        raise FileNotFoundError(
            'racerbot_car is not built and RACERBOT_CARS_DIR is unset') from None


def workspace_root() -> Path:
    return Path(os.environ.get('RACERBOT_WS') or '~/racerbot-ws').expanduser()


def current_car_id() -> str:
    from .identity import current_car_id as resolve
    return resolve()


def data_dir(kind: str, car_id: str | None = None) -> Path:
    if kind not in DATA_KINDS:
        raise ValueError(f'unknown data kind {kind!r}; expected one of {DATA_KINDS}')
    return workspace_root() / 'data' / (car_id or current_car_id()) / kind
