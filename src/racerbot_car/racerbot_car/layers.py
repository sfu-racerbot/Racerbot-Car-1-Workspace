"""Resolve car-specific ROS parameter files and their effective values."""

from pathlib import Path

from . import guards, identity, paths


class MissingCalibrationError(RuntimeError):
    """A required car calibration is absent."""


def overlay_files(config_name: str, car_id: str | None = None,
                  cars_dir: Path | None = None) -> list[str]:
    car_id = car_id if car_id is not None else identity.current_car_id()
    directory = Path(cars_dir if cars_dir is not None else paths.cars_dir()) / car_id
    candidates = [directory / kind / config_name for kind in ('calibration', 'tuning')]
    return [str(path.resolve()) for path in candidates if path.is_file()]


def layered(shared_path: str, config_name: str, car_id: str | None = None,
            cars_dir: Path | None = None) -> list[str]:
    car_id = car_id if car_id is not None else identity.current_car_id()
    directory = Path(cars_dir if cars_dir is not None else paths.cars_dir())
    calibration = directory / car_id / 'calibration' / config_name
    if config_name in guards.CALIBRATION_KEYS:
        blocks = guards.overlay_blocks(calibration) if calibration.is_file() else {}
        missing = [f'{block}.{key}' for block, keys in guards.CALIBRATION_KEYS[config_name].items()
                   for key, requirement in keys.items()
                   if requirement == 'required' and key not in blocks.get(block, {})]
        if missing:
            raise MissingCalibrationError(
                f'Car {car_id}: {calibration} is missing required calibration keys: '
                f'{", ".join(missing)}. Configure src/racerbot_car/cars/{car_id}/calibration/{config_name}.')
    return [shared_path, *overlay_files(config_name, car_id, directory)]


def effective_values(paths: list[str], node_name: str | None) -> dict:
    entries = {}
    for path in paths:
        for block, params in guards.overlay_blocks(Path(path)).items():
            entries.setdefault(block, {}).update(params)
    keys = {'/**'}
    if node_name:
        if not node_name.startswith('/'):
            node_name = '/' + node_name
        basename = node_name.rsplit('/', 1)[-1]
        keys.update((node_name, basename, '/**/' + basename))
    result = {}
    for block, params in entries.items():
        if block in keys:
            result.update(params)
    return result


def check_block_keys_match(shared_path: str, overlay_path: str) -> list[str]:
    shared = guards.overlay_blocks(Path(shared_path))
    overlay = guards.overlay_blocks(Path(overlay_path))
    problems = []
    for block, params in overlay.items():
        for key in params:
            shared_blocks = [name for name, values in shared.items() if key in values]
            if shared_blocks and block not in shared_blocks:
                problems.append(f'{overlay_path}: {block}.{key} uses a different block than {shared_path}: '
                                f'{", ".join(shared_blocks)}.')
    return problems
