"""Per-car launch wiring, specified by unified workspace plan Task 12."""

import importlib.util
import re
import shutil
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from racerbot_car import hardware
from racerbot_car.identity import UnknownCarError


PACKAGE = Path(__file__).resolve().parents[1]
CARS = PACKAGE.parent / 'racerbot_car' / 'cars'


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def includes(description):
    return [entity for entity in description.entities
            if isinstance(entity, IncludeLaunchDescription)]


def location(include):
    substitutions = include.launch_description_source._LaunchDescriptionSource__location
    return perform_substitutions(LaunchContext(), substitutions)


@pytest.fixture(autouse=True)
def no_usb_devices(monkeypatch):
    monkeypatch.setattr(hardware, 'usb_ids_present', lambda: set())
    monkeypatch.delenv('RACERBOT_CARS_DIR', raising=False)


@pytest.mark.parametrize('car', ['rb1', 'rb2'])
def test_dashboard_launch_passes_the_running_cars_yaml(monkeypatch, car):
    monkeypatch.setenv('RACERBOT_CAR', car)
    description = load_launch('dashboard_launch.py').generate_launch_description()
    (include,) = includes(description)
    arguments = dict(include.launch_arguments)
    actual = perform_substitutions(
        LaunchContext(), normalize_to_list_of_substitutions(arguments['car_config']))
    expected = Path(get_package_share_directory('racerbot_launch')) / 'config' / f'web_dashboard_{car}.yaml'
    assert actual == str(expected)


def test_dashboard_launch_refuses_an_unknown_car(monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rb9')
    with pytest.raises(UnknownCarError, match='rb9'):
        load_launch('dashboard_launch.py').generate_launch_description()


def test_dashboard_launch_refuses_a_car_with_no_dashboard_yaml(monkeypatch, tmp_path):
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    module = load_launch('dashboard_launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(tmp_path))
    expected = tmp_path / 'config' / 'web_dashboard_rb1.yaml'
    with pytest.raises(FileNotFoundError, match=re.escape(str(expected))):
        module.generate_launch_description()


@pytest.mark.parametrize('car, expected', [
    ('rb1', 'webcam_launch.py'),
    ('rb2', 'realsense_camera_launch.py'),
])
def test_camera_launch_picks_by_profile(monkeypatch, car, expected):
    monkeypatch.setenv('RACERBOT_CAR', car)
    (include,) = includes(load_launch('camera_launch.py').generate_launch_description())
    assert location(include).endswith('/' + expected)


def test_camera_launch_starts_nothing_for_a_car_without_a_camera(monkeypatch, tmp_path):
    car_dir = tmp_path / 'rb1'
    car_dir.mkdir()
    profile = yaml.safe_load((CARS / 'rb1' / 'car.yaml').read_text())
    profile['camera'] = {'kind': 'none'}
    (car_dir / 'car.yaml').write_text(yaml.safe_dump(profile))
    shutil.copytree(CARS / 'rb1' / 'calibration', car_dir / 'calibration')
    (tmp_path / 'machines.yaml').write_text('{}\n')
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(tmp_path))
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    description = load_launch('camera_launch.py').generate_launch_description()
    assert includes(description) == []
    assert len(description.entities) == 1
    assert isinstance(description.entities[0], LogInfo)


def test_dashboard_test_launch_includes_the_camera_launch(monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    description = load_launch('dashboard_test_launch.py').generate_launch_description()
    paths = [location(include) for include in includes(description)]
    assert sum(path.endswith('/camera_launch.py') for path in paths) == 1
    assert not any(path.endswith('/webcam_launch.py') for path in paths)
