import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_car import paths


def test_cars_dir_env_override(tmp_path, monkeypatch):
    monkeypatch.setenv('RACERBOT_CARS_DIR', str(tmp_path))
    assert paths.cars_dir() == tmp_path


def test_cars_dir_source_tree_default(monkeypatch):
    monkeypatch.delenv('RACERBOT_CARS_DIR', raising=False)
    expected = Path(__file__).resolve().parents[1] / 'cars'
    assert paths.cars_dir() == expected
    assert (expected / 'machines.yaml').is_file()


def test_workspace_root_env_and_default(tmp_path, monkeypatch):
    monkeypatch.setenv('RACERBOT_WS', '/x')
    assert paths.workspace_root() == Path('/x')
    monkeypatch.delenv('RACERBOT_WS')
    monkeypatch.setenv('HOME', str(tmp_path))
    assert paths.workspace_root() == tmp_path / 'racerbot-ws'


def test_data_dir_layout(monkeypatch):
    monkeypatch.setenv('RACERBOT_WS', '/x')
    assert paths.data_dir('racerbot_runs', 'rb1') == Path('/x/data/rb1/racerbot_runs')


def test_data_dir_uses_current_car(tmp_path, monkeypatch):
    monkeypatch.setenv('RACERBOT_WS', str(tmp_path))
    monkeypatch.setattr(paths, 'current_car_id', lambda: 'rb2')
    assert paths.data_dir('racerbot_auto') == tmp_path / 'data/rb2/racerbot_auto'


def test_data_dir_unknown_kind_rejected():
    with pytest.raises(ValueError, match='racerbot_runs'):
        paths.data_dir('wrong', 'rb1')


def test_data_dir_creates_nothing(tmp_path, monkeypatch):
    monkeypatch.setenv('RACERBOT_WS', str(tmp_path))
    result = paths.data_dir('imu_calibration', 'rb2')
    assert result == tmp_path / 'data/rb2/imu_calibration'
    assert not (tmp_path / 'data').exists()
