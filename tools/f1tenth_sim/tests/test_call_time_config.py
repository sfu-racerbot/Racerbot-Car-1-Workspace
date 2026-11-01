"""Library calls select a car now, rather than inheriting a previous call."""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car.identity import UnknownCarError  # noqa: E402


def test_default_call_refreshes_changed_car(car_profiles, monkeypatch):
    import run_validation as rv

    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    rv._ensure_config()
    assert rv.CALIBRATION.speed_to_erpm_gain == 2000.0
    monkeypatch.setenv('RACERBOT_CAR', 'rb2')
    rv._ensure_config()
    # Oracle: the fixture overlay and eRPM / abs(gain), hardware-reference.md.
    assert rv.CALIBRATION.speed_to_erpm_gain == 4000.0
    assert rv.CALIBRATION.speed_max == pytest.approx(23250.0 / 4000.0, abs=1e-12)
    assert rv.STEERING_LIMIT == 0.3


def test_default_call_refuses_removed_selection(car_profiles, monkeypatch):
    import run_validation as rv

    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    rv._ensure_config()
    monkeypatch.delenv('RACERBOT_CAR')
    with pytest.raises(UnknownCarError, match='RACERBOT_CAR'):
        rv._ensure_config()


def test_command_shapers_keep_their_explicit_configuration(car_profiles, monkeypatch):
    import run_validation as rv

    first = rv.CommandShaper(car_id='rb1')
    second = rv.CommandShaper(car_id='rb2')
    monkeypatch.delenv('RACERBOT_CAR')
    # Oracle: straight-line slew ceiling a * dt, dt = 1/40 s. Creating the
    # second car must not change the first object's already selected limits.
    assert first.command(0.0, 5.0)[1] == pytest.approx(3.0 / 40.0, abs=1e-12)
    assert second.command(0.0, 5.0)[1] == pytest.approx(1.0 / 40.0, abs=1e-12)


def test_optimized_line_cache_is_car_and_calibration_specific(
        car_profiles, monkeypatch, tmp_path):
    import run_validation as rv

    csv = tmp_path / 'circle_centerline.csv'
    track = SimpleNamespace(filepath=tmp_path / 'circle_map.yaml',
                            spec=SimpleNamespace(name='circle'))

    def circle(radius):
        angles = np.linspace(0, 2 * np.pi, 100, endpoint=False)
        np.savetxt(csv, np.column_stack((radius * np.cos(angles),
                                        radius * np.sin(angles),
                                        np.ones(100), np.ones(100))), delimiter=',')

    # A uniform circular corridor's minimum-curvature line is its outermost
    # allowed circle: radius + half corridor width - half car width - margin.
    # The 2 cm tolerance covers polygon/resampling error, not the 1 m change.
    for car, radius in (('rb1', 5.0), ('rb2', 6.0), ('rb2', 7.0)):
        if radius == 7.0:
            car_profiles.set_calibration('rb2', speed_to_erpm_gain=4500.0)
        circle(radius)
        monkeypatch.setenv('RACERBOT_CAR', car)
        line = rv.optimized_raceline(track)
        assert np.max(np.abs(np.linalg.norm(line, axis=1)
                             - (radius + 1.0 - 0.31 / 2.0 - 0.15))) < 0.02
