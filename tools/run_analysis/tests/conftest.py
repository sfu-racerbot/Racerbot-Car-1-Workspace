"""Select a calibrated car for analysis tests that use the current config."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/racerbot_car'))
from racerbot_car.identity import known_car_ids  # noqa: E402
from racerbot_car.paths import cars_dir  # noqa: E402

CARS = sorted(car for car in known_car_ids(cars_dir())
              if (cars_dir() / car / 'calibration/vesc.yaml').is_file())


@pytest.fixture(autouse=True, params=CARS)
def selected_test_car(request, monkeypatch):
    monkeypatch.setenv('RACERBOT_CAR', request.param)
