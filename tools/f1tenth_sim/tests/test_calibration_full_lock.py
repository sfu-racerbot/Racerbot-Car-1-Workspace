"""Full lock must follow the explicit car, even against the environment."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_fidelity.calibration import calibration_for_car


@pytest.mark.parametrize('car', ['rb1', 'rb2'])
def test_calibration_for_car_uses_the_cars_full_lock(car):
    calibration = calibration_for_car(car)
    # Oracles: hardware-reference.md circles; rb1/calibration/vesc.yaml servo map.
    left, right = ((0.401, -(0.75 - 0.366) / 0.83) if car == 'rb1'
                   else (math.atan(0.36 / 0.645), -math.atan(0.36 / 0.750)))
    assert calibration.steering_max_left == pytest.approx(left, abs=1e-3)  # rad
    assert calibration.steering_max_right == pytest.approx(right, abs=1e-3)  # rad
