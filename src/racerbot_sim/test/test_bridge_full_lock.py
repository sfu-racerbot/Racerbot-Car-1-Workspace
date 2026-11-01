"""Selected physical model, from node identity through gym parameters."""
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/racerbot_sim'))
sys.path.insert(0, str(ROOT))


@pytest.mark.parametrize('car', ['rb1', 'rb2'])
def test_bridge_models_the_selected_cars_full_lock(car):
    from racerbot_sim.sim_bridge import resolve_car_profile, vehicle_parameter_updates
    profile = resolve_car_profile(car)
    params = vehicle_parameter_updates(profile.full_lock_left_rad, profile.full_lock_right_rad)
    # Independent oracles: docs/hardware-reference.md and rb1/calibration/vesc.yaml.
    left, right = ((0.401, -(0.75 - 0.366) / 0.83) if car == 'rb1'
                   else (math.atan(0.36 / 0.645), -math.atan(0.36 / 0.750)))
    assert params['s_max'] == pytest.approx(left, abs=1e-3)  # rad
    assert params['s_min'] == pytest.approx(right, abs=1e-3)  # rad


def test_sim_bringup_passes_the_car_to_the_bridge(monkeypatch):
    from tools.dev.launch_params import capture
    monkeypatch.setenv('RACERBOT_CAR', 'rb1')
    result = capture(str(ROOT / 'src/racerbot_sim/launch/sim_bringup_launch.py'), [], str(ROOT))
    assert not result['errors']
    bridges = [node for node in result['nodes'] if node['executable'] == 'gym_bridge_node']
    assert len(bridges) == 1
    assert bridges[0]['params']['car'] == 'rb1'


def test_bridge_refuses_without_a_car(monkeypatch, tmp_path):
    from racerbot_car import identity
    from racerbot_sim.sim_bridge import resolve_car_profile
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    # Filesystem boundary: an unlisted machine with no identity override.
    monkeypatch.setattr(identity, 'SERIAL_PATH', tmp_path / 'absent-serial')
    with pytest.raises(ValueError, match='car.*RACERBOT_CAR'):
        resolve_car_profile('')
