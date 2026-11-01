import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car import guards, layers

ROOT = Path(__file__).resolve().parents[3]
CARS = ROOT / 'src/racerbot_car/cars'
CONFIGS = {name: ROOT / 'src/f1tenth_system/f1tenth_stack/config' / name
           for name in ('vesc.yaml', 'sensors.yaml', 'joy_teleop.yaml')}
CONFIGS.update({f'{name}.yaml': ROOT / 'src' / name / 'config' / f'{name}.yaml'
                for name in ('gap_follow', 'pure_pursuit', 'neural_racer', 'odom_calibration')})
OVERLAYS = sorted(CARS.glob('*/calibration/*.yaml'))


@pytest.mark.parametrize('overlay', OVERLAYS)
def test_calibration_overlay_valid(overlay):
    assert guards.check_calibration_overlay(overlay, overlay.name) == []
    assert layers.check_block_keys_match(str(CONFIGS[overlay.name]), str(overlay)) == []


@pytest.mark.parametrize('overlay', sorted((CARS / 'rb1/calibration').glob('*.yaml')))
def test_rb1_overlay_values_equal_shared_values_today(overlay):
    # Oracle: shared copies remain unchanged until a later task removes them.
    shared = guards.overlay_blocks(CONFIGS[overlay.name])
    for block, values in guards.overlay_blocks(overlay).items():
        for key, value in values.items():
            assert shared[block][key] == value
