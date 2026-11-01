"""Full lock oracles: hardware-reference.md circles and rb1's VESC map."""
import math
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racerbot_car.profile import ProfileError, load_profile

CARS = Path(__file__).resolve().parents[1] / 'cars'


@pytest.mark.parametrize('car', ['rb1', 'rb2'])
def test_profile_full_lock_is_cited_per_car(car):
    profile = load_profile(car, CARS)
    left, right = ((0.401, -(0.75 - 0.366) / 0.83) if car == 'rb1'
                   else (math.atan(0.36 / 0.645), -math.atan(0.36 / 0.750)))
    assert profile.full_lock_left_rad == pytest.approx(left, abs=1e-3)  # rad
    assert profile.full_lock_right_rad == pytest.approx(right, abs=1e-3)  # rad
    text = (CARS / car / 'car.yaml').read_text()
    for side in ('left', 'right'):
        line = next(line for line in text.splitlines() if line.strip().startswith(side + ':'))
        assert '#' in line
        if car == 'rb1':
            assert ('2026-10-21' if side == 'left' else 'not measured') in line
        else:
            assert '2026-10-16' in line and 'half circles' in line


@pytest.mark.parametrize('key,value', [
    ('left', 0.0), ('left', -0.1), ('right', 0.0), ('right', 0.1),
    ('left', float('nan')), ('right', float('nan')),
    ('left', float('inf')), ('right', -float('inf')),
    ('left', 0.8), ('right', -0.8), ('left', 0.81), ('right', -0.81),
    ('left', True), ('right', 'angle'),
    ('steering', None), ('full_lock_rad', None), ('left', None), ('right', None),
    ('unknown', 0.2),
])
def test_profile_refuses_bad_full_lock(tmp_path, key, value):
    data = yaml.safe_load((CARS / 'rb1/car.yaml').read_text())
    data['steering'] = {'full_lock_rad': {'left': 0.401, 'right': -0.463}}
    if value is None:
        if key == 'steering':
            del data[key]
        elif key == 'full_lock_rad':
            del data['steering'][key]
        else:
            del data['steering']['full_lock_rad'][key]
    else:
        data['steering']['full_lock_rad'][key] = value
    folder = tmp_path / 'rb1'
    folder.mkdir()
    (folder / 'car.yaml').write_text(yaml.safe_dump(data))
    with pytest.raises(ProfileError, match='car.yaml.*' + key):
        load_profile('rb1', tmp_path)
