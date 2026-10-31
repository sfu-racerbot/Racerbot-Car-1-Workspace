"""check_start: is the car really on the start pose?

Runner: python3 -m pytest src/pure_pursuit/test/test_check_start.py (no sourcing needed), and colcon test.

The oracle is a known map and a scan ray-cast here from a known pose, so the true offset is a closed form. These
cover the refusal side as well: this exists because seed_start_pose says yes wherever the car is.
"""
import ast
import math
from pathlib import Path

import numpy as np
import pytest

from pure_pursuit import check_start as cs
from pure_pursuit import run_commands

RES = 0.05
ANGLE_MIN = -math.radians(135.0)
ANGLE_INC = math.radians(0.25)
BEAMS = 1081
START = (5.0, 2.0, 0.0)


def _walls():
    g = np.zeros((int(14 / RES), int(24 / RES)), dtype=bool)
    g[:2, :] = g[-2:, :] = True
    g[:, :2] = g[:, -2:] = True
    g[int(4 / RES):int(10 / RES), int(5 / RES):int(19 / RES)] = True
    for index, x in enumerate(np.arange(3.0, 22.0, 2.5)):
        depth = 0.3 + 0.25 * (index % 3)
        g[2:int(depth / RES), int(x / RES):int(x / RES) + 8] = True
        g[-int((depth + 0.2) / RES):, int((x + 1) / RES):int((x + 1) / RES) + 8] = True
    return g


WALLS = _walls()


def _scan(pose, walls=WALLS):
    x, y, yaw = pose
    lx, ly = x + 0.26 * math.cos(yaw), y + 0.26 * math.sin(yaw)
    angles = yaw + ANGLE_MIN + ANGLE_INC * np.arange(BEAMS)
    distances = np.arange(0.1, 12.0, RES / 2)
    cols = ((lx + np.outer(np.cos(angles), distances)) / RES).astype(int)
    rows = ((ly + np.outer(np.sin(angles), distances)) / RES).astype(int)
    inside = (rows >= 0) & (rows < walls.shape[0]) & (cols >= 0) & (cols < walls.shape[1])
    hit = np.zeros(cols.shape, dtype=bool)
    hit[inside] = walls[rows[inside], cols[inside]]
    return np.where(hit.any(axis=1), distances[hit.argmax(axis=1)], np.inf)


@pytest.fixture(scope='module')
def run_dir(tmp_path_factory):
    folder = tmp_path_factory.mktemp('run')
    image = np.full(WALLS.shape, 254, np.uint8)
    image[WALLS] = 0
    with open(folder / 'map.pgm', 'wb') as handle:
        handle.write(f'P5\n{WALLS.shape[1]} {WALLS.shape[0]}\n255\n'.encode())
        handle.write(image[::-1].tobytes())
    (folder / 'map.yaml').write_text(
        f'image: map.pgm\nmode: trinary\nresolution: {RES}\norigin: [0.0, 0.0, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    run_commands.write_start_pose(folder, START)
    return folder


def _check(pose, run_dir):
    return cs.verdict(_scan(pose), ANGLE_MIN, ANGLE_INC, run_dir)


def test_go_on_the_start_pose(run_dir):
    check = _check(START, run_dir)
    assert check.go
    assert check.offset_m < 0.10  # m, two map cells
    assert check.offset_deg < 2.0
    assert cs.describe(check).startswith('GO')


def test_go_a_little_off_it(run_dir):
    # oracle: closed form. 0.15 m and 5 deg off, inside the 0.30 m and 12 deg limits.
    check = _check((5.15, 2.0, math.radians(5.0)), run_dir)
    assert check.go
    assert check.offset_m == pytest.approx(0.15, abs=0.08)  # m
    assert check.offset_deg == pytest.approx(5.0, abs=2.0)  # deg


def test_no_go_further_along_the_track(run_dir):
    # oracle: closed form. 4 m along the straight: the filter's 0.5 m seed spread would not reach it.
    check = _check((9.0, 2.0, 0.0), run_dir)
    assert not check.go
    assert check.fit >= cs.MIN_FIT  # it fits the map fine; it is just somewhere else
    assert check.offset_m == pytest.approx(4.0, abs=0.15)  # m


def test_no_go_turned_on_the_spot(run_dir):
    check = _check((5.0, 2.0, math.radians(30.0)), run_dir)
    assert not check.go
    assert check.offset_deg == pytest.approx(30.0, abs=3.0)  # deg
    assert check.offset_m < 0.15  # m: the right place, facing the wrong way


def test_no_go_on_the_spot_when_the_scan_does_not_match(run_dir):
    # oracle: invariant. The car is exactly on the start, but half its beams stop at something 0.6 m away that
    # the map does not have, such as people round the car. The filter would be matching against that too, so
    # a good place is not enough: the scan has to fit the map from it.
    ranges = _scan(START)
    ranges[::2] = 0.6
    check = cs.verdict(ranges, ANGLE_MIN, ANGLE_INC, run_dir)
    assert not check.go
    assert 'fits nowhere' in cs.describe(check)


def test_no_go_says_how_to_fix_it_in_the_cars_own_terms(run_dir):
    # oracle: closed form. The car is 1 m ahead of the start, facing the same way: move it 1 m back.
    text = cs.describe(_check((6.0, 2.0, 0.0), run_dir))
    assert text.startswith('NO-GO')
    forward = float(text.split('about ')[1].split(' m forward')[0])
    assert forward == pytest.approx(-1.0, abs=0.15)  # m


def test_no_go_on_a_blank_scan(run_dir):
    # A scan consumer's edge: all-inf. Nothing to place the car with is never a yes.
    check = cs.verdict(np.full(BEAMS, np.inf), ANGLE_MIN, ANGLE_INC, run_dir)
    assert not check.go
    assert cs.describe(check).startswith('NO-GO')


@pytest.mark.parametrize('bad', [np.nan, 0.0, -1.0])
def test_no_go_on_a_scan_of_garbage(run_dir, bad):
    check = cs.verdict(np.full(BEAMS, bad), ANGLE_MIN, ANGLE_INC, run_dir)
    assert not check.go


def test_no_go_on_the_wrong_map(run_dir):
    # oracle: invariant. A scan from a place that map does not contain fits nowhere on it.
    other = np.zeros((280, 480), dtype=bool)
    other[:2, :] = other[-2:, :] = True
    other[:, :2] = other[:, -2:] = True
    other[100:115, 100:130] = other[60:80, 300:330] = other[200:210, 200:220] = True
    check = cs.verdict(_scan((3.0, 3.0, 0.7), other), ANGLE_MIN, ANGLE_INC, run_dir)
    assert not check.go
    assert 'fits nowhere' in cs.describe(check)


@pytest.mark.parametrize('limit, expected', [
    pytest.param(0.0, False, id='offset-past-a-zero-limit'),
    pytest.param(1.0, True, id='offset-within-a-wide-limit'),
])
def test_the_offset_limit_is_what_decides(run_dir, monkeypatch, limit, expected):
    # 0.15 m off, with the limit moved either side of it, so the limit itself is shown to bite.
    monkeypatch.setattr(cs, 'MAX_OFFSET_M', limit)
    assert _check((5.15, 2.0, 0.0), run_dir).go is expected


def test_a_folder_with_no_start_pose_is_refused(tmp_path):
    with pytest.raises(ValueError, match='start_pose'):
        cs.verdict(np.full(BEAMS, 3.0), ANGLE_MIN, ANGLE_INC, tmp_path)


def test_it_publishes_nothing():
    # oracle: invariant. Support code, not driving code (docs/adding-your-own-code.md): if it ever gains a
    # publisher, it has to be looked at again under the deadman policy.
    tree = ast.parse(Path(cs.__file__).read_text())
    calls = [node.func.attr for node in ast.walk(tree)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)]
    assert 'create_subscription' in calls  # the walk does see node calls
    assert 'create_publisher' not in calls


def test_race_commands_name_the_check():
    text = run_commands.commands_text('/some/run')
    assert 'check_start' in text
    assert 'run_dir:=/some/run' in text
    assert text.index('check_start') < text.index('race_launch.py')
