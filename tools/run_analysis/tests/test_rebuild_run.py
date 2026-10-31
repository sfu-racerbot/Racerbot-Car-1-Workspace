"""rebuild_run.py, from a recorded run to a folder race_launch.py can use, or a stated refusal.

Runner: python3 -m pytest tools/run_analysis/tests/ (no sourcing needed: arrays come through a fake loader).

The oracle is a course built here with its own ray-caster, so true poses and the true lap length are known in
closed form. Reading a real bag is exercised by running the tool on real runs: results in
docs/superpowers/specs/2026-10-30-slam-reliability.md.
"""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from run_analysis import rebuild_run as rb  # noqa: E402
from pure_pursuit import occupancy_map, racing_math, run_commands  # noqa: E402

RES = 0.05
ANGLE_MIN = -math.radians(135.0)
ANGLE_INC = math.radians(0.25)
BEAMS = 1081
WEST, EAST, SOUTH, NORTH, RADIUS = 3.5, 20.5, 2.0, 12.0, 2.0
LAP_M = 2 * (EAST - WEST - 2 * RADIUS) + 2 * (NORTH - SOUTH - 2 * RADIUS) + 2 * math.pi * RADIUS


def _course():
    """A walled ring with uneven bumps along its outer walls, clear of the car's path."""
    g = np.zeros((int(14 / RES), int(24 / RES)), dtype=bool)
    g[:2, :] = g[-2:, :] = True
    g[:, :2] = g[:, -2:] = True
    g[int(4 / RES):int(10 / RES), int(5 / RES):int(19 / RES)] = True
    for index, x in enumerate(np.arange(3.0, 22.0, 2.5)):
        depth = 0.3 + 0.25 * (index % 3)
        width = int((0.3 + 0.2 * (index % 2)) / RES)
        g[2:int(depth / RES), int(x / RES):int(x / RES) + width] = True
        g[-int((depth + 0.2) / RES):, int((x + 1) / RES):int((x + 1) / RES) + width] = True
    for index, y in enumerate(np.arange(3.0, 12.0, 2.5)):
        depth = 0.3 + 0.2 * (index % 3)
        g[int(y / RES):int((y + 0.4) / RES), 2:int(depth / RES)] = True
        g[int((y + 1) / RES):int((y + 1.4) / RES), -int(depth / RES):] = True
    return g


def _ray_cast(pose, walls):
    x, y, yaw = pose
    lx, ly = x + 0.26 * math.cos(yaw), y + 0.26 * math.sin(yaw)
    angles = yaw + ANGLE_MIN + ANGLE_INC * np.arange(BEAMS)
    distances = np.arange(0.1, 12.0, RES / 2)
    cols = ((lx + np.outer(np.cos(angles), distances)) / RES).astype(int)
    rows = ((ly + np.outer(np.sin(angles), distances)) / RES).astype(int)
    inside = (rows >= 0) & (rows < walls.shape[0]) & (cols >= 0) & (cols < walls.shape[1])
    hit = np.zeros(cols.shape, dtype=bool)
    hit[inside] = walls[rows[inside], cols[inside]]
    return np.where(hit.any(axis=1), distances[hit.argmax(axis=1)], np.inf).astype(np.float32)


def _pose_at(s):
    """Anticlockwise on a rounded rectangle centred in the corridor."""
    legs = [EAST - WEST - 2 * RADIUS, NORTH - SOUTH - 2 * RADIUS] * 2
    arc = math.pi / 2 * RADIUS
    s = s % LAP_M
    corners = [(WEST + RADIUS, SOUTH), (EAST, SOUTH + RADIUS), (EAST - RADIUS, NORTH), (WEST, NORTH - RADIUS)]
    for leg, (cx, cy), heading in zip(legs, corners, (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)):
        if s <= leg:
            return cx + s * math.cos(heading), cy + s * math.sin(heading), heading
        s -= leg
        if s <= arc:
            t = s / RADIUS
            ex, ey = cx + leg * math.cos(heading), cy + leg * math.sin(heading)
            ox, oy = ex - RADIUS * math.sin(heading), ey + RADIUS * math.cos(heading)
            return ox + RADIUS * math.sin(heading + t), oy - RADIUS * math.cos(heading + t), heading + t
        s -= arc
    raise AssertionError('past the lap')


COURSE = _course()


def _inputs(laps=2.3, map_walls=COURSE, spacing=0.4, odom_source='/odom/imu'):
    """The arrays a recorded run yields: true scans, and a grid that may or may not match them."""
    distances = np.arange(0.0, laps * LAP_M, spacing)
    times = 100.0 + distances / 2.5
    poses = np.array([_pose_at(s) for s in distances])
    poses[:, 2] = np.unwrap(poses[:, 2])
    scans = np.array([_ray_cast(pose, COURSE) for pose in poses])
    grid = np.zeros(map_walls.shape, dtype=np.int8)
    grid[map_walls] = 100
    return rb.RunInputs(grid=grid, resolution=RES, origin_x=0.0, origin_y=0.0, times=times, scans=scans,
                        angle_min=ANGLE_MIN, angle_increment=ANGLE_INC, odom=np.c_[times, poses],
                        odom_source=odom_source)


@pytest.fixture(scope='module')
def good_inputs():
    return _inputs()


def _rebuild(tmp_path, inputs, **kwargs):
    run = tmp_path / '20261030-120000'
    run.mkdir(exist_ok=True)
    return rb.rebuild(run, tmp_path / 'auto', load=lambda _: inputs, **kwargs)


def _doubled():
    bad = COURSE.copy()
    bad[int(3.2 / RES):int(3.3 / RES), int(5 / RES):int(19 / RES)] = True  # a false wall across open corridor
    bad[int(1.0 / RES):int(1.1 / RES), int(3 / RES):int(21 / RES)] = True
    bad[int(11.0 / RES):int(11.1 / RES), int(3 / RES):int(21 / RES)] = True
    bad[int(3 / RES):int(11 / RES), int(3.0 / RES):int(3.1 / RES)] = True
    bad[int(3 / RES):int(11 / RES), int(21.0 / RES):int(21.1 / RES)] = True
    return bad


# --- an accepted run ------------------------------------------------------------------------------

def test_a_good_run_is_accepted_with_every_file(tmp_path, good_inputs):
    result = _rebuild(tmp_path, good_inputs)
    assert result.verdict.ok
    assert result.raceline_ok
    assert result.out_dir == tmp_path / 'auto' / '20261030-120000-rebuilt'
    for name in ('map.yaml', 'map.pgm', 'raceline_raw.csv', 'raceline_profiled.csv', 'raceline_2p5.csv',
                 'raceline_4p0.csv', 'start_pose.yaml', 'commands.txt', 'rebuild.json'):
        assert (result.out_dir / name).is_file(), name


def test_the_saved_map_loads_and_matches_the_input(tmp_path, good_inputs):
    # oracle: invariant. What goes to disk is the same grid, the right way up: row 0 at the lowest y.
    result = _rebuild(tmp_path, good_inputs)
    saved = occupancy_map.OccupancyMap.from_yaml(str(result.out_dir / 'map.yaml'), inflate_cells=0)
    assert saved.resolution == pytest.approx(RES, abs=1e-9)
    walls = saved.grid == occupancy_map.OCCUPIED
    assert walls.shape == COURSE.shape
    assert (walls == COURSE).mean() > 0.999


def test_the_line_is_the_course(tmp_path, good_inputs):
    # oracle: closed form. The car drove one known rounded rectangle, so the line must be that long and
    # stay on that path, here within 0.25 m where cleaning rounds the corners.
    result = _rebuild(tmp_path, good_inputs)
    xy, _ = racing_math.load_profiled_csv(str(result.out_dir / 'raceline_profiled.csv'))[:2]
    length = racing_math.compute_segment_lengths(xy, closed=True).sum()
    assert length == pytest.approx(LAP_M, abs=1.0)  # m, 2% of a lap
    truth = np.array([_pose_at(s)[:2] for s in np.arange(0.0, LAP_M, 0.05)])
    nearest = np.min(np.hypot(xy[:, 0, None] - truth[:, 0], xy[:, 1, None] - truth[:, 1]), axis=1)
    assert nearest.max() < 0.25  # m


def test_the_first_line_is_capped_and_the_faster_ones_are_not(tmp_path, good_inputs):
    # oracle: invariant. The profile a car first races tops out at max_speed, whatever the corners allow.
    result = _rebuild(tmp_path, good_inputs, max_speed=1.5)
    _, speed = racing_math.load_profiled_csv(str(result.out_dir / 'raceline_profiled.csv'))[:2]
    assert speed.max() == pytest.approx(1.5, abs=1e-3)  # m/s
    _, faster = racing_math.load_profiled_csv(str(result.out_dir / 'raceline_4p0.csv'))[:2]
    assert faster.max() > 2.0


def test_max_speed_is_honoured_when_changed(tmp_path, good_inputs):
    result = _rebuild(tmp_path, good_inputs, max_speed=1.0)
    _, speed = racing_math.load_profiled_csv(str(result.out_dir / 'raceline_profiled.csv'))[:2]
    assert speed.max() == pytest.approx(1.0, abs=1e-3)  # m/s


def test_the_start_pose_is_where_the_car_stopped(tmp_path, good_inputs):
    # oracle: the last true pose. That is where the car is parked now, so it is where racing starts.
    result = _rebuild(tmp_path, good_inputs)
    x, y, yaw = run_commands.read_start_pose(result.out_dir)
    truth = good_inputs.odom[-1, 1:]
    assert math.hypot(x - truth[0], y - truth[1]) < 0.15  # m
    assert abs(math.remainder(yaw - truth[2], 2 * math.pi)) < math.radians(3.0)


def test_the_report_records_what_was_measured(tmp_path, good_inputs):
    result = _rebuild(tmp_path, good_inputs)
    report = json.loads((result.out_dir / 'rebuild.json').read_text())
    assert report['verdict']['ok'] is True
    assert report['odom_source'] == '/odom/imu'
    assert report['laps'] == result.lap_count == 2
    assert report['verdict']['mean_fit'] > 0.9


def test_commands_say_how_to_race_and_at_what_speed(tmp_path, good_inputs):
    result = _rebuild(tmp_path, good_inputs)
    text = (result.out_dir / 'commands.txt').read_text()
    assert f'race_launch.py run_dir:={result.out_dir}' in text
    assert '1.5 m/s' in text
    assert 'raceline_2p5.csv' in text


# --- refusals: review focus 1-3 --------------------------------------------------------------------

def test_a_doubled_map_is_refused_with_no_line(tmp_path):
    result = _rebuild(tmp_path, _inputs(map_walls=_doubled()))
    assert not result.verdict.ok
    assert not result.raceline_ok
    assert 'see-through' in result.message
    assert not (result.out_dir / 'raceline_profiled.csv').exists()  # so race_launch.py refuses the folder


def test_a_refused_map_gives_no_race_command(tmp_path):
    result = _rebuild(tmp_path, _inputs(map_walls=_doubled()))
    text = (result.out_dir / 'commands.txt').read_text()
    assert 'race_launch.py' not in text
    assert 'see-through' in text
    assert 'Map again' in text


def test_a_map_of_somewhere_else_is_refused(tmp_path):
    # oracle: invariant. Walls that are not the ones the scans saw: nowhere to put the car.
    other = np.zeros_like(COURSE)
    other[100:102, 40:300] = True
    other[200:202, 100:200] = True
    result = _rebuild(tmp_path, _inputs(laps=0.4, map_walls=other))
    assert not result.verdict.ok
    assert not result.raceline_ok


def test_a_map_with_no_walls_is_refused_not_a_crash(tmp_path):
    result = _rebuild(tmp_path, _inputs(laps=0.2, map_walls=np.zeros_like(COURSE)))
    assert not result.verdict.ok
    assert 'no walls' in result.message


def test_less_than_a_lap_keeps_the_map_and_says_there_is_no_line(tmp_path):
    result = _rebuild(tmp_path, _inputs(laps=0.7))
    assert result.verdict.ok
    assert not result.raceline_ok
    assert result.lap_count == 0
    assert (result.out_dir / 'map.yaml').is_file()
    assert not (result.out_dir / 'raceline_profiled.csv').exists()
    assert 'lap' in result.message
    assert 'race_launch.py' not in (result.out_dir / 'commands.txt').read_text()


def test_an_existing_folder_is_never_overwritten(tmp_path, good_inputs):
    _rebuild(tmp_path, good_inputs)
    with pytest.raises(FileExistsError, match='already exists'):
        _rebuild(tmp_path, good_inputs)


def test_commands_are_written_last(tmp_path, good_inputs, monkeypatch):
    # map_course's report picks the newest folder with a commands.txt, so a rebuild that dies part-way must
    # leave none, and be reported as no map rather than half of one.
    def boom(*args, **kwargs):
        raise RuntimeError('died part-way')
    monkeypatch.setattr(rb.racing_math, 'compute_velocity_profile', boom)
    with pytest.raises(RuntimeError, match='died part-way'):
        _rebuild(tmp_path, good_inputs)
    assert not (tmp_path / 'auto' / '20261030-120000-rebuilt' / 'commands.txt').exists()


# --- the weaker odometry: review focus 4 -----------------------------------------------------------

def test_steering_odometry_is_named_as_the_weaker_source(tmp_path):
    result = _rebuild(tmp_path, _inputs(odom_source='/odom'))
    assert result.verdict.ok
    assert '/odom' in result.message
    assert 'gyro' in result.message


# --- the loader ------------------------------------------------------------------------------------

def _fake_run(tmp_path, monkeypatch, slam=None, slam_error=None, run_error=None):
    def load_slam(_):
        if slam_error is not None:
            raise slam_error
        return slam

    def load_run(_):
        if run_error is not None:
            raise run_error
        return type('Run', (), dict(scan_t=np.arange(3.0), scan=np.ones((3, 5)), angle_min=-1.0, angle_increment=0.5))
    monkeypatch.setattr(rb.slam_bag, 'load', load_slam)
    monkeypatch.setattr(rb.runlog, 'load_run', load_run)
    return tmp_path


def _slam_arrays(odom_imu_rows=20):
    rows = np.zeros((20, 4))
    rows[:, 0] = np.arange(20.0)
    return dict(map=(0.05, -1.0, -2.0, np.zeros((4, 4), np.int8)), odom=rows,
                odom_imu=rows[:odom_imu_rows] if odom_imu_rows else np.zeros((0, 4)))


def test_loader_prefers_the_gyro(tmp_path, monkeypatch):
    inputs = rb.load_inputs(_fake_run(tmp_path, monkeypatch, slam=_slam_arrays()))
    assert inputs.odom_source == '/odom/imu'
    assert (inputs.resolution, inputs.origin_x, inputs.origin_y) == (0.05, -1.0, -2.0)


def test_loader_falls_back_to_wheels_without_a_gyro(tmp_path, monkeypatch):
    inputs = rb.load_inputs(_fake_run(tmp_path, monkeypatch, slam=_slam_arrays(odom_imu_rows=0)))
    assert inputs.odom_source == '/odom'


def test_loader_says_there_is_nothing_when_slam_never_mapped(tmp_path, monkeypatch):
    run = _fake_run(tmp_path, monkeypatch, slam_error=rb.slam_bag.NoSlamMapping('no map -> odom'))
    with pytest.raises(rb.NothingToRebuild, match='SLAM'):
        rb.load_inputs(run)


def test_loader_says_there_is_nothing_with_no_map_message(tmp_path, monkeypatch):
    slam = _slam_arrays()
    slam['map'] = None
    with pytest.raises(rb.NothingToRebuild, match='/map'):
        rb.load_inputs(_fake_run(tmp_path, monkeypatch, slam=slam))


def test_loader_says_there_is_nothing_with_no_bag(tmp_path, monkeypatch):
    run = _fake_run(tmp_path, monkeypatch, slam=_slam_arrays(), slam_error=FileNotFoundError('no bag'))
    with pytest.raises(rb.NothingToRebuild, match='bag'):
        rb.load_inputs(run)


def test_loader_says_there_is_nothing_with_no_scans(tmp_path, monkeypatch):
    def empty_run(_):
        return type('Run', (), dict(scan_t=np.zeros(0), scan=np.zeros((0, 0)), angle_min=0.0, angle_increment=0.1))
    monkeypatch.setattr(rb.slam_bag, 'load', lambda _: _slam_arrays())
    monkeypatch.setattr(rb.runlog, 'load_run', empty_run)
    with pytest.raises(rb.NothingToRebuild, match='scans'):
        rb.load_inputs(tmp_path)


# --- the command line -----------------------------------------------------------------------------

def test_exit_zero_for_a_racing_folder(tmp_path, good_inputs, capsys):
    run = tmp_path / '20261030-120000'
    run.mkdir()
    code = rb.main([str(run), '--out-root', str(tmp_path / 'auto')], load=lambda _: good_inputs)
    assert code == 0
    assert 'GO' in capsys.readouterr().out


def test_exit_one_for_a_refused_map(tmp_path, capsys):
    run = tmp_path / '20261030-120000'
    run.mkdir()
    inputs = _inputs(map_walls=_doubled())
    code = rb.main([str(run), '--out-root', str(tmp_path / 'auto')], load=lambda _: inputs)
    assert code == 1
    assert 'NO-GO' in capsys.readouterr().out


def test_exit_two_for_nothing_to_work_from(tmp_path, capsys):
    run = tmp_path / '20261030-120000'
    run.mkdir()

    def nothing(_):
        raise rb.NothingToRebuild('this run has no /map message')
    code = rb.main([str(run), '--out-root', str(tmp_path / 'auto')], load=nothing)
    assert code == 2
    assert 'no /map' in capsys.readouterr().out
    assert not (tmp_path / 'auto' / '20261030-120000-rebuilt').exists()


def test_config_is_the_shipped_mapping_config():
    # oracle: cited file. Lines are checked by the same limits a live mapping run uses.
    config = rb.load_config()
    shipped = yaml.safe_load(rb.CONFIG_PATH.read_text())
    assert config == next(iter(shipped.values()))['ros__parameters']
    assert config['profile_wall_clearance'] > 0.0
