"""A SLAM correction is map-frame motion that odometry did not see.

Runner: colcon test --packages-select pure_pursuit, or pytest src/pure_pursuit/test/ with ROS sourced
(this file imports auto_map_race_node, which imports rclpy).

Oracles are closed forms, named per test: a car on a known circle, sampled the way the supervisor
really samples it, 40 Hz ticks reading a transform that only updates at 50 Hz.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from pure_pursuit.auto_map_race_node import LapRecorder, angle_difference

TICK_S = 1.0 / 40.0  # control_rate_hz in auto_map_race.yaml
ODOM_S = 1.0 / 50.0  # the rate of vesc_to_odom's odom -> base_link


def _recorder(**overrides):
    settings = dict(
        spacing=0.15, min_distance=5.0, departure_distance=2.0, closure_distance=1.5,
        closure_heading_rad=math.radians(30.0), min_duration_sec=4.0,
        min_turn_rad=math.radians(300.0), max_pose_jump=0.12)
    settings.update(overrides)
    return LapRecorder(**settings)


def _circle_pose(s, radius=6.0):
    """Pose at arc length s on a circle centred at (0, radius), starting at the origin."""
    angle = s / radius
    return radius * math.sin(angle), radius - radius * math.cos(angle), angle


def _held(t):
    """The latest 50 Hz update at time t: what a TF lookup at a 40 Hz tick returns."""
    return math.floor(t / ODOM_S + 1e-9) * ODOM_S


def _drive(recorder, speed, seconds, radius=6.0, correction=None):
    """Feed a recorder a car driving the circle. Odometry and the map frame are the same frame.

    correction(t) -> (dx, dy) is added to the map-frame position only: a SLAM correction.
    Returns the number of ticks on which the lap closed.
    """
    closes = 0
    for index in range(int(seconds / TICK_S)):
        now = index * TICK_S
        x, y, yaw = _circle_pose(speed * _held(now), radius)
        dx, dy = correction(now) if correction else (0.0, 0.0)
        if recorder.update(x + dx, y + dy, yaw, now, odom_yaw=yaw, odom_xy=(x, y)):
            closes += 1
            recorder.reset()
    return closes


def test_a_tick_spanning_two_odom_updates_is_not_a_correction():
    # oracle: closed form. At 3.5 m/s, two 50 Hz updates in one tick move the car 0.14 m, past the
    # 0.12 m limit, and all of that step is on odometry: nothing corrected, so nothing is absorbed.
    recorder = _recorder(max_closure_distance=4.0)
    _drive(recorder, speed=3.5, seconds=5.0)
    assert recorder.reanchor_count == 0


def test_the_beat_really_does_exceed_the_limit_at_this_speed():
    # oracle: closed form. Guards the test above against passing because the fixture is too slow.
    steps = []
    previous = None
    for index in range(200):
        pose = _circle_pose(3.5 * _held(index * TICK_S))
        if previous is not None:
            steps.append(math.hypot(pose[0] - previous[0], pose[1] - previous[1]))
        previous = pose
    assert max(steps) > 0.12
    assert max(steps) == pytest.approx(3.5 * 2 * ODOM_S, abs=1e-3)  # 1 mm, chord of a 6 m circle


def test_the_stored_start_stays_put_while_driving_at_speed():
    # oracle: invariant. With no correction at all, the start of the lap is where the car started.
    recorder = _recorder()
    _drive(recorder, speed=3.5, seconds=5.0)
    assert recorder.start[0] == pytest.approx(0.0, abs=1e-9)  # m
    assert recorder.start[1] == pytest.approx(0.0, abs=1e-9)  # m


def test_a_lap_at_speed_closes_once_per_revolution():
    # oracle: closed form. 2*pi*6 m at 3.5 m/s is 10.77 s a lap, so 33 s holds exactly three laps,
    # each closing as the car comes back through its own start.
    assert _drive(_recorder(), speed=3.5, seconds=33.0) == 3


def test_a_real_correction_while_moving_is_absorbed_once():
    # oracle: closed form. The map jumps 0.5 m sideways once, under a moving car.
    recorder = _recorder()
    _drive(recorder, speed=3.5, seconds=3.0, correction=lambda t: (0.0, 0.5) if t >= 1.5 else (0.0, 0.0))
    assert recorder.reanchor_count == 1


def test_a_correction_moves_the_stored_start_by_exactly_the_correction():
    # oracle: closed form. A pure 0.5 m translation of the map frame moves the start 0.5 m and
    # nothing else, with no share of the car's own motion that tick.
    recorder = _recorder()
    _drive(recorder, speed=3.5, seconds=3.0, correction=lambda t: (0.0, 0.5) if t >= 1.5 else (0.0, 0.0))
    assert recorder.start[0] == pytest.approx(0.0, abs=1e-6)  # m
    assert recorder.start[1] == pytest.approx(0.5, abs=1e-6)  # m


def test_a_rotation_correction_turns_the_start_about_the_car():
    # oracle: closed form. Car at (5, 0) heading east. The map rotates +90 deg about the car, so a
    # start 5 m behind the car ends up 5 m south of it, at (5, -5).
    recorder = _recorder(max_pose_jump=0.12)
    recorder.update(0.0, 0.0, 0.0, 0.0, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    for index in range(1, 11):
        x = 0.5 * index
        recorder.update(x, 0.0, 0.0, index * TICK_S, odom_yaw=0.0, odom_xy=(x, 0.0))
    recorder.update(5.0, 0.0, math.pi / 2, 11 * TICK_S, odom_yaw=0.0, odom_xy=(5.0, 0.0))
    assert recorder.reanchor_count == 1
    assert recorder.start[0] == pytest.approx(5.0, abs=1e-9)  # m
    assert recorder.start[1] == pytest.approx(-5.0, abs=1e-9)  # m
    assert recorder.start_yaw == pytest.approx(math.pi / 2, abs=1e-9)  # rad


@pytest.mark.parametrize('frame_yaw', [0.0, math.pi / 2, -2.5])
def test_odometry_in_a_rotated_frame_is_still_not_a_correction(frame_yaw):
    # oracle: invariant. Odometry's own frame is arbitrary; only motion expressed in the car's body
    # counts. The map and odom frames differ by frame_yaw and a 7 m offset, and nothing is corrected.
    recorder = _recorder()
    c, s = math.cos(frame_yaw), math.sin(frame_yaw)
    for index in range(200):
        now = index * TICK_S
        x, y, yaw = _circle_pose(3.5 * _held(now))
        recorder.update(x, y, yaw, now,
                        odom_yaw=math.atan2(math.sin(yaw + frame_yaw), math.cos(yaw + frame_yaw)),
                        odom_xy=(7.0 + c * x - s * y, -7.0 + s * x + c * y))
    assert recorder.reanchor_count == 0


@pytest.mark.parametrize('offset, expected', [
    pytest.param(0.11875, 0, id='inside'),
    pytest.param(0.12, 0, id='exactly-at-the-limit'),
    pytest.param(0.12 + 1 / 1024, 1, id='just-past'),
])
def test_the_limit_applies_to_the_unexplained_part_of_the_step(offset, expected):
    # oracle: closed form. The car moves 1 m on odometry, the map frame moves 1 m plus an offset to
    # the side. Only the offset is unexplained. Nothing in this step is motion the odom did not see
    # except that offset, so a 1 m step can still be no correction at all.
    recorder = _recorder()
    recorder.update(0.0, 0.0, 0.0, 0.0, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    recorder.update(1.0, offset, 0.0, TICK_S, odom_yaw=0.0, odom_xy=(1.0, 0.0))
    assert recorder.reanchor_count == expected


@pytest.mark.parametrize('bad', [
    pytest.param((float('nan'), 0.0), id='nan'),
    pytest.param((float('inf'), 0.0), id='inf'),
    pytest.param(None, id='missing'),
])
def test_without_usable_odometry_the_raw_step_decides(bad):
    # oracle: invariant. A step we cannot explain is treated as a correction, the conservative side
    # for a recorded line: the old behaviour, and no worse than it.
    recorder = _recorder()
    recorder.update(0.0, 0.0, 0.0, 0.0, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    recorder.update(0.05, 0.0, 0.0, TICK_S, odom_yaw=0.0, odom_xy=bad)
    assert recorder.reanchor_count == 0  # 0.05 m is inside the raw limit
    recorder.update(0.25, 0.0, 0.0, 2 * TICK_S, odom_yaw=0.0, odom_xy=bad)
    assert recorder.reanchor_count == 1  # 0.20 m is outside it


def test_odometry_returning_after_a_gap_does_not_look_like_a_correction():
    # oracle: invariant. During the gap the raw step decided. The first tick back has no previous
    # odometry to compare with, so it must not compare against the stale pre-gap sample.
    recorder = _recorder()
    recorder.update(0.0, 0.0, 0.0, 0.0, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    recorder.update(0.05, 0.0, 0.0, TICK_S, odom_yaw=None, odom_xy=None)
    recorder.update(0.10, 0.0, 0.0, 2 * TICK_S, odom_yaw=0.0, odom_xy=(50.0, 0.0))
    assert recorder.reanchor_count == 0


@pytest.mark.parametrize('turn, expected', [
    pytest.param(0.09375, 0, id='inside'),
    pytest.param(0.1 + 1 / 1024, 1, id='just-past'),
])
def test_unexplained_turning_alone_is_a_correction(turn, expected):
    # oracle: closed form. The car stands still and the map frame rotates about it: no position
    # change at the car, and yet a start 5 m away moves by nearly 5 m * turn.
    recorder = _recorder()
    recorder.update(0.0, 0.0, 0.0, 0.0, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    recorder.update(0.0, 0.0, turn, TICK_S, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    assert recorder.reanchor_count == expected


def test_real_turning_seen_by_odometry_is_not_a_correction():
    # oracle: invariant. The same 0.5 rad, on odometry as well this time.
    recorder = _recorder()
    recorder.update(0.0, 0.0, 0.0, 0.0, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    recorder.update(0.0, 0.0, 0.5, TICK_S, odom_yaw=0.5, odom_xy=(0.0, 0.0))
    assert recorder.reanchor_count == 0


def test_without_odom_xy_the_old_call_still_works():
    # oracle: closed form. Five positional arguments, as every existing caller passes: a 0.8 m step.
    recorder = _recorder()
    recorder.update(0.0, 0.0, 0.0, 0.0)
    recorder.update(0.8, 0.0, 0.0, TICK_S)
    assert recorder.reanchor_count == 1


def test_max_pose_jump_zero_still_turns_the_check_off():
    # oracle: invariant, the documented meaning of 0 (auto_map_race.yaml).
    recorder = _recorder(max_pose_jump=0.0)
    recorder.update(0.0, 0.0, 0.0, 0.0, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    recorder.update(0.8, 0.0, 0.0, TICK_S, odom_yaw=0.0, odom_xy=(0.0, 0.0))
    assert recorder.reanchor_count == 0


def _replay_recorded_run(recorder):
    """The poses auto_map_race_node saw on run 20261030-185036, from LB held to the Ctrl+C."""
    data = np.load(Path(__file__).parent / 'data' / 'lap_poses_20261030-185036.npz')
    laps = []
    for row in data['rows'][data['rows'][:, 0] >= float(data['lb_held_s'])]:
        t, x, y, yaw, odom_x, odom_y, odom_yaw = (float(value) for value in row)
        if recorder.update(x, y, yaw, t, odom_yaw=angle_difference(odom_yaw, 0.0),
                           odom_xy=(odom_x, odom_y)):
            laps.append((recorder.distance, recorder.reanchor_count))
            recorder.reset()
    return laps


def _shipped_recorder():
    return _recorder(closure_widen_after_revolutions=1.25, max_closure_distance=4.0,
                     close_at_closest_approach=True, closest_approach_margin=0.05,
                     min_duration_fraction=0.5)


def _replay_both_frames(columns):
    """Laps from run 20261030-180355's replay through map_course, with its held-LB poses, on one odometry.

    columns are where that odometry's x, y, yaw sit: 4 for steering (`odom`), 7 for gyro (`odom_imu`).
    """
    recorder = _shipped_recorder()
    data = np.load(Path(__file__).parent / 'data' / 'lap_poses_20261030-180355-replay.npz')
    laps = []
    for row in data['rows']:
        if recorder.update(row[1], row[2], row[3], row[0], odom_yaw=angle_difference(row[columns + 2], 0.0),
                           odom_xy=(row[columns], row[columns + 1])):
            laps.append(recorder.distance)
            recorder.reset()
    return laps


def test_laps_close_when_counted_in_the_odometry_slam_maps_on():
    # oracle: recorded measurement. In that replay SLAM's own pose passes the start 4 times, 0.06-0.41 m
    # away and 47 m of path apart, and a disparity_extender run steers harder than gap_follow does.
    laps = _replay_both_frames(7)
    assert len(laps) == 4
    for distance in laps:
        assert distance == pytest.approx(47.0, abs=1.0)  # m, 2% of a lap


def test_steering_odometry_is_why_that_run_never_closed():
    # The reason map_course counts in odom_imu. In hard turns the steering model's yaw differs from the
    # gyro's by more than the 0.1 rad a correction is, and each false correction swings the start point.
    # If this one starts passing, the frame stopped mattering and imu_lap_turns' default can be revisited.
    assert _replay_both_frames(4) == []


def test_the_real_run_that_never_closed_closes_its_laps():
    # oracle: recorded measurement, run 20261030-185036. Its log says 5.0 revolutions, 196 m and no
    # lap closed. The course is 50.4 m round, measured by scan-matching every lap against the map
    # (data/rb2/racerbot_auto/20261030-185036-fixed), independently of this recorder.
    laps = _replay_recorded_run(_shipped_recorder())
    assert len(laps) >= 4
    for distance, _ in laps:
        assert distance == pytest.approx(50.4, abs=2.0)  # m, 4% of a lap


def test_the_real_run_is_not_a_stream_of_corrections():
    # oracle: recorded measurement. The live node absorbed 305 corrections in that run, 70 a lap,
    # which fails mapping_lap_max_reanchors (20, auto_map_race.yaml) on every lap it would have
    # closed. Real SLAM updates over this limit are a few a lap.
    laps = _replay_recorded_run(_shipped_recorder())
    assert max(corrections for _, corrections in laps) <= 20
