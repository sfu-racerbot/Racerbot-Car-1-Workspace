"""Closed-form turn oracles; collected by the package pytest runner."""
import math

import pytest

from pure_pursuit.auto_map_race_node import LapRecorder


def _circle(stride=1, gap=None, frozen=False, jump=False):
    recorder = LapRecorder(0.15, 1e9, 1e9, 0.0, 0.0, 1e9,
                           max_pose_jump=0.5)
    steps = 2000
    dt = 0.01
    for i in range(steps + 1):
        yaw = 2 * math.pi * i / steps
        missing = gap is not None and gap[0] < i <= gap[1]
        map_yaw = yaw
        if frozen and gap is not None:
            map_yaw -= 2 * math.pi * max(0, min(i, gap[1]) - gap[0]) / steps
        shifted = jump and i >= 810
        recorder.update(math.cos(yaw) + 10 * shifted, math.sin(yaw),
                        map_yaw + 0.7 * shifted, i * dt,
                        odom_yaw=yaw if stride and i % stride == 0 and not missing else None)
    return recorder


@pytest.mark.parametrize('stride', [1, 2, 10, 0])
def test_exact_circle_with_intermittent_or_missing_odom(stride):
    # oracle: closed form, turning-tangent theorem: one circle = 2*pi.
    assert math.degrees(_circle(stride).turn) == pytest.approx(360, abs=0.5)  # degrees


def test_short_frozen_map_gap_is_reconciled():
    # oracle: closed form, odometry describes exactly one circle.
    assert math.degrees(_circle(gap=(800, 820), frozen=True).turn) == pytest.approx(
        360, abs=0.5)  # degrees


def test_one_second_frozen_map_gap_exceeds_safe_bound():
    # oracle: closed form, long-gap fallback retains map turning only.
    expected = 360 * (1 - 100 / 2000)
    assert math.degrees(_circle(gap=(800, 900), frozen=True).turn) == pytest.approx(
        expected, abs=0.5)  # degrees


def test_long_gap_does_not_count_map_turning_twice():
    # oracle: closed form, map yaw itself traverses exactly one circle.
    assert math.degrees(_circle(gap=(800, 1000)).turn) == pytest.approx(
        360, abs=0.5)  # degrees


def test_gap_with_map_reanchor_is_reconciled():
    recorder = _circle(gap=(800, 820), jump=True)
    # oracle: invariant, a map-frame jump is not physical turning.
    assert recorder.reanchor_count == 1
    assert math.degrees(recorder.turn) == pytest.approx(360, abs=0.5)  # degrees


def test_reanchor_with_continuous_odom_preserves_physical_turn():
    # oracle: closed form, map correction cannot change the odometry circle.
    assert math.degrees(_circle(jump=True).turn) == pytest.approx(
        360, abs=0.5)  # degrees
