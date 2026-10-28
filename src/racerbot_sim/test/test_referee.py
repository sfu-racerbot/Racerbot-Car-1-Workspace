import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_sim.referee import LapCounter, Referee  # noqa: E402


def _circle(points=400):
    angles = np.linspace(0.0, 2.0 * math.pi, points, endpoint=False)
    return np.column_stack((np.cos(angles), np.sin(angles)))


def test_lap_counter_counts_forward_and_reverse_revolutions():
    line = _circle()
    counter = LapCounter(line, line[0])
    for point in np.vstack((line[1:], line[0])):
        laps = counter.update(point)
    assert laps == pytest.approx(1.0, abs=1e-3)  # oracle: closed form, one circumference
    reverse = LapCounter(line, line[0], reverse=True)
    for point in np.vstack((line[::-1][1:], line[0])):
        reverse_laps = reverse.update(point)
    assert reverse_laps == pytest.approx(1.0, abs=1e-3)  # oracle: closed form, reverse circumference


def test_lap_counter_starts_behind_the_common_line():
    line = _circle()
    counter = LapCounter(line, line[-12:][0])
    assert counter.position_m < 0.0


def test_referee_finish_order_uses_constant_speed_closed_form():
    line = _circle(800)
    distance = float(np.hypot(*np.diff(np.vstack((line, line[:1])), axis=0).T).sum())
    referee = Referee(line, [line[0], line[-40]], laps_required=1.0, dt_s=0.01)
    for tick in range(2000):
        t = (tick + 1) * 0.01
        first = round((2.0 * t / distance) * len(line)) % len(line)
        second = round((1.0 * t / distance) * len(line) - 40) % len(line)
        referee.tick(t, [line[first], line[second]], [2.0, 1.0], [False, False], False)
        if referee.finished:
            break
    result = referee.result()
    assert result['finish_order'] == [1, 2]
    assert result['finish_time_s'][0] == pytest.approx(distance / 2.0, abs=0.1)  # oracle: closed form


def test_contact_ends_race_and_contacting_car_cannot_win():
    line = _circle()
    referee = Referee(line, [line[0], line[0]], 1.0, 0.1)
    referee.tick(0.1, [line[0], line[0]], [1.0, 1.0], [False, True], True)
    assert referee.finished
    assert referee.result()['winner'] is None
    assert referee.result()['contacts'][1] == {'wall': True, 'car': True}


def test_stall_requires_another_car_to_move():
    line = _circle()
    stopped = Referee(line, [line[0], line[0]], 10.0, 0.1)
    for tick in range(21):
        stopped.tick(tick * 0.1, [line[0], line[0]], [0.0, 1.0], [False, False], False)
    assert stopped.result()['stalls'] == [1, 0]  # oracle: spec threshold is >1 second
    both = Referee(line, [line[0], line[0]], 10.0, 0.1)
    for tick in range(21):
        both.tick(tick * 0.1, [line[0], line[0]], [0.0, 0.0], [False, False], False)
    assert both.result()['stalls'] == [0, 0]


def test_faster_trailer_closes_time_gap_by_closed_form():
    line = _circle()
    line = np.array([[float(i), 0.0] for i in range(100)])
    referee = Referee(line, [line[0], line[3]], 10.0, 1.0)
    for tick in range(11):
        referee.tick(float(tick), [[2.0 * tick, 0.0], [3.0 + 2.2 * tick, 0.0]],
                     [2.0, 2.2], [False, False], False)
    change = referee.result()['gap_changes'][0]['gap_change_s']
    expected = ((25.0 - 20.0) / 2.0) - (3.0 / 2.0)  # oracle: closed form from the fed traces
    assert change == pytest.approx(expected, abs=1e-9)  # seconds


@pytest.mark.parametrize('positions,speeds', [([[0.0], []], [[1.0], [1.0]]), ([[0.0], [1.0]], [[1.0], []])])
def test_referee_rejects_unequal_inputs(positions, speeds):
    line = _circle()
    referee = Referee(line, [line[0], line[1]], 1.0, 0.1)
    with pytest.raises(ValueError, match='match'):
        referee.tick(0.1, positions, speeds, [False, False], False)


def test_referee_rejects_nonfinite_and_zero_cars():
    line = _circle()
    with pytest.raises(ValueError, match='at least one'):
        Referee(line, [], 1.0, 0.1)
    referee = Referee(line, [line[0]], 1.0, 0.1)
    with pytest.raises(ValueError, match='finite'):
        referee.tick(0.1, [[math.nan, 0.0]], [1.0], [False], False)
