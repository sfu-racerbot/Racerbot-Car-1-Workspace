import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'src', 'racerbot_sim'))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'f1tenth_sim'))
from racerbot_sim.referee import Referee  # noqa: E402
import eval_gap_multi as egm  # noqa: E402


DT = 0.1
RADIUS = 20.0
POINTS = 20000
ANGLES = np.linspace(0.0, 2.0 * math.pi, POINTS, endpoint=False)
LINE = np.column_stack((RADIUS * np.cos(ANGLES), RADIUS * np.sin(ANGLES)))


def _xy(progress):
    angle = progress / RADIUS
    return [RADIUS * math.cos(angle), RADIUS * math.sin(angle)]


def _score(traces, speeds, times):
    referee = Referee(LINE, [_xy(trace[0]) for trace in traces], 99.0, DT)
    referee.traces = [list(trace) for trace in traces]
    referee.speeds = [list(speed) for speed in speeds]
    referee.times = list(times)
    referee._sim_time = (len(traces[0]) - 1) * DT
    result = referee.result()
    fast = dict(cars=[dict(progress_m=t, speed_mps=s, finish_time_s=time,
                            laps=0.0, wall_contact=False, car_contact=False)
                      for t, s, time in zip(traces, speeds, times)], dt_s=DT)
    return result, egm.head_to_head(fast)


def test_referee_matches_eval_equal_pace_faster_trailer_and_stall():
    scenarios = [
        ([[0.2 * i for i in range(30)], [-3.0 + 0.2 * i for i in range(30)]],
         [[2.0] * 30, [2.0] * 30], [None, None]),
        ([[0.2 * i for i in range(30)], [-3.0 + 0.22 * i for i in range(30)]],
         [[2.0] * 30, [2.2] * 30], [15.0, 13.64]),
        ([[0.2 * i for i in range(30)], [-3.0] * 30],
         [[2.0] * 30, [0.0] * 30], [None, None]),
    ]
    for traces, speeds, times in scenarios:
        actual, expected = _score(traces, speeds, times)
        assert actual['overtakes'] == expected['overtakes']
        assert actual['stalls'] == expected['stalls']
        assert len(actual['gap_changes']) == len(expected['gap_changes'])
        for actual_pair, expected_pair in zip(actual['gap_changes'], expected['gap_changes']):
            assert actual_pair['pair'] == expected_pair['pair']
            assert actual_pair['leader'] == expected_pair['leader']
            assert actual_pair['trailer'] == expected_pair['trailer']
            for field in ('start_gap_s', 'end_gap_s', 'gap_change_s'):
                assert actual_pair[field] == pytest.approx(expected_pair[field], abs=1e-9)
        assert actual['time_gained_s'] == pytest.approx(expected['time_gained_s'], abs=1e-9)
        assert [number - 1 for number in actual['finish_order']] == expected['finish_order']
