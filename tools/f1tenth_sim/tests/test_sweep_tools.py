"""sweep2.py, summ2.py and the friction option. Pure Python except the two runs that
prove friction reaches the plant, which drive the real simulator for a moment.
Run by: python3 -m pytest tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ -q
"""
import json
import math
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SIM), str(SIM.parent)]
from sim_fidelity.bootstrap import bootstrap  # noqa: E402

bootstrap()
import eval_gap  # noqa: E402
import summ2  # noqa: E402
import sweep2  # noqa: E402
from run_analysis import simgrip  # noqa: E402
from sim_fidelity import CALIBRATION, with_friction  # noqa: E402


def test_with_friction_changes_only_the_friction():
    changed = with_friction(0.42)
    assert changed.surface_friction == 0.42
    assert changed.wheelbase == CALIBRATION.wheelbase
    # oracle: closed form, the friction circle's radius is mu * g.
    assert changed.friction_accel_limit == pytest.approx(0.42 * 9.81, abs=1e-9)
    assert CALIBRATION.surface_friction != 0.42       # the shared value is not touched


def test_with_friction_none_is_the_calibration_itself():
    assert with_friction(None) is CALIBRATION


@pytest.mark.parametrize('bad', [0.0, -0.5, math.nan, math.inf])
def test_with_friction_refuses_a_floor_that_cannot_exist(bad):
    with pytest.raises(ValueError, match='friction'):
        with_friction(bad)


def test_friction_reaches_the_plant():
    # oracle: closed form, read back from the plant the run used, not from the argument.
    params = eval_gap.parse_set(['enable_opponent_awareness=false'])
    for friction in (0.42, 0.55):
        result = eval_gap.run_track('indoor_oval', params, 1, 0.1, False, with_friction(friction))
        assert result['grip_limit_mps2'] == pytest.approx(friction * 9.81, abs=2e-3)   # m/s^2, rounded


def test_parse_candidate():
    assert sweep2.parse_candidate('base:') == ('base', [])
    assert sweep2.parse_candidate('a:x=1,y=2') == ('a', ['x=1', 'y=2'])


@pytest.mark.parametrize('bad', ['nocolon', ':x=1', ''])
def test_parse_candidate_refuses_a_malformed_one(bad):
    with pytest.raises(SystemExit, match='candidate'):
        sweep2.parse_candidate(bad)


def run(passed=True, seconds=20.0, laps=1.0, contact=0, clear=0.1, stops=8):
    return dict(passed=passed, sim_time_s=seconds, laps=laps, contact_steps=contact,
                min_body_clearance_m=clear, stop_steps=stops)


def test_summarize_averages_laps_of_passes_only():
    row = sweep2.summarize('c', 0.42, 't', [run(seconds=20.0), run(seconds=22.0), run(False, seconds=99.0, contact=5, clear=-0.1)])
    # oracle: closed form, (20 + 22) / 2; the failed run is left out of the lap.
    assert row['lap'] == pytest.approx(21.0, abs=1e-9)
    assert (row['passed'], row['total'], row['contact']) == (2, 3, 1)
    assert row['minclear'] == -0.1
    assert row['stops'] == 24


def test_summarize_with_no_passes_has_no_lap():
    assert sweep2.summarize('c', None, 't', [run(False)])['lap'] is None


def test_summarize_scales_a_lap_that_was_not_one_lap():
    assert sweep2.summarize('c', None, 't', [run(seconds=40.0, laps=2.0)])['lap'] == pytest.approx(20.0, abs=1e-9)


def test_summ2_totals_each_candidate_and_friction(tmp_path):
    rows = [dict(cand='a', mu=0.42, track='x', passed=3, total=4, lap=20.0, contact=1, minclear=0.0, stops=1),
            dict(cand='a', mu=0.42, track='y', passed=4, total=4, lap=10.0, contact=0, minclear=0.1, stops=1),
            dict(cand='a', mu=0.55, track='x', passed=4, total=4, lap=19.0, contact=0, minclear=0.1, stops=1)]
    path = tmp_path / 'r.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in rows) + '\n\n')
    assert summ2.totals(summ2.read_rows(path)) == [('a', 0.42, 7, 8, 1), ('a', 0.55, 4, 4, 0)]


def test_summ2_prints_a_row_with_no_lap():
    text = summ2.format_row(dict(cand='a', mu=None, track='x', passed=0, total=4, lap=None, contact=2, minclear=None, stops=0))
    assert '0/4' in text and 'contact  2' in text and '-' in text


def test_yaw_rate_survives_the_wrap_at_180():
    # oracle: closed form, 1 rad/s through the +/-180 deg wrap.
    import numpy as np
    t = np.arange(0.0, 2.0, 0.025)
    yaw = np.degrees((np.pi - 1.0 + t + np.pi) % (2.0 * np.pi) - np.pi)   # wraps at t = 1 s
    assert yaw.max() - yaw.min() > 300.0                                   # it really wraps
    assert simgrip.yaw_rate(t, yaw)[10:-10] == pytest.approx(1.0, abs=1e-6)  # rad/s
