"""Run with pytest tools/racerbot_sim/tests/; no ROS nodes are required."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_gap_follow_validation as validation


def test_arguments_accept_repeated_launch_options_and_known_fault():
    args = validation.parse_args(['--gap-follow-arg', 'fused_speed:=true',
                                  '--gap-follow-arg', 'imu_traction:=true',
                                  '--imu-fault', 'stale', '--fault-at', '40'])
    # oracle: command-line specification
    assert args.gap_follow_arg == ['fused_speed:=true', 'imu_traction:=true']
    assert args.imu_fault == 'stale' and args.fault_at == 40


def test_unknown_fault_is_rejected_before_run():
    # oracle: sim_imu.FAULTS does not contain this name.
    with pytest.raises(SystemExit) as exc:
        validation.parse_args(['--imu-fault', 'does_not_exist', '--fault-at', '40'])
    assert exc.value.code == 2


def test_fraction_uses_only_messages_after_settle():
    log = [(39.0, 'steering_damping_off: imu stale'),
           (40.5, 'steering_damping_off: imu stale'), (41.0, '')]
    # oracle: two eligible messages, one names the reason.
    assert validation.fallback_fractions(log, 40.0, 'stale')['steering_damping'] == 0.5


def test_empty_and_fault_after_end_have_no_fraction():
    # oracle: no eligible observations cannot establish a fraction.
    assert validation.fallback_fractions([], 40, 'stale')['steering_damping'] is None
    assert validation.fallback_fractions([(1, '')], 40, 'stale')['steering_damping'] is None


def test_speed_summary_rejects_nonfinite_samples():
    # oracle: finite arithmetic means, with nonfinite samples excluded.
    result = validation.speed_summary([(1, 2.0, 3.0), (2, float('nan'), 5.0),
                                       (3, 4.0, float('nan'))], 2)
    assert result['before']['mean_speed'] == 2.0
    assert result['after']['mean_speed'] == 4.0
    assert result['after']['mean_commanded_speed'] == 5.0
