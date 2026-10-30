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
           (40.5, 'steering_damping_off: imu stale'), (41.0, 'other reason')]
    # oracle: two eligible reason-bearing messages, one names the fallback.
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


def test_fault_evidence_includes_later_and_last_imu_sample():
    # oracle: a fault may take effect after the parameter request completes;
    # a stale stream must expose its final sample and zero must show later zeros.
    from types import SimpleNamespace
    rec = SimpleNamespace(intent=[], odom=[], sim=[], status=[], imu=[
        (139.9, 9.8, 0.2), (140.1, 9.8, 0.3),
        (141.1, 0.0, 0.0), (150.0, 0.0, 0.0)])
    evidence = validation.recorder_report(rec, 100.0, 40.0, 'zero')['imu_evidence']
    assert [41.1, 0.0, 0.0] in evidence
    assert [50.0, 0.0, 0.0] in evidence


def test_evidence_does_not_stop_at_first_three_post_fault_samples():
    # oracle: after a dense stream of pre-transition samples, the output must
    # still show the later zero reading and last available sample.
    from types import SimpleNamespace
    rec = SimpleNamespace(intent=[], odom=[], sim=[], status=[], imu=[
        (139.9, 9.8, 0.2), (140.1, 9.8, 0.2), (140.2, 9.8, 0.2),
        (140.3, 9.8, 0.2), (141.1, 0.0, 0.0), (150.0, 0.0, 0.0)])
    evidence = validation.recorder_report(rec, 100.0, 40.0, 'zero')['imu_evidence']
    assert [41.1, 0.0, 0.0] in evidence
    assert [50.0, 0.0, 0.0] in evidence


def test_fraction_ignores_reasonless_but_counts_other_reasons():
    # oracle: the revised brief counts non-empty reasons only after settling.
    log = [(40.1, 'early reason'), (40.5, 'steering_damping_off: imu stale'),
           (40.6, None), (40.7, ''), (40.8, 'ordinary driving')]
    assert validation.fallback_fractions(log, 40, 'stale')['steering_damping'] == 0.5


def test_reason_report_first_and_missing_examples():
    # oracle: first reason is measured from injection, not from settling;
    # missing examples include only post-settle, non-empty reasons.
    log = [(40.1, 'early reason'), (40.5, 'steering_damping_off: imu stale'),
           (40.6, None), (40.7, ''), (40.8, 'ordinary driving'),
           (41.0, 'another reason'), (41.2, 'third'), (41.4, 'fourth')]
    report = validation.reason_report(log, 40, 'stale')
    assert report['reason_messages_after_settle'] == 5
    assert report['first_reason_after_fault'] == {'time_s': 40.1, 'text': 'early reason'}
    assert report['missing_fallback']['steering_damping'] == {
        'count': 4, 'first_three': ['ordinary driving', 'another reason', 'third']}


def test_only_empty_reasons_cannot_establish_fraction():
    # oracle: no non-empty post-settle reasons means no observations.
    assert validation.fallback_fractions([(41, None), (42, '')], 40, 'stale')[
        'steering_damping'] is None


def test_injection_guard_requires_owned_unique_process_and_clean_graph():
    # oracle: review fix round 1 requires a unique descendant and no hardware.
    parents = {21: 15, 15: 10, 99: 1}
    assert validation.injection_guard([], [21], 10, parents) is None
    assert 'hardware' in validation.injection_guard(['joy'], [21], 10, parents)
    assert 'graph' in validation.injection_guard(None, [21], 10, parents)
    assert 'exactly one' in validation.injection_guard([], [], 10, parents)
    assert 'exactly one' in validation.injection_guard([], [21, 99], 10, parents)
    assert 'descendant' in validation.injection_guard([], [99], 10, parents)
    assert 'descendant' in validation.injection_guard([], [21], 10, {21: 15})
    assert 'descendant' in validation.injection_guard([], [21], 10, {21: 15, 15: 21})


def test_proc_snapshot_parses_parent_and_bridge_executable(tmp_path):
    # oracle: Linux proc stat field 4 is ppid; comm may contain spaces and ')'.
    for pid, ppid, command in [(12, 7, b'/usr/bin/python3\0/opt/gym_bridge_node\0'),
                               (13, 1, b'/usr/bin/python3\0/tmp/not_gym_bridge_node\0')]:
        proc = tmp_path / str(pid)
        proc.mkdir()
        (proc / 'stat').write_text(f'{pid} (name ) with spaces) S {ppid} 0 0\n')
        (proc / 'cmdline').write_bytes(command)
    bridges, parents = validation.process_snapshot(tmp_path)
    assert bridges == [12]
    assert parents == {12: 7, 13: 1}
