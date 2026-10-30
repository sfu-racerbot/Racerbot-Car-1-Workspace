"""Synthetic SLAM report checks. Runner: pytest tools/run_analysis/tests/."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from run_analysis import slam_report as report


def test_heading_linear_drift_different_rates():
    # oracle: closed form: relative angular rate .01 rad/s for 20 s.
    a = np.arange(801) / 40
    b = np.arange(141) / 7
    out = report.heading_drift(np.column_stack((a, .03*a)), np.column_stack((b, .02*b)))
    assert out['final_deg'] == pytest.approx(math.degrees(.2), abs=1e-8)
    assert out['max_abs_deg'] == pytest.approx(math.degrees(.2), abs=1e-8)
    assert out['rms_rate_deg_s'] == pytest.approx(math.degrees(.01), abs=1e-8)


def test_heading_unwrap_multiple_revolutions():
    # oracle: closed form: wrapped 1 rad/s versus .9 rad/s for 20 s.
    t = np.arange(21.)
    wrap = lambda x: (x + np.pi) % (2*np.pi) - np.pi
    out = report.heading_drift(np.column_stack((t, wrap(t))), np.column_stack((t, wrap(.9*t))))
    assert out['final_deg'] == pytest.approx(math.degrees(2), abs=1e-8)
    assert out['rms_rate_deg_s'] == pytest.approx(math.degrees(.1), abs=1e-8)


def test_heading_one_common_sample():
    # oracle: spec: fewer than two common samples cannot establish drift.
    out = report.heading_drift([[0, 0], [1, 1]], [[1, 0], [2, 1]])
    assert all(math.isnan(v) for v in out.values())


def test_slam_pose_composition_and_yaw():
    # oracle: geometry: rotating (1, 0) by +90 degrees gives (0, 1).
    slam = {'map_odom': np.array([[0, 0, 0, np.pi/2], [2, 0, 0, np.pi/2]]),
            'odom_base': np.array([[1, 1, 0, .2]])}
    pose = report.slam_pose(slam)
    assert pose[0] == pytest.approx([1, 0, 1, np.pi/2 + .2], abs=1e-9)
    assert report.slam_yaw(slam) == pytest.approx(pose[:, [0, 3]], abs=1e-9)
    slam['pose'] = np.array([[1, 3, 4, .5]])
    assert report.slam_pose(slam)[0] == pytest.approx([1, 3, 4, .5], abs=1e-9)


def test_map_wall_and_laser_offset():
    # oracle: geometry: laser at x=.26 and range .76 ends in wall column 20.
    grid = np.zeros((40, 40), dtype=np.int8)
    grid[:, 20] = 100
    grid_map = (.05, 0., 0., grid)
    poses = lambda x: np.array([[0, x, .5, 0], [2, x, .5, 0]])
    def score(x, ranges):
        return report.map_consistency([1], [ranges], 0, 0, 10, poses(x), grid_map)
    assert score(0, [.76]) == 1
    assert score(.05, [.76]) == 1
    assert score(.10, [.76]) == 0
    assert score(0, [.76, np.inf]) == 1
    assert math.isnan(report.map_consistency([3], [[.76]], 0, 0, 10, poses(0), grid_map))


def test_drop_bins_ignore_null_cpu():
    # oracle: closed form: first bin mean (10+30)/2=20, second 50.
    records = [dict(t=0, node='/slam_toolbox', cpu_pct=10),
               dict(t=1, node='/slam_toolbox', cpu_pct=None),
               dict(t=2, node='/slam_toolbox', cpu_pct=30),
               dict(t=10, node='/slam_toolbox', cpu_pct=50)]
    bins = report.scan_drop_timeline([0, 2, 10], records)
    assert [b['drops'] for b in bins] == [2, 1]
    assert [b['cpu_mean'] for b in bins] == [20, 50]
    assert [b['cpu_max'] for b in bins] == [30, 50]
    assert math.isnan(report.scan_drop_timeline([0], records, node='/absent')[0]['cpu_mean'])


def test_path_length_square():
    # oracle: closed form: four sides of two metres.
    assert report.path_length([[0,0,0], [1,2,0], [2,2,2], [3,0,2], [4,0,0]]) == 8


def test_load_inputs_no_slam_and_other_error(tmp_path):
    # oracle: slam_bag.py:69-71 raises this exact diagnostic when no TF exists.
    (tmp_path / 'metadata' / 'params').mkdir(parents=True)
    (tmp_path / 'metadata' / 'params' / 'slam_toolbox.yaml').write_text('/slam_toolbox:\n  ros__parameters:\n    odom_frame: odom_imu\n')
    (tmp_path / 'events.jsonl').write_text('')
    reason = f'{tmp_path}/bag: no map -> odom or map -> odom_imu transform on /tf; SLAM did not map this run'
    def no_slam(path):
        raise report.slam_bag.NoSlamMapping(reason)
    def broken(path):
        raise RuntimeError('broken bag')
    kwargs = dict(load_scans=lambda path: None, load_rosout=lambda path: [])
    run = report.load_inputs(tmp_path, load_slam=no_slam, **kwargs)
    assert run['slam'] is None
    assert run['no_slam_reason'] == reason
    assert report.build_report(run)[0][0] == 'No SLAM in this run: ' + reason
    with pytest.raises(RuntimeError, match='broken bag'):
        report.load_inputs(tmp_path, load_slam=broken, **kwargs)


def test_lap_windows_use_closure_times():
    # oracle: spec: successive closure times partition the run from its start.
    assert report.lap_windows([{'lap': 1, 't': 10}, {'lap': 2, 't': 20}], 0, 30) == [
        (1, 0, 10), (2, 10, 20)]


def test_build_report_complete_synthetic_run():
    # oracle: closed form: a 2 m straight path at 1 m/s, zero corrections,
    # matching scan endpoints, one drop, and a single 2 m lap.
    grid = np.zeros((40, 80), dtype=np.int8)
    grid[:, 20] = 100
    pose = np.array([[0., 0., .5, 0.], [2., 2., .5, 0.]])
    slam = dict(pose=pose, odom=pose, odom_imu=pose, odom_base=pose,
                map_odom=np.array([[0., 0., 0., 0.], [2., 0., 0., 0.]]),
                map=(.05, 0., 0., grid), odom_frame='odom_imu')
    run = dict(slam=slam, params={'odom_frame': 'odom_imu'},
               scan_t=np.array([0.]), scan=np.array([[.76]]), angle_min=0.,
               angle_increment=0., range_max=10., laps=[dict(lap=1, of=1, t=2.,
               distance_m=2., closest_m=0., corrections=0)], quality=[], drops=[1.],
               proc_records=[dict(t=1., node='/slam_toolbox', cpu_pct=20.)], log_text='')
    lines, summary = report.build_report(run)
    for heading in ('Setup', 'Heading drift', 'Corrections and loop closures', 'Laps',
                    'Map consistency', 'Scan drops', 'Wheel vs scan-matched distance'):
        assert any(heading in line for line in lines)
        assert heading in summary
    assert summary['Setup']['distance_m'] == pytest.approx(2., abs=1e-9)  # m
    assert summary['Corrections and loop closures']['updates'] == 0
    assert summary['Map consistency']['overall'] == 1.
    assert summary['Scan drops']['count'] == 1
    assert summary['Wheel vs scan-matched distance']['ratio'] == pytest.approx(1., abs=1e-9)


def test_build_report_without_system_log():
    # oracle: spec: absent proc samples are explicitly identified, not inferred as zero CPU.
    lines, _ = report.build_report({'slam': None, 'no_slam_reason': 'no map'})
    assert lines[0] == 'No SLAM in this run: no map'
    assert any('no system log' in line for line in lines)


def test_main_no_slam_exit_zero(tmp_path):
    # oracle: spec: the no-SLAM report still succeeds at the CLI boundary.
    def no_slam(path):
        raise report.slam_bag.NoSlamMapping('SLAM did not map this run')
    assert report.main([str(tmp_path), '--no-plots'], load_slam=no_slam,
                       load_scans=lambda path: None, load_rosout=lambda path: []) == 0
    assert (tmp_path / 'analysis' / 'slam_report' / 'slam_report.txt').read_text().startswith(
        'No SLAM in this run: SLAM did not map this run')


def test_failed_revolutions_lap_at_ceiling_never_becomes_passed(tmp_path):
    failure = 'Lap 5 did not pass the quality check (revolutions=2.04 > 1.50 (closed over roughly two overlapping laps, not one)), but the 5-lap ceiling is reached -- accepting it anyway rather than mapping indefinitely.'
    closed = ('Closed mapping lap 5/3 detected (58.6m, 734deg of turning, '
              '2.04 revolutions, 326 samples, closest approach to the start '
              '0.08m against a 2.68m gate, 0 SLAM corrections absorbed).')
    profile = 'Generated 100-point racing profile at /tmp/profile.csv (1.0-3.0m/s).'
    log = tmp_path / 'launch.log'
    log.write_text('\n'.join((closed, failure, profile)))
    run = report.load_inputs(tmp_path, log_path=log,
                             load_slam=lambda path: {},
                             load_scans=lambda path: None,
                             load_rosout=lambda path: [])
    lines, summary = report.build_report(run)
    # oracle: spec, accepting a failed lap at the ceiling is not a pass,
    # even when the supervisor subsequently saves a racing profile.
    assert summary['Laps'][0]['verdict'] == 'failed (accepted at the ceiling)'
    lap_line, = [line for line in lines if line.startswith('  lap 5/')]
    assert 'quality failed revolutions=2.04 > 1.5' in lap_line
    assert 'accepted at the ceiling' in lap_line
    assert 'passed' not in lap_line
    assert 'deg' not in lap_line
