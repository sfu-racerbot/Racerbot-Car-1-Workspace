"""Partial SLAM report regressions; runner: pytest tools/run_analysis/tests/."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from run_analysis import slam_report as report


def test_missing_scans_keeps_partial_report_and_cli(tmp_path, capsys):
    # oracle: runlog.py reports missing topics and empty scans as ValueError;
    # the partial report must retain independent sections and exit successfully.
    reason = 'bag: bag needs /scan and /drive, has []'
    def no_scans(path):
        raise ValueError(reason)
    def no_slam(path):
        raise ValueError('SLAM did not map this run')
    kwargs = dict(load_slam=no_slam, load_scans=no_scans, load_rosout=lambda p: [])
    run = report.load_inputs(tmp_path, **kwargs)
    lines, summary = report.build_report(run)
    assert lines[0] == 'No SLAM in this run: SLAM did not map this run'
    assert lines[lines.index('Map consistency:') + 1] == '  no scans: ' + reason
    assert 'Laps' in summary and 'Setup' in summary and 'Heading drift' in summary
    assert report.main([str(tmp_path), '--no-plots'], **kwargs) == 0
    assert capsys.readouterr().out.startswith(lines[0] + '\n')
    def broken(path):
        raise RuntimeError('broken bag')
    with pytest.raises(RuntimeError, match='broken bag'):
        report.load_inputs(tmp_path, load_slam=no_slam, load_scans=broken,
                           load_rosout=lambda p: [])
    def malformed(path):
        raise ValueError('scans have varying beam counts')
    with pytest.raises(ValueError, match='varying beam counts'):
        report.load_inputs(tmp_path, load_slam=no_slam, load_scans=malformed,
                           load_rosout=lambda p: [])


def test_scan_drops_only_count_slam_toolbox_logger(tmp_path):
    text = 'Message Filter dropping message: queue full'
    records = [dict(t=1, name='slam_toolbox', msg=text),
               dict(t=2, name='other_node', msg=text),
               dict(t=3, name='slam_toolbox', msg='other message')]
    run = report.load_inputs(tmp_path, load_slam=lambda p: {},
                             load_scans=lambda p: None, load_rosout=lambda p: records)
    # oracle: slam_launch.py names the node slam_toolbox; only its drop counts.
    assert run['drops'] == [1]
    assert report.build_report(run)[1]['Scan drops']['count'] == 1
