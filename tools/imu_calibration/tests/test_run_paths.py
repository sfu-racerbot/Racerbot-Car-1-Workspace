"""Task 7: raw IMU runs are stored and found under the selected car."""
import pytest

from racer_imu.logger import RunLogger, latest_run


@pytest.mark.parametrize('car', ['rb1', 'rb2'])
@pytest.mark.parametrize('explicit', [False, True])
def test_imu_recording_root(tmp_path, monkeypatch, car, explicit):
    monkeypatch.setenv('RACERBOT_CAR', car)
    monkeypatch.setenv('RACERBOT_WS', str(tmp_path))
    root = tmp_path / 'override' if explicit else None
    if explicit:
        monkeypatch.delenv('RACERBOT_CAR')
    expected = root if explicit else tmp_path / 'data' / car / 'imu_calibration'
    log = RunLogger('test', root=root)
    try:
        assert log.dir.parent == expected
        assert (log.dir / 'meta.json').is_file()
        assert latest_run(root=root, contains='test') == log.dir
    finally:
        log.close()
