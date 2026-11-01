"""The IMU CLI imports racer_imu.logger before parsing --help."""
import os
from pathlib import Path
import subprocess


def test_imu_help_without_sourced_workspace(tmp_path):
    root = Path(__file__).resolve().parents[3]
    result = subprocess.run(
        ['env', '-i', f'HOME={os.environ["HOME"]}', 'PATH=/usr/bin:/bin',
         'python3', str(root / 'tools/imu_calibration/tools/calibrate_imu.py'), '--help'],
        cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert '--gyro-seconds' in result.stdout
