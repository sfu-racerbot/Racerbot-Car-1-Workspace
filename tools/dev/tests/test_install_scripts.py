"""Boot installer behavior and the recorded car 2 boot contract."""

import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/racerbot_car'))

from racerbot_car import hardware, identity, layers  # noqa: E402

CAR1 = 'tools/systemd/install.sh'
CAR2 = 'src/racerbot_launch/systemd/install.sh'
UDEV = 'tools/udev/install.sh'


def run_installer(script, car, *args, cars_dir=None):
    env = os.environ.copy()
    env.pop('RACERBOT_CAR', None)
    env.pop('RACERBOT_CARS_DIR', None)
    if cars_dir is not None:
        env['RACERBOT_CARS_DIR'] = str(cars_dir)
    if car is not None:
        env['RACERBOT_CAR'] = car
    return subprocess.run(['bash', str(ROOT / script), '--dry-run', *args],
                          text=True, capture_output=True, env=env)


def test_car1_systemd_install_dry_run_on_rb1_lists_car_1s_units():
    result = run_installer(CAR1, 'rb1', 'testuser', '/tmp/test-workspace')
    assert result.returncode == 0, result.stderr
    for item in ('rb1', 'testuser', '/tmp/test-workspace', 'web-dashboard.service',
                 'x11vnc.service', 'novnc.service'):
        assert item in result.stdout


def test_car1_systemd_install_refuses_on_rb2():
    result = run_installer(CAR1, 'rb2', 'testuser', '/tmp/test-workspace')
    assert result.returncode == 1
    assert 'src/racerbot_launch/systemd' in result.stderr


def test_car2_systemd_install_dry_run_on_rb2():
    result = run_installer(CAR2, 'rb2', 'testuser', '/tmp/test-workspace')
    assert result.returncode == 0, result.stderr
    for item in ('rb2', 'testuser', '/tmp/test-workspace', 'racerbot-dashboard.service'):
        assert item in result.stdout


def test_car2_systemd_install_refuses_on_rb1():
    result = run_installer(CAR2, 'rb1', 'testuser', '/tmp/test-workspace')
    assert result.returncode == 1
    assert 'tools/systemd' in result.stderr


@pytest.mark.parametrize('script', (CAR1, CAR2, UDEV))
def test_install_scripts_refuse_without_a_car(tmp_path, script):
    # Even on a Jetson, no serial is recognized in this isolated registry.
    (tmp_path / 'machines.yaml').write_text('{}\n')
    for car in ('rb1', 'rb2'):
        directory = tmp_path / car
        directory.mkdir()
        (directory / 'car.yaml').write_text(
            (ROOT / 'src/racerbot_car/cars' / car / 'car.yaml').read_text())
    args = () if script == UDEV else ('testuser', '/tmp/test-workspace')
    result = run_installer(script, None, *args, cars_dir=tmp_path)
    assert result.returncode != 0, script
    assert 'RACERBOT_CAR' in result.stderr, (script, result.stderr)


def test_udev_dry_run_installs_hokuyo_only_on_a_serial_lidar_car():
    rb1 = run_installer(UDEV, 'rb1')
    rb2 = run_installer(UDEV, 'rb2')
    assert rb1.returncode == rb2.returncode == 0, (rb1.stderr, rb2.stderr)
    assert '99-hokuyo.rules' in rb1.stdout
    assert '99-hokuyo.rules' not in rb2.stdout
    for result in (rb1, rb2):
        assert '99-vesc.rules' in result.stdout
        assert '99-wiimote-leds.rules' in result.stdout


def test_no_hard_coded_car_home_paths():
    # git grep -I ignores binary matches and searches tracked files.
    # Calibration JSON preserves its recording location; the map catalog keeps
    # import-command provenance; the bridge test records observed process lines.
    exclusions = (
        ':!docs/**', ':!data/**', ':!src/racerbot_a/**', ':!src/racerbot_b/**',
        ':!src/web_dashboards/**', ':!src/realsense-ros/**', ':!src/particle_filter/**',
        ':!src/range_libc/**', ':!src/transport_drivers/**',
        ':!src/racerbot_car/cars/*/imu/calib/*.json',
        ':!tools/f1tenth_sim/maps/catalog.yaml',
        ':!src/imu_bridge/test/test_vehicle_state_config.py',
    )
    result = subprocess.run(['git', 'grep', '-I', '-n', '/home/' + 'racerbotcar-', '--', '.',
                             *exclusions], cwd=ROOT, text=True, capture_output=True)
    assert result.returncode == 1, result.stdout + result.stderr


def test_car2_installed_units_targets_exist():
    unit = (ROOT / 'src/racerbot_launch/systemd/racerbot-dashboard.service').read_text()
    starts = [line for line in unit.splitlines() if line.startswith('ExecStart=')]
    assert len(starts) == 1
    assert 'Environment=RACERBOT_WS=@WORKSPACE@' in unit
    assert 'source "$${RACERBOT_WS}/install/setup.bash"' in starts[0]
    assert 'ros2 launch racerbot_launch dashboard_launch.py' in starts[0]
    assert (ROOT / 'src/racerbot_launch/launch/dashboard_launch.py').is_file()
    assert (ROOT / 'src/web_dashboards/car/systemd/foxglove-bridge.sh').is_file()


def test_dashboard_launch_resolves_rb2_from_car_2s_serial(tmp_path, monkeypatch):
    machines = yaml.safe_load((ROOT / 'src/racerbot_car/cars/machines.yaml').read_text())
    serials = [serial for serial, car in machines.items() if car == 'rb2']
    assert len(serials) == 1
    serial = tmp_path / 'serial-number'
    serial.write_text(serials[0])
    monkeypatch.setattr(identity, 'SERIAL_PATH', str(serial))
    monkeypatch.setattr(hardware, 'usb_ids_present', lambda: set())
    monkeypatch.delenv('RACERBOT_CAR', raising=False)
    monkeypatch.delenv('RACERBOT_CARS_DIR', raising=False)
    assert layers.launch_car_id() == 'rb2'
