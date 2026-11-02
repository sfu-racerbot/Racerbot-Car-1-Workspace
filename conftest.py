"""Workspace-wide test isolation: every test runs the same on a car as on a development machine.

racerbot_car decides which car it is running on from two facts about the host, and refuses to
start on a contradiction (fail-closed):
- identity reads the Jetson's serial from /proc/device-tree/serial-number; on a car that serial
  decides the car before RACERBOT_CAR does, and a contradicting RACERBOT_CAR is refused;
- layers.launch_car_id then scans /sys/bus/usb/devices, and a device distinctive to another car
  (car 2's RealSense, car 1's USB Hokuyo) refuses the launch.
That is right for launches and wrong for tests: a test that selects the other car, or no car,
was refused on the car it ran on (2026-11-01: about 100 tests on car 1 and 628 on car 2 on the
serial, then 309 on car 2 on its RealSense).

So, for every test, both point at paths that do not exist, exactly as on a development machine:
- in this process, `racerbot_car.identity.SERIAL_PATH` and `racerbot_car.hardware.USB_DEVICES_ROOT`
  are patched (every caller reads them at call time: identity.current_car_id, the doctor's CLI,
  sim_fidelity.calibration pass SERIAL_PATH explicitly; usb_ids_present() defaults to the root);
- in subprocesses a test starts (entry points run with `python3 <script>`, `ros2 launch`), a
  test-only sitecustomize (tools/dev/test_support/hide_car_hardware/) does the same at interpreter
  start. A test that sets PYTHONPATH for a subprocess must prepend to the inherited value.
Tests of identity or the USB check set their own path or devices on top (monkeypatch), which
still wins. Production code has no switch to ignore either; this lives only in the test run.
pytest.ini (beside this file) makes this directory the rootdir of every suite, so this file is
loaded for packages with their own setup.py too.
"""
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
NO_SERIAL = '/nonexistent/racerbot-test-run-has-no-jetson-serial'
NO_USB = '/nonexistent/racerbot-test-run-has-no-usb-devices'
SHIM = ROOT / 'tools' / 'dev' / 'test_support' / 'hide_car_hardware'


def _racerbot_car_modules():
    try:
        from racerbot_car import hardware, identity
    except ImportError:
        sys.path.insert(0, str(ROOT / 'src' / 'racerbot_car'))
        try:
            from racerbot_car import hardware, identity
        except ImportError:
            return None, None
    return identity, hardware


@pytest.fixture(autouse=True)
def _no_real_car_hardware(monkeypatch):
    identity, hardware = _racerbot_car_modules()
    if identity is not None:
        monkeypatch.setattr(identity, 'SERIAL_PATH', NO_SERIAL)
        monkeypatch.setattr(hardware, 'USB_DEVICES_ROOT', Path(NO_USB))
    existing = os.environ.get('PYTHONPATH', '')
    monkeypatch.setenv('PYTHONPATH', str(SHIM) + (os.pathsep + existing if existing else ''))
    monkeypatch.setenv('RACERBOT_TEST_NO_SERIAL', NO_SERIAL)
    monkeypatch.setenv('RACERBOT_TEST_NO_USB', NO_USB)
