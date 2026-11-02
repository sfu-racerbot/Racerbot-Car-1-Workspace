"""Workspace-wide test isolation: every test runs the same on a car as on a development machine.

racerbot_car.identity reads the Jetson's serial from /proc/device-tree/serial-number, and on a
car that serial decides the car before RACERBOT_CAR does (fail-closed: a contradicting
RACERBOT_CAR is refused). That is right for launches and wrong for tests: a test that selects
the other car, or no car, was refused on the car it ran on (car 1, 2026-11-01: about 100 tests).

So, for every test, the serial path points at a file that does not exist, exactly as on a
development machine:
- in this process, `racerbot_car.identity.SERIAL_PATH` is patched (every caller passes it
  explicitly: identity.current_car_id, the doctor's CLI, sim_fidelity.calibration);
- in subprocesses a test starts (entry points run with `python3 <script>`), a test-only
  sitecustomize (tools/dev/test_support/hide_jetson_serial/) does the same at interpreter start.
Tests of identity itself set their own serial path on top (monkeypatch), which still wins.
Production code has no switch to ignore the serial; this lives only in the test run.
"""
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
NO_SERIAL = '/nonexistent/racerbot-test-run-has-no-jetson-serial'
SHIM = ROOT / 'tools' / 'dev' / 'test_support' / 'hide_jetson_serial'


def _identity_module():
    try:
        from racerbot_car import identity
    except ImportError:
        sys.path.insert(0, str(ROOT / 'src' / 'racerbot_car'))
        try:
            from racerbot_car import identity
        except ImportError:
            return None
    return identity


@pytest.fixture(autouse=True)
def _no_real_jetson_serial(monkeypatch):
    identity = _identity_module()
    if identity is not None:
        monkeypatch.setattr(identity, 'SERIAL_PATH', NO_SERIAL)
    existing = os.environ.get('PYTHONPATH', '')
    monkeypatch.setenv('PYTHONPATH', str(SHIM) + (os.pathsep + existing if existing else ''))
    monkeypatch.setenv('RACERBOT_TEST_NO_SERIAL', NO_SERIAL)
