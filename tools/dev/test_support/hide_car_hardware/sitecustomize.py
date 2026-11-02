"""Test-only: hide the car's Jetson serial and USB devices in a subprocess started by a test.

The workspace conftest.py puts this directory on PYTHONPATH (and sets RACERBOT_TEST_NO_SERIAL
and RACERBOT_TEST_NO_USB) for the duration of each test only, so a script or launch a test runs
with `python3 <script>` or `ros2 launch` resolves its car the way it would on a development
machine. Never on PYTHONPATH outside a test run.
"""
import os
import sys
from pathlib import Path

_serial = os.environ.get('RACERBOT_TEST_NO_SERIAL')
_usb = os.environ.get('RACERBOT_TEST_NO_USB')
if _serial or _usb:
    try:
        from racerbot_car import hardware as _hardware, identity as _identity
    except ImportError:
        sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'src' / 'racerbot_car'))
        try:
            from racerbot_car import hardware as _hardware, identity as _identity
        except ImportError:
            _hardware = _identity = None
    if _identity is not None:
        if _serial:
            _identity.SERIAL_PATH = _serial
        if _usb:
            _hardware.USB_DEVICES_ROOT = Path(_usb)
