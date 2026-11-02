"""Test-only: hide the Jetson serial in a subprocess started by a test.

The workspace conftest.py puts this directory on PYTHONPATH (and sets RACERBOT_TEST_NO_SERIAL)
for the duration of each test only, so a script a test runs with `python3 <script>` resolves
its car the way it would on a development machine. Never on PYTHONPATH outside a test run.
"""
import os
import sys
from pathlib import Path

_path = os.environ.get('RACERBOT_TEST_NO_SERIAL')
if _path:
    try:
        from racerbot_car import identity as _identity
    except ImportError:
        sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'src' / 'racerbot_car'))
        try:
            from racerbot_car import identity as _identity
        except ImportError:
            _identity = None
    if _identity is not None:
        _identity.SERIAL_PATH = _path
