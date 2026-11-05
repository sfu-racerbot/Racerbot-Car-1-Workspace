import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def pytest_configure(config):
    # Registered here rather than in a [tool:pytest] section of setup.cfg: that section makes this
    # package pytest's rootdir, so the workspace conftest.py that hides the car's Jetson serial and
    # USB devices from every test never loads (src/racerbot_car/test/test_no_package_pytest_config.py).
    config.addinivalue_line(
        'markers', 'ros: Requires sourced ROS 2 Jazzy and isolated ROS domain 79 / LOCALHOST.')
