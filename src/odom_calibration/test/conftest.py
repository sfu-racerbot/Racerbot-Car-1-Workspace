"""Make the workspace's pure-Python car helpers available without ROS."""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'racerbot_car'))
