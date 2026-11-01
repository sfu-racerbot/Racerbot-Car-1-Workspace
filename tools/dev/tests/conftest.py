"""Allow source car-path imports with no ROS installation sourced."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/racerbot_car'))
