"""Extract the topics the lap-gap study needs from each run's bag into tmp/bag/<run>.npz.

Run under `rb` (needs rosbag2_py). Writes nothing into data/.
"""
import sys
from pathlib import Path

import numpy as np

from _paths import CACHE, LINE, LOC  # noqa: E402,F401  (also puts tools/ on sys.path)
from run_analysis.bagread import messages  # noqa: E402

OUT = CACHE / "bag"
OUT.mkdir(parents=True, exist_ok=True)


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


for run in sys.argv[1:]:
    name = Path(run).name
    dest = OUT / f"{name}.npz"
    if dest.exists():
        print("cached", name)
        continue
    core, mcmd, drive, odom, scan, joy, ack = [], [], [], [], [], [], []
    topics = ["/sensors/core", "/commands/motor/speed", "/drive", "/odom", "/scan", "/joy", "/ackermann_cmd"]
    for topic, t, msg in messages(run, topics):
        if topic == "/sensors/core":
            s = msg.state
            core.append((t, stamp(msg.header), s.speed, s.voltage_input, s.current_motor, s.duty_cycle, s.current_input))
        elif topic == "/commands/motor/speed":
            mcmd.append((t, msg.data))
        elif topic == "/drive":
            drive.append((t, stamp(msg.header), msg.drive.speed, msg.drive.steering_angle))
        elif topic == "/ackermann_cmd":
            ack.append((t, stamp(msg.header), msg.drive.speed, msg.drive.steering_angle))
        elif topic == "/odom":
            odom.append((t, stamp(msg.header), msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        elif topic == "/scan":
            r = np.asarray(msg.ranges)
            scan.append((t, stamp(msg.header), len(r), msg.scan_time, msg.time_increment,
                         msg.angle_min, msg.angle_max, np.isfinite(r).sum()))
        elif topic == "/joy":
            joy.append((t, int(len(msg.buttons) > 4 and msg.buttons[4])))
    np.savez(dest, core=np.array(core), mcmd=np.array(mcmd), drive=np.array(drive), odom=np.array(odom),
             scan=np.array(scan), joy=np.array(joy), ack=np.array(ack))
    print("wrote", name, len(core), len(drive), len(scan))
