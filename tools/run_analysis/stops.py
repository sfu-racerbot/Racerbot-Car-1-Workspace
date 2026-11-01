#!/usr/bin/env python3
"""Real stopping distances, measured by the wall ahead rather than by the wheels.

    python3 tools/run_analysis/stops.py data/racerbot_runs/20261030-175124

Finds each hard stop (speed command at or near zero from above --min-speed) and
compares its distance with the model the braking ceiling assumes: v^2 / (2 a) + v * latency.
A stop only counts as measured by the wall when the car ran straight and the wheel
and wall distances roughly agree.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

STOPPED_COMMAND = 0.3   # m/s
STOPPED_SPEED = 0.3     # m/s
STRAIGHT = 0.08         # rad of steering, most
AGREE = 0.25            # fraction wheel and wall distance may differ by
LONGEST = 3.0           # s, longest a stop may take
MIN_DURATION = 0.6      # s; shorter is a blip in the command, not a stop


@dataclass
class Stop:
    start_speed: float
    end_speed: float
    seconds: float
    wheel_m: float
    wall_m: float
    max_steer: float

    @property
    def straight(self) -> bool:
        return self.max_steer < STRAIGHT and abs(self.wheel_m - self.wall_m) < AGREE * self.wheel_m


def model_distance(speed: float, decel: float, latency: float) -> float:
    """What the braking ceiling assumes it takes to stop from ``speed``."""
    return speed ** 2 / (2.0 * decel) + speed * latency


def implied_decel(speed: float, distance: float, latency: float) -> float:
    """The deceleration a measured stop implies, or nan if latency alone used the room."""
    room = distance - speed * latency
    return speed ** 2 / (2.0 * room) if room > 0.0 else float("nan")


def find_stops(t, wheel_speed, command, forward_range, steering, min_speed: float = 2.0) -> list:
    """Each stop in time-ordered arrays, as a :class:`Stop`."""
    t, v, c = np.asarray(t), np.asarray(wheel_speed), np.asarray(command)
    r, st = np.asarray(forward_range), np.asarray(steering)
    stops = []
    i = 0
    while i < len(t) - 1:
        if not (v[i] > min_speed and c[i] <= STOPPED_COMMAND):
            i += 1
            continue
        j = i
        while (j < len(t) - 1 and c[j] <= STOPPED_COMMAND and v[j] > STOPPED_SPEED
               and t[j] - t[i] < LONGEST):
            j += 1
        if t[j] - t[i] >= MIN_DURATION:
            wheel = float(np.sum(np.maximum(v[i:j], 0.0) * np.diff(t[i:j + 1])))
            stops.append(Stop(float(v[i]), float(v[j]), float(t[j] - t[i]), wheel,
                              float(r[i] - r[j]), float(np.max(np.abs(st[i:j])))))
        i = max(j, i + 1)
    return stops


def forward_median(ranges, half_beams: int = 8) -> float:
    """Median of the finite beams straight ahead, or nan."""
    r = np.asarray(ranges, dtype=float)
    mid = len(r) // 2
    ahead = r[mid - half_beams:mid + half_beams + 1]
    ahead = ahead[np.isfinite(ahead)]
    return float(np.median(ahead)) if len(ahead) else float("nan")


def read_run(run) -> np.ndarray:
    """Columns t, wheel speed, speed command, forward range, steering, one row per scan."""
    from run_analysis.bagread import messages
    rows = []
    v = cmd = steer = 0.0
    for topic, stamp, msg in messages(run, ["/odom", "/drive", "/scan"]):
        if topic == "/odom":
            v = msg.twist.twist.linear.x
        elif topic == "/drive":
            cmd, steer = msg.drive.speed, msg.drive.steering_angle
        else:
            rows.append((stamp, v, cmd, forward_median(msg.ranges), steer))
    return np.array(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories")
    ap.add_argument("--min-speed", type=float, default=2.0, help="only stops from above this, m/s")
    ap.add_argument("--latency", type=float, default=0.25, help="assumed latency, s")
    ap.add_argument("--decel", nargs="*", type=float, default=[3.0, 4.0, 5.0], help="modelled deceleration, m/s^2")
    args = ap.parse_args(argv)
    for run in args.runs:
        a = read_run(run)
        for s in find_stops(*a.T, min_speed=args.min_speed):
            model = " ".join(f"{d:g}:{model_distance(s.start_speed, d, args.latency):.2f}" for d in args.decel)
            kind = "wall " if s.straight else "wheel"
            print(f"{Path(run).name} {s.start_speed:4.2f}->{s.end_speed:4.2f} m/s in {s.seconds:4.2f} s, "
                  f"wheel {s.wheel_m:4.2f} m, wall {s.wall_m:5.2f} m ({kind}), model {model}, "
                  f"implied {implied_decel(s.start_speed, s.wall_m if s.straight else s.wheel_m, args.latency):.1f} m/s^2")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
