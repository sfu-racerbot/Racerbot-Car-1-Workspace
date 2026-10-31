#!/usr/bin/env python3
"""Replay recorded scans through the opponent tracker, to see how it treats a run.

    python3 tools/run_analysis/oa_replay.py data/racerbot_runs/20261030-175728 \\
        --set follow_floor=0.0

With one car on the track every "moving car" is false, so this counts false movers
and how long the follow limit would hold the car back. It uses the real
gap_follow.opponent_tracking code, open loop: the car's path stays as it was
recorded, so it shows what the tracker would say, not how the car would then drive.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

LOW_CAP = 0.4        # m/s; under this the car is effectively stopped
EPISODE = 0.3        # s; a stall the driver would notice
TICK = 0.025         # s, the Hokuyo's 40 Hz
MAX_RANGE = 10.0     # m, gap_follow.yaml max_range


@dataclass
class Replay:
    movers: int = 0
    seconds: float = 0.0
    capped: float = 0.0
    near_zero: float = 0.0
    episodes: list = field(default_factory=list)

    @property
    def stalls(self) -> int:
        return sum(1 for e in self.episodes if e >= EPISODE)


def replay(frames, params, wheelbase: float, laser_x: float, laser_y: float, car_length: float) -> Replay:
    """Run ``frames`` of (ranges, valid, angles, stamp, wheel speed, steering) through
    the tracker, in the order the node calls it."""
    from gap_follow import opponent_tracking as ot
    aware = ot.OpponentAwareness(params, wheelbase, laser_x, laser_y, car_length)
    out = Replay()
    seen, started, last = set(), None, None
    for ranges, valid, angles, stamp, speed, steering in frames:
        aware.observe(ranges, valid, angles, speed, steering,
                      math.nan if last is None else stamp - last)
        last = stamp
        out.seconds += TICK
        seen |= {t.track_id for t in aware.tracker.tracks if t.moving}
        cap = aware.follow_law_cap(False) if aware.followed_body() else math.inf
        cap, _held = aware.follow_memory.update(cap, stamp)
        cap = min(cap, aware.alongside_cap(False))
        if math.isfinite(cap):
            out.capped += TICK
        if cap < LOW_CAP:
            out.near_zero += TICK
            started = stamp if started is None else started
        elif started is not None:
            out.episodes.append(stamp - started)
            started = None
    out.movers = len(seen)
    return out


def bag_frames(run, max_range: float = MAX_RANGE):
    """Scans with LB held, as the node would have seen them."""
    from gap_follow import gap_logic
    from run_analysis.bagread import messages
    v, steer, lb = 0.0, 0.0, 0
    for topic, _stamp, msg in messages(run, ["/joy", "/odom", "/drive", "/scan"]):
        if topic == "/odom":
            v = msg.twist.twist.linear.x
        elif topic == "/joy":
            lb = int(len(msg.buttons) > 4 and msg.buttons[4])
        elif topic == "/drive":
            steer = msg.drive.steering_angle
        elif lb:
            clean, valid = gap_logic.sanitize_ranges(msg.ranges, max_range, msg.range_min)
            angles = msg.angle_min + np.arange(clean.size) * msg.angle_increment
            yield clean, valid, angles, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, v, steer


def parse_overrides(items) -> dict:
    """``name=value`` pairs for OpponentParams fields, as numbers."""
    from gap_follow.opponent_tracking import OpponentParams
    out = {}
    for item in items or []:
        name, sep, raw = item.partition("=")
        if not sep or name not in OpponentParams.__dataclass_fields__:
            raise SystemExit(f"--set expects an OpponentParams field as name=value, got {item!r}")
        out[name] = float(raw)
    return out


def main(argv=None) -> int:
    import yaml
    from gap_follow import opponent_tracking as ot
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories")
    ap.add_argument("--set", action="append", default=[], help="OpponentParams override, name=value")
    ap.add_argument("--config", default=str(Path(__file__).resolve().parents[2] / "src" / "gap_follow"
                                            / "config" / "gap_follow.yaml"), help="gap_follow YAML")
    args = ap.parse_args(argv)
    values = yaml.safe_load(Path(args.config).read_text())["gap_follow_node"]["ros__parameters"]
    params = ot.OpponentParams.from_mapping(values)
    for name, value in parse_overrides(args.set).items():
        setattr(params, name, value)
    for run in args.runs:
        r = replay(bag_frames(run), params, values["wheelbase"], values["laser_offset_x"],
                   values["laser_offset_y"], values["car_length"])
        print(f"{Path(run).name}: {r.movers} false movers in {r.seconds:.0f} s; under a follow limit "
              f"{r.capped:.1f} s; near zero {r.near_zero:.1f} s; stalls {r.stalls}, "
              f"longest {max(r.episodes, default=0.0):.2f} s")
    return 0


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(root / "tools"), str(root / "src" / "gap_follow")]
    raise SystemExit(main())
