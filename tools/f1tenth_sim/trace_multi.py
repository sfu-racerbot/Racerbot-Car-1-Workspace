#!/usr/bin/env python3
"""Two or more gap_follow cars in the simulator, one seed, with a per-tick log of one car:
why it slowed, what it was following, and what the tracker thought of the others.

    python3 tools/f1tenth_sim/trace_multi.py --track course_2026_10_30 --speeds 2.5,3.5 --watch 1 \\
        --out /tmp/car1.csv
    python3 tools/f1tenth_sim/trace_multi.py --track indoor_wide --speeds 3.0,3.0 --set opponent_follow_floor=0.5

Prints the run's result, then the first stall of the watched car (under 0.05 m/s for
1 s) with the ticks around it. --out writes every tick of the watched car. It wraps
eval_gap_multi.run_multi and reads the same GapController the scored runs use.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_gap  # noqa: E402
import eval_gap_multi  # noqa: E402
from gap_mirror import GapController  # noqa: E402
from sim_fidelity import FLOORS, with_floor, with_friction  # noqa: E402

COLUMNS = ("t", "speed", "cmd_speed", "steering", "stop_reason", "binding", "follow_cap", "mode",
           "min_clear", "fwd_clear", "detections", "tracks")


def first_stall(speeds, dt: float, below: float = 0.05, seconds: float = 1.0):
    """Index where the first run of ``seconds`` under ``below`` starts, or None."""
    need, run = int(round(seconds / dt)), 0
    for i, v in enumerate(speeds):
        run = run + 1 if v < below else 0
        if run == need:
            return i - need + 1
    return None


def describe_tracks(controller, limit: int = 3) -> str:
    """The tracker's moving or confirmed tracks: id, moving, speed, seconds stopped."""
    if controller.opponents is None:
        return ""
    rows = []
    for t in controller.opponents.tracker.tracks:
        if t.moving or getattr(t, "confirmed_car", False):
            rows.append(f"{t.track_id}{'M' if t.moving else 'h'}:{t.speed:.2f}/{getattr(t, 'stopped_sec', 0.0):.1f}s")
    return " ".join(rows[:limit])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="course_2026_10_30")
    ap.add_argument("--speeds", default="2.5,3.5", help="max_speed per car, car 0 first")
    ap.add_argument("--watch", type=int, default=1, help="which car to log (default %(default)s)")
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--spacing", type=float, default=2.0, help="m between start positions")
    ap.add_argument("--laps", type=float, default=3.0)
    ap.add_argument("--set", action="append", default=[], help="a parameter for every car, name=value")
    ap.add_argument("--friction", type=float)
    ap.add_argument("--floor", choices=sorted(FLOORS))
    ap.add_argument("--out", help="CSV of every tick of the watched car")
    args = ap.parse_args(argv)
    speeds = args.speeds.split(",")
    params = eval_gap_multi.per_car_params(eval_gap.parse_set(args.set), {}, len(speeds), args.speeds)
    order, log = {}, []
    original = GapController.command

    def command(self, scan, speed, step, **kwargs):
        result = original(self, scan, speed, step, **kwargs)
        if order.setdefault(id(self), len(order)) == args.watch:
            t = self.telemetry
            follow = t.get("follow_cap")
            log.append(dict(t=round(step * eval_gap.CONTROL_DT, 3), speed=round(float(speed), 3),
                            cmd_speed=round(float(result[1]), 3), steering=round(float(result[0]), 4),
                            stop_reason=self.stop_reason, binding=self.binding_cap,
                            follow_cap=round(follow, 3) if isinstance(follow, float) and math.isfinite(follow) else "",
                            mode=self.opponent_mode or "", min_clear=t.get("min_clear", ""), fwd_clear=t.get("fwd_clear", ""),
                            detections=len(self.opponents.detections) if self.opponents else 0,
                            tracks=describe_tracks(self)))
        return result

    GapController.command = command
    try:
        result = eval_gap_multi.run_multi(eval_gap.track_path(args.track), params, args.seed, args.duration,
                                          args.spacing, args.laps,
                                          calibration=with_friction(args.friction, with_floor(args.floor)))
    finally:
        GapController.command = original
    print({k: result.get(k) for k in ("passed", "contact", "min_center_distance_m", "sim_time_s")})
    for i, car in enumerate(result["cars"]):
        print(f"car {i}: laps {car.get('laps')}, finish {car.get('finish_time_s')}, mean speed {car.get('mean_speed')}")
    start = first_stall([row["speed"] for row in log], eval_gap.CONTROL_DT)
    if start is None:
        print(f"car {args.watch} never stalled")
    else:
        print(f"car {args.watch} stalled at {log[start]['t']:.2f} s:")
        for row in log[max(0, start - 60):start + 80:10]:
            print("   " + " ".join(f"{k}={row[k]}" for k in COLUMNS if row[k] not in ("", None)))
    if args.out:
        with open(args.out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(COLUMNS))
            writer.writeheader()
            writer.writerows(log)
        print(f"wrote {len(log)} ticks to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
