#!/usr/bin/env python3
"""Replay recorded scans through the opponent tracker, to see how it treats a run.

    python3 tools/run_analysis/oa_replay.py data/racerbot_runs/20261030-175728 \\
        --set follow_floor=0.0
    python3 tools/run_analysis/oa_replay.py RUN --ego gyro           # does ego motion cause them?
    python3 tools/run_analysis/oa_replay.py RUN --truth --map MAP --plot /tmp/fragments.png

With one car on the track every "moving car" is false, so this counts false movers
and how long the follow limit would hold the car back. It uses the real
gap_follow.opponent_tracking code, open loop: the car's path stays as it was
recorded, so it shows what the tracker would say, not how the car would then drive.

--ego swaps what the tracker is told about the car's own motion: the recorded wheel
speed and steering command (what the node had), true speed from loc.py, steering from
the gyro, or both. If false movers vanish with true motion, ego motion caused them; on
2026-10-30 they did not, so the scans did.

--truth (needs loc.py's track and --map) puts each false track in the map frame by the
car's true pose and reports how far it really moved and how far it sat from a mapped
wall: a "car" that never moved and sits on a wall is a wall fragment. --plot draws every
car-sized detection on the map.
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
GYRO_BIAS = -0.0032  # rad/s, parked, car 2's VESC IMU, 2026-10-30 (car_measure.py)
EGO_MODES = ("recorded", "true-speed", "gyro", "true")


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


def replay(frames, params, wheelbase: float, laser_x: float, laser_y: float, car_length: float,
           on_frame=None) -> Replay:
    """Run ``frames`` of (ranges, valid, angles, stamp, wheel speed, steering, ...) through
    the tracker, in the order the node calls it. ``on_frame(aware, frame)`` sees each step."""
    from gap_follow import opponent_tracking as ot
    aware = ot.OpponentAwareness(params, wheelbase, laser_x, laser_y, car_length)
    out = Replay()
    seen, started, last = set(), None, None
    for frame in frames:
        ranges, valid, angles, stamp, speed, steering = frame[:6]
        aware.observe(ranges, valid, angles, speed, steering,
                      math.nan if last is None else stamp - last)
        last = stamp
        out.seconds += TICK
        seen |= {t.track_id for t in aware.tracker.tracks if t.moving}
        if on_frame is not None:
            on_frame(aware, frame)
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


def ego_motion(mode: str, wheel_speed: float, steering: float, true_speed: float, gyro: float,
               wheelbase: float) -> tuple:
    """(speed, steering) to hand the tracker. The gyro's steering is the bicycle model's
    angle for that yaw rate, at no less than 0.5 m/s so a crawl cannot blow it up."""
    if mode not in EGO_MODES:
        raise ValueError(f"ego must be one of {EGO_MODES}, got {mode!r}")
    speed = true_speed if mode in ("true-speed", "true") else wheel_speed
    if mode in ("gyro", "true"):
        steering = math.atan(wheelbase * gyro / max(abs(speed), 0.5))
    return speed, steering


def bag_frames(run, max_range: float = MAX_RANGE, ego: str = "recorded", track=None, wheelbase: float = 0.36):
    """Scans with LB held, as the node would have seen them. With ``track`` (loc.py's), each
    frame also carries the car's true pose (x, y, yaw) as a seventh item."""
    from gap_follow import gap_logic
    from run_analysis.bagread import messages
    if ego != "recorded" or track is not None:
        from run_analysis.loc import COL
        from run_analysis.sectors import true_speed
        if track is None:
            raise ValueError(f"--ego {ego} needs the run located with loc.py")
        tt = track[:, COL["t"]]
        tv = true_speed(tt, track[:, COL["x"]], track[:, COL["y"]])
    v, steer, lb, gz = 0.0, 0.0, 0, 0.0
    for topic, stamp, msg in messages(run, ["/joy", "/odom", "/drive", "/scan", "/sensors/imu/raw"]):
        if topic == "/odom":
            v = msg.twist.twist.linear.x
        elif topic == "/joy":
            lb = int(len(msg.buttons) > 4 and msg.buttons[4])
        elif topic == "/drive":
            steer = msg.drive.steering_angle
        elif topic == "/sensors/imu/raw":
            gz = math.radians(msg.angular_velocity.z) - GYRO_BIAS
        elif lb:
            clean, valid = gap_logic.sanitize_ranges(msg.ranges, max_range, msg.range_min)
            angles = msg.angle_min + np.arange(clean.size) * msg.angle_increment
            scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if track is None:
                yield clean, valid, angles, scan_stamp, v, steer
                continue
            speed, steering = ego_motion(ego, v, steer, float(np.interp(stamp, tt, tv)), gz, wheelbase)
            pose = tuple(float(np.interp(stamp, tt, track[:, COL[k]])) for k in ("x", "y"))
            pose += (float(np.interp(stamp, tt, np.unwrap(track[:, COL["yaw"]]))),)
            yield clean, valid, angles, scan_stamp, speed, steering, pose


def to_map(pose, bx: float, by: float) -> tuple:
    """A base_link point in the map frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * bx - s * by, pose[1] + s * bx + c * by


def track_truth(points, wall_distance) -> tuple:
    """(net displacement m, median distance to a mapped wall m) of a track's map points."""
    p = np.asarray(points, float)
    return float(np.hypot(*(p[-1] - p[0]))), float(np.median([wall_distance(x, y) for x, y in p]))


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
    from run_analysis.loc import DEFAULT_OUT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories")
    ap.add_argument("--set", action="append", default=[], help="OpponentParams override, name=value")
    ap.add_argument("--config", default=str(Path(__file__).resolve().parents[2] / "src" / "gap_follow"
                                            / "config" / "gap_follow.yaml"), help="gap_follow YAML")
    ap.add_argument("--ego", choices=EGO_MODES, default="recorded", help="what the tracker is told of ego motion")
    ap.add_argument("--truth", action="store_true", help="check each false track against the map (needs --map)")
    ap.add_argument("--map", help="the map the runs were located on, for --truth and --plot")
    ap.add_argument("--plot", help="PNG of every car-sized detection on the map (needs --map)")
    ap.add_argument("--loc", default=str(DEFAULT_OUT), help="where loc.py wrote (default %(default)s)")
    args = ap.parse_args(argv)
    if (args.truth or args.plot) and not args.map:
        ap.error("--truth and --plot need --map")
    values = yaml.safe_load(Path(args.config).read_text())["gap_follow_node"]["ros__parameters"]
    params = ot.OpponentParams.from_mapping(values)
    for name, value in parse_overrides(args.set).items():
        setattr(params, name, value)
    grid = None
    if args.map:
        from run_analysis.loc import Grid
        grid = Grid.from_map(args.map)
    for run in args.runs:
        needs_pose = args.ego != "recorded" or args.truth or args.plot
        track = None
        if needs_pose:
            from run_analysis.loc import load_track
            track = load_track(run, args.loc)
        chains, detections = {}, []

        def watch(aware, frame):
            if len(frame) < 7:
                return
            pose = frame[6]
            for d in aware.detections:
                detections.append(to_map(pose, d.x, d.y))
            for t in aware.tracker.tracks:
                if t.moving or t.track_id in chains:
                    chains.setdefault(t.track_id, []).append(to_map(pose, *aware.ego.to_body(t.x, t.y)))

        r = replay(bag_frames(run, ego=args.ego, track=track, wheelbase=values["wheelbase"]), params,
                   values["wheelbase"], values["laser_offset_x"], values["laser_offset_y"], values["car_length"],
                   on_frame=watch if needs_pose else None)
        print(f"{Path(run).name}: {r.movers} false movers in {r.seconds:.0f} s; under a follow limit "
              f"{r.capped:.1f} s; near zero {r.near_zero:.1f} s; stalls {r.stalls}, "
              f"longest {max(r.episodes, default=0.0):.2f} s" + ("" if args.ego == "recorded" else f" (ego {args.ego})"))
        if args.truth and chains:
            wall = lambda x, y: float(grid.sample(grid.distance, np.array([x]), np.array([y]))[0])
            rows = np.array([track_truth(points, wall) for points in chains.values() if len(points) > 1])
            if len(rows):
                print(f"   false tracks really moved: median {np.median(rows[:, 0]):.2f} m, p90 {np.percentile(rows[:, 0], 90):.2f}; "
                      f"from a mapped wall: median {np.median(rows[:, 1]):.2f} m, p90 {np.percentile(rows[:, 1], 90):.2f}")
            near = np.mean([wall(x, y) < 0.35 for x, y in detections]) if detections else math.nan
            print(f"   detections: {len(detections)}, {near:.0%} within 35 cm of a mapped wall")
        if args.plot and detections:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(10, 9))
            image = np.ones(grid.occ.shape + (3,))
            image[grid.occ] = (0.6, 0.6, 0.6)
            ax.imshow(image, origin="lower", extent=(grid.ox, grid.ox + grid.occ.shape[1] * grid.res,
                                                     grid.oy, grid.oy + grid.occ.shape[0] * grid.res))
            p = np.array(detections)
            ax.scatter(p[:, 0], p[:, 1], s=1, c="red", alpha=0.3)
            ax.set_title(f"{Path(run).name}: car-sized detections, {len(p) / max(r.seconds / TICK, 1):.2f} per scan")
            ax.set_aspect("equal")
            out = Path(args.plot)
            if len(args.runs) > 1:
                out = out.with_name(f"{out.stem}_{Path(run).name}{out.suffix}")
            fig.savefig(out, dpi=90, bbox_inches="tight")
            print(f"   wrote {out}")
    return 0


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(root / "tools"), str(root / "src" / "gap_follow")]
    raise SystemExit(main())
