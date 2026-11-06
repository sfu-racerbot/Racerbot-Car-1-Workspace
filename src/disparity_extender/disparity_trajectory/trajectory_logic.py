"""Method D: pick a steering arc by sweeping the car's footprint, not by chasing one long ray.

Pure Python + numpy, no rclpy: test/test_trajectory_logic.py calls it directly.

The classic disparity extender aims at the single furthest LiDAR return. That
return can sit in a slot too narrow for the car (a box beside a wall), and the
car then drives into it -- measured on ra_compTestTrackObs in the simulator,
2026-11-06. Method D instead asks, for every arc the car can actually steer:
"if I drive this arc, how far does my whole footprint stay clear?"

For each of ``n_curvatures`` curvatures from -k_max to +k_max
(k_max = tan(max_steering) / wheelbase), the footprint -- three circles of
``body_radius_m + margin_m`` at 0, 0.18 and 0.36 m ahead of the rear axle,
covering the measured 0.535 x 0.30 m body -- is swept along the arc in
``step_m`` steps up to ``horizon_m``. An arc scores how far it stays clear of
every LiDAR point. An arc clear over the whole horizon also earns how far a
strip of half-width ``strip_half_m`` continues straight on from its end
(capped at ``continuation_cap_m``), which favours arcs that leave room to keep
going. The pick is the best score minus ``smooth_w * |k - k_previous| / k_max``.
No arc scoring ``min_score_m`` means no safe direction: the caller stops.

This is the same algorithm screened headless in tools/f1tenth_sim (0 wall
contacts in 36 runs against 17 for the furthest-ray extender) and run for 13
clean laps of indoor_tight in the full ROS 2 simulator. Simulation only so far.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

# The footprint circles' centres, metres ahead of the rear axle (base_link).
FOOTPRINT_OFFSETS_M = (0.0, 0.18, 0.36)


@dataclass(frozen=True)
class TrajectoryParams:
    wheelbase: float = 0.36
    max_steering: float = 0.33
    n_curvatures: int = 21
    step_m: float = 0.1
    horizon_m: float = 2.0
    margin_m: float = 0.05
    body_radius_m: float = 0.175
    strip_half_m: float = 0.25
    continuation_cap_m: float = 5.0
    point_range_m: float = 6.0
    smooth_w: float = 0.1
    min_score_m: float = 0.2
    lidar_x_m: float = 0.26          # LiDAR ahead of the rear axle (bringup's base_link -> laser)

    def __post_init__(self):
        if not (self.wheelbase > 0 and self.max_steering > 0 and self.step_m > 0
                and self.horizon_m >= self.step_m and self.n_curvatures >= 2):
            raise ValueError(f'invalid TrajectoryParams: {self}')


@dataclass(frozen=True)
class Target:
    steering: float      # rad, before smoothing
    score: float         # m: clear arc length (+ continuation)
    no_points: bool      # nothing within point_range_m: straight on, full range


def scan_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                range_min: float, p: TrajectoryParams) -> np.ndarray:
    """LiDAR returns as (N, 2) points in the rear-axle frame: every second valid one.

    Valid: finite, above range_min, below point_range_m. NaN and inf are dropped,
    never treated as an obstacle or as open space.
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(r) & (r > range_min) & (r < p.point_range_m)
    r, a = r[ok][::2], a[ok][::2]
    return np.c_[p.lidar_x_m + r * np.cos(a), r * np.sin(a)]


def valid_fraction(ranges: Sequence[float], range_min: float, range_max: float) -> float:
    """Fraction of beams that are real returns: finite and within [range_min, range_max]. 0.0 for none."""
    r = np.asarray(ranges, dtype=float)
    if r.size == 0:
        return 0.0
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return float(np.count_nonzero(ok)) / r.size


def trajectory_target(points: np.ndarray, previous_steering: float,
                      p: TrajectoryParams) -> Optional[Target]:
    """The arc to steer, or None when no arc stays clear for min_score_m."""
    if points.shape[0] == 0:
        return Target(steering=0.0, score=float('inf'), no_points=True)
    kmax = math.tan(p.max_steering) / p.wheelbase
    K = np.linspace(-kmax, kmax, int(p.n_curvatures))
    S = np.arange(p.step_m, p.horizon_m + 1e-9, p.step_m)
    ks, ss = K[:, None], S[None, :]
    th = ks * ss
    straight = np.abs(ks) < 1e-9
    with np.errstate(divide='ignore', invalid='ignore'):
        xs = np.where(straight, ss, np.sin(th) / np.where(straight, 1.0, ks))
        ys = np.where(straight, 0.0, (1.0 - np.cos(th)) / np.where(straight, 1.0, ks))
    rc2 = (p.body_radius_m + p.margin_m) ** 2
    blocked = np.zeros(th.shape, dtype=bool)
    for off in FOOTPRINT_OFFSETS_M:
        cx = xs + off * np.cos(th)
        cy = ys + off * np.sin(th)
        d2 = (cx[..., None] - points[:, 0]) ** 2 + (cy[..., None] - points[:, 1]) ** 2
        blocked |= (d2 < rc2).any(axis=-1)
    first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), S.size)
    score = np.where(first > 0, S[np.maximum(first - 1, 0)], 0.0)
    full = first == S.size
    score[full] = S[-1]
    w = p.strip_half_m
    for i in np.nonzero(full)[0]:
        ex, ey, et = xs[i, -1], ys[i, -1], th[i, -1]
        dx, dy = points[:, 0] - ex, points[:, 1] - ey
        t = dx * math.cos(et) + dy * math.sin(et)
        lat = -dx * math.sin(et) + dy * math.cos(et)
        m = (t > 0) & (np.abs(lat) < w)
        cont = p.continuation_cap_m if not m.any() else float(np.min(t[m] - np.sqrt(w * w - lat[m] ** 2)))
        score[i] = S[-1] + max(0.0, min(cont, p.continuation_cap_m))
    prev_k = math.tan(previous_steering) / p.wheelbase
    pick = score - p.smooth_w * np.abs(K - prev_k) / kmax
    i = int(np.argmax(pick))
    if score[i] < p.min_score_m:
        return None
    return Target(steering=math.atan(K[i] * p.wheelbase), score=float(score[i]), no_points=False)


def count_obstacles(ranges: Sequence[float], angle_min: float, angle_increment: float,
                    range_min: float, fov_half_angle: float, count_range: float,
                    cluster_gap: float, min_beams: int) -> int:
    """Separate obstacles (walls count) within count_range in the forward field of view.

    A run of consecutive in-range hits whose neighbouring ranges differ by at most
    cluster_gap is one obstacle; runs shorter than min_beams are noise. A beam
    with no close hit ends a run; a NaN beam is skipped.
    """
    count, run, prev = 0, 0, 0.0
    for i, x in enumerate(ranges):
        x = float(x)
        angle = angle_min + angle_increment * i
        inside = abs(angle) <= fov_half_angle
        if math.isnan(x) and inside:
            continue
        hit = inside and math.isfinite(x) and range_min <= x <= count_range
        if not hit:
            count += run >= min_beams
            run = 0
            continue
        if run > 0 and abs(x - prev) > cluster_gap:
            count += run >= min_beams
            run = 0
        run += 1
        prev = x
    return count + (run >= min_beams)


@dataclass(frozen=True)
class SpeedParams:
    max_speed: float = 2.5
    min_speed: float = 1.0
    max_lateral_acc: float = 2.5
    wheelbase: float = 0.36
    enable_obstacle_slowdown: bool = True
    obstacle_count_threshold: int = 3
    obstacle_slowdown: float = 1.0
    obstacle_min_speed: float = 1.0


def speed_for(steering: float, clear_m: float, obstacles: int, s: SpeedParams) -> float:
    """The disparity extender's speed law: slower in tight turns and with little room ahead.

    v = min(max_speed, sqrt(a_lat * L / tan|steer|), sqrt(a_lat * clear)), clamped
    to [min_speed, max_speed]; then, with enough obstacles in view, minus
    obstacle_slowdown but never below obstacle_min_speed (nor raised to it).
    """
    t = math.tan(abs(steering))
    v_turn = s.max_speed if t <= 1e-3 else math.sqrt(s.max_lateral_acc * s.wheelbase / t)
    v_room = math.sqrt(s.max_lateral_acc * max(0.0, clear_m)) if math.isfinite(clear_m) else s.max_speed
    v = min(max(min(s.max_speed, v_turn, v_room), s.min_speed), s.max_speed)
    if s.enable_obstacle_slowdown and obstacles >= s.obstacle_count_threshold:
        v = max(v - s.obstacle_slowdown, min(s.obstacle_min_speed, v))
    return v


def smooth(target: float, previous: float, alpha: float, limit: float) -> tuple[float, float]:
    """(command, new previous): exponential smoothing, then the steering clamp.

    The previous value is kept *before* the clamp, as the validated controller did.
    """
    raw = alpha * target + (1.0 - alpha) * previous
    return max(-limit, min(limit, raw)), raw
