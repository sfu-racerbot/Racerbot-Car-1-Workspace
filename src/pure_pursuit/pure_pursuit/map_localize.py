"""Localize recorded scans against a finished map, and judge that map. No ROS.

A live SLAM pose is not a position on the final map. slam_toolbox builds the map from a pose graph it keeps
correcting, while the pose it publishes is whatever scan matching gave at that moment: on run 20261030-185036
the final graph nodes sit 3 cm from where the scans say the car was, and the live pose sits up to 2 m away.
A line recorded from the live pose can therefore run through the walls of the map it is raced on.

So once a run is over, the scans are matched again, each one against the finished map, with odometry only
carrying the pose from one scan to the next. That gives the car's path on the map itself, and two independent
answers about the map:

    fit           the share of beams that end on a wall. A map that does not match the world scores low.
    see-through   the share of beams that pass through a wall before they end. A laser cannot do that, so
                  this catches a doubled wall drawn in open corridor, which fit cannot: a beam ending on
                  either copy of a wall still counts as on a wall. Run 20261030-184556's doubled map has a
                  fit of 0.97; its see-through is 25-47%, against 5-10% for clean maps of the same course.

Numpy and scipy only, so it runs unsourced: tools/run_analysis/rebuild_run.py and pure_pursuit's check_start
both use it.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage

#: base_link -> laser on this car (bringup_launch.py, docs/hardware-reference.md).
LASER_X = 0.26
#: Beams shorter than this are the car's own body, and longer ones add nothing but lever arm.
MIN_RANGE_M = 0.1
MAX_RANGE_M = 12.0
#: A beam within this of a wall counts as on it: two cells of a 5 cm map.
ON_WALL_M = 0.10
#: Distances are capped here for matching, so a far-off beam cannot outvote the rest.
MATCH_CAP_M = 0.7
#: Fewer usable beams than this says too little to correct anything.
MIN_BEAMS = 30
#: How hard a match is held to the pose it started from. One beam on a wall's gradient counts about 1, so this
#: is worth a beam or two against the 100+ that see any direction a scan can observe at all.
PRIOR_WEIGHT = 2.0

#: Judgement limits, from the 2026-10-30 replays (docs/superpowers/specs/2026-10-30-slam-reliability.md):
#: clean maps of that course scored 0.975+ fit and 5-10% see-through; doubled ones 22-47%.
MIN_MEAN_FIT = 0.90
MIN_WINDOW_FIT = 0.70
MAX_SEE_THROUGH = 0.15
#: A beam may not cross a wall this long before its end: past wall thickness and range noise.
SEE_THROUGH_MARGIN_M = 0.25


def scan_points(ranges, angle_min, angle_increment, laser_x=LASER_X, stride=3,
                max_range=MAX_RANGE_M):
    """(x, y) of each usable beam's end, in base_link. Unusable beams (NaN, inf, too short, too long) drop."""
    ranges = np.asarray(ranges, dtype=float)
    if ranges.size == 0:
        return np.zeros(0), np.zeros(0)
    with np.errstate(invalid='ignore'):
        usable = np.flatnonzero(np.isfinite(ranges) & (ranges > MIN_RANGE_M) & (ranges < max_range))
    usable = usable[::stride]
    angles = angle_min + angle_increment * usable
    return laser_x + ranges[usable] * np.cos(angles), ranges[usable] * np.sin(angles)


class WallField:
    """Distance to the nearest wall anywhere on a map, smooth enough to follow downhill."""

    def __init__(self, walls, free, resolution, origin_x, origin_y):
        walls = np.asarray(walls, dtype=bool)
        free = np.asarray(free, dtype=bool)
        if walls.shape != free.shape:
            raise ValueError('walls and free must be the same shape')
        if not walls.any():
            raise ValueError('this map has no walls to localize against')
        self.walls = walls
        self.free = free
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.raw = ndimage.distance_transform_edt(~walls) * self.resolution
        self.smooth = ndimage.gaussian_filter(np.minimum(self.raw, MATCH_CAP_M), 0.7)
        self.grad_y, self.grad_x = np.gradient(self.smooth, self.resolution)

    @classmethod
    def from_occupancy(cls, occupancy_map):
        """From pure_pursuit.occupancy_map.OccupancyMap."""
        from pure_pursuit.occupancy_map import FREE, OCCUPIED
        return cls(occupancy_map.grid == OCCUPIED, occupancy_map.grid == FREE, occupancy_map.resolution,
                   occupancy_map.origin_x, occupancy_map.origin_y)

    def _sample(self, field, xs, ys):
        return ndimage.map_coordinates(
            field, [(ys - self.origin_y) / self.resolution - 0.5,
                    (xs - self.origin_x) / self.resolution - 0.5],
            order=1, mode='nearest')

    def distance(self, xs, ys):
        """Distance to the nearest wall in metres; points off the map read as far from everything."""
        cols = np.floor((xs - self.origin_x) / self.resolution).astype(int)
        rows = np.floor((ys - self.origin_y) / self.resolution).astype(int)
        inside = (cols >= 0) & (cols < self.raw.shape[1]) & (rows >= 0) & (rows < self.raw.shape[0])
        out = np.full(np.shape(xs), np.inf)
        out[inside] = self.raw[rows[inside], cols[inside]]
        return out


def _to_map(bx, by, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * bx - s * by, pose[1] + s * bx + c * by


def scan_fit(field, bx, by, pose, within=ON_WALL_M):
    """The share of beams ending within `within` of a wall, with the car at `pose`. No beams, no fit."""
    if len(bx) == 0:
        return 0.0
    return float((field.distance(*_to_map(bx, by, pose)) < within).mean())


def match_scan(field, bx, by, pose, iterations=12, start_trim=0.30, min_trim=0.12):
    """The pose near `pose` that puts these beams on walls: Gauss-Newton down the distance field.

    Each pass only listens to beams already within a shrinking distance of a wall, so a beam through a door
    to somewhere unmapped has no say. With too few beams the pose is returned untouched.

    The result is held weakly to `pose`. Hundreds of beams outvote that wherever a scan can see the car move,
    and where it cannot, along a bare corridor, nothing else says where the car is, so odometry should stand.
    """
    x, y, yaw = pose
    anchor = np.array(pose, dtype=float)
    weight = np.diag([PRIOR_WEIGHT, PRIOR_WEIGHT, PRIOR_WEIGHT])
    for iteration in range(iterations):
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * bx - s * by, y + s * bx + c * by
        d = field._sample(field.smooth, wx, wy)
        keep = d < max(start_trim * 0.7 ** iteration, min_trim)
        if keep.sum() < MIN_BEAMS:
            break
        gx, gy = field._sample(field.grad_x, wx, wy)[keep], field._sample(field.grad_y, wx, wy)[keep]
        jacobian = np.column_stack(
            [gx, gy, gx * (-s * bx[keep] - c * by[keep]) + gy * (c * bx[keep] - s * by[keep])])
        step = -np.linalg.solve(jacobian.T @ jacobian + weight,
                                jacobian.T @ d[keep] + weight @ (np.array([x, y, yaw]) - anchor))
        x, y, yaw = x + step[0], y + step[1], yaw + step[2]
        if math.hypot(step[0], step[1]) < 1e-3 and abs(step[2]) < 3e-4:
            break
    return x, y, yaw


def find_start(field, bx, by, near=None, near_radius=1.0, cell_stride=4, yaw_step=math.radians(10.0)):
    """(pose, fit) of the best place on the map for this scan, with no guess or, with `near`, around one.

    Tries every `cell_stride`-th free cell at every `yaw_step`, then refines the best sixty. `near` limits the
    search to a disc round that pose, at headings within 30 deg of it.
    """
    if len(bx) < MIN_BEAMS:
        return (0.0, 0.0, 0.0), 0.0
    rows, cols = np.nonzero(field.free)
    keep = (rows % cell_stride == 0) & (cols % cell_stride == 0)
    xs = field.origin_x + (cols[keep] + 0.5) * field.resolution
    ys = field.origin_y + (rows[keep] + 0.5) * field.resolution
    yaws = np.arange(-math.pi, math.pi, yaw_step)
    if near is not None:
        close = np.hypot(xs - near[0], ys - near[1]) <= near_radius
        xs, ys = xs[close], ys[close]
        yaws = near[2] + np.arange(-math.radians(30.0), math.radians(30.0) + 1e-9, yaw_step)
        if len(xs) == 0:
            xs, ys = np.array([near[0]]), np.array([near[1]])
    sparse_x, sparse_y = bx[::4], by[::4]
    candidates = []
    for yaw in yaws:
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = c * sparse_x - s * sparse_y, s * sparse_x + c * sparse_y
        for begin in range(0, len(xs), 2000):
            batch_x, batch_y = xs[begin:begin + 2000], ys[begin:begin + 2000]
            score = (field.distance(batch_x[:, None] + wx, batch_y[:, None] + wy) < 0.15).mean(axis=1)
            for index in np.argsort(score)[-5:]:
                candidates.append((float(score[index]), batch_x[index], batch_y[index], float(yaw)))
    candidates.sort(key=lambda candidate: candidate[0], reverse=True)
    best_pose, best_fit = (0.0, 0.0, 0.0), -1.0
    for _, x, y, yaw in candidates[:60]:
        pose = match_scan(field, bx, by, (x, y, yaw), iterations=25, start_trim=0.5)
        fit = scan_fit(field, bx, by, pose)
        if fit > best_fit:
            best_pose, best_fit = pose, fit
    return best_pose, max(best_fit, 0.0)


@dataclass
class Track:
    """One row per scan: pose (x, y, yaw) of base_link on the map, and that scan's fit."""
    times: np.ndarray
    poses: np.ndarray
    fit: np.ndarray


def track_scans(field, times, scans, odom, angle_min, angle_increment, start, laser_x=LASER_X, stride=3):
    """Follow a run across a finished map, scan by scan, from a known `start` pose.

    odom rows are (t, x, y, yaw). Only odometry's step between scans is used, in the car's body frame, so its
    drift and scale do not reach the result: the scan corrects each step. A scan with too few usable beams
    keeps the pose odometry gave and reports a fit of zero.
    """
    odom = np.asarray(odom, dtype=float)
    if odom.ndim != 2 or len(odom) == 0:
        raise ValueError('track_scans needs odometry rows (t, x, y, yaw); got none')
    yaw_track = np.unwrap(odom[:, 3])

    def at(t):
        return (np.interp(t, odom[:, 0], odom[:, 1]), np.interp(t, odom[:, 0], odom[:, 2]),
                np.interp(t, odom[:, 0], yaw_track))

    count = len(times)
    poses, fit = np.zeros((count, 3)), np.zeros(count)
    pose = tuple(float(value) for value in start)
    last = None
    for index in range(count):
        here = at(times[index])
        if last is not None:
            dx, dy = here[0] - last[0], here[1] - last[1]
            c, s = math.cos(last[2]), math.sin(last[2])
            forward, left = c * dx + s * dy, -s * dx + c * dy
            c, s = math.cos(pose[2]), math.sin(pose[2])
            pose = (pose[0] + c * forward - s * left, pose[1] + s * forward + c * left,
                    pose[2] + here[2] - last[2])
        last = here
        bx, by = scan_points(scans[index], angle_min, angle_increment, laser_x, stride)
        pose = match_scan(field, bx, by, pose)
        poses[index] = pose
        fit[index] = scan_fit(field, bx, by, pose)
    return Track(np.asarray(times, dtype=float), poses, fit)


def see_through_rate(field, poses, scans, angle_min, angle_increment, laser_x=LASER_X, stride=4,
                     margin=SEE_THROUGH_MARGIN_M):
    """For each scan, the share of its beams that cross a mapped wall well before they end."""
    rates = np.zeros(len(poses))
    usable_range = 1.5 * margin  # shorter beams have nothing to cross
    steps = np.arange(0.15, MAX_RANGE_M, field.resolution)
    for index, (x, y, yaw) in enumerate(np.asarray(poses, dtype=float).reshape(-1, 3)):
        ranges = np.asarray(scans[index], dtype=float)
        with np.errstate(invalid='ignore'):
            beams = np.flatnonzero(np.isfinite(ranges) & (ranges > usable_range)
                                   & (ranges < MAX_RANGE_M))[::stride]
        if len(beams) == 0:
            continue
        angles = yaw + angle_min + angle_increment * beams
        lx, ly = x + laser_x * math.cos(yaw), y + laser_x * math.sin(yaw)
        # one row per beam, one column per step along it
        distances = steps[None, :]
        wx = lx + np.cos(angles)[:, None] * distances
        wy = ly + np.sin(angles)[:, None] * distances
        cols = np.floor((wx - field.origin_x) / field.resolution).astype(int)
        rows = np.floor((wy - field.origin_y) / field.resolution).astype(int)
        inside = (cols >= 0) & (cols < field.walls.shape[1]) & (rows >= 0) & (rows < field.walls.shape[0])
        on_wall = np.zeros(wx.shape, dtype=bool)
        on_wall[inside] = field.walls[rows[inside], cols[inside]]
        before_end = distances < (ranges[beams] - margin)[:, None]
        rates[index] = float((on_wall & before_end).any(axis=1).mean())
    return rates


@dataclass
class Lap:
    """Indices into the pose array, the path length between them, and how far apart its ends are."""
    start: int
    end: int
    length: float
    gap: float


def find_laps(poses, min_length=5.0, max_gap=1.0, heading_tolerance=math.radians(30.0)):
    """Consecutive whole laps in a path of (x, y, yaw) with yaw unwrapped or wrapped.

    A lap ends at the first place, after turning a full circle, where the car is nearest its lap's start while
    still facing the same way. By the turning-tangent theorem one lap of a closed circuit is 2*pi of turning
    whatever its size. A full turn that ends elsewhere, or a spin on the spot, is no lap.
    """
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    if len(poses) < 3:
        return []
    xy, yaw = poses[:, :2], np.unwrap(poses[:, 2])
    travelled = np.r_[0.0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    laps = []
    start = 0
    while start < len(poses) - 2:
        turned = np.abs(yaw - yaw[start]) >= 2.0 * math.pi - heading_tolerance
        travelled_enough = travelled - travelled[start] >= min_length
        candidates = np.flatnonzero(turned & travelled_enough)
        candidates = candidates[candidates > start]
        if len(candidates) == 0:
            break
        first = candidates[0]
        gap = np.hypot(*(xy[first:] - xy[start]).T)
        # walk to the nearest point of this pass, stopping once the car is clearly leaving it
        best = first
        for index in range(first, len(poses)):
            if gap[index - first] < gap[best - first]:
                best = index
            elif gap[index - first] > gap[best - first] + 0.5:
                break
        best_gap = float(gap[best - first])
        if best_gap > max_gap:
            break
        laps.append(Lap(start, best, float(travelled[best] - travelled[start]), best_gap))
        start = best
    return laps


@dataclass
class Verdict:
    ok: bool
    reason: str
    mean_fit: float = 0.0
    worst_window_fit: float = 0.0
    see_through: float = 0.0


def judge(fit, see_through, window=20):
    """Is this map good enough to race on? Either score can refuse it, and each says why.

    The fit is averaged over `window` scans for its worst stretch, because a lost stretch hides in a mean.
    Anything not finite is a refusal: an unmeasured map is not a good one.
    """
    fit, see_through = np.asarray(fit, dtype=float), np.asarray(see_through, dtype=float)
    if fit.size == 0 or see_through.size == 0:
        return Verdict(False, 'no scans were localized on this map')
    if not (np.all(np.isfinite(fit)) and np.all(np.isfinite(see_through))):
        return Verdict(False, 'the fit or see-through scores are not finite')
    size = min(window, fit.size)
    worst = float(np.convolve(fit, np.ones(size) / size, mode='valid').min())
    mean_fit, through = float(fit.mean()), float(see_through.mean())
    problems = []
    if mean_fit < MIN_MEAN_FIT:
        problems.append(f'mean fit {mean_fit:.2f} is under {MIN_MEAN_FIT:.2f}: scans do not match this map')
    if worst < MIN_WINDOW_FIT:
        problems.append(f'worst stretch fit {worst:.2f} is under {MIN_WINDOW_FIT:.2f}: the car got lost on this map')
    if through > MAX_SEE_THROUGH:
        problems.append(f'see-through {through * 100:.0f}% is over {MAX_SEE_THROUGH * 100:.0f}%: '
                        'beams pass through mapped walls, so walls are drawn twice')
    return Verdict(not problems, '; '.join(problems) or 'clean', mean_fit, worst, through)
