"""ID-based tracking in a fixed frame. Seconds, metres, radians throughout."""

from dataclasses import asdict, dataclass
import math

from .geometry import planar_pose


@dataclass(frozen=True)
class TrackingParams:
    velocity_gain: float = 0.5
    fresh_sec: float = 0.25
    lost_sec: float = 0.75
    max_speed_mps: float = 8.0
    position_uncertainty_m: float = 0.05
    uncertainty_growth_mps: float = 0.5
    horizon_sec: float = 1.0
    step_sec: float = 0.1

    def __post_init__(self):
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0
               for v in asdict(self).values()):
            raise ValueError('tracking parameters must be positive and finite')
        if not (self.velocity_gain <= 1 and self.fresh_sec <= self.lost_sec
                and self.step_sec <= self.horizon_sec <= 2
                and self.horizon_sec / self.step_sec <= 100):
            raise ValueError('invalid tracking parameter bounds')


@dataclass
class Track:
    vehicle_id: str
    tag_id: int
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    reprojection_error_px: float = 0.0


class Tracker:
    def __init__(self, frame_id='odom', params=None):
        if not isinstance(frame_id, str) or not frame_id or frame_id.startswith('/'):
            raise ValueError('frame_id must be a non-empty relative TF frame')
        if frame_id in ('base_link', 'camera_link') or 'optical' in frame_id:
            raise ValueError('tracking requires a fixed frame, not a sensor/car frame')
        self.frame_id = frame_id
        self.params = params or TrackingParams()
        self.tracks = {}
        self.last_stamp = None

    def clear(self):
        self.tracks.clear()
        self.last_stamp = None

    def observe(self, stamp, observations):
        """One image's observations: (vehicle_id, tag_id, T_fixed_car, pixel_error).

        Empty frames still advance the input clock. Out-of-order/repeated images
        and malformed batches raise without modifying state. Kinematic outliers
        are ignored without refreshing the track's last-seen timestamp.
        """
        if (type(stamp) not in (float, int) or not math.isfinite(stamp) or stamp < 0
                or (self.last_stamp is not None and stamp <= self.last_stamp)):
            raise ValueError('image stamp must be finite, nonnegative and increasing')
        prepared = []
        seen = set()
        for vehicle_id, tag_id, transform, error in observations:
            if (not isinstance(vehicle_id, str) or not vehicle_id or len(vehicle_id) > 64
                    or vehicle_id in seen or type(tag_id) is not int or tag_id < 0
                    or not math.isfinite(error) or error < 0):
                raise ValueError('invalid or duplicate opponent observation')
            seen.add(vehicle_id)
            prepared.append((vehicle_id, tag_id, *planar_pose(transform), float(error)))
        self.last_stamp = float(stamp)
        self.tracks = {key: t for key, t in self.tracks.items()
                       if stamp - t.stamp <= self.params.lost_sec}
        for vehicle_id, tag_id, x, y, yaw, error in prepared:
            old = self.tracks.get(vehicle_id)
            new = Track(vehicle_id, tag_id, float(stamp), x, y, yaw,
                        reprojection_error_px=error)
            if old is not None:
                dt = stamp - old.stamp
                vx, vy = (x - old.x) / dt, (y - old.y) / dt
                if math.hypot(vx, vy) > self.params.max_speed_mps:
                    continue
                gain = self.params.velocity_gain if old.hits > 1 else 1.0
                new.vx = gain * vx + (1 - gain) * old.vx
                new.vy = gain * vy + (1 - gain) * old.vy
                new.hits = old.hits + 1
            self.tracks[vehicle_id] = new

    def snapshot(self, now):
        """Prediction at publication time; last_seen retains measurement time.

        uncertainty_m is an uncalibrated heuristic growth indicator, NOT a
        probability, covariance or guaranteed error bound. No velocity or
        predicted path is claimed until two accepted observations exist.
        """
        if (type(now) not in (float, int) or not math.isfinite(now) or now < 0
                or (self.last_stamp is not None and now < self.last_stamp)):
            raise ValueError('publication time must be finite and not before the image')
        output = []
        p = self.params
        for key in sorted(self.tracks):
            track = self.tracks[key]
            age = now - track.stamp
            if age > p.lost_sec:
                continue
            valid = track.hits >= 2
            x, y = track.x + age * track.vx, track.y + age * track.vy
            offsets = [i * p.step_sec for i in range(int(p.horizon_sec / p.step_sec) + 1)]
            output.append({
                'vehicle_id': key, 'tag_id': track.tag_id, 'last_seen_sec': track.stamp,
                'age_sec': age, 'fresh': age <= p.fresh_sec, 'hits': track.hits,
                'position_m': [x, y], 'yaw_rad': track.yaw,
                'velocity_valid': valid, 'velocity_mps': [track.vx, track.vy] if valid else None,
                'uncertainty_m': p.position_uncertainty_m + p.uncertainty_growth_mps * age,
                'reprojection_error_px': track.reprojection_error_px,
                'prediction': [
                    {'dt_sec': dt, 'position_m': [x + dt * track.vx, y + dt * track.vy]}
                    for dt in offsets] if valid else [],
            })
        return {'schema_version': 1, 'stamp_sec': float(now),
                'frame_id': self.frame_id, 'source': 'apriltag_36h11', 'tracks': output}
