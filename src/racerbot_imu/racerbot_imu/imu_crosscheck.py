"""The two-IMU cross-check. No rclpy import.

Yaw rate is the same at every point of a rigid car, so the VESC's gyro and
the front-bumper gyro must agree wherever each is mounted. With only two
sensors there is no majority to say which one is wrong, so a sustained
disagreement is a fault and the car stops (plan, A3 and P0.3).
"""
import math
from collections import deque


def aligned_window_mean(samples, t_truth_end: float, window: float, delay: float,
                        min_samples: int = 5):
    """Mean gyro.z over the truth-time window [t_truth_end - window, t_truth_end].

    A stream with `delay` stamps each value `delay` after it was true, so its
    samples for that window carry stamps [t - window + delay, t + delay].
    Callers pick t_truth_end = now - max(delays) so every stream has arrived.
    None with fewer than min_samples in the window."""
    lo, hi = t_truth_end - window + delay, t_truth_end + delay
    vals = [w for t, w in samples if lo <= t <= hi]
    if len(vals) < min_samples:
        return None
    return sum(vals) / len(vals)


class GyroAgreement:
    """Tracks whether two yaw rates agree, with a hold time before a fault.

    Tolerance is base_tol + rel_tol * |w_primary|: the relative term allows
    for a small scale error between the two gyros at high yaw rates.
    """

    def __init__(self, base_tol: float, rel_tol: float, hold_sec: float):
        self.base_tol = float(base_tol)
        self.rel_tol = float(rel_tol)
        self.hold_sec = float(hold_sec)
        self._since = None
        self.disagreement = 0.0

    def tolerance(self, w_primary: float) -> float:
        return self.base_tol + self.rel_tol * abs(w_primary)

    def update(self, t: float, w_primary, w_secondary):
        """(agree, disagreement). A missing value counts as disagreeing."""
        if w_primary is None or w_secondary is None \
                or not (math.isfinite(w_primary) and math.isfinite(w_secondary)):
            self.disagreement = math.inf
            bad = True
        else:
            self.disagreement = abs(w_primary - w_secondary)
            bad = self.disagreement > self.tolerance(w_primary)
        if not bad:
            self._since = None
            return True, self.disagreement
        if self._since is None:
            self._since = t
        return (t - self._since) < self.hold_sec - 1e-9, self.disagreement


class SampleBuffer:
    """The last `keep_sec` of (t, gyro_z) samples."""

    def __init__(self, keep_sec: float = 1.0):
        self.keep_sec = keep_sec
        self.samples = deque()

    def add(self, t: float, w: float) -> None:
        self.samples.append((t, w))
        while self.samples and self.samples[0][0] < t - self.keep_sec:
            self.samples.popleft()
