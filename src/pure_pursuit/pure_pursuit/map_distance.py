"""ROS-free chassis-to-map clearance geometry."""
import math
from pathlib import Path
import numpy as np

class MapDistance:
    """Continuous distance (m) from a world point to the nearest occupied cell's edge.

    Same occupancy rule as gym's track loader (pixel <= 128 occupied, > 128
    free, so a SLAM map's unknown grey counts as free). Sampling gym's own
    per-cell distance field snaps every clearance to multiples of the map
    resolution (0.05, 0.071, 0.10 m at 5 cm): far too coarse for a 0.10 m
    threshold. Here the nearest occupied cell comes from the EDT's index map
    and the distance is measured from the query point to that cell's square,
    so it is continuous; <= 0 inside an occupied cell.
    """

    def __init__(self, image, resolution, origin_xy):
        from scipy import ndimage
        img = np.asarray(image)
        self.occ = img <= 128
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.h, self.w = self.occ.shape
        _, self.idx = ndimage.distance_transform_edt(~self.occ, return_indices=True)

    @classmethod
    def from_yaml(cls, map_yaml):
        import yaml
        from PIL import Image
        meta = yaml.safe_load(Path(map_yaml).read_text())
        img = np.array(Image.open(Path(map_yaml).parent / meta['image']).convert('L'))
        if meta.get('negate'):
            img = 255 - img
        return cls(img, meta['resolution'], meta['origin'][:2])

    def body_clearance(self, corners, search_m=0.5):
        """Exact clearance (m) between an oriented rectangle and every occupied cell.

        ``corners`` is the (4, 2) chassis outline, consecutive, as
        FidelityPlant._chassis_outline gives it. Each occupied cell is a closed
        square; the result is the minimum vertex-to-shape distance both ways,
        which is the exact distance between two disjoint convex polygons and
        0.0 when they overlap (a vertex of either then lies inside the other --
        exact here, since a cell is narrower than the body and cannot cross it
        without one). Cells beyond ``search_m`` of the body's
        bounding box are not examined, so results saturate at ``search_m``.
        """
        corners = np.asarray(corners, dtype=float)
        lo = corners.min(axis=0) - search_m
        hi = corners.max(axis=0) + search_m
        c0 = max(0, int(math.floor((lo[0] - self.ox) / self.res)))
        c1 = min(self.w - 1, int(math.floor((hi[0] - self.ox) / self.res)))
        r0 = max(0, self.h - 1 - int(math.floor((hi[1] - self.oy) / self.res)))
        r1 = min(self.h - 1, self.h - 1 - int(math.floor((lo[1] - self.oy) / self.res)))
        if c0 > c1 or r0 > r1:
            return 0.0                                   # body entirely off the map
        rr, cc = np.nonzero(self.occ[r0:r1 + 1, c0:c1 + 1])
        if rr.size == 0:
            return float(search_m)
        x0 = self.ox + (cc + c0) * self.res
        y0 = self.oy + (self.h - 1 - (rr + r0)) * self.res
        x1, y1 = x0 + self.res, y0 + self.res
        # body frame: origin at corners[0], axes along its two edges
        u = corners[1] - corners[0]; length = float(np.hypot(*u)); u = u / length
        v = corners[3] - corners[0]; width = float(np.hypot(*v)); v = v / width
        cv = np.stack([np.c_[x0, y0], np.c_[x1, y0], np.c_[x1, y1], np.c_[x0, y1]], axis=1) - corners[0]
        bx, by = cv @ u, cv @ v                          # (n, 4) cell vertices in the body frame
        d_cell = np.hypot(np.maximum.reduce([-bx, np.zeros_like(bx), bx - length]),
                          np.maximum.reduce([-by, np.zeros_like(by), by - width]))
        px, py = corners[:, 0][None, :], corners[:, 1][None, :]      # body vertices vs squares
        dx = np.maximum.reduce([x0[:, None] - px, np.zeros((x0.size, 4)), px - x1[:, None]])
        dy = np.maximum.reduce([y0[:, None] - py, np.zeros((y0.size, 4)), py - y1[:, None]])
        return float(min(d_cell.min(), np.hypot(dx, dy).min()))

    def __call__(self, x, y):
        c = int(math.floor((x - self.ox) / self.res))
        r = self.h - 1 - int(math.floor((y - self.oy) / self.res))
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0                                   # off the map: treat as blocked
        if self.occ[r, c]:
            return 0.0
        rr, cc = self.idx[0][r, c], self.idx[1][r, c]
        cx0 = self.ox + cc * self.res                    # that occupied cell's square
        cy0 = self.oy + (self.h - 1 - rr) * self.res
        dx = max(cx0 - x, 0.0, x - (cx0 + self.res))
        dy = max(cy0 - y, 0.0, y - (cy0 + self.res))
        return math.hypot(dx, dy)

