"""AprilTag 36h11 detection and metric pose, using OpenCV (no ROS).

Tag frame: x right, y up, z out of the printed front face. Size is the
outer black square edge, excluding white margin. OpenCV camera optical
frame: x right, y down, z forward. No confidence probability is invented.
"""

from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray

    def __post_init__(self):
        k = np.asarray(self.matrix, dtype=float)
        d = np.asarray(self.distortion, dtype=float).reshape(-1)
        if (type(self.width) is not int or type(self.height) is not int
                or self.width <= 0 or self.height <= 0
                or k.shape != (3, 3) or not np.isfinite(k).all()
                or k[0, 0] <= 0 or k[1, 1] <= 0
                or not np.allclose(k[2], [0, 0, 1], atol=1e-8, rtol=0)
                or d.size not in (0, 4, 5, 8, 12, 14) or not np.isfinite(d).all()):
            raise ValueError('invalid camera calibration')
        object.__setattr__(self, 'matrix', k.copy())
        object.__setattr__(self, 'distortion', d.copy())


@dataclass(frozen=True)
class Detection:
    tag_id: int
    camera_from_tag: np.ndarray
    reprojection_error_px: float


def tag_corners(size):
    if not math.isfinite(size) or size <= 0:
        raise ValueError('tag size must be positive and finite')
    h = size / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=float)


def estimate_pose(corners, size, camera, max_error_px=2.0, max_range_m=8.0):
    """Planar PnP with a reprojection gate; planar heading can be ambiguous."""
    image_points = np.asarray(corners, dtype=float)
    if image_points.shape != (4, 2) or not np.isfinite(image_points).all():
        raise ValueError('corners must be finite 4x2 pixels')
    if any(not math.isfinite(x) or x <= 0 for x in (max_error_px, max_range_m)):
        raise ValueError('pose limits must be positive and finite')
    points = tag_corners(size)
    if abs(cv2.contourArea(image_points.astype(np.float32))) < 16:
        return None
    ok, rotation, translation = cv2.solvePnP(
        points, image_points, camera.matrix, camera.distortion,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(np.r_[rotation.ravel(), translation.ravel()]).all():
        return None
    rmat = cv2.Rodrigues(rotation)[0]
    if np.any((points @ rmat.T + translation.reshape(3))[:, 2] <= 0):
        return None
    if np.linalg.norm(translation) > max_range_m:
        return None
    projected = cv2.projectPoints(points, rotation, translation,
                                  camera.matrix, camera.distortion)[0].reshape(4, 2)
    error = float(np.sqrt(np.mean(np.sum((projected - image_points)**2, axis=1))))
    if error > max_error_px:
        return None
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rmat, translation.reshape(3)
    return result, error


class TagDetector:
    def __init__(self, sizes, max_error_px=2.0, max_range_m=8.0):
        if not sizes or any(type(i) is not int or not 0 <= i < 587 for i in sizes):
            raise ValueError('configure tag IDs in 36h11 range 0..586')
        for size in sizes.values():
            tag_corners(size)
        self.sizes = dict(sizes)
        self.max_error_px, self.max_range_m = max_error_px, max_range_m
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        # 4.6 exposes the class name too, but its direct constructor can
        # create an invalid binding and crash when setting a property.
        # Prefer the legacy factory whenever it exists.
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # Ubuntu 24.04's OpenCV 4.6 has the older contrib API.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        else:
            self.detector = None

    def detect(self, image, camera):
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
                or image.shape not in ((camera.height, camera.width),
                                       (camera.height, camera.width, 3))):
            raise ValueError('image must be uint8 mono or BGR at calibrated resolution')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        if self.detector is None:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)
        results = []
        if ids is None:
            return results
        # Duplicate copies of one ID in a frame are ambiguous; ignore both.
        values = ids.reshape(-1).tolist()
        for pixels, tag_id in zip(corners, values):
            if tag_id not in self.sizes or values.count(tag_id) != 1:
                continue
            pose = estimate_pose(pixels.reshape(4, 2), self.sizes[tag_id], camera,
                                 self.max_error_px, self.max_range_m)
            if pose is not None:
                results.append(Detection(tag_id, pose[0], pose[1]))
        return results
