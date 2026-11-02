"""Rigid transforms: T_parent_child maps child coordinates into parent."""

import math
import numpy as np


def rigid_transform(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('transform must be a finite 4x4 matrix')
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(np.linalg.det(rotation), 1, abs_tol=1e-6)):
        raise ValueError('transform must be rigid with a proper rotation')
    return matrix.copy()


def transform_from_pose(position, quaternion):
    """ROS quaternion order x,y,z,w. Reject zero/non-unit quaternions."""
    p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all():
        raise ValueError('pose must have finite position[3] and quaternion[4]')
    if not math.isclose(float(q @ q), 1.0, abs_tol=1e-5):
        raise ValueError('quaternion must be unit length')
    x, y, z, w = q
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    result[:3, 3] = p
    return rigid_transform(result)


def opponent_pose(tracking_from_camera, camera_from_tag, opponent_from_tag):
    """Remove the tag mount before expressing the car in the tracking frame."""
    return (rigid_transform(tracking_from_camera)
            @ rigid_transform(camera_from_tag)
            @ np.linalg.inv(rigid_transform(opponent_from_tag)))


def planar_pose(matrix):
    transform = rigid_transform(matrix)
    return (float(transform[0, 3]), float(transform[1, 3]),
            math.atan2(transform[1, 0], transform[0, 0]))
