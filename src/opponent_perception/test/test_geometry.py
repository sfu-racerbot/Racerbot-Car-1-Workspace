import math
import numpy as np
import pytest

from opponent_perception.geometry import (
    opponent_pose, planar_pose, rigid_transform, transform_from_pose)


def test_rotation_translation_and_mount_offset():
    # oracle: closed form. Camera yaw +90deg sends camera +X to world +Y.
    camera = transform_from_pose([10, -2, 0], [0, 0, math.sqrt(0.5), math.sqrt(0.5)])
    detection = np.eye(4)
    detection[:3, 3] = [3, 1, 0]
    mount = np.eye(4)
    mount[0, 3] = 0.5
    pose = opponent_pose(camera, detection, mount)
    assert planar_pose(pose) == pytest.approx([9, 0.5, math.pi/2], abs=1e-9)


@pytest.mark.parametrize('angle', [-1.2, 0, 1.2])
def test_moving_camera_cannot_move_a_stationary_opponent(angle):
    # oracle: invariant. T_world_camera * T_camera_tag * inv(T_car_tag) = T_world_car.
    camera = transform_from_pose([angle, -2*angle, 0.3],
                                 [0, 0, math.sin(angle/2), math.cos(angle/2)])
    car = transform_from_pose([4, -3, 0], [0, 0, 0, 1])
    mount = transform_from_pose([0.2, 0.1, 0.3], [0, 0, 1, 0])
    observed = np.linalg.inv(camera) @ car @ mount
    assert opponent_pose(camera, observed, mount) == pytest.approx(car, abs=1e-9)


@pytest.mark.parametrize('bad', [
    [], np.eye(3), np.zeros((4, 4)), np.full((4, 4), math.nan),
    np.diag([-1, 1, 1, 1]), np.diag([1.1, 1, 1, 1])])
def test_invalid_transforms_are_refused(bad):
    # oracle: spec README.md, proper finite SE(3) contract.
    with pytest.raises(ValueError, match='transform'):
        rigid_transform(bad)


@pytest.mark.parametrize('position,q', [
    ([math.inf, 0, 0], [0, 0, 0, 1]), ([0, 0, 0], [0, 0, 0, 0]),
    ([0, 0, 0], [0, 0, 0, 2]), ([], [0, 0, 0, 1])])
def test_invalid_poses_are_refused(position, q):
    # oracle: spec README.md, finite position and unit quaternion required.
    with pytest.raises(ValueError, match='pose|quaternion'):
        transform_from_pose(position, q)
