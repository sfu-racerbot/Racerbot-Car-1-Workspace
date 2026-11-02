import math
import cv2
import numpy as np
import pytest

from opponent_perception.detector import Camera, TagDetector, estimate_pose, tag_corners


def camera():
    return Camera(640, 480, np.array([[600, 0, 320], [0, 610, 240], [0, 0, 1]]), np.zeros(5))


def marker_image(dictionary, size):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, 7, size)
    return cv2.aruco.drawMarker(dictionary, 7, size)


@pytest.mark.parametrize('translation', [[0, 0, 2], [-0.2, 0.1, 1], [0.3, -0.15, 3]])
def test_pnp_recovers_metric_pose_from_pinhole_projection(translation):
    # oracle: closed form pinhole projection, independently computed without projectPoints.
    c, s = math.cos(0.2), math.sin(0.2)
    rotation = np.array([[c, 0, s], [0, -1, 0], [s, 0, -c]])
    # Expected proper rotation = Ry(-0.2) Rx(pi).
    points = np.array([[-.1, .1, 0], [.1, .1, 0], [.1, -.1, 0], [-.1, -.1, 0]])
    xyz = points @ rotation.T + translation
    pixels = np.column_stack([600*xyz[:, 0]/xyz[:, 2] + 320,
                              610*xyz[:, 1]/xyz[:, 2] + 240])
    pose, error = estimate_pose(pixels, .2, camera())
    assert pose[:3, 3] == pytest.approx(translation, abs=1e-6)  # m
    assert pose[:3, :3] == pytest.approx(rotation, abs=1e-6)
    assert error < 1e-5  # px, noiseless analytical projection


def test_real_detector_finds_id_and_distance_in_rendered_marker():
    # oracle: closed form Z = f * size / pixel_width (within one pixel rasterization).
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    image = np.full((480, 640), 255, dtype=np.uint8)
    image[180:300, 260:380] = marker_image(dictionary, 120)
    detected = TagDetector({7: .2}).detect(image, camera())
    assert [d.tag_id for d in detected] == [7]
    assert detected[0].camera_from_tag[2, 3] == pytest.approx(600*.2/119, abs=.02)  # m
    assert TagDetector({8: .2}).detect(image, camera()) == []


def test_blank_image_and_duplicate_id_are_not_opponents():
    # oracle: invariant: absence/ambiguous identity never creates a detection.
    image = np.full((480, 640), 255, dtype=np.uint8)
    detector = TagDetector({7: .2})
    assert detector.detect(image, camera()) == []
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = marker_image(dictionary, 100)
    image[100:200, 100:200] = marker
    image[250:350, 400:500] = marker
    assert detector.detect(image, camera()) == []


def test_degenerate_or_out_of_range_pose_is_not_returned():
    # oracle: spec README.md: zero-area tags and ranges past limit are rejected.
    assert estimate_pose(np.zeros((4, 2)), .2, camera()) is None
    pixels = np.array([[290, 210], [350, 210], [350, 270], [290, 270]], dtype=float)
    assert estimate_pose(pixels, .2, camera(), max_range_m=1) is None


@pytest.mark.parametrize('bad', [0, -1, math.nan, math.inf])
def test_tag_size_requires_finite_positive_metres(bad):
    # oracle: spec README.md: tag outer black edge must be positive and finite.
    with pytest.raises(ValueError, match='size'):
        tag_corners(bad)


@pytest.mark.parametrize('image', [
    np.zeros((2, 2), np.uint8), np.zeros((480, 640)), np.zeros((480, 640, 4), np.uint8)])
def test_image_calibration_mismatch_is_refused(image):
    # oracle: spec README.md: resolution/format cannot silently change camera geometry.
    with pytest.raises(ValueError, match='image'):
        TagDetector({7: .2}).detect(image, camera())


@pytest.mark.parametrize('matrix,distortion', [
    (np.zeros((3, 3)), []), (np.eye(3)*math.nan, []),
    (np.eye(3), [0, 0]), (np.eye(3), [math.inf]*5)])
def test_bad_intrinsics_are_refused(matrix, distortion):
    # oracle: spec README.md: finite calibrated pinhole camera with supported distortion.
    with pytest.raises(ValueError, match='calibration'):
        Camera(640, 480, matrix, distortion)
