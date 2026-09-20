"""Unit tests for RGB-D filtering helpers."""

from geometry_msgs.msg import Point
import numpy as np

from jaka_zu3_vision.target_detector import TargetDetector
from jaka_zu3_vision.target_motion import TargetMotion


def test_depth_sampling_uses_object_median_and_millimetres():
    """Invalid background and one outlier must not move the depth estimate."""
    depth = np.zeros((8, 8), dtype=np.uint16)
    depth[2:7, 2:7] = 500
    depth[4, 4] = 4000
    mask = np.zeros((8, 8), dtype=np.uint8)
    contour = np.array([[[1, 1]], [[1, 7]], [[7, 7]], [[7, 1]]])

    sampled = TargetDetector.sample_depth(depth, mask, contour, min_samples=4)

    assert sampled == 0.5


def test_depth_sampling_supports_different_stream_resolutions():
    """An aligned lower-resolution depth stream is sampled safely."""
    depth = np.full((4, 4), 0.7, dtype=np.float32)
    mask = np.zeros((8, 8), dtype=np.uint8)
    contour = np.array([[[0, 0]], [[0, 7]], [[7, 7]], [[7, 0]]])

    sampled = TargetDetector.sample_depth(depth, mask, contour, min_samples=4)

    assert np.isclose(sampled, 0.7)


def test_median_point_rejects_coordinate_outlier_in_result():
    """Coordinate medians provide a stable grasp centre."""
    points = []
    for x, y, z in ((0.30, 0.10, 0.05),
                    (0.31, 0.11, 0.05),
                    (2.00, 0.10, 0.90)):
        point = Point()
        point.x, point.y, point.z = x, y, z
        points.append(point)

    result = TargetMotion.median_point(points)

    assert np.isclose(result.x, 0.31)
    assert np.isclose(result.y, 0.10)
    assert np.isclose(result.z, 0.05)


def test_point_distance_is_euclidean():
    """Stability checks use a three-dimensional Euclidean distance."""
    first = Point(x=0.0, y=0.0, z=0.0)
    second = Point(x=1.0, y=2.0, z=2.0)

    assert TargetMotion.point_distance(first, second) == 3.0
