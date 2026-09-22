"""Unit tests for RGB-D filtering helpers."""

from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Point
import numpy as np
import pytest
from sensor_msgs.msg import JointState

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


@pytest.mark.parametrize('left, stamp_sec, expected', [
    (None, 10, False),       # Right-only feedback must not imply left closure.
    (0.0, 10, False),        # A stationary left finger must prevent lift.
    (0.006, 10, False),      # Partial closure passed the old loose tolerances.
    (0.010, 9, False),       # Old feedback must not authorize a new lift.
    (0.010, 10, True),       # Both fresh, measured positions permit lift.
])
def test_close_requires_both_measured_fingers(left, stamp_sec, expected):
    """A missing, stalled or stale finger must keep the arm at grasp height."""
    parameters = {
        'finger_sync_tolerance': 0.001,
        'finger_goal_tolerance': 0.002,
        'finger_state_timeout': 0.5,
        'use_sim_attachment': True,
    }
    node = SimpleNamespace(
        finger_positions={},
        finger_state_times={},
        finger_sync_started_ns=9_900_000_000,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: Mock(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)),
    )
    message = JointState()
    message.header.stamp.sec = stamp_sec
    message.name = ['right_finger_joint']
    message.position = [0.010]
    if left is not None:
        message.name.append('left_finger_joint')
        message.position.append(left)

    TargetMotion.joint_state_callback(node, message)

    assert TargetMotion.fingers_reached_goal(node, 0.010) is expected
