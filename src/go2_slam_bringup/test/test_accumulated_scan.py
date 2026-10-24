"""Check real geometry invariants and PointCloud2 layouts used by accumulation."""

import importlib.util
import math
from pathlib import Path
import struct

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


SPEC = importlib.util.spec_from_file_location(
    'go2_accumulated_scan',
    Path(__file__).parents[1] / 'scripts' / 'go2_accumulated_scan.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_stationary_landmark_stays_fixed_when_robot_moves_and_turns():
    """Two views of one landmark must align in world and current body frames."""
    history = np.array([
        [0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 0, 0, 0, 0, math.sqrt(.5), math.sqrt(.5)]])
    positions, quaternions = MODULE.interpolate_poses(history, np.array([0., 1.]))
    body_points = np.array([[1., 1., 0.], [1., 0., 0.]])
    world_points = MODULE.rotate_points(body_points, quaternions) + positions
    np.testing.assert_allclose(world_points, [[1, 1, 0], [1, 1, 0]], atol=1e-12)
    inverse = quaternions[1:2].copy()
    inverse[:, :3] *= -1
    compensated = MODULE.rotate_points(world_points - positions[1], inverse)
    np.testing.assert_allclose(compensated, [[1, 0, 0], [1, 0, 0]], atol=1e-12)


def test_heading_wrap_uses_short_rotation():
    """Crossing plus/minus pi must not interpolate through zero heading."""
    a, b = math.radians(170), math.radians(-170)
    history = np.array([
        [0, 0, 0, 0, 0, 0, math.sin(a / 2), math.cos(a / 2)],
        [1, 0, 0, 0, 0, 0, math.sin(b / 2), math.cos(b / 2)]])
    _, rotation = MODULE.interpolate_poses(history, np.array([.5]))
    vector = MODULE.rotate_points(np.array([[1., 0., 0.]]), rotation)
    np.testing.assert_allclose(vector, [[-1, 0, 0]], atol=1e-12)


def test_organized_cloud_respects_row_padding_and_point_times():
    """Padding bytes between rows must not be read as valid points."""
    message = PointCloud2()
    message.height, message.width = 2, 2
    message.point_step, message.row_step = 16, 40
    message.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'time'))]
    payload = bytearray(80)
    values = [(1., 2., 3., 0.), (4., 5., 6., .02),
              (7., 8., 9., .04), (10., 11., 12., .06)]
    for i, point in enumerate(values):
        row, col = divmod(i, 2)
        struct.pack_into('<ffff', payload, row * 40 + col * 16, *point)
    message.data = bytes(payload)
    points, times = MODULE.read_points(message)
    np.testing.assert_allclose(points, [point[:3] for point in values])
    np.testing.assert_allclose(times, [point[3] for point in values])


def test_cloud_after_long_source_gap_retains_actual_point_times():
    """A real source gap exceeding three seconds is not an invalid time unit."""
    message = PointCloud2()
    message.height, message.width = 1, 3
    message.point_step, message.row_step = 16, 48
    message.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'time'))]
    values = [(1., 0., 0., 0.), (2., 0., 0., 5.7), (3., 0., 0., 5.77)]
    message.data = b''.join(struct.pack('<ffff', *point) for point in values)
    points, times = MODULE.read_points(message)
    np.testing.assert_allclose(points, [point[:3] for point in values])
    np.testing.assert_allclose(times, [point[3] for point in values], atol=1e-6)
