#!/usr/bin/env python3
"""Estimate the lidar-observed 2D motion between two windows of a static bag.

The transform maps late-window points into the early-window body frame. A
multi-start point-to-point ICP is used because the scan is sparse and the
scene is not guaranteed to constrain yaw uniquely. This is an onboard lidar
consistency check, not external ground truth. Source ROS 2 Humble first;
the bag is never written.
"""

import argparse
import json
import math
import shutil
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField


def open_reader(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/utlidar/cloud_base']))
    return reader


def stamp(message):
    value = message.header.stamp
    return value.sec + value.nanosec * 1e-9


def read_points(message):
    fields = {field.name: field for field in message.fields}
    endian = '>' if message.is_bigendian else '<'
    types = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
    arrays = []
    for axis in ('x', 'y', 'z'):
        field = fields[axis]
        arrays.append(np.ndarray(
            (message.height, message.width), dtype=endian + types[field.datatype],
            buffer=message.data, offset=field.offset,
            strides=(message.row_step, message.point_step)).reshape(-1).astype(float))
    points = np.stack(arrays, axis=1)
    return points[np.isfinite(points).all(axis=1)]


def select(points, minimum_height=-0.2, maximum_height=1.0,
           minimum_range=0.8, maximum_range=5.0):
    radius = np.hypot(points[:, 0], points[:, 1])
    keep = ((points[:, 2] >= minimum_height) &
            (points[:, 2] <= maximum_height) &
            (radius >= minimum_range) & (radius <= maximum_range))
    return points[keep, :2]


def voxel(points, size=0.02):
    if not len(points):
        return points
    keys = np.round(points / size).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[index]


def icp(source, target, init_yaw, iterations=40, trim=0.8):
    rotation = np.eye(2)
    translation = np.zeros(2)
    tree = cKDTree(target)
    previous = float('inf')
    for _ in range(iterations):
        moved = source @ rotation.T + translation
        distance, index = tree.query(moved)
        threshold = np.quantile(distance, trim)
        keep = distance <= threshold
        if keep.sum() < 10:
            return None
        src = moved[keep]
        dst = target[index[keep]]
        src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
        covariance = (src - src_mean).T @ (dst - dst_mean)
        u, _, vt = np.linalg.svd(covariance)
        delta = vt.T @ u.T
        if np.linalg.det(delta) < 0:
            vt[-1] *= -1
            delta = vt.T @ u.T
        delta_t = dst_mean - delta @ src_mean
        rotation = delta @ rotation
        translation = delta @ translation + delta_t
        moved = source @ rotation.T + translation
        distance = tree.query(moved)[0]
        rmse = float(np.sqrt(np.mean(np.square(distance))))
        if abs(previous - rmse) < 1e-6:
            break
        previous = rmse
    distance = tree.query(source @ rotation.T + translation)[0]
    return {
        'yaw_deg': math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])),
        'translation_m': [float(translation[0]), float(translation[1])],
        'translation_norm_m': float(np.linalg.norm(translation)),
        'rmse_m': float(np.sqrt(np.mean(np.square(distance)))),
        'median_m': float(np.median(distance)),
        'inlier_fraction_10cm': float(np.mean(distance <= 0.10)),
        'points_source': int(len(source)),
        'points_target': int(len(target)),
        'initial_yaw_deg': init_yaw}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--window-seconds', type=float, default=60.0)
    parser.add_argument('--cloud-stride', type=int, default=10)
    parser.add_argument('--initial-yaw-step-deg', type=float, default=1.0,
                        help='Multi-start step over +/- 10 degrees')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output directory must not exist')
    args.output.mkdir(parents=True)

    times, points = [], []
    reader = open_reader(args.bag)
    seen = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        seen += 1
        if seen % max(1, args.cloud_stride):
            continue
        message = deserialize_message(data, PointCloud2)
        selected = select(read_points(message))
        if len(selected):
            times.append(stamp(message))
            points.append(selected)
    times = np.asarray(times)
    if len(times) < 4:
        parser.error('not enough clouds with points')
    order = np.argsort(times)
    times, points = times[order], [points[index] for index in order]
    start, end = times[0], times[-1]
    duration = end - start
    window = min(args.window_seconds, max(10.0, duration / 4.0))
    early = voxel(np.vstack([p for t, p in zip(times, points)
                             if t <= start + window]))
    late = voxel(np.vstack([p for t, p in zip(times, points)
                            if t >= end - window]))

    results = []
    step = math.radians(args.initial_yaw_step_deg)
    for initial in np.arange(-math.radians(10), math.radians(10) + step / 2, step):
        result = icp(late, early, initial)
        if result:
            results.append(result)
    if not results:
        parser.error('ICP failed')
    best = min(results, key=lambda item: (item['median_m'], item['rmse_m']))
    ranked = sorted(results, key=lambda item: item['median_m'])
    output = {
        'bag': str(args.bag.resolve()),
        'duration_s': duration,
        'window_seconds': window,
        'early_points': int(len(early)),
        'late_points': int(len(late)),
        'best': best,
        'best_three_initial_yaws': ranked[:3],
        'interpretation': (
            'Transform maps late points into the early body frame. If the scene is '
            'static, its yaw is the physical robot yaw change between the windows. '
            'Multiple distinct low-error minima would indicate ambiguous geometry.'),
    }
    shutil.copy2(Path(__file__).resolve(), args.output / 'analyze_stationary_icp.py')
    (args.output / 'stationary_icp.json').write_text(
        json.dumps(output, ensure_ascii=False, indent=2))
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
