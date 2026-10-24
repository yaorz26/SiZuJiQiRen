#!/usr/bin/env python3
"""Compare odometry yaw, gyro yaw and lidar bearing shift on a stationary bag.

All three signals come from the robot. This is a consistency check, not an
external ground-truth measurement. A finished, read-only bag is required.
Source the ROS 2 Humble environment before running; the bag is never written.
"""

import argparse
import csv
import json
import math
import shutil
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry


def open_reader(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def stamp(message):
    value = message.header.stamp
    return value.sec + value.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap_deg(values):
    return np.degrees(np.unwrap(np.asarray(values, dtype=float)))


def circular_shift(reference, moving):
    """Return k and score such that roll(moving, k) best matches reference."""
    reference = np.asarray(reference, dtype=float)
    moving = np.asarray(moving, dtype=float)
    reference = reference / max(reference.sum(), 1e-12)
    moving = moving / max(moving.sum(), 1e-12)
    scores = [float(np.dot(reference, np.roll(moving, k)))
              for k in range(len(reference))]
    k = int(np.argmax(scores))
    if k > len(reference) // 2:
        k -= len(reference)
    return k, scores[k]


def read_points(message):
    fields = {field.name: field for field in message.fields}
    if not all(axis in fields for axis in ('x', 'y', 'z')):
        return np.empty((0, 3))
    endian = '>' if message.is_bigendian else '<'
    types = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
    arrays = []
    for axis in ('x', 'y', 'z'):
        field = fields[axis]
        if field.datatype not in types:
            return np.empty((0, 3))
        arrays.append(np.ndarray(
            (message.height, message.width), dtype=endian + types[field.datatype],
            buffer=message.data, offset=field.offset,
            strides=(message.row_step, message.point_step)).reshape(-1).astype(float))
    points = np.stack(arrays, axis=1)
    return points[np.isfinite(points).all(axis=1)]


def bearing_histogram(points, bins=720, minimum_range=0.8, maximum_range=5.0,
                      minimum_height=-0.2, maximum_height=1.0):
    if not len(points):
        return np.zeros(bins)
    radius = np.hypot(points[:, 0], points[:, 1])
    keep = ((points[:, 2] >= minimum_height) &
            (points[:, 2] <= maximum_height) &
            (radius >= minimum_range) & (radius <= maximum_range))
    if not np.any(keep):
        return np.zeros(bins)
    angle = np.arctan2(points[keep, 1], points[keep, 0])
    histogram, _ = np.histogram(angle, bins=bins, range=(-math.pi, math.pi))
    return histogram.astype(float)


def finest_rate(values):
    differences = np.diff(values)
    differences = differences[differences > 0]
    return 1.0 / np.median(differences) if len(differences) else 0.0


def integration(times, rates):
    if len(times) < 2:
        return np.zeros(0), np.zeros(0)
    dt = np.diff(times)
    area = 0.5 * (rates[1:] + rates[:-1]) * dt
    return times[1:], np.cumsum(area)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cloud-stride', type=int, default=15,
                        help='Read one cloud out of N; bearings use real observations only')
    parser.add_argument('--bins', type=int, default=720)
    parser.add_argument('--window-seconds', type=float, default=60.0)
    args = parser.parse_args()
    if not (args.bag / 'metadata.yaml').is_file():
        parser.error('finished bag metadata is required')
    if args.output.exists():
        parser.error('output directory must not exist')
    args.output.mkdir(parents=True)

    odom_times, odom_x, odom_y, odom_yaw = [], [], [], []
    imu_times, imu_wz, imu_orientation_yaw = [], [], []
    cloud_times, cloud_histograms = [], []
    cloud_seen = 0

    topics = ['/utlidar/robot_odom', '/utlidar/imu', '/utlidar/cloud_base']
    reader = open_reader(args.bag, topics)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/utlidar/robot_odom':
            message = deserialize_message(data, Odometry)
            odom_times.append(stamp(message))
            odom_x.append(message.pose.pose.position.x)
            odom_y.append(message.pose.pose.position.y)
            odom_yaw.append(yaw_from_quaternion(message.pose.pose.orientation))
        elif topic == '/utlidar/imu':
            message = deserialize_message(data, Imu)
            imu_times.append(stamp(message))
            imu_wz.append(message.angular_velocity.z)
            orientation = message.orientation
            norm = math.sqrt(orientation.x ** 2 + orientation.y ** 2 +
                             orientation.z ** 2 + orientation.w ** 2)
            imu_orientation_yaw.append(
                yaw_from_quaternion(orientation) if norm > 1e-6 else float('nan'))
        else:
            cloud_seen += 1
            if cloud_seen % max(1, args.cloud_stride):
                continue
            message = deserialize_message(data, PointCloud2)
            cloud_times.append(stamp(message))
            cloud_histograms.append(bearing_histogram(
                read_points(message), bins=args.bins))

    odom_times = np.asarray(odom_times)
    odom_x = np.asarray(odom_x)
    odom_y = np.asarray(odom_y)
    odom_yaw = np.asarray(odom_yaw)
    imu_times = np.asarray(imu_times)
    imu_wz = np.asarray(imu_wz)
    imu_orientation_yaw = np.asarray(imu_orientation_yaw)

    order = np.argsort(odom_times)
    odom_times, odom_x, odom_y, odom_yaw = (
        odom_times[order], odom_x[order], odom_y[order], odom_yaw[order])
    order = np.argsort(imu_times)
    imu_times, imu_wz, imu_orientation_yaw = (
        imu_times[order], imu_wz[order], imu_orientation_yaw[order])

    if len(odom_times) < 2 or len(imu_times) < 2:
        parser.error('odometry and IMU are required')

    start = max(odom_times[0], imu_times[0])
    end = min(odom_times[-1], imu_times[-1])
    duration = end - start
    odom_mask = (odom_times >= start) & (odom_times <= end)
    imu_mask = (imu_times >= start) & (imu_times <= end)

    odom_yaw_deg = unwrap_deg(odom_yaw[odom_mask])
    odom_yaw_deg -= odom_yaw_deg[0]
    odom_t = odom_times[odom_mask]
    imu_yaw_rad, imu_yaw_integral = integration(imu_times[imu_mask], imu_wz[imu_mask])
    imu_yaw_deg_raw = np.degrees(imu_yaw_integral - imu_yaw_integral[0])
    imu_yaw_deg_raw = np.concatenate([[0.0], imu_yaw_deg_raw])
    imu_yaw_t = np.concatenate([[imu_times[imu_mask][0]], imu_yaw_rad])

    initial = imu_times[imu_mask] < (start + min(30.0, duration / 4.0))
    bias = float(np.median(imu_wz[imu_mask][initial])) if np.any(initial) else 0.0
    corrected = imu_wz[imu_mask] - bias
    corrected_t, corrected_integral = integration(imu_times[imu_mask], corrected)
    imu_yaw_deg_corrected = np.degrees(corrected_integral - corrected_integral[0])
    imu_yaw_deg_corrected = np.concatenate([[0.0], imu_yaw_deg_corrected])
    imu_yaw_t_corrected = np.concatenate([[imu_times[imu_mask][0]],
                                          corrected_t])

    # Interpolate odometry yaw on the IMU timeline for a like-for-like difference.
    odom_on_imu_raw = np.interp(imu_yaw_t, odom_t, odom_yaw_deg)
    odom_on_imu_corrected = np.interp(imu_yaw_t_corrected, odom_t, odom_yaw_deg)

    # Lidar bearing shifts: first window versus last window and versus each minute.
    cloud_times = np.asarray(cloud_times)
    cloud_histograms = np.asarray(cloud_histograms)
    cloud_mask = (cloud_times >= start) & (cloud_times <= end)
    cloud_times = cloud_times[cloud_mask]
    cloud_histograms = cloud_histograms[cloud_mask]
    bin_deg = 360.0 / args.bins
    lidar = {}
    if len(cloud_times) >= 4:
        first = cloud_histograms[cloud_times < start + args.window_seconds].sum(axis=0)
        last = cloud_histograms[cloud_times > end - args.window_seconds].sum(axis=0)
        k, score = circular_shift(first, last)
        lidar['first_last'] = {
            'window_seconds': args.window_seconds,
            'frames': int(np.sum(cloud_times < start + args.window_seconds) +
                          np.sum(cloud_times > end - args.window_seconds)),
            'shift_bins': k,
            'bearing_shift_deg': -k * bin_deg,
            'implied_robot_yaw_change_deg_if_static_scene': k * bin_deg,
            'normalized_correlation': score}
        per_minute = []
        reference = cloud_histograms[cloud_times < start + args.window_seconds].sum(axis=0)
        for minute in range(int(math.floor(duration / 60.0)) + 1):
            low = start + minute * 60.0
            high = min(end, low + 60.0)
            mask = (cloud_times >= low) & (cloud_times <= high)
            if mask.sum() < 2:
                continue
            k, score = circular_shift(reference, cloud_histograms[mask].sum(axis=0))
            per_minute.append({'minute': minute, 'frames': int(mask.sum()),
                               'shift_bins': k,
                               'bearing_shift_deg': -k * bin_deg,
                               'implied_robot_yaw_change_deg_if_static_scene': k * bin_deg,
                               'normalized_correlation': score})
        lidar['per_minute'] = per_minute

    result = {
        'bag': str(args.bag.resolve()),
        'duration_s': duration,
        'odometry': {
            'samples': int(odom_mask.sum()),
            'rate_hz': finest_rate(odom_times),
            'net_yaw_deg': float(odom_yaw_deg[-1]),
            'maximum_absolute_yaw_deg': float(np.max(np.abs(odom_yaw_deg))),
            'net_xy_m': float(math.hypot(odom_x[odom_mask][-1] - odom_x[odom_mask][0],
                                         odom_y[odom_mask][-1] - odom_y[odom_mask][0])),
            'yaw_fit_deg_per_minute': float(np.polyfit(
                odom_t - odom_t[0], odom_yaw_deg, 1)[0] * 60.0),
        },
        'gyro': {
            'samples': int(imu_mask.sum()),
            'rate_hz': finest_rate(imu_times),
            'initial_bias_estimate_rad_s': bias,
            'bias_window_s': min(30.0, duration / 4.0),
            'net_yaw_deg_zero_bias': float(imu_yaw_deg_raw[-1]),
            'net_yaw_deg_initial_bias_removed': float(imu_yaw_deg_corrected[-1]),
            'odometry_yaw_same_interval_zero_bias_deg': float(odom_on_imu_raw[-1]),
            'odometry_yaw_same_interval_corrected_deg': float(odom_on_imu_corrected[-1]),
            'difference_zero_bias_deg': float(imu_yaw_deg_raw[-1] - odom_on_imu_raw[-1]),
            'difference_corrected_deg': float(imu_yaw_deg_corrected[-1] -
                                              odom_on_imu_corrected[-1]),
            'finite_orientation_samples': int(np.isfinite(imu_orientation_yaw).sum()),
        },
        'lidar_bearing': lidar,
        'interpretation': (
            'All signals are onboard observations. Odometry yaw and gyro yaw describe '
            'the robot estimate; lidar bearings describe the environment orientation in '
            'base_link. This does not establish external ground truth.'),
    }
    shutil.copy2(Path(__file__).resolve(), args.output / 'analyze_stationary_drift.py')
    (args.output / 'stationary_drift.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2))
    with (args.output / 'stationary_drift.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['record_elapsed_s', 'odometry_yaw_deg', 'gyro_yaw_deg_raw',
                         'gyro_yaw_deg_bias_removed'])
        for index, (time, yaw_value) in enumerate(zip(odom_t, odom_yaw_deg)):
            gyro_raw = float(np.interp(time, imu_yaw_t, imu_yaw_deg_raw))
            gyro_corrected = float(np.interp(time, imu_yaw_t_corrected,
                                             imu_yaw_deg_corrected))
            writer.writerow([time - start, yaw_value, gyro_raw, gyro_corrected])
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
