#!/usr/bin/env python3
"""Build denser scans from real points with odometry motion compensation."""

from collections import deque
import math
import signal

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rotate_points(points, quaternions):
    """Apply XYZW unit quaternions to corresponding row-vector points."""
    xyz = quaternions[..., :3]
    w = quaternions[..., 3:4]
    return points + 2 * np.cross(xyz, np.cross(xyz, points) + w * points)


def interpolate_poses(history, times):
    """Interpolate positions and short quaternion segments on source time."""
    poses = np.asarray(history, dtype=float)
    right = np.searchsorted(poses[:, 0], times, side='right')
    right = np.clip(right, 1, len(poses) - 1)
    left = right - 1
    dt = poses[right, 0] - poses[left, 0]
    ratio = np.clip((times - poses[left, 0]) / np.maximum(dt, 1e-9), 0, 1)
    position = poses[left, 1:4] * (1 - ratio[:, None]) + poses[right, 1:4] * ratio[:, None]
    qa = poses[left, 4:8]
    qb = poses[right, 4:8].copy()
    qb[np.sum(qa * qb, axis=1) < 0] *= -1
    quaternion = qa * (1 - ratio[:, None]) + qb * ratio[:, None]
    quaternion /= np.linalg.norm(quaternion, axis=1, keepdims=True)
    return position, quaternion


def read_points(message):
    """Read XYZ and relative point times using the PointCloud2 layout."""
    fields = {field.name: field for field in message.fields}
    endian = '>' if message.is_bigendian else '<'
    types = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
    xyz = np.stack([
        np.ndarray(
            (message.height, message.width), dtype=endian + types[fields[axis].datatype],
            buffer=message.data, offset=fields[axis].offset,
            strides=(message.row_step, message.point_step)).reshape(-1)
        for axis in ('x', 'y', 'z')], axis=1).astype(float)
    offsets = np.zeros(len(xyz))
    if 'time' in fields and fields['time'].datatype in types:
        field = fields['time']
        offsets = np.ndarray(
            (message.height, message.width), dtype=endian + types[field.datatype],
            buffer=message.data, offset=field.offset,
            strides=(message.row_step, message.point_step)).reshape(-1).astype(float)
        if not np.isfinite(offsets).all() or offsets.min() < 0:
            raise ValueError('Point time field must contain finite nonnegative relative times')
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], offsets[valid]


class AccumulatedScan(Node):
    def __init__(self):
        super().__init__('go2_accumulated_scan')
        for name, value in (
                ('cloud_topic', '/go2/slam/cloud_base'), ('odom_topic', '/go2/slam/odom'),
                ('scan_topic', '/scan'), ('window_seconds', 1.0),
                ('minimum_height', -0.2), ('maximum_height', 1.0),
                ('range_min', 0.2), ('range_max', 5.0),
                ('angle_increment', math.pi / 180), ('gap_reset_seconds', .4),
                ('minimum_publish_translation', .05), ('minimum_publish_rotation', .05),
                ('minimum_publish_interval', .2), ('stationary_refresh_seconds', 10.0)):
            self.declare_parameter(name, value)
        self.window = float(self.get_parameter('window_seconds').value)
        self.low = float(self.get_parameter('minimum_height').value)
        self.high = float(self.get_parameter('maximum_height').value)
        self.minimum_range = float(self.get_parameter('range_min').value)
        self.maximum_range = float(self.get_parameter('range_max').value)
        self.increment = float(self.get_parameter('angle_increment').value)
        self.gap_reset = float(self.get_parameter('gap_reset_seconds').value)
        self.publish_translation = float(self.get_parameter('minimum_publish_translation').value)
        self.publish_rotation = float(self.get_parameter('minimum_publish_rotation').value)
        self.publish_interval = float(self.get_parameter('minimum_publish_interval').value)
        self.stationary_refresh = float(self.get_parameter('stationary_refresh_seconds').value)
        self.last_published_pose = None
        self.last_publish_time = None
        self.bootstrap_scans = 0
        self.history = deque()
        self.pending = deque(maxlen=10)
        self.clouds = deque()
        self.last_cloud_time = None
        self.last_frame = None
        self.processed = self.resets = self.dropped = 0
        self.latest_directions = self.latest_points = 0
        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value, qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(
            DiagnosticArray, '/go2/slam/scan_diagnostics', 1)
        self.odom_subscription = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.on_odom, qos_profile_sensor_data)
        self.cloud_subscription = self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self.on_cloud, qos_profile_sensor_data)
        self.timer = self.create_timer(2.0, self.publish_diagnostics)

    def on_odom(self, message):
        t = stamp_seconds(message.header.stamp)
        if self.history and t <= self.history[-1][0]:
            if t < self.history[-1][0]:
                self.history.clear()
                self.clouds.clear()
                self.pending.clear()
                self.last_published_pose = None
                self.last_publish_time = None
                self.bootstrap_scans = 0
            else:
                return
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        self.history.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        self.last_frame = message.child_frame_id
        while self.history and t - self.history[0][0] > max(5, self.window * 3):
            self.history.popleft()
        self.process_pending()

    def on_cloud(self, message):
        if message.header.frame_id != self.last_frame:
            self.dropped += 1
            return
        try:
            points, offsets = read_points(message)
        except (KeyError, ValueError, TypeError) as error:
            self.get_logger().error(str(error))
            self.dropped += 1
            return
        if not len(points):
            return
        t = stamp_seconds(message.header.stamp)
        times = t + offsets
        # A long source gap can legitimately produce offsets of several seconds.
        # Reject future point stamps instead of imposing a false 3-second limit.
        if times.max() > self.get_clock().now().nanoseconds * 1e-9 + .5:
            self.get_logger().warning('Cloud point times exceed the current clock by over 0.5 s')
            self.dropped += 1
            return
        if len(self.pending) == self.pending.maxlen:
            self.dropped += 1
        self.pending.append((message.header, points, times))
        self.process_pending()

    def process_pending(self):
        while self.pending and len(self.history) >= 2:
            header, points, times = self.pending[0]
            reference = float(times.max())
            if reference > self.history[-1][0]:
                return
            self.pending.popleft()
            # Use only recent points with recorded pose support. Older points in
            # a long-gap frame must not force rejection of its supported returns.
            supported = times >= max(reference - self.window, self.history[0][0])
            points, times = points[supported], times[supported]
            if not len(points):
                self.dropped += 1
                continue
            if self.last_cloud_time is not None and (
                    reference < self.last_cloud_time
                    or reference - self.last_cloud_time > self.gap_reset):
                self.clouds.clear()
                self.resets += 1
            position, quaternion = interpolate_poses(self.history, times)
            world = rotate_points(points, quaternion) + position
            self.clouds.append((reference, world, times))
            while self.clouds and reference - self.clouds[0][0] > self.window:
                self.clouds.popleft()
            world = np.concatenate([cloud[1] for cloud in self.clouds])
            point_times = np.concatenate([cloud[2] for cloud in self.clouds])
            world = world[point_times >= reference - self.window]
            position, quaternion = interpolate_poses(self.history, np.array([reference]))
            inverse = quaternion.copy()
            inverse[:, :3] *= -1
            current = rotate_points(world - position, inverse)
            ranges = np.linalg.norm(current[:, :2], axis=1)
            keep = ((current[:, 2] >= self.low) & (current[:, 2] <= self.high)
                    & (ranges >= self.minimum_range) & (ranges <= self.maximum_range))
            current, ranges = current[keep], ranges[keep]
            bins = np.floor(
                (np.arctan2(current[:, 1], current[:, 0]) + math.pi)
                / self.increment).astype(int)
            size = math.ceil(2 * math.pi / self.increment)
            valid = (bins >= 0) & (bins < size)
            distances = np.full(size, np.inf, dtype=np.float32)
            np.minimum.at(distances, bins[valid], ranges[valid])
            self.latest_points = int(keep.sum())
            self.latest_directions = int(np.isfinite(distances).sum())
            self.last_cloud_time = reference
            if reference - self.clouds[0][0] < min(.5, self.window * .8):
                continue
            qx, qy, qz, qw = quaternion[0]
            pose = (position[0, 0], position[0, 1],
                    math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))
            if self.last_publish_time is not None:
                elapsed = reference - self.last_publish_time
                if elapsed < self.publish_interval:
                    continue
                a = self.last_published_pose
                moved = (math.hypot(pose[0]-a[0], pose[1]-a[1]) >= self.publish_translation
                         or abs(math.remainder(pose[2]-a[2], 2*math.pi)) >= self.publish_rotation)
                refresh = elapsed >= self.stationary_refresh
                bootstrap = self.bootstrap_scans < 3 and elapsed >= .5
                if not (moved or refresh or bootstrap):
                    continue
            msg = LaserScan()
            ns = round(reference * 1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
            msg.header.frame_id = header.frame_id
            msg.angle_min = -math.pi
            msg.angle_max = -math.pi + (size - 1) * self.increment
            msg.angle_increment = self.increment
            msg.scan_time = .0666667
            msg.time_increment = 0.0
            msg.range_min, msg.range_max = self.minimum_range, self.maximum_range
            msg.ranges = distances.tolist()
            self.publisher.publish(msg)
            self.processed += 1
            self.bootstrap_scans += 1
            self.last_publish_time = reference
            self.last_published_pose = pose

    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'go2/slam/accumulated_scan'
        status.level = (
            DiagnosticStatus.OK if self.latest_directions >= 90 else DiagnosticStatus.WARN)
        status.message = 'Measured points accumulated with per-point odometry compensation'
        values = {'scans': self.processed, 'window_seconds': self.window,
                  'valid_points': self.latest_points, 'finite_directions': self.latest_directions,
                  'gap_resets': self.resets, 'dropped_clouds': self.dropped}
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics.publish(message)


def main():
    rclpy.init()
    node = AccumulatedScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
