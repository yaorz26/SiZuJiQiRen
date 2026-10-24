#!/usr/bin/env python3
"""Align cloud and odometry headers to one ROS clock without changing source topics."""

from collections import deque
from copy import deepcopy
import math
import signal
import statistics

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class SensorTimeBridge(Node):
    def __init__(self):
        super().__init__('go2_sensor_time_bridge')
        self.declare_parameter('cloud_topic', '/utlidar/cloud_base')
        self.declare_parameter('odom_topic', '/utlidar/robot_odom')
        self.declare_parameter('aligned_cloud_topic', '/go2/slam/cloud_base')
        self.declare_parameter('aligned_odom_topic', '/go2/slam/odom')
        self.declare_parameter('alignment_mode', 'auto')
        self.declare_parameter('fixed_offset_seconds', 0.0)
        self.declare_parameter('calibration_samples', 30)
        self.mode = self.get_parameter('alignment_mode').value
        if self.mode not in ('auto', 'fixed', 'passthrough'):
            raise ValueError('alignment_mode must be auto, fixed or passthrough')
        self.samples = deque(maxlen=max(1, int(self.get_parameter('calibration_samples').value)))
        self.offset_ns = None
        if self.mode != 'auto':
            seconds = (
                float(self.get_parameter('fixed_offset_seconds').value)
                if self.mode == 'fixed' else 0.0)
            if not math.isfinite(seconds):
                raise ValueError('fixed_offset_seconds must be finite')
            self.offset_ns = round(seconds * 1e9)
        self.counts = {'odom': 0, 'cloud': 0}
        self.last_source_cloud = None
        self.max_cloud_gap = 0.0
        self.cloud_gaps = 0
        self.clock_resets = 0
        self.last_ros_time = None
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('aligned_odom_topic').value, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('aligned_cloud_topic').value, qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/go2/slam/time_diagnostics', 1)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.on_odom, qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self.on_cloud, qos_profile_sensor_data)
        self.diagnostic_timer = self.create_timer(2.0, self.publish_diagnostics)

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    def clock_ready(self):
        now = self.get_clock().now().nanoseconds
        if now == 0:
            return None
        if self.last_ros_time is not None and now < self.last_ros_time:
            self.clock_resets += 1
            self.samples.clear()
            self.last_source_cloud = None
            if self.mode == 'auto':
                self.offset_ns = None
            self.get_logger().warning(
                'ROS clock moved backwards; restarting automatic calibration')
        self.last_ros_time = now
        return now

    def aligned(self, message):
        result = deepcopy(message)
        stamp = self.stamp_ns(message.header.stamp) + self.offset_ns
        if stamp < 0:
            raise ValueError('Aligned timestamp would be negative')
        result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 1000000000)
        return result

    def on_odom(self, message):
        now = self.clock_ready()
        source = self.stamp_ns(message.header.stamp)
        if now is None or source == 0:
            return
        if self.offset_ns is None:
            self.samples.append(now - source)
            if len(self.samples) < self.samples.maxlen:
                return
            self.offset_ns = round(statistics.median(self.samples))
            self.get_logger().info(
                f'Common timestamp offset calibrated: {self.offset_ns / 1e9:.6f} s; '
                'source topics unchanged')
        self.odom_pub.publish(self.aligned(message))
        self.counts['odom'] += 1

    def on_cloud(self, message):
        if self.clock_ready() is None:
            return
        source = self.stamp_ns(message.header.stamp)
        if source == 0:
            return
        if self.last_source_cloud is not None and source >= self.last_source_cloud:
            gap = (source - self.last_source_cloud) / 1e9
            self.max_cloud_gap = max(gap, self.max_cloud_gap)
            if gap > 0.25:
                self.cloud_gaps += 1
        self.last_source_cloud = source
        if self.offset_ns is not None:
            self.cloud_pub.publish(self.aligned(message))
            self.counts['cloud'] += 1

    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'go2/slam/time_alignment'
        status.hardware_id = 'go2_sensor_input'
        status.level = (
            DiagnosticStatus.WARN
            if self.offset_ns is None or self.cloud_gaps else DiagnosticStatus.OK)
        status.message = (
            'Awaiting common clock calibration' if self.offset_ns is None
            else 'Source timestamps shifted by a shared fixed offset')
        values = {'offset_seconds': None if self.offset_ns is None else self.offset_ns / 1e9,
                  'aligned_clouds': self.counts['cloud'], 'aligned_odometry': self.counts['odom'],
                  'maximum_source_cloud_gap_seconds': self.max_cloud_gap,
                  'source_cloud_gaps_over_0_25_seconds': self.cloud_gaps,
                  'clock_resets': self.clock_resets}
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostics_pub.publish(msg)


def main():
    rclpy.init()
    node = SensorTimeBridge()
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
