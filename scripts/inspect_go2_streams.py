#!/usr/bin/env python3
"""Passively inspect Go2 streams and save a compact diagnostic report."""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import time
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from unitree_go.msg import SportModeState


STREAMS = {
    '/utlidar/cloud': PointCloud2,
    '/utlidar/cloud_base': PointCloud2,
    '/utlidar/cloud_deskewed': PointCloud2,
    '/utlidar/voxel_map': PointCloud2,
    '/utlidar/robot_odom': Odometry,
    '/utlidar/imu': Imu,
    '/sportmodestate': SportModeState,
    '/lf/sportmodestate': SportModeState,
    '/uslam/cloud_map': PointCloud2,
    '/uslam/frontend/odom': Odometry,
    '/uslam/localization/odom': Odometry,
    '/tf': TFMessage,
    '/tf_static': TFMessage,
}


def source_stamp(message):
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    if stamp is None:
        stamp = getattr(message, 'stamp', None)
    return None if stamp is None else stamp.sec + stamp.nanosec * 1e-9


def cloud_sample(message):
    fields = {field.name: field for field in message.fields}
    if not all(axis in fields for axis in ('x', 'y', 'z')):
        return {'error': 'Missing x/y/z fields'}
    formats = {PointField.FLOAT32: 'f', PointField.FLOAT64: 'd'}
    if any(fields[axis].datatype not in formats for axis in ('x', 'y', 'z')):
        return {'error': 'Unsupported XYZ field type'}
    count = message.width * message.height
    if count == 0 or len(message.data) == 0:
        return {'points': count, 'finite_samples': 0, 'error': 'Empty cloud'}
    unpackers = {
        axis: struct.Struct(('>' if message.is_bigendian else '<') + formats[fields[axis].datatype])
        for axis in ('x', 'y', 'z')
    }
    samples = []
    for index in range(0, count, max(1, count // 128)):
        row, col = divmod(index, message.width)
        base = row * message.row_step + col * message.point_step
        try:
            xyz = [unpackers[axis].unpack_from(message.data, base + fields[axis].offset)[0]
                   for axis in ('x', 'y', 'z')]
        except struct.error:
            return {'error': 'Cloud payload shorter than layout'}
        if all(math.isfinite(value) for value in xyz):
            samples.append(xyz)
    return {
        'points': count,
        'payload_bytes': len(message.data),
        'finite_samples': len(samples),
        'sample_xyz_min': [min(point[axis] for point in samples) for axis in range(3)] if samples else None,
        'sample_xyz_max': [max(point[axis] for point in samples) for axis in range(3)] if samples else None,
        'fields': [{'name': field.name, 'offset': field.offset, 'datatype': field.datatype}
                   for field in message.fields],
    }


class StreamInspector(Node):
    def __init__(self, light=False):
        super().__init__('go2_passive_stream_inspector')
        self.records = {}
        self.subscriptions_kept = []
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, cls in STREAMS.items():
            if light and topic not in (
                    '/utlidar/cloud', '/utlidar/cloud_base', '/utlidar/robot_odom',
                    '/utlidar/imu', '/sportmodestate', '/lf/sportmodestate', '/tf', '/tf_static'):
                continue
            self.records[topic] = {'count': 0, 'frames': Counter(), 'payload_bytes': 0,
                                   'max_receive_gap_s': 0.0, 'stamp_backwards': 0}
            qos = static_qos if topic == '/tf_static' else qos_profile_sensor_data
            self.subscriptions_kept.append(self.create_subscription(
                cls, topic, lambda msg, key=topic: self.observe(key, msg), qos))

    def observe(self, topic, message):
        record = self.records[topic]
        now = time.monotonic()
        if record['count']:
            record['max_receive_gap_s'] = max(record['max_receive_gap_s'], now - record['last_received'])
        else:
            record['first_received'] = now
        record['last_received'] = now
        record['count'] += 1
        stamp = source_stamp(message)
        if stamp is not None:
            if 'first_source_stamp' not in record:
                record['first_source_stamp'] = stamp
            if 'last_source_stamp' in record and stamp < record['last_source_stamp']:
                record['stamp_backwards'] += 1
            record['last_source_stamp'] = stamp
            record['source_to_pc_time_offset_s'] = time.time() - stamp
        if hasattr(message, 'header'):
            record['frames'][message.header.frame_id] += 1
        if isinstance(message, PointCloud2):
            record['payload_bytes'] += len(message.data)
            if record['count'] == 1 or record['count'] % 20 == 0:
                record['latest_sample'] = cloud_sample(message)
        elif isinstance(message, Odometry):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            record['child_frame_id'] = message.child_frame_id
            pose = {'position': [p.x, p.y, p.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w]}
            record.setdefault('first_pose', pose)
            record['latest_pose'] = pose
        elif isinstance(message, SportModeState):
            record['latest_state'] = {'mode': int(message.mode), 'error_code': int(message.error_code),
                                       'position': [float(value) for value in message.position],
                                       'velocity': [float(value) for value in message.velocity],
                                       'yaw_speed': float(message.yaw_speed), 'body_height': float(message.body_height)}
        elif isinstance(message, TFMessage):
            record['transform_edges'] = sorted({(tf.header.frame_id, tf.child_frame_id)
                                                 for tf in message.transforms})

    def report(self, duration):
        result = {}
        for topic, record in self.records.items():
            record['frames'] = dict(record['frames'])
            record['observed_hz'] = record['count'] / duration
            record['payload_mib_per_s'] = record['payload_bytes'] / duration / 1024**2
            if record['count'] > 1:
                span = record['last_received'] - record['first_received']
                record['active_receive_hz'] = (record['count'] - 1) / span if span else None
            record['publishers_discovered'] = self.count_publishers(topic)
            record.pop('first_received', None)
            record.pop('last_received', None)
            result[topic] = record
        return {'observed_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                'duration_s': duration, 'method': 'Passive subscriptions; frequency reflects reception during this observation',
                'streams': result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--light', action='store_true')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init()
    node = StreamInspector(light=args.light)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report(time.monotonic() - start)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        for topic, record in report['streams'].items():
            sample = record.get('latest_sample', {})
            print(f"{topic}: messages={record['count']}, received_hz={record['observed_hz']:.1f}, "
                  f"frames={list(record['frames'])}, points={sample.get('points', '-')}, "
                  f"finite_samples={sample.get('finite_samples', '-')}")
        if args.output:
            print(f'Report saved: {args.output}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
