#!/usr/bin/env python3
"""Measure source gaps and host receive errors without opening a live rosbag."""

import argparse
from collections import Counter
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import time
from zoneinfo import ZoneInfo

import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from unitree_go.msg import LidarState, LowState


def link_is_up():
    """Check the configured robot Ethernet interface without changing it."""
    return Path('/sys/class/net/enp2s0/carrier').read_text().strip() == '1'


def network_counters():
    """Read host-wide UDP and interface counters; do not change network settings."""
    lines = Path('/proc/net/snmp').read_text().splitlines()
    counters = {}
    for i, line in enumerate(lines[:-1]):
        if line.startswith('Udp:'):
            counters.update({'udp_' + key: int(value) for key, value in
                             zip(line.split()[1:], lines[i + 1].split()[1:])})
            break
    for key in ('rx_bytes', 'rx_packets', 'rx_errors', 'rx_dropped', 'tx_bytes'):
        counters['enp2s0_' + key] = int(
            Path('/sys/class/net/enp2s0/statistics', key).read_text())
    return counters


class ContinuityMonitor(Node):
    def __init__(self, output, power_state_topic=None):
        super().__init__('go2_continuity_monitor')
        self.output = output
        self.begin = self.window_begin = time.monotonic()
        self.network_start = network_counters()
        self.network_previous = self.network_start
        self.last_source = {}
        self.first_receive = {}
        self.last_receive = {}
        self.totals = Counter()
        self.windows = []
        self.health = {}
        self.logs = []
        self.stop_reason = 'requested_duration_completed'
        self.power_topic = power_state_topic
        self.latest_power = None
        self.power_file = None
        self.power_csv = None
        self.reset_window()
        self.csv_file = (output / 'cloud_timing.csv').open('w')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['receive_elapsed_s', 'source_stamp_s', 'source_gap_s',
                           'point_count', 'maximum_relative_point_time_s'])
        self.subscriptions_kept = []
        for topic, message_type in (
                ('/utlidar/cloud_base', PointCloud2), ('/utlidar/robot_odom', Odometry),
                ('/utlidar/imu', Imu), ('/utlidar/lidar_state', LidarState),
                ('/utlidar/server_log', String)):
            self.subscriptions_kept.append(self.create_subscription(
                message_type, topic, lambda msg, key=topic: self.observe(key, msg),
                qos_profile_sensor_data))
        if power_state_topic:
            self.power_file = (output / 'power_state.csv').open('w')
            self.power_csv = csv.writer(self.power_file)
            self.power_csv.writerow([
                'receive_elapsed_s', 'state_tick_raw', 'bms_soc_raw', 'bms_status_raw',
                'power_v_raw', 'power_a_raw', 'bms_current_raw',
                'bq_ntc_raw', 'mcu_ntc_raw', 'cell_vol_raw',
                'board_temperature_ntc1_raw', 'board_temperature_ntc2_raw', 'fan_frequency_raw'])
            self.subscriptions_kept.append(self.create_subscription(
                LowState, power_state_topic,
                lambda msg: self.observe(power_state_topic, msg), qos_profile_sensor_data))

    def reset_window(self):
        self.counts = Counter()
        self.intervals = {}
        self.frequencies = []
        self.losses = []
        self.errors = Counter()
        self.maximum_point_offset = 0.

    def observe(self, topic, message):
        elapsed = time.monotonic() - self.begin
        self.first_receive.setdefault(topic, elapsed)
        self.last_receive[topic] = elapsed
        self.totals[topic] += 1
        self.counts[topic] += 1
        if isinstance(message, (PointCloud2, Odometry, Imu)):
            source = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            gap = None
            if topic in self.last_source:
                gap = source - self.last_source[topic]
                self.intervals.setdefault(topic, []).append(gap)
            self.last_source[topic] = source
            if isinstance(message, PointCloud2):
                field = next((f for f in message.fields if f.name == 'time'), None)
                maximum = None
                if field and field.datatype in (7, 8) and message.width * message.height:
                    dtype = ('>' if message.is_bigendian else '<') + (
                        'f4' if field.datatype == 7 else 'f8')
                    times = np.ndarray((message.height, message.width), dtype=dtype,
                                       buffer=message.data, offset=field.offset,
                                       strides=(message.row_step, message.point_step))
                    maximum = float(times.max())
                    if np.isfinite(maximum):
                        self.maximum_point_offset = max(self.maximum_point_offset, maximum)
                self.csv.writerow([time.monotonic() - self.begin, source, gap,
                                   message.width * message.height, maximum])
        elif isinstance(message, LidarState):
            self.frequencies.append(float(message.cloud_frequency))
            self.losses.append(float(message.cloud_packet_loss_rate))
            self.errors[int(message.error_state)] += 1
            self.health = {key: getattr(message, key) for key in (
                'firmware_version', 'software_version', 'sdk_version', 'error_state',
                'cloud_frequency', 'cloud_packet_loss_rate', 'serial_buffer_size',
                'serial_buffer_read', 'sys_rotation_speed', 'com_rotation_speed')}
        elif isinstance(message, String):
            self.logs.append({'elapsed_s': time.monotonic() - self.begin, 'text': message.data})
        elif isinstance(message, LowState):
            bms = message.bms_state
            self.latest_power = {
                'receive_elapsed_s': elapsed, 'state_tick_raw': int(message.tick),
                'bms_soc_raw': int(bms.soc), 'bms_status_raw': int(bms.status),
                'power_v_raw': float(message.power_v), 'power_a_raw': float(message.power_a),
                'bms_current_raw': int(bms.current),
                'bq_ntc_raw': [int(value) for value in bms.bq_ntc],
                'mcu_ntc_raw': [int(value) for value in bms.mcu_ntc],
                'cell_vol_raw': [int(value) for value in bms.cell_vol],
                'board_temperature_ntc1_raw': int(message.temperature_ntc1),
                'board_temperature_ntc2_raw': int(message.temperature_ntc2),
                'fan_frequency_raw': [int(value) for value in message.fan_frequency]}
            self.power_csv.writerow([
                value if not isinstance(value, list) else json.dumps(value)
                for value in self.latest_power.values()])

    def finish_window(self):
        now = time.monotonic()
        duration = now - self.window_begin
        network = network_counters()
        streams = {}
        for topic, count in self.counts.items():
            intervals = self.intervals.get(topic, [])
            streams[topic] = {'count': count, 'received_hz': count / duration,
                             'source_interval_median_s': statistics.median(intervals) if intervals else None,
                             'source_interval_max_s': max(intervals) if intervals else None,
                             'gaps_over_025s': sum(t > .25 for t in intervals),
                             'nonincreasing_stamps': sum(t <= 0 for t in intervals)}
        result = {'end_elapsed_s': now - self.begin, 'duration_s': duration,
                  'streams': streams, 'maximum_relative_point_time_s': self.maximum_point_offset,
                  'lidar_error_state_counts': dict(self.errors),
                  'driver_cloud_frequency_median': statistics.median(self.frequencies) if self.frequencies else None,
                  'driver_packet_loss_field_median': statistics.median(self.losses) if self.losses else None,
                  'network_counter_change': {key: value - self.network_previous[key]
                                             for key, value in network.items()}}
        self.windows.append(result)
        with (self.output / 'windows.jsonl').open('a') as handle:
            handle.write(json.dumps(result) + '\n')
        self.csv_file.flush()
        if self.power_file:
            self.power_file.flush()
        cloud = streams.get('/utlidar/cloud_base', {})
        print(f"{now - self.begin:.0f}s: cloud {cloud.get('received_hz', 0):.2f} Hz, "
              f"max gap {cloud.get('source_interval_max_s')}, "
              f"UDP receive drops +{result['network_counter_change'].get('udp_RcvbufErrors', 0)}",
              flush=True)
        self.window_begin = now
        self.network_previous = network
        self.reset_window()

    def save(self):
        self.finish_window()
        duration = time.monotonic() - self.begin
        report = {'duration_s': duration, 'message_counts': dict(self.totals),
                  'stop_reason': self.stop_reason,
                  'topic_receive_coverage': {
                      topic: {'first_receive_elapsed_s': self.first_receive[topic],
                              'last_receive_elapsed_s': self.last_receive[topic],
                              'active_receive_span_s': self.last_receive[topic] - self.first_receive[topic],
                              'active_receive_hz': (count - 1) / (
                                  self.last_receive[topic] - self.first_receive[topic])
                              if self.last_receive[topic] > self.first_receive[topic] else None}
                      for topic, count in self.totals.items()},
                  'windows': self.windows, 'latest_lidar_state': self.health,
                  'power_state_topic': self.power_topic, 'latest_power_state': self.latest_power,
                  'robot_server_logs': self.logs,
                  'network_delta': {key: value - self.network_start[key]
                                    for key, value in network_counters().items()},
                  'limitations': ['UDP counters are host-wide, not robot-only',
                                  'Driver packet-loss field units are not confirmed',
                                  'Power/BMS fields retain raw message values; units and zero-value validity need verification',
                                  'Partial or disconnected windows must not be interpreted as sensor-only frequency',
                                  'Receiving gaps alone does not prove the originating component']}
        (self.output / 'report.json').write_text(json.dumps(report, indent=2))
        self.csv_file.close()
        if self.power_file:
            self.power_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=300)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase', required=True)
    parser.add_argument('--power-state-topic', choices=('/lf/lowstate', '/lowstate'),
                        help='Optionally record raw battery, voltage, temperature and fan fields')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    if not link_is_up():
        raise SystemExit('Robot Ethernet has no carrier; reconnect before starting an observation')
    workspace = Path(__file__).resolve().parents[1]
    subprocess.run(['python3', str(workspace / 'scripts/check_external_storage.py'),
                    '--root', str(args.output.parent), '--probe'], check=True)
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, args.output / 'check_go2_continuity.py')
    (args.output / 'metadata.json').write_text(json.dumps({
        'phase': args.phase, 'started_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        'requested_duration_s': args.seconds, 'physical_control': 'stationary; no motion publishers',
        'power_state_topic': args.power_state_topic,
        'inspection': 'ROS subscriptions and /proc network counters; no active SQLite reads'}, indent=2))
    rclpy.init()
    node = ContinuityMonitor(args.output, power_state_topic=args.power_state_topic)
    link_down_since = None
    next_link_check = time.monotonic()
    try:
        while time.monotonic() - node.begin < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if now >= next_link_check:
                next_link_check = now + .5
                if link_is_up():
                    link_down_since = None
                elif link_down_since is None:
                    link_down_since = now
                elif now - link_down_since >= 3:
                    node.stop_reason = 'robot_ethernet_link_down'
                    break
            if time.monotonic() - node.window_begin >= 60:
                node.finish_window()
    except KeyboardInterrupt:
        node.stop_reason = 'operator_interrupted'
    finally:
        node.save()
        print('Report:', args.output / 'report.json', flush=True)
        if not link_is_up():
            # Observed DDS cleanup can retry indefinitely after interface removal.
            # The passive instrument has flushed/closed all output files above;
            # let the OS close middleware resources when the interface is gone.
            os._exit(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
