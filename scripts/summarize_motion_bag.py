#!/usr/bin/env python3
"""Read a finished SQLite rosbag and quantify motion from robot odometry."""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import sqlite3
import statistics
from zoneinfo import ZoneInfo

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--bag-directory', type=Path, help='Explicitly select a recovered bag directory')
    args = parser.parse_args()
    run = args.run.resolve()
    bag = (args.bag_directory or run / 'bag').resolve()
    config_path = run / 'run_metadata.yaml'
    config = yaml.safe_load(config_path.read_text())
    recovered_prefix = (run / 'recovery_report.json').is_file() and bag == (run / 'recovered_bag').resolve()
    # Reading a live SQLite recorder with a zero-timeout writer can abort it.
    # Inspect ROS messages while recording; only read the closed bag here.
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            command = (process / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if b'record' in command and b'--output' in command and str(bag).encode() in command:
            raise RuntimeError('Recorder is still active: stop it cleanly before checking its SQLite files')
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    files = [bag / name for name in metadata['relative_file_paths']]
    start_ns = metadata['starting_time']['nanoseconds_since_epoch']
    duration = metadata['duration']['nanoseconds'] / 1e9
    summary = {'checked_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
               'run_path': str(run), 'bag_path': str(bag), 'duration_s': duration,
               'recording_completion': 'recovered_prefix' if recovered_prefix else
                   'incomplete' if config.get('recording_completed_normally') is False else 'completed_file',
               'total_messages': metadata['message_count'],
               'bag_size_mib': sum(path.stat().st_size for path in files) / 1024**2,
               'streams': {}, 'sqlite_checks': {}}
    poses = []
    offsets = []
    errors, modes = Counter(), Counter()
    power_fields = {}
    power_status = Counter()
    power_samples = 0
    for db in files:
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
            check = conn.execute('PRAGMA quick_check').fetchone()[0]
            summary['sqlite_checks'][db.name] = check
            if check != 'ok':
                raise RuntimeError(f'SQLite integrity failed: {db}: {check}')
            for topic_id, name, type_name in conn.execute('SELECT id,name,type FROM topics'):
                count = conn.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?', (topic_id,)).fetchone()[0]
                entry = summary['streams'].setdefault(name, {'messages': 0})
                entry['messages'] += count
                if name not in ('/utlidar/robot_odom', '/sportmodestate',
                                '/utlidar/cloud', '/utlidar/cloud_base', '/lf/lowstate'):
                    continue
                cls = get_message(type_name)
                for timestamp, data in conn.execute(
                        'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic_id,)):
                    message = deserialize_message(data, cls)
                    if name in ('/utlidar/cloud', '/utlidar/cloud_base'):
                        source_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
                        entry.setdefault('_source_ns', []).append(source_ns)
                        entry.setdefault('_record_ns', []).append(timestamp)
                        entry.setdefault('empty_clouds', 0)
                        if message.width * message.height == 0:
                            entry['empty_clouds'] += 1
                            continue
                        field = next((f for f in message.fields if f.name == 'time'), None)
                        if field and field.datatype in (7, 8):
                            dtype = ('>' if message.is_bigendian else '<') + ('f4' if field.datatype == 7 else 'f8')
                            times = np.ndarray((message.height, message.width), dtype=dtype,
                                               buffer=message.data, offset=field.offset,
                                               strides=(message.row_step, message.point_step)).reshape(-1)
                            entry.setdefault('invalid_relative_time_clouds', 0)
                            if not np.isfinite(times).all() or times.min() < 0:
                                entry['invalid_relative_time_clouds'] += 1
                            else:
                                entry['maximum_relative_point_time_s'] = max(
                                    entry.get('maximum_relative_point_time_s', 0), float(times.max()))
                                entry['clouds_with_relative_times_over_3s'] = (
                                    entry.get('clouds_with_relative_times_over_3s', 0) + int(times.max() > 3))
                    elif name == '/lf/lowstate':
                        power_samples += 1
                        power_status[int(message.bms_state.status)] += 1
                        values = {'bms_soc_raw': int(message.bms_state.soc),
                                  'bms_current_raw': int(message.bms_state.current),
                                  'power_v_raw': float(message.power_v),
                                  'power_a_raw': float(message.power_a),
                                  'board_temperature_ntc1_raw': int(message.temperature_ntc1),
                                  'board_temperature_ntc2_raw': int(message.temperature_ntc2)}
                        for key, value in values.items():
                            if not math.isfinite(value):
                                raise RuntimeError(f'Non-finite power field: {key}')
                            power_entry = power_fields.setdefault(key, {'first': value, 'last': value,
                                                                        'min': value, 'max': value})
                            power_entry['last'] = value
                            power_entry['min'] = min(power_entry['min'], value)
                            power_entry['max'] = max(power_entry['max'], value)
                    elif name == '/utlidar/robot_odom':
                        source_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
                        offsets.append((timestamp-source_ns)/1e9)
                        p, q = message.pose.pose.position, message.pose.pose.orientation
                        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                        pose = ((timestamp-start_ns)/1e9, p.x, p.y, p.z, yaw)
                        if not all(math.isfinite(value) for value in pose):
                            raise RuntimeError('Non-finite odometry pose')
                        poses.append(pose)
                    else:
                        errors[int(message.error_code)] += 1
                        modes[int(message.mode)] += 1
    for entry in summary['streams'].values():
        entry['received_hz'] = entry['messages'] / duration if duration else None
        if '_source_ns' in entry:
            for key, output in (('_source_ns', 'source_interval_s'), ('_record_ns', 'receive_interval_s')):
                stamps = sorted(entry.pop(key)) if key == '_record_ns' else entry.pop(key)
                intervals = np.diff(stamps) / 1e9
                entry[output] = {'median': float(np.median(intervals)) if len(intervals) else None,
                                 'maximum': float(intervals.max()) if len(intervals) else None,
                                 'gaps_over_025s': int((intervals > .25).sum()),
                                 'nonincreasing': int((intervals <= 0).sum())}
    summary['reported_error_codes'] = dict(errors)
    summary['reported_modes'] = dict(modes)
    if power_samples:
        summary['raw_power_fields'] = {'samples': power_samples, 'status_counts': dict(power_status),
                                       'fields': power_fields,
                                       'interpretation': 'Raw LowState fields; units/calibration and zero values require verification'}
    poses.sort()
    if not poses:
        raise RuntimeError('No odometry messages to quantify movement')
    initial = [p for p in poses if p[0] <= poses[0][0] + 1]
    final = [p for p in poses if p[0] >= poses[-1][0] - 1]
    a = [statistics.median(p[i] for p in initial) for i in (1, 2, 3, 4)]
    b = [statistics.median(p[i] for p in final) for i in (1, 2, 3, 4)]
    dx, dy = b[0]-a[0], b[1]-a[1]
    tail = [p for p in poses if p[0] >= poses[-1][0]-5]
    summary['odometry_motion'] = {
        'measurement': 'Recorded odometry estimate; independent ground truth not measured',
        'start_pose_median_xyz_yaw': a,
        'end_pose_median_xyz_yaw': b,
        'net_xy_displacement_m': math.hypot(dx, dy),
        'forward_projection_m': dx*math.cos(a[3]) + dy*math.sin(a[3]),
        'lateral_projection_m': -dx*math.sin(a[3]) + dy*math.cos(a[3]),
        'net_yaw_change_deg': math.degrees(math.remainder(b[3]-a[3], 2*math.pi)),
        'maximum_yaw_change_from_start_deg': max(
            abs(math.degrees(math.remainder(p[4]-a[3], 2*math.pi))) for p in poses),
        'max_distance_from_start_m': max(math.hypot(p[1]-a[0], p[2]-a[1]) for p in poses),
        'last_5s_position_range_m': {label: max(p[i] for p in tail)-min(p[i] for p in tail)
                                     for i, label in ((1, 'x'), (2, 'y'), (3, 'z'))},
        'last_5s_yaw_range_deg': math.degrees(max(p[4] for p in tail)-min(p[4] for p in tail)),
    }
    (run / 'motion_validation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    with (run / 'odometry_trajectory.csv').open('w') as output:
        output.write('record_elapsed_s,x_m,y_m,z_m,yaw_rad\n')
        last = -1
        for pose in poses:
            if pose[0]-last >= 0.2:
                output.write(','.join(str(value) for value in pose)+'\n')
                last = pose[0]
    config['result'] = ('RECOVERED_READABLE_PREFIX; original recording aborted, incomplete run'
                        if recovered_prefix else
                        'Readable recording; completion status retained as incomplete'
                        if config.get('recording_completed_normally') is False else
                        'Readable completed recording; odometry quantified in motion_validation.json')
    config['source_timestamp_meaning'] = 'Original robot source times retained'
    config['record_time_minus_odom_source_time_median_seconds'] = statistics.median(offsets)
    config_path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
