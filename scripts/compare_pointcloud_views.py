#!/usr/bin/env python3
"""Compare actual full-height cloud points and the SLAM height slice in one window."""

import argparse
from datetime import datetime
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sqlite3
import subprocess
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--start', type=float, default=120)
    parser.add_argument('--seconds', type=float, default=5)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.start, args.seconds)) or args.start < 0 or not 0 < args.seconds <= 20:
        parser.error('Use a finite nonnegative start and a duration in (0,20] seconds')
    workspace = Path(__file__).resolve().parents[1]
    script = workspace / 'src/go2_slam_bringup/scripts/go2_accumulated_scan.py'
    spec = importlib.util.spec_from_file_location('scan_math', script)
    scan_math = importlib.util.module_from_spec(spec); spec.loader.exec_module(scan_math)
    bag = args.run.resolve() / 'bag'
    for process in Path('/proc').iterdir():
        if not process.name.isdigit(): continue
        try: command = (process / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError): continue
        if b'record' in command and str(bag).encode() in command:
            parser.error('Stop the active recorder before inspecting the bag')
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start_ns = metadata['starting_time']['nanoseconds_since_epoch'] + round(args.start * 1e9)
    end_ns = start_ns + round(args.seconds * 1e9)
    poses, clouds = [], []
    for file in metadata['relative_file_paths']:
        with sqlite3.connect(f'file:{bag/file}?mode=ro', uri=True) as connection:
            topics = {name: tid for tid, name in connection.execute('SELECT id,name FROM topics')}
            for data, in connection.execute(
                    'SELECT data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
                    (topics['/utlidar/robot_odom'], start_ns-2000000000, end_ns+2000000000)):
                m = deserialize_message(data, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
                poses.append([scan_math.stamp_seconds(m.header.stamp), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            for data, in connection.execute(
                    'SELECT data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
                    (topics['/utlidar/cloud_base'], start_ns, end_ns)):
                m = deserialize_message(data, PointCloud2)
                points, offsets = scan_math.read_points(m)
                clouds.append((points, scan_math.stamp_seconds(m.header.stamp) + offsets))
    if not clouds or len(poses) < 2:
        raise RuntimeError('Window lacks cloud or odometry observations')
    poses.sort(key=lambda p: p[0])
    times = np.concatenate([c[1] for c in clouds])
    points = np.concatenate([c[0] for c in clouds])
    supported = (times >= poses[0][0]) & (times <= poses[-1][0])
    points, times = points[supported], times[supported]
    position, quaternion = scan_math.interpolate_poses(poses, times)
    world = scan_math.rotate_points(points, quaternion) + position
    reference_position, reference_quaternion = scan_math.interpolate_poses(poses, np.array([times.max()]))
    inverse = reference_quaternion.copy(); inverse[:, :3] *= -1
    body = scan_math.rotate_points(world-reference_position, inverse)
    ranges = np.linalg.norm(body[:, :2], axis=1)
    in_range = body[(ranges >= .2) & (ranges <= 5)]
    height_slice = in_range[(in_range[:, 2] >= -.2) & (in_range[:, 2] <= 1.)]
    voxels = np.floor(in_range / .03).astype(np.int64)
    _, indices = np.unique(voxels, axis=0, return_index=True)
    display = in_range[indices]
    selected = display[(display[:, 2] >= -.2) & (display[:, 2] <= 1.)]
    root = Path('/media/yaorz/Elements/unitree_go2/data_validation') / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d')
    subprocess.run(['python3', str(workspace / 'scripts/check_external_storage.py'), '--root', str(root), '--probe'], check=True)
    out = root / ('point_views_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H%M%S_%f'))
    out.mkdir(parents=True)
    shutil.copy2(__file__, out / 'compare_pointcloud_views.py')
    shutil.copy2(script, out / 'go2_accumulated_scan.py')
    np.savez_compressed(out / 'actual_points.npz', body_points=in_range.astype(np.float32),
                        height_slice=height_slice.astype(np.float32))
    report = {'source_bag': str(bag), 'record_time_window_s': [args.start, args.start+args.seconds],
              'cloud_frames': len(clouds), 'range_m': [.2, 5], 'height_slice_m': [-.2, 1.],
              'points_in_range': len(in_range), 'points_in_height_slice': len(height_slice),
              'height_slice_fraction': len(height_slice) / len(in_range),
              'points_below_slice': int((in_range[:, 2] < -.2).sum()),
              'points_above_slice': int((in_range[:, 2] > 1.).sum()),
              'visualization': '3cm voxel selection preserves measured points; no interpolation or new surfaces',
              'coordinates': 'Per-point odometry compensation into the last body pose of this five-second window',
              'limits': 'Point views are not a 3D SLAM map, independent geometry truth or occupancy-grid validation'}
    (out / 'comparison.json').write_text(json.dumps(report, indent=2))
    fig = plt.figure(figsize=(13, 6.6))
    ax = fig.add_subplot(121, projection='3d')
    scatter = ax.scatter(display[:, 0], display[:, 1], display[:, 2], c=display[:, 2],
                         cmap='viridis', s=2, alpha=.65, linewidths=0)
    ax.view_init(elev=25, azim=-125)
    ax.set_xlabel('body x (m)'); ax.set_ylabel('body y (m)'); ax.set_zlabel('height (m)')
    ax.set_title('Same 5-second window: all observed heights')
    ax.set_box_aspect(np.maximum(np.ptp(display, axis=0), .5))
    fig.colorbar(scatter, ax=ax, shrink=.6, label='Measured height relative to body (m)', pad=.1)
    ax2 = fig.add_subplot(122)
    ax2.scatter(selected[:, 0], selected[:, 1], s=3, c='#222222', alpha=.65)
    ax2.scatter([0], [0], s=45, c='#d94c36', label='Body origin')
    ax2.set_xlabel('body x (m)'); ax2.set_ylabel('body y (m)'); ax2.set_aspect('equal')
    ax2.set_xlim(in_range[:, 0].min(), in_range[:, 0].max())
    ax2.set_ylim(in_range[:, 1].min(), in_range[:, 1].max())
    ax2.set_title(f'Same points after height slice: -0.2 to 1.0 m\n{100*report["height_slice_fraction"]:.1f}% of in-range points retained')
    ax2.grid(alpha=.2); ax2.legend(loc='lower right')
    fig.suptitle('3D scene appearance and 2D navigation input retain different information')
    fig.subplots_adjust(left=.08, right=.98, bottom=.12, top=.86, wspace=.28)
    fig.savefig(out / 'point_views.png', dpi=150);plt.close(fig)
    print('Comparison:', out)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
