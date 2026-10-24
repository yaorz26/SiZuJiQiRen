#!/usr/bin/env python3
"""Run the Go2 SLAM launch on a sensor bag in an isolated local DDS domain."""

import argparse
from collections import Counter
import csv
from datetime import datetime
import json
import hashlib
import math
import os
from pathlib import Path
import shutil
import signal
import statistics
import subprocess
import time
from zoneinfo import ZoneInfo

# Configure the middleware before importing ROS type support.
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'

from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from slam_toolbox.srv import SerializePoseGraph
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
import yaml
from slam_artifacts import preserve_map_geometry, trajectory_timing_summary


class Monitor(Node):
    def __init__(self):
        super().__init__('go2_slam_replay_monitor')
        self.counts = Counter()
        self.latest_map = None
        self.latest_clock_ns = None
        self.cloud_clock_deltas = []
        self.scan_finite_counts = []
        self.edges = set()
        self.transforms = {}
        self.first_slam_pose = None
        self.last_slam_pose = None
        self.graph_markers = []
        self.diagnostics = {}
        self.trajectory = []
        self.last_trajectory_ns = None
        self.map_to_odom_stamp_ns = None
        self.subscriptions_kept = []
        for topic, cls in (('/go2/slam/cloud_base', PointCloud2), ('/go2/slam/odom', Odometry),
                           ('/scan', LaserScan), ('/tf', TFMessage), ('/clock', Clock),
                           ('/go2/slam/time_diagnostics', DiagnosticArray),
                           ('/go2/slam/scan_diagnostics', DiagnosticArray),
                           ('/slam_toolbox/graph_visualization', MarkerArray)):
            self.subscriptions_kept.append(self.create_subscription(
                cls, topic, lambda msg, key=topic: self.observe(key, msg), qos_profile_sensor_data))
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_kept.append(self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos))

    def on_map(self, message):
        self.counts['/map'] += 1
        self.latest_map = message

    def observe(self, topic, message):
        self.counts[topic] += 1
        if topic == '/clock':
            self.latest_clock_ns = message.clock.sec * 1000000000 + message.clock.nanosec
        elif topic == '/go2/slam/cloud_base' and self.latest_clock_ns is not None:
            source_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
            self.cloud_clock_deltas.append((self.latest_clock_ns-source_ns)/1e9)
        elif topic == '/scan':
            self.scan_finite_counts.append(sum(math.isfinite(r) and message.range_min <= r <= message.range_max for r in message.ranges))
        elif topic == '/go2/slam/odom' and self.latest_map is not None and ('map', 'odom') in self.transforms:
            stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
            if self.last_trajectory_ns is None or stamp_ns - self.last_trajectory_ns >= 100000000:
                p, q = message.pose.pose.position, message.pose.pose.orientation
                x, y, yaw = self.transforms[('map', 'odom')]
                body_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                age = ((stamp_ns - self.map_to_odom_stamp_ns) / 1e9
                       if self.map_to_odom_stamp_ns is not None else None)
                self.trajectory.append([stamp_ns, x+math.cos(yaw)*p.x-math.sin(yaw)*p.y,
                    y+math.sin(yaw)*p.x+math.cos(yaw)*p.y, math.remainder(yaw+body_yaw, 2*math.pi),
                    p.x, p.y, body_yaw, age, age is not None and -.2 <= age <= .5])
                self.last_trajectory_ns = stamp_ns
        elif topic == '/tf':
            self.edges.update((tf.header.frame_id, tf.child_frame_id) for tf in message.transforms)
            for tf in message.transforms:
                p, q = tf.transform.translation, tf.transform.rotation
                self.transforms[(tf.header.frame_id, tf.child_frame_id)] = (
                    p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                if (tf.header.frame_id, tf.child_frame_id) == ('map', 'odom'):
                    self.map_to_odom_stamp_ns = tf.header.stamp.sec * 1000000000 + tf.header.stamp.nanosec
            if self.latest_map is not None and ('map', 'odom') in self.transforms and ('odom', 'base_link') in self.transforms:
                x, y, yaw = self.transforms[('map', 'odom')]
                bx, by, byaw = self.transforms[('odom', 'base_link')]
                pose = [x+math.cos(yaw)*bx-math.sin(yaw)*by,
                        y+math.sin(yaw)*bx+math.cos(yaw)*by,
                        math.remainder(yaw+byaw, 2*math.pi)]
                if self.first_slam_pose is None:
                    self.first_slam_pose = pose
                self.last_slam_pose = pose
        elif isinstance(message, DiagnosticArray):
            for status in message.status:
                self.diagnostics[status.name] = {value.key: value.value for value in status.values}
        elif isinstance(message, MarkerArray):
            self.graph_markers = [{'namespace': marker.ns, 'type': int(marker.type),
                                   'points': len(marker.points)} for marker in message.markers]

    def spin_for(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def summary(self):
        result = {'received_messages': dict(self.counts), 'tf_edges': sorted(self.edges),
                  'time_diagnostics': self.diagnostics,
                  'pose_graph_markers': self.graph_markers,
                  'scan_finite_bins': {'min': min(self.scan_finite_counts), 'max': max(self.scan_finite_counts),
                                       'median': statistics.median(self.scan_finite_counts)} if self.scan_finite_counts else None}
        if self.cloud_clock_deltas:
            result['cloud_clock_delta_seconds'] = {'min': min(self.cloud_clock_deltas), 'max': max(self.cloud_clock_deltas)}
        if self.first_slam_pose and self.last_slam_pose:
            a, b = self.first_slam_pose, self.last_slam_pose
            result['slam_pose_change'] = {'first_xy_yaw': a, 'last_xy_yaw': b,
                                         'net_xy_m': math.hypot(b[0]-a[0], b[1]-a[1]),
                                         'net_yaw_deg': math.degrees(math.remainder(b[2]-a[2], 2*math.pi))}
        if self.latest_map:
            msg = self.latest_map
            cells = Counter(int(value) for value in msg.data)
            result['map'] = {'frame': msg.header.frame_id, 'width': msg.info.width, 'height': msg.info.height,
                             'resolution_m': msg.info.resolution, 'cells': dict(cells),
                             'origin': map_origin(msg)}
        return result

    def call(self, client, request, timeout=10):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        future = client.call_async(request)
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        return future.result()


def map_origin(message):
    pose = message.info.origin
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def stop_process(process):
    if process.poll() is not None:
        return
    for sig, seconds in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=seconds)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--accumulate-scan', action='store_true')
    parser.add_argument('--single-frame-scan', action='store_false', dest='accumulate_scan')
    parser.set_defaults(accumulate_scan=True)
    parser.add_argument('--accumulation-window', type=float, default=1.0)
    parser.add_argument('--output-root', type=Path, default=Path(
        '/media/yaorz/Elements/unitree_go2/slam_validation') / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d'))
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate must be positive')
    if not math.isfinite(args.accumulation_window) or args.accumulation_window <= 0:
        parser.error('--accumulation-window must be finite and positive')
    args.bag = args.bag.resolve()
    if not (args.bag / 'metadata.yaml').is_file():
        parser.error('Bag metadata is missing; finalize or recover the recording first')
    for process_dir in Path('/proc').iterdir():
        if not process_dir.name.isdigit():
            continue
        try:
            command = (process_dir / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if b'record' in command and b'--output' in command and str(args.bag).encode() in command:
            parser.error('Stop the active recorder before opening this bag for replay')
    workspace = Path(__file__).resolve().parents[1]
    subprocess.run(['python3', str(workspace/'scripts/check_external_storage.py'),
                    '--root', str(args.output_root), '--probe'], check=True)
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H%M%S_%f')
    out = args.output_root / f'{args.bag.parent.name}_{stamp}'
    out.mkdir(parents=True)
    (out/'maps').mkdir()
    shutil.copytree(workspace/'src/go2_slam_bringup/config', out/'config')
    source_dir = out/'source'
    source_dir.mkdir()
    for source in (
            workspace/'src/go2_slam_bringup/scripts/go2_sensor_time_bridge.py',
            workspace/'src/go2_slam_bringup/scripts/go2_accumulated_scan.py',
            workspace/'src/go2_slam_bringup/launch/go2_slam.launch.py',
            Path(__file__).resolve(), workspace/'scripts/slam_artifacts.py'):
        shutil.copy2(source, source_dir/source.name)
    revision = subprocess.run(['git', '-C', str(workspace), 'rev-parse', 'HEAD'],
                              capture_output=True, text=True).stdout.strip()
    metadata = {'source_bag': str(args.bag.resolve()), 'playback_rate': args.rate,
                'recorded_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                'workspace_revision': revision,
                'accumulate_scan': args.accumulate_scan,
                'accumulation_window_seconds': args.accumulation_window,
                'validation_mode': 'isolated loopback DDS; replay only cloud_base and robot_odom; no motion publishers',
                'source_sha256': {str(source.relative_to(out)): hashlib.sha256(source.read_bytes()).hexdigest()
                                  for source in sorted(out.rglob('*')) if source.is_file()},
                'trajectory_method': 'At odometry time, compose with latest received map->odom TF; record TF time delta; not independent ground truth'}
    (out/'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    processes = []
    handles = []
    rclpy.init()
    node = Monitor()
    result = {}
    try:
        launch_log = (out/'launch.log').open('w'); handles.append(launch_log)
        launch = subprocess.Popen(['ros2', 'launch', 'go2_slam_bringup', 'go2_slam.launch.py', 'use_sim_time:=true',
                                    f'accumulate_scan:={str(args.accumulate_scan).lower()}',
                                    f'accumulation_window:={args.accumulation_window}'],
                                  stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(launch)
        node.spin_for(5)
        if launch.poll() is not None:
            raise RuntimeError('SLAM launch exited; inspect launch.log')
        play_log = (out/'playback.log').open('w'); handles.append(play_log)
        player = subprocess.Popen(['ros2', 'bag', 'play', str(args.bag), '--rate', str(args.rate), '--clock', '100',
                                   '--delay', '2', '--disable-keyboard-controls', '--topics',
                                   '/utlidar/cloud_base', '/utlidar/robot_odom'],
                                  stdout=play_log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(player)
        info = yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        deadline = time.monotonic()+info['duration']['nanoseconds']/1e9/args.rate+30
        while player.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if player.poll() is None:
            raise RuntimeError('Replay exceeded expected time')
        if player.returncode != 0:
            raise RuntimeError(f'Bag player failed with exit {player.returncode}; inspect playback.log')
        node.spin_for(3)
        if node.latest_map is None:
            raise RuntimeError('SLAM did not publish a map; inspect launch.log and metrics.json')
        result = node.summary()
        save_log = (out/'map_save.log').open('w'); handles.append(save_log)
        saved = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(out/'maps/map'),
                                '-t', '/map', '--free', '0.196', '--occ', '0.65',
                                '--ros-args', '-p', 'map_subscribe_transient_local:=true'],
                               stdout=save_log, stderr=subprocess.STDOUT, timeout=30)
        result['map_save_exit_code'] = saved.returncode
        serialize = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if serialize.wait_for_service(timeout_sec=5):
            request = SerializePoseGraph.Request()
            request.filename = str(out/'maps/posegraph')
            future = serialize.call_async(request)
            end = time.monotonic()+15
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            result['posegraph_save_result'] = future.result().result if future.done() else 'timeout'
        else:
            result['posegraph_save_result'] = 'service_unavailable'
        result['saved_files'] = [path.name for path in (out/'maps').iterdir()]
        if saved.returncode != 0 or not (out/'maps/map.yaml').exists():
            raise RuntimeError('Map saving failed; inspect map_save.log')
        if result.get('posegraph_save_result') != 0 or not all(
                (out / 'maps' / f'posegraph.{extension}').is_file()
                and (out / 'maps' / f'posegraph.{extension}').stat().st_size > 0
                for extension in ('data', 'posegraph')):
            raise RuntimeError('Posegraph serialization failed or files are empty')
        origin = map_origin(node.latest_map)
        result['map_yaml_geometry'] = preserve_map_geometry(
            out / 'maps/map.yaml', node.latest_map.info.resolution,
            [origin[0], origin[1], math.atan2(
                2*(origin[6]*origin[5]+origin[3]*origin[4]),
                1-2*(origin[4]*origin[4]+origin[5]*origin[5]))])
        result['saved_files'] = [path.name for path in (out/'maps').iterdir()]
        reload_log = (out/'map_reload.log').open('w'); handles.append(reload_log)
        loader = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
                                    '-r', '__node:=go2_saved_map_verifier',
                                    '-p', f'yaml_filename:={out / "maps/map.yaml"}',
                                    '-p', 'topic_name:=/restored_map'],
                                   stdout=reload_log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(loader)
        configure = node.create_client(ChangeState, '/go2_saved_map_verifier/change_state')
        change = ChangeState.Request()
        change.transition.id = Transition.TRANSITION_CONFIGURE
        if not node.call(configure, change).success:
            raise RuntimeError('Saved map server configuration failed')
        change.transition.id = Transition.TRANSITION_ACTIVATE
        if not node.call(configure, change).success:
            raise RuntimeError('Saved map server activation failed')
        get_map = node.create_client(GetMap, '/go2_saved_map_verifier/map')
        restored = node.call(get_map, GetMap.Request()).map
        result['map_reload'] = {'width': restored.info.width, 'height': restored.info.height,
                                'resolution_m': restored.info.resolution, 'data_cells': len(restored.data),
                                'origin': map_origin(restored)}
        if (restored.info.width, restored.info.height) != (node.latest_map.info.width, node.latest_map.info.height):
            raise RuntimeError('Reloaded map dimensions differ from saved map')
        restored_cells = Counter(int(value) for value in restored.data)
        result['map_reload']['cells'] = dict(restored_cells)
        if list(restored.data) != list(node.latest_map.data):
            raise RuntimeError('Reloaded map cells differ from saved map')
        result['map_reload']['exact_cell_match'] = True
        if not math.isclose(restored.info.resolution, node.latest_map.info.resolution, rel_tol=0, abs_tol=1e-8):
            raise RuntimeError('Reloaded map resolution differs from saved map')
        if not all(math.isclose(a, b, rel_tol=0, abs_tol=1e-5)
                   for a, b in zip(map_origin(restored), map_origin(node.latest_map))):
            raise RuntimeError('Reloaded map origin differs from saved map')
        result['map_reload']['geometry_matches'] = True
        if not restored_cells[0] or not restored_cells[100]:
            raise RuntimeError('Map lacks confirmed free or occupied cells')
        result['pose_graph_vertices'] = sum(
            marker['namespace'] == 'slam_toolbox' and marker['type'] == 2
            for marker in node.graph_markers)
        reference_file = args.bag.parent/'motion_validation.json'
        if reference_file.exists() and 'slam_pose_change' in result:
            reference = json.loads(reference_file.read_text()).get('odometry_motion', {})
            if 'net_yaw_change_deg' in reference:
                result['motion_consistency'] = {
                    'reference': 'Recorded odometry; this is not independent ground truth',
                    'odometry_yaw_deg': reference['net_yaw_change_deg'],
                    'slam_yaw_deg': result['slam_pose_change']['net_yaw_deg'],
                    'yaw_difference_deg': abs(math.remainder(
                        result['slam_pose_change']['net_yaw_deg']-reference['net_yaw_change_deg'], 360)),
                    'odometry_net_xy_m': reference.get('net_xy_displacement_m'),
                    'slam_net_xy_m': result['slam_pose_change']['net_xy_m']}
        result['geometry_accuracy'] = 'Not independently ground-truth validated'
        result['pipeline_status'] = 'passed'
        result['status'] = 'success'
    except Exception as error:
        result = {**node.summary(), **result, 'status': 'failed', 'error': str(error)}
    finally:
        (out/'metrics.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        with (out / 'estimated_trajectory.csv').open('w') as output:
            writer = csv.writer(output)
            writer.writerow(['aligned_odom_stamp_ns', 'map_x_m', 'map_y_m', 'map_yaw_rad',
                             'odom_x_m', 'odom_y_m', 'odom_yaw_rad', 'odom_stamp_minus_map_tf_stamp_s',
                             'recent_map_tf_for_time_evaluation'])
            writer.writerows(node.trajectory)
        result['trajectory_timing'] = trajectory_timing_summary(out / 'estimated_trajectory.csv')
        for process in reversed(processes):
            stop_process(process)
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.shutdown()
        (out/'metrics.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(f'Output: {out}')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    if result.get('status') != 'success':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
