#!/usr/bin/env python3
"""Save a fixed live map snapshot and verify reloading; publish no motion requests."""

import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from slam_toolbox.srv import SerializePoseGraph
from visualization_msgs.msg import MarkerArray

from slam_artifacts import preserve_map_geometry


class SnapshotSaver(Node):
    def __init__(self):
        super().__init__('go2_live_snapshot_saver')
        self.latest_map = None
        self.diagnostics = {}
        self.vertices = None
        self.map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, self.map_qos)
        for topic in ('/go2/slam/time_diagnostics', '/go2/slam/scan_diagnostics'):
            self.create_subscription(DiagnosticArray, topic, self.diag_received, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization',
                                 self.graph_received, qos_profile_sensor_data)
        self.publisher = self.create_publisher(OccupancyGrid, '/go2/slam/save_snapshot', self.map_qos)

    def map_received(self, message):
        self.latest_map = message

    def diag_received(self, message):
        for status in message.status:
            self.diagnostics[status.name] = {value.key: value.value for value in status.values}

    def graph_received(self, message):
        self.vertices = sum(marker.ns == 'slam_toolbox' and marker.type == 2 for marker in message.markers)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done():
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        return future.result()


def geometry(message):
    p, q = message.info.origin.position, message.info.origin.orientation
    return [message.info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    subprocess.run(['python3', str(workspace / 'scripts/check_external_storage.py'),
                    '--root', str(args.output_root), '--probe'], check=True)
    out = args.output_root / ('verified_snapshot_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H%M%S_%f'))
    (out / 'maps').mkdir(parents=True)
    shutil.copy2(__file__, out / 'save_live_slam.py')
    shutil.copy2(workspace / 'scripts/slam_artifacts.py', out / 'slam_artifacts.py')
    processes, logs = [], []
    result = {'status': 'failed', 'snapshot_method': 'Copy one received OccupancyGrid; publish only that fixed grid to a separate save topic',
              'geometry_accuracy': 'Not independently ground-truth validated',
              'posegraph_timing': 'Serialized after grid capture; these are not an atomic combined checkpoint'}
    rclpy.init()
    node = SnapshotSaver()
    try:
        node.spin_for(5)
        if node.latest_map is None:
            raise RuntimeError('Live /map has not been received')
        snapshot = deepcopy(node.latest_map)
        result['captured_at'] = datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()
        result['map'] = {'width': snapshot.info.width, 'height': snapshot.info.height,
                         'geometry': geometry(snapshot), 'cells': dict(Counter(snapshot.data))}
        if not result['map']['cells'].get(0) or not result['map']['cells'].get(100):
            raise RuntimeError('Snapshot has no free or occupied evidence')
        np.savez_compressed(out / 'snapshot_grid.npz',
                            grid=np.asarray(snapshot.data).reshape(snapshot.info.height, snapshot.info.width),
                            resolution=snapshot.info.resolution,
                            origin=[snapshot.info.origin.position.x, snapshot.info.origin.position.y])
        node.publisher.publish(snapshot)
        log = (out / 'map_save.log').open('w'); logs.append(log)
        saver = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                                   '-f', str(out / 'maps/map'), '-t', '/go2/slam/save_snapshot',
                                   '--free', '0.196', '--occ', '0.65', '--ros-args',
                                   '-p', 'map_subscribe_transient_local:=true'],
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(saver)
        deadline = time.monotonic() + 20
        while saver.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if saver.poll() != 0:
            raise RuntimeError('Snapshot map saver failed or timed out')
        p, q = snapshot.info.origin.position, snapshot.info.origin.orientation
        result['map_yaml_geometry'] = preserve_map_geometry(out / 'maps/map.yaml', snapshot.info.resolution,
            [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])
        serialize = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        request = SerializePoseGraph.Request(); request.filename = str(out / 'maps/posegraph')
        result['posegraph_save_result'] = node.call(serialize, request).result
        if result['posegraph_save_result'] != 0 or not all(
                (out / 'maps' / f'posegraph.{extension}').is_file()
                and (out / 'maps' / f'posegraph.{extension}').stat().st_size > 0
                for extension in ('data', 'posegraph')):
            raise RuntimeError('Live posegraph serialization failed')
        log = (out / 'map_reload.log').open('w'); logs.append(log)
        loader = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
                                    '-r', '__node:=go2_live_saved_map_verifier',
                                    '-p', f'yaml_filename:={out / "maps/map.yaml"}',
                                    '-p', 'topic_name:=/go2/slam/restored_snapshot'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(loader)
        lifecycle = node.create_client(ChangeState, '/go2_live_saved_map_verifier/change_state')
        request = ChangeState.Request(); request.transition.id = Transition.TRANSITION_CONFIGURE
        if not node.call(lifecycle, request).success:
            raise RuntimeError('Saved map configuration failed')
        request.transition.id = Transition.TRANSITION_ACTIVATE
        if not node.call(lifecycle, request).success:
            raise RuntimeError('Saved map activation failed')
        restored = node.call(node.create_client(GetMap, '/go2_live_saved_map_verifier/map'), GetMap.Request()).map
        cells_match = list(restored.data) == list(snapshot.data)
        dimensions_match = (restored.info.width, restored.info.height) == (snapshot.info.width, snapshot.info.height)
        geometry_match = all(math.isclose(a, b, rel_tol=0, abs_tol=1e-5)
                             for a, b in zip(geometry(restored), geometry(snapshot)))
        result['map_reload'] = {'exact_cell_match': cells_match, 'dimensions_match': dimensions_match,
                                'geometry_matches': geometry_match}
        if not all((cells_match, dimensions_match, geometry_match)):
            raise RuntimeError('Reloaded snapshot differs from the captured grid')
        result['graph_vertices'] = node.vertices
        result['diagnostics'] = node.diagnostics
        result['status'] = 'success'
    except Exception as error:
        result['error'] = str(error)
    finally:
        (out / 'live_verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        for process in reversed(processes):
            for sig, wait in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                if process.poll() is not None:
                    break
                try:
                    os.killpg(process.pid, sig); process.wait(timeout=wait)
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
        for log in logs:
            log.close()
        print('Live snapshot verification:', out, flush=True)
        print(json.dumps(result, indent=2), flush=True)
        if Path('/sys/class/net/enp2s0/carrier').read_text().strip() != '1':
            os._exit(1 if result['status'] != 'success' else 0)
        node.destroy_node(); rclpy.shutdown()
    if result['status'] != 'success':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
