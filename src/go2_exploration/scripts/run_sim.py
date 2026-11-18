#!/usr/bin/env python3
"""Run SLAM, Nav2 and a virtual base on a private loopback ROS graph."""

import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from zoneinfo import ZoneInfo

import yaml


def simulation_configs(out):
    from ament_index_python.packages import get_package_share_directory
    nav_share = Path(get_package_share_directory('nav2_bringup'))
    nav = yaml.safe_load((nav_share/'params/nav2_params.yaml').read_text())
    nav['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    for key in ('global_costmap', 'local_costmap'):
        cost = nav[key][key]['ros__parameters']
        cost.update(robot_base_frame='base_link', robot_radius=.30, resolution=.05)
    ctrl = nav['controller_server']['ros__parameters']
    ctrl['general_goal_checker'].update(xy_goal_tolerance=.15, yaw_goal_tolerance=.30)
    ctrl['progress_checker'].update(required_movement_radius=.20, movement_time_allowance=12.)
    ctrl['FollowPath'] = dict(
        plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=.35, lookahead_dist=.45, min_lookahead_dist=.25,
        max_lookahead_dist=.70, use_velocity_scaled_lookahead_dist=False,
        rotate_to_heading_angular_vel=.8, transform_tolerance=.3)
    nav['bt_navigator']['ros__parameters'].update(robot_base_frame='base_link', odom_topic='/odom')
    nav['velocity_smoother']['ros__parameters']['max_velocity'] = [.40, 0., 1.]
    (out/'nav2.yaml').write_text(yaml.safe_dump(nav))

    slam_share = Path(get_package_share_directory('slam_toolbox'))
    slam = yaml.safe_load((slam_share/'config/mapper_params_online_async.yaml').read_text())
    slam['slam_toolbox']['ros__parameters'].update(
        base_frame='base_link', odom_frame='odom', map_frame='map', scan_topic='/scan',
        resolution=.05, map_update_interval=.6, minimum_time_interval=.20,
        minimum_travel_distance=0., minimum_travel_heading=.08, restamp_tf=True,
        distance_variance_penalty=.05, angle_variance_penalty=.02,
        minimum_distance_penalty=.10, minimum_angle_penalty=.10,
        min_laser_range=.12, max_laser_range=2.9, min_pass_through=1, do_loop_closing=False)
    (out/'slam.yaml').write_text(yaml.safe_dump(slam))


def run(out, goals, seconds):
    import numpy as np
    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
    from std_msgs.msg import String, UInt32
    from std_srvs.srv import SetBool

    procs, logs = [], []
    result = dict(mode='simulated planar base, ideal odometry and synthetic 2D lidar',
                  domain=int(os.environ['ROS_DOMAIN_ID']), goals_requested=goals,
                  events=[], known_area_initial_m2=None, known_area_final_m2=None,
                  blocked_steps=None, goals_reached=0, distance_travelled_m=0.,
                  max_endpoint_error_m=.30, initial_map_from_odom=None, status='failed')
    latest = None
    previous = None
    def spawn(name, args):
        log = (out/(name+'.log')).open('w')
        logs.append(log)
        proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        procs.append((name, proc))

    def map_cb(msg):
        nonlocal latest
        latest = msg
        result['known_area_final_m2'] = int(np.count_nonzero(np.asarray(msg.data) >= 0))*msg.info.resolution**2

    def odom_cb(msg):
        nonlocal previous
        p = msg.pose.pose.position
        if previous is not None:
            result['distance_travelled_m'] += math.hypot(p.x-previous[0], p.y-previous[1])
        previous = (p.x, p.y)

    def event_cb(msg):
        e = json.loads(msg.data)
        e.update(elapsed_s=round(time.monotonic()-start_time, 2),
                 known_area_m2=result['known_area_final_m2'],
                 sim_position_xy=previous)
        result['events'].append(e)
        if e['event'] == 'goal_finished' and e['outcome'] == 'succeeded':
            tx, ty, angle = result['initial_map_from_odom']
            x, y = previous
            truth = (tx+math.cos(angle)*x-math.sin(angle)*y,
                     ty+math.sin(angle)*x+math.cos(angle)*y)
            e['endpoint_error_m'] = math.dist(truth, e['target'][:2])
            if e['endpoint_error_m'] <= result['max_endpoint_error_m']:
                result['goals_reached'] += 1
        print(msg.data, flush=True)

    simulation_configs(out)
    start_time = time.monotonic()
    rclpy.init()
    node = rclpy.create_node('exploration_sim_observer')
    tf = Buffer()
    listener = TransformListener(tf, node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, 'map', map_cb, qos)
    node.create_subscription(Odometry, 'odom', odom_cb, 10)
    node.create_subscription(String, '/frontier_explorer/events', event_cb, 10)
    node.create_subscription(UInt32, '/sim/blocked_steps',
                             lambda m: result.update(blocked_steps=int(m.data)), 10)
    start = node.create_client(SetBool, '/frontier_explorer/enable')
    state = node.create_client(GetState, '/bt_navigator/get_state')
    try:
        spawn('sim', ['ros2', 'run', 'go2_exploration', 'indoor_sim.py'])
        spawn('slam', ['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node', '--ros-args',
                       '-r', '__node:=slam_toolbox', '--params-file', str(out/'slam.yaml')])
        spawn('navigation', ['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                             'use_sim_time:=false', 'use_composition:=False', 'autostart:=true',
                             'params_file:='+str(out/'nav2.yaml')])
        spawn('explorer', ['ros2', 'run', 'go2_exploration', 'frontier_explorer.py'])
        deadline = time.monotonic() + seconds
        enabled = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            dead = [name for name, proc in procs if proc.poll() is not None]
            if dead:
                raise RuntimeError('Process exited: '+', '.join(dead))
            if result['blocked_steps']:
                raise RuntimeError('Virtual footprint contacted an obstacle')
            if any(e.get('endpoint_error_m', 0) > result['max_endpoint_error_m']
                   for e in result['events']):
                raise RuntimeError('Nav2 reported success outside simulation endpoint tolerance')
            if result['goals_reached'] >= goals:
                break
            if any(e['event'] in ('no_reachable_frontier', 'attempt_limit', 'action_error')
                   for e in result['events']):
                raise RuntimeError('Explorer stopped before reaching the requested goals')
            if enabled or latest is None or not (state.service_is_ready() and start.service_is_ready()):
                continue
            sf = state.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, sf, timeout_sec=1.)
            if not sf.done() or sf.result().current_state.label != 'active':
                continue
            if previous is None:
                continue
            try:
                initial_tf = tf.lookup_transform('map', 'odom', Time()).transform
            except TransformException:
                continue
            q = initial_tf.rotation
            result['initial_map_from_odom'] = [initial_tf.translation.x, initial_tf.translation.y,
                                               math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            np.savez_compressed(out/'initial_grid.npz',
                                grid=np.asarray(latest.data, dtype=np.int8).reshape(latest.info.height, latest.info.width),
                                resolution=latest.info.resolution,
                                origin=[latest.info.origin.position.x, latest.info.origin.position.y,
                                        2*math.atan2(latest.info.origin.orientation.z, latest.info.origin.orientation.w)])
            result['known_area_initial_m2'] = result['known_area_final_m2']
            f = start.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(node, f, timeout_sec=2.)
            if not f.done():
                raise RuntimeError('Timed out enabling simulated explorer')
            enabled = f.result().success
        if result['goals_reached'] < goals:
            raise RuntimeError('Simulation deadline reached')
        # Stop selecting goals, then allow the final map update to arrive.
        f = start.call_async(SetBool.Request(data=False))
        rclpy.spin_until_future_complete(node, f, timeout_sec=2.)
        end = time.monotonic()+2.
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        area_gain = result['known_area_final_m2']-result['known_area_initial_m2']
        if result['blocked_steps'] != 0 or result['distance_travelled_m'] < .5 or area_gain < 1.0:
            raise RuntimeError('Insufficient map growth, travel or collision evidence')
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(out/'map'),
                        '-t', '/map', '--occ', '.65', '--free', '.196'], check=True, timeout=15)
        if not all((out/f).is_file() for f in ('map.pgm', 'map.yaml')):
            raise RuntimeError('Map output missing')
        result['status'] = 'passed'
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or type(exc).__name__
    finally:
        for _, proc in reversed(procs):
            for sig, wait in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
                if proc.poll() is not None:
                    break
                try:
                    os.killpg(proc.pid, sig)
                    proc.wait(timeout=wait)
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
        for log in logs:
            log.close()
        if latest is not None:
            grid = np.asarray(latest.data, dtype=np.int8).reshape(latest.info.height, latest.info.width)
            origin = latest.info.origin
            np.savez_compressed(out/'last_grid.npz', grid=grid, resolution=latest.info.resolution,
                                origin=[origin.position.x, origin.position.y,
                                        2*math.atan2(origin.orientation.z, origin.orientation.w)])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (out/'result.json').write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goals', type=int, default=3)
    parser.add_argument('--seconds', type=float, default=180.)
    parser.add_argument('--output-root', type=Path)
    args = parser.parse_args()
    if not (args.goals > 0 and math.isfinite(args.seconds) and args.seconds > 0):
        parser.error('Goals and timeout must be positive')
    mount = Path('/media/yaorz/Elements')
    root = (args.output_root or mount/'unitree_go2/simulation/frontier').resolve()
    if (root == mount or mount in root.parents) and not mount.is_mount():
        parser.error('Elements is not mounted')
    # Configure isolation before any ROS imports or child nodes are started.
    # CycloneDDS selects lo explicitly. Humble's ROS_LOCALHOST_ONLY=1 would
    # add the same interface a second time and prevent domain creation.
    os.environ.update(ROS_DOMAIN_ID='83', ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    with (Path(tempfile.gettempdir())/'go2-exploration-sim-83.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('Another exploration simulation is using domain 83')
        stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d_%H%M%S_%f')
        out = root/stamp
        out.mkdir(parents=True)
        snapshot = out/'source'
        snapshot.mkdir()
        for script in ('run_sim.py', 'indoor_sim.py', 'frontier_explorer.py', 'frontiers.py'):
            shutil.copy2(Path(__file__).resolve().with_name(script), snapshot/script)
        os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
        print('Simulation output:', out, flush=True)
        try:
            result = run(out, args.goals, args.seconds)
        except (Exception, KeyboardInterrupt) as exc:
            result = dict(status='startup_failed', error=str(exc) or type(exc).__name__)
            (out/'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items() if k != 'events'}), flush=True)
        return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
