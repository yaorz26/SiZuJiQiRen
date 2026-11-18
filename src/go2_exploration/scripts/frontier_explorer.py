#!/usr/bin/env python3
"""Preview frontier goals; explicit operator enable hands one goal at a time to Nav2."""

import json
import math
import signal
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from frontiers import select_goal


class Explorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.clearance = self.declare_parameter('clearance_m', .36).value
        self.deadline_s = self.declare_parameter('goal_timeout_s', 45.).value
        self.limit = self.declare_parameter('max_attempts', 24).value
        if not (self.clearance > 0 and self.deadline_s > 0 and self.limit >= 1):
            raise ValueError('Clearance, timeout and attempt limit must be positive')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_received, qos)
        self.preview = self.create_publisher(PoseStamped, '~/preview', 1)
        self.events = self.create_publisher(String, '~/events', 10)
        self.control = self.create_service(SetBool, '~/enable', self.set_enabled)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.map = None
        self.map_time = -math.inf
        self.enabled = False
        self.active = None
        self.goal_handle = None
        self.cancel_reason = None
        self.visited = []
        self.attempts = 0
        self.next_selection = 0.
        self.no_goal_since = None
        self.timer = self.create_timer(.5, self.tick)

    def event(self, name, **fields):
        msg = String()
        msg.data = json.dumps(dict(event=name, attempt=self.attempts, **fields))
        self.events.publish(msg)
        self.get_logger().info(msg.data)

    def map_received(self, msg):
        self.map, self.map_time = msg, time.monotonic()

    def set_enabled(self, req, resp):
        if req.data and not self.enabled:
            if self.active is not None or not self.nav.server_is_ready():
                resp.success, resp.message = False, 'Waiting for Nav2 or previous goal termination'
                return resp
            self.visited.clear()
            self.attempts = 0
            self.no_goal_since = None
            self.next_selection = 0.
            self.enabled = True
            self.event('enabled')
        elif not req.data:
            self.enabled = False
            self.cancel('disabled')
        resp.success = True
        resp.message = 'Enabled' if self.enabled else 'Disabled; cancellation requested if active'
        return resp

    def cancel(self, reason):
        if self.active is None or self.cancel_reason:
            return
        self.cancel_reason = reason
        self.event('cancel_requested', reason=reason)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.finish('rejected')
                return
            self.goal_handle.get_result_async().add_done_callback(self.result)
            if self.cancel_reason:
                self.goal_handle.cancel_goal_async()
        except Exception as exc:
            self.fault(exc)

    def result(self, future):
        try:
            status = future.result().status
            self.finish('succeeded' if status == GoalStatus.STATUS_SUCCEEDED else f'status_{status}')
        except Exception as exc:
            self.fault(exc)

    def fault(self, exc):
        # Delivery is uncertain. Keep the goal outstanding and prohibit resends.
        self.enabled = False
        self.cancel('action_error')
        self.event('action_error', error=str(exc))

    def finish(self, outcome):
        self.event('goal_finished', outcome=outcome, target=self.active,
                   cancel_reason=self.cancel_reason)
        self.visited.append(self.active[:2])
        self.active = self.goal_handle = self.cancel_reason = None
        self.next_selection = time.monotonic() + 1.5

    def tick(self):
        now = time.monotonic()
        if self.active is None and (now < self.next_selection or self.map is None):
            return
        pose = None
        if now - self.map_time < 5. and self.map is not None:
            try:
                t = self.tf.lookup_transform(self.map.header.frame_id, 'base_link', Time())
                age = (self.get_clock().now().nanoseconds -
                       Time.from_msg(t.header.stamp).nanoseconds) * 1e-9
                if -.5 <= age <= 4.:
                    pose = t.transform.translation
            except TransformException:
                pass
        if self.active is not None:
            if pose is None or now >= self.expiry:
                self.cancel('stale_map_or_tf' if pose is None else 'timeout')
            return
        if pose is None:
            return
        self.next_selection = now + 2.
        if self.enabled and self.attempts >= self.limit:
            self.enabled = False
            self.event('attempt_limit')
            return
        info = self.map.info
        q = info.origin.orientation
        origin = (info.origin.position.x, info.origin.position.y,
                  math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        grid = np.asarray(self.map.data, dtype=np.int8).reshape(info.height, info.width)
        goal = select_goal(grid, info.resolution, origin, (pose.x, pose.y),
                           self.visited, self.clearance)
        if goal is None:
            if self.enabled:
                if self.no_goal_since is None:
                    self.no_goal_since = now
                elif now - self.no_goal_since > 10.:
                    self.enabled = False
                    self.event('no_reachable_frontier')
            return
        self.no_goal_since = None
        p = PoseStamped()
        p.header.frame_id = self.map.header.frame_id
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = goal[:2]
        p.pose.orientation.z = math.sin(goal[2]/2)
        p.pose.orientation.w = math.cos(goal[2]/2)
        self.preview.publish(p)
        if self.enabled and self.nav.server_is_ready():
            self.active, self.expiry = goal, now + self.deadline_s
            self.attempts += 1
            self.event('goal_sent', target=goal)
            request = NavigateToPose.Goal()
            request.pose = p
            try:
                self.nav.send_goal_async(request).add_done_callback(self.accepted)
            except Exception as exc:
                self.fault(exc)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Explorer()
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.2)
    finally:
        node.enabled = False
        if rclpy.ok() and node.active is not None:
            node.cancel('shutdown')
            end = time.monotonic()+3.
            while rclpy.ok() and node.active is not None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
