#!/usr/bin/env python3
"""Operator-armed Nav2 Twist adapter. The default output is a preview topic."""

import json
import math
import signal
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool
from unitree_api.msg import Request
from unitree_go.msg import LowState, WirelessController

MOVE, STOP_MOVE = 1008, 1003


def decode_lowstate_handset(payload):
    """Return the buttons and axes for the 55 51 packet observed on this Go2."""
    if len(payload) != 40:
        raise ValueError('Expected 40 handset bytes')
    head, keys, lx, rx, ry, trigger, ly = struct.unpack_from('<2sH5f', bytes(payload))
    axes = (lx, ly, rx, ry, trigger)
    if head != b'\x55\x51' or not all(math.isfinite(v) and abs(v) <= 1.001 for v in axes):
        raise ValueError('Unrecognised handset packet or invalid axes')
    return keys, axes


class VelocityBridge(Node):
    def __init__(self):
        super().__init__('guarded_velocity_bridge')
        physical = self.declare_parameter('hardware_output', False).value
        source = self.declare_parameter('handset_source', 'wirelesscontroller').value
        if source not in ('wirelesscontroller', 'lowstate_preview'):
            raise ValueError('Unknown handset_source')
        if source == 'lowstate_preview' and physical:
            raise ValueError('LowState physical stopping behavior has not been verified; preview only')
        self.timeout = self.declare_parameter('command_timeout_s', .35).value
        self.linear_limit = self.declare_parameter('linear_limit_m_s', .20).value
        self.yaw_limit = self.declare_parameter('yaw_limit_rad_s', .40).value
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.timeout, self.linear_limit, self.yaw_limit)):
            raise ValueError('Command timeout and velocity limits must be finite and positive')
        output = '/api/sport/request' if physical else '~/preview_sport_request'
        if not physical and self.resolve_topic_name(output).endswith('/api/sport/request'):
            raise ValueError('Preview cannot be remapped to the Sport API')
        self.tx = self.create_publisher(Request, output, 1)
        self.cmd_topic = '/go2/navigation/cmd_vel'
        self.cmd_sub = None
        self.odom_sub = self.create_subscription(
            Odometry, '/utlidar/robot_odom', self.odometry, qos_profile_sensor_data)
        self.lowstate_tick = None
        if source == 'lowstate_preview':
            self.rc_sub = self.create_subscription(
                LowState, '/lf/lowstate', self.lowstate_handset, qos_profile_sensor_data)
        else:
            self.rc_sub = self.create_subscription(
                WirelessController, '/wirelesscontroller', self.handset, qos_profile_sensor_data)
        self.arm_srv = self.create_service(SetBool, '~/arm', self.arm)
        self.timer = self.create_timer(.05, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.armed = False
        self.last_cmd = None
        self.armed_at = -math.inf
        self.last_odom = self.last_remote = -math.inf
        self.neutral_since = None
        self.get_logger().info(f'Disarmed. Handset: {self.rc_sub.topic_name}; output: {self.tx.topic_name}')

    def odometry(self, _msg):
        self.last_odom = time.monotonic()

    def handset(self, msg):
        self.observe_handset(msg.keys, (msg.lx, msg.ly, msg.rx, msg.ry))

    def lowstate_handset(self, msg):
        try:
            keys, axes = decode_lowstate_handset(msg.wireless_remote)
        except ValueError:
            self.neutral_since = None
            self.last_remote = -math.inf
            self.disarm('invalid LowState handset packet')
            return
        tick = int(msg.tick)
        prev = self.lowstate_tick
        self.lowstate_tick = tick
        if prev is not None and not (0 < (tick-prev) % 2**32 < 2**31):
            self.neutral_since = None
            self.last_remote = -math.inf
            self.disarm('duplicate or out-of-order LowState tick')
            return
        self.observe_handset(keys, axes)

    def observe_handset(self, keys, axes):
        now = time.monotonic()
        if now-self.last_remote >= .5:
            self.neutral_since = None
            self.disarm('handset signal gap')
        self.last_remote = now
        if keys or not all(math.isfinite(v) and abs(v) < .10 for v in axes):
            self.neutral_since = None
            self.disarm('manual input')
        elif self.neutral_since is None:
            self.neutral_since = self.last_remote

    def inputs_ready(self, now):
        return (now-self.last_odom < .5 and now-self.last_remote < .5 and
                self.neutral_since is not None and now-self.neutral_since >= .5 and
                self.count_publishers(self.odom_sub.topic_name) == 1 and
                self.count_publishers(self.rc_sub.topic_name) == 1)

    def arm(self, req, resp):
        if not req.data:
            self.disarm('operator')
        elif not self.inputs_ready(time.monotonic()) or self.count_publishers(self.cmd_topic) != 1:
            self.disarm('inputs unavailable')
            resp.success = False
            resp.message = 'Need one velocity source, fresh odometry and a neutral paired handset'
            return resp
        elif not self.armed:
            # Volatile subscription created on arm; queued pre-arm commands
            # cannot become the first command of this session.
            self.cmd_sub = self.create_subscription(Twist, self.cmd_topic, self.command, 1)
            self.armed = True
            self.armed_at = time.monotonic()
            self.last_cmd = None
        resp.success = True
        resp.message = 'armed' if self.armed else 'disarmed'
        return resp

    def send(self, api_id, parameters='{}'):
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = parameters
        self.tx.publish(req)

    def disarm(self, reason):
        if self.armed:
            self.armed = False
            self.last_cmd = None
            self.send(STOP_MOVE)
            self.destroy_subscription(self.cmd_sub)
            self.cmd_sub = None
            self.get_logger().info('Disarmed: '+reason)

    def watchdog(self):
        if self.armed:
            now = time.monotonic()
            if not self.inputs_ready(now) or self.count_publishers(self.cmd_topic) != 1:
                self.disarm('input lost or ambiguous')
            elif self.last_cmd is None and now-self.armed_at > 5.:
                self.disarm('no initial velocity command')
            elif self.last_cmd is not None and now-self.last_cmd > self.timeout:
                self.disarm('velocity command expired')

    def command(self, msg):
        if not self.armed:
            return
        now = time.monotonic()
        if not self.inputs_ready(now) or self.count_publishers(self.cmd_topic) != 1:
            self.disarm('input lost or ambiguous')
            return
        if ((self.last_cmd is not None and now-self.last_cmd > self.timeout) or
                (self.last_cmd is None and now-self.armed_at > 5.)):
            self.disarm('velocity command expired')
            return
        vx, wz = msg.linear.x, msg.angular.z
        if (not math.isfinite(vx) or not math.isfinite(wz) or
                any(v != 0. for v in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y))):
            self.disarm('invalid or unsupported velocity')
            return
        self.last_cmd = now
        if vx == 0. and wz == 0.:
            self.send(STOP_MOVE)
        else:
            vx = max(-self.linear_limit, min(self.linear_limit, vx))
            wz = max(-self.yaw_limit, min(self.yaw_limit, wz))
            self.send(MOVE, json.dumps(dict(x=vx, y=0., z=wz)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityBridge()
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if rclpy.ok():
            node.disarm('process exit')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
