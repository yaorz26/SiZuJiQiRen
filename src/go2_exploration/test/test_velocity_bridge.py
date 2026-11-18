"""Exercise the published SportRequest messages on an isolated ROS graph."""

import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import pytest


@pytest.fixture
def isolated_graph(monkeypatch):
    # Pin both test processes to loopback, independently of the user's shell.
    monkeypatch.setenv('ROS_DOMAIN_ID', '84')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Domain><General><Interfaces>'
                       '<NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast>'
                       '</General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
                       '<Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')


@pytest.mark.parametrize('source', ['wirelesscontroller', 'lowstate_preview'])
def test_preview_clamp_takeover_timeout_and_exit(isolated_graph, source):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_srvs.srv import SetBool
    from unitree_api.msg import Request
    from unitree_go.msg import LowState, WirelessController

    rclpy.init()
    node = rclpy.create_node('velocity_bridge_test')
    odom = node.create_publisher(Odometry, '/utlidar/robot_odom', 1)
    lowstate = source == 'lowstate_preview'
    rc = node.create_publisher(LowState if lowstate else WirelessController,
                               '/lf/lowstate' if lowstate else '/wirelesscontroller', 1)
    cmd = node.create_publisher(Twist, '/go2/navigation/cmd_vel', 1)
    messages = []
    node.create_subscription(Request, '/guarded_velocity_bridge/preview_sport_request',
                             messages.append, 10)
    arm = node.create_client(SetBool, '/guarded_velocity_bridge/arm')
    child = subprocess.Popen([sys.executable, str(Path(__file__).parents[1] /
                                                  'scripts/guarded_velocity_bridge.py'),
                              '--ros-args', '-p', 'handset_source:='+source],
                             env=os.environ.copy())
    speed = Twist()
    speed.linear.x, speed.angular.z = 3., -2.

    tick = 0

    def pump(seconds, request_speed=False, manual=False, rc_online=True, corrupt=False):
        nonlocal tick
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            odom.publish(Odometry())
            if rc_online:
                if lowstate:
                    tick += 1
                    msg = LowState()
                    msg.tick = tick
                    msg.wireless_remote = list(struct.pack(
                        '<2sH5f16x', b'\x00\x00' if corrupt else b'\x55\x51',
                        0, .5 if manual else 0., 0., 0., 0., 0.))
                else:
                    msg = WirelessController(lx=.5 if manual else 0.)
                rc.publish(msg)
            if request_speed:
                cmd.publish(speed)
            rclpy.spin_once(node, timeout_sec=.02)
            time.sleep(.015)

    def enable():
        pump(.8)
        reply = arm.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, reply, timeout_sec=2.)
        assert reply.done() and reply.result().success
        pump(.4, request_speed=True)
        assert messages[-1].header.identity.api_id == 1008
        assert json.loads(messages[-1].parameter) == dict(x=.20, y=0., z=-.40)

    try:
        assert arm.wait_for_service(timeout_sec=8.)
        pump(.6, request_speed=True)
        assert not messages and node.count_publishers('/api/sport/request') == 0
        enable()
        pump(.3, request_speed=True, manual=True)
        assert messages[-1].header.identity.api_id == 1003
        count = len(messages)
        pump(.3, request_speed=True)
        assert len(messages) == count
        enable()
        pump(.65)
        assert messages[-1].header.identity.api_id == 1003
        if lowstate:
            enable()
            pump(.75, request_speed=True, rc_online=False)
            assert messages[-1].header.identity.api_id == 1003
            enable()
            pump(.25, request_speed=True, corrupt=True)
            assert messages[-1].header.identity.api_id == 1003
        enable()
        child.send_signal(signal.SIGTERM)
        pump(.4)
        assert child.wait(timeout=3.) == 0
        assert messages[-1].header.identity.api_id == 1003
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        node.destroy_node()
        rclpy.shutdown()


def test_lowstate_packet_layout():
    sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
    from guarded_velocity_bridge import decode_lowstate_handset
    packet = struct.pack('<2sH5f16x', b'\x55\x51', 0x100, .25, -.5, .125, .75, -.25)
    assert decode_lowstate_handset(packet) == (0x100, (.25, -.25, -.5, .125, .75))
    for invalid in (packet[:-1], bytes(40), packet[:4]+struct.pack('<f', math.nan)+packet[8:]):
        with pytest.raises(ValueError):
            decode_lowstate_handset(invalid)


@pytest.mark.parametrize(('args', 'reason'), [
    (['-p', 'hardware_output:=true'], 'preview only'),
    (['-r', '~/preview_sport_request:=/api/sport/request'], 'Preview cannot be remapped'),
])
def test_lowstate_refuses_physical_output(isolated_graph, args, reason):
    script = Path(__file__).parents[1]/'scripts/guarded_velocity_bridge.py'
    proc = subprocess.run([sys.executable, str(script), '--ros-args',
                           '-p', 'handset_source:=lowstate_preview', *args],
                          capture_output=True, text=True, timeout=8.)
    assert proc.returncode != 0 and reason in proc.stderr
