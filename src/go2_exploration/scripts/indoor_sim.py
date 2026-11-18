#!/usr/bin/env python3
"""Planar indoor LiDAR and odometry for a simulated mobile base."""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt32
from tf2_ros import TransformBroadcaster


class IndoorSim(Node):
    # Only the sensor/collision simulator can read the scene geometry.
    bounds = (-1.8, -1.8, 7.0, 2.3)
    blocks = ((1.7, -.8, 2.15, -.35), (3.3, .3, 3.7, 2.3), (5., -1.8, 5.4, -.6))
    radius = .30

    def __init__(self):
        super().__init__('indoor_sim')
        self.x = self.y = self.yaw = 0.
        self.cmd = Twist()
        self.received = -math.inf
        self.last_tick = time.monotonic()
        self.hits = 0
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.lidar = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.contacts = self.create_publisher(UInt32, 'sim/blocked_steps', 10)
        self.tf = TransformBroadcaster(self)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.command, 1)
        self.timer = self.create_timer(.05, self.tick)
        self.scan_timer = self.create_timer(.10, self.scan)

    def command(self, msg):
        self.cmd, self.received = msg, time.monotonic()

    def valid(self, x, y):
        x0, y0, x1, y1 = self.bounds
        if min(x-x0, x1-x, y-y0, y1-y) <= self.radius:
            return False
        for x0, y0, x1, y1 in self.blocks:
            if math.hypot(max(x0-x, 0., x-x1), max(y0-y, 0., y-y1)) <= self.radius:
                return False
        return True

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(now-self.last_tick, .15), now
        v = float(np.clip(self.cmd.linear.x, -.45, .45))
        w = float(np.clip(self.cmd.angular.z, -1.5, 1.5))
        if now-self.received > .45 or not math.isfinite(v+w):
            v = w = 0.
        mid = self.yaw + .5*w*dt
        nx, ny = self.x + dt*v*math.cos(mid), self.y + dt*v*math.sin(mid)
        if self.valid(nx, ny):
            self.x, self.y = nx, ny
        else:
            self.hits += 1
            v = 0.
        self.yaw += w*dt
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        self.tf.sendTransform(t)
        o = Odometry()
        o.header, o.child_frame_id = t.header, t.child_frame_id
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        self.odom_pub.publish(o)
        self.contacts.publish(UInt32(data=self.hits))

    def scan(self):
        count, maximum = 360, 3.0
        step = 2*math.pi/count
        angles = self.yaw - math.pi + step*np.arange(count)
        ux, uy = np.cos(angles), np.sin(angles)
        dist = np.full(count, np.inf)
        for x0, y0, x1, y1 in (self.bounds, *self.blocks):
            for edge in (x0, x1):
                t = np.divide(edge-self.x, ux, out=np.full(count, -1.), where=np.abs(ux)>1e-9)
                y = self.y + t*uy
                valid = (t > 0) & (y >= y0) & (y <= y1)
                dist[valid] = np.minimum(dist[valid], t[valid])
            for edge in (y0, y1):
                t = np.divide(edge-self.y, uy, out=np.full(count, -1.), where=np.abs(uy)>1e-9)
                x = self.x + t*ux
                valid = (t > 0) & (x >= x0) & (x <= x1)
                dist[valid] = np.minimum(dist[valid], t[valid])
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, -math.pi+(count-1)*step, step
        scan.range_min, scan.range_max = .12, maximum+.05
        scan.scan_time = .1
        # No reflector in range: do not give scan matching an artificial
        # ring of endpoints that moves with the robot through empty space.
        scan.ranges = np.where(dist <= maximum, dist, np.inf).astype(np.float32).tolist()
        self.lidar.publish(scan)


def main():
    rclpy.init()
    node = IndoorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
