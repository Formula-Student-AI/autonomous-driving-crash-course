"""Fixed-path pure-pursuit driver for EUFS sim2 (FSUK 2023 trackdrive).

Loads fixed_path_centerline.csv (ordered midpoints, start ~ car_start 4,0),
localises from /odom (ground truth, remapped from
/plugin/vehicle_state_plugin/ground_truth/state), and publishes
AckermannDriveStamped on /cmd at 20 Hz.

Run AFTER: /set_mission {mission: 4} + /go
"""
import csv
import math
import os

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


def load_path(csv_path):
    pts = []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            pts.append((float(row['x']), float(row['y'])))
    return pts


def yaw_from_quat(q):
    # yaw only (planar car)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class FixedPathDriver(Node):
    def __init__(self):
        super().__init__('fixed_path_driver')
        self.declare_parameter('path_file', os.path.join(
            os.getcwd(), 'fixed_path_centerline.csv'))
        # fallback: repo root location
        default = '/workspaces/autonomous-driving-crash-course/fixed_path_centerline.csv'
        path_file = self.get_parameter('path_file').value
        if not os.path.isfile(path_file):
            path_file = default
        self.path = load_path(path_file)
        self.get_logger().info(f'Loaded {len(self.path)} centerline points from {path_file}')

        self.declare_parameter('lookahead', 4.0)   # m
        self.declare_parameter('target_speed', 5.0)  # m/s ~ accel-controlled
        self.declare_parameter('wheelbase', 1.58)  # eufs robot; ads-dv use 1.53

        self.x, self.y, self.yaw, self.v = 4.0, 0.0, 0.0, 0.0
        self.have_odom = False

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        self.timer = self.create_timer(0.05, self.step)  # 20 Hz

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.v = msg.twist.twist.linear.x
        self.have_odom = True

    def nearest_idx(self):
        best, bd = 0, 1e18
        for i, (px, py) in enumerate(self.path):
            d = (px - self.x) ** 2 + (py - self.y) ** 2
            if d < bd:
                bd, best = d, i
        return best

    def step(self):
        if not self.have_odom:
            return
        lookahead = self.get_parameter('lookahead').value
        target_speed = self.get_parameter('target_speed').value
        L = self.get_parameter('wheelbase').value

        n = len(self.path)
        i0 = self.nearest_idx()
        # first path point at least `lookahead` ahead
        goal = self.path[i0]
        for k in range(n):
            i = (i0 + k) % n
            px, py = self.path[i]
            if math.hypot(px - self.x, py - self.y) >= lookahead:
                goal = (px, py)
                break

        # transform goal to vehicle frame (+x forward)
        dx, dy = goal[0] - self.x, goal[1] - self.y
        lx = math.cos(-self.yaw) * dx - math.sin(-self.yaw) * dy
        ly = math.sin(-self.yaw) * dx + math.cos(-self.yaw) * dy
        # pure pursuit: curvature = 2*y / Ld^2, steer = atan(k*L)
        steer = math.atan2(2.0 * L * ly, max(lookahead ** 2, 1e-3))
        steer = max(min(steer, 0.5), -0.5)

        # simple P on speed -> accel command
        accel = 2.0 * (target_speed - self.v)
        accel = max(min(accel, 3.0), -5.0)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = float(accel)
        msg.drive.steering_angle = float(steer)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixedPathDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
