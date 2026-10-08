"""Exercise 4: close the loop.

Read the car's speed from /odom and command acceleration on /cmd so the car
holds a target speed. This is a proportional controller: the further below the
target speed we are, the harder we accelerate.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

TARGET_SPEED = 5.0  # m/s
KP = 1.0            # proportional gain: m/s^2 of acceleration per m/s of speed error
MAX_ACCEL = 3.0     # m/s^2, keep commands gentle


class SpeedHold(Node):
    def __init__(self):
        super().__init__('speed_hold')
        self.speed = 0.0

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        self.create_timer(0.1, self.control)

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def control(self):
        # TODO 1: compute the speed error: TARGET_SPEED - self.speed.
        # TODO 2: acceleration = KP * error, clamped to [-MAX_ACCEL, MAX_ACCEL].
        acceleration = 0.0

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = acceleration
        msg.drive.steering_angle = 0.0
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'speed {self.speed:.2f} m/s, accel cmd {acceleration:.2f}', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
