"""Exercise 1 solution. Run directly: python3 ex1_listener.py"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        self.get_logger().info(f'speed: {speed:.2f} m/s', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
