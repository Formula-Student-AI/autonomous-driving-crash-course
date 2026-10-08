"""Exercise 1: listen to the car.

Subscribe to /odom (nav_msgs/msg/Odometry) and log the car's speed once a second.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        # TODO 1: create a subscription to '/odom' with message type Odometry,
        #         calling self.on_odom, with a queue size of 10.
        #         Hint: self.create_subscription(msg_type, topic, callback, queue_size)

    def on_odom(self, msg: Odometry):
        # TODO 2: compute the speed in m/s from the linear velocity in
        #         msg.twist.twist.linear (x and y components). Hint: math.hypot.
        speed = 0.0

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
