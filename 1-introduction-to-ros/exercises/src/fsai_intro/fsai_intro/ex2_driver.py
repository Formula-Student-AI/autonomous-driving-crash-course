"""Exercise 2: drive the car.

Publish an AckermannDriveStamped command on /cmd at 10 Hz.
"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        # TODO 1: create a publisher on '/cmd' for AckermannDriveStamped, queue size 10.
        #         Hint: self.create_publisher(msg_type, topic, queue_size)

        # TODO 2: create a timer that calls self.send_command every 0.1 s (10 Hz).
        #         Hint: self.create_timer(period_seconds, callback)

    def send_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        # TODO 3: set msg.drive.acceleration (m/s^2) and msg.drive.steering_angle (radians).
        #         Start with acceleration 2.0 and steering 0.2.

        # TODO 4: publish msg.


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
