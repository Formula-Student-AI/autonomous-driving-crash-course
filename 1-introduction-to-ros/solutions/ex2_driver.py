"""Exercise 2 solution. Run directly: python3 ex2_driver.py"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        self.create_timer(0.1, self.send_command)

    def send_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = 2.0
        msg.drive.steering_angle = 0.2
        self.cmd_pub.publish(msg)


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
