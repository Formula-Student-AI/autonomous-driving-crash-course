"""Exercise 5 solution. Run directly: python3 ex5_nearest_cone.py"""
import math

import rclpy
from rclpy.node import Node
from eufs_msgs.msg import ConeWithColorProbabilityArray


def most_likely_colour(cone):
    """Return the name of the colour with the highest probability for this cone."""
    probabilities = {
        'blue': cone.blue_prob,
        'yellow': cone.yellow_prob,
        'orange': cone.orange_prob,
        'big orange': cone.big_orange_prob,
        'unknown': cone.unknown_prob,
    }
    return max(probabilities, key=probabilities.get)


class NearestCone(Node):
    def __init__(self):
        super().__init__('nearest_cone')
        self.create_subscription(ConeWithColorProbabilityArray, '/cones', self.on_cones, 10)

    def on_cones(self, msg: ConeWithColorProbabilityArray):
        if not msg.cones:
            return

        nearest = min(msg.cones, key=lambda cone: math.hypot(cone.point.x, cone.point.y))
        distance = math.hypot(nearest.point.x, nearest.point.y)
        self.get_logger().info(
            f'{len(msg.cones)} cones visible; nearest is {distance:.1f} m away, '
            f'most likely {most_likely_colour(nearest)}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = NearestCone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
