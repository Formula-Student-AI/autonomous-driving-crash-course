"""Exercise 5: see what the car sees.

Subscribe to /cones (eufs_msgs/msg/ConeWithColorProbabilityArray) and log the
nearest cone: how far away it is, and what colour it most likely is.
"""
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
    # TODO 1: return the key with the largest value. Hint: max(probabilities, key=probabilities.get)
    return 'unknown'


class NearestCone(Node):
    def __init__(self):
        super().__init__('nearest_cone')
        # TODO 2: subscribe to '/cones' with type ConeWithColorProbabilityArray,
        #         calling self.on_cones, queue size 10.

    def on_cones(self, msg: ConeWithColorProbabilityArray):
        if not msg.cones:
            return

        # Cone positions are relative to the car (the car is at x=0, y=0, x pointing forward),
        # so a cone's distance is just the length of its (x, y) position.
        # TODO 3: find the cone in msg.cones with the smallest distance
        #         math.hypot(cone.point.x, cone.point.y), and store it in `nearest`.
        nearest = msg.cones[0]

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
