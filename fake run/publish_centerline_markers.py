"""Publish fixed centerline as Foxglove/RViz MarkerArray (one-shot, latched)."""
import csv
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class CenterlineMarkers(Node):
    def __init__(self):
        super().__init__('centerline_markers')
        pts = []
        with open('/workspaces/autonomous-driving-crash-course/fixed_path_centerline.csv') as f:
            for row in csv.DictReader(f):
                pts.append((float(row['x']), float(row['y'])))
        pub = self.create_publisher(MarkerArray, '/centerline_markers', 1)
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'centerline', 0, Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
        for x, y in pts + [pts[0]]:
            from geometry_msgs.msg import Point
            m.points.append(Point(x=x, y=y, z=0.0))
        arr.markers.append(m)
        # publish a few times so Foxglove latches it
        for _ in range(5):
            pub.publish(arr)
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'Published {len(pts)} centerline points on /centerline_markers')


def main(args=None):
    rclpy.init(args=args)
    CenterlineMarkers().destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
