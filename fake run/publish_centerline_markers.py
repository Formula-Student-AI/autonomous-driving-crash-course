#!/usr/bin/env python3
"""Publish the fixed centreline as a periodic, latched MarkerArray.

    python3 "fake run/publish_centerline_markers.py"

Topic: /centerline_markers (visualization_msgs/MarkerArray, red LINE_STRIP)

The same centreline is also drawn inside /track/scenery as the marker
namespace `scenery/centerline`; this script is for a separate topic/panel.
"""
from __future__ import annotations

import csv
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "fixed_path_centerline.csv")


class CenterlineMarkers(Node):
    def __init__(self) -> None:
        super().__init__("centerline_markers")
        self.path_file = str(self.declare_parameter(
            "path_file", DEFAULT_PATH).value)
        pts = []
        with open(self.path_file) as f:
            for row in csv.DictReader(f):
                pts.append((float(row["x"]), float(row["y"])))
        if len(pts) < 2:
            raise ValueError(f"not enough centreline points in {self.path_file}")
        self.pts = pts + [pts[0]]

        m = Marker()
        m.header.frame_id = "map"
        m.ns = "centerline"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.10
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.15, 0.15, 1.0
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(x), y=float(y), z=0.04) for x, y in self.pts]
        self.arr = MarkerArray(markers=[m])

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/centerline_markers", qos)
        self.create_timer(2.0, self.publish)
        self.publish()
        self.get_logger().info(
            f"centerline: {len(self.pts)-1} points from {self.path_file} "
            f"-> /centerline_markers")

    def publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for m in self.arr.markers:
            m.header.stamp = stamp
        self.pub.publish(self.arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CenterlineMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
