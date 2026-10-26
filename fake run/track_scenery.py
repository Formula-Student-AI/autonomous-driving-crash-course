#!/usr/bin/env python3
"""Publish a realistic 3D track scene to Foxglove (main race view).

    python3 "fake run/track_scenery.py"
    python3 "fake run/track_scenery.py" --ros-args -p publish_sky:=true -p trees:=false

Topics:
    /track/scenery   visualization_msgs/MarkerArray (latched, republished 10 s)

Scene: asphalt road with edge lines and a start/finish checker, low barriers
with red rails, real tapered cone meshes (white band + black base), trees, and
an optional sky dome.  This is independent of the fruit-fly side-project.
"""
from __future__ import annotations

import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

from track_geometry import Track
from track_scenery_lib import build_scenery


def default_track_file() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("map_lib"),
            "maps", "competitions", "FSUK", "2023", "trackdrive.csv")
    except Exception:  # noqa: BLE001
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(
            here, "..", "sim", "src", "map_lib", "maps",
            "competitions", "FSUK", "2023", "trackdrive.csv"))


class TrackScenery(Node):
    def __init__(self) -> None:
        super().__init__("track_scenery")
        here = os.path.dirname(os.path.abspath(__file__))
        self.track_file = self.declare_parameter(
            "track_file", default_track_file()).value
        self.centerline_file = self.declare_parameter(
            "centerline_file", os.path.join(here, "fixed_path_centerline.csv")).value
        self.publish_sky = bool(self.declare_parameter("publish_sky", False).value)
        self.trees = bool(self.declare_parameter("trees", True).value)
        self.rate = float(self.declare_parameter("rate", 10.0).value)

        if not os.path.isfile(self.track_file):
            raise FileNotFoundError(
                f"track file not found: {self.track_file}\n"
                "Pass -p track_file:=/path/to/trackdrive.csv")
        self.track = Track(self.track_file, self.centerline_file)
        self.scene = build_scenery(self.track, include_trees=self.trees,
                                   include_sky=self.publish_sky)
        points = sum(len(m.points) for m in self.scene.markers)
        self.get_logger().info(
            f"{self.track.summary()}\n"
            f"scenery: {len(self.scene.markers)} markers / {points} points "
            f"({'sky' if self.publish_sky else 'no sky'}, "
            f"{'trees' if self.trees else 'no trees'})")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/track/scenery", latched)
        self.create_timer(max(1.0, self.rate), self._publish)
        self._publish()

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.scene.markers:
            marker.header.stamp = stamp
        self.pub.publish(self.scene)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackScenery()
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
