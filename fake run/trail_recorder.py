#!/usr/bin/env python3
"""Record the car's actual path: live Foxglove trail + CSV diagnostics.

    python3 "fake run/trail_recorder.py"

Topics:
    /race/trail   visualization_msgs/MarkerArray (LINE_STRIP, green)

CSV (default `fake run/out/race_trail_<timestamp>.csv`):
    t_sim,t_wall,x,y,yaw,v,s,lateral,half_width,limit

The lateral/limit columns use the same track geometry as the driver, so you can
see whether a corner of the car left the cone corridor.  The trail is capped at
20,000 points.
"""
from __future__ import annotations

import csv
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from track_geometry import Track


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


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class TrailRecorder(Node):
    def __init__(self) -> None:
        super().__init__("trail_recorder")
        here = os.path.dirname(os.path.abspath(__file__))
        self.track_file = str(self.declare_parameter(
            "track_file", default_track_file()).value)
        self.centerline_file = str(self.declare_parameter(
            "centerline_file", os.path.join(here, "fixed_path_centerline.csv")).value)
        self.car_half_width = float(self.declare_parameter(
            "car_half_width", 0.75).value)
        self.wall_margin = float(self.declare_parameter(
            "wall_margin", 0.35).value)
        self.publish_rate = float(self.declare_parameter(
            "publish_rate", 10.0).value)
        csv_path = str(self.declare_parameter(
            "csv_path",
            os.path.join(here, "out",
                         f"race_trail_{time.strftime('%Y%m%d_%H%M%S')}.csv")).value)
        self.csv_path = csv_path

        self.track = Track(self.track_file, self.centerline_file)
        self.trail: list[tuple[float, float]] = []
        self.lap = 0
        self._last_s: float | None = None
        self._reset_marker = False

        self.csv_file = None
        self.csv = None
        if csv_path and csv_path.lower() not in ("none", "off", ""):
            os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
            self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(["lap", "t_sim", "t_wall", "x", "y", "yaw", "v",
                               "s", "lateral", "half_width", "limit"])
            self.get_logger().info(f"trail CSV -> {csv_path}")
        else:
            self.get_logger().info("trail CSV disabled")

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.pub = self.create_publisher(MarkerArray, "/race/trail", 1)
        self.create_timer(1.0 / max(1.0, self.publish_rate), self.publish)
        self.get_logger().info("trail -> /race/trail (green line)")

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        x, y = float(p.x), float(p.y)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        v = float(msg.twist.twist.linear.x)
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        s, lat, _ = self.track.project(x, y)
        half = self.track.half_width_at(s)
        limit = max(0.35, half - self.car_half_width - self.wall_margin)

        # New lap: clear the green trail when crossing the start line.
        if self._last_s is not None and s < self._last_s - self.track.length * 0.5:
            self.lap += 1
            self.trail.clear()
            self._reset_marker = True
            self.get_logger().info(
                f"start line crossed: trail reset (lap {self.lap})")
        self._last_s = s

        self.trail.append((x, y))
        if len(self.trail) > 20000:
            del self.trail[:1000]

        if self.csv is not None:
            self.csv.writerow([self.lap, f"{t_sim:.3f}", f"{time.time():.3f}",
                               f"{x:.3f}", f"{y:.3f}", f"{yaw:.4f}", f"{v:.3f}",
                               f"{s:.3f}", f"{lat:.3f}", f"{half:.3f}",
                               f"{limit:.3f}"])
            if len(self.trail) % 20 == 0:
                self.csv_file.flush()

    def publish(self) -> None:
        if self._reset_marker:
            clear = Marker()
            clear.header.frame_id = "map"
            clear.header.stamp = self.get_clock().now().to_msg()
            clear.ns = "race_trail"
            clear.id = 0
            clear.action = Marker.DELETE
            self.pub.publish(MarkerArray(markers=[clear]))
            self._reset_marker = False
        if not self.trail:
            return
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "race_trail"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.3, 0.9
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(x), y=float(y), z=0.05)
                    for x, y in self.trail]
        self.pub.publish(MarkerArray(markers=[m]))

    def destroy_node(self) -> bool:
        try:
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrailRecorder()
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
