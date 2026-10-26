#!/usr/bin/env python3
"""Probe the main race stack: odom, /cmd, mux mode, cone penalty, scenery.

    python3 "fake run/race_probe.py" --seconds 20 --verbose
"""
from __future__ import annotations

import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Int32, String
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import DurabilityPolicy, QoSProfile


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("race_probe")
    counts = {"odom": 0, "cmd": 0, "hits": 0, "penalty": 0,
              "lap": 0, "mode": 0, "scenery": 0, "trail": 0, "camera": 0,
              "caminfo": 0}
    last = {}

    def on_odom(msg: Odometry):
        counts["odom"] += 1
        p = msg.pose.pose.position
        last["odom"] = (p.x, p.y, msg.twist.twist.linear.x)
        if a.verbose and counts["odom"] % 40 == 1:
            print(f"  x={p.x:7.2f} y={p.y:7.2f} v={msg.twist.twist.linear.x:4.2f}",
                  flush=True)

    def on_cmd(msg: AckermannDriveStamped):
        counts["cmd"] += 1
        last["cmd"] = (msg.drive.acceleration, msg.drive.steering_angle)

    def on_hits(msg: Int32):
        counts["hits"] += 1
        last["hits"] = msg.data

    def on_penalty(msg: Float32):
        counts["penalty"] += 1
        last["penalty"] = msg.data

    def on_lap(msg: Int32):
        counts["lap"] += 1
        last["lap"] = msg.data

    def on_mode(msg: String):
        counts["mode"] += 1
        last["mode"] = msg.data

    def on_scenery(msg: MarkerArray):
        counts["scenery"] += 1
        last["scenery"] = (len(msg.markers),
                           sum(len(m.points) for m in msg.markers))

    def on_trail(msg: MarkerArray):
        counts["trail"] += 1
        last["trail"] = (len(msg.markers),
                         len(msg.markers[0].points) if msg.markers else 0)

    def on_camera(msg):
        counts["camera"] += 1
        last["camera"] = (msg.width, msg.height, msg.encoding, len(msg.data))

    def on_caminfo(msg: CameraInfo):
        counts["caminfo"] += 1
        last["caminfo"] = (msg.header.frame_id, msg.width, msg.height,
                           list(msg.k)[:3])

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    node.create_subscription(AckermannDriveStamped, "/cmd", on_cmd, 10)
    node.create_subscription(Int32, "/race/cone_hits", on_hits, 10)
    node.create_subscription(Float32, "/race/penalty_s", on_penalty, 10)
    node.create_subscription(Int32, "/race/lap", on_lap, 10)
    node.create_subscription(String, "/control_mode_active", on_mode, 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MarkerArray, "/track/scenery", on_scenery, latched)
    node.create_subscription(MarkerArray, "/race/trail", on_trail, 1)
    node.create_subscription(Image, "/race/camera", on_camera, 1)
    node.create_subscription(CameraInfo, "/race/camera_info", on_caminfo, 1)

    print("publishers /cmd:", node.count_publishers("/cmd"),
          " /race/cmd:", node.count_publishers("/race/cmd"),
          " /track/scenery:", node.count_publishers("/track/scenery"), flush=True)

    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("counts:", counts, flush=True)
    for k, v in last.items():
        print(k, "last:", v, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
