#!/usr/bin/env python3
"""Minimal rclpy subscriber probe (useful when the ros2 CLI misbehaves)."""
from __future__ import annotations

import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from ackermann_msgs.msg import AckermannDriveStamped


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--verbose", action="store_true",
                    help="print lateral offset every ~2 s")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("fly_probe")
    counts = {"odom": 0, "clock": 0, "cmd": 0, "status": 0,
              "brain": 0, "body": 0, "retina": 0, "scenery": 0}
    last = {}
    samples = []
    track = None
    try:
        from ament_index_python.packages import get_package_share_directory
        from fly_bridge.track import Track
        share = get_package_share_directory("fly_bridge")
        track = Track(f"{share}/data/trackdrive.csv",
                      f"{share}/data/centerline.csv")
        print("track loaded for lateral checks", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"track unavailable ({type(e).__name__}: {e})", flush=True)

    def on_odom(msg):
        counts["odom"] += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x
        last["odom"] = (x, y, v)
        if track is not None:
            s, lat, _ = track.project(x, y)
            half = track.half_width_at(s)
            last["odom"] = (x, y, v, lat, half)
            if a.verbose and counts["odom"] % 40 == 1:
                print(f"  x={x:7.2f} y={y:7.2f} v={v:4.2f} "
                      f"lat={lat:+5.2f} half={half:4.2f} s={s:6.1f}", flush=True)
        if len(samples) < 5:
            samples.append(last["odom"])

    def on_clock(msg):
        counts["clock"] += 1
        last["clock"] = (msg.clock.sec, msg.clock.nanosec)

    def on_cmd(msg):
        counts["cmd"] += 1
        last["cmd"] = (msg.drive.acceleration, msg.drive.steering_angle)

    def on_status(msg):
        counts["status"] += 1
        last["status"] = msg.data

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    node.create_subscription(Clock, "/clock", on_clock, 10)
    node.create_subscription(AckermannDriveStamped, "/cmd", on_cmd, 10)
    from std_msgs.msg import String
    node.create_subscription(String, "/fly/status", on_status, 10)

    from sensor_msgs.msg import CompressedImage, Image

    def on_brain(msg):
        counts["brain"] += 1
        last["brain"] = (len(msg.data), msg.format)

    def on_body(msg):
        counts["body"] += 1
        last["body"] = (len(msg.data), msg.format)

    def on_retina(msg):
        counts["retina"] += 1
        last["retina"] = (msg.width, msg.height, msg.encoding, len(msg.data))

    node.create_subscription(CompressedImage, "/fly/brain/compressed", on_brain, 1)
    node.create_subscription(CompressedImage, "/fly/body/compressed", on_body, 1)
    node.create_subscription(Image, "/fly/retina", on_retina, 1)

    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import MarkerArray

    def on_scenery(msg):
        counts["scenery"] += 1
        last["scenery"] = (len(msg.markers),
                           sum(len(m.points) for m in msg.markers))

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MarkerArray, "/fly/scenery", on_scenery, latched)
    print("publishers /odom:", node.count_publishers("/odom"),
          " /cmd:", node.count_publishers("/cmd"),
          " /fly/status:", node.count_publishers("/fly/status"), flush=True)

    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("counts:", counts, flush=True)
    print("odom samples:", samples, flush=True)
    for k, v in last.items():
        print(k, "last:", v, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
