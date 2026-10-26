#!/usr/bin/env python3
"""Print /map and /cones frames and the first cone positions.

    python3 "fake run/map_probe.py" --seconds 5

/map is eufs_msgs/ConeWithColorProbabilityArray, TRANSIENT_LOCAL; /cones is the
same type, VOLATILE (car frame).  Compare the positions with
fixed_path_centerline.csv / trackdrive.csv.
"""
from __future__ import annotations

import argparse
import time

import rclpy
from eufs_msgs.msg import ConeWithColorProbabilityArray
from rclpy.qos import DurabilityPolicy, QoSProfile


def dominant(cone) -> str:
    probs = {
        "blue": cone.blue_prob,
        "yellow": cone.yellow_prob,
        "orange": cone.orange_prob,
        "big_orange": cone.big_orange_prob,
    }
    return max(probs, key=probs.get)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("map_probe")
    seen = set()

    def show(label, msg):
        if label in seen:
            return
        seen.add(label)
        print(f"[{label}] frame={msg.header.frame_id!r} n={len(msg.cones)}")
        for c in msg.cones[:8]:
            print(f"  {dominant(c):10s} ({c.point.x:7.2f}, {c.point.y:7.2f})")

    node.create_subscription(
        ConeWithColorProbabilityArray, "/map",
        lambda m: show("map", m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(
        ConeWithColorProbabilityArray, "/cones",
        lambda m: show("cones", m), 10)
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("seen:", sorted(seen))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
