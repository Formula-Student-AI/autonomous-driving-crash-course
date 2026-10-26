#!/usr/bin/env python3
"""Save one frame from /race/camera to a PNG (for checking the OpenCV view).

    python3 "fake run/camera_snapshot.py"
    python3 "fake run/camera_snapshot.py" --out "fake run/out/camera.png"
"""
from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/race/camera")
    ap.add_argument("--out", default=os.path.join(here, "out", "camera.png"))
    ap.add_argument("--timeout", type=float, default=10.0)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("camera_snapshot")
    saved = {"ok": False}

    def cb(msg: Image) -> None:
        if saved["ok"]:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        if msg.encoding != "bgr8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(a.out, img)
        saved["ok"] = True
        print(f"saved {msg.width}x{msg.height} {msg.encoding} "
              f"frame={msg.header.frame_id} -> {a.out}")

    node.create_subscription(Image, a.topic, cb, 1)
    t0 = time.time()
    while not saved["ok"] and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if saved["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
