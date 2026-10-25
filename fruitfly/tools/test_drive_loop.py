#!/usr/bin/env python3
"""Exercise the full flyscreen live loop with a moving image.

Posts a moving bright bar (plus the wall retina if available) for a few
seconds, then prints the fly's motor readout.  This proves that brain ->
body -> /api/live/motor is alive, not just the HTTP plumbing.

    python fruitfly/tools/test_drive_loop.py --seconds 8
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PGM = ROOT / "out" / "retina_start.pgm"
W, H = 160, 120


def get(url: str, timeout: float = 2.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def post(url: str, data: bytes, timeout: float = 2.0) -> bytes:
    req = urllib.request.Request(url, data=data, method="POST",
                                 headers={"Content-Type": "application/octet-stream"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def base_frame() -> bytearray:
    buf = bytearray(W * H)
    if PGM.exists():
        raw = PGM.read_bytes()
        start = raw.find(b"\n255\n") + len(b"\n255\n")
        img = raw[start:start + W * H]
        if len(img) == W * H:
            buf[:] = img
    return buf


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8760")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--rate", type=float, default=12.0)
    a = ap.parse_args()
    base = a.url.rstrip("/")

    for _ in range(120):
        try:
            get(base + "/api/state")
            break
        except (urllib.error.URLError, TimeoutError, OSError):
            time.sleep(0.5)
    else:
        print("FAIL: server did not come up")
        return 1

    bg = base_frame()
    t0 = time.time()
    last_print = 0.0
    sent = 0
    last = {}
    while time.time() - t0 < a.seconds:
        i = int((time.time() - t0) * a.rate)
        buf = bytearray(bg)
        # a bright bar sweeping across the retina + a second one the other way
        x0 = (i * 13) % W
        x1 = (W - (i * 7) % W) % W
        for y in range(H):
            for x in (x0, (x0 + 1) % W, (x0 + 2) % W, x1, (x1 + 1) % W):
                buf[y * W + x] = 255
        try:
            post(f"{base}/api/live/frame?w={W}&h={H}&fmt=gray", bytes(buf))
            sent += 1
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            print("post failed:", e)
        if time.time() - last_print >= 1.0:
            last_print = time.time()
            snap = json.loads(get(base + "/api/live/motor"))
            motor = snap.get("motor", {})
            print(f"t={time.time()-t0:4.1f}s frames={sent:3d} "
                  f"processed={snap.get('status', {}).get('frame')} "
                  f"steer={motor.get('steer', 0):+.3f} "
                  f"thr={motor.get('throttle', 0):.3f} "
                  f"L={motor.get('left', 0):+.3f} R={motor.get('right', 0):+.3f} "
                  f"lit={motor.get('activity', 0)} up={motor.get('upright')} "
                  f"body={snap.get('has_body')}")
            last = snap
        time.sleep(1.0 / max(1.0, a.rate))

    try:
        brain = get(base + "/api/live/brain.jpg")
        body = get(base + "/api/live/body.jpg")
        print(f"brain.jpg {len(brain)} B, body.jpg {len(body)} B")
    except urllib.error.HTTPError as e:
        print("image endpoint not ready:", e.code)

    motor = last.get("motor", {})
    if not motor:
        print("RESULT: FAIL (no motor readout)")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
