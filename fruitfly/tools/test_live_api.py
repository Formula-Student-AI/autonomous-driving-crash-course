#!/usr/bin/env python3
"""Smoke-test the flyscreen live API without ROS.

Start flyscreen in live mode first:

    python live_webui.py --no-brain --no-body --size small

Then:

    python fruitfly/tools/test_live_api.py
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PGM = ROOT / "out" / "retina_start.pgm"


def _get(url: str, timeout: float = 2.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def _post(url: str, data: bytes, ctype: str = "application/octet-stream",
          timeout: float = 2.0) -> bytes:
    req = urllib.request.Request(url, data=data, method="POST",
                                 headers={"Content-Type": ctype})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8760")
    a = ap.parse_args()
    base = a.url.rstrip("/")

    # 1. wait for the server
    state = None
    for _ in range(60):
        try:
            state = json.loads(_get(base + "/api/state"))
            break
        except (urllib.error.URLError, TimeoutError, OSError):
            time.sleep(0.5)
    if state is None:
        print("FAIL: server did not come up; is live_webui.py running?")
        return 1
    print("server up; live =", state.get("live"))

    # 2. build a 160x120 frame: the retina PGM if present, else a gradient
    if PGM.exists():
        raw = PGM.read_bytes()
        header_end = raw.find(b"\n255\n") + len(b"\n255\n")
        frame = raw[header_end:]
    else:
        w, h = 160, 120
        frame = bytes(((x * 255) // max(1, w - 1) for y in range(h) for x in range(w)))
    w, h = 160, 120
    assert len(frame) == w * h, len(frame)

    # 3. POST it
    resp = json.loads(_post(f"{base}/api/live/frame?w={w}&h={h}&fmt=gray", frame))
    print("POST /api/live/frame ->", resp)
    if not resp.get("ok"):
        print("FAIL: frame rejected")
        return 1

    # 4. read the motor + status (the render loop picks up the frame asynchronously)
    snap = {}
    for _ in range(60):
        snap = json.loads(_get(base + "/api/live/motor"))
        if int(snap.get("status", {}).get("frame", 0)) >= int(resp["frame_id"]):
            break
        time.sleep(0.05)
    print("GET /api/live/motor -> connected=", snap.get("connected"),
          "frame_id=", snap.get("frame_id"),
          "motor=", snap.get("motor"))
    if not snap.get("connected"):
        print("FAIL: motor endpoint does not see the frame")
        return 1
    if int(snap.get("status", {}).get("frame", 0)) < int(resp["frame_id"]):
        print("FAIL: render loop did not process the frame")
        return 1

    # 5. brain JPEG (the body may be disabled in the test)
    try:
        brain = _get(base + "/api/live/brain.jpg")
        print("GET /api/live/brain.jpg ->", len(brain), "bytes, jpeg =",
              brain[:2] == b"\xff\xd8")
    except urllib.error.HTTPError as e:
        print("brain JPEG not ready:", e.code)

    # 6. reset handshake
    print("POST /api/live/reset ->", _post(base + "/api/live/reset", b""))

    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
