#!/usr/bin/env python3
"""Offline checks for the fly bridge geometry, vision and control.

Run on Windows or Linux, no ROS installation needed:

    python fruitfly/tools/test_track.py
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "ros2_ws", "src", "fly_bridge")
sys.path.insert(0, SRC)

from fly_bridge.control import ControlConfig, PurePursuit  # noqa: E402
from fly_bridge.track import Track  # noqa: E402
from fly_bridge.vision import render_to_pgm, render_view  # noqa: E402

CONES = os.path.join(SRC, "data", "trackdrive.csv")
CENTER = os.path.join(SRC, "data", "centerline.csv")


def main() -> int:
    ok = True

    track = Track(CONES, CENTER)
    print(track.summary())
    print(f"track length: {track.length:.2f} m")

    # --- walls exist and are continuous ---------------------------------
    for name in ("blue", "yellow"):
        wall = track.walls[name]
        print(f"{name}: {wall.n_points} points, {len(wall.polylines)} polylines, "
              f"{wall.n_segments} segments")
        if wall.n_points < 50 or wall.n_segments < 50:
            print(f"FAIL: {name} wall is too sparse")
            ok = False

    # --- blue should be on the left, yellow on the right -----------------
    signs = {}
    for name in ("blue", "yellow"):
        lats = [c[3] for c in track._cone_data[name]]
        pos = sum(1 for v in lats if v > 0)
        neg = sum(1 for v in lats if v < 0)
        signs[name] = (pos, neg)
        print(f"{name}: lateral sign +{pos} / -{neg}")
    if not (signs["blue"][0] > signs["blue"][1]):
        print("FAIL: blue cones are mostly on the right; wall colour convention is wrong")
        ok = False
    if not (signs["yellow"][1] > signs["yellow"][0]):
        print("FAIL: yellow cones are mostly on the left; wall colour convention is wrong")
        ok = False

    # --- centreline queries ----------------------------------------------
    for frac in (0.0, 0.25, 0.5, 0.75):
        s = track.length * frac
        x, y = track.point_at(s)
        _s, lat, dist = track.project(x, y)
        print(f"s={s:7.2f} point=({x:7.2f},{y:7.2f}) lateral={lat:+.4f} "
              f"dist={dist:.4f} half={track.half_width_at(s):.2f}")
        if abs(lat) > 0.05 or dist > 0.05:
            print("FAIL: centreline point does not project back onto the centreline")
            ok = False

    # --- vision ----------------------------------------------------------
    front = track.heading_at(0.0)
    x0, y0 = track.point_at(0.0)
    img = render_view(track.walls, x0, y0, front, 160, 120)
    nonzero = sum(1 for v in img if v)
    hist = {}
    for v in img:
        if v:
            hist[v] = hist.get(v, 0) + 1
    print(f"retina: {len(img)} pixels, {nonzero} lit, values={sorted(hist.items())[:6]}")
    if nonzero < 150:
        print("FAIL: retina is nearly empty")
        ok = False
    out_dir = os.path.join(ROOT, "out")
    os.makedirs(out_dir, exist_ok=True)
    render_to_pgm(img, 160, 120, os.path.join(out_dir, "retina_start.pgm"))
    print(f"wrote {os.path.join(out_dir, 'retina_start.pgm')}")

    # --- control ---------------------------------------------------------
    cfg = ControlConfig(lookahead=4.0, target_speed=3.0)
    pp = PurePursuit(track, cfg)
    cmd = pp.command(x0, y0, front, 0.0)
    print(f"on centreline: steer={cmd.steer:+.3f} accel={cmd.accel:+.2f} "
          f"contact={cmd.wall_contact} lat={cmd.lateral:+.2f}")
    if abs(cmd.steer) > 0.15 or cmd.wall_contact:
        print("FAIL: on-centreline command is not gentle")
        ok = False

    # offset to the left of the centreline by 1.5 m -> steer right (negative)
    nx, ny = math.cos(front + math.pi / 2), math.sin(front + math.pi / 2)
    xl, yl = x0 + 1.5 * nx, y0 + 1.5 * ny
    cmd_l = pp.command(xl, yl, front, 0.0)
    print(f"1.5 m left: steer={cmd_l.steer:+.3f} accel={cmd_l.accel:+.2f} "
          f"contact={cmd_l.wall_contact} lat={cmd_l.lateral:+.2f} half={cmd_l.half_width:.2f}")
    if cmd_l.steer >= 0:
        print("FAIL: left offset should steer right (negative)")
        ok = False

    # beyond the wall -> contact + brake
    far = max(0.8, cmd.half_width + 0.5)
    xf, yf = x0 + far * nx, y0 + far * ny
    cmd_f = pp.command(xf, yf, front, 3.0)
    print(f"{far:.2f} m left: steer={cmd_f.steer:+.3f} accel={cmd_f.accel:+.2f} "
          f"contact={cmd_f.wall_contact} lat={cmd_f.lateral:+.2f}")
    if not cmd_f.wall_contact or cmd_f.accel >= 0:
        print("FAIL: crossing the wall should brake")
        ok = False

    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
