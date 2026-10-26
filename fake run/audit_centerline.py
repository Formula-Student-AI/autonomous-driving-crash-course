#!/usr/bin/env python3
"""Check the fixed centreline against the actual blue/yellow cone pairs.

    python3 "fake run/audit_centerline.py"
    python3 "fake run/audit_centerline.py" --out "fake run/centerline_audit.csv"

For every centreline point it finds the nearest blue and yellow cone, computes
their midpoint, and reports the distance from the centreline.  It also checks
that blue is consistently on the left and yellow on the right.  Pure standard
library, so it runs on Windows or in the container.
"""
from __future__ import annotations

import argparse
import csv
import math
import os


def default_track() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(
        here, "..", "sim", "src", "map_lib", "maps",
        "competitions", "FSUK", "2023", "trackdrive.csv"))


def side(ax, ay, bx, by, px, py) -> float:
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax)


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", default=default_track())
    ap.add_argument("--centerline",
                    default=os.path.join(here, "fixed_path_centerline.csv"))
    ap.add_argument("--out", default="",
                    help="optional CSV of centreline vs cone-pair midpoints")
    a = ap.parse_args()

    cones = []
    with open(a.track, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("tag") in ("blue", "yellow"):
                cones.append((r["tag"], float(r["x"]), float(r["y"])))
    centre = []
    with open(a.centerline, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            centre.append((float(r["x"]), float(r["y"])))

    rows = []
    deviations = []
    anomalies = 0
    n = len(centre)
    for i, (cx, cy) in enumerate(centre):
        best = {}
        for tag, x, y in cones:
            d = math.hypot(x - cx, y - cy)
            if tag not in best or d < best[tag][0]:
                best[tag] = (d, x, y)
        if "blue" not in best or "yellow" not in best:
            continue
        db, bx, by = best["blue"]
        dy, yx, yy = best["yellow"]
        mx, my = (bx + yx) / 2.0, (by + yy) / 2.0
        dev = math.hypot(mx - cx, my - cy)
        deviations.append(dev)
        rows.append((i, cx, cy, mx, my, dev, db, dy))

        ax, ay = centre[i]
        bx2, by2 = centre[(i + 1) % n]
        sb = side(ax, ay, bx2, by2, *best["blue"][1:])
        sy = side(ax, ay, bx2, by2, *best["yellow"][1:])
        if not (sb > 0 and sy < 0):
            anomalies += 1

    deviations.sort()
    if not deviations:
        print("no valid cone pairs found")
        return 1
    mean = sum(deviations) / len(deviations)
    median = deviations[len(deviations) // 2]
    print(f"cones: {len(cones)}  centreline points: {n}")
    print(f"midpoint deviation: mean {mean:.3f} m, median {median:.3f} m, "
          f"max {deviations[-1]:.3f} m")
    print(f"blue-left / yellow-right anomalies: {anomalies} of {n}")
    print("worst 8:")
    for i, cx, cy, mx, my, dev, db, dy in sorted(rows, key=lambda r: r[5],
                                                 reverse=True)[:8]:
        print(f"  idx {i:3d} centre=({cx:7.2f},{cy:7.2f}) "
              f"mid=({mx:7.2f},{my:7.2f}) dev={dev:.2f} "
              f"blue={db:.2f} yellow={dy:.2f}")

    if a.out:
        with open(a.out, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["idx", "cx", "cy", "mx", "my", "deviation",
                        "dist_blue", "dist_yellow"])
            w.writerows(rows)
        print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
