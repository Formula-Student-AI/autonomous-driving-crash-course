"""Track geometry for the fly bridge.

Loads the EUFS cone CSV and the ordered centreline CSV, then:

* orders the blue and yellow cones along the centreline,
* connects consecutive cones of the same colour into wall polylines,
* estimates the local half-width of the track from paired blue/yellow cones,
* answers "where is the car relative to the centreline / walls?".

Pure standard library on purpose: the ROS 2 container does not need numpy or
OpenCV for this, and the same module can be exercised on Windows.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field


@dataclass
class Wall:
    """One colour of track boundary, split into gap-free polylines."""

    name: str
    color: tuple[float, float, float, float]
    polylines: list[list[tuple[float, float]]] = field(default_factory=list)

    @property
    def n_points(self) -> int:
        return sum(len(line) for line in self.polylines)

    @property
    def n_segments(self) -> int:
        return sum(max(0, len(line) - 1) for line in self.polylines)


def _read_xy(path: str) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, TypeError, ValueError):
                continue
            if out and abs(out[-1][0] - x) < 1e-9 and abs(out[-1][1] - y) < 1e-9:
                continue
            out.append((x, y))
    if not out:
        raise ValueError(f"no x,y rows in {path}")
    return out


def _read_cones(path: str) -> list[tuple[str, float, float]]:
    out: list[tuple[str, float, float]] = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            tag = (row.get("tag") or "").strip().lower()
            if tag not in ("blue", "yellow", "orange"):
                continue
            try:
                out.append((tag, float(row["x"]), float(row["y"])))
            except (KeyError, TypeError, ValueError):
                continue
    if not out:
        raise ValueError(f"no cone rows in {path}")
    return out


class Track:
    """Centreline + cone walls + geometric queries."""

    def __init__(
        self,
        cones_csv: str,
        centerline_csv: str | None = None,
        max_link: float = 12.0,
        pair_window: float = 6.0,
    ) -> None:
        self.cones_csv = cones_csv
        self.centerline_csv = centerline_csv
        self.max_link = float(max_link)
        self.pair_window = float(pair_window)

        self.cones = _read_cones(cones_csv)
        if centerline_csv:
            self.center = _read_xy(centerline_csv)
        else:
            self.center = self._center_from_cones()

        self._build_segments()
        self._project_cones()
        self.walls = self._build_walls()
        self._build_half_width()

    # ------------------------------------------------------------ centreline
    def _center_from_cones(self) -> list[tuple[float, float]]:
        """Fallback: pair blue/yellow by nearest neighbour and take midpoints."""
        blue = [(x, y) for tag, x, y in self.cones if tag == "blue"]
        yellow = [(x, y) for tag, x, y in self.cones if tag == "yellow"]
        used: set[int] = set()
        mid: list[tuple[float, float, float]] = []
        for bx, by in blue:
            best, bd = -1, 1e18
            for j, (yx, yy) in enumerate(yellow):
                if j in used:
                    continue
                d = (bx - yx) ** 2 + (by - yy) ** 2
                if d < bd:
                    bd, best = d, j
            if best >= 0:
                used.add(best)
                yx, yy = yellow[best]
                mid.append(((bx + yx) / 2.0, (by + yy) / 2.0, math.sqrt(bd)))
        if not mid:
            raise ValueError("cannot infer a centreline from the cones")
        # Order the midpoints with a nearest-neighbour walk. This fallback is
        # only used when no centreline CSV is supplied; the course track always
        # ships one, which is already ordered and much better.
        pts = [mid[0]]
        remaining = set(range(1, len(mid)))
        while remaining:
            lx, ly = pts[-1][0], pts[-1][1]
            j = min(remaining, key=lambda k: (mid[k][0] - lx) ** 2
                    + (mid[k][1] - ly) ** 2)
            remaining.remove(j)
            pts.append(mid[j])
        return [(x, y) for x, y, _ in pts]

    def _build_segments(self) -> None:
        pts = list(self.center)
        if len(pts) < 2:
            raise ValueError("centreline needs at least two points")
        # Close the loop if the CSV is not already closed.
        if math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) > 0.5:
            pts = pts + [pts[0]]
        self.center_closed = pts
        segs = []
        s0 = 0.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            length = math.hypot(x1 - x0, y1 - y0)
            if length < 1e-9:
                continue
            segs.append((x0, y0, x1, y1, length, s0))
            s0 += length
        self._segments = segs
        self.length = s0

    # ------------------------------------------------------------ queries
    def project(self, x: float, y: float) -> tuple[float, float, float]:
        """Return (arc length s, signed lateral, distance to centreline).

        Signed lateral is positive to the *left* of the centreline direction.
        """
        best_d2 = 1e300
        best_s = 0.0
        best_lat = 0.0
        for x0, y0, x1, y1, length, s0 in self._segments:
            dx, dy = x1 - x0, y1 - y0
            t = ((x - x0) * dx + (y - y0) * dy) / (length * length)
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0
            px, py = x0 + t * dx, y0 + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = s0 + t * length
                best_lat = (dx * (y - y0) - dy * (x - x0)) / length
        return best_s, best_lat, math.sqrt(best_d2)

    def point_at(self, s: float) -> tuple[float, float]:
        s %= self.length
        for x0, y0, x1, y1, length, s0 in self._segments:
            if s <= s0 + length:
                t = (s - s0) / length
                return x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        x0, y0, x1, y1, _, _ = self._segments[-1]
        return x1, y1

    def heading_at(self, s: float) -> float:
        s %= self.length
        for x0, y0, x1, y1, length, s0 in self._segments:
            if s <= s0 + length:
                return math.atan2(y1 - y0, x1 - x0)
        x0, y0, x1, y1, _, _ = self._segments[-1]
        return math.atan2(y1 - y0, x1 - x0)

    def half_width_at(self, s: float) -> float:
        samples = self._half_samples
        if not samples:
            return self.fallback_half_width
        s %= self.length
        # samples sorted by s; binary search then interpolate
        lo, hi = 0, len(samples)
        while lo < hi:
            mid = (lo + hi) // 2
            if samples[mid][0] < s:
                lo = mid + 1
            else:
                hi = mid
        if lo == 0:
            s0, w0 = samples[-1]
            s1, w1 = samples[0]
        elif lo >= len(samples):
            s0, w0 = samples[-1]
            s1, w1 = samples[0][0] + self.length, samples[0][1]
        else:
            s0, w0 = samples[lo - 1]
            s1, w1 = samples[lo]
        if s < s0:
            s += self.length
        if s1 <= s0:
            s1 += self.length
        t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        t = max(0.0, min(1.0, t))
        return w0 + t * (w1 - w0)

    # ------------------------------------------------------------ internal
    def _project_cones(self) -> None:
        self._cone_data: dict[str, list[tuple[float, float, float, float]]] = {
            "blue": [], "yellow": [], "orange": []}
        for tag, x, y in self.cones:
            s, lat, _ = self.project(x, y)
            self._cone_data[tag].append((s, x, y, lat))
        for tag in self._cone_data:
            self._cone_data[tag].sort(key=lambda c: c[0])

    def _build_walls(self) -> dict[str, Wall]:
        walls: dict[str, Wall] = {}
        specs = {
            "blue": (0.15, 0.45, 1.0, 0.95),
            "yellow": (1.0, 0.85, 0.1, 0.95),
        }
        for tag, color in specs.items():
            pts = self._cone_data[tag]
            polylines: list[list[tuple[float, float]]] = []
            cur: list[tuple[float, float]] = []
            prev: tuple[float, float, float, float] | None = None
            for s, x, y, lat in pts:
                if prev is not None:
                    gap = math.hypot(x - prev[1], y - prev[2])
                    if gap > self.max_link:
                        if len(cur) >= 2:
                            polylines.append(cur)
                        cur = []
                cur.append((x, y))
                prev = (s, x, y, lat)
            if len(cur) >= 2:
                polylines.append(cur)
            # Close the loop across s=0 if the wrap gap is small.
            if polylines and pts:
                first = pts[0]
                last = pts[-1]
                wrap_s = self.length - last[0] + first[0]
                wrap_d = math.hypot(first[1] - last[1], first[2] - last[2])
                if wrap_s <= self.max_link and wrap_d <= self.max_link:
                    polylines[-1].append((first[1], first[2]))
            walls[tag] = Wall(tag, color, polylines)
        return walls

    def _build_half_width(self) -> None:
        blue = self._cone_data["blue"]
        yellow = self._cone_data["yellow"]
        pairs: list[tuple[float, float]] = []

        def nearest_in_s(other, s):
            best, bd = None, 1e18
            for cand in other:
                ds = abs(cand[0] - s)
                ds = min(ds, self.length - ds)
                if ds < bd:
                    bd, best = ds, cand
            return best, bd

        for s, x, y, lat in blue:
            cand, ds = nearest_in_s(yellow, s)
            if cand is not None and ds <= self.pair_window:
                half = abs(lat - cand[3]) / 2.0
                if 0.2 < half < 20.0:
                    pairs.append(((s + cand[0]) / 2.0 % self.length, half))
        for s, x, y, lat in yellow:
            cand, ds = nearest_in_s(blue, s)
            if cand is not None and ds <= self.pair_window:
                half = abs(lat - cand[3]) / 2.0
                if 0.2 < half < 20.0:
                    pairs.append(((s + cand[0]) / 2.0 % self.length, half))
        pairs.sort()
        # Collapse to one sample per 0.5 m bin (median).
        bins: dict[float, list[float]] = {}
        for s, w in pairs:
            key = round(s * 2.0) / 2.0
            bins.setdefault(key, []).append(w)
        self._half_samples = [(k, sorted(v)[len(v) // 2]) for k, v in sorted(bins.items())]
        widths = [w for _, w in self._half_samples]
        if widths:
            widths.sort()
            self.fallback_half_width = widths[len(widths) // 2]
            self.half_width_min = widths[0]
            self.half_width_max = widths[-1]
        else:
            self.fallback_half_width = 2.0
            self.half_width_min = self.half_width_max = 2.0

    # ------------------------------------------------------------ summary
    def summary(self) -> str:
        parts = [f"track: {len(self.cones)} cones, centreline {self.length:.1f} m"]
        for tag in ("blue", "yellow", "orange"):
            parts.append(f"{tag}={len(self._cone_data[tag])}")
        for tag, wall in self.walls.items():
            parts.append(f"{tag} walls: {wall.n_points} pts / "
                         f"{len(wall.polylines)} polylines / {wall.n_segments} segs")
        parts.append(f"half-width {self.half_width_min:.2f}.."
                     f"{self.half_width_max:.2f} m (median {self.fallback_half_width:.2f})")
        return "; ".join(parts)
