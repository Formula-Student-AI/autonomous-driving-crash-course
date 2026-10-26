"""Static 3D scenery for Foxglove: road, barriers, real cones, trees, sky.

The EUFS simulator has no renderer; its Foxglove view is just markers.  The
default extension draws each cone as a cylinder.  This module builds a much
richer scene from the same track CSV:

* asphalt ribbon + white edge lines + start/finish checker,
* low Armco-style barriers with red top rails,
* tapered cone meshes with a white reflective band and a black base,
* trees outside the barriers,
* an optional sky dome (the Foxglove panel background is the cheaper option).

Everything is a `visualization_msgs/MarkerArray`, published latched on
`/fly/scenery`.  It is static, so the node builds it once and republishes it
periodically for late subscribers.
"""
from __future__ import annotations

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .track import Track

# ---------------------------------------------------------------- helpers


def _p(x: float, y: float, z: float) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


def _marker(ns: str, mid: int, mtype: int, color: tuple[float, float, float, float],
            scale: tuple[float, float, float] = (1.0, 1.0, 1.0),
            points: list[Point] | None = None) -> Marker:
    m = Marker()
    m.header.frame_id = "map"
    m.ns = ns
    m.id = int(mid)
    m.type = int(mtype)
    m.action = Marker.ADD
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
    m.color.r, m.color.g, m.color.b, m.color.a = color
    m.pose.orientation.w = 1.0
    if points:
        m.points = points
    return m


def _center_samples(track: Track):
    """Return [(x, y, s, nx, ny, half)] for every centreline vertex."""
    pts = track.center
    n = len(pts)
    if n < 3:
        return []
    s = [0.0] * n
    for i in range(1, n):
        s[i] = s[i - 1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                     pts[i][1] - pts[i - 1][1])
    out = []
    for i in range(n):
        px, py = pts[i]
        qx, qy = pts[(i - 1) % n]
        rx, ry = pts[(i + 1) % n]
        tx, ty = rx - qx, ry - qy
        length = math.hypot(tx, ty)
        if length < 1e-9:
            tx, ty, length = 1.0, 0.0, 1.0
        nx, ny = -ty / length, tx / length  # left normal
        out.append((px, py, s[i], nx, ny, track.half_width_at(s[i])))
    return out


def _ribbon(samples, offset_fn):
    """Return left/right edge points for a centreline offset function."""
    left, right = [], []
    for x, y, _s, nx, ny, half in samples:
        dl = offset_fn(half, +1)
        dr = offset_fn(half, -1)
        left.append((x + nx * dl, y + ny * dl))
        right.append((x - nx * dr, y - ny * dr))
    return left, right


def _road_triangles(left, right, z):
    pts: list[Point] = []
    n = len(left)
    for i in range(n):
        j = (i + 1) % n
        l0, r0 = left[i], right[i]
        l1, r1 = left[j], right[j]
        pts += [_p(*l0, z), _p(*r0, z), _p(*r1, z)]
        pts += [_p(*l0, z), _p(*r1, z), _p(*l1, z)]
    return pts


def _quad(pts: list[Point], a, b, c, d) -> None:
    """Two triangles for a quad, duplicated with reversed winding."""
    for p0, p1, p2 in ((a, b, c), (a, c, d)):
        pts += [_p(*p0), _p(*p1), _p(*p2)]
    for p0, p1, p2 in ((a, c, b), (a, d, c)):
        pts += [_p(*p0), _p(*p1), _p(*p2)]


def _barrier_triangles(line, outward_normals, z0, z1, width=0.18):
    """Build a solid vertical barrier along a polyline."""
    pts: list[Point] = []
    n = len(line)
    for i in range(n):
        j = (i + 1) % n
        ax, ay = line[i]
        bx, by = line[j]
        ox0, oy0 = outward_normals[i]
        ox1, oy1 = outward_normals[j]
        dx, dy = ax + ox0 * width, ay + oy0 * width
        cx, cy = bx + ox1 * width, by + oy1 * width
        # track-facing face, outside face, and top
        _quad(pts, (ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1))
        _quad(pts, (dx, dy, z0), (cx, cy, z0), (cx, cy, z1), (dx, dy, z1))
        _quad(pts, (ax, ay, z1), (bx, by, z1), (cx, cy, z1), (dx, dy, z1))
    return pts


def _cone_geometry(x, y, z0, z1, r0, r1, segments=10):
    """Side + base of a tapered cone, as a triangle list."""
    pts: list[Point] = []
    for k in range(segments):
        a0 = 2.0 * math.pi * k / segments
        a1 = 2.0 * math.pi * (k + 1) / segments
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)
        b0 = (x + r0 * c0, y + r0 * s0, z0)
        b1 = (x + r0 * c1, y + r0 * s1, z0)
        t0 = (x + r1 * c0, y + r1 * s0, z1)
        t1 = (x + r1 * c1, y + r1 * s1, z1)
        pts += [_p(*b0), _p(*b1), _p(*t1)]
        pts += [_p(*b0), _p(*t1), _p(*t0)]
        # base disc (downwards)
        pts += [_p(x, y, z0), _p(*b1), _p(*b0)]
    return pts


def _cone_band(x, y, z0, z1, r0, r1, segments=10):
    pts: list[Point] = []
    for k in range(segments):
        a0 = 2.0 * math.pi * k / segments
        a1 = 2.0 * math.pi * (k + 1) / segments
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)
        b0 = (x + r0 * c0, y + r0 * s0, z0)
        b1 = (x + r0 * c1, y + r0 * s1, z0)
        t0 = (x + r1 * c0, y + r1 * s0, z1)
        t1 = (x + r1 * c1, y + r1 * s1, z1)
        pts += [_p(*b0), _p(*b1), _p(*t1)]
        pts += [_p(*b0), _p(*t1), _p(*t0)]
    return pts


def _checker(track: Track, samples, half_width: float, rows=2, cols=8):
    """White/black start-finish checker at s=0."""
    if not samples:
        return [], []
    x, y, _s, nx, ny, _h = samples[0]
    # tangent at s=0
    tx, ty = -ny, nx
    length = 1.0
    width = 2.0 * half_width
    white: list[Point] = []
    black: list[Point] = []
    for r in range(rows):
        for c in range(cols):
            u0 = -width / 2 + width * c / cols
            u1 = -width / 2 + width * (c + 1) / cols
            v0 = length * r / rows
            v1 = length * (r + 1) / rows

            def pt(u, v):
                return (x + nx * u + tx * v, y + ny * u + ty * v, 0.02)

            a, b, cpx, d = pt(u0, v0), pt(u1, v0), pt(u1, v1), pt(u0, v1)
            tri = [_p(*a), _p(*b), _p(*cpx), _p(*a), _p(*cpx), _p(*d)]
            (white if (r + c) % 2 == 0 else black).extend(tri)
    return white, black


# ---------------------------------------------------------------- public


def build_scenery(track: Track, include_trees: bool = True,
                  include_sky: bool = False) -> MarkerArray:
    arr = MarkerArray()
    samples = _center_samples(track)
    if not samples:
        return arr

    # ---- grass ground ---------------------------------------------------
    cx = sum(p[0] for p in samples) / len(samples)
    cy = sum(p[1] for p in samples) / len(samples)
    ground = _marker("scenery/ground", 0, Marker.CUBE,
                     (0.12, 0.27, 0.10, 1.0), (200.0, 200.0, 0.06))
    ground.pose.position.x = cx
    ground.pose.position.y = cy
    ground.pose.position.z = -0.05
    arr.markers.append(ground)

    # ---- road -----------------------------------------------------------
    left, right = _ribbon(samples, lambda half, side: half)
    arr.markers.append(_marker(
        "scenery/road", 0, Marker.TRIANGLE_LIST, (0.11, 0.11, 0.12, 1.0),
        points=_road_triangles(left, right, 0.012)))

    arr.markers.append(_marker(
        "scenery/edge_left", 1, Marker.LINE_STRIP, (0.92, 0.92, 0.92, 1.0),
        (0.08, 0.0, 0.0), [_p(x, y, 0.03) for x, y in left]))
    arr.markers.append(_marker(
        "scenery/edge_right", 2, Marker.LINE_STRIP, (0.92, 0.92, 0.92, 1.0),
        (0.08, 0.0, 0.0), [_p(x, y, 0.03) for x, y in right]))

    white, black = _checker(track, samples, samples[0][5])
    arr.markers.append(_marker("scenery/start_white", 0, Marker.TRIANGLE_LIST,
                               (0.95, 0.95, 0.95, 1.0), points=white))
    arr.markers.append(_marker("scenery/start_black", 1, Marker.TRIANGLE_LIST,
                               (0.03, 0.03, 0.03, 1.0), points=black))

    # ---- barriers -------------------------------------------------------
    barrier_left = []
    barrier_right = []
    left_normals = []
    right_normals = []
    for x, y, _s, nx, ny, half in samples:
        off = half + 0.65
        barrier_left.append((x + nx * off, y + ny * off))
        barrier_right.append((x - nx * off, y - ny * off))
        left_normals.append((nx, ny))
        right_normals.append((-nx, -ny))

    arr.markers.append(_marker(
        "scenery/barrier_left", 0, Marker.TRIANGLE_LIST,
        (0.72, 0.72, 0.75, 1.0),
        points=_barrier_triangles(barrier_left, left_normals, 0.0, 0.45)))
    arr.markers.append(_marker(
        "scenery/barrier_right", 1, Marker.TRIANGLE_LIST,
        (0.72, 0.72, 0.75, 1.0),
        points=_barrier_triangles(barrier_right, right_normals, 0.0, 0.45)))

    arr.markers.append(_marker(
        "scenery/rail_left", 2, Marker.LINE_STRIP, (0.85, 0.12, 0.10, 1.0),
        (0.10, 0.0, 0.0), [_p(x, y, 0.47) for x, y in barrier_left]))
    arr.markers.append(_marker(
        "scenery/rail_right", 3, Marker.LINE_STRIP, (0.85, 0.12, 0.10, 1.0),
        (0.10, 0.0, 0.0), [_p(x, y, 0.47) for x, y in barrier_right]))

    # ---- cones ----------------------------------------------------------
    colors = {
        "blue": (0.06, 0.18, 1.0, 1.0),
        "yellow": (1.0, 0.85, 0.05, 1.0),
        "orange": (1.0, 0.40, 0.0, 1.0),
    }
    body_pts: dict[str, list[Point]] = {k: [] for k in colors}
    band_pts: list[Point] = []
    base_pts: list[Point] = []
    for tag, x, y in track.cones:
        if tag not in body_pts:
            continue
        z0, z1 = 0.02, 0.32
        r0, r1 = 0.145, 0.018
        body_pts[tag] += _cone_geometry(x, y, z0, z1, r0, r1)
        # white reflective band, slightly proud of the body
        zb0, zb1 = 0.16, 0.23
        rb0 = r0 + (r1 - r0) * (zb0 - z0) / (z1 - z0) + 0.006
        rb1 = r0 + (r1 - r0) * (zb1 - z0) / (z1 - z0) + 0.006
        band_pts += _cone_band(x, y, zb0, zb1, rb0, rb1)
        # dark square base
        s = 0.17
        base_pts += [
            _p(x - s, y - s, 0.021), _p(x + s, y - s, 0.021), _p(x + s, y + s, 0.021),
            _p(x - s, y - s, 0.021), _p(x + s, y + s, 0.021), _p(x - s, y + s, 0.021),
        ]

    for i, (tag, color) in enumerate(colors.items()):
        if body_pts[tag]:
            arr.markers.append(_marker(f"scenery/cone_{tag}", i,
                                       Marker.TRIANGLE_LIST, color,
                                       points=body_pts[tag]))
    arr.markers.append(_marker("scenery/cone_band", 10, Marker.TRIANGLE_LIST,
                               (0.95, 0.95, 0.95, 1.0), points=band_pts))
    arr.markers.append(_marker("scenery/cone_base", 11, Marker.TRIANGLE_LIST,
                               (0.05, 0.05, 0.05, 1.0), points=base_pts))

    # ---- trees ----------------------------------------------------------
    if include_trees:
        mid = 0
        stride = max(3, len(samples) // 16)
        for k in range(0, len(samples), stride):
            x, y, _s, nx, ny, half = samples[k]
            for side in (+1, -1):
                jitter = 2.0 + 4.0 * (((k * 37 + (0 if side > 0 else 7)) % 13) / 13.0)
                off = half + 8.0 + jitter
                tx = x + nx * off * side
                ty = y + ny * off * side
                h = 2.8 + 1.4 * (((k * 19) % 7) / 7.0)
                trunk = _marker("scenery/tree_trunk", mid, Marker.CYLINDER,
                                (0.32, 0.20, 0.10, 1.0), (0.32, 0.32, h))
                trunk.pose.position.x = tx
                trunk.pose.position.y = ty
                trunk.pose.position.z = h / 2.0
                arr.markers.append(trunk)
                mid += 1
                for c in range(2):
                    r = 1.5 + 0.7 * (((k * 11 + c * 3) % 7) / 7.0)
                    canopy = _marker("scenery/tree_canopy", mid, Marker.SPHERE,
                                     (0.10 + 0.04 * c, 0.38 - 0.05 * c, 0.12, 1.0),
                                     (2 * r, 2 * r, 2 * r))
                    canopy.pose.position.x = tx + 0.4 * (c - 0.5)
                    canopy.pose.position.y = ty + 0.3 * (c - 0.5)
                    canopy.pose.position.z = h + r * 0.55 + 0.35 * c
                    arr.markers.append(canopy)
                    mid += 1

    # ---- optional sky dome ---------------------------------------------
    if include_sky:
        radius = 250.0
        height = 150.0
        segs = 32
        pts: list[Point] = []
        for k in range(segs):
            a0 = 2.0 * math.pi * k / segs
            a1 = 2.0 * math.pi * (k + 1) / segs
            x0, y0 = cx + radius * math.cos(a0), cy + radius * math.sin(a0)
            x1, y1 = cx + radius * math.cos(a1), cy + radius * math.sin(a1)
            for z0, z1 in ((0.0, height),):
                # double-sided side wall
                pts += [_p(x0, y0, z0), _p(x1, y1, z0), _p(x1, y1, z1)]
                pts += [_p(x0, y0, z0), _p(x1, y1, z1), _p(x0, y0, z1)]
                pts += [_p(x1, y1, z0), _p(x0, y0, z0), _p(x0, y0, z1)]
                pts += [_p(x1, y1, z0), _p(x0, y0, z1), _p(x1, y1, z1)]
            # ceiling fan
            pts += [_p(cx, cy, height), _p(x1, y1, height), _p(x0, y0, height)]
            pts += [_p(cx, cy, height), _p(x0, y0, height), _p(x1, y1, height)]
        arr.markers.append(_marker("scenery/sky", 0, Marker.TRIANGLE_LIST,
                                   (0.55, 0.78, 0.95, 1.0), points=pts))

    return arr
