"""
Geometry primitives for the physics core.

Vectors are plain (x, y) float tuples, not numpy arrays. At 1 kHz with a
handful of bodies, numpy's per-call overhead dominates for 2-element
vectors -- plain tuples are several times faster here. numpy is used where
it belongs: batched grid work in the reachability and estimator layers.

Collision routines return a `Contact` or None. The convention throughout:

    normal    unit vector pointing from body A toward body B, i.e. the
              direction A must move to separate
    depth     penetration depth, always >= 0
    point     approximate contact point in world coordinates
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional

Vec = tuple[float, float]

EPS = 1e-12


# ---------------------------------------------------------------------------
# Scalars
# ---------------------------------------------------------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a: float, b: float) -> float:
    """Signed smallest rotation taking b to a."""
    return wrap_angle(a - b)


def sign(x: float) -> float:
    return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)


# ---------------------------------------------------------------------------
# Vectors
# ---------------------------------------------------------------------------

def add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1])


def sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1])


def scale(a: Vec, s: float) -> Vec:
    return (a[0] * s, a[1] * s)


def dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1]


def cross(a: Vec, b: Vec) -> float:
    """2D scalar cross product (z component of the 3D cross)."""
    return a[0] * b[1] - a[1] * b[0]


def length(a: Vec) -> float:
    return math.hypot(a[0], a[1])


def length_sq(a: Vec) -> float:
    return a[0] * a[0] + a[1] * a[1]


def dist(a: Vec, b: Vec) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist_sq(a: Vec, b: Vec) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def normalize(a: Vec) -> Vec:
    n = math.hypot(a[0], a[1])
    if n < EPS:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


def perp(a: Vec) -> Vec:
    """Rotate 90 degrees counter-clockwise."""
    return (-a[1], a[0])


def rotate(a: Vec, angle: float) -> Vec:
    c = math.cos(angle)
    s = math.sin(angle)
    return (a[0] * c - a[1] * s, a[0] * s + a[1] * c)


def from_angle(angle: float, magnitude: float = 1.0) -> Vec:
    return (math.cos(angle) * magnitude, math.sin(angle) * magnitude)


def heading(a: Vec) -> float:
    return math.atan2(a[1], a[0])


def lerp(a: Vec, b: Vec, t: float) -> Vec:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def clamp_magnitude(a: Vec, max_len: float) -> Vec:
    n = math.hypot(a[0], a[1])
    if n <= max_len or n < EPS:
        return a
    s = max_len / n
    return (a[0] * s, a[1] * s)


# ---------------------------------------------------------------------------
# Frame transforms
#
# A body's pose is (position, angle). "Local" means the body's own frame,
# with +X forward along the body angle.
# ---------------------------------------------------------------------------

def to_local(world_pt: Vec, origin: Vec, angle: float) -> Vec:
    dx = world_pt[0] - origin[0]
    dy = world_pt[1] - origin[1]
    c = math.cos(-angle)
    s = math.sin(-angle)
    return (dx * c - dy * s, dx * s + dy * c)


def to_world(local_pt: Vec, origin: Vec, angle: float) -> Vec:
    c = math.cos(angle)
    s = math.sin(angle)
    return (origin[0] + local_pt[0] * c - local_pt[1] * s,
            origin[1] + local_pt[0] * s + local_pt[1] * c)


def rotate_to_world(local_vec: Vec, angle: float) -> Vec:
    return rotate(local_vec, angle)


def rotate_to_local(world_vec: Vec, angle: float) -> Vec:
    return rotate(world_vec, -angle)


def point_velocity(body_vel: Vec, omega: float, r_world: Vec) -> Vec:
    """Velocity of a point offset r_world from a body's centre of mass."""
    return (body_vel[0] - omega * r_world[1],
            body_vel[1] + omega * r_world[0])


# ---------------------------------------------------------------------------
# Contacts
# ---------------------------------------------------------------------------

class Contact(NamedTuple):
    depth: float      # penetration, >= 0
    normal: Vec       # unit, points from A toward B
    point: Vec        # world-space contact point


def closest_point_on_segment(p: Vec, a: Vec, b: Vec) -> tuple[Vec, float]:
    """Closest point to p on segment ab, plus the parameter t in [0, 1]."""
    ab = sub(b, a)
    denom = length_sq(ab)
    if denom < EPS:
        return a, 0.0
    t = clamp(dot(sub(p, a), ab) / denom, 0.0, 1.0)
    return (a[0] + ab[0] * t, a[1] + ab[1] * t), t


def circle_vs_circle(ca: Vec, ra: float, cb: Vec, rb: float) -> Optional[Contact]:
    d = sub(cb, ca)
    dist2 = length_sq(d)
    r_sum = ra + rb
    if dist2 >= r_sum * r_sum:
        return None
    dl = math.sqrt(dist2)
    if dl < EPS:
        # Exactly concentric: pick an arbitrary but deterministic normal.
        n = (1.0, 0.0)
        depth = r_sum
    else:
        n = (d[0] / dl, d[1] / dl)
        depth = r_sum - dl
    point = (ca[0] + n[0] * ra, ca[1] + n[1] * ra)
    return Contact(depth, n, point)


def circle_vs_capsule(
    c: Vec, r: float, a: Vec, b: Vec, cap_r: float
) -> Optional[Contact]:
    """Circle against a thick line segment (a capsule).

    Used for the robot's horns. Returns a contact whose normal points from
    the capsule toward the circle... no: toward B by the module convention,
    where A is the capsule and B is the circle.
    """
    closest, _ = closest_point_on_segment(c, a, b)
    hit = circle_vs_circle(closest, cap_r, c, r)
    return hit


def circle_vs_rect(
    c: Vec, r: float, rect_centre: Vec, rect_angle: float,
    half_w: float, half_h: float,
) -> Optional[Contact]:
    """Circle against an oriented rectangle.

    Works in the rectangle's local frame, where the test is axis-aligned and
    exact, then transforms the result back. `half_w` is the half-extent
    along the rectangle's local +X (forward), `half_h` along local +Y.

    Normal points from the RECTANGLE toward the CIRCLE.
    """
    local = to_local(c, rect_centre, rect_angle)
    lx, ly = local

    # Nearest point on the rectangle to the circle centre.
    nx = clamp(lx, -half_w, half_w)
    ny = clamp(ly, -half_h, half_h)

    dx = lx - nx
    dy = ly - ny
    d2 = dx * dx + dy * dy

    if d2 > EPS:
        # Circle centre is outside the rectangle.
        if d2 >= r * r:
            return None
        dl = math.sqrt(d2)
        n_local = (dx / dl, dy / dl)
        depth = r - dl
        contact_local = (nx, ny)
    else:
        # Circle centre is INSIDE the rectangle. Push out through the
        # nearest face. Without this branch, deep penetrations (which happen
        # at speed with a small timestep budget) resolve in a random
        # direction and the ball tunnels through the body.
        to_right = half_w - lx
        to_left = lx + half_w
        to_top = half_h - ly
        to_bottom = ly + half_h
        min_pen = min(to_right, to_left, to_top, to_bottom)
        if min_pen == to_right:
            n_local = (1.0, 0.0)
            contact_local = (half_w, ly)
        elif min_pen == to_left:
            n_local = (-1.0, 0.0)
            contact_local = (-half_w, ly)
        elif min_pen == to_top:
            n_local = (0.0, 1.0)
            contact_local = (lx, half_h)
        else:
            n_local = (0.0, -1.0)
            contact_local = (lx, -half_h)
        depth = r + min_pen

    normal = rotate(n_local, rect_angle)
    point = to_world(contact_local, rect_centre, rect_angle)
    return Contact(depth, normal, point)


def rect_corners(
    centre: Vec, angle: float, half_w: float, half_h: float
) -> tuple[Vec, Vec, Vec, Vec]:
    c = math.cos(angle)
    s = math.sin(angle)
    cx, cy = centre
    out = []
    for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        lx = sx * half_w
        ly = sy * half_h
        out.append((cx + lx * c - ly * s, cy + lx * s + ly * c))
    return tuple(out)  # type: ignore[return-value]


def _project(corners, axis: Vec) -> tuple[float, float]:
    lo = hi = dot(corners[0], axis)
    for i in range(1, len(corners)):
        p = dot(corners[i], axis)
        if p < lo:
            lo = p
        elif p > hi:
            hi = p
    return lo, hi


def rect_vs_rect(
    ca: Vec, aa: float, hwa: float, hha: float,
    cb: Vec, ab: float, hwb: float, hhb: float,
) -> Optional[Contact]:
    """Separating-axis test between two oriented rectangles.

    Returns the minimum-translation contact. The contact point is taken as
    the vertex of one rectangle lying deepest inside the other -- an
    approximation, but an adequate one for robot-robot shoving, where the
    exact manifold matters far less than the normal and depth.

    Normal points from A toward B.
    """
    corners_a = rect_corners(ca, aa, hwa, hha)
    corners_b = rect_corners(cb, ab, hwb, hhb)

    axes = (
        from_angle(aa), from_angle(aa + math.pi / 2),
        from_angle(ab), from_angle(ab + math.pi / 2),
    )

    best_depth = float("inf")
    best_axis: Vec = (1.0, 0.0)

    for axis in axes:
        lo_a, hi_a = _project(corners_a, axis)
        lo_b, hi_b = _project(corners_b, axis)
        if hi_a < lo_b or hi_b < lo_a:
            return None  # separating axis found
        overlap = min(hi_a, hi_b) - max(lo_a, lo_b)
        if overlap < best_depth:
            best_depth = overlap
            best_axis = axis

    # Orient the axis from A toward B.
    if dot(sub(cb, ca), best_axis) < 0.0:
        best_axis = (-best_axis[0], -best_axis[1])

    # Contact point: B's corner furthest along -normal (deepest into A).
    deepest = corners_b[0]
    best_proj = dot(corners_b[0], best_axis)
    for i in range(1, 4):
        p = dot(corners_b[i], best_axis)
        if p < best_proj:
            best_proj = p
            deepest = corners_b[i]

    return Contact(best_depth, best_axis, deepest)


# ---------------------------------------------------------------------------
# Segment intersection -- used for goal-line crossing tests
# ---------------------------------------------------------------------------

def segments_intersect(
    p1: Vec, p2: Vec, q1: Vec, q2: Vec
) -> Optional[Vec]:
    """Intersection point of segments p1p2 and q1q2, or None."""
    r = sub(p2, p1)
    s = sub(q2, q1)
    denom = cross(r, s)
    if abs(denom) < EPS:
        return None  # parallel or collinear
    qp = sub(q1, p1)
    t = cross(qp, s) / denom
    u = cross(qp, r) / denom
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return (p1[0] + r[0] * t, p1[1] + r[1] * t)
    return None


def segment_crosses_x(
    p1: Vec, p2: Vec, x: float, y_lo: float, y_hi: float
) -> Optional[Vec]:
    """Where a moving point's path crosses the vertical line X = x,
    within the y band [y_lo, y_hi]. This is the goal test.

    Handles the swept path rather than the instantaneous position, so a ball
    travelling fast enough to jump the goal mouth in one timestep is still
    detected.
    """
    x1, y1 = p1
    x2, y2 = p2
    if (x1 - x) * (x2 - x) > 0.0:
        return None  # both on the same side, no crossing
    if abs(x2 - x1) < EPS:
        return None
    t = (x - x1) / (x2 - x1)
    if t < 0.0 or t > 1.0:
        return None
    y = y1 + (y2 - y1) * t
    if y_lo <= y <= y_hi:
        return (x, y)
    return None
