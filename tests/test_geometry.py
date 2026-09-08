"""Tests for the geometry primitives.

Run:  python -m tests.test_geometry
"""

import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim import geometry as g


FAILURES = []


def check(name, cond, detail=""):
    if cond:
        print(f"  ok   {name}")
    else:
        print(f"  FAIL {name}  {detail}")
        FAILURES.append(name)


def close(a, b, tol=1e-9):
    return abs(a - b) < tol


def vclose(a, b, tol=1e-9):
    return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol


# ---------------------------------------------------------------------------

def test_angles():
    print("angles")
    check("wrap pi stays pi", close(g.wrap_angle(math.pi), math.pi))
    check("wrap 3pi -> pi", close(abs(g.wrap_angle(3 * math.pi)), math.pi))
    check("wrap -3pi -> pi", close(abs(g.wrap_angle(-3 * math.pi)), math.pi))
    check("wrap 0", close(g.wrap_angle(0.0), 0.0))
    check("diff across branch cut",
          close(g.angle_diff(0.1, -0.1), 0.2),
          f"got {g.angle_diff(0.1, -0.1)}")
    d = g.angle_diff(-3.0, 3.0)
    check("diff wraps the short way", abs(d) < math.pi, f"got {d}")


def test_frames():
    print("frame transforms")
    origin = (1.0, 2.0)
    ang = 0.7
    pt = (3.5, -0.25)
    loc = g.to_local(pt, origin, ang)
    back = g.to_world(loc, origin, ang)
    check("to_local/to_world round trip", vclose(back, pt, 1e-12))

    # A point straight ahead of a body should have local y == 0.
    body_ang = 1.234
    ahead = g.to_world((0.5, 0.0), origin, body_ang)
    loc2 = g.to_local(ahead, origin, body_ang)
    check("forward point has zero lateral offset", close(loc2[1], 0.0, 1e-12))
    check("forward point keeps its range", close(loc2[0], 0.5, 1e-12))


def test_point_velocity():
    print("point velocity")
    # Pure rotation about the origin: a point at +x moves in +y.
    v = g.point_velocity((0.0, 0.0), 2.0, (1.0, 0.0))
    check("omega x r gives +y", vclose(v, (0.0, 2.0)))
    v = g.point_velocity((0.0, 0.0), 2.0, (0.0, 1.0))
    check("omega x r gives -x", vclose(v, (-2.0, 0.0)))
    # Translation adds directly.
    v = g.point_velocity((1.0, -1.0), 0.0, (5.0, 5.0))
    check("pure translation", vclose(v, (1.0, -1.0)))


def test_circle_circle():
    print("circle vs circle")
    c = g.circle_vs_circle((0.0, 0.0), 1.0, (1.5, 0.0), 1.0)
    check("overlap detected", c is not None)
    check("depth correct", close(c.depth, 0.5))
    check("normal points A->B", vclose(c.normal, (1.0, 0.0)))
    check("no contact when apart",
          g.circle_vs_circle((0.0, 0.0), 1.0, (3.0, 0.0), 1.0) is None)
    check("touching exactly is not a contact",
          g.circle_vs_circle((0.0, 0.0), 1.0, (2.0, 0.0), 1.0) is None)
    # Concentric must not divide by zero.
    c = g.circle_vs_circle((0.0, 0.0), 1.0, (0.0, 0.0), 1.0)
    check("concentric handled", c is not None and close(c.depth, 2.0))


def test_circle_rect():
    print("circle vs rect")
    # Axis-aligned rect 2x1 (half extents 1.0, 0.5) at the origin.
    c = g.circle_vs_rect((1.4, 0.0), 0.5, (0.0, 0.0), 0.0, 1.0, 0.5)
    check("side overlap detected", c is not None)
    check("side depth", close(c.depth, 0.1), f"got {c.depth}")
    check("side normal +x", vclose(c.normal, (1.0, 0.0)))

    check("clear of rect",
          g.circle_vs_rect((3.0, 0.0), 0.5, (0.0, 0.0), 0.0, 1.0, 0.5) is None)

    # Corner case: circle near a corner should get a diagonal normal.
    c = g.circle_vs_rect((1.2, 0.7), 0.5, (0.0, 0.0), 0.0, 1.0, 0.5)
    check("corner contact", c is not None)
    check("corner normal is diagonal",
          c is not None and c.normal[0] > 0.1 and c.normal[1] > 0.1)

    # Deep penetration: centre inside the rect must push out the near face,
    # not in an arbitrary direction. This is the anti-tunnelling branch.
    c = g.circle_vs_rect((0.9, 0.0), 0.2, (0.0, 0.0), 0.0, 1.0, 0.5)
    check("centre-inside resolves through nearest face",
          c is not None and vclose(c.normal, (1.0, 0.0)),
          f"normal {c.normal if c else None}")
    check("centre-inside depth includes radius",
          c is not None and close(c.depth, 0.2 + 0.1), f"got {c.depth}")

    # Rotated rect: rotating both the rect and the query point by the same
    # angle must give the same depth.
    ang = 0.9
    p = g.rotate((1.4, 0.0), ang)
    c2 = g.circle_vs_rect(p, 0.5, (0.0, 0.0), ang, 1.0, 0.5)
    check("rotation invariance of depth",
          c2 is not None and close(c2.depth, 0.1, 1e-9), f"got {c2.depth}")


def test_circle_capsule():
    print("circle vs capsule (horn)")
    a = (0.0, 0.0)
    b = (1.0, 0.0)
    c = g.circle_vs_capsule((0.5, 0.15), 0.1, a, b, 0.05)
    check("side of bar", c is not None)
    check("depth", c is not None and close(c.depth, 0.0), f"got {c.depth if c else None}")
    c = g.circle_vs_capsule((0.5, 0.10), 0.1, a, b, 0.05)
    check("closer contact detected", c is not None and c.depth > 0.0)
    check("normal points away from bar",
          c is not None and c.normal[1] > 0.9)
    # Past the end of the bar, the cap is a rounded end.
    c = g.circle_vs_capsule((1.12, 0.0), 0.1, a, b, 0.05)
    check("rounded end cap", c is not None and c.depth > 0.0)
    check("beyond the cap is clear",
          g.circle_vs_capsule((1.3, 0.0), 0.1, a, b, 0.05) is None)


def test_rect_rect():
    print("rect vs rect (SAT)")
    c = g.rect_vs_rect((0.0, 0.0), 0.0, 1.0, 0.5,
                       (1.8, 0.0), 0.0, 1.0, 0.5)
    check("overlap detected", c is not None)
    check("depth", c is not None and close(c.depth, 0.2), f"got {c.depth if c else None}")
    check("normal A->B", c is not None and c.normal[0] > 0.9)
    check("clear when apart",
          g.rect_vs_rect((0.0, 0.0), 0.0, 1.0, 0.5,
                         (2.5, 0.0), 0.0, 1.0, 0.5) is None)
    # A rotated rect can overlap where an axis-aligned one would not.
    c = g.rect_vs_rect((0.0, 0.0), 0.0, 1.0, 0.5,
                       (2.05, 0.0), math.pi / 4, 1.0, 0.5)
    check("rotated overlap detected", c is not None)


def test_segment_crossing():
    print("goal-line crossing")
    hit = g.segment_crosses_x((-0.1, 0.0), (0.1, 0.0), 0.0, -0.2, 0.2)
    check("straight crossing", hit is not None and vclose(hit, (0.0, 0.0)))
    check("outside the mouth misses",
          g.segment_crosses_x((-0.1, 0.5), (0.1, 0.5), 0.0, -0.2, 0.2) is None)
    check("same side is no crossing",
          g.segment_crosses_x((0.1, 0.0), (0.3, 0.0), 0.0, -0.2, 0.2) is None)
    # A very fast ball jumping the line in one step must still register.
    hit = g.segment_crosses_x((-1.0, 0.05), (1.0, -0.05), 0.0, -0.2, 0.2)
    check("fast ball still caught", hit is not None)
    # Diagonal entry lands at the interpolated y.
    hit = g.segment_crosses_x((-1.0, -1.0), (1.0, 1.0), 0.0, -0.2, 0.2)
    check("interpolated y", hit is not None and close(hit[1], 0.0, 1e-12))


def test_closest_point():
    print("closest point on segment")
    p, t = g.closest_point_on_segment((0.5, 1.0), (0.0, 0.0), (1.0, 0.0))
    check("perpendicular foot", vclose(p, (0.5, 0.0)) and close(t, 0.5))
    p, t = g.closest_point_on_segment((-1.0, 1.0), (0.0, 0.0), (1.0, 0.0))
    check("clamped to start", vclose(p, (0.0, 0.0)) and close(t, 0.0))
    p, t = g.closest_point_on_segment((2.0, 1.0), (0.0, 0.0), (1.0, 0.0))
    check("clamped to end", vclose(p, (1.0, 0.0)) and close(t, 1.0))
    p, t = g.closest_point_on_segment((1.0, 1.0), (2.0, 2.0), (2.0, 2.0))
    check("degenerate segment", vclose(p, (2.0, 2.0)))


def main():
    for fn in (test_angles, test_frames, test_point_velocity,
               test_circle_circle, test_circle_rect, test_circle_capsule,
               test_rect_rect, test_segment_crossing, test_closest_point):
        fn()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED: {', '.join(FAILURES)}")
        return 1
    print("all geometry tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
