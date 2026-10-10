"""Generate the STLs for the robots' 3D-printed parts.

    pip install manifold3d trimesh
    python hardware/print/make_printed_parts.py

Each part is written already lying in its print orientation (z = up). Units
are millimetres. The parts fit the laser-cut side plates and bottom deck in
hardware/laser/ as already cut: axle holes 19 mm, M3 holes on a 40 mm square
around each axle, deck edge rows of M3 holes 10 mm in from the ends.

Parts marked PROVISIONAL depend on motor dimensions nobody has measured yet.
Measure the motor, change the MOTOR_* constants, and re-run before printing
them.
"""

from __future__ import annotations

import math
import os

import numpy as np
import trimesh
from manifold3d import CrossSection, JoinType, Manifold, OpType

HERE = os.path.dirname(os.path.abspath(__file__))
SEG = 96

# ---------------------------------------------------------------- shared sizes
PLY = 6.0
HOLE_SQ = 20.0           # plate M3 holes at (+-20, +-20) around each axle
M3 = 3.4
M3_TAP = 2.8             # M3 screw self-taps into this
M3_NUT_AF = 5.8          # nut pocket across flats (M3 nut is 5.5)
M3_NUT_T = 2.8
AXLE_D = 6.0

# wheel
WHEEL_RIM_D = 90.0       # tyre adds 5 all round -> 100 = config WHEEL_RADIUS_M * 2
WHEEL_W = 24.0
LIP = 1.0                # rim lips that keep the tyre on
BEARING_D = 19.3         # 626ZZ is 19.0; printed holes come out small
BEARING_W = 6.0
GT2_TEETH = 20           # fixed by the 400 mm belt and the 180 mm axle spacing
PULLEY_TEETH_L = 7.0     # 6 mm belt + 1 mm play
PULLEY_FLANGE_D = 16.0
PULLEY_FLANGE_T = 1.0

# motor (PROVISIONAL: measure and replace)
MOTOR_SHAFT_D = 6.0
MOTOR_SHAFT_FLAT = 2.5   # D-flat distance from axis (6 mm shaft, 0.5 mm flat)
MOTOR_BOSS_D = 14.0      # clearance for the gearbox's front bearing boss
MOTOR_HOLE_PCD = 25.0    # face screw circle
MOTOR_HOLES = 4
MOTOR_HOLE_ANGLE0 = 45.0

# tyre
TYRE_ID = WHEEL_RIM_D - 1.0   # 1 mm smaller so the TPU grips the rim
TYRE_OD = 100.0
TYRE_W = WHEEL_W - 2 * LIP


# ---------------------------------------------------------------- helpers
def cyl(d, h, z0=0.0, x=0.0, y=0.0, seg=SEG):
    return Manifold.cylinder(h, d / 2, d / 2, seg).translate((x, y, z0))


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def hex_prism(af, h, z0=0.0, x=0.0, y=0.0):
    r = af / math.sqrt(3)
    pts = [(r * math.cos(math.radians(30 + 60 * i)), r * math.sin(math.radians(30 + 60 * i))) for i in range(6)]
    return Manifold.extrude(CrossSection([pts]), h).translate((x, y, z0))


def rounded_square(w, r, h, z0=0.0):
    cs = CrossSection.square((w - 2 * r, w - 2 * r), True).offset(r, JoinType.Round, 2.0, SEG)
    return Manifold.extrude(cs, h).translate((0, 0, z0))


def profile_extrude(pts_xz, y0, y1):
    """Extrude a polygon drawn in the x-z plane along y from y0 to y1."""
    m = Manifold.extrude(CrossSection([pts_xz]), y1 - y0)   # polygon in x-y, extruded along z
    # map (x, y, z) -> (x, z, y): profile y becomes z, extrusion z becomes y
    return m.transform([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0]]).translate((0, y0, 0))


def gt2_pulley(teeth, length, z0):
    """GT2 pulley teeth: outer circle with semicircular grooves."""
    pitch_d = teeth * 2.0 / math.pi
    od = pitch_d - 0.508
    rg = 0.555
    rc = od / 2 - 0.75 + rg
    body = Manifold.cylinder(length, od / 2, od / 2, SEG)
    grooves = [Manifold.cylinder(length + 2, rg, rg, 24).translate(
        (rc * math.cos(2 * math.pi * i / teeth), rc * math.sin(2 * math.pi * i / teeth), -1))
        for i in range(teeth)]
    return (body - union(grooves)).translate((0, 0, z0))


def union(parts):
    return Manifold.batch_boolean(parts, OpType.Add)


def save(m: Manifold, name: str):
    mesh = m.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                         faces=np.asarray(mesh.tri_verts), process=True)
    assert tm.is_watertight, name
    tm.export(os.path.join(HERE, name))
    lo, hi = tm.bounds
    size = hi - lo
    print(f"wrote {name:<22} {size[0]:6.1f} x {size[1]:6.1f} x {size[2]:5.1f} mm  {tm.volume / 1000:6.1f} cm3")


# ---------------------------------------------------------------- wheel
def wheel_body():
    """Printed outboard face down, pulley pointing up (towards the plate)."""
    rim = cyl(WHEEL_RIM_D, WHEEL_W)
    lips = union([cyl(WHEEL_RIM_D + 2, LIP), cyl(WHEEL_RIM_D + 2, LIP, WHEEL_W - LIP)])
    z = WHEEL_W
    pulley = union([cyl(PULLEY_FLANGE_D, PULLEY_FLANGE_T, z),
                    gt2_pulley(GT2_TEETH, PULLEY_TEETH_L, z + PULLEY_FLANGE_T),
                    cyl(PULLEY_FLANGE_D, PULLEY_FLANGE_T, z + PULLEY_FLANGE_T + PULLEY_TEETH_L)])
    holes = union([cyl(18, WHEEL_W + 2, -1, 28 * math.cos(math.radians(a)), 28 * math.sin(math.radians(a)))
                   for a in range(0, 360, 60)])
    return rim + lips + pulley - holes


def wheel_idle():
    """Spins on a fixed 6 mm axle: two 626ZZ pressed in from the outboard face."""
    total = WHEEL_W + 2 * PULLEY_FLANGE_T + PULLEY_TEETH_L
    w = wheel_body()
    w -= cyl(BEARING_D, 2 * BEARING_W + 0.4 + 1, -1)          # pocket from the outboard face
    w -= cyl(AXLE_D + 1.0, total + 2, -1)                     # clearance through the rest
    save(w, "wheel_idle.stl")


def wheel_drive():
    """PROVISIONAL. Clamps straight onto the motor's 6 mm D-shaft, which enters
    from the pulley end. One M3 x 16 screw, put in through a lightening hole,
    self-taps into the hub and bears on the flat."""
    total = WHEEL_W + 2 * PULLEY_FLANGE_T + PULLEY_TEETH_L
    w = wheel_body()
    d = MOTOR_SHAFT_D + 0.2
    dbore = cyl(d, total + 2, -1) ^ box(-d, MOTOR_SHAFT_FLAT + 0.1, -d, d, -2, total + 2)
    w -= dbore
    # radial screw towards the flat (+x), in the lightening hole at 0 deg
    screw = cyl(M3_TAP, 22).rotate((0, 90, 0)).translate((0, 0, 21))
    w -= screw
    save(w, "wheel_drive_PROVISIONAL.stl")


def tyre():
    t = cyl(TYRE_OD, TYRE_W) - cyl(TYRE_ID, TYRE_W + 2, -1)
    # shallow tread grooves
    grooves = union([box(-1, 1, TYRE_OD / 2 - 1.2, TYRE_OD / 2 + 1, -1, TYRE_W + 1).rotate((0, 0, a))
                     for a in range(0, 360, 10)])
    save(t - grooves, "tyre_TPU.stl")


# ---------------------------------------------------------------- axle mount
def axle_mount():
    """Holds the idle wheel's fixed axle. Flange on the INSIDE of the plate,
    boss filling the 19 mm hole, 1 mm thrust washer standing proud of the
    outside face. Printed flange-down."""
    t = 8.0
    m = rounded_square(46, 4, t)
    m += cyl(18.8, PLY - 0.2, t)
    m += cyl(10.0, 1.0, t + PLY - 0.2)
    top = t + PLY + 0.8
    m -= cyl(AXLE_D + 0.1, top + 2, -1)
    for sx in (-1, 1):
        for sy in (-1, 1):
            m -= cyl(M3, t + 2, -1, sx * HOLE_SQ, sy * HOLE_SQ)
            m -= hex_prism(M3_NUT_AF, M3_NUT_T, -0.01, sx * HOLE_SQ, sy * HOLE_SQ)
    # set screw: M3 x 16 in from the flange edge, nut dropped into a slot from below
    m -= cyl(M3, 30).rotate((0, 90, 0)).translate((0, 0, t / 2))
    m -= box(8, 8 + M3_NUT_T, -M3_NUT_AF / 2, M3_NUT_AF / 2, -1, t / 2 + M3_NUT_AF / 2)
    save(m, "axle_mount.stl")


# ---------------------------------------------------------------- motor adapter
def motor_adapter():
    """PROVISIONAL. Bolts the motor face to the inside of the plate. Screw the
    motor on first (heads sit in the counterbores on the plate side), then
    bolt the adapter to the plate through the four 40 mm-square holes."""
    t = 6.0
    m = rounded_square(46, 4, t)
    m -= cyl(MOTOR_BOSS_D, t + 2, -1)
    for sx in (-1, 1):
        for sy in (-1, 1):
            m -= cyl(M3, t + 2, -1, sx * HOLE_SQ, sy * HOLE_SQ)
            m -= hex_prism(M3_NUT_AF, M3_NUT_T, t - M3_NUT_T + 0.01, sx * HOLE_SQ, sy * HOLE_SQ)
    for i in range(MOTOR_HOLES):
        a = math.radians(MOTOR_HOLE_ANGLE0 + 360 * i / MOTOR_HOLES)
        x, y = MOTOR_HOLE_PCD / 2 * math.cos(a), MOTOR_HOLE_PCD / 2 * math.sin(a)
        m -= cyl(M3, t + 2, -1, x, y)
        m -= cyl(6.0, 3.0, -0.01, x, y)          # head counterbore, plate side (printed down)
    save(m, "motor_adapter_PROVISIONAL.stl")


# ---------------------------------------------------------------- idler
def idler_roller():
    """Runs on an M3 bolt through the plate's middle slot, outside the plate,
    pressing down on the belt's top run."""
    m = cyl(18, 1) + cyl(12, 7, 1) + cyl(18, 1, 8)
    m -= cyl(3.6, 12, -1)
    save(m, "idler_roller.stl")


# ---------------------------------------------------------------- horns / bumper
def clamp_profile(x_edge, direction, reach):
    """Side profile (x, z) of a part that clamps the deck edge at x_edge and
    sticks out `reach` beyond it. direction = +1 front, -1 rear.
    Deck: z 15..21. Foot on top z 21..29 back 30 mm; lip under z 9..15 back 15 mm."""
    d = direction
    e = x_edge
    pts = [(e - d * 30, 21), (e, 21), (e, 15), (e - d * 15, 15), (e - d * 15, 9), (e, 9), (e, 3),
           (e + d * (reach - 6), 3), (e + d * reach, 9), (e + d * reach, 39), (e + d * (reach - 6), 45),
           (e, 45), (e, 29), (e - d * 30, 29)]
    if d < 0:
        pts = pts[::-1]
    return pts


def horn(side):
    """Left (side=+1) or right (side=-1). Inner face 50 from the centre line,
    so the ball pocket is 100 wide; sticks out 90 past the deck."""
    y0, y1 = (50, 65) if side > 0 else (-65, -50)
    m = profile_extrude(clamp_profile(150, 1, 90), y0, y1)
    yh = 60 * side
    m -= cyl(M3, 60, -1, 140, yh)                       # through lip, deck, foot
    m -= hex_prism(M3_NUT_AF, M3_NUT_T + 0.5, 29 - M3_NUT_T, 140, yh)
    # lay it on its side for printing: the y face goes on the bed
    m = m.translate((0, -y0, 0)).rotate((-90, 0, 0)) if side > 0 else m.translate((0, -y1, 0)).rotate((90, 0, 0))
    lo = m.bounding_box()
    m = m.translate((-lo[0], -lo[1], -lo[2]))
    save(m, f"horn_{'left' if side > 0 else 'right'}.stl")


def rear_bumper():
    m = profile_extrude(clamp_profile(-150, -1, 15), -70, 70)
    for y in (-60, -30, 0, 30, 60):
        m -= cyl(M3, 60, -1, -140, y)
        m -= hex_prism(M3_NUT_AF, M3_NUT_T + 0.5, 29 - M3_NUT_T, -140, y)
    m = m.rotate((-90, 0, 0))
    lo = m.bounding_box()
    save(m.translate((-lo[0], -lo[1], -lo[2])), "rear_bumper.stl")


if __name__ == "__main__":
    wheel_idle()
    wheel_drive()
    tyre()
    axle_mount()
    motor_adapter()
    idler_roller()
    horn(+1)
    horn(-1)
    rear_bumper()
