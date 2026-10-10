"""Generate the laser-cut SVGs for the two robots.

    python hardware/laser/make_laser_parts.py

Writes one SVG per part next to this script. Units are millimetres.
Red (#FF0000) is cut, blue (#0000FF) is a light score/engrave line.

Every size the parts depend on is a constant below, so a change to the
motor, belt, wheels or fit is one edit and a re-run.
"""

from __future__ import annotations

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- material
PLY = 6.0            # birch ply thickness. Tabs and slots are this deep.
SLOT_W = 6.0         # slot width for a PLY tab. Tune from test_coupon.svg.
BEARING_D = 19.0     # 626ZZ outer diameter (6 x 19 x 6). Tune from the coupon.
M3_D = 3.4           # M3 clearance hole

# ---------------------------------------------------------------- robot
BODY_L = 300.0       # config.ROBOT_LENGTH_M
BODY_W = 250.0       # config.ROBOT_WIDTH_M (the lid covers the wheels)
HEIGHT = 110.0       # top of lid = config.MARKER_HEIGHT_M. Lowest that clears
                     # the 100 mm wheels under the lid with 4 mm to spare
CLEARANCE = 15.0     # floor to underside of the bottom deck (ball is 43 mm)
WHEEL_R = 50.0       # config.WHEEL_RADIUS_M
AXLE_X = 90.0        # axles at +-90: 180 mm apart, set by the 400 mm GT2 belt
                     # on 20-tooth pulleys: (400 - 20*2) / 2 = 180
PLATE_Y = 74.0       # inner face of each side plate (outer face at 80).
                     # Wheels are centred at +-100 (config.TRACK_WIDTH_M).
DECK_HALF_W = 86.0   # bottom deck stops 2 mm short of the wheels
PLATE_HALF_L = 140.0
MOTOR_PCD = 20.0     # mount holes at +-20 mm square around each axle, for a
                     # printed motor bracket or printed outer bearing block

# derived
DECK_TOP = CLEARANCE + PLY                    # 21
LID_BOTTOM = HEIGHT - PLY                     # 144
PLATE_H = LID_BOTTOM - DECK_TOP               # 123
AXLE_Z = WHEEL_R - DECK_TOP                   # axle height above plate bottom: 29
TAB_W = 30.0
TAB_X = (-115.0, 0.0, 115.0)
TSLOT_X = (-60.0, 60.0)
TSLOT_DEPTH = 14.0   # M3 x 16 screw: 6 through the deck + 10 into the plate
NUT_W, NUT_T, NUT_Z = 5.7, 2.6, 6.0           # M3 nut pocket

# front and rear panels close the box between the side plates
END_X = 114.0        # inner face (outer face at 120): clears the 46 mm motor
                     # adapter and axle mount (to 113) and the horn and bumper
                     # feet (from 120)
END_TAB_Y = (-40.0, 40.0)
END_TAB_W = 24.0
END_SIDE_TAB = (31.0, 51.0)   # side tabs into the plates, height above plate bottom
END_TSLOT_U = (-66.0, 66.0)   # screws kept clear of the heading dots on the lid

# markers (config.MARKER_*)
MARK_CENTRE_D = 110.0
MARK_DOT_D = 55.0
MARK_DOT_X = 105.0
MARK_B_DOT_Y = 35.0  # robot B: two dots side by side, 15 mm apart

ESTOP_D = 22.0       # standard 22 mm panel hole
ESTOP_X = -90.0      # inside the box, over the rear motor: body may hang
                     # at most 38 mm below the lid

MARGIN = 2.0
CUT = 'stroke="#FF0000" stroke-width="0.1" fill="none"'
ENG = 'stroke="#0000FF" stroke-width="0.1" fill="none"'


def f(v: float) -> str:
    return f"{v:.3f}".rstrip("0").rstrip(".")


def poly_path(pts) -> str:
    return "M " + " L ".join(f"{f(x)} {f(y)}" for x, y in pts) + " Z"


def circle(cx, cy, d) -> str:
    return f'<circle cx="{f(cx)}" cy="{f(cy)}" r="{f(d / 2)}"/>'


def rrect(x0, y0, x1, y1, r) -> str:
    return (f'<rect x="{f(x0)}" y="{f(y0)}" width="{f(x1 - x0)}" height="{f(y1 - y0)}" '
            f'rx="{f(r)}" ry="{f(r)}"/>')


def slot(x0, y0, x1, y1, w) -> str:
    """Stadium slot between two centre points."""
    a = math.atan2(y1 - y0, x1 - x0)
    nx, ny = -math.sin(a) * w / 2, math.cos(a) * w / 2
    r = f(w / 2)
    return (f'<path d="M {f(x0 + nx)} {f(y0 + ny)} L {f(x1 + nx)} {f(y1 + ny)} '
            f'A {r} {r} 0 0 0 {f(x1 - nx)} {f(y1 - ny)} L {f(x0 - nx)} {f(y0 - ny)} '
            f'A {r} {r} 0 0 0 {f(x0 + nx)} {f(y0 + ny)} Z"/>')


def write(name, w, h, cut, eng=()):
    body = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{f(w)}mm" height="{f(h)}mm" '
            f'viewBox="0 0 {f(w)} {f(h)}">',
            f'<g id="cut" {CUT}>', *cut, '</g>']
    if eng:
        body += [f'<g id="engrave" {ENG}>', *eng, '</g>']
    body.append('</svg>\n')
    with open(os.path.join(HERE, name), "w") as fh:
        fh.write("\n".join(body))
    print("wrote", name, f"({f(w)} x {f(h)} mm)")


# ---------------------------------------------------------------- outlines
def walled_outline(half_l, h, tab_x, tab_w, tslot_x, side_tab=None):
    """Outline of an upright panel: tabs and M3 T-slots on its bottom and top
    edges, optional tabs on both ends (z0, z1) that go into the side plates."""
    d = TSLOT_DEPTH
    pts = [(-half_l, 0.0)]
    feats = sorted([(c, "tab") for c in tab_x] + [(c, "t") for c in tslot_x])
    for c, kind in feats:                      # bottom edge, left to right
        if kind == "tab":
            pts += [(c - tab_w / 2, 0), (c - tab_w / 2, -PLY), (c + tab_w / 2, -PLY), (c + tab_w / 2, 0)]
        else:
            s, n = 1.6, NUT_W / 2
            pts += [(c - s, 0), (c - s, NUT_Z), (c - n, NUT_Z), (c - n, NUT_Z + NUT_T), (c - s, NUT_Z + NUT_T),
                    (c - s, d), (c + s, d), (c + s, NUT_Z + NUT_T), (c + n, NUT_Z + NUT_T), (c + n, NUT_Z),
                    (c + s, NUT_Z), (c + s, 0)]
    pts += [(half_l, 0.0)]
    if side_tab:
        z0, z1 = side_tab
        pts += [(half_l, z0), (half_l + PLY, z0), (half_l + PLY, z1), (half_l, z1)]
    pts += [(half_l, h)]
    for c, kind in reversed(feats):            # top edge, right to left
        if kind == "tab":
            pts += [(c + tab_w / 2, h), (c + tab_w / 2, h + PLY), (c - tab_w / 2, h + PLY), (c - tab_w / 2, h)]
        else:
            s, n = 1.6, NUT_W / 2
            pts += [(c + s, h), (c + s, h - NUT_Z), (c + n, h - NUT_Z), (c + n, h - NUT_Z - NUT_T),
                    (c + s, h - NUT_Z - NUT_T), (c + s, h - d), (c - s, h - d), (c - s, h - NUT_Z - NUT_T),
                    (c - n, h - NUT_Z - NUT_T), (c - n, h - NUT_Z), (c - s, h - NUT_Z), (c - s, h)]
    pts += [(-half_l, h)]
    if side_tab:
        z0, z1 = side_tab
        pts += [(-half_l, z1), (-half_l - PLY, z1), (-half_l - PLY, z0), (-half_l, z0)]
    return pts


# ---------------------------------------------------------------- side plate
def side_plate():
    """Vertical plate along the robot's length. x = along robot, z = up."""
    h = PLATE_H
    pts = walled_outline(PLATE_HALF_L, h, TAB_X, TAB_W, TSLOT_X)
    W = 2 * PLATE_HALF_L + 2 * MARGIN
    H = h + 2 * PLY + 2 * MARGIN
    X = lambda x: x + PLATE_HALF_L + MARGIN
    Y = lambda z: h + PLY + MARGIN - z
    cut = [f'<path d="{poly_path([(X(x), Y(z)) for x, z in pts])}"/>']
    for ax in (-AXLE_X, AXLE_X):
        cut.append(circle(X(ax), Y(AXLE_Z), BEARING_D))
        for dx in (-MOTOR_PCD, MOTOR_PCD):
            for dz in (-MOTOR_PCD, MOTOR_PCD):
                cut.append(circle(X(ax + dx), Y(AXLE_Z + dz), M3_D))
    # belt idler: an M3 bolt with a small bearing pressing down on the top run
    cut.append(slot(X(0), Y(37), X(0), Y(50), M3_D))
    # wiring window, between the idler slot and the top T-slots
    cut.append(rrect(X(-40), Y(71), X(40), Y(56), 5))
    # slots for the front and rear panels' side tabs
    for sx in (-1, 1):
        x0 = sx * END_X if sx > 0 else -END_X - PLY
        cut.append(f'<rect x="{f(X(x0) - (SLOT_W - PLY) / 2)}" y="{f(Y(END_SIDE_TAB[1]))}" '
                   f'width="{f(SLOT_W)}" height="{f(END_SIDE_TAB[1] - END_SIDE_TAB[0])}"/>')
    write("side_plate.svg", W, H, cut)


def end_panel(which):
    """Front or rear wall, standing across the robot between the side plates.
    u = across the robot (robot y), z = up."""
    h = PLATE_H
    pts = walled_outline(PLATE_Y, h, END_TAB_Y, END_TAB_W, END_TSLOT_U, END_SIDE_TAB)
    W = 2 * (PLATE_Y + PLY) + 2 * MARGIN
    H = h + 2 * PLY + 2 * MARGIN
    X = lambda u: u + PLATE_Y + PLY + MARGIN
    Y = lambda z: h + PLY + MARGIN - z
    cut = [f'<path d="{poly_path([(X(u), Y(z)) for u, z in pts])}"/>']
    if which == "rear":
        # access for the battery's charge lead and the ESP32's USB cable
        cut.append(rrect(X(-30), Y(64), X(30), Y(40), 4))
    write(f"end_panel_{which}.svg", W, H, cut)


# ---------------------------------------------------------------- decks
def deck_common(half_w, X, Y):
    cut = []
    for sy in (-1, 1):
        y0, y1 = sy * PLATE_Y, sy * (PLATE_Y + PLY)
        for c in TAB_X:
            cut.append(f'<rect x="{f(X(c - TAB_W / 2))}" y="{f(min(Y(y0), Y(y1)) - (SLOT_W - PLY) / 2)}" '
                       f'width="{f(TAB_W)}" height="{f(SLOT_W)}"/>')
        for c in TSLOT_X:
            cut.append(circle(X(c), Y(sy * (PLATE_Y + PLY / 2)), M3_D))
    for sx in (-1, 1):
        x0 = END_X if sx > 0 else -END_X - PLY
        for c in END_TAB_Y:
            ya, yb = sorted((Y(c - END_TAB_W / 2), Y(c + END_TAB_W / 2)))
            cut.append(f'<rect x="{f(X(x0) - (SLOT_W - PLY) / 2)}" y="{f(ya)}" width="{f(SLOT_W)}" height="{f(yb - ya)}"/>')
        for c in END_TSLOT_U:
            cut.append(circle(X(sx * (END_X + PLY / 2)), Y(c), M3_D))
    return cut


def bottom_deck():
    hw, hl = DECK_HALF_W, BODY_L / 2
    W, H = 2 * hl + 2 * MARGIN, 2 * hw + 2 * MARGIN
    X = lambda x: x + hl + MARGIN
    Y = lambda y: hw + MARGIN - y            # +y (robot left) at the top
    cut = [rrect(X(-hl), Y(hw), X(hl), Y(-hw), 6)]
    cut += deck_common(hw, X, Y)
    # front and rear rows: horns, bumper, ball guards
    for x in (-140, 140):
        for y in range(-75, 76, 15):
            cut.append(circle(X(x), Y(y), M3_D))
    # electronics grid either side of the battery (motor drivers, ESP32, buck)
    for sy in (-1, 1):
        for x in range(-60, 61, 10):
            for y in range(35, 66, 10):
                cut.append(circle(X(x), Y(sy * y), M3_D))
    # battery strap slots just past each end of a 3S pack lying along x (up to
    # 105 long), so a 20 mm Velcro strap runs lengthwise over it and leaves the
    # 57 mm either side of the battery clear for the 50 x 50 motor drivers
    for x in (-58, 58):
        cut.append(rrect(X(x - 2), Y(11), X(x + 2), Y(-11), 1.5))
    write("bottom_deck.svg", W, H, cut)


def lid(variant):
    hw, hl = BODY_W / 2, BODY_L / 2
    W, H = 2 * hl + 2 * MARGIN, 2 * hw + 2 * MARGIN
    X = lambda x: x + hl + MARGIN
    Y = lambda y: hw + MARGIN - y
    cut = [rrect(X(-hl), Y(hw), X(hl), Y(-hw), 10)]
    cut += deck_common(hw, X, Y)
    cut.append(circle(X(ESTOP_X), Y(0), ESTOP_D))
    eng = [circle(X(0), Y(0), MARK_CENTRE_D)]
    dots = [(MARK_DOT_X, 0)] if variant == "A" else [(MARK_DOT_X, MARK_B_DOT_Y), (MARK_DOT_X, -MARK_B_DOT_Y)]
    eng += [circle(X(x), Y(y), MARK_DOT_D) for x, y in dots]
    # small forward arrow so the lid cannot go on backwards
    eng.append(f'<path d="M {f(X(-20))} {f(Y(-95))} L {f(X(20))} {f(Y(-95))} '
               f'M {f(X(12))} {f(Y(-90))} L {f(X(20))} {f(Y(-95))} L {f(X(12))} {f(Y(-100))}"/>')
    write(f"lid_robot_{variant}.svg", W, H, cut, eng)


def markers():
    """White matte discs for both robots: 2 centre discs, 3 heading dots."""
    g = 4.0
    W = 2 * MARK_CENTRE_D + 3 * g
    H = MARK_CENTRE_D + MARK_DOT_D + 3 * g
    r, rd = MARK_CENTRE_D / 2, MARK_DOT_D / 2
    cut = [circle(g + r, g + r, MARK_CENTRE_D), circle(2 * g + 3 * r, g + r, MARK_CENTRE_D)]
    y = 2 * g + MARK_CENTRE_D + rd
    cut += [circle(g + rd + i * (MARK_DOT_D + g), y, MARK_DOT_D) for i in range(3)]
    write("markers_white.svg", W, H, cut)


def test_coupon():
    """Cut this first. Left to right: slots 5.8 / 6.0 / 6.2 wide, bearing
    holes 18.8 / 18.9 / 19.0 / 19.1, M3 holes 3.2 / 3.4. The small piece is
    a 30 x 6 tab to try in the slots."""
    W, H = 152.0, 68.0
    cut = [rrect(MARGIN, MARGIN, 110, 64, 3)]
    for i, w in enumerate((5.8, 6.0, 6.2)):
        x = 10 + i * 34
        cut.append(f'<rect x="{f(x)}" y="{f(12 - w / 2)}" width="30" height="{f(w)}"/>')
    for i, d in enumerate((18.8, 18.9, 19.0, 19.1)):
        cut.append(circle(18 + i * 25, 36, d))
    for i, d in enumerate((3.2, 3.4)):
        cut.append(circle(18 + i * 12, 56, d))
    # tab test piece: its 30 mm edge is the same as a plate tab
    cut.append(rrect(118, 20, 148, 60, 0))
    write("test_coupon.svg", W, H, cut)


if __name__ == "__main__":
    side_plate()
    end_panel("front")
    end_panel("rear")
    bottom_deck()
    lid("A")
    lid("B")
    markers()
    test_coupon()
