# Laser-cut parts for the robots

Everything the two robots need from the laser cutter. Material is 6 mm birch
ply except the markers. Units are millimetres. In every SVG, **red is cut** and
**blue is a light score** (engrave at low power, or turn the layer off).

| File | Cut | Material | What it is |
|---|---|---|---|
| `test_coupon.svg` | 1 | 6 mm ply | **Cut this first.** Checks slot, bearing and screw-hole fit |
| `side_plate.svg` | 4 | 6 mm ply | Two per robot. Left and right are the same part |
| `bottom_deck.svg` | 2 | 6 mm ply | One per robot |
| `lid_robot_A.svg` | 1 | 6 mm ply | Robot A's top: one heading dot scored on it |
| `lid_robot_B.svg` | 1 | 6 mm ply | Robot B's top: two heading dots scored on it |
| `markers_white.svg` | 1 | matte white card or vinyl | 2 centre discs (110), 3 heading dots (55) |

Bed size needed: 304 x 254 for the biggest part (the lids).

## Before cutting the real parts

1. Cut `test_coupon.svg`. From left to right it has slots 5.8 / 6.0 / 6.2 wide,
   bearing holes 18.8 / 18.9 / 19.0 / 19.1, and M3 holes 3.2 / 3.4. The
   separate 30 x 40 piece is a tab: push its 30 mm edge into each slot.
2. Pick the slot the tab fits snugly, and the bearing hole a 626ZZ presses into
   by hand without falling out.
3. If those are not 6.0 and 19.0, change `SLOT_W` and `BEARING_D` at the top of
   `make_laser_parts.py` and run it again:

```bash
python hardware/laser/make_laser_parts.py
```

## How it goes together

```
   lid (300 x 250, painted matte black, white markers on top)   z 144-150
   |                                         |
   side plate  -- belt + pulleys inside --   side plate        z 21-144
   |                                         |
   bottom deck (300 x 172)                                      z 15-21
   ----------------------------- floor ---------------------------- z 0
```

- **Side plates** stand on the bottom deck with their tabs in its slots, and the
  lid sits on their top tabs. Each joint is held by an M3 x 16 screw through
  the deck into the T-slot in the plate edge, with an M3 nut dropped into the
  slot's pocket (4 screws per deck, 4 per lid).
- **Axles and motors**: the 19 mm holes and the four M3 holes around each
  take the printed motor adapter (powered wheel) or axle mount (unpowered
  wheel). Pulleys, belt and wheels are all outside the plate, so nothing but
  the motor sits inside it. See [../print/README.md](../print/README.md).
- **Belt idler**: the short slot in the middle of each plate takes an M3 bolt
  with the printed idler roller, outside the plate, pressing down on the
  belt's top run.
- **Bottom deck**: the hole grids either side of the battery are for the two
  BTS7960 boards, the ESP32 and the buck converter (screws, standoffs or zip
  ties). The four slots are for a Velcro strap around the 3S pack. The rows of
  holes along the front and rear edges are for the printed horns and bumper.
- **Lid**: painted matte black. Stick the white discs from
  `markers_white.svg` inside the scored circles. The arrow points forward
  (towards the horns). The 22 mm hole at the back is for the latching
  E-stop. If its ring is yellow, paint it black: the camera is monochrome and
  a bright ring could look like a marker.

## Sizes this design assumes

All of them are constants at the top of `make_laser_parts.py`.

| Size | Value | Comes from |
|---|---|---|
| Body | 300 x 250 x 150 tall | `config.ROBOT_LENGTH_M`, `ROBOT_WIDTH_M`, `MARKER_HEIGHT_M` |
| Wheels | 100 diameter, centred 102 from the centre line | `WHEEL_RADIUS_M`, `TRACK_WIDTH_M` |
| Axle spacing | 180 | 400 mm GT2 belt on 20-tooth pulleys |
| Ground clearance under the deck | 15 | less than the 43 mm ball, so it can't get under |
| Room for each motor | about 134 from its face to the opposite axle mount | Measure the IG32 with its encoder |
| Bearings | 626ZZ, 6 x 19 x 6 | in the unpowered wheels |

Two things to check against the simulator:

- `config.WHEELBASE_M` is 0.200, but the belt sets 0.180. Change it to 0.180
  when the robots are built, or buy belts that give 200 (440 mm on 20-tooth
  pulleys) and set `AXLE_X = 100` here.
- The motors on the order sheet are Rhino IG32 300 RPM, not the GB37 330 RPM
  in `motors.py`. Measure them and update `motors.py`.

## Printed, not laser-cut

Wheels with built-in pulleys, TPU tyres, motor adapters, axle mounts, idler
rollers, horns and rear bumper: [../print](../print/README.md).
