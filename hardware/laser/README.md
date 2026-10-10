# Laser-cut parts for the robots

Everything the two robots need from the laser cutter. Material is 6 mm birch
ply except the markers. Units are millimetres. In every SVG, **red is cut** and
**blue is a light score** (engrave at low power, or turn the layer off).

| File | Cut | Material | What it is |
|---|---|---|---|
| `test_coupon.svg` | 1 | 6 mm ply | **Cut this first.** Checks slot, bearing and screw-hole fit |
| `side_plate.svg` | 4 | 6 mm ply | Two per robot. Left and right are the same part |
| `end_panel_front.svg` | 2 | 6 mm ply | Front wall, one per robot |
| `end_panel_rear.svg` | 2 | 6 mm ply | Rear wall, one per robot, with an opening for the charge lead and USB cable |
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
   lid (300 x 250, painted matte black, white markers on top)   z 104-110
   |                                         |
   side plate + front/rear panels (a closed box)                z 21-104
   |                                         |
   bottom deck (300 x 172)                                      z 15-21
   ----------------------------- floor ---------------------------- z 0
```

The electronics sit in a box closed on all six sides: deck, lid, two side
plates, and the front and rear panels between the plates (114-120 from the
centre, clear of the motor adapters inside and the horn and bumper feet
outside).

- **Walls**: the front and rear panels' side tabs go into the slots near the
  ends of the side plates, making a four-wall box. Lower it onto the deck with
  every bottom tab in its slot; the lid goes on the top tabs. Each joint is held
  by M3 x 16 screws through the deck or lid into the T-slots in the wall edges,
  with an M3 nut dropped into each slot's pocket (8 screws in the deck, 8 in
  the lid). The front and rear panels' screws are 66 out from the centre line
  so none sits under a white heading dot.
- **Axles and motors**: the 19 mm holes and the four M3 holes around each
  take the printed motor adapter (powered wheel) or axle mount (unpowered
  wheel). Pulleys, belt and wheels are all outside the plate, so nothing but
  the motor sits inside it. See [../print/README.md](../print/README.md).
- **Belt idler**: the short slot in the middle of each plate takes an M3 bolt
  with the printed idler roller, outside the plate, pressing down on the
  belt's top run.
- **Bottom deck**: the hole grids either side of the battery are for the two
  BTS7960 boards, the ESP32 and the buck converter (screws, standoffs or zip
  ties). The two slots at the ends take a Velcro strap running lengthwise over
  the 3S pack, leaving room beside it for the motor drivers. The rows of
  holes along the front and rear edges are for the printed horns and bumper.
- **Lid**: painted matte black. Stick the white discs from
  `markers_white.svg` inside the scored circles. The arrow points forward
  (towards the horns). The 22 mm hole over the rear motor is for the latching
  E-stop, whose body must hang no more than 38 mm below the lid. If its ring is
  yellow, paint it black: the camera is monochrome and a bright ring could look
  like a marker.
- **Rear panel**: the opening lets the battery's charge lead and the ESP32's
  USB cable out without taking the lid off.

## Sizes this design assumes

All of them are constants at the top of `make_laser_parts.py`.

| Size | Value | Comes from |
|---|---|---|
| Body | 300 x 250 x 110 tall | `config.ROBOT_LENGTH_M`, `ROBOT_WIDTH_M`, `MARKER_HEIGHT_M` |
| Lid underside | 104, clearing the 100 mm wheels by 4 | lowest the wheels allow |
| Wheels | 100 diameter, centred 102 from the centre line | `WHEEL_RADIUS_M`, `TRACK_WIDTH_M` |
| Axle spacing | 180 | 400 mm GT2 belt on 20-tooth pulleys |
| Ground clearance under the deck | 15 | less than the 43 mm ball, so it can't get under |
| Room for each motor | about 134 from its face to the opposite axle mount | Measure the IG32 with its encoder |
| Bearings | 626ZZ, 6 x 19 x 6 | in the unpowered wheels |

Three things to check against the simulator:

- `config.MARKER_HEIGHT_M` is 0.150, but the lid top is now 0.110. Change it,
  or the camera's height correction will be off by 40 mm.
- `config.WHEELBASE_M` is 0.200, but the belt sets 0.180. Change it to 0.180
  when the robots are built, or buy belts that give 200 (440 mm on 20-tooth
  pulleys) and set `AXLE_X = 100` here.
- The motors on the order sheet are Rhino IG32 300 RPM, not the GB37 330 RPM
  in `motors.py`. Measure them and update `motors.py`.

## Printed, not laser-cut

Wheels with built-in pulleys, TPU tyres, motor adapters, axle mounts, idler
rollers, horns and rear bumper: [../print](../print/README.md).
