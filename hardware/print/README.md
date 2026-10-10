# 3D-printed parts for the robots

STLs for every printed part of the two robots, already in print orientation.
They fit the laser-cut side plates and decks in `../laser/` exactly as cut.
Regenerate with:

```bash
pip install manifold3d trimesh
python hardware/print/make_printed_parts.py
```

| File | Print (both robots) | Material | What it is |
|---|---|---|---|
| `wheel_idle.stl` | 4 | PLA or PPA-CF | Unpowered wheel with its 20-tooth GT2 pulley built in. Two 626ZZ bearings press into the outer face |
| `wheel_drive_PROVISIONAL.stl` | 4 | PPA-CF | Powered wheel, same shape, clamps onto the motor's 6 mm D-shaft |
| `tyre_TPU.stl` | 8 | TPU | Stretch over the wheel rim, between the two lips |
| `axle_mount.stl` | 4 | PLA or PPA-CF | Holds the unpowered wheel's fixed 6 mm axle in the plate |
| `motor_adapter_PROVISIONAL.stl` | 4 | PLA or PPA-CF | Bolts the motor face to the inside of the plate |
| `idler_roller.stl` | 4 | PLA | Belt tensioner on an M3 bolt through the plate's middle slot |
| `horn_left.stl`, `horn_right.stl` | 2 each | PLA | The ball pocket. Clamps the deck's front edge |
| `rear_bumper.stl` | 2 | TPU or PLA | Clamps the deck's rear edge |

Suggested settings: 0.2 mm layers, 4 walls, 30 % infill for wheels, adapters
and mounts; 3 walls, 20 % for horns and bumper. Dry the PPA-CF first.

## Don't print the PROVISIONAL parts yet

`wheel_drive` and `motor_adapter` depend on the motor, which hasn't been
measured. Before printing them, measure one Rhino IG32 and set these at the top
of `make_printed_parts.py`:

| Constant | What to measure | Placeholder |
|---|---|---|
| `MOTOR_SHAFT_D`, `MOTOR_SHAFT_FLAT` | shaft diameter, and distance from the axis to the flat | 6.0, 2.5 |
| `MOTOR_BOSS_D` | diameter of the raised ring around the shaft on the motor face | 14 |
| `MOTOR_HOLE_PCD`, `MOTOR_HOLES`, `MOTOR_HOLE_ANGLE0` | screw circle diameter, number of screws, angle of the first | 25, 4, 45 |

**The shaft must stick out at least about 27 mm from the motor face.** It
passes the 6 mm adapter, the 6 mm plate, a 1 mm gap and the 9 mm pulley, and
needs about 5 mm inside the wheel for the clamp screw. If it is shorter, tell
me the length: the adapter can get thinner, or the wheel can be redesigned to
use the "6 mm shaft hub coupling" from the order sheet.

## How they go together (one side of one robot)

Looking along an axle from the centre of the robot outwards:

```
 inside of robot      plate (6 mm)        outside
 motor -> adapter ->  |  19 mm hole |  -> 1 mm -> pulley (9) -> wheel (24) + tyre
 axle_mount ------->  |  boss fills |  -> 1 mm washer -> pulley -> wheel -> shaft collar
```

- **Powered wheel** (front on the left side, rear on the right side): screw
  the motor to `motor_adapter` first, then bolt the adapter to the inside of
  the plate through the four M3 holes around the axle hole, nuts in the
  adapter's hex pockets. Slide `wheel_drive` onto the shaft from outside,
  pulley first, leaving a 1 mm gap to the plate, and tighten an M3 x 16 screw
  through the lightening hole onto the shaft's flat.
- **Unpowered wheel**: push `axle_mount`'s boss into the plate hole from inside
  and bolt it through the same four holes. Push a 60 mm length of 6 mm steel
  rod through it and lock it with an M3 x 16 screw and nut in the flange's
  edge slot. Press two 626ZZ bearings into `wheel_idle`'s outer face, slide
  the wheel onto the rod, and fit a 6 mm shaft collar outside it.
- **Belt**: loops round the two pulleys on the outside of the plate, between
  plate and wheels. Put `idler_roller` on an M3 bolt through the middle slot,
  outside the plate, and slide it down until the belt is just taut.
- **Horns**: each clamps the deck's front edge (top foot and bottom lip) and is
  held by one M3 x 25 screw up through the lip, the deck hole 60 from the centre line
  and the foot, with the nut in the foot's pocket. The bars' inner faces are
  100 apart and stick out 90.
- **Rear bumper**: clamps the rear edge the same way with five M3 x 25 screws.

## What this changes in the simulator

When the robots are built, update `config.py`:

- `WHEELBASE_M` 0.200 -> **0.180** (set by the 400 mm belt)
- `TRACK_WIDTH_M` 0.200 -> **0.204** (wheel centres 102 from the centre line)
- `HORN_WIDTH_M` 0.010 -> **0.015** (the inner gap stays 100)
- `MARKER_HEIGHT_M` 0.150 -> **0.110** (the lid top is now 110 above the floor)

## Not used in this design

The metal GT2 pulleys and the 6 mm hub couplings on the order sheet: the
pulleys are built into the wheels. They're spares unless the motor shaft turns
out too short (see above).

## Other hardware for both robots

8 x 626ZZ bearings, 4 x 60 mm of 6 mm steel rod, 4 x 6 mm shaft collars, about
90 M3 x 16 screws and nuts (walls, mounts, adapters, idlers, lid), and 14
M3 x 25 with nuts (horns and bumpers: 20 mm of lip, deck and foot).
