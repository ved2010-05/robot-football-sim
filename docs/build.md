# Build plan

Two identical robots, one arena, one overhead camera, one PC. Budget for
bought parts: Rs 10,000. Everything else comes from the lab or the X1C.

## Bill of materials (bought)

| Item | Qty | Unit | Total | Source |
|---|---|---|---|---|
| Rhino GB37 12V 330 RPM encoder motor (RMCS-4093) | 4 | 680 | 2,720 | [Robokits](https://robokits.co.in/motors/rhino-gb37-12v-dc-geared-motor/dc-12v-encoder-servo-motors/rhino-gb37-12v-330rpm-2.2kgcm-dc-geared-encoder-servo-motor) |
| BTS7960 43A motor driver | 4 | 256 | 1,024 | [Robokits](https://robokits.co.in/motor-drives-drivers/dc-motor-driver/bts7960-high-power-driver-module-43a) |
| Arducam OV9281 global-shutter USB camera, 70 deg M12 lens (B0332) | 1 | 4,438 | 4,438 | [Fab.to.Lab](https://www.fabtolab.com/arducam-b0332-120fps-global-shutter-usb-camera-board-1mp-ov9281-uvc-webcam-module-low-distortion-m12-lens-microphones) |
| LM2596 buck converter, 12 V to 5 V | 2 | ~40 | ~80 | Robokits / Robu |
| XT60 pairs, blade fuse holders + 10 A fuses, 1 latching E-stop switch per robot | | | ~400 | Robokits / Robu |
| Shipping (two sellers) | | | ~500 | |
| **Total** | | | **~9,160** | |

Robokits ships same working day for orders paid before 4 pm, BlueDart, 1-3
working days after dispatch. Check Fab.to.Lab's delivery estimate at checkout
before paying. Order all four motors from ONE batch so the robots match.

## From the lab (not bought)

- 3 x ESP32 dev kit: one per robot, one on the PC as the radio base station
- 3S LiPo packs and charger (already have)
- FlySky transmitter and receiver (already have). The receiver goes on the
  human robot; that robot's ESP32 reads it over iBUS, drives its motors, and
  also forwards the stick values to the base station. That is what gives the
  AI the "intent" channel. Without the forwarding, intent must be off.
- 6 mm birch ply, laser-cut: two decks per robot
- X1C prints: wheels (PETG hub, TPU tyre), belt pulleys, motor clamps, horns,
  bumper, camera mount
- GT2 belt and bearings from lab stock, or printed gear pair if none
- Golf ball (paint it white or matte orange, see markers)

## Drivetrain

One motor per side, belted to both wheels on that side. Skid steer, 4 wheels,
2 motors, 2 encoders. The sim models exactly this (`DRIVE_MOTORS_PER_SIDE = 1`)
and gives, with estimated motor constants: 1.67 m/s top speed, half speed in
0.26 s, 4.4 rad/s spin (a half turn in 0.71 s), about 3 A per motor while
spinning. The BTS7960 has more than ten times that headroom, which matters in
shoving contests where a motor can sit at stall.

## Camera and markers

The OV9281 is MONOCHROME. Colour blobs will not work. Use contrast instead:

- dark floor, matte
- each robot's top plate black, with WHITE circles: one large centre disc plus
  a heading dot; robot A one heading dot, robot B two, so they cannot be
  confused
- ball white or pale, matte

Mount the lens pointing straight down at about 3.05 m, rigidly (2020 or 4040
extrusion gantry). Lower than that, the 70 deg lens does not see the whole
pitch plus margin; then buy a wider M12 lens and recalibrate distortion.
Lighting: flicker-free DC LED, diffuse, even. Mains-flicker lights beat
against a 100 fps camera.

## Arena

3.6 x 2.2 m, 0.55 m goals, 45 deg corner blocks with 0.35 m legs in all four
corners (measured best; 0.25 and 0.45 were worse). Walls at least bumper
height, rigid.

## Measure on day one, then update config

The sim is only as good as these. Each has a config line waiting for it.

| What | How | Config |
|---|---|---|
| Robot mass | kitchen scale, both robots | `ROBOT_MASS_KG` |
| Motor no-load RPM, stall current | bench PSU at 12 V, clamp meter | `motors.py` rhino_gb37_330 |
| Encoder counts per output rev | turn the wheel one turn by hand | `encoder_cpr_motor` |
| Top speed, 0-to-half-speed time | drive straight, film from above | compare to 1.67 m/s, 0.26 s |
| Spin rate | spin on the spot, film | compare to 4.4 rad/s |
| Camera height, lens distortion | tape measure, checkerboard | `CAM_HEIGHT_M`, calibration |
| Loop latency | the AI measures it online | `AUTO_LATENCY_ID` |
| Ball rolling deceleration | roll it, film it | `BALL_ROLL_DECEL` |
