# Robot football: one AI, one person, identical robots

One arena, two mechanically identical skid-steer robots, one ball, one
overhead camera. One robot is driven by a person on an RC transmitter. The
other is driven by an AI running on a PC. Because the machines are the same,
any difference in the result comes from the driver, not the robot.

This repository is the simulator and the AI. The AI is developed here first,
against a simulation that models every delay and source of noise the real
system will have, and then moves to the real robots unchanged through a
hardware interface layer. The robots are designed but not yet built; the
parts list is in [docs/build.md](docs/build.md).

Requires **Python 3.10+** and **numpy**. The game window uses tkinter from the
standard library.

## Play it

```bash
python -m game.main
```

W/S drive, A/D turn, space pause, R restart, F1-F5 overlays, Esc quit.
The window sizes itself to the screen.

```bash
python -m game.main --no-intent      # the AI cannot read your key presses
python -m game.main --speed-cap 0.6  # handicap the AI
```

## The rules of the setup

- **Identical robots.** 300 x 250 mm, 3 kg, two Robokits GB37 330 RPM
  gearmotors each (one per side, belted to both wheels on that side), two
  passive horns at the front forming a pocket. No kicker.
- **The AI senses only what the real system can.** An overhead global-shutter
  camera at 100 fps, both robots' wheel encoders, and optionally the person's
  live stick commands (the "intent channel"). No ground truth.
- **No learning of any kind.** No player profile, nothing fitted, nothing
  carried between matches, no adaptation during a match. The AI plays every
  person the same way.
- **No referee.** The ball goes back to the centre only after a goal. A ball
  that gets stuck stays stuck, so the AI has to deal with it.
- **Arena:** 3.6 x 2.2 m, 0.55 m goals, 45 degree blocks across every corner
  with 0.35 m legs (measured best; see the experiment log).

## Where it stands

Measured with `tools/evaluate.py`: 24 matches of 120 s per opponent,
identical seeds, every number with its standard error. The primary metric is
goal difference against the three human-shaped test opponents, because the
real opponent is a person; the two simple bots only check for regressions.

| opponent | what it is | AI goal difference per match |
|---|---|---|
| human | decides every 0.25 s, holds commands | +0.08 +- 0.28 |
| human_fast | decides every 0.15 s, less sloppy | +0.17 +- 0.16 |
| human_rc | continuous analog steering from a 0.2 s-old view, like an RC pilot | +0.25 +- 0.15 |
| simple | turn on the spot, then drive straight | +0.21 +- 0.21 |
| runner | always driving, small steady turns | +2.62 +- 0.41 |

The AI is ahead of every opponent, but narrowly against the human-shaped
ones. It was developed on a heavier 4-motor robot and has not yet been
retuned for the 2-motor build these numbers use; on the old robot the same
code was 1.75 goals per match further ahead of the human proxies.

**Honest status.** The rollout planner, the reachability analysis and the
opponent model in `ai/` exist, are tested, and currently never run: simpler
behaviours now handle every situation above them. The AI therefore always
aims at the centre of the goal, and reading the opponent's stick commands is
measured to be worth nothing (+0.33 +- 0.51 with it OFF, no detectable
effect). Reconnecting those parts is the next piece of work.

The full record of what was tried, measured, adopted and rejected, including
the ideas that failed, is in [docs/experiments.md](docs/experiments.md).

## Architecture

```
        GROUND TRUTH (1 kHz)                     AI (100 Hz)
  +--------------------------+          +---------------------------+
  | world     ball, robots,  |          | calibration  undistort,   |
  |           walls, chamfers|  --HAL-> |              2-plane homog|
  | chassis   4-wheel slip   |  Sensors | estimator    fixed-lag KF |
  | drivetrain motor, belt,  |          | tactics      striker,     |
  |           encoder, PID   |          |              sweep, shadow|
  +--------------------------+          | controller   -> wheels    |
              ^                         +---------------------------+
              +-------- Actuators ----------------+
```

`sim/sim_backend.py` is the only bridge between the two sides. Nothing under
`ai/` imports the simulator's ground truth; `tests/test_architecture.py`
enforces that in CI. On the real system a different pair of `Sensors` and
`Actuators` objects is passed in and nothing in `ai/` changes.

## What is modelled

**Vision:** exposure blur, frame timing, transfer and detection latency with
jitter, centroid noise that grows toward the frame edge, lens distortion,
marker-height parallax, occlusion, dropouts, false positives.
**Encoders:** quantisation, sampling, telemetry latency and loss, and wheel
scrub, which on a skid-steer robot makes encoder odometry wrong by 4-74%
depending on turn rate, so encoders are used for velocity only.
**Commands:** input latency, radio latency and loss, watchdog, firmware speed
loop, PWM quantisation, motor deadband, gearbox backlash, battery sag.

## Measuring without fooling yourself

Every rule here was learned by getting it wrong first.

- **Check the scoring.** A goal was once counted twice when the ball bounced
  back out of the net; every result before the fix was inflated by about a
  quarter.
- **Do not let the rules rescue the AI.** An automatic reset of stuck balls
  hid the AI's worst failure: with it off, the ball was dead 83-95% of every
  match.
- **Freeze the code for every run.** `tools/evaluate.py` snapshots the source
  at launch, because runs take tens of minutes and an edit mid-run silently
  mixes two versions under one label.
- **Pictures find hypotheses; numbers accept them.** Two fixes that looked
  obviously right in filmstrips measured clearly worse.
- **Switch capabilities off to see what they are worth.** That is the only
  way the disconnected planner was found.
- **Under about two standard errors is noise.** Goals are rare, so the tools
  also report dead-ball time, shots on target and danger-zone time.

```bash
python -m tools.evaluate --label mine --matches 24
python -m tools.evaluate --label mine_off --matches 24 --set USE_INTENT_CHANNEL=false
python -m tools.evaluate --report mine,mine_off
```

## Scope

- **Not validated against hardware yet.** Every figure comes from simulation.
  Motor stall figures are estimates until the motors are bench-measured.
- **The human opponents are models.** None is fitted to a real person.
- **Not a physics engine.** It models what this robot needs and nothing more.

## Licence

MIT, see [LICENSE](LICENSE). Provided as is, with no warranty.
