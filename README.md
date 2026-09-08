# 2D Robot Football Simulator

One arena, two mechanically identical skid-steer robots, one ball. Robot 0 is
autonomous. Robot 1 is you, on the keyboard.

The point is not the game. The point is that the AI stack developed here —
estimation, latency compensation, planning — should transfer to real hardware
unchanged, so the simulator models every delay and degradation the real system
will have, and then corrects what can be corrected.

Requires **Python 3.10+** and **numpy**. The renderer uses tkinter from the
standard library, so there is nothing else to install.

## Run it

```bash
python -m game.main
```

`W`/`S` drive, `A`/`D` turn, `space` pause, `R` kickoff, `Esc` quit.
`F1`–`F5` toggle overlays: AI belief, prediction, reachable set, MPC rollouts,
estimator error readout.

Other entry points:

```bash
python -m game.main --opponent chase     # play the naive baseline instead
python -m game.main --planner fsm        # state machine instead of rollout MPC
python -m game.main --no-intent          # deny the AI the intent channel
python -m game.main --speed-cap 0.6      # handicap the AI
python -m motors                         # print the motor catalogue
python -m tests.test_geometry            # 45 geometry tests
python -m tests.test_estimator           # correction-layer validation
python -m tools.headless --matches 20    # batch matches, no window
python -m tools.headless --ablate        # what each capability is worth
```

Measurement tooling — this is how every claim below was produced:

```bash
# The honesty check: our side AND the 40-line control, against every
# opponent, identical seeds, one table.
python -m tools.verify --matches 9 --seconds 120 --jobs 2

# N matches in ONE fresh process, with config overrides applied before any
# project module is imported. Prints a single JSON line.
python -m tools.arena --ai agent --opponent human --matches 9 --seconds 120     --set STRIKE_CLAIM_MARGIN_M=0.40

# Sweep a constant, one fresh process per value. Note the "--values=" form:
# written as a separate argument, a negative value is parsed as an option.
python -m tools.sweep --param STRIKE_CLAIM_MARGIN_M "--values=0.10,0.40,0.75"     --opponent human --matches 9 --seconds 120 --jobs 2

# Where the match actually goes: fraction of control ticks per behaviour mode.
python -m tools.census --opponent simple --matches 6 --seconds 120
```

`--ai` is the important flag. It swaps *our* controller for one of the
benchmark bots, so every number has a control measured through the identical
probe. `--ai simple` is the yardstick the AI has to beat; beating the opponent
while losing to that row means the sophistication is decorative.

## The file you edit

`config.py` holds every tunable, in SI units, grouped and commented. Arena and
goal dimensions, ball size, robot body and horn geometry, drivetrain, motor
choice, every delay and noise source, AI capability flags, scoring weights,
match rules.

Three that matter more than they look:

| Constant | Why |
|---|---|
| `HORN_GAP_M` | Must **exceed** `BALL_DIAMETER_M` or the ball never enters the pocket. Retention depends on the clearance being *small*: 55 mm against a 43 mm ball carries through a turn; 70 mm loses it almost immediately. |
| `MARKER_DOT_SEPARATION_M` | Heading noise is `≈ position_noise / separation`. At the default this is ~1.8°, and heading is by far the weakest part of a colour-blob pose. Doubling the separation halves the error — the cheapest real-world fix available. |
| `MOTOR` | Applied to all 8 motors, both robots. The catalogue derives electrical constants from published datasheet figures. |

## Architecture

```
        GROUND TRUTH (1 kHz)                     AI (100 Hz)
  ┌──────────────────────────┐          ┌───────────────────────────┐
  │ world     ball, robots   │          │ calibration  undistort,   │
  │ chassis   4-wheel slip   │  ──HAL─▶ │              2-plane homog│
  │ drivetrain motor, encoder│  Sensors │ estimator    fixed-lag KF │
  └──────────────────────────┘          │ reachability non-holonomic│
              ▲                         │ opponent     analytic     │
              └────── Actuators ────────│ strategy     MPC | FSM    │
                                        │ controller   → wheels     │
                                        └───────────────────────────┘
```

`sim/sim_backend.py` is the only module bridging the two sides. Nothing under
`ai/` imports `sim.world`. If that ever changes, the estimator stops being
tested and none of this transfers to hardware.

## What is modelled, and what corrects it

**Vision** — exposure blur, frame quantisation, transfer and detection latency
with jitter, sub-pixel centroid noise that worsens toward the frame edge,
radial lens distortion, marker-height parallax, occlusion, dropout, false
positives.
*Corrected by:* undistortion with deliberately imperfect coefficients, a
two-plane homography, a Kalman filter with outlier gating.

**Encoders** — tick quantisation, sampling rate, telemetry latency, jitter and
packet loss, and wheel scrub, which on a skid-steer makes encoder odometry
diverge from truth by anywhere from 4% to 74% depending on turn rate.
*Corrected by:* using encoders for velocity only, never integrating them for
position, and inflating their covariance with turn rate.

**Commands** — keyboard poll rate, input latency, a ramped virtual analog stick
(so the keyboard player is not handicapped against a continuous AI command),
radio latency, jitter, loss, watchdog, firmware PID rate, PWM quantisation,
motor deadband, gearbox backlash, reflected rotor inertia, battery sag.
*Corrected by:* latency identification and forward prediction.

## Notes on the AI

**No learning.** No player profile, nothing persisted, nothing fitted, no
in-match adaptation. The opponent predictor reads their live command off the
intent channel and propagates it through dynamics known exactly, because their
robot is identical to ours. Beyond the commitment window it falls back to one
fixed prior (`OPPONENT_ASSUME_CHASES_BALL`) that is the same for every player.

`ai/calibration.py` does identify the loop latency online, but that is system
identification of *your own hardware* — it would work with nobody in the room.
Set `AUTO_LATENCY_ID = False` to pin it to a constant.

**The intent channel** (`USE_INTENT_CHANNEL`) lets the AI read the player's
command as it is issued, ~20 ms before their robot visibly responds. It is the
strongest single capability and also the one most likely to feel unfair; it is
a flag for that reason.

## Current status — read this

The estimation stack is validated and behaves as designed. The strategy layer
has been rebuilt around one measured fact and is now genuinely competitive.

Measured over 9 matches of 120 s per cell, identical seeds, swapping only which
controller plays our side. **`control` is `tools/simplebot.py`** — forty lines
that turn on the spot until aimed, then drive straight, and nothing else:

| our side | opponent | goals | diff | ± | W-D-L |
|---|---|---|---|---|---|
| AI | simple | 9–13 | −0.44 | 0.59 | 1-6-2 |
| AI | runner | 23–6 | **+1.89** | 0.82 | 4-5-0 |
| AI | human | 19–12 | **+0.78** | 0.78 | 5-1-3 |
| control | simple | 10–5 | +0.56 | 0.32 | 4-4-1 |
| control | runner | 17–2 | +1.67 | 0.42 | 7-2-0 |
| control | human | 11–26 | −1.67 | 0.54 | 0-3-6 |

The row that matters is `human` — a fixture with a human-shaped reaction delay
and a tendency to hold a command once committed. It is the only proxy for an
actual player. The AI wins it; the trivial policy loses it heavily.

**Be careful with that number.** The human cell has measured +1.12, −0.73 and
+0.78 across three independent seed sets. The honest claim is "roughly
break-even to modestly winning, and far better than the control", not the
single best figure.

**The known regression:** against `simple` the AI is now *worse* than the
control (−0.44 against +0.56). `STRIKE_CLAIM_MARGIN_M = 0.40` causes it —
best overall (+2.57 summed across opponents against +0.58 at 0.10) but the
worst of the three values against a straight-line charger. The optimum is
opponent-dependent, and selecting per opponent would require identifying who is
playing, which is the profiling this project forbids. One value serves everyone.

### What actually moved the needle

Four findings, each measured, in the order they were found:

1. **The attacking control law was the problem, not the intelligence.** The
   full planner took 3× fewer shots on target and held half the territory of
   a forty-line bot. `DirectStriker` is now that bot's two-state law — turn on
   the spot, then drive straight, committed — fed the *predicted* ball and the
   planner's chosen aim rather than ground truth and the goal centre. Same law,
   strictly better inputs.
2. **A committed run must not be interruptible.** The strike was being hijacked
   at the exact moment it succeeded: the ball entered the horns, `possessor()`
   flipped, and `CARRY` replaced the control law mid-run.
3. **A ball on a wall has no reachable "behind".** The strike point
   `ball − û·standoff` is *outside the arena* for a cornered ball, so the robot
   drove at a point it could never occupy and wedged: 78.8% of striking ticks
   aimed outside the pitch, 45.2% of the match spent stalled against a wall.
   `wall_extraction()` sweeps along the wall instead. Wedging fell to 18.7%.
4. **Defending was worse than not defending.** The AI outscored the control
   against every opponent while conceding 14 and 11 against its 2 and 2. The
   reason is an accident of the trivial bot's geometry: its attacking target is
   behind the ball on the ball-to-their-goal line, which is *always* goal-side
   of the ball, so it defends perfectly with no defensive code at all.
   `ShadowDefender` gives the AI that geometry deliberately — conceded 14 → 6
   against `simple` and 11 → 0 against `runner`.

### How to measure anything here without fooling yourself

Three rules, each learned by getting it wrong:

- **One fresh process per config value.** Reloading modules to change a
  constant leaves `game.match` bound to the *old* `sim.world`, so half the
  stack runs the previous value. That produced a sharp, confident, entirely
  fake optimum which inverted when re-run properly. `tools/arena.py` and
  `tools/sweep.py` exist to make this impossible to get wrong.
- **Under 0.5 goals per match is noise.** Goals here are rare and clustered:
  in one 12-match run the AI scored 6 goals, *all of them inside 2 matches*,
  giving a per-match standard deviation of 1.61 against a mean of −0.08.
  Steer on the `shots`, `on_target` and `att_third_pct` columns, which sample
  the same behaviour a hundred times more often, then confirm on goals.
- **Do not use more than ~3 parallel jobs.** The simulation is memory-bandwidth
  bound. Measured on a 12-logical-core machine: a 30 s match costs 4.4 s of
  wall clock alone and 23 s with six sims running. Past three jobs,
  parallelism runs *slower than serial*.

## Things worth knowing

- **The zero-noise test is the one that matters.** `tests/test_estimator.py`
  turns every noise and delay source off and requires the belief to reproduce
  truth to sub-millimetre. It caught two real bugs during development, both
  invisible under normal noise — including one where a ball pinched between
  the two robots was buried inside a chassis 94% of the time, blinding the
  camera exactly when the game was most contested.
- The physics runs at 1 kHz with a seeded RNG, so matches are reproducible;
  `tools/replay.py --bench` re-runs your recorded inputs against a modified AI.
- The headless runner does ~1.5× realtime with the full MPC; `--ablate` is how
  you find out whether a capability is earning its place instead of assuming.
