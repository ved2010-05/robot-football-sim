# Experiment log

One entry per question. Each says what was asked, how it was measured, what
came out, and what was decided, including the ideas that failed. Numbers are
goal difference per match (ours minus theirs) with one standard error unless
stated. "Within noise" means the difference is under two standard errors.

Raw per-match data for every labelled run is in `scratch/evals/<label>.json`,
and the exact code each run used is frozen in
`scratch/evals/trees/<label>-<timestamp>/` (runs 1-8 predate the timestamp and
used `trees/<label>/`).

---

## 1. Goals were being counted twice

**Question.** Two goals 0.6 s apart with nobody touching the ball appeared in
a goal-by-goal log.

**Finding.** During the 1.5 s pause after a goal the ball is still simulated.
It bounced off the back of the net, crossed the goal line on the way out, and
the direction-blind crossing test counted that as a second goal for the same
side. Re-running the control rows after the fix: control vs human conceded 26
before, 19 after.

**Decision.** A goal is only an outward crossing, and one goal disarms scoring
until the ball is put back in play. Every table before this fix is inflated by
roughly a quarter.

## 2. The stalled-ball reset was hiding the AI's worst failure

**Question.** What does the AI do when nobody rescues it?

**Method.** Turn off the rule that teleports a ball stationary for 12 s back
to the centre. Add a dead-ball probe: time with the ball unmoved for 3 s or
more.

**Finding.** Ball dead 83-95% of every match. The old rule had also been
creating goals: 3 of 5 conceded in one traced sample came within 4 s of a
reset.

**Decision.** Reset stays off permanently. Dead-ball time becomes the primary
low-noise metric, because goals are too rare to steer by.

## 3. Where dead-ball time goes

**Method.** Classify every dead second by where the ball is (open, side wall,
end wall, goal mouth, corner) and who is within 0.32 m of it. 12 matches.

**Finding.** Square corners, current AI at the time: 27 of 39 dead percentage
points were corner balls, mostly with the opponent nowhere near. The AI's own
wall sweep was failing, not the opponent blocking.

## 4. Corner chamfers

**Method.** 45 degree blocks across each corner, leg length swept. Scenario
test: 40 random wall-ball placements from a fixed seed, opponent parked, "solved"
means the ball ends 0.30 m clear of every wall or scores within 10 s.

**Finding.** With the scenario set alone, chamfer size barely mattered (17-20
of 40 solved at every size, square included), and 0.20 m was worse than square
on the hand-picked set. In full matches, on the same AI, 0.35 m chamfers cut
dead-ball time and raised goals for both sides.

**Decision.** 0.35 m default. Size to be settled by a full-match sweep.

## 5. Wall skirt (rejected)

**Idea.** A shallow ramp along every wall so a ball cannot rest against it.

**Finding.** A ramp only acts on the strip between the ball's resting centre
(its radius from the wall) and the ramp edge. At 30 mm wide that is 5 mm, and
it rolled the ball about 2 cm. To work it must be about 60 mm wide, beyond the
robot's 25 mm wheel inset, so robots would ride up it at every wall. No effect
measured in the scenario test.

**Decision.** Rejected. Removed from the code.

## 6. The wall sweep, piece by piece

Each fix was found by tracing the sweep's internal state every 0.2 s in one
scenario, then checked on the random-scenario set.

- The heading-hold sweep ran from wherever the robot was, including the far
  side of the ball. Now it starts only from the lane upstream of the ball.
- From that lane, 0.27 m off the wall, the front face never reached a ball
  lying 0.1 m from it. Now the robot steers in toward the wall.
- The target it steered for was geometrically impossible (body inside the
  wall), so it never straightened. Now the target is "body edge just clear".
- The lane test counted closing in toward the wall as leaving the lane.
- The progress watchdog counted driving to the lane as a stalled sweep, and
  banned the correct direction before the robot arrived.
- At a 20 degree tilt the leading corner hit the wall with the centre still
  0.17 m out, and the robot crept along on its corner at 0.15 m/s. The tilt is
  now limited by the clearance left.
- Along their end wall the sweep carried the ball across the open goal and
  past it. It now turns into the net once the ball is in front of the mouth.
- A run-up before the ball was tried at 0.12 and 0.25 m: 17 and 10 of 40
  solved against 18 with none. Rejected.

## 7. Ball velocity estimate

**Finding.** A stationary ball was believed to be moving at up to 0.8 m/s.
The striker and the defender both lead the ball, so both chased a ghost.

**Method.** Velocity error against truth at the filter's own timestamp, 3
matches, split by ball still / moving / 150 ms after a hit.

| velocity noise | still (median/p90) | moving | after hit |
|---|---|---|---|
| 2.5 | 0.29 / 0.59 | 0.26 / 0.51 | 0.28 / 0.74 |
| 1.0 | 0.16 / 0.30 | 0.14 / 0.28 | 0.21 / 0.90 |
| 0.5 | 0.08 / 0.18 | 0.08 / 0.18 | 0.18 / 0.92 |

Inflating the noise only near robots made the still case worse, since a still
ball usually has a robot beside it.

**Decision.** 0.5.

## 8. Current AI against the baseline AI, same arena

**Method.** `tools/evaluate.py`, 0.35 m chamfers, 24 matches of 120 s per
opponent, identical seeds, baseline = every new behaviour switched off.

| opponent | change | verdict |
|---|---|---|
| simple | +0.17 +- 0.22 | within noise |
| runner | +3.12 +- 0.61 | better |
| human | +0.54 +- 0.28 | within noise |
| human_fast | +0.42 +- 0.28 | within noise |
| two human proxies combined | +0.96 +- 0.40 | better |

Dead-ball time 82-90% to 62-67%.

## 9. Who claims the ball: distance or time-to-strike

**Question.** Against the faster human proxy most goals conceded began with
the AI claiming a midfield ball it was "as close to" while standing on the
wrong side of it, then spending 2-3 s turning and routing round while the
opponent drove straight through it.

**Change.** Claim by the time each robot needs to reach its own striking
position (path round the ball plus on-the-spot turns, identical robots, current
state only), and stop steering round the opponent's body when it is within
0.45 m of the ball.

**Decision rule.** The primary metric is the combined goal difference over the
three human proxies, because the real opponent is a person; the bots only check
for regressions. Stated honestly: this rule was set AFTER seeing the first 24
matches per opponent (which pointed the same way but sat on the edge of the
noise) and BEFORE the 24 extra matches per human proxy were run. From here on
it is fixed in `tools/evaluate.py` and applies to every comparison.

| opponent | change vs previous version | verdict |
|---|---|---|
| human (48) | +0.02 +- 0.22 | within noise |
| human_fast (48) | +0.56 +- 0.28 | better |
| human_rc (48) | +0.44 +- 0.22 | within noise |
| **three human proxies** | **+1.02 +- 0.42** | **better** |
| runner (24) | -1.50 +- 0.66 | worse |

Against the baseline AI on the same arena, human proxies combined: +2.21 +- 0.64.

**Decision.** Adopted. The runner regression is real and not yet explained.

## 10. Chamfer size, in full matches

**Method.** Current AI, three human proxies, 24 matches each, same seeds.

| chamfer | goals/match (both sides) | dead % | human proxies vs 0.35 m |
|---|---|---|---|
| 0.25 m | 1.53 | 57.9 | -1.08 +- 0.60 (within noise) |
| 0.35 m | 1.56 | 61.0 | reference |
| 0.45 m | 1.85 | 57.0 | -1.12 +- 0.54 (worse) |

**Finding.** Bigger chamfers open the game up for both sides but do not help
our AI in particular. A geometric reason for an interior optimum: to sweep a
ball along the end wall into the goal, the robot's start point sits about
0.22 m upstream and 0.27 m off the wall. With 0.35 m chamfers that point is
inside the chamfer unless the ball is within about 0.42 m of the centre line,
so a band of end wall next to each chamfer becomes a trap. Bigger chamfers
widen that band; smaller ones leave more of the corner.

**Caveat.** The AI was developed on the 0.35 m arena and may be tuned to it.

**Decision.** 0.35 m for the real arena.

## 11. Two plausible fixes that measurement rejected

**Head-on deadlock.** Filmstrips of a 0-7 match showed the AI backing out of
every head-on shove at the kickoff while the opponent drove through and
scored. It looked obvious that disengaging hands over the ball. Two
alternatives were measured against it on the three human proxies:

| deadlock response | vs backing off |
|---|---|
| keep pushing and twist the ball out | -1.98 +- 0.78 (worse) |
| never disengage | -3.00 +- 0.66 (worse) |

The frames were a real sequence but not a representative one. Kept.

**Not avoiding an opponent already in contact.** Human proxies +0.38 +- 0.55,
runner -1.25 +- 0.46. Rejected.

**Lesson for the method.** Looking at frames is how hypotheses are found, not
how they are accepted. Both of these would have shipped on the strength of
the pictures.

## 12. The real build in the sim

**Change.** The sim was moved from a 7 kg robot with a motor on every wheel to
the robot actually being built: 3 kg, one Robokits GB37 330 RPM motor per
side belted to both wheels on that side, Arducam OV9281 at 100 fps, 3.05 m up.
Motor stall figures are estimates until bench-measured (see `docs/build.md`).

**Measured in sim.** 1.67 m/s top speed, half speed in 0.26 s, 4.4 rad/s spin,
about 3 A per motor while spinning.

**Effect on the AI**, same code, three human proxies, 24 matches each:
-1.75 +- 0.61 against the old robot. Still ahead of every proxy (+0.08,
+0.17, +0.25), but only just. Everything tuned so far was tuned on a robot
that will not exist.

## 13. The AI's delay estimate is three times too high

**Finding.** The online latency identifier reported 67-93 ms against a true
26 ms, throughout every match. It correlates commanded speed with observed
speed, and observed speed lags the command by the delay PLUS the motors'
spin-up time, which the correlation absorbs.

**Tried.** Passing the command through a first-order model of the drivetrain
before correlating: 4-15 ms, now too low. The real response is limited by
motor current, so it ramps rather than decays exponentially, and a wrong
model gives a wrong answer in the other direction. Left off.

**Open.** Whether a fixed lead does better in matches (runs at 20, 55 and 90
ms were started and paused unfinished).

## 14. The intent channel is worth nothing, because nothing reads it

**Question.** How much is it worth to read the opponent's live stick command?

**Method.** Same code, seeds and opponents, intent on against intent off.

| opponent | change with intent OFF |
|---|---|
| human (24) | +0.12 +- 0.36 |
| human_fast (24) | +0.17 +- 0.25 |
| human_rc (24) | +0.04 +- 0.27 |
| **three human proxies** | **+0.33 +- 0.51, no detectable effect** |
| simple (16) | -0.38 +- 0.30 |

**Why.** Intent feeds two places. One is the opponent's position filter, where
it slightly sharpens an estimate no decision needs that sharply. The other is
the rollout planner, which predicts the opponent and picks the part of the
goal they cannot cover. That planner was called 0 times in a 20 s match: the
direct striker and shadow defender now handle every case above it, so the
planner branch is unreachable and the aim is always the centre of the goal.
The reachability analysis and the opponent model are unreachable with it.

**Consequence.** A capability can be present, tested, and switched on, and
still contribute nothing, because the code path that used it was bypassed by
later work. The only way this surfaced was switching it off and measuring.
Intent has to be wired into the decisions that now run (claiming the ball,
aim, defensive position) before "what is it worth" means anything.

## Open questions
- The human proxies are the weakest link. The real opponent drives a FlySky RC
  transmitter; `human_rc` models continuous analog steering from a 0.2 s-old
  view. None of the proxies is fitted to a person.
- Intent, aim and the opponent model are disconnected from the decisions that
  run. Reconnect them, then re-measure intent on and off.
- The AI has not been retuned for the real build.
- The latency lead: identified figure wrong, best fixed value not yet measured.
- Runner regression from the time-to-strike claim rule, unexplained.
