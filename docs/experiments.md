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

## Open questions

- Chamfer size, settled by full matches, not the scenario set.
- The human proxies are the weakest link. The real opponent drives a FlySky RC
  transmitter; `human_rc` models continuous analog steering from a 0.2 s-old
  view. None of the proxies is fitted to a person.
- With a FlySky, the intent channel only exists if the stick values reach the
  PC. It has to be measured both on and off.
