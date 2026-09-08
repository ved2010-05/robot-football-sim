"""
Top-level autonomous agent: sensors in, wheel commands out.

The whole stack, in the order it runs each control tick:

    sensors  ->  estimator  ->  latency compensation  ->  planner
                                                      ->  controller
                                                      ->  wheel speeds

The agent holds NO reference to ground truth for any decision it makes. The
`_world_for_debug` handle exists solely to compute the belief-versus-truth
error readout, is used nowhere else, and is the first thing to check if the
AI ever starts behaving suspiciously well.
"""

from __future__ import annotations

import math

import config
from ai import controller
from ai.estimator import Estimator
from ai.strategy_fsm import FSM
from ai.tactics import (StuckMonitor, CaptureMonitor, CarryController,
                        DeadlockBreaker, OrientToBall, StrikeSequence,
                        DirectStriker, possessor, effective_aim, ball_against_wall, ShadowDefender)
from sim.io_interface import Sensors, Actuators
from sim.geometry import dist, wrap_angle


class Agent:
    def __init__(self, sensors: Sensors, actuators: Actuators,
                 robot_index: int = 0, *, truth=None) -> None:
        """The AI, given a way to sense and a way to act. Nothing else.

        `sensors` and `actuators` are the HAL interfaces from
        `sim/io_interface.py`. This class never learns where they came from,
        which is the whole portability claim: swapping the simulator for a
        real camera, a real radio and a real robot means passing a different
        pair of objects here and changing nothing below.

        Build the simulator-backed pair with `Match.hal()`.

        `truth` is a DEBUG-ONLY handle on ground truth, used to score belief
        against reality in `error_summary()`. It defaults to None and the AI
        is fully functional without it -- which is the point. On real hardware
        there is no ground truth to pass, so if any decision path ever starts
        depending on it, that path cannot ship, and the omission will surface
        here as an AttributeError rather than as a robot that works in
        simulation and fails in the arena.
        """
        self.index = robot_index
        self.opp_index = 1 - robot_index
        self.attack_dir = 1.0 if robot_index == 0 else -1.0

        self.sensors = sensors
        self.actuators = actuators

        self.estimator = Estimator(robot_index)
        self.fsm = FSM(robot_index)
        self.stuck = StuckMonitor()
        self.capture = CaptureMonitor()
        self.carry = CarryController()
        self.deadlock = DeadlockBreaker()
        self.orient = OrientToBall()
        self.strike = StrikeSequence()
        self.direct = DirectStriker(self.attack_dir)
        self.shadow = ShadowDefender(self.attack_dir)
        self.planner = None
        if config.PLANNER == "mpc":
            try:
                from ai.strategy_mpc import RolloutPlanner
                self.planner = RolloutPlanner(robot_index)
            except ImportError:
                self.planner = None

        self._cmd = (0.0, 0.0)          # last commanded (v, omega)
        self._wheels = (0.0, 0.0)

        # Debug only. Never read by any decision path.
        self._world_for_debug = truth
        self._err_accum = {"ball": 0.0, "self": 0.0, "opp": 0.0}
        self._err_n = 0
        self._err_samples = {"ball": [], "self": [], "opp": []}
        self._last_ball_truth = None
        self._teleport_skip = 0
        self.debug: dict = {}

        # Ring buffer of past ground truth, so belief can be scored against
        # truth AT THE SAME INSTANT. The filter deliberately runs ~30 ms
        # behind; comparing it to the present would charge it 60 mm of
        # "error" at 2 m/s that is not error at all, just the lag it was
        # designed to have.
        self._truth_log: list[tuple[float, tuple, tuple, tuple]] = []

    # -- called every physics tick ----------------------------------------

    def sensor_step(self, dt: float) -> None:
        """Drive the sensor devices at the physics rate.

        Separate from update() because the camera samples at 90 Hz and
        telemetry at 100 Hz, neither of which aligns with the control rate.
        """
        self.sensors.step(dt)

        w = self._world_for_debug
        if w is None:
            return                  # no ground truth available: real hardware
        self._truth_log.append((
            w.t, w.ball.pos,
            w.robots[self.index].pos, w.robots[self.opp_index].pos,
        ))
        # Keep a little more than the filter lag.
        cutoff = w.t - 0.25
        while len(self._truth_log) > 2 and self._truth_log[0][0] < cutoff:
            self._truth_log.pop(0)

    def _truth_at(self, t: float):
        """Ground truth interpolated to time t, for scoring only."""
        log = self._truth_log
        if not log:
            return None
        if t <= log[0][0]:
            return log[0][1:]
        if t >= log[-1][0]:
            return log[-1][1:]
        lo, hi = 0, len(log) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if log[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, *a = log[lo]
        t1, *b = log[hi]
        if t1 - t0 < 1e-12:
            return tuple(a)
        f = (t - t0) / (t1 - t0)
        return tuple(
            (p0[0] + (p1[0] - p0[0]) * f, p0[1] + (p1[1] - p0[1]) * f)
            for p0, p1 in zip(a, b)
        )

    # -- called at the control rate ---------------------------------------

    def update(self, dt: float) -> tuple[float, float]:
        self.estimator.step(self.sensors, self._cmd)

        now = self.sensors.now()
        belief = self.estimator.compensated_belief(now)

        me = belief.robots[self.index]
        if not me.valid or not belief.ball_valid:
            # Nothing usable yet -- sit still rather than drive blind.
            self._cmd = (0.0, 0.0)
            self._wheels = (0.0, 0.0)
            self._update_debug(belief, None)
            return self._cmd

        # Jam escape runs ABOVE the planner and overrides it.
        #
        # The planner cannot detect this situation on its own: from its point
        # of view the target it picked is still the best available, so it
        # keeps commanding full throttle into whatever is blocking it. Only a
        # comparison of commanded against achieved speed reveals the jam.
        if self.stuck.update(dt, me.pos, me.theta, me.v, self._cmd[0]):
            v, omega = self.stuck.command()
            self._cmd = (v, omega)
            self._wheels = controller.to_wheels(v, omega)
            self.actuators.set_wheels(*self._wheels)
            self._update_debug(belief, None, "ESCAPE",
                               {"escapes": self.stuck.escapes})
            return self._cmd

        # DEADLOCK: nose-to-nose with the ball trapped. Pushing harder
        # cannot win against an identical robot, so disengage and go round.
        if self.deadlock.update(dt, me, belief.robots[self.opp_index],
                                belief.ball_pos):
            v, omega = self.deadlock.command()
            self._cmd = (v, omega)
            self._wheels = controller.to_wheels(v, omega)
            self.actuators.set_wheels(*self._wheels)
            self._update_debug(belief, None, "UNJAM",
                               {"breaks": self.deadlock.breaks})
            return self._cmd

        holding = possessor(belief) == self.index

        # Who has a claim on the ball. Computed HERE, above the reflexes,
        # because the direct striker needs it before they get a chance to
        # interrupt it.
        opp_b = belief.robots[self.opp_index]
        my_d = dist(me.pos, belief.ball_pos)
        opp_d = dist(opp_b.pos, belief.ball_pos) if opp_b.valid else 9e9
        deep = ((belief.ball_pos[0] * self.attack_dir)
                < -config.HALF_LENGTH_M * config.STRIKE_THREATENED_FRAC)
        margin = (config.STRIKE_DEEP_MARGIN_M if deep
                  else config.STRIKE_CLAIM_MARGIN_M)
        ours = my_d < opp_d + margin

        # A PINNED BALL IS ALWAYS WORTH CLAIMING.
        #
        # None of the planner's candidates can extract a ball from a wall or a
        # corner -- they position, cover and cut off, all of which are things
        # to do about a ball that is in play. Measured over the corner
        # episodes: in HALF of them the wall sweep was never once the active
        # behaviour, and those episodes were extracted 33% of the time against
        # 80% for the ones where it ran. A cornered ball that nobody extracts
        # is a ball the rules teleport back to the centre after 12 seconds,
        # which is exactly the "the only option is reset" the user reported.
        #
        # So the ordinary claim margin -- which is about contesting a loose
        # ball fairly -- does not apply to a ball that is stuck. Take it
        # unless the opponent is clearly better placed.
        if ball_against_wall(belief.ball_pos) != (0.0, 0.0):
            ours = ours or (my_d < opp_d + config.PINNED_CLAIM_MARGIN_M)

        # THE ATTACKING RUN OWNS ITS OWN TICKS.
        #
        # This is the difference between the AI and the forty-line bot that
        # outscores it. The bot drives at the ball and keeps driving. The AI
        # had five reflexes above its striker, so a committed run was taken
        # over the moment it succeeded: the ball entered the horns, possessor()
        # flipped, and CARRY replaced the control law mid-strike -- while
        # CAPTURE and ORIENT could interrupt the approach before that.
        #
        # Measured against the runner opponent: the AI held the ball in the
        # attacking third 20.8% of the match to the bot's 12.0% and scored 3
        # goals to its 15. Territory was never the problem. Being interrupted
        # at the moment of conversion was.
        #
        # So when the ball is ours and we are attacking directly, nothing
        # between here and the goal gets a vote. Jam escape and deadlock still
        # sit above, because a robot that is physically stuck is not striking.
        if (config.ATTACK_MODE == "direct"
                and config.DIRECT_OWNS_APPROACH and ours):
            aim = self._attack_aim(belief)
            v, omega, ph = self.direct.command(
                dt, me, belief.ball_pos, aim, belief.ball_vel)
            self._cmd = (v, omega)
            self._wheels = controller.to_wheels(v, omega)
            self.actuators.set_wheels(*self._wheels)
            self.orient.reset()
            self.capture.reset()
            self.carry.reset()
            self._update_debug(belief, aim, f"STRIKE:{ph}",
                               {"strikes": self.direct.strikes})
            return self._cmd

        # ORIENT: close to the ball but facing away. Turn to meet it --
        # the horns are on the front, so proximity without facing is nothing.
        #
        # But ONLY from behind the ball. Standing between the ball and the
        # opponent's goal, "face the ball" points the robot at its own goal,
        # and this reflex outranks the strike gate, so it cancels the line-up
        # that was walking round to the right side. Measured, that wrong-side
        # case was 79% of every tick ORIENT fired.
        overrun = ((me.pos[0] - belief.ball_pos[0]) * self.attack_dir
                   > config.ORIENT_BEHIND_SLACK_M)
        may_orient = not (config.ORIENT_BEHIND_ONLY and overrun)
        if may_orient and self.orient.update(dt, me, belief.ball_pos, holding):
            v, omega = self.orient.command(me, belief.ball_pos)
            self._cmd = (v, omega)
            self._wheels = controller.to_wheels(v, omega)
            self.actuators.set_wheels(*self._wheels)
            self._update_debug(belief, None, "ORIENT",
                               {"turns": self.orient.turns})
            return self._cmd

        # CAPTURE runs above the planner, for the same reason the jam escape
        # does: it is a reflex, not a decision, and deliberating in the middle
        # of it is what loses the ball.
        cap_aim = (self.attack_dir * config.HALF_LENGTH_M, 0.0)
        if self.planner is not None and self.planner._aim is not None:
            cap_aim = self.planner._aim
        if self.capture.update(dt, me, belief.ball_pos, holding,
                               aim=cap_aim,
                               opp=belief.robots[self.opp_index]):
            v, omega = self.capture.command(me, belief.ball_pos)
            self._cmd = (v, omega)
            self._wheels = controller.to_wheels(v, omega)
            self.actuators.set_wheels(*self._wheels)
            self._update_debug(belief, None, "CAPTURE",
                               {"captures": self.capture.captures})
            return self._cmd

        # HOLDING THE BALL: drive it, or give it up and go round again.
        #
        # There is no "turn with the ball" mode any more, because there
        # cannot be one. Measured: while carrying, the robot achieves
        # 0.52 rad/s of yaw, so swinging 142 degrees onto the goal takes
        # 4.8 seconds -- and mean possession lasts 0.43 seconds. It could
        # never once complete a turn while holding the ball, so every
        # attempt was 88% of possession spent pirouetting and losing it.
        #
        # A passive pocket on equal robots cannot carry the ball round a
        # corner. The ball is STRUCK, not driven: line up behind it in open
        # space and go through it. So if we are holding the ball and already
        # pointed somewhere useful, commit and drive. If we are not, hand
        # back to the planner and let it walk around the ball instead of
        # wrestling with it.
        if holding:
            goal = (self.attack_dir * config.HALF_LENGTH_M, 0.0)
            aim = goal
            if self.planner is not None and self.planner._aim is not None:
                aim = self.planner._aim
            aim, _ = effective_aim(belief.ball_pos, aim, self.attack_dir)
            err = abs(wrap_angle(math.atan2(aim[1] - me.pos[1],
                                            aim[0] - me.pos[0]) - me.theta))
            if err > math.radians(config.CARRY_ABANDON_DEG):
                # Badly aimed. Turning here is futile, so do not hold on --
                # let the planner reposition and approach again.
                self.carry.reset()
                holding = False
            else:
                v, omega = self.carry.command(me, belief.ball_pos, aim)
                self._cmd = (v, omega)
                self._wheels = controller.to_wheels(v, omega)
                self.actuators.set_wheels(*self._wheels)
                self._update_debug(belief, aim, self.carry.mode)
                return self._cmd
        self.carry.reset()

        # ATTACK: when the ball is ours to go for, run the committed
        # line-up / aim / strike sequence instead of re-deliberating.
        # The planner still chooses the aim point and still owns defending.
        # Deep in our own half we demand a clearer claim before attacking;
        # everywhere else, being nearly level is enough. The old rule was a
        # flat "ball is deep -> retreat" that ignored who was closer, and it
        # discarded possession we already held on 14.8% of the match while
        # only 1.1% was a forced retreat. Striking the ball out of your own
        # half IS the defence.
        # (the claim was computed above, before the reflexes.)

        # A run already committed is not re-litigated. Without this, a ball
        # drifting across the threshold wiped the sequence back to "lineup"
        # mid-strike, which is why lineup ran 6x as often as striking.
        committed = self.strike.phase == "strike" and self.strike.hold > 0.0
        if ours or committed:
            aim = self._attack_aim(belief)
            if config.ATTACK_MODE == "direct":
                v, omega, ph = self.direct.command(
                    dt, me, belief.ball_pos, aim, belief.ball_vel)
            else:
                v, omega, ph = self.strike.command(dt, me, belief.ball_pos, aim)
            self._cmd = (v, omega)
            self._wheels = controller.to_wheels(v, omega)
            self.actuators.set_wheels(*self._wheels)
            self._update_debug(belief, aim, f"STRIKE:{ph}",
                               {"strikes": (self.direct.strikes
                                            if config.ATTACK_MODE == "direct"
                                            else self.strike.strikes)})
            return self._cmd
        self.strike.reset()
        self.direct.reset()

        # SHADOW: the ball is not ours, so take the goal-side line and hold
        # it. This replaces the planner's defensive placements, which conceded
        # seven times as often as the trivial control's accidental geometry.
        if config.USE_SHADOW_DEFENCE:
            v, omega, ph = self.shadow.command(
                dt, me, belief.ball_pos, belief.ball_vel)
            self._cmd = (v, omega)
            self._wheels = controller.to_wheels(v, omega)
            self.actuators.set_wheels(*self._wheels)
            self._update_debug(belief, self.shadow.shadow_point(
                belief.ball_pos, belief.ball_vel), f"SHADOW:{ph}")
            return self._cmd

        if self.planner is not None:
            v, omega, dbg = self.planner.decide(belief, self.estimator)
            state = dbg.get("state", "mpc")
            aim = dbg.get("aim")
        else:
            target, allow_rev, final_h, aim = self.fsm.decide(belief)
            v, omega = controller.drive_to(
                me.pos, me.theta, target,
                allow_reverse=allow_rev, final_heading=final_h,
            )
            state = self.fsm.state
            dbg = {"target": target}

        self._cmd = (v, omega)
        self._wheels = controller.to_wheels(v, omega)
        self.actuators.set_wheels(*self._wheels)
        self._update_debug(belief, aim, state, dbg)
        return self._cmd

    def _attack_aim(self, belief):
        """Where to send the ball: the planner's chosen point, wall-corrected.

        The planner and the reachability analysis pick the part of the mouth
        the opponent cannot cover; effective_aim() then overrides that when the
        ball is against a wall or in a corner, where no shot exists yet and the
        ball has to be walked out first.
        """
        aim = (self.attack_dir * config.HALF_LENGTH_M, 0.0)
        if self.planner is not None and self.planner._aim is not None:
            aim = self.planner._aim
        aim, _ = effective_aim(belief.ball_pos, aim, self.attack_dir)
        return aim

    def wheel_command(self) -> tuple[float, float]:
        return self._wheels

    def clear(self) -> None:
        self._cmd = (0.0, 0.0)
        self._wheels = (0.0, 0.0)

    # -- diagnostics -------------------------------------------------------

    def _update_debug(self, belief, aim, state: str = "init",
                      extra: dict | None = None) -> None:
        raw = self.estimator.belief()

        # Score the UNCOMPENSATED belief against truth AT THE FILTER'S OWN
        # TIMESTAMP. Two things this deliberately avoids: crediting the
        # estimator for forward prediction (which would flatter it), and
        # charging it for the fixed lag it is designed to run with (which
        # would slander it).
        truth = self._truth_at(raw.t)

        # Skip the transient after a kickoff. The ball is TELEPORTED to the
        # centre spot, which no filter can track: the gate correctly rejects
        # the jump for a few frames while it re-acquires. Counting those
        # frames reports metres of "estimator error" for something that is
        # not an estimation failure at all -- and the better the AI scores,
        # the more kickoffs there are, so it would look worse the more it won.
        if truth is not None:
            if self._last_ball_truth is not None:
                jump = dist(truth[0], self._last_ball_truth)
                if jump > 0.4:
                    self._teleport_skip = 40
            self._last_ball_truth = truth[0]
        if self._teleport_skip > 0:
            self._teleport_skip -= 1
            truth = None

        if raw.ball_valid and truth is not None:
            t_ball, t_self, t_opp = truth
            self._err_accum["ball"] += dist(raw.ball_pos, t_ball)
            self._err_accum["self"] += dist(raw.robots[self.index].pos, t_self)
            e_ball = dist(raw.ball_pos, t_ball)
            e_self = dist(raw.robots[self.index].pos, t_self)
            e_opp = dist(raw.robots[self.opp_index].pos, t_opp)
            self._err_accum["opp"] += e_opp
            self._err_n += 1
            for k, v in (("ball", e_ball), ("self", e_self), ("opp", e_opp)):
                if len(self._err_samples[k]) < 20000:
                    self._err_samples[k].append(v)

        n = max(self._err_n, 1)
        self.debug = {
            "state": state,
            "latency_est": self.estimator.total_latency,
            "rms_error": {k: v / n for k, v in self._err_accum.items()},
            "ball_belief": raw.ball_pos if raw.ball_valid else None,
            "ball_predicted": belief.ball_pos if belief.ball_valid else None,
            "self_belief": (raw.robots[self.index].pos,
                            raw.robots[self.index].theta),
            "opp_belief": (raw.robots[self.opp_index].pos,
                           raw.robots[self.opp_index].theta),
            "aim_point": aim,
            "vision_gap": self.estimator.vision_gap,
        }
        if extra:
            self.debug.update(extra)

    def error_summary(self) -> dict:
        n = max(self._err_n, 1)
        return {k: v / n for k, v in self._err_accum.items()}

    def error_median(self) -> dict:
        """Median belief error -- the robust figure.

        The mean is dominated by rare transients (a re-acquisition after an
        occlusion or a kickoff teleport), so it says more about how often the
        ball was disturbed than about how well the filter tracks. The median
        is what you want when asking 'is the correction layer right'.
        """
        out = {}
        for k, v in self._err_samples.items():
            if not v:
                out[k] = 0.0
                continue
            sv = sorted(v)
            out[k] = sv[len(sv) // 2]
        return out

    def reset_errors(self) -> None:
        self._err_accum = {"ball": 0.0, "self": 0.0, "opp": 0.0}
        self._err_n = 0
        self._err_samples = {"ball": [], "self": [], "opp": []}
        self._last_ball_truth = None
        self._teleport_skip = 0
