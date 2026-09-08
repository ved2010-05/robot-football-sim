"""
Receding-horizon rollout planner.

Each control tick: propose a few dozen candidate behaviours, simulate the
next ~1.2 s of the match under each (us, the opponent, and the ball), score
the resulting futures, execute the first step of the best one, and throw the
rest away. Next tick, do it again with fresh information.

WHY THIS RATHER THAN THE STATE MACHINE
--------------------------------------
The FSM is greedy: it picks whatever looks best right now. On a holonomic
robot that costs little, because any mistake can be corrected by sliding
sideways. On a skid-steer robot it costs a great deal, because recovering
from a bad approach angle means a three-point turn.

The concrete symptom is that a greedy planner will never reverse. Backing
away from the ball always scores worse immediately, so it is never chosen --
yet it is frequently the fastest route to a good shot, because it trades
half a second now for an approach that does not need a pirouette later. The
rollout finds that on its own, because it scores the future rather than the
present. `reverse-and-realign` is in the candidate set for exactly this
reason, and watching it get selected is the clearest sign the planner is
earning its keep.

WHAT IT KNOWS ABOUT THE OPPONENT
--------------------------------
Only what ai/opponent.py provides: their observed live command, propagated
through identical known dynamics, plus one fixed non-learned prior. The
planner never adapts to the player.
"""

from __future__ import annotations

import math

import numpy as np

import config
from ai import reachability
from ai.controller import limits, drive_to
from ai.estimator import Belief, RobotBelief, predict_ball
from ai.opponent import OpponentModel
from ai.strategy_fsm import FSM, their_goal, own_goal, travel_time
from ai.tactics import effective_aim, clamp_target, possessor, cutoff_point
from sim.geometry import wrap_angle, clamp, dist


class Candidate:
    """A parameterised behaviour, evaluated by rolling it out."""

    __slots__ = ("name", "target", "allow_reverse", "force_reverse", "tag",
                 "through")

    def __init__(self, name, target, allow_reverse=True,
                 force_reverse=False, tag="", through=False):
        self.name = name
        self.target = target
        self.allow_reverse = allow_reverse
        self.force_reverse = force_reverse
        self.tag = tag
        self.through = through


class RolloutPlanner:
    def __init__(self, self_index: int) -> None:
        self.index = self_index
        self.opp_index = 1 - self_index
        self.attack_dir = 1.0 if self_index == 0 else -1.0
        self.opponent = OpponentModel(self.opp_index)
        self.fsm = FSM(self_index)          # supplies sensible candidates
        self.last_choice = "init"

        # Held between replans.
        self._chosen: Candidate | None = None
        self._paths: list = []
        self._best_i = -1
        self._score = 0.0
        self._aim = (self.attack_dir * config.HALF_LENGTH_M, 0.0)
        self._plan_t = -1e9
        self._commit_t = -1e9
        self._anchor_name: str | None = None
        self._anchor = (0.0, 0.0)
        self._reach = None
        self._reach_t = -1e9

    # -- candidate generation ---------------------------------------------

    def _candidates(self, b: Belief, aim) -> list[Candidate]:
        me = b.robots[self.index]
        opp_v = b.robots[self.opp_index].v if b.robots[self.opp_index].valid else 0.0
        ball = b.ball_pos
        goal = their_goal(self.attack_dir)
        mine = own_goal(self.attack_dir)

        stand = (config.ROBOT_LENGTH_M / 2.0
                 + config.HORN_LENGTH_M * 0.5
                 + config.BALL_RADIUS_M)

        out: list[Candidate] = []

        # Strike lines. The FIRST of these is `aim` -- the target chosen by
        # the reachability analysis (the shot the opponent provably cannot
        # cover) and already corrected for walls and corners.
        #
        # This function previously accepted `aim` and ignored it, enumerating
        # five fixed points across the goal mouth instead. The effect was that
        # the entire reachability layer computed a best shot every replan and
        # then threw it away: switching USE_REACHABILITY off produced
        # byte-identical matches. The wall/corner correction was discarded the
        # same way, so the MPC never learned to walk a cornered ball out.
        gx = self.attack_dir * config.HALF_LENGTH_M
        hg = config.HALF_GOAL_M * 0.8
        aim_pts = [aim]
        for i in range(4):
            y = -hg + 2 * hg * i / 3.0
            aim_pts.append((gx, y))

        for i, a in enumerate(aim_pts):
            dx, dy = a[0] - ball[0], a[1] - ball[1]
            n = max(math.hypot(dx, dy), 1e-6)
            ux, uy = dx / n, dy / n
            name = "aim" if i == 0 else f"strike{i}"
            out.append(Candidate(f"{name}", a, tag="attack"))
            out.append(Candidate(f"lineup{i}",
                                 (ball[0] - ux * stand, ball[1] - uy * stand),
                                 tag="attack", through=True))

        # Interception on the ball's predicted path.
        for t in (0.15, 0.35, 0.6, 0.9):
            p, _ = predict_ball(ball, b.ball_vel, t)
            out.append(Candidate(f"intercept{t}", p, tag="intercept", through=True))

        # Arc around the ball, both ways -- toward the CHOSEN aim, so a
        # cornered ball gets wrapped in the direction that walks it out.
        for side in (+1.0, -1.0):
            dx, dy = aim[0] - ball[0], aim[1] - ball[1]
            n = max(math.hypot(dx, dy), 1e-6)
            ux, uy = dx / n, dy / n
            px, py = -uy * side, ux * side
            out.append(Candidate(
                f"wrap{'L' if side > 0 else 'R'}",
                (ball[0] + px * stand * 1.9 - ux * stand * 0.6,
                 ball[1] + py * stand * 1.9 - uy * stand * 0.6),
                tag="wrap", through=True))

        # CUT-OFF DEFENDING.
        #
        # The candidate whose absence lost every match to a player who simply
        # drove forward in a straight line. Every other defensive option here
        # is a flavour of "go near the ball", and going near a ball that an
        # opponent is carrying means trailing it forever -- measured: the AI
        # was in `intercept` for 66 of the 68 samples immediately before
        # conceding, and in `defend` for none of them.
        #
        # This instead picks a point on the ball->our-goal line that we can
        # reach BEFORE the ball does, as far up the pitch as that is still
        # true. It is the standard RoboCup keeper construction: occupy the
        # place the ball must come to, rather than chase where it is.
        ball_speed = math.hypot(*b.ball_vel)
        holder = possessor(b)
        if holder == self.opp_index:
            # A carried ball travels at the CARRIER's speed and does not
            # decelerate; using the free-rolling ball speed here would
            # under-estimate how fast the attack is arriving.
            ball_speed = max(ball_speed, abs(opp_v))
        carrier = (b.robots[self.opp_index]
                   if holder == self.opp_index else None)
        cut = cutoff_point(me, ball, ball_speed, mine, travel_time,
                           carrier=carrier)
        out.append(Candidate("cutoff", cut, tag="defend", through=True))
        # A deeper fallback, in case we cannot make the forward cut-off.
        gdx, gdy = mine[0] - cut[0], mine[1] - cut[1]
        gn = max(math.hypot(gdx, gdy), 1e-6)
        out.append(Candidate("cutoff_deep",
                             (cut[0] + gdx / gn * 0.25,
                              cut[1] + gdy / gn * 0.25), tag="defend", through=True))

        # Defensive placements.
        out.append(Candidate("block", reachability.best_block_pose(
            ball, mine, None, self.attack_dir), tag="defend", through=True))
        dxm, dym = mine[0] - ball[0], mine[1] - ball[1]
        nm = max(math.hypot(dxm, dym), 1e-6)
        for frac in (0.3, 0.55):
            out.append(Candidate(
                f"cover{frac}",
                (ball[0] + dxm / nm * nm * frac, ball[1] + dym / nm * nm * frac),
                tag="defend", through=True))

        # REVERSE AND REALIGN. The candidate a greedy planner can never pick:
        # give up ground now to arrive on a better line later.
        back = (me.pos[0] - math.cos(me.theta) * 0.30,
                me.pos[1] - math.sin(me.theta) * 0.30)
        out.append(Candidate("reverse", back, force_reverse=True,
                             tag="reposition"))
        for side in (+1.0, -1.0):
            ang = me.theta + side * 2.4
            out.append(Candidate(
                f"peel{'L' if side > 0 else 'R'}",
                (me.pos[0] + math.cos(ang) * 0.32,
                 me.pos[1] + math.sin(ang) * 0.32),
                tag="reposition"))

        # Every target must be somewhere the robot can physically be.
        # A target on the wall line (the goal, for the strike
        # candidates) makes the controller drive into the wall and stall.
        for c in out:
            c.target = clamp_target(c.target)
        return out[:config.MPC_CANDIDATES]

    # -- rollout -----------------------------------------------------------

    def _rollout_all(self, b: Belief, cands: list[Candidate]):
        """Simulate EVERY candidate at once, vectorised over candidates.

        Written as array operations rather than a loop over candidates
        because the scalar version cost 6.7 ms per candidate and blew the
        control budget by 15x. Simulating all of them together turns 22
        independent 60-step integrations into 60 steps of 22-wide array
        maths, which is the same arithmetic with a fraction of the Python
        interpreter overhead.

        Returns (scores, paths).
        """
        K = len(cands)
        dt = config.MPC_STEP_S
        steps = int(config.MPC_HORIZON_S / dt)
        tau = max(config.DRIVETRAIN_TAU_S, 1e-3)
        a = 1.0 - math.exp(-dt / tau)
        v_max, w_max = limits()

        me = b.robots[self.index]
        opp = b.robots[self.opp_index]

        tx = np.array([c.target[0] for c in cands], dtype=float)
        ty = np.array([c.target[1] for c in cands], dtype=float)
        allow_rev = np.array([c.allow_reverse for c in cands], dtype=bool)
        force_rev = np.array([c.force_reverse for c in cands], dtype=bool)

        mx = np.full(K, me.pos[0]); my = np.full(K, me.pos[1])
        mth = np.full(K, me.theta); mv = np.full(K, me.v); mw = np.full(K, me.omega)
        ox = np.full(K, opp.pos[0]); oy = np.full(K, opp.pos[1])
        oth = np.full(K, opp.theta); ov = np.full(K, opp.v); ow = np.full(K, opp.omega)
        bx = np.full(K, b.ball_pos[0]); by = np.full(K, b.ball_pos[1])
        bvx = np.full(K, b.ball_vel[0]); bvy = np.full(K, b.ball_vel[1])

        score = np.zeros(K)
        possession = np.zeros(K)
        opp_contact = np.zeros(K)
        wall_time = np.zeros(K)
        rev_time = np.zeros(K)
        carry_turn = np.zeros(K)
        done = np.zeros(K, dtype=bool)
        want_paths = config.SHOW_MPC_ROLLOUTS
        paths = ([[(me.pos[0], me.pos[1])] for _ in range(K)]
                 if want_paths else [])

        hx, hy = config.HALF_LENGTH_M, config.HALF_WIDTH_M
        br = config.BALL_RADIUS_M
        e = config.WALL_RESTITUTION
        pocket = config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
        # A robot strikes the ball with its whole front, so the
        # interaction radius is the body half-width, not the pocket gap.
        catch = config.ROBOT_WIDTH_M / 2.0 + br
        ad = self.attack_dir

        for i in range(steps):
            t = i * dt
            live = ~done
            pmx, pmy = mx, my
            pox, poy = ox, oy

            # --- our control (vectorised drive_to) ------------------------
            dx = tx - mx
            dy = ty - my
            rng = np.hypot(dx, dy)
            bearing = np.arctan2(dy, dx)
            err = np.arctan2(np.sin(bearing - mth), np.cos(bearing - mth))
            rev = allow_rev & (np.abs(err) > math.pi / 2)
            err_rev = np.arctan2(np.sin(bearing + math.pi - mth),
                                 np.cos(bearing + math.pi - mth))
            err = np.where(rev, err_rev, err)

            cw = np.clip(3.2 * err, -w_max, w_max)
            speed = np.minimum(v_max, 3.4 * rng)
            speed = np.where(rng < 0.35,
                             speed * np.maximum(0.25, rng / 0.35), speed)
            cv = np.where(np.abs(err) > math.radians(70.0),
                          0.0, speed * np.cos(err))
            cv = np.where(rev, -cv, cv)
            cv = np.where(rng < 0.05, 0.0, cv)
            cv = np.where(force_rev, -np.abs(cv), cv)

            mv = mv + (cv - mv) * a
            mw = mw + (cw - mw) * a
            mx = mx + np.where(live, mv * np.cos(mth) * dt, 0.0)
            my = my + np.where(live, mv * np.sin(mth) * dt, 0.0)
            mth = mth + np.where(live, mw * dt, 0.0)

            # --- their control, from the analytic opponent model ----------
            odx = bx - ox
            ody = by - oy
            orng = np.hypot(odx, ody)
            obear = np.arctan2(ody, odx)
            oerr = np.arctan2(np.sin(obear - oth), np.cos(obear - oth))
            orev = np.abs(oerr) > math.pi / 2
            oerr = np.where(orev,
                            np.arctan2(np.sin(obear + math.pi - oth),
                                       np.cos(obear + math.pi - oth)), oerr)
            chase_w = np.clip(oerr * 2.5, -1.0, 1.0) * w_max
            chase_v = (np.clip(orng * 2.0, 0.0, 1.0) * v_max
                       * (1.0 - 0.5 * np.abs(oerr)))
            chase_v = np.where(orev, -chase_v, chase_v)

            held = self.opponent.last_intent
            hv = held[0] if held is not None else opp.v
            hw = held[1] if held is not None else opp.omega

            if t <= config.OPPONENT_ASSUMED_REACTION_S:
                ocv, ocw = hv, hw
            elif config.OPPONENT_ASSUME_CHASES_BALL:
                blend = min(1.0, (t - config.OPPONENT_ASSUMED_REACTION_S)
                            / max(config.OPPONENT_COMMAND_DECAY_S, 1e-6))
                ocv = hv * (1 - blend) + chase_v * blend
                ocw = hw * (1 - blend) + chase_w * blend
            else:
                fade = max(0.0, 1.0 - (t - config.OPPONENT_ASSUMED_REACTION_S)
                           / max(config.OPPONENT_COMMAND_DECAY_S, 1e-6))
                ocv, ocw = hv * fade, hw * fade

            ov = ov + (ocv - ov) * a
            ow = ow + (ocw - ow) * a
            ox = ox + np.where(live, ov * np.cos(oth) * dt, 0.0)
            oy = oy + np.where(live, ov * np.sin(oth) * dt, 0.0)
            oth = oth + np.where(live, ow * dt, 0.0)

            # --- ball -----------------------------------------------------
            sp = np.hypot(bvx, bvy)
            k = np.where(sp > 1e-6,
                         np.maximum(0.0, sp - config.BALL_ROLL_DECEL * dt)
                         / np.maximum(sp, 1e-9), 0.0)
            bvx *= k
            bvy *= k
            bx = bx + np.where(live, bvx * dt, 0.0)
            by = by + np.where(live, bvy * dt, 0.0)

            hit_top = (by + br > hy) & (bvy > 0)
            by = np.where(hit_top, hy - br, by)
            bvy = np.where(hit_top, -bvy * e, bvy)
            hit_bot = (by - br < -hy) & (bvy < 0)
            by = np.where(hit_bot, -hy + br, by)
            bvy = np.where(hit_bot, -bvy * e, bvy)

            in_mouth = np.abs(by) <= config.HALF_GOAL_M
            hit_r = (bx + br > hx) & (bvx > 0) & ~in_mouth
            bx = np.where(hit_r, hx - br, bx)
            bvx = np.where(hit_r, -bvx * e, bvx)
            hit_l = (bx - br < -hx) & (bvx < 0) & ~in_mouth
            bx = np.where(hit_l, -hx + br, bx)
            bvx = np.where(hit_l, -bvx * e, bvx)

            # --- contact: a robot front that reaches the ball pushes it ---
            #
            # SWEPT, not point-sampled. At 2 m/s the robot's nose moves 40 mm
            # per 20 ms step, so testing only the endpoint lets it teleport
            # straight past the ball. That made contact essentially never
            # register, which in turn meant no rollout ever predicted a goal
            # and the whole terminal reward was dead code -- the planner was
            # steering on shaping terms alone and simply chasing the ball.
            for (rx, ry, prx, pry, rth, rv, mine_) in (
                    (mx, my, pmx, pmy, mth, mv, True),
                    (ox, oy, pox, poy, oth, ov, False)):
                px = rx + np.cos(rth) * pocket
                py = ry + np.sin(rth) * pocket
                qx = prx + np.cos(rth) * pocket
                qy = pry + np.sin(rth) * pocket
                abx = px - qx
                aby = py - qy
                L2 = np.maximum(abx * abx + aby * aby, 1e-12)
                s = np.clip(((bx - qx) * abx + (by - qy) * aby) / L2, 0.0, 1.0)
                d = np.hypot(bx - (qx + abx * s), by - (qy + aby * s))
                touch = (d < catch) & live
                push = np.maximum(rv, 0.0) * 1.15
                bvx = np.where(touch, np.cos(rth) * push, bvx)
                bvy = np.where(touch, np.sin(rth) * push, bvy)
                if mine_:
                    possession += np.where(touch, dt, 0.0)
                    # Turning while carrying is what loses the ball, so make
                    # the rollout pay for it rather than discovering the loss
                    # only after it happens.
                    carry_turn += np.where(touch, np.abs(mw) * dt, 0.0)

            # --- costs accumulated along the way --------------------------
            #
            # Collisions and walls are things to be avoided BEFORE they
            # happen. The rollout already knows where the opponent will be --
            # it propagates their live command through dynamics identical to
            # ours -- so running into them is entirely foreseeable, and
            # dodging falls out of simply pricing it in.
            body_r = (config.ROBOT_LENGTH_M + config.ROBOT_WIDTH_M) / 4.0
            sep = np.hypot(mx - ox, my - oy)
            opp_contact += np.where(live & (sep < body_r * 2.0 + 0.04), dt, 0.0)

            near_wall = (
                (np.abs(mx) > config.HALF_LENGTH_M - config.ROBOT_LENGTH_M * 0.75)
                | (np.abs(my) > config.HALF_WIDTH_M - config.ROBOT_LENGTH_M * 0.75))
            wall_time += np.where(live & near_wall, dt, 0.0)
            rev_time += np.where(live & (mv < -0.05), dt, 0.0)

            # --- terminal -------------------------------------------------
            scored = live & (bx * ad > hx) & (np.abs(by) < config.HALF_GOAL_M)
            conceded = live & (bx * ad < -hx) & (np.abs(by) < config.HALF_GOAL_M)
            score += np.where(scored, config.W_GOAL * (1.0 - t / config.MPC_HORIZON_S), 0.0)
            score += np.where(conceded, config.W_CONCEDE * (1.0 - t / config.MPC_HORIZON_S), 0.0)
            done = done | scored | conceded

            # Recording every candidate's path is a Python loop inside the
            # integration loop -- the one thing worth avoiding here. It is
            # only ever needed to draw the overlay, so it is off unless the
            # overlay is on.
            if want_paths:
                for j in range(K):
                    if live[j]:
                        paths[j].append((float(mx[j]), float(my[j])))

        # --- terminal shaping (only for rollouts that did not end) --------
        goal = their_goal(ad)
        mine_goal = own_goal(ad)
        open_play = ~done

        d_their = np.hypot(bx - goal[0], by - goal[1])
        d_our = np.hypot(bx - mine_goal[0], by - mine_goal[1])
        d_self = np.hypot(mx - bx, my - by)
        d_opp = np.hypot(ox - bx, oy - by)

        shaped = (config.W_BALL_TO_THEIR_GOAL * d_their
                  + config.W_BALL_TO_OUR_GOAL * d_our
                  + config.W_SELF_TO_BALL * d_self
                  + config.W_POSSESSION * possession
                  + config.W_TURN_WHILE_CARRYING * carry_turn
                  + config.W_AVOID_OPPONENT * opp_contact
                  + config.W_AVOID_WALL * wall_time
                  + config.W_REVERSING * rev_time)

        bearing = np.arctan2(by - my, bx - mx)
        shaped += config.W_FACING_BALL * np.cos(
            np.arctan2(np.sin(bearing - mth), np.cos(bearing - mth)))
        shaped += np.where(d_opp < d_self,
                           config.W_OPPONENT_POSSESSION * 0.5, 0.0)

        # Approach alignment: are we behind the ball relative to their goal?
        #
        # +1 means we are directly behind it and driving forward sends the
        # ball at the goal. -1 means we are on the far side and would knock
        # it back toward our own end. This is what turns proximity into
        # useful position for a robot that cannot strafe and has no kicker.
        tbx = bx - mx
        tby = by - my
        tbn = np.maximum(np.hypot(tbx, tby), 1e-6)
        bgx = goal[0] - bx
        bgy = goal[1] - by
        bgn = np.maximum(np.hypot(bgx, bgy), 1e-6)
        align = (tbx * bgx + tby * bgy) / (tbn * bgn)

        # ATTACKING alignment only applies when we are attacking.
        #
        # This term rewards standing on the far side of the ball from their
        # goal. That is right when we are going forward and exactly backwards
        # when they are carrying the ball at OUR goal -- correct defensive
        # position scores -1 here, so the term was paying up to 68 points
        # AGAINST defending. That is why the planner kept choosing `intercept`
        # right up to the moment it conceded.
        #
        # When they have the ball, score the mirror image instead: be between
        # the ball and the goal we are protecting.
        ogx = mine_goal[0] - bx
        ogy = mine_goal[1] - by
        ogn = np.maximum(np.hypot(ogx, ogy), 1e-6)
        def_align = (tbx * ogx + tby * ogy) / (tbn * ogn)

        # Who holds the ball at the end of the rollout?
        opp_px = ox + np.cos(oth) * pocket
        opp_py = oy + np.sin(oth) * pocket
        near = np.hypot(bx - opp_px, by - opp_py) < catch * 1.25

        # Defend only against an actual THREAT, not mere proximity.
        #
        # Gating on "opponent is near the ball" alone made the robot defend
        # permanently -- they are near the ball almost all the time, since
        # they are chasing it too. Measured: it conceded far less but scored
        # 0 goals in 8 straight matches. A defender that never attacks has
        # simply lost slowly.
        #
        # A threat needs the ball to be in our half OR moving toward our
        # goal. Anywhere else, taking the ball off them IS the defence.
        our_half = (bx * ad) < 0.0
        incoming = (bvx * ad) < -0.15
        they_hold = near & (our_half | incoming)

        shaped += np.where(they_hold,
                           config.W_DEFEND_ALIGN * def_align,
                           config.W_APPROACH_ALIGN * align)

        score += np.where(open_play, shaped, 0.0)
        return score, paths

    # -- entry point -------------------------------------------------------

    def decide(self, b: Belief, estimator):
        me = b.robots[self.index]
        opp = b.robots[self.opp_index]
        now = b.t

        self.opponent.observe(getattr(estimator, "_last_opp_intent", None))

        # Reachable sets change slowly relative to the control loop, so they
        # are refreshed on their own timer rather than every tick.
        if (config.USE_REACHABILITY and opp.valid
                and now - self._reach_t >= 1.0 / config.REACH_REPLAN_HZ):
            self._reach = reachability.compute(
                opp, self.opponent.last_intent,
                reaction_s=self.opponent.effective_reaction())
            self._reach_t = now
        reach = self._reach if config.USE_REACHABILITY else None

        # Re-run the search on its own timer too. Between searches the
        # controller keeps driving toward the chosen target at the full
        # control rate, so motion stays smooth -- only the DECISION is held.
        if (self._chosen is None
                or now - self._plan_t >= 1.0 / config.MPC_REPLAN_HZ):
            aim = ((self.attack_dir * config.HALF_LENGTH_M, 0.0), False)
            if reach is not None:
                aim = reachability.best_aim(b.ball_pos, reach, self.attack_dir)
            # Redirect the aim when the ball is walled or cornered, so
            # every strike/lineup candidate below is generated against a
            # target that is actually achievable.
            self._aim, self._diverted = effective_aim(
                b.ball_pos, aim[0], self.attack_dir)

            cands = self._candidates(b, self._aim)
            scores, paths = self._rollout_all(b, cands)

            # Hysteresis: prefer to CARRY ON doing what we were doing.
            #
            # Without this the planner re-picks from scratch 25 times a
            # second, and since several candidates usually score within a
            # point or two of each other, it flips between them and the robot
            # dithers -- endlessly starting manoeuvres it never finishes. On
            # a non-holonomic robot that is fatal, because every switch costs
            # a turn. Committing until something is clearly better is worth
            # more than always picking the instantaneous maximum.
            incumbent = -1
            if self._chosen is not None:
                spread = float(np.max(scores) - np.min(scores))
                bonus = config.MPC_SWITCH_MARGIN_FRAC * max(spread, 1e-6)
                for i, c in enumerate(cands):
                    if c.name == self._chosen.name:
                        scores[i] += bonus
                        incumbent = i
                        break

                # TIME-BASED COMMITMENT.
                #
                # Score hysteresis alone is not enough, because it says
                # nothing about how long a decision lasts. The machine needs
                # ~0.54 s of steady command to reach speed; re-deciding every
                # 40 ms means it never gets there.
                #
                # So hold the current behaviour for a minimum time unless a
                # rival is dramatically better -- which is exactly the case
                # where a goal or a concede has appeared in the rollout and
                # changing our mind is genuinely urgent.
                held = now - self._commit_t
                if held < config.MPC_MIN_COMMIT_S and incumbent >= 0:
                    rival = float(np.max(scores))
                    need = config.MPC_OVERRIDE_FRAC * max(spread, 1e-6)
                    if rival - float(scores[incumbent]) < need:
                        scores[incumbent] = rival + 1.0

            best_i = int(np.argmax(scores))

            self._chosen = cands[best_i]
            self._paths = paths
            self._best_i = best_i
            self._score = float(scores[best_i])
            self._plan_t = now
            if self._chosen is None or self._chosen.name != self.last_choice:
                self._commit_t = now
            self.last_choice = self._chosen.name

        best = self._chosen

        # ANCHOR A SELF-REFERENTIAL TARGET.
        #
        # `peel` and `reverse` are defined RELATIVE TO THE ROBOT'S OWN HEADING
        # (ang = me.theta +/- 2.4). The commitment logic above re-matches the
        # incumbent by NAME, so every replan hands back a freshly recomputed
        # target -- which has rotated by exactly as much as the robot just
        # turned toward it. The robot can never arrive at it; it chases its
        # own tail and travels nowhere.
        #
        # So once one of these is chosen, freeze it in world coordinates for
        # as long as we stay committed to it. Turning to a fixed point on the
        # floor terminates; turning to a point welded to your own nose does
        # not.
        if config.ANCHOR_RELATIVE_TARGETS and best.tag == "reposition":
            if self._anchor_name != best.name:
                self._anchor_name = best.name
                self._anchor = best.target
            best.target = self._anchor
        elif self._anchor_name is not None:
            self._anchor_name = None

        v, w = drive_to(me.pos, me.theta, best.target,
                        allow_reverse=best.allow_reverse,
                        through=best.through)
        if best.force_reverse:
            v = -abs(v)

        dbg = {
            "state": f"{best.tag}:{best.name}",
            "aim": self._aim,
            "rollouts": self._paths if config.SHOW_MPC_ROLLOUTS else None,
            "best_index": self._best_i,
            "reach_rects": (reach.rects()
                            if (reach and config.SHOW_REACHABILITY) else None),
            "score": self._score,
            "commitment": self.opponent.commitment,
        }
        return v, w, dbg
