"""
Predicting the opponent, without learning anything about them.

WHAT THIS DELIBERATELY DOES NOT DO
----------------------------------
No player profile. Nothing persisted between matches. No fitting of observed
behaviour. No in-match adaptation of any parameter. This module would behave
identically on its first second against a new player and its thousandth
against a familiar one, and it keeps no state that could tell the two apart.

WHAT IT DOES INSTEAD
--------------------
Two things, both purely physical:

  1. READS their current command off the intent channel. Not inferred --
     observed. Their gamepad is wired into the same PC, so we know what they
     asked for before their robot has visibly begun to do it.

  2. PROPAGATES it through dynamics we know exactly, because their robot is
     mechanically identical to ours.

That combination is stronger than a learned behavioural model would be, and
needs no learning at all: rather than guessing what a player tends to do, it
watches what they just did and computes the consequence.

Beyond the commitment window, where the observed command no longer tells us
anything, it falls back to one FIXED assumption -- that they are going for
the ball. That is a designed prior, not a fitted one: it is a constant in
this file, identical for every player, and switchable off.
"""

from __future__ import annotations

import math

import config
from ai.controller import limits
from ai.estimator import RobotBelief
from sim.geometry import wrap_angle, clamp


class OpponentModel:
    def __init__(self, opp_index: int) -> None:
        self.index = opp_index
        self.attack_dir = 1.0 if opp_index == 0 else -1.0
        self.last_intent: tuple[float, float] | None = None
        self._history: list[tuple[float, float]] = []
        self.commitment = 0.0        # 0 = freely manoeuvring, 1 = locked in

    def observe(self, intent: tuple[float, float] | None) -> None:
        """Take the opponent's live command and measure how committed it is.

        COMMITMENT
        ----------
        People do not steer continuously; they push a direction and hold it.
        While they are holding, they cannot go somewhere else -- not because
        of psychology, but because of physics we know exactly: they must
        first notice (their reaction time), then their robot has to decelerate
        and turn, and their robot is mechanically identical to ours, so that
        cost is computable rather than guessable.

        So a steady command stream is not just information about what they are
        doing now. It is a guarantee about what they CANNOT do next, and it
        widens the window in which any move we complete is uncontested.

        This is measured from the live command stream only. Nothing is fitted,
        nothing persists between matches, and it would read 0 against an empty
        chair -- it is the same calculation for every player.
        """
        self.last_intent = intent
        if intent is None:
            self._history.clear()
            self.commitment = 0.0
            return

        self._history.append(intent)
        if len(self._history) > config.COMMITMENT_WINDOW_N:
            self._history.pop(0)
        if len(self._history) < 4:
            self.commitment = 0.0
            return

        # How steady has the command been? Low spread => committed.
        vs = [c[0] for c in self._history]
        ws = [c[1] for c in self._history]
        mv = sum(vs) / len(vs)
        mw = sum(ws) / len(ws)
        var = (sum((v - mv) ** 2 for v in vs) / len(vs)
               + sum((w - mw) ** 2 for w in ws) / len(ws))
        spread = math.sqrt(max(var, 0.0))

        # Also require they are actually doing something -- a player sitting
        # still is "steady" but is not committed to anything.
        magnitude = math.hypot(mv, mw)
        if magnitude < config.COMMITMENT_MIN_MAGNITUDE:
            self.commitment = 0.0
            return

        self.commitment = clamp(
            1.0 - spread / max(config.COMMITMENT_SPREAD_REF, 1e-6), 0.0, 1.0)

    def effective_reaction(self) -> float:
        """How long the opponent is locked into their current command.

        Base reaction time, plus the extra time a committed opponent needs to
        undo the motion they are already making. Feeding this into the
        reachable-set computation is what turns "they probably cannot get
        there" into "they provably cannot".
        """
        base = config.OPPONENT_ASSUMED_REACTION_S
        if not config.USE_COMMITMENT:
            return base
        return base + self.commitment * config.COMMITMENT_EXTRA_S

    # -- the assumed policy after commitment expires -----------------------

    def _assumed_command(self, rb: RobotBelief, ball_pos) -> tuple[float, float]:
        """A fixed, player-independent guess: they drive at the ball.

        Crude on purpose. A more elaborate assumption would be a model of a
        person, which is exactly what is excluded here. This is the same
        assumption you would make about any opponent, including one you had
        never seen.
        """
        v_max, w_max = limits()
        dx = ball_pos[0] - rb.pos[0]
        dy = ball_pos[1] - rb.pos[1]
        rng = math.hypot(dx, dy)
        if rng < 1e-6:
            return 0.0, 0.0
        bearing = math.atan2(dy, dx)
        err = wrap_angle(bearing - rb.theta)
        reverse = abs(err) > math.pi / 2
        if reverse:
            err = wrap_angle(bearing + math.pi - rb.theta)
        w = clamp(err * 2.5, -1.0, 1.0) * w_max
        v = clamp(rng * 2.0, 0.0, 1.0) * v_max * (1.0 - 0.5 * abs(err))
        return (-v if reverse else v), w

    def command_at(self, elapsed: float, rb: RobotBelief,
                   ball_pos) -> tuple[float, float]:
        """What we assume the opponent is commanding `elapsed` into the future.

        Phase 1 -- committed. They are still executing what we observed.
        Phase 2 -- decaying. Observed command fades out over
                   OPPONENT_COMMAND_DECAY_S.
        Phase 3 -- assumed. Fixed go-for-the-ball prior, or coasting if that
                   prior is switched off.
        """
        held = self.last_intent
        if held is None:
            # No intent channel: all we have is their observed motion, which
            # already lags reality. Assume it persists, then decays.
            held = (rb.v, rb.omega)

        if elapsed <= config.OPPONENT_ASSUMED_REACTION_S:
            return held

        if not getattr(config, "OPPONENT_ASSUME_CHASES_BALL", True):
            fade = max(0.0, 1.0 - (elapsed - config.OPPONENT_ASSUMED_REACTION_S)
                       / max(config.OPPONENT_COMMAND_DECAY_S, 1e-6))
            return (held[0] * fade, held[1] * fade)

        blend = clamp((elapsed - config.OPPONENT_ASSUMED_REACTION_S)
                      / max(config.OPPONENT_COMMAND_DECAY_S, 1e-6), 0.0, 1.0)
        assumed = self._assumed_command(rb, ball_pos)
        return (held[0] * (1.0 - blend) + assumed[0] * blend,
                held[1] * (1.0 - blend) + assumed[1] * blend)

    # -- convenience -------------------------------------------------------

    def committed_for(self) -> float:
        """How long we may assume the opponent cannot change their mind.

        A fixed constant, never measured from the player. This is the window
        inside which any move we complete is uncontested, and it is the
        single most valuable number in the planner.
        """
        return config.OPPONENT_ASSUMED_REACTION_S
