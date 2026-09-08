"""
Match runner: rules, clock, and the command transport to each robot.

This owns the loop that ties everything together. Both robots are driven
through IDENTICAL command paths -- same radio latency, same jitter, same
packet loss, same watchdog. If you change one, change the other, or the
premise of the whole project quietly breaks.

The only asymmetry the runner permits is `AI_REACTION_DELAY_S`, which is an
explicit handicap knob for making the AI beatable, not an accident of
plumbing.
"""

from __future__ import annotations

import math
import random
from enum import Enum

import config
from sim import bus
from sim.world import World
from sim.geometry import length


class Phase(Enum):
    PLAYING = "playing"
    GOAL_PAUSE = "goal_pause"
    FINISHED = "finished"


class RobotLink:
    """The PC-to-robot command path for one robot.

    Carries wheel speed targets, not a pose or a twist: four motors, two
    sides, which is what the real radio packet would hold. Includes the
    watchdog, so a robot whose link drops does what the real one must --
    stops, rather than continuing at its last command into a wall.
    """

    def __init__(self, rng: random.Random, extra_latency: float = 0.0) -> None:
        self.line = bus.DelayLine(
            latency=config.RADIO_LATENCY_S + extra_latency,
            jitter=config.RADIO_JITTER_S,
            loss_prob=config.RADIO_LOSS_PROB,
            rate_hz=config.CONTROL_HZ,
            rng=rng,
        )
        self.last_applied = (0.0, 0.0)
        self.watchdog_tripped = False

    def send(self, t: float, wheel_cmd: tuple[float, float]) -> bool:
        return self.line.send(t, wheel_cmd)

    def apply(self, t: float, robot) -> None:
        pkt = self.line.receive(t)
        if pkt is None:
            robot.chassis.stop()
            return
        age = t - pkt.sent_t
        if age > config.RADIO_WATCHDOG_S:
            self.watchdog_tripped = True
            robot.chassis.stop()
            return
        self.watchdog_tripped = False
        self.last_applied = pkt.payload
        robot.chassis.set_wheel_targets(*pkt.payload)


class Match:
    def __init__(self, controller_a, controller_b, seed: int | None = None):
        """controller_a drives robot 0, controller_b drives robot 1.

        A controller needs `update(dt) -> (v, omega)` and
        `wheel_command() -> (left, right)`.
        """
        self.world = World(seed)
        self.rng = random.Random(
            (config.RANDOM_SEED if seed is None else seed) + 977
        )
        self.controllers = [controller_a, controller_b]
        self.links = [
            RobotLink(self.rng, extra_latency=config.AI_REACTION_DELAY_S),
            RobotLink(self.rng),
        ]

        self.phase = Phase.PLAYING
        self.clock = config.MATCH_DURATION_S
        self._pause_left = 0.0
        self._restart_favour: int | None = None

        self._stuck_timer = 0.0
        self.stuck_resets = 0

        self.control_tick = bus.Sampler(config.CONTROL_HZ)

    # -- rules -------------------------------------------------------------

    def _update_stuck(self, dt: float) -> None:
        """Reset a ball that has stopped somewhere unreachable.

        Without this, a ball wedged against a wall behind both robots ends
        the match as a spectacle. Real arenas need a human to intervene;
        here the clock does it.
        """
        b = self.world.ball
        if length(b.vel) < config.STUCK_BALL_SPEED_MPS:
            self._stuck_timer += dt
        else:
            self._stuck_timer = 0.0

        if self._stuck_timer >= config.STUCK_BALL_TIMEOUT_S:
            self.world.reset_ball_to_centre()
            self._stuck_timer = 0.0
            self.stuck_resets += 1

    def _on_goal(self) -> None:
        scorer = self.world.last_goal_by
        self.phase = Phase.GOAL_PAUSE
        self._pause_left = config.POST_GOAL_PAUSE_S
        # The robot that conceded restarts nearer the ball.
        self._restart_favour = 1 - scorer if scorer is not None else None
        for c in self.controllers:
            if hasattr(c, "clear"):
                c.clear()

    # -- main step ---------------------------------------------------------

    def step(self, dt: float) -> None:
        t = self.world.t

        if self.phase is Phase.FINISHED:
            return

        if self.phase is Phase.GOAL_PAUSE:
            self._pause_left -= dt
            for r in self.world.robots:
                r.chassis.stop()
            self.world.step(dt)
            if self._pause_left <= 0.0:
                self.world.kickoff(self._restart_favour)
                for link in self.links:
                    link.line.reset()
                self.phase = Phase.PLAYING
                self._stuck_timer = 0.0
            return

        # --- sensor devices, at the PHYSICS rate --------------------------
        # The camera (90 Hz) and telemetry (100 Hz) sample on their own
        # schedules, none of which align with the control rate, so they must
        # be offered every tick and decide for themselves when to fire.
        for c in self.controllers:
            hook = getattr(c, "sensor_step", None)
            if hook is not None:
                hook(dt)

        # --- controllers, at the control rate -----------------------------
        if self.control_tick.tick(dt):
            for c in self.controllers:
                c.update(1.0 / config.CONTROL_HZ)
            for link, c in zip(self.links, self.controllers):
                link.send(t, c.wheel_command())

        # --- deliver whatever has arrived ---------------------------------
        for link, robot in zip(self.links, self.world.robots):
            link.apply(t, robot)

        # --- physics ------------------------------------------------------
        self.world.step(dt)

        if self.world.goal_event:
            self._on_goal()
            return

        self._update_stuck(dt)

        self.clock -= dt
        if self.clock <= 0.0:
            self.clock = 0.0
            self.phase = Phase.FINISHED

    # -- queries -----------------------------------------------------------

    @property
    def score(self) -> list[int]:
        return self.world.score

    def summary(self) -> str:
        a, b = self.world.score
        return (f"AI {a} - {b} Player   "
                f"[{self.clock:5.1f}s left, {self.stuck_resets} ball resets]")
