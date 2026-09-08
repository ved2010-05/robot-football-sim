"""
Entry point.

    python -m game.main                  play against the AI
    python -m game.main --opponent chase  play against the truth-chasing baseline
    python -m game.main --help            options

Controls:  W/S drive, A/D turn, space pause, R kickoff, F1-F5 overlays, Esc quit.
"""

from __future__ import annotations

import argparse
import sys

import config
from game.input import HumanController, ScriptedController
from game.match import Match
from game.render import Renderer


class TruthChaser(ScriptedController):
    """Baseline opponent that reads ground truth directly.

    THIS CHEATS, deliberately and visibly. It exists as a lower bound and as
    a way to exercise the game loop without the estimation stack, and it is
    never the thing being evaluated. The real agent (ai/agent.py) sees only
    what arrives through the sensor HAL.
    """

    def __init__(self, world, index: int, aggression: float = 1.0):
        super().__init__(aggression)
        self.world = world
        self.index = index

    def update(self, dt: float):
        me = self.world.robots[self.index]
        ball = self.world.ball
        goal = self.world.goal_centre(self.index)

        bx, by = ball.pos
        gx, gy = goal

        # Unit vector from the ball toward the goal.
        dx, dy = gx - bx, gy - by
        n = max((dx * dx + dy * dy) ** 0.5, 1e-6)
        ux, uy = dx / n, dy / n

        # Where the robot must stand to push the ball goalward.
        stand = me.half_len + config.HORN_LENGTH_M * 0.5 + ball.radius
        strike = (bx - ux * stand, by - uy * stand)

        # Which side of the ball are we on?
        rx, ry = me.pos[0] - bx, me.pos[1] - by
        rn = max((rx * rx + ry * ry) ** 0.5, 1e-6)
        behindness = (rx * ux + ry * uy) / rn      # -1 = perfectly behind

        if behindness < -0.80 and rn < stand * 2.2:
            # Lined up behind the ball: drive THROUGH it at the goal. Aiming
            # at the strike point instead would park the robot just short of
            # the ball and stall there forever, which is exactly what the
            # first version of this did.
            target = goal
        elif behindness < -0.2:
            target = strike
        else:
            # On the wrong side. Swing wide around the ball rather than
            # shoving it further from the goal on the way past. A holonomic
            # robot could strafe around; this one has to arc.
            side = 1.0 if (ux * ry - uy * rx) > 0 else -1.0
            px, py = -uy * side, ux * side
            target = (bx + px * stand * 1.9 - ux * stand * 0.5,
                      by + py * stand * 1.9 - uy * stand * 0.5)

        self.drive_toward(me.pos, me.theta, target)
        return self._twist


def build(opponent: str):
    human = HumanController()

    if opponent == "ai":
        try:
            from ai.agent import Agent
        except ImportError as exc:
            print(f"AI agent not available ({exc}); using the chase baseline.",
                  file=sys.stderr)
            opponent = "chase"

    if opponent == "ai":
        from ai.agent import Agent
        match = Match(None, human)
        agent = Agent(match, robot_index=0, opponent_controller=human)
        match.controllers[0] = agent
        return match, human, agent

    match = Match(None, human)
    bot = TruthChaser(match.world, index=0)
    match.controllers[0] = bot
    return match, human, None


def main() -> int:
    ap = argparse.ArgumentParser(description="2D robot football simulator")
    ap.add_argument("--opponent", choices=("ai", "chase"), default="ai",
                    help="who you play against")
    ap.add_argument("--motor", default=None, help="override config.MOTOR")
    ap.add_argument("--planner", choices=("mpc", "fsm"), default=None)
    ap.add_argument("--no-intent", action="store_true",
                    help="disable the AI's opponent intent channel")
    ap.add_argument("--speed-cap", type=float, default=None,
                    help="handicap the AI, e.g. 0.6")
    args = ap.parse_args()

    if args.motor:
        config.MOTOR = args.motor
    if args.planner:
        config.PLANNER = args.planner
    if args.no_intent:
        config.USE_INTENT_CHANNEL = False
    if args.speed_cap is not None:
        config.SPEED_CAP_FRAC = args.speed_cap

    match, human, agent = build(args.opponent)
    Renderer(match, human, agent).run()
    print(match.summary())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
