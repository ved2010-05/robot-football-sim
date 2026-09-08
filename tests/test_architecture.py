"""
The portability claim, enforced.

The README says the AI stack transfers to real hardware unchanged, and that
nothing under `ai/` reaches into the simulator's ground truth. That is only
true for as long as nobody adds a convenient import, and the failure is silent:
the AI keeps working in simulation and becomes unshippable on hardware, which
is exactly the bug you cannot find by playing matches.

So it is a test. Run it with the others:

    python -m tests.test_architecture
"""

from __future__ import annotations

import ast
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
AI = ROOT / "ai"

# Modules under sim/ that describe GROUND TRUTH or the simulator's own
# machinery. Importing any of these into ai/ means the AI is reading the
# answer rather than estimating it.
FORBIDDEN = {
    "sim.world",         # the actual positions of everything
    "sim.sim_backend",   # the HAL *implementation*, i.e. which world it wraps
    "sim.chassis",       # ...except see ALLOWED below
    "game.match",        # match rules, scores, the referee
    "game.main",
    "game.render",
}

# Deliberate, reasoned exceptions.
ALLOWED = {
    # Pure geometry and scalar maths. No state, no world, no truth -- the same
    # functions would ship in the firmware.
    "sim.geometry",
    # Command limits and twist->wheels. Pure arithmetic, no state, no world;
    # the firmware does the same sums.
    "sim.kinematics",
    # The HAL *interfaces* (abstract base classes). Depending on the contract
    # is the point; depending on the implementation is not.
    "sim.io_interface",
    # The camera model's undistort/parallax helpers are the correction maths
    # the real pipeline needs too, not a source of truth.
    "sim.sensors.camera",
    # chassis exposes the drivetrain FORWARD MODEL, which the estimator must
    # have to predict motion. It is physics, identical on hardware.
    "sim.chassis",
}

FAILURES: list[str] = []


def module_names(node: ast.AST):
    for n in ast.walk(node):
        if isinstance(n, ast.Import):
            for a in n.names:
                yield a.name, n.lineno
        elif isinstance(n, ast.ImportFrom):
            if n.module and n.level == 0:
                yield n.module, n.lineno


def check_imports() -> None:
    for path in sorted(AI.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), str(path))
        for name, line in module_names(tree):
            if name in ALLOWED:
                continue
            root = name.split(".")[0]
            if root in ("sim", "game") and name in FORBIDDEN:
                FAILURES.append(
                    f"{path.relative_to(ROOT)}:{line} imports {name} -- "
                    f"the AI must not read ground truth"
                )
            elif root == "game":
                FAILURES.append(
                    f"{path.relative_to(ROOT)}:{line} imports {name} -- "
                    f"the AI must not depend on the match harness"
                )
    print(f"  ok   ai/ imports nothing that reads ground truth"
          if not FAILURES else "  FAIL import boundary")


def check_agent_signature() -> None:
    """Agent must be constructible from interfaces alone."""
    import inspect
    from ai.agent import Agent
    from sim.io_interface import Sensors, Actuators

    params = inspect.signature(Agent.__init__).parameters
    for required in ("sensors", "actuators"):
        if required not in params:
            FAILURES.append(
                f"Agent.__init__ has no '{required}' parameter -- the HAL must "
                f"be injected, not constructed internally"
            )
    if params.get("truth") is not None and params["truth"].default is not None:
        FAILURES.append("Agent.__init__ 'truth' must default to None: "
                        "there is no ground truth on real hardware")
    print("  ok   Agent takes injected Sensors/Actuators, truth defaults None")


def check_runs_without_truth() -> None:
    """The real proof: a whole match with no ground-truth handle at all."""
    import config
    config.MATCH_DURATION_S = 8.0
    from game.match import Match
    from ai.agent import Agent
    from tools.simplebot import SimpleBot

    m = Match(None, None, seed=11)
    bot = SimpleBot(m.world, 1)
    agent = Agent(*m.hal(0, bot), robot_index=0)   # no truth= on purpose
    m.controllers[0] = agent
    m.controllers[1] = bot
    m.clock = 8.0
    dt = config.PHYSICS_DT
    while m.phase.value != "finished":
        m.step(dt)
    print("  ok   full match with truth=None (the hardware case)")


def main() -> int:
    print("architecture")
    check_imports()
    check_agent_signature()
    check_runs_without_truth()
    if FAILURES:
        print("\nFAILED:")
        for f in FAILURES:
            print("  " + f)
        return 1
    print("\nall architecture tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
