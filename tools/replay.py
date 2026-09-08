"""
Record and replay matches.

The single most useful development tool here, and the cheapest to build.

Changing the planner and then playing a few games to see if it feels better is
not evidence -- match-to-match variance is larger than most changes you will
make, and you are also unconsciously playing differently each time. Recording
a real match once and then replaying YOUR EXACT INPUTS against a modified AI
gives an objective before-and-after on the same situations.

    python -m tools.replay --record mygame.json      play, then it saves
    python -m tools.replay --play mygame.json        watch it back
    python -m tools.replay --bench mygame.json       re-run headless, print score

`--bench` is the regression test: change a scoring weight, re-run against the
same recorded inputs, see whether the AI does better or worse against a human
who played identically.

A recording stores the seed and the human's command stream, not the world
state. Because the physics is deterministic at a fixed seed and timestep,
replaying those inputs reproduces the match exactly -- as long as the AI is
unchanged. When the AI IS changed the match diverges, which is the entire
point: you are watching a different AI face the same player.
"""

from __future__ import annotations

import argparse
import json
import time

import config


class InputRecorder:
    """Wraps a controller and logs the twist it produces each control tick."""

    def __init__(self, inner) -> None:
        self.inner = inner
        self.log: list[list[float]] = []

    def update(self, dt: float):
        v, w = self.inner.update(dt)
        self.log.append([round(v, 5), round(w, 5)])
        return v, w

    def wheel_command(self):
        return self.inner.wheel_command()

    def intent(self):
        return self.inner.intent()

    def clear(self):
        self.inner.clear()

    def key_down(self, k):
        self.inner.key_down(k)

    def key_up(self, k):
        self.inner.key_up(k)


class PlaybackController:
    """Replays a recorded command stream in place of a human."""

    def __init__(self, log: list[list[float]]) -> None:
        self.log = log
        self.i = 0
        self._twist = (0.0, 0.0)

    def update(self, dt: float):
        if self.i < len(self.log):
            self._twist = tuple(self.log[self.i])
            self.i += 1
        else:
            self._twist = (0.0, 0.0)
        return self._twist

    def wheel_command(self):
        from game.input import twist_to_wheels_clamped
        return twist_to_wheels_clamped(*self._twist)

    def intent(self):
        if not config.USE_INTENT_CHANNEL:
            return None
        return self._twist

    def clear(self):
        self._twist = (0.0, 0.0)

    def key_down(self, k):
        pass

    def key_up(self, k):
        pass

    def exhausted(self) -> bool:
        return self.i >= len(self.log)


def save(path: str, log, seed: int, meta: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump({
            "version": 1,
            "seed": seed,
            "control_hz": config.CONTROL_HZ,
            "physics_hz": config.PHYSICS_HZ,
            "meta": meta,
            "commands": log,
        }, f)
    print(f"saved {len(log)} commands to {path}")


def load(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    if d.get("control_hz") != config.CONTROL_HZ:
        print(f"  warning: recorded at {d.get('control_hz')} Hz, "
              f"config is {config.CONTROL_HZ} Hz -- timing will not match")
    return d


# ---------------------------------------------------------------------------

def do_record(path: str) -> int:
    from game.input import HumanController
    from game.match import Match
    from game.render import Renderer
    from ai.agent import Agent

    seed = config.RANDOM_SEED
    human = InputRecorder(HumanController())
    m = Match(None, human, seed=seed)
    agent = Agent(*m.hal(0, human), robot_index=0, truth=m.world)
    m.controllers[0] = agent

    Renderer(m, human, agent, title="Recording -- Esc to stop").run()
    save(path, human.log, seed, {
        "score": m.world.score,
        "planner": config.PLANNER,
        "motor": config.MOTOR,
    })
    print(m.summary())
    return 0


def _build_playback(path: str):
    from game.match import Match
    from ai.agent import Agent

    d = load(path)
    bot = PlaybackController(d["commands"])
    m = Match(None, bot, seed=d["seed"])
    agent = Agent(*m.hal(0, bot), robot_index=0, truth=m.world)
    m.controllers[0] = agent
    return d, m, agent, bot


def do_play(path: str) -> int:
    from game.render import Renderer
    d, m, agent, bot = _build_playback(path)
    print(f"replaying {len(d['commands'])} commands "
          f"(originally {d['meta'].get('score')})")
    Renderer(m, bot, agent, title=f"Replay -- {path}").run()
    print(m.summary())
    return 0


def do_bench(path: str) -> int:
    d, m, agent, bot = _build_playback(path)
    original = d["meta"].get("score")
    dt = config.PHYSICS_DT
    t0 = time.perf_counter()
    while m.phase.value != "finished" and not bot.exhausted():
        m.step(dt)
    wall = time.perf_counter() - t0

    now = m.world.score
    print(f"  recording      {path}")
    print(f"  originally     AI {original[0]} - {original[1]} player"
          if original else "")
    print(f"  this build     AI {now[0]} - {now[1]} player")
    if original:
        delta = (now[0] - now[1]) - (original[0] - original[1])
        print(f"  goal diff      {delta:+d} versus the recorded run")
    print(f"  belief error   " + ", ".join(
        f"{k} {v*1000:.1f} mm" for k, v in agent.error_summary().items()))
    print(f"  wall time      {wall:.1f} s")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--record", metavar="FILE")
    g.add_argument("--play", metavar="FILE")
    g.add_argument("--bench", metavar="FILE")
    ap.add_argument("--planner", choices=("mpc", "fsm"), default=None)
    args = ap.parse_args()

    if args.planner:
        config.PLANNER = args.planner

    if args.record:
        return do_record(args.record)
    if args.play:
        return do_play(args.play)
    return do_bench(args.bench)


if __name__ == "__main__":
    raise SystemExit(main())
