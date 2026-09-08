"""
Command limits and twist-to-wheels conversion. Pure functions of geometry.

WHY THIS IS ITS OWN MODULE
--------------------------
These three lived in `game/input.py` — the keyboard handler — because the human
player needed them first. Everything else then imported them from there, which
meant `ai/controller.py` depended on the keyboard module, and so the AI could
not be lifted onto real hardware without dragging the match harness and a
tkinter input path along with it.

Nothing here touches the world, the match, a robot instance or any state. It is
the same arithmetic the firmware would do, which is exactly why it belongs
below both the AI and the game rather than inside either.

`game/input.py` re-exports these names, so existing callers still work.
"""

from __future__ import annotations

import config
from sim import chassis


def max_forward_speed() -> float:
    return chassis.max_forward_speed() * config.COMMAND_MAX_SPEED_FRAC


def max_yaw_rate() -> float:
    """Physical yaw ceiling, scaled by the shared command limit."""
    physical = (2.0 * chassis.max_wheel_speed() * config.WHEEL_RADIUS_M
                / config.effective_track_m())
    return physical * config.COMMAND_MAX_YAW_FRAC


def twist_to_wheels_clamped(v: float, omega: float) -> tuple[float, float]:
    """(v, omega) -> wheel speeds, scaled down together if either saturates.

    Scaling BOTH preserves the ratio, so a robot asked for more than it has
    drives a wider arc rather than losing its turn entirely. Clipping them
    independently would change the commanded path shape, which is worse: the
    robot would silently stop turning at high speed.
    """
    wl, wr = chassis.twist_to_wheels(v, omega)
    wmax = chassis.max_wheel_speed()
    peak = max(abs(wl), abs(wr))
    if peak > wmax and peak > 1e-9:
        s = wmax / peak
        wl *= s
        wr *= s
    return wl, wr
