"""
HAL implementation backed by the simulator.

This is the ONLY module that bridges ground truth and the AI. Everything the
AI learns about the world passes through here, already delayed and corrupted
by the sensor layer.

It also owns the sensor devices themselves, and must be stepped once per
physics tick so the camera and telemetry link sample at their own rates.
"""

from __future__ import annotations

import random
from typing import Optional

import config
from sim.io_interface import Sensors, Actuators
from sim.sensors.camera import Camera, VisionFrame
from sim.sensors.telemetry import TelemetryLink, TelemetryPacket


class SimSensors(Sensors):
    def __init__(self, world, opponent_controller=None,
                 seed: int | None = None) -> None:
        self._world = world          # NOT exposed; used only to drive sensors
        self._opponent = opponent_controller
        rng = random.Random(
            (config.RANDOM_SEED if seed is None else seed) + 31337
        )
        self.camera = Camera(random.Random(rng.random() * 1e9))
        self.telemetry_link = TelemetryLink(random.Random(rng.random() * 1e9))

    def step(self, dt: float) -> None:
        """Advance the sensor devices. Called once per physics tick."""
        self.camera.update(self._world, dt)
        self.telemetry_link.update(self._world, dt)

    # -- Sensors interface -------------------------------------------------

    def now(self) -> float:
        return self._world.t

    def vision(self) -> Optional[VisionFrame]:
        return self.camera.latest(self._world.t)

    def telemetry(self) -> Optional[TelemetryPacket]:
        return self.telemetry_link.latest(self._world.t)

    def opponent_intent(self) -> Optional[tuple[float, float]]:
        if not config.USE_INTENT_CHANNEL or self._opponent is None:
            return None
        getter = getattr(self._opponent, "intent", None)
        return getter() if getter else None

    def reset(self) -> None:
        self.camera.reset()
        self.telemetry_link.reset()


class SimActuators(Actuators):
    """Hands wheel targets to the match's radio link for our robot.

    Note that this does NOT touch the robot directly -- it goes through the
    same delayed, lossy RobotLink the human's commands travel on.
    """

    def __init__(self, match, robot_index: int) -> None:
        self._match = match
        self._index = robot_index
        self.last_command = (0.0, 0.0)

    def set_wheels(self, left_rads: float, right_rads: float) -> None:
        self.last_command = (left_rads, right_rads)

    def wheel_command(self) -> tuple[float, float]:
        """Read back by Match when it is time to transmit."""
        return self.last_command
