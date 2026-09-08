"""
The hardware abstraction boundary.

Everything in ai/ talks to these two interfaces and nothing else. That is
enforced by discipline rather than by the language, so it is worth stating
plainly: if any module under ai/ ever imports sim.world, the estimator stops
being tested and none of this transfers to hardware.

Two implementations are intended:

    SimSensors / SimActuators      backed by the simulator (sim_backend.py)
    <real hardware>                backed by an actual camera and radio,
                                   written later, unchanged AI above it

The interface is deliberately narrow and matches exactly what the user's
described build provides:

    vision      one overhead camera, colour blobs only, no fiducial tags
    telemetry   four encoders from each of the two robots
    intent      the opponent's live command, because their gamepad is wired
                into the same PC
    actuation   wheel speed targets for our own four motors

Nothing else. No ball velocity sensor, no IMU, no absolute heading source.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from sim.sensors.camera import VisionFrame
from sim.sensors.telemetry import TelemetryPacket


class Sensors(ABC):
    """Everything the AI can observe."""

    @abstractmethod
    def now(self) -> float:
        """The PC's clock. All timestamps share this reference."""

    @abstractmethod
    def vision(self) -> Optional[VisionFrame]:
        """Most recent camera frame available, or None before the first.

        The frame carries its own exposure timestamp -- it is ALREADY OLD
        when you receive it, and how old varies. Latency compensation depends
        entirely on reading that timestamp rather than assuming the frame is
        current.
        """

    @abstractmethod
    def telemetry(self) -> Optional[TelemetryPacket]:
        """Most recent encoder packet from both robots, or None."""

    @abstractmethod
    def opponent_intent(self) -> Optional[tuple[float, float]]:
        """The opponent's commanded (v, omega) as issued, undelayed.

        Returns None when config.USE_INTENT_CHANNEL is off, or when no such
        channel exists (e.g. a real opponent not wired into this PC). The AI
        must degrade gracefully to vision and encoders in that case.
        """


class Actuators(ABC):
    """Everything the AI can affect."""

    @abstractmethod
    def set_wheels(self, left_rads: float, right_rads: float) -> None:
        """Wheel speed targets in rad/s. Four motors, two independent sides.

        Sending speeds rather than a twist is deliberate: it is what the
        radio packet really carries, and it means the choice of track width
        -- and therefore the scrub error -- lives in the AI where it belongs,
        not hidden in firmware.
        """
