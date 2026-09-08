"""
Encoder telemetry from both robots back to the PC.

The AI receives all EIGHT encoders -- its own four and the opponent's four --
on the same link, with the same latency, jitter and loss. That is the premise
of the project: the arena is entirely home-built, so both robots report to the
same computer.

What encoders actually give you, and what they do not:

  they DO give   wheel angular position and velocity, at high rate, with
                 almost no latency compared to vision, and no dropout when
                 something drives in front of a camera

  they DO NOT    give ground truth motion. A wheel spinning is not a robot
                 moving. On a skid-steer the wheels scrub sideways through
                 every turn, so integrating encoders alone drifts fast and
                 in a state-dependent way (measured elsewhere in this
                 project at anywhere from 4% to 74% yaw error).

So encoders are a velocity source to be fused, never a position source to be
integrated. The estimator treats them accordingly, and gates them on detected
slip.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Optional

import config
from sim import bus


@dataclass
class RobotTelemetry:
    counts: list[int] = field(default_factory=list)        # 4 encoder counts
    wheel_rads: list[float] = field(default_factory=list)  # 4 wheel velocities
    battery_v: float = 0.0


@dataclass
class TelemetryPacket:
    t_sample: float
    robots: list[RobotTelemetry]


class TelemetryLink:
    def __init__(self, rng: random.Random | None = None) -> None:
        self.rng = rng or random.Random(config.RANDOM_SEED + 8181)
        self.sampler = bus.Sampler(config.TELEMETRY_HZ)
        self.line = bus.DelayLine(
            latency=config.TELEMETRY_LATENCY_S,
            jitter=config.TELEMETRY_JITTER_S,
            loss_prob=config.TELEMETRY_LOSS_PROB,
            rate_hz=None,
            rng=self.rng,
        )

    def update(self, world, dt: float) -> None:
        if not self.sampler.tick(dt):
            return
        pkt = TelemetryPacket(
            t_sample=world.t,
            robots=[
                RobotTelemetry(
                    counts=list(r.chassis.encoder_counts()),
                    wheel_rads=list(r.chassis.encoder_velocities()),
                    battery_v=r.chassis.battery_v,
                )
                for r in world.robots
            ],
        )
        self.line.send(world.t, pkt, force=True)

    def latest(self, t: float) -> Optional[TelemetryPacket]:
        p = self.line.receive(t)
        return p.payload if p else None

    def reset(self) -> None:
        self.line.reset()
        self.sampler.reset()
