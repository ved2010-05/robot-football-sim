"""
Transport delay primitive.

Every signal path in this system is lossy and late: the camera to the PC,
the PC to a robot, a robot's telemetry back to the PC, the keyboard to the
game. Rather than scatter ad-hoc delay handling through those modules, they
all push through a `DelayLine`.

A DelayLine models four things at once:

    rate      you can only put packets in so often
    latency   they arrive later than they were sent
    jitter    by a varying amount, so they can arrive OUT OF ORDER
    loss      or not at all

Out-of-order arrival is the subtle one and the reason this is a priority
queue rather than a deque. With 4 ms latency and 1 ms of jitter, packets
genuinely overtake each other, and a receiver that blindly takes the newest
arrival can go backwards in time. The `receive` here returns the packet with
the newest SEND time among those that have arrived, which is what a real
receiver does when packets carry timestamps -- and it is why every packet in
this simulator carries one.
"""

from __future__ import annotations

import heapq
import itertools
import random
from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class Packet:
    sent_t: float       # when the sender created it
    arrive_t: float     # when the receiver can first see it
    payload: Any


class DelayLine:
    def __init__(self, latency: float, jitter: float = 0.0,
                 loss_prob: float = 0.0, rate_hz: float | None = None,
                 rng: random.Random | None = None) -> None:
        self.latency = latency
        self.jitter = jitter
        self.loss_prob = loss_prob
        self.period = (1.0 / rate_hz) if rate_hz else None
        self.rng = rng or random.Random(0)

        self._queue: list[tuple[float, int, Packet]] = []
        self._counter = itertools.count()
        self._last_send_t: float | None = None

        # Most recently delivered packet, held until something newer lands.
        self._current: Optional[Packet] = None

        self.sent = 0
        self.dropped = 0
        self.delivered = 0

    # -- sender side -------------------------------------------------------

    def ready_to_send(self, t: float) -> bool:
        if self.period is None or self._last_send_t is None:
            return True
        return (t - self._last_send_t) >= self.period - 1e-12

    def send(self, t: float, payload: Any, force: bool = False) -> bool:
        """Offer a packet. Returns True if it entered the link.

        Respects the configured rate unless `force`. A packet that is dropped
        by the loss model still counts as sent -- the sender does not know.
        """
        if not force and not self.ready_to_send(t):
            return False
        self._last_send_t = t
        self.sent += 1

        if self.loss_prob > 0.0 and self.rng.random() < self.loss_prob:
            self.dropped += 1
            return True

        delay = self.latency
        if self.jitter > 0.0:
            delay += self.rng.uniform(-self.jitter, self.jitter)
        delay = max(delay, 0.0)

        pkt = Packet(sent_t=t, arrive_t=t + delay, payload=payload)
        heapq.heappush(self._queue, (pkt.arrive_t, next(self._counter), pkt))
        return True

    # -- receiver side -----------------------------------------------------

    def receive(self, t: float) -> Optional[Packet]:
        """Newest packet available at time t, or the last one if nothing new.

        Returns the packet itself, not just the payload, because the receiver
        needs `sent_t` to know how stale the data is -- that timestamp is the
        entire basis of latency compensation downstream.
        """
        while self._queue and self._queue[0][0] <= t:
            _, _, pkt = heapq.heappop(self._queue)
            self.delivered += 1
            # Keep the newest by SEND time, so a jittered overtake cannot
            # make the receiver's view of the world jump backwards.
            if self._current is None or pkt.sent_t >= self._current.sent_t:
                self._current = pkt
        return self._current

    def peek_age(self, t: float) -> float:
        """How stale the currently-held packet is."""
        if self._current is None:
            return float("inf")
        return t - self._current.sent_t

    def reset(self) -> None:
        self._queue.clear()
        self._current = None
        self._last_send_t = None
        self.sent = self.dropped = self.delivered = 0

    def stats(self) -> str:
        loss = (100.0 * self.dropped / self.sent) if self.sent else 0.0
        return f"sent={self.sent} dropped={self.dropped} ({loss:.1f}%)"


class Sampler:
    """Fires at a fixed rate inside a faster loop.

    Used wherever a subsystem runs slower than the physics tick: the camera
    at 90 Hz, telemetry at 100 Hz, the AI at 100 Hz, all inside a 1 kHz sim.
    """

    def __init__(self, rate_hz: float) -> None:
        self.period = 1.0 / rate_hz
        self._accum = 0.0
        self._primed = False

    def tick(self, dt: float) -> bool:
        if not self._primed:
            self._primed = True
            return True
        self._accum += dt
        if self._accum >= self.period:
            self._accum -= self.period
            return True
        return False

    def reset(self) -> None:
        self._accum = 0.0
        self._primed = False
