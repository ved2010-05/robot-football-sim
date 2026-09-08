"""
Turning pixels back into metres, and measuring our own loop delay.

TWO-PLANE HOMOGRAPHY
--------------------
A single homography maps the image to ONE plane. Calibrating it on the floor
(the obvious thing to do, using floor markings) makes it correct for the ball
and wrong for the robot markers, which sit 120 mm higher. The error is not
subtle: it is radial, grows with distance from the image centre, and reaches
about 50 mm at a metre off-centre for the default geometry. That is larger
than the robot's own capture pocket.

The fix is to keep two mappings and use the right one per object type, which
requires only knowing each object's height -- which we do, because we built
the robots. `USE_TWO_PLANE_HOMOGRAPHY = False` reverts to the naive single
ground-plane mapping so the cost can be seen directly.

IMPERFECT CALIBRATION
---------------------
Undistortion uses coefficients that are deliberately slightly wrong
(`CALIB_RESIDUAL`), because a real calibration never recovers the true lens
model exactly. A simulator whose correction perfectly inverts its own
corruption teaches you nothing about the residual errors you will actually
face.

LATENCY IDENTIFICATION
----------------------
`LatencyIdentifier` recovers the sense-to-act delay by correlating what we
COMMANDED against what vision later SHOWS us doing. This is system
identification of our own hardware. It measures the robot, not the player,
and involves no model of anyone's behaviour -- it would work identically with
nobody in the room.
"""

from __future__ import annotations

import math
from collections import deque

import config
from sim.sensors.camera import undistort, parallax_factor


class Calibration:
    """Pixel-to-world mapping, as recovered by a calibration procedure."""

    def __init__(self) -> None:
        # What the calibration *thinks* the lens does. Off by CALIB_RESIDUAL.
        r = config.CALIB_RESIDUAL
        self.k1 = config.LENS_K1 * (1.0 - r)
        self.k2 = config.LENS_K2 * (1.0 - r)
        self.px_per_m = (config.CAM_RESOLUTION_PX[1]
                         / (config.ARENA_WIDTH_M * config.CAM_COVERAGE_MARGIN))

    def pixel_to_world(self, px: tuple[float, float],
                       height_m: float) -> tuple[float, float]:
        """Undistort, then unproject through the plane at the given height."""
        if config.USE_UNDISTORT:
            px = undistort(px, self.k1, self.k2)

        # Without the two-plane correction, everything is unprojected through
        # the floor, and anything above it lands too far from centre.
        h = height_m if config.USE_TWO_PLANE_HOMOGRAPHY else 0.0
        k = parallax_factor(h)

        w, hgt = config.CAM_RESOLUTION_PX
        x = (px[0] - w / 2.0) / (k * self.px_per_m)
        y = -(px[1] - hgt / 2.0) / (k * self.px_per_m)
        return x, y

    def ball_to_world(self, px):
        return self.pixel_to_world(px, config.BALL_RADIUS_M)

    def marker_to_world(self, px):
        return self.pixel_to_world(px, config.MARKER_HEIGHT_M)

    def pose_from_markers(self, centre_px, dot_px):
        """Recover (x, y, theta) from the two colour blobs.

        Heading comes from the vector between the two dots, so its accuracy
        is set by their separation:

            sigma_theta  ~=  sigma_position / MARKER_DOT_SEPARATION_M

        With 0.6 px of centroid noise at 500 px/m, that is about 1.2 mm of
        position noise; over a 55 mm separation it becomes roughly 0.02 rad,
        or 1.2 degrees. Heading is always the weakest part of a colour-blob
        pose, and widening the separation is by far the cheapest fix
        available on the real robot.
        """
        pos = self.marker_to_world(centre_px) if centre_px else None
        if pos is None or dot_px is None:
            return pos, None
        dot = self.marker_to_world(dot_px)
        theta = math.atan2(dot[1] - pos[1], dot[0] - pos[0])
        return pos, theta

    def heading_sigma(self) -> float:
        """Expected 1-sigma heading error, for the filter's R matrix."""
        pos_sigma_m = config.DETECT_NOISE_PX / self.px_per_m
        # Two independent blobs contribute to the difference vector.
        return (pos_sigma_m * math.sqrt(2.0)
                / max(config.MARKER_DOT_SEPARATION_M, 1e-6))

    def position_sigma(self) -> float:
        return config.DETECT_NOISE_PX / self.px_per_m


class LatencyIdentifier:
    """Recovers total loop delay by correlating command against observation.

    Both signals are stored with timestamps. For each candidate lag we
    resample the command history at (observation time - lag) and score how
    well it matches. The lag that matches best is the loop delay.

    Only updates when the command history actually contains variation --
    correlating two constants tells you nothing, and a robot driving steadily
    in a straight line would otherwise walk the estimate somewhere silly.
    """

    def __init__(self, window_s: float = 3.0) -> None:
        self.commands: deque[tuple[float, float]] = deque()
        self.observed: deque[tuple[float, float]] = deque()
        self.window_s = window_s
        self.estimate = config.FIXED_LATENCY_S
        self._candidates = [i * 0.004 for i in range(0, 26)]  # 0..100 ms
        self.confidence = 0.0
        self._last_run_t = -1e9

    def push_command(self, t: float, v: float) -> None:
        self.commands.append((t, v))
        self._trim(self.commands, t)

    def push_observation(self, t: float, v: float) -> None:
        self.observed.append((t, v))
        self._trim(self.observed, t)

    def _trim(self, dq: deque, t: float) -> None:
        while dq and t - dq[0][0] > self.window_s:
            dq.popleft()

    def _sample_command(self, t: float) -> float | None:
        """Linear interpolation of the command history at time t."""
        if len(self.commands) < 2:
            return None
        if t <= self.commands[0][0] or t >= self.commands[-1][0]:
            return None
        lo, hi = 0, len(self.commands) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.commands[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, v0 = self.commands[lo]
        t1, v1 = self.commands[hi]
        if t1 - t0 < 1e-9:
            return v0
        a = (t - t0) / (t1 - t0)
        return v0 + (v1 - v0) * a

    def update(self, now: float | None = None) -> float:
        if not config.AUTO_LATENCY_ID:
            self.estimate = config.FIXED_LATENCY_S
            return self.estimate
        if len(self.observed) < 40 or len(self.commands) < 40:
            return self.estimate

        # Rate-limit the search. This is a hardware property that drifts over
        # minutes; re-deriving it 100 times a second is pure waste, and it
        # dominated the entire program's runtime before this guard existed.
        if now is not None:
            if now - self._last_run_t < 1.0 / config.LATENCY_ID_HZ:
                return self.estimate
            self._last_run_t = now

        obs_vals = [v for _, v in self.observed]
        mean_o = sum(obs_vals) / len(obs_vals)
        var_o = sum((v - mean_o) ** 2 for v in obs_vals) / len(obs_vals)
        if var_o < 0.02:
            # Not enough excitation to identify anything. Hold.
            return self.estimate

        best_lag, best_score = self.estimate, -1e18
        for lag in self._candidates:
            num = 0.0
            n = 0
            cmd_vals = []
            for t, ov in self.observed:
                cv = self._sample_command(t - lag)
                if cv is None:
                    continue
                cmd_vals.append((cv, ov))
                n += 1
            if n < 20:
                continue
            mc = sum(c for c, _ in cmd_vals) / n
            mo = sum(o for _, o in cmd_vals) / n
            num = sum((c - mc) * (o - mo) for c, o in cmd_vals)
            dc = math.sqrt(sum((c - mc) ** 2 for c, _ in cmd_vals))
            do = math.sqrt(sum((o - mo) ** 2 for _, o in cmd_vals))
            if dc < 1e-9 or do < 1e-9:
                continue
            score = num / (dc * do)
            if score > best_score:
                best_score = score
                best_lag = lag

        if best_score > 0.3:
            # Smooth, so a single noisy window cannot yank the estimate.
            self.estimate += 0.2 * (best_lag - self.estimate)
            self.confidence = best_score
        return self.estimate
