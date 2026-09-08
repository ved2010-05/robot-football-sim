"""
Overhead camera: ground truth in, corrupted pixel detections out.

The AI's entire picture of the world starts here, and everything this module
does to the truth is something the estimator downstream has to undo.

THE CHAIN, in the order the physical system applies it:

  1. exposure        the shutter is open for a while, so the blob centroid is
                     the AVERAGE position over that window, not the position
                     at any instant. For constant velocity that is exactly
                     pos - vel * exposure/2: a pure extra latency.
  2. parallax        markers sit 120 mm above the floor, the ball 21 mm. A
                     camera 2.5 m up sees them displaced radially outward by
                     a factor H/(H-h). For a marker 1 m off-centre that is a
                     50 mm error -- far larger than any noise term here, and
                     the single biggest reason a naive single-homography
                     setup never quite works.
  3. projection      metres to pixels
  4. lens distortion barrel distortion pushes points inward toward centre
  5. quantisation    detections land on integer pixels
  6. noise           centroid jitter, worse toward the frame edge where the
                     optics are poorer and the view more oblique
  7. dropout         occlusion and detection failure
  8. false positives the occasional blob that is not the ball
  9. transport       readout, transfer and detection time, all with jitter

Occlusion note: the ball is hidden when it passes UNDER a robot body, but it
stays visible in the capture pocket, because the horns are open-topped. That
is a genuine consequence of the mechanical design -- a covered pocket would
blind the AI at exactly the moment it had possession.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Optional

import config
from sim import bus
from sim.geometry import Vec, to_local, to_world


@dataclass
class MarkerDetection:
    """The two colour blobs on one robot, in pixels."""
    centre_px: Optional[tuple[float, float]]
    dot_px: Optional[tuple[float, float]]


@dataclass
class VisionFrame:
    """One camera frame, as the detector would hand it to the strategy code."""
    t_exposure: float                    # midpoint of the exposure window
    ball_px: Optional[tuple[float, float]]
    markers: list[MarkerDetection]
    false_ball_px: Optional[tuple[float, float]] = None


def _px_per_m() -> float:
    return (config.CAM_RESOLUTION_PX[1]
            / (config.ARENA_WIDTH_M * config.CAM_COVERAGE_MARGIN))


def parallax_factor(height_m: float) -> float:
    """Radial magnification for an object at a given height.

    A point at height h appears where the ray from the lens through it meets
    the floor, which is further from centre by H/(H-h).
    """
    H = config.CAM_HEIGHT_M
    return H / max(H - height_m, 1e-6)


def project(world_xy: Vec, height_m: float) -> tuple[float, float]:
    """World metres (at some height) to ideal pixel coordinates."""
    k = parallax_factor(height_m)
    s = _px_per_m()
    w, h = config.CAM_RESOLUTION_PX
    u = w / 2.0 + world_xy[0] * k * s
    v = h / 2.0 - world_xy[1] * k * s
    return u, v


def distort(px: tuple[float, float]) -> tuple[float, float]:
    """Apply radial lens distortion about the principal point."""
    w, h = config.CAM_RESOLUTION_PX
    cx, cy = w / 2.0, h / 2.0
    f = w / 2.0                        # normalising scale
    x = (px[0] - cx) / f
    y = (px[1] - cy) / f
    r2 = x * x + y * y
    g = 1.0 + config.LENS_K1 * r2 + config.LENS_K2 * r2 * r2
    return (cx + x * g * f, cy + y * g * f)


def undistort(px: tuple[float, float], k1: float, k2: float,
              iterations: int = 8) -> tuple[float, float]:
    """Invert the radial model by fixed-point iteration.

    There is no closed form for the inverse of a polynomial radial model, so
    real calibration pipelines iterate exactly like this.
    """
    w, h = config.CAM_RESOLUTION_PX
    cx, cy = w / 2.0, h / 2.0
    f = w / 2.0
    xd = (px[0] - cx) / f
    yd = (px[1] - cy) / f
    x, y = xd, yd
    for _ in range(iterations):
        r2 = x * x + y * y
        g = 1.0 + k1 * r2 + k2 * r2 * r2
        if abs(g) < 1e-9:
            break
        x = xd / g
        y = yd / g
    return (cx + x * f, cy + y * f)


class Camera:
    """Samples the world and emits delayed, noisy detections."""

    def __init__(self, rng: random.Random | None = None) -> None:
        self.rng = rng or random.Random(config.RANDOM_SEED + 4242)
        self.sampler = bus.Sampler(config.CAMERA_FPS)
        self.line = bus.DelayLine(
            latency=config.TRANSFER_LATENCY_S + config.DETECT_COMPUTE_S,
            jitter=config.TRANSFER_JITTER_S + config.DETECT_JITTER_S,
            loss_prob=0.0,          # frames are not lost; detections fail
            rate_hz=None,
            rng=self.rng,
        )
        self.frames_emitted = 0
        self.ball_misses = 0

    # -- noise helpers -----------------------------------------------------

    def _edge_gain(self, px: tuple[float, float]) -> float:
        """Noise multiplier, rising toward the corners of the frame."""
        w, h = config.CAM_RESOLUTION_PX
        dx = (px[0] - w / 2.0) / (w / 2.0)
        dy = (px[1] - h / 2.0) / (h / 2.0)
        r = min(1.0, math.hypot(dx, dy) / math.sqrt(2.0))
        return 1.0 + (config.DETECT_NOISE_EDGE_GAIN - 1.0) * r

    def _observe(self, world_xy: Vec, height_m: float,
                 motion_blur_m: float = 0.0) -> tuple[float, float]:
        """Full forward model for one blob."""
        px = project(world_xy, height_m)
        px = distort(px)

        sigma = config.DETECT_NOISE_PX * self._edge_gain(px)
        # A smeared blob has a less certain centroid along the smear.
        if motion_blur_m > 0.0:
            sigma += motion_blur_m * _px_per_m() * 0.25

        u = px[0] + self.rng.gauss(0.0, sigma)
        v = px[1] + self.rng.gauss(0.0, sigma)

        # Centroids are SUB-PIXEL, not integer. A colour blob spanning tens
        # of pixels has an intensity-weighted centroid good to a fraction of
        # a pixel, which is why DETECT_NOISE_PX is set below 1.0 at all.
        # Rounding to whole pixels here would double-count the very
        # uncertainty that constant already represents.
        return (u, v)

    def _in_frame(self, px: tuple[float, float]) -> bool:
        w, h = config.CAM_RESOLUTION_PX
        return 0 <= px[0] < w and 0 <= px[1] < h

    # -- occlusion ---------------------------------------------------------

    @staticmethod
    def _ball_occluded(world, ball_pos: Vec) -> bool:
        """True when a robot body hides the ball from directly above.

        Only the body occludes. The horns are thin bars with nothing over the
        top, so a carried ball stays in view.
        """
        for r in world.robots:
            lx, ly = to_local(ball_pos, r.pos, r.theta)
            if abs(lx) <= r.half_len and abs(ly) <= r.half_wid:
                return True
        return False

    # -- main --------------------------------------------------------------

    def update(self, world, dt: float) -> None:
        if not self.sampler.tick(dt):
            return

        t_now = world.t
        # The exposure midpoint is half an exposure in the past. Reporting
        # the frame at t_now while its content is from t_now - exposure/2 is
        # a real and frequently forgotten half-millisecond-scale error.
        half_exp = config.EXPOSURE_S / 2.0
        t_exp = t_now - half_exp

        # --- ball ---------------------------------------------------------
        b = world.ball
        ball_true = (b.pos[0] - b.vel[0] * half_exp,
                     b.pos[1] - b.vel[1] * half_exp)
        blur = math.hypot(b.vel[0], b.vel[1]) * config.EXPOSURE_S

        ball_px = None
        if (not self._ball_occluded(world, b.pos)
                and self.rng.random() >= config.BALL_DROPOUT_PROB):
            cand = self._observe(ball_true, config.BALL_RADIUS_M, blur)
            if self._in_frame(cand):
                ball_px = cand
        if ball_px is None:
            self.ball_misses += 1

        # --- robot markers ------------------------------------------------
        markers: list[MarkerDetection] = []
        for r in world.robots:
            centre_true = (r.pos[0] - r.vel[0] * half_exp,
                           r.pos[1] - r.vel[1] * half_exp)
            theta_true = r.theta - r.omega * half_exp
            dot_world = to_world((config.MARKER_DOT_SEPARATION_M, 0.0),
                                 centre_true, theta_true)
            rblur = math.hypot(r.vel[0], r.vel[1]) * config.EXPOSURE_S

            centre_px = dot_px = None
            if self.rng.random() >= config.MARKER_DROPOUT_PROB:
                c = self._observe(centre_true, config.MARKER_HEIGHT_M, rblur)
                if self._in_frame(c):
                    centre_px = c
            if self.rng.random() >= config.MARKER_DROPOUT_PROB:
                d = self._observe(dot_world, config.MARKER_HEIGHT_M, rblur)
                if self._in_frame(d):
                    dot_px = d
            markers.append(MarkerDetection(centre_px, dot_px))

        # --- spurious blob ------------------------------------------------
        false_px = None
        if self.rng.random() < config.FALSE_POSITIVE_PROB:
            fx = self.rng.uniform(-config.HALF_LENGTH_M, config.HALF_LENGTH_M)
            fy = self.rng.uniform(-config.HALF_WIDTH_M, config.HALF_WIDTH_M)
            false_px = self._observe((fx, fy), config.BALL_RADIUS_M)

        frame = VisionFrame(t_exposure=t_exp, ball_px=ball_px,
                            markers=markers, false_ball_px=false_px)
        self.line.send(t_now, frame, force=True)
        self.frames_emitted += 1

    def latest(self, t: float) -> Optional[VisionFrame]:
        pkt = self.line.receive(t)
        return pkt.payload if pkt else None

    def reset(self) -> None:
        self.line.reset()
        self.sampler.reset()
        self.frames_emitted = 0
        self.ball_misses = 0
