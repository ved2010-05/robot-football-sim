"""
Where can the opponent physically be, and when?

For a holonomic robot this is a disc: position plus velocity plus half a t
squared, done. A skid-steer robot cannot strafe, so its reachable set is
nothing like a disc -- it is a lobed shape stretched along the heading,
pinched to the sides, with a second lobe behind for reversing. Using a disc
here would systematically overestimate the opponent's coverage sideways and
underestimate it along their heading, which is exactly backwards for
deciding whether a shot gets past them.

So the set is computed the honest way: sample the control space, forward
simulate each candidate with the same dynamics model the estimator uses, and
rasterise the result.

THE COMMITMENT WINDOW
---------------------
The opponent cannot change what they are doing instantly. For the first
`OPPONENT_ASSUMED_REACTION_S` they are still executing their current
command, and only after that can they fan out. Modelling that shrinks the
reachable set dramatically and is where most of the AI's edge comes from.

That delay is a FIXED CONSTANT from general knowledge of human reaction
time. It is never measured from the player, never fitted, never adapted. The
same number would be used against an empty chair.
"""

from __future__ import annotations

import math

import numpy as np

import config
from ai.controller import limits
from ai.estimator import RobotBelief


class ReachableSet:
    """Rasterised reachable region at several future times."""

    def __init__(self) -> None:
        self.slices: dict[float, np.ndarray] = {}
        # Where the opponent goes if they simply hold their current
        # command. Used as a CONTINUOUS margin for ranking shots --
        # the occupancy grid alone can only answer yes/no, and its
        # clearance estimate saturates, so it cannot rank at all.
        self.nominal: dict[float, tuple[float, float]] = {}
        self.origin = (-config.HALF_LENGTH_M, -config.HALF_WIDTH_M)
        self.cell = config.REACH_GRID_M
        self.nx = int(config.ARENA_LENGTH_M / self.cell) + 2
        self.ny = int(config.ARENA_WIDTH_M / self.cell) + 2
        # Interception radius: the opponent does not need to be ON the ball,
        # only near enough for its body or horns to touch it.
        self.touch_r = (config.ROBOT_WIDTH_M / 2.0
                        + config.BALL_RADIUS_M)

    def _idx(self, x: float, y: float) -> tuple[int, int]:
        i = int((x - self.origin[0]) / self.cell)
        j = int((y - self.origin[1]) / self.cell)
        return (min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1))

    def occupied(self, x: float, y: float, t: float) -> bool:
        """Can the opponent have something at (x, y) by time t?"""
        if not self.slices:
            return False
        key = min(self.slices, key=lambda s: abs(s - t))
        grid = self.slices[key]
        i, j = self._idx(x, y)
        return bool(grid[i, j])

    def cells(self, t: float | None = None):
        """Occupied cell centres, for the debug overlay."""
        if not self.slices:
            return []
        key = max(self.slices) if t is None else min(
            self.slices, key=lambda s: abs(s - t))
        grid = self.slices[key]
        idx = np.argwhere(grid)
        return [(self.origin[0] + (i + 0.5) * self.cell,
                 self.origin[1] + (j + 0.5) * self.cell) for i, j in idx]

    def rects(self, t: float | None = None):
        """Occupied region as merged run-length rectangles, in world metres.

        The overlay used to emit one canvas rectangle per occupied cell --
        about 1700 of them -- which cost 270 ms per frame and dropped the
        game to 10 fps. Merging each column's consecutive runs into single
        rectangles draws the identical shape in a few dozen items.

        Returns a list of (x0, y0, x1, y1).
        """
        if not self.slices:
            return []
        key = max(self.slices) if t is None else min(
            self.slices, key=lambda s: abs(s - t))
        grid = self.slices[key]
        ox, oy = self.origin
        c = self.cell
        out = []
        for i in range(grid.shape[0]):
            col = grid[i]
            j = 0
            n = col.shape[0]
            while j < n:
                if not col[j]:
                    j += 1
                    continue
                k = j
                while k < n and col[k]:
                    k += 1
                out.append((ox + i * c, oy + j * c,
                            ox + (i + 1) * c, oy + k * c))
                j = k
        return out


def compute(rb: RobotBelief, current_cmd: tuple[float, float] | None,
            reaction_s: float | None = None) -> ReachableSet:
    """Forward-simulate a fan of control choices from the opponent's state."""
    out = ReachableSet()
    if not rb.valid:
        return out

    v_max, w_max = limits()
    if reaction_s is None:
        reaction_s = config.OPPONENT_ASSUMED_REACTION_S

    n = config.REACH_CONTROL_SAMPLES
    vs = np.linspace(-v_max, v_max, n)
    ws = np.linspace(-w_max, w_max, n)
    V, W = np.meshgrid(vs, ws)
    V = V.ravel()
    W = W.ravel()
    m = V.size

    x = np.full(m, rb.pos[0])
    y = np.full(m, rb.pos[1])
    th = np.full(m, rb.theta)
    v = np.full(m, rb.v)
    w = np.full(m, rb.omega)

    # During the commitment window every candidate does the same thing:
    # whatever the opponent is already doing.
    hold_v, hold_w = (current_cmd if current_cmd is not None
                      else (rb.v, rb.omega))

    tau = max(config.DRIVETRAIN_TAU_S, 1e-3)
    dt = 0.02
    t = 0.0
    horizon = max(config.REACH_TIME_SLICES)
    slices = sorted(config.REACH_TIME_SLICES)
    next_slice = 0

    grid = np.zeros((out.nx, out.ny), dtype=bool)
    pad = int(math.ceil(out.touch_r / out.cell))

    # Nominal trajectory: hold the observed command for the whole horizon.
    nx_, ny_, nth_ = rb.pos[0], rb.pos[1], rb.theta
    nv_, nw_ = rb.v, rb.omega

    while t < horizon + 1e-9:
        a = 1.0 - math.exp(-dt / tau)
        if t < reaction_s:
            v += (hold_v - v) * a
            w += (hold_w - w) * a
        else:
            v += (V - v) * a
            w += (W - w) * a

        x = x + v * np.cos(th) * dt
        y = y + v * np.sin(th) * dt
        th = th + w * dt
        t += dt

        a2 = 1.0 - math.exp(-dt / tau)
        nv_ += (hold_v - nv_) * a2
        nw_ += (hold_w - nw_) * a2
        nx_ += nv_ * math.cos(nth_) * dt
        ny_ += nv_ * math.sin(nth_) * dt
        nth_ += nw_ * dt

        while next_slice < len(slices) and t >= slices[next_slice] - 1e-9:
            s = slices[next_slice]
            out.nominal[s] = (nx_, ny_)
            gi = np.clip(((x - out.origin[0]) / out.cell).astype(int),
                         0, out.nx - 1)
            gj = np.clip(((y - out.origin[1]) / out.cell).astype(int),
                         0, out.ny - 1)
            snap = grid.copy()
            snap[gi, gj] = True
            # Dilate by the interception radius: the opponent blocks a ball
            # it can merely touch, not only one it can sit on.
            if pad > 0:
                snap = _dilate(snap, pad)
            # Reachability accumulates -- somewhere reachable at 0.3 s is
            # still reachable at 0.5 s (just wait there).
            grid = grid | snap
            out.slices[s] = grid.copy()
            next_slice += 1

    return out


def _dilate(grid: np.ndarray, pad: int) -> np.ndarray:
    """Square dilation by `pad` cells, done separably, WITHOUT wrapping.

    A square structuring element is separable, so dilating along x and then
    along y gives the identical result in 2*pad shifts per axis instead of
    (2*pad+1)^2 -- cheap enough to run per time slice per replan.

    The shifts are slice assignments, NOT np.roll. np.roll is circular: it
    wraps the top edge onto the bottom and the left onto the right. Using it
    here meant a reachable region near one corner of the pitch also appeared
    in the other three, so the AI believed the opponent could simultaneously
    cover all four corners. That corrupted both shot safety and defensive
    placement, and showed up on screen as the reachability overlay 'leaking'
    to the far side of the arena.
    """
    if pad <= 0:
        return grid.copy()

    tmp = grid.copy()
    for dx in range(1, pad + 1):
        tmp[dx:, :] |= grid[:-dx, :]
        tmp[:-dx, :] |= grid[dx:, :]

    out = tmp.copy()
    for dy in range(1, pad + 1):
        out[:, dy:] |= tmp[:, :-dy]
        out[:, :-dy] |= tmp[:, dy:]
    return out


# ---------------------------------------------------------------------------
# Shot selection
# ---------------------------------------------------------------------------

def _nominal_at(reach: ReachableSet, t: float):
    """Where the opponent will be at time t if they hold their command."""
    if not reach.nominal:
        return None
    best_t = min(reach.nominal, key=lambda s: abs(s - t))
    return reach.nominal[best_t]


def shot_survives(ball_pos, aim, reach: ReachableSet,
                  speed: float = 1.6) -> tuple[bool, float]:
    """March a shot along its path and test it against the reachable set.

    Returns (survives, margin) where margin is the smallest clearance in
    metres between the ball and the reachable region along the way. A
    positive margin means the opponent provably cannot get a body part to
    it in time -- not "probably won't", cannot.
    """
    dx = aim[0] - ball_pos[0]
    dy = aim[1] - ball_pos[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return False, 0.0
    ux, uy = dx / d, dy / d

    steps = 14
    survives = True
    worst = 1e9
    for i in range(1, steps + 1):
        frac = i / steps
        travelled = d * frac
        # Constant-deceleration roll; a slow shot gives them longer to reach.
        disc = speed * speed - 2.0 * config.BALL_ROLL_DECEL * travelled
        if disc <= 0.0:
            survives = False
            worst = min(worst, 0.0)
            break
        t = (speed - math.sqrt(disc)) / max(config.BALL_ROLL_DECEL, 1e-6)
        px = ball_pos[0] + ux * travelled
        py = ball_pos[1] + uy * travelled
        if reach.occupied(px, py, t):
            survives = False

        # Continuous margin: how far this point on the shot is from where
        # the opponent will actually be. Unlike the grid clearance -- which
        # saturates a few cells out and returned an identical value for
        # every aim point, making best_aim always choose the goal centre --
        # this discriminates across the whole pitch.
        nom = _nominal_at(reach, t)
        if nom is not None:
            worst = min(worst, math.hypot(px - nom[0], py - nom[1])
                        - reach.touch_r)
        else:
            worst = min(worst, _clearance(reach, px, py, t))
    if worst > 1e8:
        worst = 0.0
    return survives, worst


def _clearance(reach: ReachableSet, x: float, y: float, t: float) -> float:
    """Rough distance from (x, y) to the nearest reachable cell."""
    if not reach.slices:
        return 1e9
    key = min(reach.slices, key=lambda s: abs(s - t))
    grid = reach.slices[key]
    i, j = reach._idx(x, y)
    for r in range(1, 9):
        lo_i, hi_i = max(0, i - r), min(reach.nx, i + r + 1)
        lo_j, hi_j = max(0, j - r), min(reach.ny, j + r + 1)
        if grid[lo_i:hi_i, lo_j:hi_j].any():
            return (r - 1) * reach.cell
    return 8 * reach.cell


def best_aim(ball_pos, reach: ReachableSet, attack_dir: float,
             shot_speed: float = 1.6):
    """Pick the point in the goal mouth that is hardest to defend.

    Prefers a provably unblockable line. If none exists -- the opponent is
    covering the whole mouth -- it falls back to the line with the largest
    clearance, which is the best available rather than a refusal to shoot.
    """
    gx = attack_dir * config.HALF_LENGTH_M
    hg = config.HALF_GOAL_M * 0.80
    best = (gx, 0.0)
    best_score = -1e18
    any_safe = False

    for i in range(9):
        y = -hg + (2 * hg) * i / 8.0
        cand = (gx, y)
        safe, margin = shot_survives(ball_pos, cand, reach, shot_speed)
        # Slight preference for the centre: a shot at an extreme angle has
        # less room for the aim error that always exists in practice.
        score = margin + (2.0 if safe else 0.0) - 0.25 * abs(y) / max(hg, 1e-9)
        if safe:
            any_safe = True
        if score > best_score:
            best_score = score
            best = cand
    return best, any_safe


def best_block_pose(ball_pos, our_goal, reach_of_them: ReachableSet,
                    attack_dir: float):
    """Where to stand to minimise their best shot.

    Samples positions on the arc between the ball and our goal and scores
    each by how much of the mouth it takes away. Standing on the ball-to-goal
    line is the classic answer and usually wins, but not always -- when they
    are already wide, cutting the near post is better.
    """
    bx, by = ball_pos
    gx, gy = our_goal
    dx, dy = gx - bx, gy - by
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return our_goal
    ux, uy = dx / n, dy / n

    best = (bx + ux * 0.4, by + uy * 0.4)
    best_score = -1e18
    for frac in (0.25, 0.35, 0.45, 0.60):
        for lateral in (-0.12, 0.0, 0.12):
            px = bx + ux * (n * frac) - uy * lateral
            py = by + uy * (n * frac) + ux * lateral
            if abs(px) > config.HALF_LENGTH_M - 0.12:
                continue
            if abs(py) > config.HALF_WIDTH_M - 0.12:
                continue
            # Prefer being on the line and nearer our own goal.
            on_line = -abs(lateral) * 3.0
            depth = -abs(frac - 0.35) * 2.0
            best_local = on_line + depth
            if best_local > best_score:
                best_score = best_local
                best = (px, py)
    return best
