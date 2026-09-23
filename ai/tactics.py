"""
Tactical helpers: walls, corners, dribbling, and getting unstuck.

Three things that a naive "drive at the point behind the ball" policy gets
badly wrong on a real pitch, and that were visibly wrong in play:

  1. WALLS. Pushing the ball "toward the goal" is only correct in open play.
     With the ball against a side wall and the goal off to one side, that
     command drives the ball INTO the wall, where it stops. The robot then
     re-derives the same command and does it again, forever.

  2. CORNERS. Worse: in a corner both escape directions are blocked, and the
     only way out is to push the ball along one wall until it is clear. That
     is never the direction of the goal, so a goal-seeking policy will never
     choose it.

  3. JAMMING. The robot itself gets wedged -- against a wall, against the
     other robot, or nose-first into a corner -- and keeps commanding full
     throttle into an obstacle. Nothing in the planner notices, because from
     the planner's point of view its chosen target is still the best one; it
     simply never arrives.

None of this is strategy in the interesting sense. It is the floor that has
to exist before strategy means anything.
"""

from __future__ import annotations

import math

import config
from sim.geometry import clamp, wrap_angle


# How close to a wall the ball must be before wall handling kicks in.
WALL_MARGIN_M = 0.16
CORNER_MARGIN_X = 0.34
CORNER_MARGIN_Y = 0.26


def ball_against_wall(ball) -> tuple[float, float]:
    """Inward normal of whichever walls the ball is against, (0,0) if clear."""
    nx = ny = 0.0
    hx, hy = config.HALF_LENGTH_M, config.HALF_WIDTH_M
    if ball[1] > hy - WALL_MARGIN_M:
        ny = -1.0
    elif ball[1] < -hy + WALL_MARGIN_M:
        ny = 1.0
    # The goal mouth is an opening, not a wall -- do not treat a ball lined
    # up with it as trapped.
    if abs(ball[1]) > config.HALF_GOAL_M:
        if ball[0] > hx - WALL_MARGIN_M:
            nx = -1.0
        elif ball[0] < -hx + WALL_MARGIN_M:
            nx = 1.0
    return nx, ny


def in_corner(ball) -> bool:
    hx, hy = config.HALF_LENGTH_M, config.HALF_WIDTH_M
    return (abs(ball[0]) > hx - CORNER_MARGIN_X
            and abs(ball[1]) > hy - CORNER_MARGIN_Y)


def effective_aim(ball, goal, attack_dir: float):
    """Where we should actually send the ball right now.

    Usually the goal. But a ball in a corner has to be walked out along a
    wall before any shot exists, and a ball against a side wall has to be
    pushed along that wall rather than into it. Returning a different aim
    point here fixes both the FSM and the MPC at once, since both derive
    their approach geometry from it.
    """
    nx, ny = ball_against_wall(ball)
    if nx == 0.0 and ny == 0.0:
        return goal, False

    if in_corner(ball):
        # Walk it out along the LONG axis, toward the opponent's half if we
        # can, but the priority is simply getting it into open play. Aiming
        # at the goal from here just wedges it harder into the corner.
        target_x = attack_dir * config.HALF_LENGTH_M * 0.35
        target_y = ball[1] * 0.25
        return (target_x, target_y), True

    # Against one wall: project the goal direction onto the wall tangent so
    # we slide the ball along it instead of grinding it in.
    gx, gy = goal[0] - ball[0], goal[1] - ball[1]
    n = math.hypot(gx, gy)
    if n < 1e-6:
        return goal, False
    ux, uy = gx / n, gy / n

    into = ux * (-nx) + uy * (-ny)      # component pointing into the wall
    if into > 0.25:
        # Remove the into-wall part, keep the along-wall part, and add a
        # nudge back toward the middle of the pitch.
        tx = ux + nx * into
        ty = uy + ny * into
        tn = math.hypot(tx, ty)
        if tn < 1e-6:
            tx, ty = nx, ny
            tn = 1.0
        tx, ty = tx / tn, ty / tn
        reach = 0.6
        return (ball[0] + tx * reach + nx * 0.10,
                ball[1] + ty * reach + ny * 0.10), True

    return goal, False


def _nearest_wall_normal(p) -> tuple[float, float]:
    """Inward normal of the wall closest to point p."""
    hx, hy = config.HALF_LENGTH_M, config.HALF_WIDTH_M
    gaps = ((hx - p[0], (-1.0, 0.0)), (p[0] + hx, (1.0, 0.0)),
            (hy - p[1], (0.0, -1.0)), (p[1] + hy, (0.0, 1.0)))
    return min(gaps)[1]


def route_around(me_pos, ball, target, clearance: float | None = None):
    """The target, or a via-point beside the ball if the ball is in the way.

    Every approach here is "drive straight at the point behind the ball". From
    the WRONG side of the ball that straight line runs through it, so the
    robot arrives by shoving the ball the opposite way -- toward its own goal,
    when the point it wanted was goal-side of the ball. Go round instead: aim
    for a point level with the ball, off to whichever side we are already on.
    """
    if clearance is None:
        clearance = config.ROUTE_CLEARANCE_M
    dx, dy = target[0] - me_pos[0], target[1] - me_pos[1]
    L2 = dx * dx + dy * dy
    if L2 < 1e-9:
        return target
    bx, by = ball[0] - me_pos[0], ball[1] - me_pos[1]
    t = (bx * dx + by * dy) / L2
    if t <= 0.0 or t >= 1.0:
        return target
    px, py = me_pos[0] + dx * t, me_pos[1] + dy * t
    if math.hypot(ball[0] - px, ball[1] - py) >= clearance:
        return target

    ax, ay = target[0] - ball[0], target[1] - ball[1]
    an = math.hypot(ax, ay)
    if an < 1e-6:
        return target
    ax, ay = ax / an, ay / an
    nx, ny = -ay, ax
    rel = (me_pos[0] - ball[0]) * nx + (me_pos[1] - ball[1]) * ny
    side = 1.0 if rel >= 0.0 else -1.0
    # Level with the ball and slightly toward the target, so the next leg is
    # a short straight run in behind it.
    off = clearance * config.ROUTE_SIDE_MULT
    for sd in (side, -side):
        via = (ball[0] + nx * sd * off + ax * clearance * 0.5,
               ball[1] + ny * sd * off + ay * clearance * 0.5)
        clamped = reachable_point(via)
        if math.hypot(clamped[0] - via[0], clamped[1] - via[1]) < off * 0.5:
            return clamped
    return target


def strike_distance(pos, theta, ball, aim) -> float:
    """How far away a robot is from STRIKING the ball, in metres of driving.

    Straight-line distance to the ball says two robots are level when one is
    lined up behind it and the other is on the wrong side, facing away. Traced
    against the faster human proxy, that is how most goals were conceded: the
    AI claimed a ball it was "as close to", spent 2-3 s turning and routing
    round it, and the opponent drove straight through it and scored.

    This is the time for an identical robot to reach the point behind the
    ball on its own shooting line -- going round the ball if it has to, and
    turning on the spot between legs, as the striker actually drives --
    expressed back in metres at top speed so every existing margin keeps its
    meaning. Current state only; nothing about the opponent is learned.
    """
    from ai.controller import limits
    v, w = limits()
    dx, dy = aim[0] - ball[0], aim[1] - ball[1]
    n = max(math.hypot(dx, dy), 1e-6)
    ux, uy = dx / n, dy / n
    stand = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
             + config.BALL_RADIUS_M)
    strike = reachable_point((ball[0] - ux * stand, ball[1] - uy * stand))
    via = route_around(pos, ball, strike)
    pts = [pos, via, strike] if via is not strike else [pos, strike]
    length, turn, heading = 0.0, 0.0, theta
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-6:
            continue
        hd = math.atan2(b[1] - a[1], b[0] - a[0])
        turn += abs(wrap_angle(hd - heading))
        heading = hd
        length += seg
    turn += abs(wrap_angle(math.atan2(uy, ux) - heading))
    return length + turn * v / max(w, 1e-6)


def _contested(ball, opp_pos) -> bool:
    """The other robot is at the ball: a contest, not an obstacle to avoid.

    Steering round the opponent's body is right on the way somewhere. Beside
    the ball it steered the AI away from the very contest it had claimed.
    """
    return (config.AVOID_SKIP_CONTESTED_M > 0.0
            and math.hypot(ball[0] - opp_pos[0], ball[1] - opp_pos[1])
            < config.AVOID_SKIP_CONTESTED_M)


def avoid_robot(me_pos, target, opp_pos, clearance: float | None = None):
    """The target, or a via-point round the other robot if it is in the way.

    route_around() only knows about the ball. The other robot is a far bigger
    obstacle, and with no referee reset it is the one that decides matches: a
    human-shaped opponent parks nose-first on a ball against a wall, the free
    side of the ball is on the far side of it, and a straight-line approach
    ran into its body, jammed, escaped, and tried again for 115 seconds.
    """
    if clearance is None:
        clearance = config.AVOID_CLEARANCE_M
    # A target at the other robot is a target we must contest, not avoid.
    if math.hypot(target[0] - opp_pos[0],
                  target[1] - opp_pos[1]) < clearance * 0.9:
        return target
    # Already touching it: routing round a body pressed against ours only
    # spins us on the spot. Traced against the runner: 5 s of "avoid:turn"
    # from the kickoff, pinned by it, with the ball free 1.5 m away.
    if (config.AVOID_NOT_IN_CONTACT
            and math.hypot(me_pos[0] - opp_pos[0], me_pos[1] - opp_pos[1])
            < clearance * 0.85):
        return target
    return route_around(me_pos, opp_pos, target, clearance)


def goalmouth_sweep(ball, attack_dir: float, me_pos=None):
    """A ball in front of OUR goal, too close to the line to get behind.

    The robot's centre cannot come within its circumradius of the end wall, so
    any ball within about half a metre of our own goal line has no reachable
    "behind" on the line to the opponent's goal. The striker then drove at a
    point clamped to the edge of the pitch, level with the ball, and never
    once satisfied its own line-up test: in the traces, 1.5 s circling a
    stationary ball in our own goal mouth until the opponent tapped it in.

    What does exist is the goal line itself. Sweep the ball along it, out
    through the nearer post -- exactly what wall_extraction() does for a ball
    against our end wall outside the mouth.
    """
    hx = config.HALF_LENGTH_M
    x_att = ball[0] * attack_dir
    if x_att > -(hx - config.GOALMOUTH_DEPTH_M):
        return None
    if abs(ball[1]) > config.HALF_GOAL_M + config.GOALMOUTH_WIDEN_M:
        return None

    stand = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
             + config.BALL_RADIUS_M)
    best, best_cost = None, 1e9
    for ty in (1.0, -1.0):
        approach = reachable_point((ball[0], ball[1] - ty * stand))
        aim = reachable_point((ball[0] + attack_dir * 0.25,
                               ball[1] + ty * 1.2))
        cost = 0.0
        # Toward the centre of our own mouth is not a clearance, it is a pass
        # across the face of the goal. Traced: chosen because its approach
        # point was nearer, it ran the ball along the line and in.
        if ball[1] * ty < 0.0 and abs(ball[1]) > 0.04:
            continue
        if me_pos is not None:
            cost += math.hypot(approach[0] - me_pos[0],
                               approach[1] - me_pos[1])
        if cost < best_cost:
            best, best_cost = (aim, approach, (0.0, ty)), cost
    return best


def robot_circumradius() -> float:
    """Worst-case distance from the robot centre to any part of it."""
    hl = config.ROBOT_LENGTH_M / 2.0
    hw = config.ROBOT_WIDTH_M / 2.0
    r = config.HORN_WIDTH_M / 2.0
    tip = math.hypot(hl + config.HORN_LENGTH_M + r,
                     config.HORN_GAP_M / 2.0 + config.HORN_WIDTH_M + r)
    return max(math.hypot(hl, hw), tip)


def clamp_target(p, attack_dir: float = 0.0):
    """Pull a target inside the region the robot can actually occupy.

    This is the fix for the robot grinding itself into walls. Several
    candidate behaviours aim AT the goal, which sits exactly on the end wall
    -- a point the robot can never reach. `drive_to` then commands full
    throttle at it forever, the robot wedges against the wall, and the jam
    escape frees it only for the planner to re-target the identical
    unreachable point a frame later.

    Measured before this existed: the robot spent 23 s of every 60 s stalled
    against a wall, 987 of 1167 jam samples with no opponent anywhere near.

    Clamping is deliberately conservative (circumradius, not the oriented
    extent) because being a few centimetres shy of a wall costs nothing,
    whereas targeting a point outside the pitch costs everything.
    """
    margin = robot_circumradius() + 0.02
    hx = max(config.HALF_LENGTH_M - margin, 0.05)
    hy = max(config.HALF_WIDTH_M - margin, 0.05)

    # THE GOAL MOUTH IS NOT A WALL.
    #
    # Clamping every target to the pitch also clamped the goal itself, which
    # sits on the end line. The robot then drove the ball to a point short of
    # the line, decided it had ARRIVED, and stopped -- parked with the ball in
    # its pocket 117 mm from scoring. Measured: it converted only 3 of 5
    # completely unopposed chances, and the failures were all this.
    #
    # Inside the mouth the robot may legitimately aim past the line; the wall
    # collision already makes the same exception for the ball, and the
    # physical end wall still stops the robot itself.
    y = clamp(p[1], -hy, hy)

    # ...but only the goal we are ATTACKING.
    #
    # Allowing the overshoot at both ends let a defensive target land inside
    # our OWN net, so the robot drove at its own goal line and shepherded the
    # ball in with it. Measured: camped at mean x = -1.11 with its own goal
    # at -1.70, ball in our half 73% of the time, losing 0-7.
    #
    # attack_dir 0.0 means "caller did not say", so no overshoot is allowed --
    # the safe default.
    in_mouth = abs(y) < config.HALF_GOAL_M - config.BALL_RADIUS_M
    if in_mouth and attack_dir != 0.0:
        over = config.HALF_LENGTH_M + 0.10
        if attack_dir > 0:
            return (clamp(p[0], -hx, over), y)
        return (clamp(p[0], -over, hx), y)
    return (clamp(p[0], -hx, hx), y)


def dribble_target(ball, goal, me_pos, attack_dir: float):
    """Where to put the robot's NOSE to carry the ball rather than poke it.

    With no kicker, the only way to move the ball a long way accurately is to
    trap it in the pocket and drive. That needs the robot to arrive already
    pointing where the ball should go -- arriving at the right place with the
    wrong heading just knocks the ball loose.

    Returns (approach_point, desired_heading).
    """
    aim, _diverted = effective_aim(ball, goal, attack_dir)
    dx, dy = aim[0] - ball[0], aim[1] - ball[1]
    n = max(math.hypot(dx, dy), 1e-6)
    ux, uy = dx / n, dy / n

    # Stand off by exactly the pocket depth so the ball ends up captured,
    # not struck.
    stand = (config.ROBOT_LENGTH_M / 2.0
             + config.HORN_LENGTH_M * 0.45
             + config.BALL_RADIUS_M)
    approach = (ball[0] - ux * stand, ball[1] - uy * stand)
    return approach, math.atan2(uy, ux)


class StuckMonitor:
    """Detects a jammed robot and produces an escape manoeuvre.

    Jammed means: we are asking for speed and not getting it. That covers
    being wedged against a wall, shoved by the other robot, or nose-down in a
    corner, without needing to know which.

    The escape is deliberately dumb and time-boxed -- reverse while turning
    away from the nearest wall. Anything cleverer risks arguing with the
    planner, and the only job here is to break the deadlock and hand control
    back.
    """

    def __init__(self) -> None:
        self.jam_time = 0.0
        self.escape_left = 0.0
        self.escape_turn = 1.0
        self.escapes = 0
        self.escape_dir = -1.0
        self._trail: list[tuple[float, float, float, float]] = []

    def update(self, dt: float, pos, theta: float, speed: float,
               commanded_v: float, commanded_w: float = 0.0) -> bool:
        """Returns True while an escape is in progress.

        Detection uses ACTUAL DISPLACEMENT, not the believed velocity.
        That distinction matters: a robot wedged against a wall still has
        spinning wheels, its encoders faithfully report that rotation, and
        the estimator -- which fuses encoders for velocity -- therefore
        believes it is moving at full speed. Velocity is precisely the signal
        that lies in this situation. Position comes from vision and cannot.
        """
        self._trail.append((pos[0], pos[1], dt, theta))
        window = 0.0
        i = len(self._trail) - 1
        while i > 0 and window < config.STUCK_DETECT_S:
            window += self._trail[i][2]
            i -= 1
        if i > 0:
            del self._trail[:i]

        if self.escape_left > 0.0:
            self.escape_left -= dt
            if self.escape_left <= 0.0:
                self._trail.clear()
            return self.escape_left > 0.0

        moved = 0.0
        turned = 0.0
        full = window >= config.STUCK_DETECT_S * 0.8 and len(self._trail) > 2
        if full:
            x0, y0, _, _ = self._trail[0]
            moved = math.hypot(pos[0] - x0, pos[1] - y0)
            for a, b in zip(self._trail, self._trail[1:]):
                turned += abs(wrap_angle(b[3] - a[3]))

        trying = abs(commanded_v) > 0.25
        stalled = full and moved < config.STUCK_MIN_TRAVEL_M

        # A TURN CAN JAM TOO. Rotating on the spot beside a wall swings the
        # horn tips into it, and the robot grinds there asking for full yaw
        # and getting almost none. Watched in the goal traces: 1.4 s turning
        # +135 to +150 degrees with the ball stationary in our goal mouth,
        # until the opponent arrived and scored. The speed test above cannot
        # see it, because a turn on the spot commands no forward speed at all.
        if config.STUCK_DETECT_YAW:
            from ai.controller import limits
            _, w_max = limits()
            spinning = (abs(commanded_w) > 0.5 * w_max
                        and abs(commanded_v) < 0.25)
            if (spinning and full
                    and turned < math.radians(config.STUCK_MIN_TURN_DEG)):
                trying = True
                stalled = True

        if trying and stalled:
            self.jam_time += dt
        else:
            self.jam_time = max(0.0, self.jam_time - dt * 2.0)

        if self.jam_time > config.STUCK_DETECT_S:
            self.jam_time = 0.0
            self._trail.clear()
            self.escape_left = config.STUCK_ESCAPE_S
            self.escapes += 1
            # Turn toward the open side of the pitch, not into the wall we
            # are presumably already touching.
            self.escape_turn = -1.0 if pos[1] > 0 else 1.0
            if abs(pos[0]) > config.HALF_LENGTH_M - 0.3:
                self.escape_turn = -1.0 if pos[1] > 0 else 1.0
            # Back away from the nearest wall, which is not always backwards.
            # A robot whose TAIL is on the wall reverses into it and jams
            # again the moment the escape ends.
            self.escape_dir = -1.0
            if config.STUCK_ESCAPE_AWAY_FROM_WALL:
                nx, ny = _nearest_wall_normal(pos)
                if math.cos(theta) * nx + math.sin(theta) * ny > 0.0:
                    self.escape_dir = 1.0
            return True
        return False

    def command(self) -> tuple[float, float]:
        from ai.controller import limits
        v_max, w_max = limits()
        return (self.escape_dir * 0.75 * v_max,
                self.escape_turn * 0.8 * w_max)

    def reset(self) -> None:
        self.jam_time = 0.0
        self.escape_left = 0.0


# ---------------------------------------------------------------------------
# Defending
# ---------------------------------------------------------------------------

def possessor(belief, margin: float = 0.04):
    """Which robot, if any, currently controls the ball. Belief only."""
    reach = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M
             + config.BALL_RADIUS_M + margin)
    best, best_d = None, reach
    for i, r in enumerate(belief.robots):
        if not r.valid:
            continue
        px = r.pos[0] + math.cos(r.theta) * (config.ROBOT_LENGTH_M / 2.0
                                             + config.HORN_LENGTH_M * 0.5)
        py = r.pos[1] + math.sin(r.theta) * (config.ROBOT_LENGTH_M / 2.0
                                             + config.HORN_LENGTH_M * 0.5)
        d = math.hypot(belief.ball_pos[0] - px, belief.ball_pos[1] - py)
        if d < best_d:
            best, best_d = i, d
    return best


def carried_ball_at(ball, carrier, t: float):
    """Where a CARRIED ball will be in t seconds.

    A carried ball does not roll and does not decelerate -- it goes wherever
    the carrier goes, at the carrier's speed. The normal ball predictor
    assumes free rolling with friction, so it always undershoots a ball being
    driven, and every defensive position computed from it lands behind the
    play.

    Measured effect of getting this wrong: while the opponent drove straight
    at goal, the defender sat a median 129 mm off the shot line and was
    actually in the way only 30% of the time.
    """
    if carrier is None or not carrier.valid:
        return ball
    # Propagate the carrier along its own heading, then keep the ball in
    # front of it where the pocket is.
    v = carrier.v
    th = carrier.theta + carrier.omega * t * 0.5      # mid-point heading
    cx = carrier.pos[0] + v * math.cos(th) * t
    cy = carrier.pos[1] + v * math.sin(th) * t
    lead = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
            + config.BALL_RADIUS_M)
    return (cx + math.cos(th) * lead, cy + math.sin(th) * lead)


def cutoff_point(me, ball, ball_speed, own_goal, travel_time_fn,
                 carrier=None):
    """The furthest-forward point on the ball->our-goal line we can reach FIRST.

    This is the defensive primitive the AI was missing, and its absence is
    exactly why a player who simply drives forward in a straight line wins
    every time.

    Chasing the ball means arriving where it no longer is -- permanently one
    step behind a carrier, forever. What a defender must do instead is give
    up the chase and go stand somewhere the ball has to come to, as far up
    the pitch as it can still get there first. RoboCup keepers do the same
    thing: project the ball's path onto the goal line and occupy the
    intersection rather than following the ball.

    Walks outward from the ball and returns the NEAREST such point, so the
    robot cuts the attack off early rather than retreating all the way onto
    its own line and conceding the whole pitch.
    """
    bx, by = ball
    gx, gy = own_goal
    dx, dy = gx - bx, gy - by
    span = math.hypot(dx, dy)
    if span < 1e-6:
        return own_goal
    ux, uy = dx / span, dy / span

    speed = max(ball_speed, 0.35)      # a stationary ball will not stay so
    steps = 14
    fallback = None
    for i in range(1, steps + 1):
        d = span * i / steps
        t_ball = d / speed
        # Aim at the line as it WILL BE when the ball gets there, not as it
        # is now. With a carrier driving at 2 m/s, 0.4 s of flight is most of
        # a metre, and defending the stale line means defending nothing.
        if carrier is not None:
            fb = carried_ball_at(ball, carrier, t_ball)
            fdx, fdy = gx - fb[0], gy - fb[1]
            fn = math.hypot(fdx, fdy)
            if fn > 1e-6:
                p = (fb[0] + fdx / fn * d, fb[1] + fdy / fn * d)
            else:
                p = (bx + ux * d, by + uy * d)
        else:
            p = (bx + ux * d, by + uy * d)
        t_us = travel_time_fn(me, p)
        # Need a real margin: arriving simultaneously is arriving late, and
        # the robot still has to be facing the right way to do anything.
        if t_us + config.DEFEND_TIME_MARGIN_S <= t_ball:
            return p
        fallback = p
    # Cannot cut it off anywhere -- sit on our own line and make them beat us.
    guard = min(0.35, span * 0.5)
    return (gx - ux * guard, gy - uy * guard)


class CaptureMonitor:
    """Take the ball into the pocket, in one committed movement.

    THE MISSING ACTION
    ------------------
    The planner's vocabulary was "stand behind the ball" and "drive at the
    goal". Neither of those is "pick the ball up". So on arriving next to the
    ball it alternated between them forever -- measured: within 242 mm of the
    ball it changed its mind every 0.30 s, and chose `reverse` 27 times while
    standing right beside it. It could reach the ball and never take it.

    Capturing is not a thing to deliberate about. Once the ball is close and
    roughly ahead, the correct move is always the same: drive at it, put it
    between the horns, and do not re-plan halfway through. Re-planning during
    a capture is what knocks the ball loose.

    So this runs ABOVE the planner and holds for a fixed window, like the jam
    escape. It steers only gently -- hard steering while closing is what
    sweeps the ball aside instead of collecting it.
    """

    def __init__(self) -> None:
        self.active = 0.0
        self.captures = 0

    def range_m(self) -> float:
        return (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M
                + config.BALL_RADIUS_M + config.CAPTURE_EXTRA_M)

    def update(self, dt: float, me, ball_pos, holding: bool,
               aim=None, opp=None) -> bool:
        if self.active > 0.0:
            self.active -= dt
            # Stop early the moment we actually have it -- the planner should
            # take over and drive, not keep lunging.
            if holding:
                self.active = 0.0
            return self.active > 0.0

        if holding:
            return False

        dx = ball_pos[0] - me.pos[0]
        dy = ball_pos[1] - me.pos[1]
        rng = math.hypot(dx, dy)
        if rng > self.range_m():
            return False

        err = abs(wrap_angle(math.atan2(dy, dx) - me.theta))
        if err > math.radians(config.CAPTURE_CONE_DEG):
            # Too far off to collect it cleanly; let the planner line up.
            return False

        # DO NOT TAKE THE BALL FACING THE WRONG WAY.
        #
        # Capturing whenever the ball is merely in front was actively
        # counterproductive: the robot took possession pointing anywhere,
        # then had to turn while carrying, and turning while carrying is
        # exactly what sheds the ball (57% retention straight vs 20% through
        # a hard turn). The result was a capture/turn/drop/re-capture loop --
        # measured at 88% of possession time spent turning and only 12%
        # driving, which is why it could hold the ball and never score.
        #
        # So possession is only worth taking when it can be USED: the robot
        # must already be pointing somewhere near where the ball should go.
        # Otherwise leave the ball alone and let the planner walk around it
        # first. This is what the one-trick SimpleBot does, and why it scores.
        if aim is not None:
            to_goal = math.atan2(aim[1] - ball_pos[1], aim[0] - ball_pos[0])
            if abs(wrap_angle(to_goal - me.theta)) > math.radians(
                    config.CAPTURE_AIM_CONE_DEG):
                return False

        # DO NOT CONTEST INSIDE A SCRUM.
        #
        # Taking the ball while the opponent is on top of it does not win
        # possession, it wins a wrestling match. Measured: 82% of the time the
        # AI held the ball it was in body contact with the opponent, and it
        # achieved only 39% of its commanded yaw -- physically pinned. It
        # could hold the ball and be unable to turn or drive it anywhere,
        # which is precisely why it never scored.
        #
        # Equal robots means a scrum is a coin toss that costs both players
        # their speed. The elegant move is to stay out of it, keep separation,
        # and take the ball once the opponent has committed past it -- which
        # the planner's positioning candidates handle.
        if opp is not None and opp.valid:
            if (math.hypot(opp.pos[0] - ball_pos[0],
                           opp.pos[1] - ball_pos[1])
                    < config.CAPTURE_MIN_OPP_SEP_M):
                return False

        self.active = config.CAPTURE_COMMIT_S
        self.captures += 1
        return True

    def command(self, me, ball_pos) -> tuple[float, float]:
        from ai.controller import limits
        v_max, w_max = limits()
        dx = ball_pos[0] - me.pos[0]
        dy = ball_pos[1] - me.pos[1]
        err = wrap_angle(math.atan2(dy, dx) - me.theta)
        # Gentle steering only. A hard turn while closing sweeps the ball
        # sideways out of the pocket instead of collecting it.
        w = clamp(err * config.CAPTURE_STEER_GAIN, -w_max * 0.35, w_max * 0.35)
        return v_max * config.CAPTURE_SPEED_FRAC, w

    def reset(self) -> None:
        self.active = 0.0


class CarryController:
    """What to do once the ball IS in the pocket: angle, then drive.

    Capturing the ball achieved nothing on its own -- the AI would collect it
    and then hand back to a planner that immediately chose something else,
    so it took the ball 172 times and scored 0 goals.

    The one-trick SimpleBot scores freely with exactly two moves: turn until
    pointed at the goal, then drive straight. That works because a passive
    pocket cannot survive a hard turn (measured: 57% retention straight, 20%
    through a hard turn), so the only way to move the ball a long way is to
    aim FIRST and then go in a straight line.

    So while holding the ball this does the same thing, but aims at the
    reachability-chosen target rather than the goal centre:

        heading error large  ->  turn slowly, barely moving, to keep the ball
        heading error small  ->  drive straight, full commitment
    """

    def __init__(self) -> None:
        self.mode = "idle"
        self._driving = False

    def command(self, me, ball_pos, aim) -> tuple[float, float]:
        from ai.controller import limits
        v_max, w_max = limits()

        err = wrap_angle(math.atan2(aim[1] - me.pos[1],
                                    aim[0] - me.pos[0]) - me.theta)
        tol = (math.radians(config.CARRY_RESUME_DEG) if self._driving
               else math.radians(config.CARRY_ALIGN_DEG))

        if abs(err) > tol:
            self._driving = False
            self.mode = "carry-turn"
            # Turn slowly and creep forward a little. Turning on the spot
            # with the ball loose in the pocket lets it roll out sideways;
            # a little forward pressure holds it against the face.
            w = clamp(err * 2.0, -w_max * config.CARRY_TURN_FRAC,
                      w_max * config.CARRY_TURN_FRAC)
            return v_max * config.CARRY_TURN_SPEED_FRAC, w

        self._driving = True
        self.mode = "carry-drive"
        # Aimed. Go, and steer only enough to hold the line.
        w = clamp(err * 1.2, -w_max * 0.20, w_max * 0.20)
        return v_max * config.CARRY_DRIVE_FRAC, w

    def reset(self) -> None:
        self._driving = False
        self.mode = "idle"


class DeadlockBreaker:
    """Break a head-on shoving contest instead of losing it slowly.

    WHAT THIS LOOKS LIKE ON SCREEN
    ------------------------------
    Two robots nose to nose, the ball pinned between their horns, neither
    able to move it, both at full throttle. The planner flickers between
    `defend` and `carry-drive` every frame and the velocity command changes
    sign frame to frame. From the outside it reads as the robot standing next
    to the ball twitching and doing nothing -- which is exactly what it is.

    No aggregate statistic showed this. Possession looked fine, contact
    looked like ordinary contest, speed looked low but plausible. It only
    became obvious by drawing the frames and looking at them.

    Equal robots cannot win a head-on push, so continuing to push is the one
    guaranteed way to make no progress. The correct move is to disengage --
    back out and come round the side, where the ball can actually be taken.
    """

    def __init__(self) -> None:
        self.jam = 0.0
        self.active = 0.0
        self.side = 1.0
        self.breaks = 0

    def update(self, dt: float, me, opp, ball_pos, attack_dir: float = 1.0) -> bool:
        if config.DEADLOCK_MODE == "off":
            return False
        if self.active > 0.0:
            self.active -= dt
            return self.active > 0.0

        if opp is None or not opp.valid:
            self.jam = 0.0
            return False

        # Nose to nose: close, and headings roughly opposed.
        sep = math.hypot(opp.pos[0] - me.pos[0], opp.pos[1] - me.pos[1])
        facing = math.cos(wrap_angle(opp.theta - me.theta))
        contact_d = config.ROBOT_LENGTH_M + config.HORN_LENGTH_M + 0.06

        # Ball caught in the middle of it.
        mid = ((me.pos[0] + opp.pos[0]) / 2.0, (me.pos[1] + opp.pos[1]) / 2.0)
        ball_between = math.hypot(ball_pos[0] - mid[0],
                                  ball_pos[1] - mid[1]) < contact_d * 0.6

        if sep < contact_d and facing < -0.4 and ball_between:
            self.jam += dt
        else:
            self.jam = max(0.0, self.jam - dt * 2.0)

        if self.jam > config.DEADLOCK_DETECT_S:
            self.jam = 0.0
            self.active = config.DEADLOCK_ESCAPE_S
            self.breaks += 1
            # Peel toward the side with more room, so the way round is open.
            self.side = -1.0 if me.pos[1] > 0.0 else 1.0
            if config.DEADLOCK_MODE == "pivot":
                # Rotate so the ball rolls off our horns toward THEIR goal:
                # turning left swings the nose left, and the ball squeezed on
                # the face escapes to the right, and vice versa. Pick the
                # rotation whose escape side points up the pitch.
                fx, fy = math.cos(me.theta), math.sin(me.theta)
                right = (fy, -fx)            # local -y in world
                self.side = 1.0 if right[0] * attack_dir > 0.0 else -1.0
            return True
        return False

    def command(self) -> tuple[float, float]:
        from ai.controller import limits
        v_max, w_max = limits()
        if config.DEADLOCK_MODE == "pivot":
            # Keep the pressure on and twist: the ball squirts out sideways
            # on our side instead of being handed over.
            return 0.5 * v_max, self.side * w_max
        # Back out and swing the nose away: a committed disengage, not a nudge.
        return -0.85 * v_max, self.side * 0.85 * w_max

    def reset(self) -> None:
        self.jam = 0.0
        self.active = 0.0


class OrientToBall:
    """Point the horns at the ball when it is close enough to take.

    The robot's only ball-handling surface is its front. Reversing is the
    fastest way to reach a defensive spot and it keeps the nose pointing the
    wrong way, so the robot would arrive in the right PLACE facing entirely
    the wrong direction -- watched frame by frame, it backpedalled in front
    of an advancing attacker for a second and a half with its horns aimed
    away from the ball the whole time, never able to touch it.

    Forcing defensive moves to drive forwards instead fixed the facing and
    cost sixteen goals, because turning round to retreat is far too slow.

    So: retreat backwards, but once the ball is close enough to actually
    take, stop retreating and turn to meet it. Getting there fast and
    arriving usable are different problems and need different answers.
    """

    def __init__(self) -> None:
        self.active = 0.0
        self.turns = 0

    def update(self, dt: float, me, ball_pos, holding: bool) -> bool:
        if holding:
            self.active = 0.0
            return False
        if self.active > 0.0:
            self.active -= dt
            return self.active > 0.0

        rng = math.hypot(ball_pos[0] - me.pos[0], ball_pos[1] - me.pos[1])
        if rng > config.ORIENT_RANGE_M:
            return False
        err = abs(wrap_angle(math.atan2(ball_pos[1] - me.pos[1],
                                        ball_pos[0] - me.pos[0]) - me.theta))
        if err < math.radians(config.ORIENT_TOL_DEG):
            return False
        self.active = config.ORIENT_COMMIT_S
        self.turns += 1
        return True

    def command(self, me, ball_pos) -> tuple[float, float]:
        from ai.controller import limits
        v_max, w_max = limits()
        err = wrap_angle(math.atan2(ball_pos[1] - me.pos[1],
                                    ball_pos[0] - me.pos[0]) - me.theta)
        # Turn hard, creeping forward so we keep closing while we swing round.
        w = clamp(err * 3.0, -w_max, w_max)
        return v_max * config.ORIENT_SPEED_FRAC, w

    def reset(self) -> None:
        self.active = 0.0


def reachable_point(p):
    """Pull a point inside the region the robot's CENTRE can occupy."""
    margin = robot_circumradius() + 0.02
    hx = max(config.HALF_LENGTH_M - margin, 0.05)
    hy = max(config.HALF_WIDTH_M - margin, 0.05)
    x, y = clamp(p[0], -hx, hx), clamp(p[1], -hy, hy)
    c = config.ARENA_CORNER_CHAMFER_M
    if c > 0.0:
        sx = 1.0 if x > 0.0 else -1.0
        sy = 1.0 if y > 0.0 else -1.0
        lim = (config.HALF_LENGTH_M + config.HALF_WIDTH_M - c
               - margin * math.sqrt(2.0))
        over = sx * x + sy * y - lim
        if over > 0.0:
            x -= sx * over * 0.5
            y -= sy * over * 0.5
    return (x, y)


def on_chamfer(ball):
    """(sx, sy) of the chamfer the ball is against, or None."""
    c = config.ARENA_CORNER_CHAMFER_M
    if c <= 0.0:
        return None
    sx = 1.0 if ball[0] > 0.0 else -1.0
    sy = 1.0 if ball[1] > 0.0 else -1.0
    gap = (config.HALF_LENGTH_M + config.HALF_WIDTH_M - c
           - (sx * ball[0] + sy * ball[1])) / math.sqrt(2.0)
    if gap >= WALL_MARGIN_M:
        return None
    # Only if the chamfer is the NEAREST surface. A ball flat against the end
    # wall a hand's width from the chamfer is an end-wall ball.
    if gap > min(config.HALF_LENGTH_M - abs(ball[0]),
                 config.HALF_WIDTH_M - abs(ball[1])):
        return None
    return (sx, sy)


def wall_gap(p, n) -> float:
    """Distance from p to the wall whose inward normal is n."""
    if abs(n[1]) > 0.9:
        return config.HALF_WIDTH_M - abs(p[1])
    if abs(n[0]) > 0.9:
        return config.HALF_LENGTH_M - abs(p[0])
    sx = 1.0 if p[0] > 0.0 else -1.0
    sy = 1.0 if p[1] > 0.0 else -1.0
    return (config.HALF_LENGTH_M + config.HALF_WIDTH_M
            - config.ARENA_CORNER_CHAMFER_M
            - (sx * p[0] + sy * p[1])) / math.sqrt(2.0)


def is_reachable(p) -> bool:
    q = reachable_point(p)
    return abs(q[0] - p[0]) < 1e-9 and abs(q[1] - p[1]) < 1e-9


def wall_extraction(ball, attack_dir: float, me_pos=None,
                    me_theta: float = 0.0, banned=()):
    """How to attack a ball that is pinned against a wall or in a corner.

    THE GEOMETRY THAT DOES NOT EXIST
    --------------------------------
    Every attacking policy here works by standing behind the ball on the
    ball-to-aim line and driving through it. For a ball in a corner that point
    is always DEEPER INTO THE CORNER -- outside the arena, where the robot's
    centre can never be. Measured over 6 matches x 120 s: 78.8% of all ticks
    spent striking were driving at a point outside the pitch, the robot was
    wedged against a wall under throttle for 45.2% of the match, and the ball
    had to be teleported back to the centre by the stuck rule 3.0 times a
    match. "Get behind it" is not a difficult move here; it is an impossible
    one, and no amount of tuning fixes an impossible move.

    WHAT DOES EXIST
    ---------------
    The wall itself. The robot can always drive ALONG a wall, and doing so
    sweeps the ball along it with the front and inside edge. So a pinned ball
    is not struck at a goal, it is walked out:

      * against an END wall  -- travel along it toward y = 0, which walks the
        ball straight into the mouth of the goal that sits there.
      * against a SIDE wall  -- travel along it toward the opponent's end.
      * in a CORNER          -- take whichever of those two leaves the corner,
        preferring the end wall when it is the opponent's end, because that
        run finishes at the goal.

    Returns (aim, approach_point) with both guaranteed reachable, or None when
    the ball is in open play and the ordinary geometry applies.
    """
    attacking_end = (ball[0] * attack_dir) > 0.0
    ch = on_chamfer(ball)
    if ch is not None and config.CHAMFER_AWARE:
        # THE CHAMFER IS A WALL TOO, and the useful one: it is the only
        # surface that turns a ball running along a side wall onto the end
        # wall, where the goal is. Treated as a square corner, the ball was
        # left sitting on it with every approach point outside the pitch.
        sx, sy = ch
        k = 1.0 / math.sqrt(2.0)
        stand = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
                 + config.BALL_RADIUS_M)
        n_in = (-sx * k, -sy * k)
        off = robot_circumradius() + 0.02
        opts = [((sx * k, -sy * k), 0.0 if attacking_end else 0.6),
                ((-sx * k, sy * k), 0.6 if attacking_end else 0.0)]
        best, best_cost = None, 1e9
        for (tx, ty), pen in opts:
            run = stand + config.SWEEP_RUNWAY_M
            approach = reachable_point((ball[0] - tx * run + n_in[0] * off,
                                        ball[1] - ty * run + n_in[1] * off))
            aim = (ball[0] + tx * 1.2 + n_in[0] * 0.05,
                   ball[1] + ty * 1.2 + n_in[1] * 0.05)
            cost = pen + (100.0 if (tx, ty) in banned else 0.0)
            if me_pos is not None:
                cost += math.hypot(approach[0] - me_pos[0],
                                   approach[1] - me_pos[1])
            if cost < best_cost:
                best, best_cost = (aim, approach, (tx, ty)), cost
        return best

    nx, ny = ball_against_wall(ball)
    if nx == 0.0 and ny == 0.0:
        return None

    stand = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
             + config.BALL_RADIUS_M)
    # Stand off the wall by exactly as much as the robot needs, so the run is
    # parallel to it rather than into it.
    off_x = nx * (robot_circumradius() + 0.02)
    off_y = ny * (robot_circumradius() + 0.02)

    # The escape directions that physically exist. A diagonal is NOT among
    # them: pushing a ball out of a corner along the diagonal would mean
    # standing in the corner behind it, which is the very position that does
    # not exist inside the arena. The robot can only travel along a wall.
    options = []
    if nx != 0.0:
        # Along the END wall. Toward the centre line at the ATTACKING end,
        # because the goal mouth is cut into that wall and the run finishes
        # inside it.
        #
        # AWAY from the centre line at OUR end. ball_against_wall() only
        # reports the end wall outside the mouth (|y| > HALF_GOAL_M), so
        # sweeping toward y = 0 there walks the ball across the face of our
        # own goal -- the single worst place on the pitch to put it -- and
        # finishes by pushing it in. Away from the mouth is a longer road out
        # via the corner, but it is never an own goal.
        side = -1.0 if ball[1] > 0.0 else 1.0
        options.append((0.0, side if attacking_end else -side,
                        0.0 if attacking_end else 0.35))
    if ny != 0.0:
        # Along the SIDE wall, up the pitch.
        options.append((attack_dir, 0.0, 0.0 if not attacking_end else 0.35))
    if not options:
        options.append((0.0, -1.0 if ball[1] > 0.0 else 1.0, 0.0))
    # The other way along each wall, as a fallback. Never preferred, but a
    # preferred direction can be a dead end: toward their end along a side
    # wall runs the ball into the corner, and the traces showed the robot
    # grinding it there for 15-24 s. When the progress watchdog bans the
    # preferred run, this is the way out.
    # Not along our own end wall, though: reversed, that run crosses the face
    # of our goal.
    for tx, ty, pen in list(options):
        if tx == 0.0 and not attacking_end:
            continue
        options.append((-tx, -ty, pen + config.SWEEP_REVERSE_PENALTY))

    def build(tx, ty):
        # Start a run-up short of the ball. Closing in on the wall takes
        # travel, and from a start point one stand-off behind the ball the
        # robot was still 0.16 m out when it drew level, and drove past.
        run = stand + config.SWEEP_RUNWAY_M
        approach = reachable_point((ball[0] - tx * run + off_x,
                                    ball[1] - ty * run + off_y))
        aim = reachable_point((ball[0] + tx * 1.2 + off_x * 0.5,
                               ball[1] + ty * 1.2 + off_y * 0.5))
        return aim, approach

    # In a corner BOTH walls are available, and which one is right depends on
    # where the robot already is. Choosing by fixed preference could commit it
    # to the wall on the far side of the ball -- a long trip round, during
    # which the stuck timer runs out. Measured: corner episodes that fail do
    # not fail slowly, they cluster at 12.6-19.3 s against a 12.0 s stuck
    # timeout, i.e. the ball is not extracted so much as abandoned. So prefer
    # the run the robot can start NOW, with a penalty on the wall that leads
    # away from the opponent's goal.
    best, best_cost = None, 1e9
    for tx, ty, penalty in options:
        aim, approach = build(tx, ty)
        cost = penalty
        if (tx, ty) in banned:
            cost += 100.0
        if me_pos is not None and config.WALL_SWEEP_PICK_NEAREST:
            cost += math.hypot(approach[0] - me_pos[0], approach[1] - me_pos[1])
            # ...and on the turn needed to get onto that heading.
            cost += 0.25 * abs(wrap_angle(math.atan2(ty, tx) - me_theta))
        if cost < best_cost:
            best, best_cost = (aim, approach, (tx, ty)), cost
    return best


class ShadowDefender:
    """Stand between the ball and your own goal, facing the ball. Hold it.

    WHY THIS EXISTS
    ---------------
    Measured across every opponent, same seeds, swapping only our controller,
    with "control" being the 40-line tools/simplebot.py:

        opponent   AI scored / conceded      control scored / conceded
        simple           12 / 14                     8 / 2
        runner           20 / 11                    15 / 2
        human            11 / 22                    10 / 22

    The AI OUTSCORES the trivial policy against all three. It loses on goals
    conceded -- fourteen and eleven against two and two. Its attack is better
    and its defence is catastrophically worse.

    The reason is an accident of SimpleBot's geometry. Its target is the point
    behind the ball on the ball-to-THEIR-goal line, which is by construction
    on the near side of the ball to its OWN goal. So the thing it does to
    attack also keeps it permanently goal-side of the ball. It is a perfect
    defender without a single line of defensive code.

    The AI, once it decides the ball is not its to claim, hands over to the
    rollout planner's cover / cutoff / block placements, which do not hold
    that line -- and conceded seven times as often.

    Note what this is NOT: simply attacking all the time. That was measured
    and is much worse (-1.20 goal difference against -0.20), because the
    striker drives THROUGH the ball, and doing that while the opponent owns it
    surrenders both the ball and the position. The missing behaviour is to
    take the goal-side point and HOLD it, facing the ball, waiting for the
    ball to become claimable.

    The control law is the striker's, because that is the one that works on
    this drivetrain: turn on the spot until aimed, then drive, committed.
    """

    def __init__(self, attack_dir: float = 1.0) -> None:
        self.attack_dir = attack_dir
        self.driving = False

    def shadow_point(self, ball, ball_vel=(0.0, 0.0)):
        """The point to hold: goal-side of the ball, on the shooting line."""
        own_goal = (-self.attack_dir * config.HALF_LENGTH_M, 0.0)

        # Lead the ball, exactly as the striker does. Defending against where
        # the ball is going is the whole point of having an estimator.
        lead = config.SHADOW_LEAD_S
        if lead > 0.0:
            from ai.estimator import predict_ball
            bp, _ = predict_ball(ball, ball_vel, lead)
        else:
            bp = ball

        dx, dy = own_goal[0] - bp[0], own_goal[1] - bp[1]
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return bp
        ux, uy = dx / n, dy / n

        # Stand off along the ball-to-own-goal line, but never drop behind our
        # own goal line, and never further from the ball than the goal is.
        stand = min(config.SHADOW_STANDOFF_M, n * 0.9)
        return reachable_point((bp[0] + ux * stand, bp[1] + uy * stand))

    def command(self, dt, me, ball, ball_vel=(0.0, 0.0)):
        """Returns (v, omega, phase)."""
        from ai.controller import limits
        v_max, w_max = limits()
        target = self.shadow_point(ball, ball_vel)
        if config.USE_ROUTE_AROUND and config.ROUTE_AROUND_DEFENCE:
            target = route_around(me.pos, ball, target)

        dx, dy = target[0] - me.pos[0], target[1] - me.pos[1]
        rng = math.hypot(dx, dy)

        if rng < config.SHADOW_ARRIVE_M:
            # ON STATION. Face the ball and wait. The horns are on the front,
            # so a defender pointing anywhere else cannot take the ball when
            # it arrives -- and turning to meet it later is far too slow.
            err = wrap_angle(math.atan2(ball[1] - me.pos[1],
                                        ball[0] - me.pos[0]) - me.theta)
            self.driving = False
            if abs(err) < math.radians(config.SHADOW_FACE_TOL_DEG):
                return 0.0, 0.0, "hold"
            return 0.0, clamp(err * 2.5, -w_max, w_max), "hold:turn"

        err = wrap_angle(math.atan2(dy, dx) - me.theta)
        tol = math.radians(config.DIRECT_ANGLE_RESUME_DEG if self.driving
                           else config.DIRECT_ANGLE_TOL_DEG)
        if abs(err) > tol:
            self.driving = False
            return 0.0, math.copysign(w_max, err), "recover:turn"
        self.driving = True
        return v_max, 0.0, "recover"

    def reset(self) -> None:
        self.driving = False


class DirectStriker:
    """Attack the way the thing that actually scores attacks -- literally.

    WHY THIS EXISTS
    ---------------
    Measured over 14 matches of 120 s against the same opponent, from the same
    seeds, through the same probe, swapping ONLY which controller plays our
    side:

        SimpleBot on our side   9F  8A   shots 14.9   on target 1.71
                                att. third 28.8%   own third 26.9%
        the full AI             4F 12A   shots 10.2   on target 0.57
                                att. third 14.0%   own third 53.8%

    Three times fewer shots on target and half the territory, from a planner
    with state estimation, latency compensation, reachability and rollout MPC,
    against forty lines that turn on the spot and then drive straight.

    The sophistication was not losing because it was sophisticated. It was
    losing because the attacking CONTROL LAW was worse: it re-deliberated
    mid-approach, arrived facing the wrong way, and hovered. So the control law
    here is SimpleBot's, exactly -- two states, turn or drive, no arcs, no
    mid-run replanning.

    WHAT THE AI STILL BRINGS, AND WHY THIS IS NOT A DOWNGRADE
    --------------------------------------------------------
    SimpleBot reads ground truth and aims at the bare goal centre. This reads
    neither. It is handed:

      * the LATENCY-COMPENSATED, FORWARD-PREDICTED ball, and leads it by
        DIRECT_LEAD_S -- so it drives to where the ball is going to be, while
        SimpleBot drives at where the ball was.
      * the aim point chosen by the planner and the reachability analysis --
        the corner of the mouth the opponent cannot cover -- rather than the
        middle of the goal.

    Same control law, strictly better inputs. That is where the AI's advantage
    is supposed to come from, and this is the first version that actually
    spends it on something that converts.
    """

    def __init__(self, attack_dir: float = 1.0) -> None:
        self.driving = False
        self.mode = "lineup"
        self.strikes = 0
        self.pinned = False
        self.pin_hold = 0.0
        self.tangent = (1.0, 0.0)
        self.close_in = True
        self.attack_dir = attack_dir
        self.banned: dict = {}
        self._in_lane = False
        self._appr_dir = None
        self._appr_anchor = None
        self._appr_t = 0.0
        self.stalls = 0
        self._sweep_anchor = None
        self._sweep_dir = None
        self._sweep_t = 0.0

    def command(self, dt: float, me, ball, aim, ball_vel=(0.0, 0.0),
                opp_pos=None):
        """Returns (v, omega, phase)."""
        from ai.controller import limits
        v_max, w_max = limits()

        # Lead the ball. This is the whole information advantage: drive at
        # where it is going, not where it is.
        lead = config.DIRECT_LEAD_S
        if lead > 0.0:
            from ai.estimator import predict_ball
            bp, _ = predict_ball(ball, ball_vel, lead)
        else:
            bp = ball

        dx, dy = aim[0] - bp[0], aim[1] - bp[1]
        n = max(math.hypot(dx, dy), 1e-6)
        ux, uy = dx / n, dy / n

        stand = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
                 + config.BALL_RADIUS_M)
        strike = (bp[0] - ux * stand, bp[1] - uy * stand)

        # PROGRESS WATCHDOG. A sweep direction that has not moved the ball in
        # SWEEP_STALL_S is a dead end, whatever the geometry says; ban it for
        # a while so the other way out gets its turn.
        for k in list(self.banned):
            self.banned[k] -= dt
            if self.banned[k] <= 0.0:
                del self.banned[k]
        # Only time actually SWEEPING counts. Counting the drive to the lane
        # banned the right direction before the robot had even arrived, and
        # the sweep flipped ends every 1.5 s without touching the ball.
        if self.pinned and config.SWEEP_WATCHDOG and self._in_lane:
            if self._sweep_anchor is None or self._sweep_dir != self.tangent:
                self._sweep_anchor, self._sweep_dir = ball, self.tangent
                self._sweep_t = 0.0
            self._sweep_t += dt
            if math.hypot(ball[0] - self._sweep_anchor[0],
                          ball[1] - self._sweep_anchor[1]) > 0.08:
                self._sweep_anchor, self._sweep_t = ball, 0.0
            elif self._sweep_t > config.SWEEP_STALL_S:
                self.banned[self.tangent] = config.SWEEP_BAN_S
                self.stalls += 1
                self.pin_hold = 0.0
                self._sweep_anchor = None
        else:
            self._sweep_anchor = None

        # A sweep whose start point we cannot REACH is a dead end too -- the
        # opponent parked on it, most often. Traced against the RC proxy: 30 s
        # of "sweep:approach" toward a lane the other robot was sitting in.
        # Longer limit than a stalled sweep, because getting to the lane
        # legitimately takes a while.
        # One clock per direction, reset only by the ball moving or the
        # direction changing. Separate clocks for "approaching" and
        # "sweeping" were each reset by the other as the robot flickered in
        # and out of the lane, and neither ever ran out.
        if self.pinned and config.SWEEP_WATCHDOG:
            if (self._appr_dir != self.tangent or self._appr_anchor is None
                    or math.hypot(ball[0] - self._appr_anchor[0],
                                  ball[1] - self._appr_anchor[1]) > 0.08):
                self._appr_dir, self._appr_t = self.tangent, 0.0
                self._appr_anchor = ball
            self._appr_t += dt
            if self._appr_t > config.SWEEP_APPROACH_STALL_S:
                self.banned[self.tangent] = config.SWEEP_BAN_S
                self.stalls += 1
                self.pin_hold = 0.0
                self._appr_t = 0.0
                self._appr_anchor = None
        else:
            self._appr_t = 0.0
            self._appr_anchor = None

        # A ball on a wall has no "behind" inside the arena. Sweep it out
        # along the wall instead of driving at a point outside the pitch.
        #
        # COMMIT to that decision. Re-deciding every tick was the vibration:
        # the pinned test sits right on a threshold, so it flickered on and
        # off and the robot alternated between two different control laws
        # forty times a second without ever travelling anywhere.
        if self.pin_hold > 0.0:
            self.pin_hold -= dt
        else:
            self.pinned = False
        if config.USE_WALL_EXTRACTION and not is_reachable(strike):
            alt = wall_extraction(bp, self.attack_dir, me.pos, me.theta,
                                  banned=tuple(self.banned))
            self.close_in = alt is not None
            if alt is None and config.USE_GOALMOUTH_SWEEP:
                # Beside our goal mouth the "wall" is the open goal. Never
                # steer in toward it.
                alt = goalmouth_sweep(bp, self.attack_dir, me.pos)
            if alt is not None:
                aim, strike, tangent = alt
                self.pinned = True
                self.tangent = tangent
                self.pin_hold = config.WALL_SWEEP_COMMIT_S
                dx, dy = aim[0] - bp[0], aim[1] - bp[1]
                n = max(math.hypot(dx, dy), 1e-6)
                ux, uy = dx / n, dy / n
        strike = reachable_point(strike)

        rx, ry = me.pos[0] - bp[0], me.pos[1] - bp[1]
        rn = max(math.hypot(rx, ry), 1e-6)
        behind = (rx * ux + ry * uy) / rn

        if (behind < -config.DIRECT_BEHIND_COS
                and rn < stand * config.DIRECT_RANGE_MULT):
            target = reachable_point(aim)  # lined up: go through the ball
            phase = "sweep" if self.pinned else "strike"
            if self.mode != "strike":
                self.strikes += 1
        else:
            target = strike              # get behind it first
            phase = "lineup"
            if config.USE_ROUTE_AROUND and not self.pinned:
                via = route_around(me.pos, bp, strike)
                if via is not strike:
                    target = via
                    phase = "around"
            if (config.AVOID_OPPONENT_BODY and opp_pos is not None
                    and not _contested(bp, opp_pos)):
                via = avoid_robot(me.pos, target, opp_pos)
                if via is not target:
                    target = via
                    phase = "avoid"
        self.mode = phase

        # A SWEEP STARTS FROM THE SWEEP LANE, NOT FROM WHEREVER WE ARE.
        #
        # The heading-hold below points along the wall and drives. Run from
        # the far side of the ball, that drives AWAY from it: with the
        # referee reset switched off, the dead-ball traces showed the robot
        # "sweeping" from beyond the opponent, 0.8 s commit after 0.8 s
        # commit, while the ball sat untouched for the rest of the match.
        # So only hold the heading once we are upstream of the ball, in its
        # lane along the wall. Until then, drive to the approach point.
        in_lane = True
        self._in_lane = False
        if self.pinned and config.SWEEP_REQUIRE_LANE:
            tx, ty = self.tangent
            along = rx * tx + ry * ty            # < 0: upstream of the ball
            # Offset from the lane the robot's CENTRE runs along, which is the
            # line through the approach point, not through the ball: the
            # centre can never get closer to the wall than its circumradius.
            # Only drifting AWAY from the wall leaves the lane. Closing in
            # toward it is the whole point of the sweep, and counting it as
            # "out of lane" sent the robot back out every time it got there.
            ax_, ay_ = me.pos[0] - strike[0], me.pos[1] - strike[1]
            wx, wy = strike[0] - bp[0], strike[1] - bp[1]
            kk = wx * tx + wy * ty
            wx, wy = wx - kk * tx, wy - kk * ty
            wn = math.hypot(wx, wy)
            if wn > 1e-6:
                across = max(0.0, (ax_ * wx + ay_ * wy) / wn)
            else:
                across = abs(ax_ * -ty + ay_ * tx)
            in_lane = (along < 0.05 and across < config.SWEEP_LANE_M
                       and rn < config.SWEEP_LANE_RANGE_M)
            self.dbg = dict(along=round(along, 3), across=round(across, 3),
                            rn=round(rn, 3), strike=strike, tangent=self.tangent)
            if not in_lane:
                target = strike
                phase = "sweep:approach"
                # Round the ball first. The approach point sits beside the
                # ball, and a straight line to it from the wrong side runs
                # through the ball: two of three goals conceded in one traced
                # match were the robot knocking the ball into its own net on
                # the way to start a goal-mouth sweep.
                if config.USE_ROUTE_AROUND:
                    target = route_around(me.pos, bp, target)
                if config.AVOID_OPPONENT_BODY and opp_pos is not None:
                    target = avoid_robot(me.pos, target, opp_pos)

        self._in_lane = self.pinned and in_lane
        if self.pinned and in_lane:
            # SWEEPING A WALL IS A HEADING PROBLEM, NOT A POSITION ONE.
            #
            # The approach point for a pinned ball lands centimetres from the
            # robot, and the bearing to a point that close swings through tens
            # of degrees per tick as the robot creeps -- so a point-chasing law
            # turns, overshoots, turns back, and vibrates on the spot next to
            # the wall without ever sweeping anything.
            #
            # Driving a ball along a wall does not require arriving anywhere.
            # It requires pointing along the wall and going. So steer to the
            # tangent heading and hold it, which is stable however close the
            # robot is to the ball or the wall.
            tx, ty = self.tangent
            desired = math.atan2(ty, tx)
            if config.SWEEP_CLOSE_IN and self.close_in:
                # CLOSE IN ON THE WALL WHILE RUNNING ALONG IT.
                #
                # The lane is set a full circumradius off the wall so the robot
                # can turn there without its horn tips striking it. But from
                # that lane the front face does not reach a ball lying 0.1 m
                # from the wall: the sweep drove straight past it, which was
                # the dead-ball trace in which the robot ran the lane again
                # and again beside a ball it never touched. Once pointed along
                # the wall, steer in toward it until the ball sits on the face.
                dxs, dys = strike[0] - bp[0], strike[1] - bp[1]
                k = dxs * tx + dys * ty
                nx_, ny_ = dxs - k * tx, dys - k * ty
                nn = math.hypot(nx_, ny_)
                if nn > 1e-6:
                    nx_, ny_ = nx_ / nn, ny_ / nn
                    lat = rx * nx_ + ry * ny_       # centre offset, + = off wall
                    # Where the centre SHOULD run: body edge just clear of
                    # the wall. A fixed ball-relative offset asked for the
                    # centre closer to the wall than half the body width
                    # allows, so the tilt never came off and the robot ground
                    # along the wall on its front corner, clipping the ball
                    # 0.47 m in 10 s.
                    wall_d = wall_gap(bp, (nx_, ny_))
                    want = (config.ROBOT_WIDTH_M / 2.0
                            + config.SWEEP_WALL_CLEAR_M - wall_d)
                    tilt = clamp((lat - want) * config.SWEEP_CLOSE_GAIN,
                                 -math.radians(8.0),
                                 math.radians(config.SWEEP_CLOSE_MAX_DEG))
                    # Never tilt so far that the leading corner reaches the
                    # wall first. At 20 degrees the corner touched with the
                    # centre still 0.17 m out; pinned there the robot could
                    # neither close in nor straighten, and crept along the
                    # wall on its corner at 0.15 m/s. Limiting the tilt by
                    # the clearance left makes it shrink to zero exactly as
                    # the robot arrives alongside.
                    gap_c = wall_gap(me.pos, (nx_, ny_)) - 0.008
                    hl, hw = config.ROBOT_LENGTH_M / 2.0, config.ROBOT_WIDTH_M / 2.0
                    tip = hl + config.HORN_LENGTH_M
                    hy_ = config.HORN_GAP_M / 2.0 + config.HORN_WIDTH_M
                    a_ok = 0.0
                    for deg in range(0, int(config.SWEEP_CLOSE_MAX_DEG) + 1):
                        aa = math.radians(deg)
                        ext = max(hl * math.sin(aa) + hw * math.cos(aa),
                                  tip * math.sin(aa) + hy_ * math.cos(aa))
                        if ext > gap_c:
                            break
                        a_ok = aa
                    tilt = min(tilt, a_ok)
                    desired = math.atan2(ty * math.cos(tilt) - ny_ * math.sin(tilt),
                                         tx * math.cos(tilt) - nx_ * math.sin(tilt))
            # FINISH. Sweeping along THEIR end wall walks the ball across the
            # face of the goal -- and, traced, straight on past it, 8 cm short
            # of the line, because nothing told the robot the wall had become
            # an open net. Once the ball is in front of the mouth, turn into
            # it and drive the ball over the line.
            hx = config.HALF_LENGTH_M
            if (config.SWEEP_FINISH and abs(tx) < 0.5
                    and bp[0] * self.attack_dir > hx - 0.30
                    and abs(bp[1]) < config.HALF_GOAL_M - config.BALL_RADIUS_M):
                gx = self.attack_dir * (hx + 0.20)
                desired = math.atan2(bp[1] - me.pos[1], gx - me.pos[0])
                self.pin_hold = 0.0
            err = wrap_angle(desired - me.theta)
            if abs(err) > math.radians(config.WALL_SWEEP_TOL_DEG
                                       + (config.SWEEP_CLOSE_MAX_DEG
                                          if config.SWEEP_CLOSE_IN else 0.0)):
                self.driving = False
                return 0.0, math.copysign(w_max, err), "sweep:turn"
            self.driving = True
            # Square up BEFORE pushing. Driving on while still angled into the
            # wall pressed the front corner onto it, wall contact bleeds away
            # yaw, and the robot could never straighten: traced at 45 degrees
            # into the wall, creeping the ball along at 0.15 m/s.
            fade = max(0.0, 1.0 - abs(err) / math.radians(
                config.WALL_SWEEP_TOL_DEG + config.SWEEP_CLOSE_MAX_DEG))
            return (v_max * config.WALL_SWEEP_SPEED_FRAC * fade,
                    clamp(err * 4.0, -w_max, w_max), "sweep")

        err = wrap_angle(math.atan2(target[1] - me.pos[1],
                                    target[0] - me.pos[0]) - me.theta)

        # A target closer than the robot is long has an unstable bearing --
        # the same vibration in miniature. Below that, steer by where the
        # target is RELATIVE TO THE RUN rather than by its exact bearing.
        if math.hypot(target[0] - me.pos[0],
                      target[1] - me.pos[1]) < config.DIRECT_MIN_TARGET_M:
            err = clamp(err, -math.radians(config.DIRECT_ANGLE_RESUME_DEG),
                        math.radians(config.DIRECT_ANGLE_RESUME_DEG))

        tol = math.radians(config.DIRECT_ANGLE_RESUME_DEG if self.driving
                           else config.DIRECT_ANGLE_TOL_DEG)
        if abs(err) > tol:
            # TURN. On the spot, at full rate, with no forward component --
            # an arc here is what carries the robot past the strike point.
            self.driving = False
            return 0.0, math.copysign(w_max, err), phase + ":turn"
        # DRIVE. Dead straight, full speed, and do not stop for anything.
        self.driving = True
        return v_max, 0.0, phase

    def reset(self) -> None:
        self.driving = False
        self.mode = "lineup"
        self.pinned = False
        self.pin_hold = 0.0
        self.banned = {}
        self._sweep_anchor = None


class StrikeSequence:
    """Attack the way the thing that actually scores attacks.

    A one-trick opponent that only ever (a) turns on the spot until aimed and
    (b) drives dead straight, scores 16 goals in 10 matches against the full
    planner, which scores 0. It wins because it COMMITS: it never
    re-deliberates in the middle of a run, so it reaches full speed and
    arrives with the ball going where it was pointed.

    The planner, by contrast, had seven reflexes and twenty-four candidates
    all competing every 40 ms, and the result was a robot that hovered near
    the ball rearranging itself. Sophistication was not beating simplicity;
    it was preventing it.

    So attacking is now this sequence, and the clever machinery is used for
    what it is actually good at -- choosing the aim point (reachability),
    deciding WHEN to defend instead, and estimating everything -- rather than
    for re-litigating the approach thirty times a second.

        LINE UP   turn and drive to the strike point behind the ball
        AIM       turn on the spot until pointed at the aim
        STRIKE    drive straight through the ball, committed
    """

    def __init__(self) -> None:
        self.phase = "lineup"
        self.hold = 0.0
        self.strikes = 0

    def _strike_point(self, ball, aim):
        dx, dy = aim[0] - ball[0], aim[1] - ball[1]
        n = max(math.hypot(dx, dy), 1e-6)
        ux, uy = dx / n, dy / n
        stand = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
                 + config.BALL_RADIUS_M)
        return (ball[0] - ux * stand, ball[1] - uy * stand), (ux, uy)

    def command(self, dt: float, me, ball, aim):
        """Returns (v, omega, phase)."""
        from ai.controller import limits, drive_to
        v_max, w_max = limits()
        strike, (ux, uy) = self._strike_point(ball, aim)

        # Are we behind the ball, lined up with the aim?
        rx, ry = me.pos[0] - ball[0], me.pos[1] - ball[1]
        rn = max(math.hypot(rx, ry), 1e-6)
        behind = (rx * ux + ry * uy) / rn
        stand = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M * 0.5
                 + config.BALL_RADIUS_M)

        if self.hold > 0.0:
            self.hold -= dt

        # Once striking, stay striking until the run is spent -- this is the
        # commitment that makes it work.
        if self.phase == "strike" and self.hold > 0.0:
            err = wrap_angle(math.atan2(aim[1] - me.pos[1],
                                        aim[0] - me.pos[0]) - me.theta)
            return (v_max, clamp(err * 1.2, -w_max * 0.25, w_max * 0.25),
                    "strike")

        if (behind < -config.STRIKE_BEHIND_COS
                and rn < stand * config.STRIKE_RANGE_MULT):
            err = wrap_angle(math.atan2(aim[1] - me.pos[1],
                                        aim[0] - me.pos[0]) - me.theta)
            if abs(err) < math.radians(config.STRIKE_AIM_DEG):
                self.phase = "strike"
                self.hold = config.STRIKE_COMMIT_S
                self.strikes += 1
                return v_max, 0.0, "strike"
            # Aimed wrong: turn on the spot, do not shove the ball meanwhile.
            self.phase = "aim"
            return 0.0, clamp(err * 3.0, -w_max, w_max), "aim"

        self.phase = "lineup"
        v, w = drive_to(me.pos, me.theta, strike,
                        allow_reverse=config.STRIKE_LINEUP_REVERSE,
                        through=True)
        return v, w, "lineup"

    def reset(self) -> None:
        self.phase = "lineup"
        self.hold = 0.0
