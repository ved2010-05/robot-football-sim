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
        self._trail: list[tuple[float, float, float]] = []

    def update(self, dt: float, pos, theta: float, speed: float,
               commanded_v: float) -> bool:
        """Returns True while an escape is in progress.

        Detection uses ACTUAL DISPLACEMENT, not the believed velocity.
        That distinction matters: a robot wedged against a wall still has
        spinning wheels, its encoders faithfully report that rotation, and
        the estimator -- which fuses encoders for velocity -- therefore
        believes it is moving at full speed. Velocity is precisely the signal
        that lies in this situation. Position comes from vision and cannot.
        """
        self._trail.append((pos[0], pos[1], dt))
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
        if window >= config.STUCK_DETECT_S * 0.8 and len(self._trail) > 2:
            x0, y0, _ = self._trail[0]
            moved = math.hypot(pos[0] - x0, pos[1] - y0)

        trying = abs(commanded_v) > 0.25
        stalled = (window >= config.STUCK_DETECT_S * 0.8
                   and moved < config.STUCK_MIN_TRAVEL_M)
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
            return True
        return False

    def command(self) -> tuple[float, float]:
        from ai.controller import limits
        v_max, w_max = limits()
        return (-0.75 * v_max, self.escape_turn * 0.8 * w_max)

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

    def update(self, dt: float, me, opp, ball_pos) -> bool:
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
            return True
        return False

    def command(self) -> tuple[float, float]:
        from ai.controller import limits
        v_max, w_max = limits()
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
    """Pull a point inside the rectangle the robot's CENTRE can occupy."""
    margin = robot_circumradius() + 0.02
    hx = max(config.HALF_LENGTH_M - margin, 0.05)
    hy = max(config.HALF_WIDTH_M - margin, 0.05)
    return (clamp(p[0], -hx, hx), clamp(p[1], -hy, hy))


def is_reachable(p) -> bool:
    margin = robot_circumradius() + 0.02
    return (abs(p[0]) <= config.HALF_LENGTH_M - margin
            and abs(p[1]) <= config.HALF_WIDTH_M - margin)


def wall_extraction(ball, attack_dir: float, me_pos=None,
                    me_theta: float = 0.0):
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
    nx, ny = ball_against_wall(ball)
    if nx == 0.0 and ny == 0.0:
        return None

    attacking_end = (ball[0] * attack_dir) > 0.0

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

    def build(tx, ty):
        approach = reachable_point((ball[0] - tx * stand + off_x,
                                    ball[1] - ty * stand + off_y))
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
        self.attack_dir = attack_dir

    def command(self, dt: float, me, ball, aim, ball_vel=(0.0, 0.0)):
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
            alt = wall_extraction(bp, self.attack_dir, me.pos, me.theta)
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
        self.mode = phase

        if self.pinned:
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
            desired = math.atan2(self.tangent[1], self.tangent[0])
            err = wrap_angle(desired - me.theta)
            if abs(err) > math.radians(config.WALL_SWEEP_TOL_DEG):
                self.driving = False
                return 0.0, math.copysign(w_max, err), "sweep:turn"
            self.driving = True
            return (v_max * config.WALL_SWEEP_SPEED_FRAC,
                    clamp(err * 2.0, -w_max * 0.35, w_max * 0.35), "sweep")

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
