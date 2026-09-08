"""
The physical world: arena, ball, two robots, and the collisions between them.

This module is GROUND TRUTH. The AI never sees any of it directly -- it only
ever receives what comes through the sensor layer, which delays, quantises
and corrupts everything here. Keeping that boundary strict is the whole point
of the exercise: if the AI could peek at `world`, none of the estimation work
would be exercised, and none of it would transfer to hardware.

Geometry:

    +y  ^                 ARENA_WIDTH
        |   +---------------------------------+
        |   |                                 |
      [GOAL]|              (0,0)              |[GOAL]
   robot 1  |                                 |  robot 0
   attacks  |                                 |  attacks
        |   +---------------------------------+
        +-----------------------------------------> +x   ARENA_LENGTH

Robot 0 defends the -x goal and attacks +x. Robot 1 is the mirror.
"""

from __future__ import annotations

import math
import random

import config
import motors
from sim import chassis
from sim.geometry import (
    Vec, add, sub, scale, dot, length, normalize, rotate, to_world,
    clamp, wrap_angle, circle_vs_rect, circle_vs_capsule, rect_vs_rect,
    segment_crosses_x, dist,
)

GRAVITY = 9.81

# How many times to relax the ball against the robots and walls per tick.
# Four is comfortably enough to resolve a ball pinched between both robots;
# the loop exits early when nothing moves, so the usual cost is one pass.
BALL_RELAXATION_PASSES = 4


class Ball:
    def __init__(self) -> None:
        self.pos: Vec = (0.0, 0.0)
        self.vel: Vec = (0.0, 0.0)
        self.prev_pos: Vec = (0.0, 0.0)
        self.radius = config.BALL_RADIUS_M
        self.mass = config.BALL_MASS_KG

    def reset(self, pos: Vec = (0.0, 0.0)) -> None:
        self.pos = pos
        self.prev_pos = pos
        self.vel = (0.0, 0.0)

    def step(self, dt: float) -> None:
        self.prev_pos = self.pos
        speed = length(self.vel)
        if speed > config.BALL_MIN_SPEED:
            # Rolling resistance is very nearly a constant deceleration for a
            # ball on a flat surface, not a velocity-proportional drag.
            decel = config.BALL_ROLL_DECEL * dt
            new_speed = max(0.0, speed - decel)
            s = new_speed / speed
            self.vel = (self.vel[0] * s, self.vel[1] * s)
        else:
            self.vel = (0.0, 0.0)
        self.pos = (self.pos[0] + self.vel[0] * dt,
                    self.pos[1] + self.vel[1] * dt)


class Robot:
    """Chassis plus the physical shape that touches the ball."""

    def __init__(self, index: int, motor: motors.Motor,
                 x: float, y: float, theta: float) -> None:
        self.index = index
        self.chassis = chassis.Chassis(motor, x, y, theta)

        self.half_len = config.ROBOT_LENGTH_M / 2.0
        self.half_wid = config.ROBOT_WIDTH_M / 2.0

        # Attacking direction: robot 0 attacks +x, robot 1 attacks -x.
        self.attack_dir = 1.0 if index == 0 else -1.0

    # -- pose passthrough --------------------------------------------------

    @property
    def pos(self) -> Vec:
        return self.chassis.pos

    @property
    def theta(self) -> float:
        return self.chassis.theta

    @property
    def vel(self) -> Vec:
        return self.chassis.vel

    @property
    def omega(self) -> float:
        return self.chassis.omega

    # -- shape -------------------------------------------------------------

    def horn_segments(self) -> list[tuple[Vec, Vec]]:
        """The two bars, in world coordinates, as capsule spines.

        Each horn runs forward from the front face. The clear span between
        their inner faces is HORN_GAP_M, so the spine offset is half the gap
        plus half the bar thickness.
        """
        y_off = config.HORN_GAP_M / 2.0 + config.HORN_WIDTH_M / 2.0
        x0 = self.half_len
        x1 = self.half_len + config.HORN_LENGTH_M
        out = []
        for sy in (+1.0, -1.0):
            a = to_world((x0, sy * y_off), self.pos, self.theta)
            b = to_world((x1, sy * y_off), self.pos, self.theta)
            out.append((a, b))
        return out

    def pocket_centre(self) -> Vec:
        """Middle of the capture pocket, in world coordinates.

        This is where the AI wants the ball to be, and where it aims when
        approaching. With no kicker, possession means the ball sitting here.
        """
        x = self.half_len + config.HORN_LENGTH_M * 0.5
        return to_world((x, 0.0), self.pos, self.theta)

    def has_ball(self, ball: Ball) -> bool:
        """True when the ball is captured between the horns."""
        from sim.geometry import to_local
        lx, ly = to_local(ball.pos, self.pos, self.theta)
        in_x = self.half_len - 0.01 <= lx <= self.half_len + config.HORN_LENGTH_M
        in_y = abs(ly) <= config.HORN_GAP_M / 2.0 + ball.radius * 0.5
        return in_x and in_y

    def forward(self) -> Vec:
        return (math.cos(self.theta), math.sin(self.theta))

    def hull_local(self) -> list[Vec]:
        """Body corners plus horn tips, in the robot frame.

        The horns are part of the silhouette that hits a wall, and they only
        stick out forwards, so the hull is not symmetric about the centre.
        """
        hl, hw = self.half_len, self.half_wid
        y_off = config.HORN_GAP_M / 2.0 + config.HORN_WIDTH_M / 2.0
        tip = hl + config.HORN_LENGTH_M
        r = config.HORN_WIDTH_M / 2.0
        return [
            (hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw),
            (tip + r, y_off + r), (tip + r, -y_off - r),
            (tip + r, y_off - r), (tip + r, -y_off + r),
        ]

    def extent_bounds(self) -> tuple[float, float, float, float]:
        """(min_x, max_x, min_y, max_y) of the hull relative to the centre,
        in WORLD axes, for the current heading.

        Signed and asymmetric on purpose -- see the note in
        World._resolve_robot_wall about why a bounding circle was wrong.
        """
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        xs = []
        ys = []
        for lx, ly in self.hull_local():
            xs.append(lx * c - ly * s)
            ys.append(lx * s + ly * c)
        return min(xs), max(xs), min(ys), max(ys)


class World:
    """Ground truth simulation."""

    def __init__(self, seed: int | None = None) -> None:
        self.motor = motors.get(config.MOTOR)
        self.rng = random.Random(
            config.RANDOM_SEED if seed is None else seed
        )

        off = config.KICKOFF_ROBOT_OFFSET_M
        self.robots = [
            Robot(0, self.motor, -off, 0.0, 0.0),          # AI, attacks +x
            Robot(1, self.motor, +off, 0.0, math.pi),      # human, attacks -x
        ]
        self.ball = Ball()
        self.ball.reset(config.KICKOFF_BALL_POS)

        self.t = 0.0
        self.score = [0, 0]
        self.last_goal_by: int | None = None
        self.goal_event = False       # set for one step when a goal is scored

    # -- collision helpers -------------------------------------------------

    def _resolve_ball_wall(self) -> None:
        b = self.ball
        r = b.radius
        hx = config.HALF_LENGTH_M
        hy = config.HALF_WIDTH_M
        e = config.WALL_RESTITUTION
        f = config.WALL_FRICTION

        x, y = b.pos
        vx, vy = b.vel

        # Side walls (always solid).
        if y + r > hy:
            y = hy - r
            if vy > 0:
                vy = -vy * e
                vx *= (1.0 - f)
        elif y - r < -hy:
            y = -hy + r
            if vy < 0:
                vy = -vy * e
                vx *= (1.0 - f)

        # End walls, except across the goal mouth where there is an opening.
        in_mouth = abs(y) <= config.HALF_GOAL_M
        if not in_mouth:
            if x + r > hx:
                x = hx - r
                if vx > 0:
                    vx = -vx * e
                    vy *= (1.0 - f)
            elif x - r < -hx:
                x = -hx + r
                if vx < 0:
                    vx = -vx * e
                    vy *= (1.0 - f)
        else:
            # Inside the recess behind the goal line, stop the ball at the
            # back of the net so it does not escape the world.
            back = hx + config.GOAL_DEPTH_M
            if x + r > back:
                x = back - r
                vx = -vx * e
            elif x - r < -back:
                x = -back + r
                vx = -vx * e

        b.pos = (x, y)
        b.vel = (vx, vy)

    def _resolve_ball_robot(self, robot: Robot) -> bool:
        """Ball against the robot body and both horns.

        Uses a proper impulse including the robot's rotational inertia, so a
        spinning robot flicks the ball rather than just shoving it. The ball
        is treated as a point mass without spin -- ball spin would matter for
        a real dribbler, but there is no dribbler here.

        Returns True if it moved the ball, so the caller knows whether
        another relaxation pass is worth running. Safe to call repeatedly:
        after the first impulse the contact is separating, so later passes
        correct position only and cannot pump energy in.
        """
        b = self.ball
        if not self._ball_near(robot):
            return False
        contacts = []

        hit = circle_vs_rect(b.pos, b.radius, robot.pos, robot.theta,
                             robot.half_len, robot.half_wid)
        if hit is not None:
            contacts.append(hit)

        cap_r = config.HORN_WIDTH_M / 2.0
        for a, bb in robot.horn_segments():
            hit = circle_vs_capsule(b.pos, b.radius, a, bb, cap_r)
            if hit is not None:
                contacts.append(hit)

        if not contacts:
            return False

        for c in contacts:
            n = c.normal          # points from robot surface toward the ball
            # Positional correction first, so repeated contacts do not stack
            # penetration into a launch.
            b.pos = (b.pos[0] + n[0] * c.depth, b.pos[1] + n[1] * c.depth)

            # Contact point relative to the robot centre of mass.
            rx = c.point[0] - robot.pos[0]
            ry = c.point[1] - robot.pos[1]

            # Velocity of the robot's surface at the contact point.
            rv = (robot.vel[0] - robot.omega * ry,
                  robot.vel[1] + robot.omega * rx)

            rel = (b.vel[0] - rv[0], b.vel[1] - rv[1])
            vn = dot(rel, n)
            if vn > 0.0:
                continue  # already separating

            inv_mb = 1.0 / b.mass
            inv_mr = 1.0 / robot.chassis.mass
            rn = rx * n[1] - ry * n[0]
            inv_ir = (rn * rn) / robot.chassis.inertia

            # Resting contact vs. genuine impact. An impulse solver cannot
            # tell them apart, so below a threshold approach speed the
            # collision is made fully inelastic and the ball settles against
            # the face instead of re-bouncing every timestep.
            e = (config.BALL_RESTITUTION
                 if -vn > config.BALL_RESTITUTION_THRESHOLD else 0.0)
            j = -(1.0 + e) * vn / (inv_mb + inv_mr + inv_ir)

            b.vel = (b.vel[0] + n[0] * j * inv_mb,
                     b.vel[1] + n[1] * j * inv_mb)
            robot.chassis.apply_impulse((-n[0] * j, -n[1] * j), c.point)

        return True

    def _ball_near(self, robot: Robot) -> bool:
        """Cheap broad-phase: can the ball possibly touch this robot?

        One squared-distance compare, to skip the six exact tests (body rect
        plus two horn capsules, run up to four relaxation passes) that would
        otherwise execute every tick.

        This matters far more with a large robot and a large ball: at 200 mm
        robots and a 100 mm ball the ball is in contact ~94% of the time, so
        the exact tests ran constantly and physics cost 42 ms per rendered
        frame. With a small ball they almost never ran, which is why this was
        not needed before.
        """
        r = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M
             + self.ball.radius + 0.01)
        dx = self.ball.pos[0] - robot.pos[0]
        dy = self.ball.pos[1] - robot.pos[1]
        return dx * dx + dy * dy < r * r

    def _all_ball_contacts(self):
        """Every current ball contact against either robot."""
        b = self.ball
        cap_r = config.HORN_WIDTH_M / 2.0
        out = []
        for robot in self.robots:
            if not self._ball_near(robot):
                continue
            hit = circle_vs_rect(b.pos, b.radius, robot.pos, robot.theta,
                                 robot.half_len, robot.half_wid)
            if hit is not None:
                out.append((robot, hit))
            for a, bb in robot.horn_segments():
                hit = circle_vs_capsule(b.pos, b.radius, a, bb, cap_r)
                if hit is not None:
                    out.append((robot, hit))
        return out

    def _relieve_pinch(self) -> None:
        """Let a ball squeezed between two robots escape sideways.

        When both robots press the ball from opposite sides, the contact
        constraints are mutually unsatisfiable: 2.2 kg of robot on each side
        against 46 g of ball, with a gap narrower than the ball. Position
        correction alone just shuffles it from one body into the other, and
        it ends up buried -- which also makes it invisible to the overhead
        camera, so the AI loses the ball exactly when the game is most
        contested.

        A real ball does not stay there. It shoots out of the gap. So when
        opposing normals are detected, the ball is ejected along the axis
        PERPENDICULAR to the squeeze, toward whichever side has more room,
        and given the velocity that escape implies.
        """
        # A pinch needs two bodies. If the ball is not close to both robots
        # there is nothing to relieve, and the contact scan can be skipped.
        if not all(self._ball_near(r) for r in self.robots):
            return
        contacts = self._all_ball_contacts()
        if len(contacts) < 2:
            return

        # Look for two contacts whose normals substantially oppose.
        worst_dot = 0.0
        pair = None
        for i in range(len(contacts)):
            for j in range(i + 1, len(contacts)):
                d = dot(contacts[i][1].normal, contacts[j][1].normal)
                if d < worst_dot:
                    worst_dot = d
                    pair = (contacts[i], contacts[j])
        if pair is None or worst_dot > -0.35:
            return

        (_, ca), (_, cb) = pair
        # Squeeze axis, and the two ways out perpendicular to it.
        axis = normalize(sub(ca.normal, cb.normal))
        if axis == (0.0, 0.0):
            return
        escape = (-axis[1], axis[0])

        # Prefer the direction the ball is already drifting; failing that,
        # the one with more open pitch.
        if dot(self.ball.vel, escape) < 0:
            escape = (-escape[0], -escape[1])
        elif abs(dot(self.ball.vel, escape)) < 1e-3:
            probe = 0.12
            here = (self.ball.pos[0] + escape[0] * probe,
                    self.ball.pos[1] + escape[1] * probe)
            if abs(here[1]) > config.HALF_WIDTH_M - self.ball.radius:
                escape = (-escape[0], -escape[1])

        clear = max(ca.depth, cb.depth) + self.ball.radius * 0.5
        self.ball.pos = (self.ball.pos[0] + escape[0] * clear,
                         self.ball.pos[1] + escape[1] * clear)

        # Squirt speed, from how hard the two bodies are converging.
        squeeze = abs(dot(sub(pair[0][0].vel, pair[1][0].vel), axis))
        speed = max(0.25, min(squeeze, 1.5))
        self.ball.vel = (self.ball.vel[0] + escape[0] * speed,
                         self.ball.vel[1] + escape[1] * speed)

    def _resolve_robot_robot(self) -> None:
        a, bb = self.robots
        hit = rect_vs_rect(a.pos, a.theta, a.half_len, a.half_wid,
                           bb.pos, bb.theta, bb.half_len, bb.half_wid)
        if hit is None:
            return

        n = hit.normal
        # Split the positional correction evenly -- equal masses.
        push = hit.depth * 0.5
        a.chassis.pos = (a.chassis.pos[0] - n[0] * push,
                         a.chassis.pos[1] - n[1] * push)
        bb.chassis.pos = (bb.chassis.pos[0] + n[0] * push,
                          bb.chassis.pos[1] + n[1] * push)

        rel = (bb.vel[0] - a.vel[0], bb.vel[1] - a.vel[1])
        vn = dot(rel, n)
        if vn > 0.0:
            return

        inv_m = 1.0 / a.chassis.mass + 1.0 / bb.chassis.mass
        e = config.ROBOT_RESTITUTION
        j = -(1.0 + e) * vn / inv_m
        a.chassis.apply_impulse((-n[0] * j, -n[1] * j), hit.point)
        bb.chassis.apply_impulse((n[0] * j, n[1] * j), hit.point)

    def _resolve_robot_wall(self, robot: Robot) -> None:
        """Keep robots inside the pitch.

        Robots collide with the full end wall, INCLUDING across the goal
        mouth. Letting a robot drive into the recess only ever produces a
        stuck robot and a dead match; real arenas prevent it with a lip or a
        mouth too narrow to enter.
        """
        # Use the robot's TRUE oriented extent, not a bounding circle.
        #
        # A bounding circle is the radius of the most awkward possible
        # orientation applied to every orientation. Here that meant holding
        # the robot 197 mm off every wall when side-on it only needs 100 mm,
        # leaving a ~97 mm dead band along every wall that the robot could
        # physically never enter. A ball hugging a wall was therefore
        # impossible to capture -- only nudgeable -- and in a corner the two
        # dead bands overlapped into a region where the ball simply died.
        # That is the corner-stuck bug.
        #
        # The shape is also asymmetric: the horns protrude forwards only, so
        # the forward and rearward extents genuinely differ and a single
        # radius cannot express it. Project the actual hull instead.
        lo_x, hi_x, lo_y, hi_y = robot.extent_bounds()

        x, y = robot.pos
        vx, vy = robot.vel
        hx = config.HALF_LENGTH_M
        hy = config.HALF_WIDTH_M
        changed = False

        if x + hi_x > hx:
            x = hx - hi_x
            vx = min(vx, 0.0)
            changed = True
        elif x + lo_x < -hx:
            x = -hx - lo_x
            vx = max(vx, 0.0)
            changed = True
        if y + hi_y > hy:
            y = hy - hi_y
            vy = min(vy, 0.0)
            changed = True
        elif y + lo_y < -hy:
            y = -hy - lo_y
            vy = max(vy, 0.0)
            changed = True

        if changed:
            robot.chassis.pos = (x, y)
            robot.chassis.vel = (vx, vy)
            robot.chassis.omega *= 0.8   # scrubbing along a wall bleeds spin

    def _check_goal(self) -> int | None:
        """Return the index of the robot that scored, or None.

        Tests the ball's SWEPT path, not its instantaneous position, so a
        shot fast enough to clear the goal mouth within one timestep still
        counts. At 1 kHz that needs an implausible speed, but the same code
        runs in the MPC rollout at 20 ms, where it is entirely possible.
        """
        b = self.ball
        margin = b.radius if config.GOAL_REQUIRES_FULL_CROSS else 0.0
        hy = config.HALF_GOAL_M

        # Ball crossing +x line => robot 0 scored (it attacks +x).
        hit = segment_crosses_x(b.prev_pos, b.pos,
                                config.HALF_LENGTH_M + margin, -hy, hy)
        if hit is not None:
            return 0
        hit = segment_crosses_x(b.prev_pos, b.pos,
                                -(config.HALF_LENGTH_M + margin), -hy, hy)
        if hit is not None:
            return 1
        return None

    # -- main step ---------------------------------------------------------

    def step(self, dt: float) -> None:
        self.goal_event = False

        for r in self.robots:
            r.chassis.step(dt, self.t)
        self.ball.step(dt)

        # Settle the heavy bodies FIRST. Both of these move robots, and a
        # robot moved after the ball has been resolved can be left sitting
        # on top of it.
        self._resolve_robot_robot()
        for r in self.robots:
            self._resolve_robot_wall(r)

        # A pinched ball has to be let out before anything else, or the
        # relaxation below will spend its passes shuffling it between two
        # contacts it cannot simultaneously satisfy.
        self._relieve_pinch()

        # Then relax the ball against everything, repeatedly.
        #
        # One pass is not enough. When the ball is trapped between the two
        # robots -- which happens constantly, since both are chasing it --
        # pushing it out of one shoves it into the other, and a single
        # sequential pass leaves it embedded. Iterating lets the squeeze
        # converge, and is what any contact solver does for the same reason.
        for _ in range(BALL_RELAXATION_PASSES):
            moved = False
            for r in self.robots:
                if self._resolve_ball_robot(r):
                    moved = True
            self._resolve_ball_wall()
            if not moved:
                break

        scorer = self._check_goal()
        if scorer is not None:
            self.score[scorer] += 1
            self.last_goal_by = scorer
            self.goal_event = True

        self.t += dt

    # -- resets ------------------------------------------------------------

    def kickoff(self, favour: int | None = None) -> None:
        """Reset to the kickoff arrangement.

        `favour` gives that robot the slight advantage of starting nearer the
        ball, as a restart after conceding. None places both symmetrically.
        """
        off = config.KICKOFF_ROBOT_OFFSET_M
        self.robots[0].chassis.reset(-off, 0.0, 0.0)
        self.robots[1].chassis.reset(+off, 0.0, math.pi)
        if favour == 0:
            self.robots[0].chassis.reset(-off * 0.6, 0.0, 0.0)
        elif favour == 1:
            self.robots[1].chassis.reset(+off * 0.6, 0.0, math.pi)
        self.ball.reset(config.KICKOFF_BALL_POS)

    def reset_ball_to_centre(self) -> None:
        self.ball.reset(config.KICKOFF_BALL_POS)

    # -- queries -----------------------------------------------------------

    def goal_centre(self, attacking_index: int) -> Vec:
        """Centre of the goal the given robot is attacking."""
        d = self.robots[attacking_index].attack_dir
        return (d * config.HALF_LENGTH_M, 0.0)

    def goal_posts(self, attacking_index: int) -> tuple[Vec, Vec]:
        d = self.robots[attacking_index].attack_dir
        x = d * config.HALF_LENGTH_M
        return ((x, -config.HALF_GOAL_M), (x, config.HALF_GOAL_M))

    def possession(self) -> int | None:
        for r in self.robots:
            if r.has_ball(self.ball):
                return r.index
        return None
