"""
Central configuration for the 2D robot football simulator.

THIS IS THE FILE YOU EDIT. Everything tunable lives here.

Units are SI throughout, without exception:
    lengths   metres
    mass      kilograms
    time      seconds
    angles    radians
    force     newtons
    torque    newton-metres
    voltage   volts
    current   amperes

Only the renderer converts to pixels. Only the motor catalogue talks RPM/kg-cm,
and it converts on the way in.

Nothing here is read at import time by the simulation. The world reads config
when it is constructed, so tools can mutate these module attributes before
building a match (see tools/headless.py) without needing a config object.
"""

import math

# =====================================================================
#  SIMULATION CORE
# =====================================================================

PHYSICS_HZ = 1000.0          # inner integration rate. Do not lower much;
                             # the wheel slip model needs a small timestep.
PHYSICS_DT = 1.0 / PHYSICS_HZ

CONTROL_HZ = 100.0           # rate the AI runs at (its "strategy loop")
RENDER_HZ = 60.0             # display refresh

RANDOM_SEED = 12345          # fixed seed => byte-identical replays


# =====================================================================
#  ARENA
#
#  Long axis is X (goal to goal). Short axis is Y.
#  Origin (0,0) is the centre of the pitch.
#  So the pitch spans x in [-L/2, +L/2], y in [-W/2, +W/2].
#  Goals are always centred on the two short sides.
# =====================================================================

ARENA_LENGTH_M = 3.600       # X extent, goal line to goal line
ARENA_WIDTH_M = 2.200        # Y extent, wall to wall

GOAL_WIDTH_M = 0.550         # A 250 mm robot parked in a 400 mm goal covers
                             # 62% of the mouth, which makes scoring nearly
                             # impossible and defending trivial. Measured vs
                             # the one-trick SimpleBot, 10 matches each:
                             #   400 mm (62% covered)  +0.40, AI 4-0
                             #   550 mm (45% covered)  +0.60, AI 6-0  <-
                             #   700 mm (36% covered)  +0.50, AI 6-1
                             #   900 mm (28% covered)  +0.60, AI 6-0
                             # Scale this with the robot, not with the pitch:
                             # what matters is goal width vs ROBOT width.         # opening size, centred on the short side
GOAL_DEPTH_M = 0.300         # recess depth behind the goal line

WALL_RESTITUTION = 0.45      # ball bounciness off walls (0 = dead, 1 = perfect)
WALL_FRICTION = 0.3         # tangential speed scrubbed off on wall contact


# =====================================================================
#  BALL
# =====================================================================

BALL_DIAMETER_M = 0.05      # 43 mm golf ball
BALL_MASS_KG = 0.060          # was 0.100, which is 1.53 g/cm3 at 50 mm --
                              # denser than a golf ball (1.15) and denser than
                              # any ball you would actually buy. 60 g is about
                              # right for a 50 mm practice/plastic ball.

BALL_ROLL_DECEL = 0.50       # m/s^2 constant rolling deceleration.
                             # Smooth board ~0.15, short carpet ~0.6.
BALL_RESTITUTION = 0.20      # bounciness off robot bodies. Low, because a
                             # bouncy front face punts the ball away instead
                             # of carrying it -- with no kicker, the robot
                             # needs the ball to stay put against the face.
                             # Raise it and the game becomes all nudging.

BALL_RESTITUTION_THRESHOLD = 0.15
                             # Below this relative approach speed (m/s) a
                             # contact is treated as fully inelastic.
                             # Without it, a ball resting against a pushing
                             # robot re-bounces every timestep and buzzes
                             # away instead of being carried. Every physics
                             # engine does this; it is not a fudge, it is the
                             # difference between a collision and a resting
                             # contact, which impulse methods cannot
                             # distinguish on their own.
BALL_MIN_SPEED = 0.005       # below this the ball is snapped to rest


# =====================================================================
#  ROBOT BODY
#
#  Body is an axis-aligned rectangle in the robot's own frame, centred on
#  the robot origin. +X local is "forward" (the direction the horns point).
#
#      local +Y
#         ^
#         |   +-----+ <-- horn (left)
#     +---------+   |
#     |         |   |
#     |  BODY   |   |        ---> local +X  (forward)
#     |         |   |
#     +---------+   |
#         |   +-----+ <-- horn (right)
#
#  The two horns protrude forward from the front face, forming an
#  open-topped pocket. If HORN_GAP_M < BALL_DIAMETER_M the pocket traps
#  the ball and the robot can carry it -- with no kicker, this single
#  number largely decides whether the game is about carrying or nudging.
# =====================================================================

ROBOT_LENGTH_M = 0.300       # X extent of the body, EXCLUDING horns       # X extent of the body (front-back)
ROBOT_WIDTH_M = 0.250        # Y extent of the body        # Y extent of the body (left-right)
ROBOT_MASS_KG = 7.000        # A 300 x 250 x ~150 mm machine at 4.5 kg is
                             # 0.40 g/cm3 -- a normal mostly-air robot:
                             # 4x 37D gearmotor 800 g, 4x 100 mm wheels 240 g,
                             # chassis/plate/standoffs 1.6 kg, 3S 5000 mAh
                             # LiPo 400 g, electronics 250 g, horns +
                             # fixings 400 g, contingency 800 g.         # was 4.200, which is 1.98 g/cm3 for this
                              # footprint -- denser than nylon and approaching
                              # solid aluminium. Itemised build: 4x 37D
                              # gearmotor 800 g, 4x 100 mm wheels 240 g,
                              # chassis 300 g, 3S LiPo 180 g, ESP32 + drivers
                              # 100 g, fixings 120 g = ~1.74 kg.
ROBOT_INERTIA_SCALE = 1.0    # multiplier on the computed rectangular inertia,
                             # to account for mass not being evenly spread

HORN_LENGTH_M = 0.090        # how far the bars stick out past the front face.
                             #
                             # Left at your value. An earlier sweep appeared to
                             # show a sharp optimum at 75 mm, but that sweep
                             # reloaded sim.world between trials while match.py
                             # held an old binding, so the trials were not
                             # isolated. Re-run with a fresh process per value
                             # the ordering inverted, and across 50-90 mm the
                             # spread is about two standard errors -- i.e. no
                             # reliable effect at this sample size.
                             #
                             # It still must exceed the ball RADIUS to cradle
                             # at all. Beyond that, measure before believing.
HORN_WIDTH_M = 0.010         # bar thickness
HORN_GAP_M = 0.1           # INNER clear span between the two bars.
                             #
                             # The gap must EXCEED BALL_DIAMETER_M or the ball
                             # never enters the pocket at all -- it just gets
                             # nudged by the horn tips. Retention depends on
                             # the CLEARANCE above that being small:
                             #
                             #   gap slightly > ball    snug pocket, carries well
                             #   gap much   > ball      ball rattles and escapes
                             #   gap        < ball      no pocket at all
                             #
                             # Default is 55 mm against a 43 mm ball: 12 mm of
                             # total play. Widen it and carrying degrades fast.

ROBOT_RESTITUTION = 0.10     # robot-robot collision bounciness


# =====================================================================
#  DRIVETRAIN  (skid-steer, 4 driven wheels, 2 DOF)
#
#  Wheel layout in the robot frame:
#      FL (+x, +y)      FR (+x, -y)
#      RL (-x, +y)      RR (-x, -y)
#
#  Left pair receives one command, right pair the other. All four have
#  their own encoder, and they will read differently from each other
#  because of scrub -- that difference is real signal.
# =====================================================================

WHEEL_RADIUS_M = 0.050
TRACK_WIDTH_M = 0.200        # wheels INSIDE the 250 mm body, 25 mm clear each side        # geometric left-right wheel separation
WHEELBASE_M = 0.200          # front-rear wheel separation          # front-rear wheel separation
WHEEL_INERTIA_KGM2 = 4.0e-5  # per wheel, about its axle

COG_HEIGHT_M = 0.070         # ~150 mm tall robot, mass low in the chassis         # centre of gravity height. Drives longitudinal
                             # weight transfer under acceleration, which on a
                             # short-wheelbase robot is NOT negligible: at
                             # 8 m/s^2 the rear wheels gain most of a wheel's
                             # worth of load and the fronts lose it.

# Skid-steer reality: because the wheels must scrub sideways to turn, the
# robot behaves as though its track were wider than it geometrically is.
# Measured values for real skid-steer platforms are typically 1.2 - 1.7.
# The per-wheel force model below produces this effect on its own; this
# factor is what the AI's *kinematic* model uses, and is deliberately
# allowed to be imperfect so the estimator has something to correct.
SCRUB_FACTOR = 1.35

MU_LONGITUDINAL = 0.85       # tyre-surface friction, rolling direction
MU_LATERAL = 0.65            # tyre-surface friction, sideways (scrub)

# Slip model: how much wheel/ground speed difference is needed before the
# tyre saturates. Smaller = stickier/stiffer tyre.
SLIP_REFERENCE_MPS = 0.10


# =====================================================================
#  MOTORS
#
#  One choice, applied identically to all 8 motors (4 per robot, 2 robots).
#  See motors.py for the catalogue and the derivation of the electrical
#  constants from published datasheet figures.
#
#  Available keys:
#     pololu_37d_19_1     500 rpm,  5 kg-cm   <- fast, good default
#     pololu_37d_30_1     350 rpm,  8 kg-cm
#     pololu_37d_50_1     200 rpm, 21 kg-cm
#     pololu_37d_100_1    100 rpm, 34 kg-cm   <- too slow for football
#     gobilda_5203_5_2   1150 rpm,  ~7 kg-cm  <- very fast, small wheels only
#     gobilda_5203_13_7   435 rpm, ~17 kg-cm
#     gobilda_5203_19_2   312 rpm, ~24 kg-cm
#     n20_12v_200         200 rpm,  ~2 kg-cm  <- small robots only
#
#  goBILDA torque figures are ESTIMATED (their spec pages were unreachable
#  at build time) -- see motors.py. The Pololu figures are published.
# =====================================================================

MOTOR = "pololu_37d_30_1"

BATTERY_NOMINAL_V = 12.0
BATTERY_INTERNAL_R = 0.025    # was 0.08, which is a small or tired pack and
                              # cost 1.6 V of sag at 20 A. A healthy 3S LiPo
                              # is ~0.02 ohm -> ~0.4 V.    # ohms -> voltage sags under load

GEARBOX_BACKLASH_RAD = 0.010 # output-shaft slop, dead zone on reversal
MOTOR_DEADBAND_DUTY = 0.06   # duty below which static friction wins
MOTOR_VISCOUS_DAMPING = 2.0e-4   # N-m per (rad/s) at the output shaft
MOTOR_COULOMB_FRICTION = 0.010   # N-m constant drag at the output shaft

PWM_BITS = 10                # duty cycle quantisation on the robot
FIRMWARE_PID_HZ = 1000.0     # on-robot wheel velocity loop rate
FIRMWARE_KP = 0.55           # duty per (rad/s) of wheel speed error
FIRMWARE_KI = 2.20
FIRMWARE_KD = 0.0
FIRMWARE_I_LIMIT = 0.6


# =====================================================================
#  OVERHEAD CAMERA / VISION
#
#  Every one of these degrades what the AI sees. The correction layer in
#  ai/ undoes what can be undone; the rest becomes measurement noise.
# =====================================================================

CAM_HEIGHT_M = 2.500         # lens height above the playing surface
CAM_RESOLUTION_PX = (1280, 800)
CAM_COVERAGE_MARGIN = 1.18   # The camera must see MORE than the arena.
                             #
                             # Framed exactly to the arena width, a robot
                             # against a side wall has its markers projected
                             # OUT OF FRAME -- parallax pushes anything at
                             # marker height ~5% further from centre, so the
                             # edges fall outside the sensor. The AI then goes
                             # blind exactly where the ball spends most of its
                             # time. Measured: 35 mm belief error in open play
                             # with otherwise perfect sensors, versus 0.2 mm
                             # elsewhere.
                             #
                             # Budget for parallax, lens distortion and
                             # mounting error. This applies to the real rig
                             # too: frame wider than the pitch.
CAMERA_FPS = 90.0

EXPOSURE_S = 0.002           # 2 ms. Longer = more motion blur, more lag.
                             # Adds EXPOSURE_S/2 of effective latency and
                             # smears the blob centroid along the direction
                             # of travel.

TRANSFER_LATENCY_S = 0.008   # sensor readout + USB transfer
TRANSFER_JITTER_S = 0.002
DETECT_COMPUTE_S = 0.005     # blob detection time on the PC
DETECT_JITTER_S = 0.001

DETECT_NOISE_PX = 0.6        # 1-sigma centroid noise at frame centre
DETECT_NOISE_EDGE_GAIN = 2.0 # noise multiplier at the frame corner
                             # (worse optics + oblique view off-axis)

LENS_K1 = -0.180             # radial barrel distortion coefficients
LENS_K2 = 0.030
CALIB_RESIDUAL = 0.06        # fraction of the distortion the calibration
                             # FAILS to remove. 0 = perfect calibration
                             # (unrealistic). This is what makes undistortion
                             # imperfect, as it is in real life.

BALL_DROPOUT_PROB = 0.010    # random per-frame detection failure
MARKER_DROPOUT_PROB = 0.005
FALSE_POSITIVE_PROB = 0.002  # spurious blob somewhere on the pitch


# =====================================================================
#  ROBOT MARKERS  (colour circles on top -- no fiducial tags)
#
#  Large centre circle gives position and team identity.
#  Smaller offset dot gives heading.
#
#  Heading is recovered as atan2 of the vector between the two dots, so
#  heading noise is roughly  atan(pixel_noise / dot_separation).  A small
#  separation makes heading MUCH noisier than position -- this is the
#  single biggest weakness of colour-blob marker schemes, and it is
#  modelled faithfully.
# =====================================================================

MARKER_HEIGHT_M = 0.150      # top of a ~150 mm tall chassis      # height of the marker plane above the floor.
                             # Because this differs from the ball's height,
                             # a single homography cannot serve both --
                             # see the two-plane correction in ai/calibration.
MARKER_CENTRE_DIA_M = 0.110   # A 300 x 250 mm lid has room for a proper
                              # marker pair, and that is worth a great deal:
                              # heading noise scales as 1/separation, so the
                              # single biggest accuracy win available on this
                              # robot is simply that it is big.
MARKER_DOT_DIA_M = 0.055
MARKER_DOT_SEPARATION_M = 0.105
                              # Dot outer edge at 132.5 mm, inside the 150 mm
                              # chassis half-length. Keep the two blobs clear
                              # of each other or the detector merges them.   # centre-to-dot distance. Increase to make
                                  # heading estimation dramatically better.


# =====================================================================
#  ENCODERS AND TELEMETRY
#
#  The AI receives all 8 encoders -- its own 4 and the opponent's 4 --
#  over the same radio link, with the same delay and loss.
# =====================================================================

ENCODER_SAMPLE_HZ = 1000.0   # on-robot encoder read rate
ENCODER_VELOCITY_WINDOW = 5  # samples averaged to estimate wheel velocity

TELEMETRY_HZ = 100.0         # rate encoder data is sent back to the PC
TELEMETRY_LATENCY_S = 0.004
TELEMETRY_JITTER_S = 0.001
TELEMETRY_LOSS_PROB = 0.01


# =====================================================================
#  RADIO / COMMAND PATH  (PC -> robot)
# =====================================================================

RADIO_LATENCY_S = 0.003      # ESP-NOW class link
RADIO_JITTER_S = 0.001
RADIO_LOSS_PROB = 0.01
RADIO_WATCHDOG_S = 0.200     # robot halts if no packet arrives within this

STRATEGY_COMPUTE_S = 0.004   # time the AI itself takes to decide


# =====================================================================
#  HUMAN INPUT
#
#  The player uses a keyboard here, but will use an analog stick on the
#  real thing. A raw on/off keyboard would handicap them against the AI's
#  continuous command, so held keys ramp a virtual stick instead.
# =====================================================================

KEYBOARD_POLL_HZ = 125.0     # USB HID polling rate
INPUT_LATENCY_S = 0.016      # OS input queue + display latency
STICK_RAMP_UP = 6.0          # units/second toward the held direction
STICK_RAMP_DOWN = 10.0       # units/second back to centre on release

# Keys (see game/input.py). Forward/back and turn left/right.
KEY_FORWARD = "w"
KEY_BACK = "s"
KEY_LEFT = "a"
KEY_RIGHT = "d"


# =====================================================================
#  COMMAND LIMITS  --  APPLIED IDENTICALLY TO BOTH ROBOTS
#
#  These cap what any controller, human or AI, may ask for. They exist
#  because the physical yaw limit of this drivetrain is about 20 rad/s
#  (three revolutions per second), which is not controllable by anyone
#  and makes for a silly game.
#
#  Keep these symmetric. The AI's advantage is meant to come from
#  decisions and information, not from being allowed to move differently.
# =====================================================================

COMMAND_MAX_SPEED_FRAC = 1.00   # fraction of no-load wheel speed
COMMAND_MAX_YAW_FRAC = 0.35     # fraction of the physical yaw limit


# =====================================================================
#  AI CAPABILITY FLAGS
#
#  Each correction and capability toggles independently. Turn things off
#  to see exactly what each one is worth, or to build a difficulty ladder.
# =====================================================================

USE_UNDISTORT = True         # remove lens distortion
USE_TWO_PLANE_HOMOGRAPHY = True   # correct marker-vs-ball height parallax
USE_KALMAN = True            # filter and fuse; without it, raw detections
USE_ENCODER_FUSION = True    # fuse own encoders into own state
USE_OPPONENT_ENCODERS = True # fuse opponent encoders into their state
USE_SLIP_GATING = True       # distrust encoders when scrubbing
USE_LATENCY_COMP = True      # forward-predict by the measured loop delay
AUTO_LATENCY_ID = True       # measure own loop delay online. This is system
                             # identification of YOUR OWN hardware, not
                             # modelling the player. False -> use the constant.
FIXED_LATENCY_S = 0.030      # used when AUTO_LATENCY_ID is False
LATENCY_ID_HZ = 2.0          # how often to re-run the correlation search.
                             # Loop delay is a property of the hardware and
                             # drifts on a timescale of minutes, so this is
                             # already generous. Running it every control
                             # tick costs more than the entire planner --
                             # measured at 69% of total runtime before this
                             # was rate-limited.

USE_INTENT_CHANNEL = True    # read the opponent's live command as it is
                             # issued, before their robot visibly responds
USE_REACHABILITY = True      # provable shot / block geometry

# The AI's internal model of its own drivetrain. Deliberately approximate --
# the real response is traction-limited, voltage-dependent and nonlinear, and
# a first-order lag is what you would actually fit to it. The mismatch is
# part of the point.
DRIVETRAIN_TAU_S = 0.18      # velocity response time constant
BALL_CONTESTED_Q_GAIN = 1.0  # DEFAULT OFF (1.0 = no effect).
                             # Process-noise multiplier while a robot is
                             # close enough to be pushing the ball. The
                             # filter's constant-velocity model is exact for
                             # a free ball (measured: 0.000 mm) and wrong for
                             # a contested one (~4 mm, spikes past 150 mm), so
                             # it should stop trusting itself when contact is
                             # plausible and lean on the camera instead.
                             #
                             # Theoretically right, but NOT demonstrated: across
                             # gains of 1/10/30/60/150 the full-noise ball error
                             # came out 14.9/13.4/23.2/13.2/14.9 mm -- non-
                             # monotonic, i.e. seed variance swamps the effect at
                             # 3 seeds. Left off rather than shipped on the
                             # strength of a number that is really noise. Raise it
                             # and measure over ~30 matches if you want to settle it.
SLIP_GATE_GAIN = 0.9         # how hard to distrust encoders per rad/s of yaw.
                             # Scrub grows with turn rate, so encoder-derived
                             # twist gets less trustworthy the harder we turn.
FILTER_LAG_S = 0.030         # how far behind "now" the filter runs, so that
                             # out-of-order measurements can still be applied
                             # in timestamp order. Must exceed the slowest
                             # sensor path (vision, ~16 ms worst case).

PLANNER = "mpc"              # "mpc" | "fsm"
                             #
                             # HONEST STATUS: over 8 matches against the
                             # chase baseline the FSM scored +0.38 goals per
                             # match and the MPC +0.12. That gap is well
                             # inside the noise at this sample size, but the
                             # MPC is NOT yet demonstrably better, and it
                             # costs about 3x the compute. Run
                             #   python -m tools.headless --compare --matches 30
                             # before trusting either over the other.

SPEED_CAP_FRAC = 1.0         # handicap: fraction of full speed the AI may use
AI_REACTION_DELAY_S = 0.0    # handicap: artificial extra delay on the AI
AI_COMMAND_QUANTIZATION = False   # force AI onto the same discrete command
                                  # set the keyboard produces

# Analytic opponent prediction. NOT learned -- a fixed assumption about how
# long a human holds a command before they can change it. Set from general
# knowledge of human reaction time, never measured from the player.
OPPONENT_ASSUMED_REACTION_S = 0.250
OPPONENT_COMMAND_DECAY_S = 0.400  # after that, assume their command decays

USE_COMMITMENT = True        # Detect when the opponent is holding a command
                             # and widen the window in which they cannot
                             # respond. Measured from the live command stream
                             # only -- nothing fitted, nothing remembered, and
                             # it reads zero against a player doing nothing.
COMMITMENT_WINDOW_N = 30     # commands to look back over (~0.3 s at 100 Hz)
COMMITMENT_SPREAD_REF = 0.9  # command spread at which commitment reads zero
COMMITMENT_MIN_MAGNITUDE = 0.25
                             # a player sitting still is steady but is not
                             # committed to anything
COMMITMENT_EXTRA_S = 0.30    # extra locked-in time at full commitment. This
                             # is the number that turns "they probably cannot
                             # reach that" into "they provably cannot".

OPPONENT_ASSUME_CHASES_BALL = False
                             # Once the observed command has decayed, assume
                             # they are going for the ball. A FIXED prior,
                             # identical for every player and never fitted to
                             # one. Set False to assume only coasting, which
                             # is safer but makes the planner short-sighted.


# =====================================================================
#  PLANNER TUNING
# =====================================================================

MPC_HORIZON_S = 2.00         # Was 1.20, which could not see a goal from
                             # midfield: pushing the ball 1.4 m takes just
                             # over 1.2 s from a standing start, so the
                             # terminal reward never fired and the planner
                             # chose between near-identical shaping scores.
MPC_STEP_S = 0.033           # Coarsened from 0.020 so the longer horizon
                             # costs no more compute (60 steps either way).
                             # Safe because the contact test is swept, not
                             # point-sampled.
MPC_CANDIDATES = 32

MPC_REPLAN_HZ = 25.0         # how often to re-run the rollout search. The
                             # CONTROLLER still runs at CONTROL_HZ, tracking
                             # the chosen target -- only the search is slower.
                             # Replanning every control tick is wasted work:
                             # the world barely changes in 10 ms, and the
                             # chosen behaviour almost never does.
REACH_REPLAN_HZ = 15.0       # reachable sets change slowly; recomputing them
                             # every tick is the single most expensive thing
                             # the planner could do.

# Scoring weights for a rolled-out future state
W_GOAL = 1000.0              # we scored
W_CONCEDE = -500.0           # they scored.
                             # Was -1400: conceding weighted 40% heavier than
                             # scoring made the planner structurally unwilling
                             # to leave its own half. Measured at 16 matches x
                             # 120 s with the direct striker, -500 beat -1400
                             # by 15F 6A to 13F 7A and put 1.4 more points of
                             # the match in the attacking third.
W_BALL_TO_THEIR_GOAL = -6.0  # per metre
W_BALL_TO_OUR_GOAL = 3.0     # per metre
W_POSSESSION = 25.0          # ball sitting in our pocket
W_OPEN_ANGLE = 12.0          # radians of unblocked goal mouth available
W_OPPONENT_POSSESSION = -30.0
W_SELF_TO_BALL = -1.5        # per metre, mild pull toward the ball
W_FACING_BALL = 2.0

W_AVOID_OPPONENT = 0.0    # penalty per second of predicted body contact
                             # with the opponent. The rollout already
                             # simulates their motion from their live command
                             # and known dynamics, so a candidate that runs
                             # into them is visible BEFORE it happens -- this
                             # is what turns that foresight into dodging.
                             #
                             # MEASURED TRADE-OFF (10 matches each): avoidance
                             # buys fewer collisions and costs goals, because
                             # contesting the ball MEANS being near them.
                             #      0  contact 8.5%  AI 3-2  speed 0.44
                             #    -50  contact 7.4%  AI 1-1  speed 0.35
                             #    -90  contact 4.1%  AI 1-0  speed 0.34  <-
                             #   -140  contact 6.6%  AI 0-0  speed 0.31
                             # Goal difference is flat across all of them, so
                             # SET TO 0. Avoiding the opponent means not
                             # contesting the ball, because the ball is where
                             # the opponent is -- it cut goals from 6 to 1 and
                             # made the robot safe but inert. Raise it only if
                             # you would rather have fewer collisions than
                             # goals. The wall and reversing terms below are
                             # independent and stay on: those were bugs, not
                             # trade-offs.
W_AVOID_WALL = -55.0         # penalty per second spent pressed against a wall.
                             # The robot was touching a wall 56% of the match;
                             # a wall is not a place to play from.
W_REVERSING = -18.0          # mild cost per second of driving backwards.
                             # Reversing is sometimes right on a robot that
                             # cannot strafe, so this discourages rather than
                             # forbids it.

W_TURN_WHILE_CARRYING = -55.0
                             # Penalty per radian turned while the ball is in
                             # the pocket.
                             #
                             # Passive horns cannot resist lateral
                             # acceleration -- there is no dribbler bar
                             # pressing the ball down. Measured carry through
                             # a manoeuvre: 57% straight, 51% gentle turn,
                             # 20% hard turn. So the ball is not lost by bad
                             # luck, it is lost by TURNING while holding it.
                             #
                             # The robot therefore has to do its turning
                             # BEFORE it takes possession, and drive straight
                             # once it has the ball. This term is what makes
                             # the planner line up first instead of grabbing
                             # the ball and then trying to steer with it.

W_DEFEND_ALIGN = 46.0        # mirror of W_APPROACH_ALIGN, used when the
                             # OPPONENT holds the ball: reward being between
                             # the ball and the goal we are defending.
                             # Weighted higher than the attacking term because
                             # conceding costs more than scoring gains
                             # (W_CONCEDE is larger than W_GOAL for the same
                             # reason).

W_APPROACH_ALIGN = 34.0      # THE important positional term.
                             #
                             # Rewards being on the correct SIDE of the ball
                             # -- behind it, on the line to their goal -- and
                             # not merely near it. Without this, whenever no
                             # candidate can score inside the horizon the
                             # planner falls back on proximity alone, always
                             # picks "intercept", and arrives at the ball from
                             # whatever angle it happened to be at. With no
                             # kicker, arriving on the wrong side means
                             # shoving the ball away from the goal, so
                             # proximity without alignment is worthless.
                             #
                             # Measured: omitting this made the rollout
                             # planner LOSE to a naive chase baseline.

MPC_MIN_COMMIT_S = 0.45      # minimum time to stick with a chosen behaviour.
                             #
                             # The drivetrain has a 0.18 s time constant, so it
                             # needs ~0.54 s of STEADY command to reach speed.
                             # Measured without this: the planner switched
                             # candidate 9 times a second and flipped the sign
                             # of its velocity command 3.4 times a second, so
                             # the robot never accelerated -- 0.30 m/s against
                             # a 2.62 m/s top speed, with nothing blocking it.
                             #
                             # Deciding faster than the machine can respond is
                             # not responsiveness, it is paralysis. The
                             # one-trick SimpleBot beats the planner 66-2
                             # purely by never interrupting itself.
                             #
                             # A large score change (a goal or concede
                             # appearing in the rollout) still overrides at once.
MPC_OVERRIDE_FRAC = 0.45     # how far a rival must beat the incumbent, as a
                             # fraction of the score spread, to cut a
                             # commitment short
MPC_SWITCH_MARGIN_FRAC = 0.12
                             # Hysteresis as a FRACTION of the score spread,
                             # not an absolute. It was 12.0 absolute while the
                             # spread between best and worst candidate was ~4,
                             # so the planner could never leave its first choice
                             # no matter how much better another became -- it sat
                             # in 'lineup' with the ball 20 cm away, forever.
MPC_SWITCH_MARGIN = 12.0      # how much better a new candidate must score
                             # before the planner abandons its current one.
                             # Zero means re-deciding from scratch 25 times a
                             # second, which makes the robot dither: several
                             # candidates usually score within a point of each
                             # other, so it flips between them and never
                             # finishes a manoeuvre. Every switch costs a turn
                             # on a robot that cannot strafe.

REACH_TIME_SLICES = (0.15, 0.30, 0.50, 0.75, 1.00)
REACH_GRID_M = 0.04          # occupancy grid resolution for reachable sets
REACH_CONTROL_SAMPLES = 11   # (v, omega) samples per axis.
                             # 11x11=121 control choices is plenty to trace
                             # the boundary of the reachable set; the cost
                             # is quadratic in this number.


# =====================================================================
#  MATCH RULES
# =====================================================================

MATCH_DURATION_S = 300.0
GOAL_REQUIRES_FULL_CROSS = False   # False: ball centre past the line
                                   # True:  entire ball past the line
KICKOFF_BALL_POS = (0.0, 0.0)
KICKOFF_ROBOT_OFFSET_M = 0.55      # distance from centre each robot starts
POST_GOAL_PAUSE_S = 1.5

STUCK_BALL_TIMEOUT_S = 12.0        # ball barely moving for this long -> reset
STUCK_BALL_SPEED_MPS = 0.05

# Robot jam detection (ai/tactics.py). "Asking for speed and not getting it"
# covers being wedged on a wall, shoved by the other robot, or nose-first in
# a corner, without needing to distinguish them.
# --- ball capture (ai/tactics.CaptureMonitor) ---------------------------
# Taking the ball is not a decision, it is a reflex. These govern the one
# committed lunge that puts the ball between the horns.
# --- carrying (ai/tactics.CarryController) -----------------------------
# With the ball in the pocket the robot must AIM FIRST and then drive
# straight, because a passive pocket sheds the ball in a hard turn (57%
# retention straight vs 20% through a hard turn).
CARRY_ABANDON_DEG = 45.0     # if we hold the ball but are pointed further
                             # than this from the aim, give it up and go round
                             # again. Turning with the ball takes 4.8 s and
                             # possession lasts 0.43 s -- the turn can never
                             # finish, so attempting it only loses the ball
                             # slowly instead of quickly.
CARRY_ALIGN_DEG = 20.0       # must be within this of the aim before driving.
                             # Was 8.0, which meant the robot spent 5x longer
                             # turning with the ball than driving it (980 vs
                             # 206 samples) -- it pirouetted instead of
                             # playing. It can steer while driving; it does
                             # not need to be perfectly aimed first.
CARRY_RESUME_DEG = 26.0      # hysteresis, so it does not flicker while driving
CARRY_TURN_FRAC = 0.30       # yaw limit while turning with the ball
CARRY_TURN_SPEED_FRAC = 0.34 # forward pressure holds the ball on the face
                             # AND keeps the robot moving while it lines up.
CARRY_DRIVE_FRAC = 1.00      # once aimed, commit fully

CAPTURE_EXTRA_M = 0.10       # how far beyond horn reach to start a capture
CAPTURE_CONE_DEG = 40.0      # must be roughly facing the ball to try
# --- striking (ai/tactics.StrikeSequence) ------------------------------
# Attacking is line-up / aim / strike, executed with commitment. Copied in
# spirit from the one-trick bot that beats the planner 16-0.
STRIKE_CLAIM_MARGIN_M = 0.40 # go for the ball if we are at least this
                             # close to being nearer it than they are.
                             #
                             # RAISED FROM 0.10 once ShadowDefender existed.
                             # With a real defensive line behind it the robot
                             # can afford to contest far more; without one,
                             # contesting and losing left the goal open.
                             # Measured, 8-9 matches x 120 s per cell:
                             #             0.10            0.40
                             #   simple    +0.44 10F 6A    -0.44  9F 13A
                             #   runner    +1.89 17F 0A    +1.89 23F  6A
                             #   human     -1.75  2F 16A   +1.12 18F  9A
                             #   total     +0.58           +2.57
                             # 0.75 was also tried against the human and was
                             # worse (-1.12), so this is a real interior
                             # optimum, not a monotonic trend. The human-shaped
                             # fixture is the only proxy for the actual
                             # opponent, and it swings by 2.87 goals.
                             # NOTE: the optimum is opponent-dependent, and
                             # picking per opponent would mean identifying who
                             # is playing -- that is profiling, which this
                             # project forbids. One value serves everyone.
STRIKE_AIM_DEG = 12.0        # aim this precisely before committing to the run
STRIKE_COMMIT_S = 0.85       # once striking, do not re-deliberate for this long

# When the ball is deep in OUR half, demand a clearer claim before attacking
# instead of falling back to defend. Measured: the old hardcoded rule marked
# the back 65% of our own half as "threatened" regardless of who was nearer,
# and threw away possession we already had on 14.8% of the whole match --
# against 1.1% where a retreat was genuinely forced. Being deep is not itself
# a reason to give up the ball: the fastest way out of your own half is to
# strike the ball out of it.
STRIKE_THREATENED_FRAC = 0.35 # fraction of our half that counts as "deep"
STRIKE_DEEP_MARGIN_M = -0.15  # claim margin while deep: negative means we
                              # must be clearly closer, not merely level

STRIKE_BEHIND_COS = 0.80     # how squarely behind the ball we must be
                             # before the run counts as a strike (1.0 = dead
                             # in line, 0.0 = anywhere on the near side)
STRIKE_RANGE_MULT = 2.40     # ...and how close, as a multiple of the
                             # stand-off distance behind the ball

# --- orienting (ai/tactics.OrientToBall) -------------------------------
# The horns are on the front only, so being near the ball is useless while
# pointing away from it. Retreat may be backwards; taking the ball may not.
ORIENT_RANGE_M = 0.65        # start caring about facing inside this range
ORIENT_TOL_DEG = 35.0        # turn to face if further off than this
ORIENT_COMMIT_S = 0.30       # committed turn, no re-deliberation
ORIENT_SPEED_FRAC = 0.30     # creep forward while swinging round
ORIENT_BEHIND_ONLY = True    # only orient from BEHIND the ball.
                             # Measured over 8 x 120 s: the robot is within
                             # ORIENT_RANGE_M of the ball 85.7% of the match,
                             # and on 22.5% of the WHOLE match it is near the
                             # ball, off-aim, AND standing on the opponent-goal
                             # side of it -- 79% of every tick ORIENT could
                             # fire. From that side "turn to face the ball"
                             # means "turn to face your own goal", and because
                             # ORIENT sits above the strike gate it pre-empts
                             # the line-up that would have walked round to the
                             # correct side. The robot then fails CAPTURE's aim
                             # cone (the binding gate: 19.0% of all ticks, only
                             # 0.40% ever pass), never takes the ball, and
                             # loops. ORIENT's real job is the retreating
                             # defender turning to meet an incoming ball, and
                             # that case is always from behind.
ORIENT_BEHIND_SLACK_M = 0.05 # hysteresis on that side test

STRIKE_LINEUP_REVERSE = True # may the line-up REVERSE into the strike point?

# --- direct striking (ai/tactics.DirectStriker) ------------------------
# ATTACK_MODE picks the attacking control law:
#   "direct"   SimpleBot's two-state law -- turn on the spot until aimed, then
#              drive dead straight -- but fed the predicted ball and the
#              planner's chosen aim instead of ground truth and the goal centre.
#   "sequence" the older lineup/aim/strike StrikeSequence.
# Measured, same seeds and probe, only our controller swapped: SimpleBot took
# 14.9 shots a match with 1.71 on target and 28.8% of the match in the
# attacking third; the full AI managed 10.2 / 0.57 / 14.0%. The control law was
# the difference, not the intelligence behind it.
ATTACK_MODE = "direct"
DIRECT_ANGLE_TOL_DEG = 7.0    # must be this well aimed before it will drive
DIRECT_ANGLE_RESUME_DEG = 22.0 # ...and tolerates this much once rolling, so it
                               # does not flicker between turning and driving
DIRECT_BEHIND_COS = 0.85      # how squarely behind the ball before it commits
DIRECT_RANGE_MULT = 2.60      # ...and how close, in stand-off multiples
DIRECT_LEAD_S = 0.12          # lead the ball by this much. The AI knows where
                              # the ball is GOING; SimpleBot only knows where it
                              # was. This is where that advantage is spent.
DIRECT_OWNS_APPROACH = True   # a committed attacking run outranks ORIENT,
                              # CAPTURE and CARRY. Without this the run was
                              # taken over at the exact moment it worked: the
                              # ball reached the horns, possession flipped, and
                              # CARRY swapped the control law mid-strike.
USE_WALL_EXTRACTION = True    # a ball on a wall has no reachable "behind":
                              # the strike point is outside the arena, so the
                              # robot drove at it and wedged. Measured: 78.8%
                              # of striking ticks aimed outside the pitch, the
                              # robot ground against a wall for 45.2% of the
                              # match, and the stuck rule teleported the ball
                              # back to centre 3.0 times a match. With this on,
                              # a pinned ball is swept ALONG the wall instead.
WALL_SWEEP_COMMIT_S = 0.80    # hold the sweep decision this long. Without
                              # it the pinned test sat on a threshold, flicked
                              # on and off, and the robot alternated between
                              # two control laws 40 times a second -- visible
                              # as vibrating on the spot beside the wall.
WALL_SWEEP_TOL_DEG = 12.0     # heading tolerance while sweeping along a wall
WALL_SWEEP_SPEED_FRAC = 0.85  # sweeping is a committed run, not a nudge
WALL_SWEEP_PICK_NEAREST = True # in a corner, both walls are escape routes.
                              # Pick the one the robot can start on NOW rather
                              # than by fixed preference, which could send it
                              # the long way round the ball while the stuck
                              # timer ran out.
PINNED_CLAIM_MARGIN_M = 0.00  # claim a ball stuck on a wall far more
                              # readily than a loose one. SET TO ZERO, i.e.
                              # OFF, because measuring it refuted the idea.
                              #
                              # The reasoning was sound: the planner has no
                              # candidate that extracts a pinned ball, and the
                              # sweep never ran at all in half of all corner
                              # episodes. But claiming those balls costs more
                              # than it wins. Swept at 10 matches x 120 s
                              # against both deep-margin settings:
                              #   0.00 / deep -0.15   -0.20   12F 14A
                              #   0.00 / deep -0.70   +0.00    8F  8A
                              #   0.45 / deep -0.15   -0.70    3F 10A
                              #   0.45 / deep -0.70   -0.80    4F 12A
                              # 0.45 is worse on BOTH rows, by 0.5 and 0.8
                              # goals. Chasing every ball on a wall abandons
                              # the goal, and a wall ball is worth less than
                              # the position given up to go and get it.

# --- shadow defending (ai/tactics.ShadowDefender) ----------------------
# When the ball is not ours to claim, stand between it and our own goal and
# hold that line facing it, instead of handing over to the planner's cover /
# cutoff / block placements. Measured, same seeds, only our controller
# swapped: the AI OUTSCORES the 40-line control against every opponent
# (12 v 8, 20 v 15, 11 v 10) and loses on goals conceded (14 v 2, 11 v 2,
# 22 v 22). SimpleBot's attacking target is behind the ball on the
# ball-to-their-goal line, which is by construction goal-side of the ball, so
# it defends perfectly by accident. This gives the AI that geometry
# deliberately.
USE_SHADOW_DEFENCE = True
SHADOW_STANDOFF_M = 0.55     # how far goal-side of the ball to stand
SHADOW_ARRIVE_M = 0.12       # within this, stop and face the ball
SHADOW_FACE_TOL_DEG = 10.0   # ...to this accuracy
SHADOW_LEAD_S = 0.20         # defend where the ball is GOING. Longer than the
                             # striker's lead: a defender is reacting to a
                             # shot, and being early costs nothing.
DIRECT_MIN_TARGET_M = 0.30    # below this range a target's BEARING is unstable
                              # (it swings tens of degrees per tick as the
                              # robot creeps), so stop steering hard by it.
TARGET_BEARING_FLOOR_M = 0.22 # the same guard, for EVERY drive_to target.
                              # `through` targets skip the arrival branch, so
                              # defensive and repositioning candidates chased
                              # points they were standing on. Near-zero-net-
                              # displacement seconds: defend:cover0.3 37.9%,
                              # defend:cutoff_deep 28.0%, reposition:peelL
                              # 21.9%, against the guarded striker's 10.3%.
ANCHOR_RELATIVE_TARGETS = True # freeze peel/reverse targets in world space
                              # once chosen. They are defined from the robot's
                              # own heading, so a live recompute rotates them
                              # away as fast as the robot turns toward them.
DRIVE_MIN_BEARING_M = 0.30    # the same fault in the shared controller:
                              # below this range drive_to() fades out its
                              # steering gain and stops rotating on the spot,
                              # because a target this close has a bearing that
                              # swings tens of degrees per tick. This is what
                              # the defending and repositioning behaviours were
                              # doing when they appeared to vibrate.
                             # Reversing gets there sooner but arrives facing
                             # away from the ball, and the robot then has to
                             # swing right round before it can strike. Measured:
                             # STRIKE:lineup runs 19.9% of the match and
                             # STRIKE:strike only 1.23% -- a 16:1 ratio, against
                             # SimpleBot's 1:2.5 turn:drive ratio. Arriving in
                             # the right place facing the wrong way is a strong
                             # candidate for where those runs die.

# --- deadlock (ai/tactics.DeadlockBreaker) -----------------------------
# Two equal robots nose to nose with the ball pinned between them cannot
# resolve it by pushing. Detect it and go round instead.
DEADLOCK_DETECT_S = 0.35     # nose-to-nose with the ball trapped this long
DEADLOCK_ESCAPE_S = 0.60     # committed back-out-and-turn

CAPTURE_MIN_OPP_SEP_M = 0.42 # do not go for the ball if the opponent is
                             # this close to it. Contesting a ball the
                             # opponent is sitting on produces a scrum, not
                             # possession -- and in a scrum the robot achieves
                             # 39% of its commanded yaw, so it cannot use the
                             # ball even when it wins it.
CAPTURE_AIM_CONE_DEG = 60.0  # AND must already be pointing roughly where the
                             # ball needs to go. Taking the ball facing the
                             # wrong way just means turning with it, and
                             # turning with it loses it.
CAPTURE_COMMIT_S = 0.40      # hold the lunge this long, no re-planning
CAPTURE_SPEED_FRAC = 0.75    # fast enough to close, slow enough to collect
CAPTURE_STEER_GAIN = 1.6     # gentle -- hard steering sweeps the ball away

DEFEND_TIME_MARGIN_S = 0.18  # how much earlier than the ball we must reach a
                             # cut-off point before it counts as blocked.
                             # Arriving simultaneously is arriving late -- the
                             # robot still has to be facing the right way.

STUCK_DETECT_S = 0.90              # how long to be jammed before reacting.
                                   # Was 0.45, which fired 352 times in four
                                   # matches -- more escape time than match
                                   # time. Since the escape command is REVERSE,
                                   # that alone was 45% of the match spent
                                   # driving backwards. A jam that clears
                                   # itself in under a second was never a jam.
STUCK_ESCAPE_S = 0.55              # how long the reverse-and-turn escape runs
STUCK_MIN_TRAVEL_M = 0.020         # travelled less than this over the detect
                                   # window, while asking for speed, = jammed.
                                   # Measured on POSITION, not velocity: a
                                   # wedged robot's wheels still spin, so its
                                   # encoders (and therefore its believed
                                   # velocity) insist it is moving.


# =====================================================================
#  RENDERING
# =====================================================================

RENDER_SCALE_PX_PER_M = 340.0
RENDER_MARGIN_PX = 40

SHOW_TRUE_STATE = True       # ground truth (solid)
SHOW_AI_BELIEF = True        # what the AI thinks (dashed/ghost)
SHOW_PREDICTION = True       # latency-compensated forward prediction
SHOW_REACHABILITY = False    # opponent reachable set overlay
SHOW_MPC_ROLLOUTS = False    # candidate trajectories, coloured by score
SHOW_ESTIMATOR_HUD = True    # live RMS belief-vs-truth error readout
SHOW_DELAY_BUDGET = True     # breakdown of where the latency goes


# =====================================================================
#  DERIVED  (computed from the above -- do not edit)
# =====================================================================

BALL_RADIUS_M = BALL_DIAMETER_M / 2.0
HALF_LENGTH_M = ARENA_LENGTH_M / 2.0
HALF_WIDTH_M = ARENA_WIDTH_M / 2.0
HALF_GOAL_M = GOAL_WIDTH_M / 2.0


def robot_inertia() -> float:
    """Yaw inertia of the body about its centre, rectangular plate."""
    base = (ROBOT_MASS_KG / 12.0) * (ROBOT_LENGTH_M ** 2 + ROBOT_WIDTH_M ** 2)
    return base * ROBOT_INERTIA_SCALE


def effective_track_m() -> float:
    """Track width the AI's kinematic model should assume."""
    return TRACK_WIDTH_M * SCRUB_FACTOR


def total_vision_latency_s() -> float:
    """Nominal camera-to-decision latency, excluding the command path."""
    return (EXPOSURE_S / 2.0
            + 0.5 / CAMERA_FPS
            + TRANSFER_LATENCY_S
            + DETECT_COMPUTE_S)


def total_loop_latency_s() -> float:
    """Full sense-decide-act delay. What latency compensation must undo."""
    return (total_vision_latency_s()
            + STRATEGY_COMPUTE_S
            + RADIO_LATENCY_S
            + AI_REACTION_DELAY_S)


def px_per_m() -> float:
    """Camera scale. Assumes the camera frames the arena width exactly."""
    return CAM_RESOLUTION_PX[1] / ARENA_WIDTH_M
