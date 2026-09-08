"""
Estimator and correction-layer tests.

The first test here is the most important one in the project.

THE ZERO-NOISE TEST
-------------------
Turn every noise, delay, dropout and distortion source off, and the AI's
belief must reproduce ground truth to within numerical precision. If it does
not, the correction MATHS is wrong -- not mistuned, wrong -- and no amount of
gain tweaking later will fix it.

This matters because almost every "my robot is jittery / it drifts / it
overshoots" problem on real hardware is a correction bug wearing a costume,
and it is invisible once real noise is switched on, because everything looks
approximately plausible when everything is approximately wrong.

Run:  python -m tests.test_estimator
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config

FAILURES = []


def check(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"   {detail}" if detail else ""))
    if not cond:
        FAILURES.append(name)


def silence_all_noise():
    """Every corruption source off. The perfect-sensor world."""
    config.DETECT_NOISE_PX = 1e-9
    config.DETECT_NOISE_EDGE_GAIN = 1.0
    config.LENS_K1 = 0.0
    config.LENS_K2 = 0.0
    config.CALIB_RESIDUAL = 0.0
    config.EXPOSURE_S = 0.0
    config.TRANSFER_LATENCY_S = 0.0
    config.TRANSFER_JITTER_S = 0.0
    config.DETECT_COMPUTE_S = 0.0
    config.DETECT_JITTER_S = 0.0
    config.BALL_DROPOUT_PROB = 0.0
    config.MARKER_DROPOUT_PROB = 0.0
    config.FALSE_POSITIVE_PROB = 0.0
    config.TELEMETRY_LATENCY_S = 0.0
    config.TELEMETRY_JITTER_S = 0.0
    config.TELEMETRY_LOSS_PROB = 0.0
    config.RADIO_LATENCY_S = 0.0
    config.RADIO_JITTER_S = 0.0
    config.RADIO_LOSS_PROB = 0.0
    config.STRATEGY_COMPUTE_S = 0.0
    config.AI_REACTION_DELAY_S = 0.0
    config.CAMERA_FPS = 1000.0
    config.TELEMETRY_HZ = 1000.0
    config.AUTO_LATENCY_ID = False
    config.FIXED_LATENCY_S = 0.0
    # FILTER_LAG_S is deliberately left at its normal value. Shrinking it
    # below one measurement interval makes the filter run AHEAD of the data
    # it has and coast uncorrected, which looks like an estimator error but
    # is just a starved filter -- an easy way to fool yourself with this test.


def restore_defaults():
    import importlib
    importlib.reload(config)


def run_match(seconds, agent_cls=None):
    """Build a match with the real agent and run it headlessly."""
    from game.input import HumanController
    from game.match import Match
    from game.main import TruthChaser
    from ai.agent import Agent

    human = HumanController()
    m = Match(None, human)
    agent = Agent(*m.hal(0, human), robot_index=0, truth=m.world)
    m.controllers[0] = agent
    # Give the opponent something to do so the world is not static.
    opp = TruthChaser(m.world, 1)
    m.controllers[1] = opp

    dt = config.PHYSICS_DT
    for _ in range(int(seconds / dt)):
        m.step(dt)
    return m, agent



def _ball_error_split(m, agent):
    """Mean ball belief error, split by whether a robot was touching it.

    Re-runs a short match rather than trusting the aggregate, because the
    aggregate cannot distinguish the two regimes and they differ by orders
    of magnitude.
    """
    import statistics
    from sim.geometry import dist
    from game.input import HumanController
    from game.match import Match
    from game.main import TruthChaser
    from ai.agent import Agent

    human = HumanController()
    m2 = Match(None, human, seed=4242)
    a2 = Agent(*m2.hal(0, human), robot_index=0, truth=m2.world)
    m2.controllers[0] = a2
    m2.controllers[1] = TruthChaser(m2.world, 1)

    reach = (config.ROBOT_LENGTH_M / 2.0 + config.HORN_LENGTH_M
             + config.BALL_RADIUS_M + 0.02)
    free, touch = [], []
    for i in range(10000):
        m2.step(config.PHYSICS_DT)
        if i % 5:
            continue
        raw = a2.estimator.belief()
        if not raw.ball_valid:
            continue
        tr = a2._truth_at(raw.t)
        if tr is None:
            continue
        e = dist(raw.ball_pos, tr[0])
        contact = any(dist(m2.world.ball.pos, r.pos) < reach
                      for r in m2.world.robots)
        (touch if contact else free).append(e)

    # MEDIAN, not mean.
    #
    # A handful of samples land in the few frames after a kickoff, where the
    # ball has been TELEPORTED to the centre spot. No filter can track a
    # teleport: the outlier gate correctly rejects the jump while it
    # re-acquires. Those frames produce metre-scale figures that swamp a mean
    # -- and because they only happen after goals, the estimator would appear
    # to get worse the better the AI scored, which is exactly backwards.
    def med(xs):
        if not xs:
            return 0.0
        s = sorted(xs)
        return s[len(s) // 2]

    return med(free), med(touch)


# ---------------------------------------------------------------------------

def test_zero_noise():
    print("ZERO-NOISE TEST  (the correction maths must be exact)")
    silence_all_noise()
    import importlib
    import ai.calibration, ai.estimator, sim.sensors.camera
    importlib.reload(ai.calibration)
    importlib.reload(ai.estimator)

    m, agent = run_match(12.0)

    # The ball is scored ONLY while it is free.
    #
    # A ball being shoved by a robot is not evidence about the correction
    # maths: the filter's process model is constant velocity plus rolling
    # drag, which is exactly right for a free ball and knowingly wrong for a
    # contested one. Measured with perfect sensors, the split is stark --
    # 0.000 mm free, ~4 mm in contact with spikes past 150 mm.
    #
    # Conflating the two also makes the test depend on robot-to-ball size
    # ratio: the original 180 mm robot / 43 mm ball config left the ball free
    # most of the time, while a 200 mm robot with a 100 mm ball has it in
    # contact 94% of the time. Same estimator, five times the reported error,
    # purely from geometry. Scoring the free ball keeps this a test of the
    # maths rather than of the config.
    free_err, contact_err = _ball_error_split(m, agent)
    print(f"       ball free {free_err*1000:7.4f} mm   "
          f"in contact {contact_err*1000:7.3f} mm  (diagnostic)")
    check("free ball tracks truth exactly", free_err < 0.0005,
          f"{free_err*1000:.4f} mm")

    err = agent.error_summary()
    for key, tol in (("self", 0.0025), ("opp", 0.0025)):
        check(f"{key} belief matches truth", err[key] < tol,
              f"{err[key]*1000:.4f} mm  (tol {tol*1000:.0f} mm)")
    restore_defaults()


def test_parallax_matters():
    print("\nPARALLAX  (two-plane homography vs single ground plane)")
    restore_defaults()
    from sim.sensors.camera import project, parallax_factor
    from ai.calibration import Calibration
    import importlib
    import ai.calibration
    importlib.reload(ai.calibration)
    cal = ai.calibration.Calibration()

    # A marker 1 m off-centre, seen from 2.5 m up, sitting 120 mm high.
    true_xy = (1.0, 0.4)
    px = project(true_xy, config.MARKER_HEIGHT_M)

    config.USE_TWO_PLANE_HOMOGRAPHY = True
    config.USE_UNDISTORT = False
    good = cal.pixel_to_world(px, config.MARKER_HEIGHT_M)
    config.USE_TWO_PLANE_HOMOGRAPHY = False
    bad = cal.pixel_to_world(px, config.MARKER_HEIGHT_M)
    config.USE_TWO_PLANE_HOMOGRAPHY = True

    e_good = math.hypot(good[0] - true_xy[0], good[1] - true_xy[1])
    e_bad = math.hypot(bad[0] - true_xy[0], bad[1] - true_xy[1])

    check("two-plane correction is exact", e_good < 1e-9,
          f"{e_good*1000:.6f} mm")
    check("single-plane error is large", e_bad > 0.04,
          f"{e_bad*1000:.1f} mm error if you use one homography")
    check("ball plane differs from marker plane",
          abs(parallax_factor(config.MARKER_HEIGHT_M)
              - parallax_factor(config.BALL_RADIUS_M)) > 0.03)


def test_heading_noise_model():
    print("\nHEADING FROM TWO BLOBS  (noise amplification)")
    restore_defaults()
    import importlib, ai.calibration
    importlib.reload(ai.calibration)
    cal = ai.calibration.Calibration()

    sigma_pos = cal.position_sigma()
    sigma_th = cal.heading_sigma()
    expected = sigma_pos * math.sqrt(2.0) / config.MARKER_DOT_SEPARATION_M
    check("heading sigma follows 1/separation", abs(sigma_th - expected) < 1e-12,
          f"{math.degrees(sigma_th):.2f} deg from {sigma_pos*1000:.2f} mm "
          f"over {config.MARKER_DOT_SEPARATION_M*1000:.0f} mm")

    # Doubling the marker separation must halve the heading noise.
    old = config.MARKER_DOT_SEPARATION_M
    config.MARKER_DOT_SEPARATION_M = old * 2
    cal2 = ai.calibration.Calibration()
    check("wider markers halve heading noise",
          abs(cal2.heading_sigma() - sigma_th / 2) < 1e-12,
          f"{math.degrees(cal2.heading_sigma()):.2f} deg")
    config.MARKER_DOT_SEPARATION_M = old


def test_latency_identification():
    print("\nLATENCY IDENTIFICATION  (system ID of our own hardware)")
    restore_defaults()
    config.AUTO_LATENCY_ID = True
    import importlib, ai.calibration, ai.estimator
    importlib.reload(ai.calibration)
    importlib.reload(ai.estimator)

    from ai.calibration import LatencyIdentifier

    # Synthetic: a command signal, and an observation that is the same
    # signal delayed by a known amount.
    true_lag = 0.036
    lid = LatencyIdentifier()
    t = 0.0
    while t < 3.0:
        cmd = math.sin(t * 5.0) + 0.6 * math.sin(t * 1.7)
        lid.push_command(t, cmd)
        obs_t = t
        past = obs_t - true_lag
        obs = math.sin(past * 5.0) + 0.6 * math.sin(past * 1.7)
        lid.push_observation(obs_t, obs)
        t += 0.01
    for _ in range(40):
        est = lid.update()
    check("recovers a known lag", abs(est - true_lag) < 0.006,
          f"estimated {est*1000:.1f} ms, true {true_lag*1000:.1f} ms")

    # With a constant command there is nothing to identify; it must hold
    # rather than wander.
    lid2 = LatencyIdentifier()
    t = 0.0
    while t < 3.0:
        lid2.push_command(t, 1.0)
        lid2.push_observation(t, 1.0)
        t += 0.01
    before = lid2.estimate
    for _ in range(20):
        lid2.update()
    check("holds steady without excitation",
          abs(lid2.estimate - before) < 1e-9)


def test_full_noise_performance():
    print("\nFULL NOISE  (how good is the belief in realistic conditions?)")
    restore_defaults()
    import importlib, ai.calibration, ai.estimator
    importlib.reload(ai.calibration)
    importlib.reload(ai.estimator)

    m, agent = run_match(20.0)
    err = agent.error_summary()
    print(f"       ball {err['ball']*1000:6.2f} mm   "
          f"self {err['self']*1000:6.2f} mm   "
          f"opp {err['opp']*1000:6.2f} mm")
    check("ball belief usable under full noise", err["ball"] < 0.030,
          f"{err['ball']*1000:.1f} mm")
    check("self belief usable under full noise", err["self"] < 0.030,
          f"{err['self']*1000:.1f} mm")
    check("opponent belief usable under full noise", err["opp"] < 0.030,
          f"{err['opp']*1000:.1f} mm")
    check("latency estimate is sane",
          0.0 < agent.estimator.total_latency < 0.12,
          f"{agent.estimator.total_latency*1000:.1f} ms "
          f"(nominal {config.total_loop_latency_s()*1000:.1f} ms)")


def main():
    for fn in (test_zero_noise, test_parallax_matters,
               test_heading_noise_model, test_latency_identification,
               test_full_noise_performance):
        fn()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED: {', '.join(FAILURES)}")
        return 1
    print("all estimator tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
