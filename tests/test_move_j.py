# SPDX-License-Identifier: Apache-2.0
"""``move_j`` — the point-to-point joint move: offline Ruckig plan → ``move_trajectory``.

Two layers, tested separately because they fail differently.

The PLANNER (:func:`airo_fanuc._core.plan_joint_move`) is pure: given a start state, a
target and the arm's limits it returns knots. Everything worth knowing about it is a
property of those knots, so the tests here assert the properties directly rather than
pinning numbers — that the timeline is a legal ``move_trajectory`` argument, that the
endpoints are exact, that the leading-axis speed means what it says, that every joint
lands at the same instant, and that the profile stays inside the SCALED acceleration
and jerk envelope. That last check reconstructs a/j by finite differences, which is an
exact bound in continuous time and a near-exact one here; ``_A_FD_RTOL`` / ``_J_FD_RTOL``
below carry the derivation of the slack.

The DRIVER method is then thin, and its tests are about the seams: that the plan
actually arrives on a (fake) robot, that the guards fire with typed errors, and that
the universal preempt still works mid-move.
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
import pytest
from conftest import TEST_PROFILE
from test_driver import DriverRig  # noqa: F401  (the `rig` fixture below builds one)

from airo_fanuc import DriverConfig, MotionResult, TrajectoryValidationError, _core
from airo_fanuc import controller_facts as cf

_NDOF = 6
_ITP_NS = int(round(cf.ITP_S * 1e9))
_A_SCALE = cf.MOVEJ_LIMIT_SCALE_A
_J_SCALE = cf.MOVEJ_LIMIT_SCALE_J
#: Slowest joint's velocity limit — the ceiling on a leading-axis speed.
_V_MIN = float(np.min(TEST_PROFILE.velocity_limits))
_DEFAULT_SPEED = _V_MIN * cf.MOVEJ_DEFAULT_SPEED_FRACTION


def _rt_cfg() -> Any:
    """The RtCoreConfig a driver on TEST_PROFILE runs with — the same struct the plan
    must be shaped by, since the tick engine clamps against the limits it carries."""
    return DriverConfig(profile=TEST_PROFILE).to_rt_core_config()


def _plan(
    q_target: list[float],
    *,
    q0: list[float] | None = None,
    qd0: list[float] | None = None,
    speed: float = _DEFAULT_SPEED,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    raw = _core.plan_joint_move(
        q0 if q0 is not None else [0.0] * _NDOF,
        qd0 if qd0 is not None else [0.0] * _NDOF,
        q_target,
        _rt_cfg(),
        max_velocity_rad_s=speed,
        accel_scale=_A_SCALE,
        jerk_scale=_J_SCALE,
    )
    return (
        np.asarray(raw["times_ns"], dtype=np.int64),
        np.asarray(raw["q"], dtype=np.float64),
        np.asarray(raw["qd"], dtype=np.float64),
        raw,
    )


# --------------------------------------------------------------------------- #
# Planner: the knots are a legal move_trajectory argument
# --------------------------------------------------------------------------- #


def test_plan_is_a_valid_move_trajectory_timeline() -> None:
    t, q, qd, raw = _plan([0.6, -0.4, 0.3, 0.0, 0.2, -0.1])

    assert len(t) >= 2, "move_trajectory needs at least two knots"
    assert q.shape == (len(t), _NDOF) and qd.shape == q.shape
    assert int(raw["count"]) == len(t)
    assert np.all(np.diff(t) > 0), "times must be strictly increasing"
    assert t[0] == 0, "times are relative to the start of the motion"
    assert np.all(np.isfinite(q)) and np.all(np.isfinite(qd))
    # Sampled at the interpolation period so the core's cubic-Hermite playback is
    # reproducing the planned profile rather than inventing one between distant knots.
    assert np.all(np.diff(t) <= _ITP_NS)
    assert int(t[-1]) == pytest.approx(raw["duration_s"] * 1e9, rel=1e-9)


def test_plan_endpoints_are_exact() -> None:
    q0 = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6]
    target = [0.6, -0.4, 0.3, 0.0, 0.2, -0.1]
    _, q, qd, _ = _plan(target, q0=q0)

    np.testing.assert_allclose(q[0], q0, atol=1e-12)
    np.testing.assert_allclose(qd[0], 0.0, atol=1e-12)
    # Pinned, not merely converged: the settle tolerance is measured against the pose
    # the caller named, so the last knot states it exactly.
    np.testing.assert_array_equal(q[-1], np.asarray(target))
    np.testing.assert_array_equal(qd[-1], np.zeros(_NDOF))


def test_plan_never_exceeds_the_arms_velocity_limits() -> None:
    """With no leading-axis cap the profile is bounded by the arm's own limits."""
    raw = _core.plan_joint_move(
        [0.0] * _NDOF,
        [0.0] * _NDOF,
        [2.0, -2.0, 3.0, 3.0, -3.0, 3.0],
        _rt_cfg(),
        max_velocity_rad_s=0.0,  # "use the config's velocity limits"
        accel_scale=_A_SCALE,
        jerk_scale=_J_SCALE,
    )
    qd = np.asarray(raw["qd"], dtype=np.float64)
    assert np.all(np.abs(qd) <= TEST_PROFILE.velocity_limits + 1e-9)


# --------------------------------------------------------------------------- #
# Planner: joint_speed is the LEADING-AXIS speed
# --------------------------------------------------------------------------- #


def test_leading_axis_reaches_the_requested_speed_and_no_joint_exceeds_it() -> None:
    speed = 0.4
    # J2 travels furthest, so J2 is the leading axis.
    _, _, qd, _ = _plan([0.2, -1.2, 0.3, 0.0, 0.0, 0.0], speed=speed)

    peak = np.max(np.abs(qd), axis=0)
    assert np.all(peak <= speed + 1e-9), f"no joint may exceed the leading-axis speed: {peak}"
    assert peak[1] == pytest.approx(speed, rel=1e-6), "the furthest-travelling joint runs at it"


def test_all_joints_land_together() -> None:
    """Time-synchronization: the short joints slow down, they do not finish early.

    This is what makes a scalar `joint_speed` mean "leading axis" — without it a
    two-joint move would be two independent moves that happen to share a start.
    """
    target = np.array([0.2, -1.2, 0.05, 0.0, 0.0, 0.0])
    t, q, _, _ = _plan(target.tolist(), speed=0.4)

    moving = np.flatnonzero(np.abs(target) > 1e-9)
    duration_ns = int(t[-1])
    for j in moving:
        # First index at which this joint is within 1% of its own travel of the target.
        remaining = np.abs(q[:, j] - target[j])
        arrived = int(np.argmax(remaining <= 0.01 * abs(target[j])))
        assert t[arrived] > 0.9 * duration_ns, (
            f"joint {j} arrived at {t[arrived] / duration_ns:.2%} of the move — not synchronized"
        )


def test_halving_the_speed_roughly_doubles_the_duration() -> None:
    """A long, cruise-dominated move: duration is set by joint_speed, not by the ramps."""
    target = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
    _, _, _, fast = _plan(target, speed=0.8)
    _, _, _, slow = _plan(target, speed=0.4)
    ratio = float(slow["duration_s"]) / float(fast["duration_s"])
    assert 1.8 < ratio < 2.0, f"expected ~2x, got {ratio:.3f}"


# --------------------------------------------------------------------------- #
# Planner: the profile stays inside the SCALED envelope
# --------------------------------------------------------------------------- #


#: Tolerances for the finite-difference envelope check below.
#:
#: In continuous time these bounds are exact: forward-differencing a piecewise-quadratic
#: velocity over an interval yields the MEAN acceleration there, which cannot exceed its
#: peak, and the same argument one derivative up bounds the jerk. What loosens them is
#: that the knots carry int64 NANOSECOND stamps, so each `dt` is quantized by up to
#: 0.5 ns out of ~8 ms — 6e-8 relative. One difference inherits that. The second divides
#: a ~j*dt difference (two orders smaller than the accelerations it came from) by `dt`
#: again, which amplifies it ~40x. Hence the looser jerk figure; both stay four orders
#: below any real envelope violation.
_A_FD_RTOL = 1e-6
_J_FD_RTOL = 1e-4


def test_profile_respects_the_scaled_acceleration_and_jerk() -> None:
    """The plan must clear the clamps the tick engine enforces, not sit on them."""
    t, _, qd, _ = _plan([1.5, -1.5, 2.0, 1.0, -1.0, 1.0], speed=_V_MIN)
    dt = np.diff(t).astype(np.float64) / 1e9

    a = np.diff(qd, axis=0) / dt[:, None]
    j = np.diff(a, axis=0) / dt[:-1, None]

    a_cap = _A_SCALE * TEST_PROFILE.acceleration_limits
    j_cap = _J_SCALE * TEST_PROFILE.jerk_limits
    assert np.all(np.abs(a) <= a_cap * (1 + _A_FD_RTOL)), f"peak |a| {np.abs(a).max(axis=0)} vs {a_cap}"
    assert np.all(np.abs(j) <= j_cap * (1 + _J_FD_RTOL)), f"peak |j| {np.abs(j).max(axis=0)} vs {j_cap}"
    # And it is genuinely using the envelope it was given, not crawling.
    assert np.abs(a).max() > 0.25 * a_cap.max()


def test_the_planning_envelope_stays_clear_of_the_clamps_the_core_enforces() -> None:
    """The point of scaling: the RT core REFUSES a command past the profile's limits,
    so a plan shaped at those limits is one ordinary slop gets clipped against. Assert
    the headroom exists rather than trusting the constants stay below 1.0."""
    assert 0.0 < _A_SCALE < 1.0
    assert 0.0 < _J_SCALE < _A_SCALE, "jerk is the phantom-contact trigger; scale it harder"


def test_plan_is_shaped_by_the_config_it_is_given() -> None:
    """Omitting the config plans under the synthetic C++ fallback envelope, not the
    arm's — the same trap `generate_capture_path` documents. Prove the config binds."""
    target = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    with_cfg = _core.plan_joint_move([0.0] * _NDOF, [0.0] * _NDOF, target, _rt_cfg(), 0.0, _A_SCALE, _J_SCALE)
    without = _core.plan_joint_move([0.0] * _NDOF, [0.0] * _NDOF, target, None, 0.0, _A_SCALE, _J_SCALE)
    assert with_cfg["duration_s"] != without["duration_s"]


# --------------------------------------------------------------------------- #
# Planner: edges
# --------------------------------------------------------------------------- #


def test_already_at_the_target_still_yields_a_legal_two_knot_plan() -> None:
    """The case airo-robots hits on every trajectory: `execute_trajectory` ends with a
    move to the configuration it just reached. Ruckig returns a zero-duration plan;
    move_trajectory still needs two strictly-increasing knots."""
    q0 = [0.3] * _NDOF
    t, q, qd, raw = _plan(q0, q0=q0)

    assert len(t) == 2
    assert t[0] == 0 and t[1] == _ITP_NS, "a sub-tick plan is stretched over one ITP"
    np.testing.assert_allclose(q[0], q0, atol=1e-12)
    np.testing.assert_array_equal(q[-1], np.asarray(q0))
    np.testing.assert_array_equal(qd[-1], np.zeros(_NDOF))
    assert float(raw["duration_s"]) == pytest.approx(cf.ITP_S)


def test_a_moving_start_is_carried_into_the_first_knot() -> None:
    """A MoveJ issued while the arm still coasts plans a continuation, not a restart —
    the first knot states the velocity the arm actually has."""
    qd0 = [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]
    _, _, qd, _ = _plan([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], qd0=qd0, speed=_DEFAULT_SPEED)
    np.testing.assert_allclose(qd[0], qd0, atol=1e-12)


def test_a_start_faster_than_the_requested_speed_decelerates_instead_of_failing() -> None:
    """The requested speed is a target, not a precondition. Ruckig needs a ceiling it
    can honour at t=0, so a fast start is planned down into the requested speed."""
    qd0 = [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]  # faster than the 0.05 rad/s asked for
    _, _, qd, _ = _plan([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], qd0=qd0, speed=0.05)
    assert qd[0][2] == pytest.approx(0.25)
    assert abs(qd[-1][2]) < 1e-12
    # It settles into the requested speed rather than cruising at the entry speed.
    assert np.abs(qd[len(qd) // 2 :, 2]).max() <= 0.05 + 1e-6


@pytest.mark.parametrize(
    "kwargs",
    [
        {"accel_scale": 0.0},
        {"jerk_scale": -1.0},
        {"accel_scale": float("nan")},
        {"accel_scale": 1.01},
        {"jerk_scale": 2.0},
        {"accel_scale": float("inf")},
    ],
)
def test_planner_rejects_scales_outside_the_unit_interval(kwargs: dict[str, float]) -> None:
    """1.0 is the ceiling, not merely the default.

    The scales multiply the config's a/j limits, the core replays the knots without
    re-timing them, and its only per-tick clip on the command is positional — so a scale
    above 1.0 puts a profile past the arm's acceleration on the wire with nothing left
    to reshape it.
    """
    args = {"accel_scale": _A_SCALE, "jerk_scale": _J_SCALE, **kwargs}
    with pytest.raises(ValueError, match=r"must be in \(0, 1\]"):
        _core.plan_joint_move([0.0] * _NDOF, [0.0] * _NDOF, [0.1] * _NDOF, _rt_cfg(), 0.5, **args)


def test_planner_rejects_a_wrong_length_vector() -> None:
    with pytest.raises(ValueError, match="q_target must have length 6"):
        _core.plan_joint_move([0.0] * _NDOF, [0.0] * _NDOF, [0.1, 0.2], _rt_cfg())


# --------------------------------------------------------------------------- #
# Driver: move_j on a (fake) robot
# --------------------------------------------------------------------------- #


@pytest.fixture
def rig(tmp_path: Any) -> Any:
    r = DriverRig(tmp_path)
    try:
        yield r
    finally:
        r.close()


def _q_cmd(rig: DriverRig) -> np.ndarray:
    return np.asarray(rig.driver.get_state()["q_cmd"], dtype=np.float64)


def test_move_j_reaches_the_target(rig: DriverRig) -> None:
    target = _q_cmd(rig)
    target[0] += 0.4
    target[3] -= 0.2

    handle = rig.driver.move_j(target, joint_speed=0.5)
    assert handle.wait(timeout=8.0) == MotionResult.DONE

    reached = np.asarray(rig.driver.get_state()["q_meas"], dtype=np.float64)
    tol = math.radians(rig.driver._policy.settle.tol_deg)  # noqa: SLF001 - assert on the real criterion
    assert np.all(np.abs(reached - target) <= tol), f"{np.degrees(reached - target)}° off"


def test_move_j_to_the_current_pose_completes(rig: DriverRig) -> None:
    """airo-robots' `execute_trajectory` finishes with exactly this call."""
    handle = rig.driver.move_j(_q_cmd(rig))
    assert handle.wait(timeout=6.0) == MotionResult.DONE


def test_move_j_uses_the_default_speed_when_none_is_given(rig: DriverRig) -> None:
    """The default is a quarter of the slowest joint, matching airo-robots — assert it
    through the only externally visible consequence, the duration."""
    target = _q_cmd(rig)
    target[0] += 0.5

    t0 = time.monotonic()
    assert rig.driver.move_j(target).wait(timeout=10.0) == MotionResult.DONE
    elapsed = time.monotonic() - t0

    # 0.5 rad at the default speed, plus the ramps and the settle.
    ideal = 0.5 / _DEFAULT_SPEED
    assert ideal < elapsed < ideal + 3.0, f"{elapsed:.2f}s for an ideal {ideal:.2f}s cruise"


def test_move_j_slower_speed_takes_longer(rig: DriverRig) -> None:
    target = _q_cmd(rig)
    target[0] += 0.4

    t0 = time.monotonic()
    assert rig.driver.move_j(target, joint_speed=0.8).wait(timeout=10.0) == MotionResult.DONE
    fast = time.monotonic() - t0

    back = _q_cmd(rig)
    back[0] -= 0.4
    t0 = time.monotonic()
    assert rig.driver.move_j(back, joint_speed=0.2).wait(timeout=15.0) == MotionResult.DONE
    slow = time.monotonic() - t0

    assert slow > fast * 1.5, f"slow={slow:.2f}s fast={fast:.2f}s"


def test_stop_j_during_move_j_resolves_stopped(rig: DriverRig) -> None:
    target = _q_cmd(rig)
    target[0] += 1.0

    handle = rig.driver.move_j(target, joint_speed=0.2, asynchronous=True)
    time.sleep(0.4)
    rig.driver.stop_j()
    assert handle.wait(timeout=6.0) == MotionResult.STOPPED


# --------------------------------------------------------------------------- #
# Driver: the guards, each with its own typed error
# --------------------------------------------------------------------------- #


def test_move_j_rejects_a_target_outside_the_position_limits(rig: DriverRig) -> None:
    target = _q_cmd(rig)
    target[3] = TEST_PROFILE.position_limits_upper[3] + 0.2

    # A silent clamp here would report DONE somewhere other than where it was asked.
    with pytest.raises(TrajectoryValidationError, match=r"position limits on joint\(s\) \[3\]"):
        rig.driver.move_j(target)


def test_move_j_rejects_a_speed_above_the_slowest_joints_limit(rig: DriverRig) -> None:
    with pytest.raises(TrajectoryValidationError, match="LEADING-AXIS speed"):
        rig.driver.move_j(_q_cmd(rig), joint_speed=_V_MIN * 1.01)


@pytest.mark.parametrize("speed", [0.0, -0.5, float("inf")])
def test_move_j_rejects_a_degenerate_speed(rig: DriverRig, speed: float) -> None:
    with pytest.raises(TrajectoryValidationError, match="must be finite and > 0"):
        rig.driver.move_j(_q_cmd(rig), joint_speed=speed)


@pytest.mark.parametrize(
    ("bad", "match"),
    [
        ([0.0, 0.0, 0.0], "expects 6 joint values"),
        ([0.0, 0.0, float("nan"), 0.0, 0.0, 0.0], "not finite"),
    ],
)
def test_move_j_rejects_a_malformed_target(rig: DriverRig, bad: list[float], match: str) -> None:
    with pytest.raises(TrajectoryValidationError, match=match):
        rig.driver.move_j(bad)


@pytest.mark.timeout(30)
def test_move_j_refuses_to_start_faster_than_the_capture_envelope(rig: DriverRig) -> None:
    """The capture splice bridges the commanded pose to the plan's first knot and
    cannot reach an arbitrary velocity, so a MoveJ issued mid-flight is a typed
    refusal naming the remedy — never a REJECTED motion the caller has to decode."""
    start = _q_cmd(rig)
    target = start.copy()
    target[0] += 1.0
    # A 1 rad move in 1 s peaks near 1.5 rad/s, far above the 15°/s envelope.
    times = [0, 1_000_000_000]
    q = [start.tolist(), target.tolist()]
    qd = [[0.0] * _NDOF, [0.0] * _NDOF]
    handle = rig.driver.move_trajectory(times, q, qd, asynchronous=True)

    capture_rate = math.radians(cf.CAPTURE_RATE_DEG_S)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if np.max(np.abs(_q_cmd_velocity(rig))) > capture_rate:
            break
        time.sleep(0.005)
    else:  # pragma: no cover - the fake would have to stall for this
        pytest.fail("the in-flight trajectory never exceeded the capture envelope")

    with pytest.raises(TrajectoryValidationError, match="capture envelope"):
        rig.driver.move_j(target)

    rig.driver.stop_j()
    handle.wait(timeout=6.0)


def _q_cmd_velocity(rig: DriverRig) -> np.ndarray:
    return np.asarray(rig.driver.get_state()["qd_cmd"], dtype=np.float64)
