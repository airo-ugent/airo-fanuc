# SPDX-License-Identifier: Apache-2.0
"""Fast, controller-free unit tests for the driver's pure layers.

Covers the lifecycle state-machine helpers (:mod:`airo_fanuc.lifecycle`), the
single-sourced :class:`~airo_fanuc.config.DriverConfig` limits, and the CAPTURE
collision-check binding (:func:`airo_fanuc._core.generate_capture_path`) parity —
none of which need a running FakeCRX, so they stay millisecond-fast.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from conftest import TEST_PROFILE

from airo_fanuc import controller_facts as cf
from airo_fanuc._core import FaultReason, Mode, MotionStatus, RtCoreConfig, generate_capture_path
from airo_fanuc.config import DriverConfig, MotionResult
from airo_fanuc.lifecycle import (
    ARM_FAULTS,
    FAULT_STATES,
    OPERATOR_REQUIRED_HINT,
    LifecycleState,
    classify,
    fault_reason_string,
    motion_result_of,
    needs_rmi_ladder,
    operator_hint_for,
    requires_arm,
)
from airo_fanuc.robot_profile import RobotProfile

#: A second arm, an order of magnitude slower than :data:`TEST_PROFILE`, for the tests
#: that have to show a profile makes a difference downstream rather than merely being
#: stored.
_SLOWER_PROFILE = RobotProfile.from_degrees(
    name="slower",
    model="test",
    velocity_limits_deg_s=[10.0] * 6,
    acceleration_limits_deg_s2=[20.0] * 6,
    jerk_limits_deg_s3=[160.0] * 6,
    position_limits_lower_deg=[-90.0] * 6,
    position_limits_upper_deg=[90.0] * 6,
)

# --------------------------------------------------------------------------- #
# The arm's limits have to REACH the C++ tick engine, which is the only thing that
# enforces them. A profile that stopped at the Python boundary would leave the core
# clamping against its own synthetic fallback envelope while every Python-side check
# passed, so assert the whole path: profile → DriverConfig → RtCoreConfig.
# --------------------------------------------------------------------------- #


def test_profile_limits_reach_the_rt_core_config() -> None:
    rc = DriverConfig(profile=TEST_PROFILE).to_rt_core_config()
    assert np.allclose(rc.velocity_limits, TEST_PROFILE.velocity_limits)
    assert np.allclose(rc.acceleration_limits, TEST_PROFILE.acceleration_limits)
    assert np.allclose(rc.jerk_limits, TEST_PROFILE.jerk_limits)


def test_profile_limits_are_not_the_cpp_fallback_defaults() -> None:
    """Guard the guard: if the fallback happened to equal the profile, the test above
    would pass on a config that never plumbed anything."""
    fallback = RtCoreConfig()
    assert not np.allclose(fallback.velocity_limits, TEST_PROFILE.velocity_limits)


def test_a_different_profile_produces_different_core_limits() -> None:
    rc = DriverConfig(profile=_SLOWER_PROFILE).to_rt_core_config()
    assert np.allclose(rc.velocity_limits, np.radians([10.0] * 6))
    assert np.allclose(rc.jerk_limits, np.radians([160.0] * 6))


def test_brake_scales_and_slew_factor_are_driver_tuning_not_profile() -> None:
    """These scale a profile's limits rather than replacing them, so they come from
    controller_facts and travel across arms unchanged."""
    cfg = DriverConfig(profile=TEST_PROFILE)
    assert cfg.stop_scale_va == cf.STOP_LIMIT_SCALE_VA
    assert cfg.stop_scale_j == cf.STOP_LIMIT_SCALE_J
    assert cfg.slew_factor == cf.SLEW_FACTOR
    rc = cfg.to_rt_core_config()
    assert rc.stop_scale_va == cf.STOP_LIMIT_SCALE_VA
    assert rc.stop_scale_j == cf.STOP_LIMIT_SCALE_J
    assert rc.slew_factor == cf.SLEW_FACTOR


def test_driver_config_requires_a_profile() -> None:
    """No default arm: a DriverConfig without a profile is a driver that would clamp
    against limits nobody chose."""
    with pytest.raises(TypeError, match="profile"):
        DriverConfig()  # type: ignore[call-arg]


def test_to_rt_core_config_maps_protocol_and_hygiene() -> None:
    cfg = DriverConfig(profile=TEST_PROFILE, sm_version=4, rt_priority=80, sched_fifo=True, mlock=True)
    rc = cfg.to_rt_core_config()
    assert rc.sm_version == 4
    assert rc.rt_priority == 80
    assert rc.sched_fifo is True
    assert rc.mlock is True


def test_the_gate_that_executes_uses_the_constants_the_refusal_is_written_in() -> None:
    """``move_trajectory`` refuses a submission against the ``controller_facts`` capture
    window and splice envelope, so the core has to gate on the same two numbers.

    Left as C++-only defaults these were a second, independently editable copy: editing
    ``controller_facts`` moved the refusal and the error text but not the gate, and
    "the checked path IS the executed path" stopped holding for them without anything
    failing. ``float(np.deg2rad(...))`` here mirrors how ``driver.py`` builds the
    threshold, because a refusal decided by a different bit pattern than the gate is the
    same bug in miniature.
    """
    rc = DriverConfig(profile=TEST_PROFILE).to_rt_core_config()
    assert rc.capture_rate_rad_s == float(np.deg2rad(cf.CAPTURE_RATE_DEG_S))
    assert rc.capture_tol_rad == float(np.deg2rad(cf.CAPTURE_TOL_DEG))


def test_mirrored_watchdog_windows_are_populated_from_controller_facts() -> None:
    """Every window ``controller_facts`` claims to single-source must actually be sent.

    The C++ defaults agree with these values, which is exactly why omitting them was
    invisible — the mirror comments in ``rt_core_config.hpp`` name a Python symbol that
    was not in fact reaching the core.
    """
    rc = DriverConfig(profile=TEST_PROFILE).to_rt_core_config()
    assert rc.rx_silence_blind_hold_ms == cf.RX_SILENCE_BLIND_HOLD_MS
    assert rc.rx_silence_qd_ramp_ms == cf.RX_SILENCE_QD_RAMP_MS
    assert rc.rx_silent_park_ms == cf.RX_SILENT_PARK_MS
    assert rc.antiflap_dwell_ms == cf.ANTIFLAP_DWELL_MS
    assert rc.max_plan_stale_ms == cf.MAX_PLAN_STALE_MS


def test_stream_rate_is_derived_from_the_interpolation_period() -> None:
    """One fact, one encoding. ``125.0`` written out is a second editable copy of ITP_S."""
    assert cf.STREAM_RATE_HZ == 1.0 / cf.ITP_S


# --------------------------------------------------------------------------- #
# Lifecycle classification: (core Mode, FaultReason) → LifecycleState.
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "mode,fault,expected",
    [
        (Mode.HOLD, FaultReason.NONE, LifecycleState.STREAMING),
        (Mode.TRAJECTORY, FaultReason.NONE, LifecycleState.STREAMING),
        (Mode.BRAKE, FaultReason.NONE, LifecycleState.STREAMING),
        (Mode.SAFE_FOLLOW, FaultReason.E_STOP, LifecycleState.FAULTED),
        (Mode.SAFE_FOLLOW, FaultReason.IN_ERROR, LifecycleState.FAULTED),
        (Mode.SAFE_FOLLOW, FaultReason.MOTION_NOT_POSSIBLE, LifecycleState.FAULTED),
        (Mode.SAFE_FOLLOW, FaultReason.CONTACT_STOP, LifecycleState.DEGRADED),
        (Mode.SAFE_FOLLOW, FaultReason.SAFETY_CLAMP, LifecycleState.DEGRADED),
        (Mode.SAFE_FOLLOW, FaultReason.TEACH_MODE, LifecycleState.DEGRADED),
        (Mode.RX_SILENT, FaultReason.RX_SILENT, LifecycleState.FAULTED),
    ],
)
def test_classify(mode: Mode, fault: FaultReason, expected: LifecycleState) -> None:
    assert classify(mode, fault) == expected


def test_classify_lost_when_rmi_down_and_rx_silent() -> None:
    assert classify(Mode.RX_SILENT, FaultReason.RX_SILENT, rmi_down=True) == LifecycleState.LOST
    assert classify(Mode.STREAM_DOWN, FaultReason.NONE, rmi_down=True) == LifecycleState.LOST


def test_fault_states_cover_every_fault_classification() -> None:
    """FAULT_STATES is what "faulted" means for everything outside the state machine
    (the supervisor snapshot, the republished status). Any observed fault must land in
    it, and no healthy or bring-up state may."""
    for fault in FaultReason.__members__.values():
        if fault == FaultReason.NONE:
            continue
        assert classify(Mode.SAFE_FOLLOW, fault) in FAULT_STATES, fault
    assert classify(Mode.RX_SILENT, FaultReason.RX_SILENT, rmi_down=True) in FAULT_STATES
    assert LifecycleState.STREAMING not in FAULT_STATES
    assert LifecycleState.RECOVERING not in FAULT_STATES
    assert LifecycleState.SHUTTING_DOWN not in FAULT_STATES
    assert LifecycleState.DISCONNECTED not in FAULT_STATES
    # "none" is a fault-reason string, not an absence of one: nothing may classify by
    # testing that field for emptiness.
    assert fault_reason_string(FaultReason.NONE) == "none"


def test_fault_classification_predicates() -> None:
    assert requires_arm(FaultReason.E_STOP)
    assert requires_arm(FaultReason.IN_ERROR)
    assert not requires_arm(FaultReason.CONTACT_STOP)
    assert not requires_arm(FaultReason.SAFETY_CLAMP)
    assert ARM_FAULTS == frozenset({FaultReason.E_STOP, FaultReason.IN_ERROR})
    assert needs_rmi_ladder(FaultReason.E_STOP)
    assert needs_rmi_ladder(FaultReason.MOTION_NOT_POSSIBLE)
    assert not needs_rmi_ladder(FaultReason.CONTACT_STOP)
    assert not needs_rmi_ladder(FaultReason.TEACH_MODE)


def test_motion_result_of() -> None:
    assert motion_result_of(MotionStatus.DONE) == MotionResult.DONE
    assert motion_result_of(MotionStatus.STOPPED) == MotionResult.STOPPED
    assert motion_result_of(MotionStatus.PREEMPTED) == MotionResult.PREEMPTED
    assert motion_result_of(MotionStatus.FAULTED) == MotionResult.FAULTED
    assert motion_result_of(MotionStatus.REJECTED) == MotionResult.REJECTED
    assert motion_result_of(MotionStatus.SETTLE_TIMEOUT) == MotionResult.SETTLE_TIMEOUT
    # Non-terminal → None (wait() keeps polling).
    assert motion_result_of(MotionStatus.PENDING) is None
    assert motion_result_of(MotionStatus.RUNNING) is None


def test_operator_hints() -> None:
    assert operator_hint_for(FaultReason.NONE) is None
    assert operator_hint_for(FaultReason.E_STOP) is not None
    assert "E-stop" in operator_hint_for(FaultReason.E_STOP)  # type: ignore[operator]
    assert operator_hint_for(FaultReason.TEACH_MODE, operator_required=True) == OPERATOR_REQUIRED_HINT
    assert "SYST-328" in OPERATOR_REQUIRED_HINT
    assert fault_reason_string(FaultReason.NONE) == "none"
    assert fault_reason_string(FaultReason.E_STOP) == "e_stop"


# --------------------------------------------------------------------------- #
# CAPTURE collision-check binding parity: the Python binding must return exactly the
# knots capture.hpp would splice in the RT loop, or a host-side collision check would
# be validating a different path than the one the core actually executes.
# --------------------------------------------------------------------------- #

_ZERO6 = [0.0] * 6


def test_capture_path_deterministic() -> None:
    a = generate_capture_path([0.05, 0, 0, 0, 0, 0], _ZERO6, _ZERO6, _ZERO6)
    b = generate_capture_path([0.05, 0, 0, 0, 0, 0], _ZERO6, _ZERO6, _ZERO6)
    assert a["q"] == b["q"] and a["qd"] == b["qd"], "same inputs must give byte-identical output"


def test_capture_path_endpoints_and_finish() -> None:
    # Splice from commanded 3° (0.052 rad) to target 0.
    p = generate_capture_path([0.052, 0, 0, 0, 0, 0], _ZERO6, _ZERO6, _ZERO6)
    assert p["would_reject"] is False
    assert p["finished"] is True and p["overflow"] is False
    count = int(p["count"])
    assert count >= 2
    knots_q = np.asarray(p["q"][:count])
    # knot[0] == commanded (q_cmd); last knot reaches (q0).
    assert knots_q[0][0] == pytest.approx(0.052)
    assert knots_q[-1][0] == pytest.approx(0.0, abs=1e-6)


def test_capture_path_velocity_within_capture_rate() -> None:
    p = generate_capture_path([0.06, 0, 0, 0, 0, 0], _ZERO6, _ZERO6, _ZERO6)
    count = int(p["count"])
    knots_qd = np.asarray(p["qd"][:count])
    # The capture profile is capped at 15°/s per joint (controller_facts.CAPTURE_RATE_DEG_S).
    assert np.max(np.abs(knots_qd)) <= math.radians(cf.CAPTURE_RATE_DEG_S) + 1e-6


def test_capture_would_reject_at_five_degrees() -> None:
    # Just inside 5° → accepted; well beyond → rejected.
    inside = generate_capture_path([math.radians(4.5), 0, 0, 0, 0, 0], _ZERO6, _ZERO6, _ZERO6)
    beyond = generate_capture_path([math.radians(34.0), 0, 0, 0, 0, 0], _ZERO6, _ZERO6, _ZERO6)
    assert inside["would_reject"] is False
    assert beyond["would_reject"] is True


# The capture gate's velocity-shed term, through the binding, plus the typed message built
# from it. The message is asserted here rather than in test_driver because the formatter
# takes the gate's dict and nothing else — which is the property worth pinning: the Python
# side derives no part of the feasibility condition, so nothing on this side can drift from
# tick_engine/capture.cpp.
def test_capture_gate_rejects_a_velocity_the_window_cannot_absorb() -> None:
    from airo_fanuc._core import RtCoreConfig

    cfg = RtCoreConfig()  # the synthetic tick-engine envelope
    v = 0.8  # rad/s commanded on joint 0
    gap = 0.08  # rad, ~4.58 deg — inside the 5 deg window
    p = generate_capture_path([0.0] * 6, [v, 0, 0, 0, 0, 0], [gap, 0, 0, 0, 0, 0], _ZERO6, cfg)
    assert p["would_reject"] is True
    assert p["tol_exceeded"] is False, "the endpoint gap is inside the window"
    assert list(p["reject_joints"]) == [0]

    a_b = cfg.stop_scale_va * cfg.acceleration_limits[0]
    j_b = cfg.stop_scale_j * cfg.jerk_limits[0]
    need = 0.5 * v * (v / a_b + a_b / j_b)  # trapezoidal shed
    assert float(p["shed_travel"][0]) == pytest.approx(need, rel=1e-12)
    assert need > math.radians(cf.CAPTURE_TOL_DEG), "shed travel exceeds the window"


def test_capture_gate_is_vacuous_when_the_endpoint_velocities_match() -> None:
    # Every submission out of HOLD (both velocities zero) and every continuation replan
    # (both equal and non-zero) has nothing to shed, so the gate is the endpoint window
    # alone and behaves exactly as it always did.
    inside = generate_capture_path([math.radians(4.5), 0, 0, 0, 0, 0], _ZERO6, _ZERO6, _ZERO6)
    beyond = generate_capture_path([math.radians(34.0), 0, 0, 0, 0, 0], _ZERO6, _ZERO6, _ZERO6)
    assert inside["would_reject"] is False
    assert beyond["would_reject"] is True and beyond["tol_exceeded"] is True
    for p in (inside, beyond):
        assert list(p["reject_joints"]) == []
        assert list(p["shed_travel"]) == [0.0] * 6

    # A continuation whose target IS the current commanded state, at speed: not refused.
    moving = [0.8, 0, 0, 0, 0, 0]
    same = generate_capture_path([0.1] * 6, moving, [0.1] * 6, moving)
    assert same["would_reject"] is False, "a phase-join submission must not be refused"
    assert list(same["shed_travel"]) == [0.0] * 6


def test_capture_reject_message_names_the_joint_and_the_shed_cost() -> None:
    from airo_fanuc._core import RtCoreConfig
    from airo_fanuc.driver import _capture_reject_message

    cfg = RtCoreConfig()
    v, gap = 0.8, 0.08
    q_cmd, qd_cmd = [0.0] * 6, [v, 0, 0, 0, 0, 0]
    q0, qd0 = [gap, 0, 0, 0, 0, 0], list(_ZERO6)
    p = generate_capture_path(q_cmd, qd_cmd, q0, qd0, cfg)
    msg = _capture_reject_message(p, q_cmd, qd_cmd, q0, qd0)

    assert "joint 0" in msg
    assert f"{math.degrees(v):+.3f}°/s" in msg, "the velocity it must shed"
    assert f"{math.degrees(float(p['shed_travel'][0])):.3f}°" in msg, "what shedding costs"
    assert f"{cf.CAPTURE_TOL_DEG:g}°" in msg, "and the window it is compared against"
    assert "unreachable" in msg and "from rest" in msg, "and what to do about it"


def test_capture_reject_message_for_the_endpoint_window() -> None:
    from airo_fanuc.driver import _capture_reject_message

    q_cmd, q0 = [math.radians(34.0), 0, 0, 0, 0, 0], list(_ZERO6)
    p = generate_capture_path(q_cmd, _ZERO6, q0, _ZERO6)
    msg = _capture_reject_message(p, q_cmd, list(_ZERO6), q0, list(_ZERO6))
    assert "capture window" in msg and "joint 0" in msg
    assert "34.000°" in msg and f"{cf.CAPTURE_TOL_DEG:g}°" in msg


def test_capture_path_bad_length_raises() -> None:
    with pytest.raises((ValueError, RuntimeError)):
        generate_capture_path([0.0, 0.0], _ZERO6, _ZERO6, _ZERO6)


# --------------------------------------------------------------------------- #
# The interpolation period is a configured quantity, and the CAPTURE check must be
# synthesized with the SAME config the core runs, or it validates a path the core
# will not execute.
# --------------------------------------------------------------------------- #


def test_itp_s_maps_into_rt_core_config() -> None:
    assert DriverConfig(profile=TEST_PROFILE).itp_s == cf.ITP_S
    assert DriverConfig(profile=TEST_PROFILE).to_rt_core_config().itp_s == pytest.approx(cf.ITP_S)
    rc = DriverConfig(profile=TEST_PROFILE, itp_s=0.004).to_rt_core_config()
    assert rc.itp_s == pytest.approx(0.004)


def test_capture_path_is_bounded_by_the_configs_arm_limits() -> None:
    """Omitting the config synthesizes under the C++ fallback envelope, not the arm's.

    The driver always passes the RtCoreConfig it built the core from: the splice's
    acceleration and jerk are brake-class fractions of the *profile's* limits, so a
    check run under a different envelope would validate a path the core will not
    execute. One config in, one path out — but not the same path for a different arm.
    """
    q_cmd = [0.05, 0, 0, 0, 0, 0]
    omitted = generate_capture_path(q_cmd, _ZERO6, _ZERO6, _ZERO6)
    slower = generate_capture_path(
        q_cmd, _ZERO6, _ZERO6, _ZERO6, DriverConfig(profile=_SLOWER_PROFILE).to_rt_core_config()
    )
    assert omitted != slower
    # Deterministic for a given config, which is what makes the check meaningful.
    assert slower == generate_capture_path(
        q_cmd, _ZERO6, _ZERO6, _ZERO6, DriverConfig(profile=_SLOWER_PROFILE).to_rt_core_config()
    )


def test_capture_path_honours_the_configured_itp() -> None:
    """A different period must change the synthesized splice.

    This is the guard that the config is actually threaded through: if the binding
    ignored it and used the shipped defaults, both calls would return identical knots
    and a caller running a non-default period would be collision-checking a path the
    core never executes.
    """
    q_cmd = [0.05, 0, 0, 0, 0, 0]
    default = generate_capture_path(
        q_cmd, _ZERO6, _ZERO6, _ZERO6, DriverConfig(profile=TEST_PROFILE).to_rt_core_config()
    )
    half_itp = DriverConfig(profile=TEST_PROFILE, itp_s=cf.ITP_S / 2).to_rt_core_config()
    halved = generate_capture_path(q_cmd, _ZERO6, _ZERO6, _ZERO6, half_itp)
    # Knots are ITP-spaced, so halving the period roughly doubles their number.
    assert int(halved["count"]) > int(default["count"])
