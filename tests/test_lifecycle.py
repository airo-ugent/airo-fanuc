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

from airo_fanuc import controller_facts as cf
from airo_fanuc._core import FaultReason, Mode, MotionStatus, generate_capture_path
from airo_fanuc.config import DriverConfig, MotionResult
from airo_fanuc.lifecycle import (
    ARM_FAULTS,
    OPERATOR_REQUIRED_HINT,
    LifecycleState,
    classify,
    fault_reason_string,
    motion_result_of,
    needs_rmi_ladder,
    operator_hint_for,
    requires_arm,
)

# --------------------------------------------------------------------------- #
# DriverConfig limits are single-sourced from controller_facts: the kinematic
# envelope lives in exactly one place, so a config default can never drift from
# the datasheet/measured facts it is derived from.
# --------------------------------------------------------------------------- #


def test_driver_config_limits_match_controller_facts() -> None:
    cfg = DriverConfig()
    assert np.array_equal(cfg.velocity_limits, cf.CRX10IAL_VELOCITY_LIMITS)
    assert np.array_equal(cfg.acceleration_limits, cf.CRX10IAL_ACCELERATION_LIMITS)
    assert np.array_equal(cfg.jerk_limits, cf.CRX10IAL_JERK_LIMITS)
    assert cfg.stop_scale_va == cf.STOP_LIMIT_SCALE_VA
    assert cfg.stop_scale_j == cf.STOP_LIMIT_SCALE_J
    assert cfg.slew_factor == cf.SLEW_FACTOR
    assert cfg.tracking_lag_s == cf.INTERIM_FACTS.tracking_lag_s


def test_to_rt_core_config_maps_protocol_and_hygiene() -> None:
    cfg = DriverConfig(sm_version=4, rt_priority=80, sched_fifo=True, mlock=True)
    rc = cfg.to_rt_core_config()
    assert rc.sm_version == 4
    assert rc.rt_priority == 80
    assert rc.sched_fifo is True
    assert rc.mlock is True


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


def test_capture_path_bad_length_raises() -> None:
    with pytest.raises((ValueError, RuntimeError)):
        generate_capture_path([0.0, 0.0], _ZERO6, _ZERO6, _ZERO6)


# --------------------------------------------------------------------------- #
# The interpolation period is a configured quantity, and the CAPTURE check must be
# synthesized with the SAME config the core runs, or it validates a path the core
# will not execute.
# --------------------------------------------------------------------------- #


def test_itp_s_maps_into_rt_core_config() -> None:
    assert DriverConfig().itp_s == cf.ITP_S
    assert DriverConfig().to_rt_core_config().itp_s == pytest.approx(cf.ITP_S)
    rc = DriverConfig(itp_s=0.004).to_rt_core_config()
    assert rc.itp_s == pytest.approx(0.004)
    # The drift guard's lag window is a tick count, so it must be derived from the
    # configured period rather than a module constant: halving the period doubles it.
    assert rc.drift_lag_ticks == 2 * DriverConfig().to_rt_core_config().drift_lag_ticks


def test_capture_path_omitted_config_equals_default_config() -> None:
    q_cmd = [0.05, 0, 0, 0, 0, 0]
    assert generate_capture_path(q_cmd, _ZERO6, _ZERO6, _ZERO6) == generate_capture_path(
        q_cmd, _ZERO6, _ZERO6, _ZERO6, DriverConfig().to_rt_core_config()
    )


def test_capture_path_honours_the_configured_itp() -> None:
    """A different period must change the synthesized splice.

    This is the guard that the config is actually threaded through: if the binding
    ignored it and used the shipped defaults, both calls would return identical knots
    and a caller running a non-default period would be collision-checking a path the
    core never executes.
    """
    q_cmd = [0.05, 0, 0, 0, 0, 0]
    default = generate_capture_path(q_cmd, _ZERO6, _ZERO6, _ZERO6, DriverConfig().to_rt_core_config())
    halved = generate_capture_path(
        q_cmd, _ZERO6, _ZERO6, _ZERO6, DriverConfig(itp_s=cf.ITP_S / 2).to_rt_core_config()
    )
    # Knots are ITP-spaced, so halving the period roughly doubles their number.
    assert int(halved["count"]) > int(default["count"])
