# SPDX-License-Identifier: Apache-2.0
"""Pure lifecycle state-machine helpers (no threads, no I/O).

The lifecycle *policy* lives in :class:`airo_fanuc.supervisor.Supervisor`; this
module holds the small, deterministic, unit-testable pieces it composes:

* :class:`LifecycleState` — the mission states.
* :func:`classify` — the observed lifecycle state for a ``(mode, fault, rmi_down)``
  tuple (LOST is a ``classify`` output gated on ``rmi_down``). The supervisor
  overrides the result with RECOVERING / SHUTTING_DOWN when it knows more than the
  snapshot alone.
* :func:`motion_result_of` — terminal :class:`airo_fanuc._core.MotionStatus` →
  :class:`airo_fanuc.config.MotionResult` (``None`` for the non-terminal
  PENDING/RUNNING).
* fault classification (``needs_rmi_ladder`` / ``requires_arm``), the priority
  ordering that turns the C++ condition bitmask into an ordered ``fault_reason``,
  and the operator-hint catalog (one actionable teach-pendant instruction per
  fault).

Kept import-light on purpose (only the ``_core`` enums + :class:`MotionResult`) so
``tests/test_lifecycle.py`` can exercise every branch without a controller.
"""

from __future__ import annotations

from enum import Enum

from ._core import FaultReason, Mode, MotionStatus
from .config import MotionResult

__all__ = [
    "ARM_FAULTS",
    "DEGRADED_FAULTS",
    "FAULTED_FAULTS",
    "FAULT_STATES",
    "LifecycleState",
    "classify",
    "fault_reason_string",
    "motion_result_of",
    "needs_rmi_ladder",
    "operator_hint_for",
    "requires_arm",
]


class LifecycleState(Enum):
    """Mission states. STREAMING is the only commandable one."""

    DISCONNECTED = "disconnected"
    PREFLIGHT = "preflight"
    RMI_CONNECTING = "rmi_connecting"
    TP_LAUNCH = "tp_launch"
    SM_HANDSHAKE = "sm_handshake"
    STREAMING = "streaming"
    DEGRADED = "degraded"
    FAULTED = "faulted"
    RECOVERING = "recovering"
    LOST = "lost"
    SHUTTING_DOWN = "shutting_down"


#: States in which the robot is stopped or held by a controller-side condition, as
#: opposed to a bring-up step or a clean shutdown. The single definition of "faulted"
#: for anything outside the state machine — the supervisor reports it in its snapshot
#: and the republisher publishes it, so no consumer re-derives it from a fault-reason
#: string (``fault_reason`` is ``"none"``, not empty, when nothing is wrong).
#: State-level: the FaultReason sets below classify *reasons*, this classifies the
#: *state* those reasons produce, and DEGRADED belongs here because motion is refused
#: there too even though the controller manages the decel itself.
FAULT_STATES: frozenset[LifecycleState] = frozenset(
    {LifecycleState.DEGRADED, LifecycleState.FAULTED, LifecycleState.LOST}
)


# --------------------------------------------------------------------------- #
# Fault classification.
# --------------------------------------------------------------------------- #

#: Hard faults → FAULTED (need the RMI reset/relaunch ladder or SM re-handshake).
FAULTED_FAULTS: frozenset[FaultReason] = frozenset(
    {
        FaultReason.E_STOP,
        FaultReason.IN_ERROR,
        FaultReason.MOTION_NOT_POSSIBLE,
        FaultReason.RX_SILENT,
        FaultReason.RX_DEGRADED,
        FaultReason.WATCHDOG_EXPIRED,
        FaultReason.FORCE_GUARD,
        FaultReason.SUPERVISOR_LOST,
        FaultReason.INTERNAL,
        FaultReason.REJECTED_START_MISMATCH,
    }
)

#: Soft conditions → DEGRADED (controller manages decel; re-arm on condition-exit).
DEGRADED_FAULTS: frozenset[FaultReason] = frozenset(
    {
        FaultReason.TEACH_MODE,
        FaultReason.CONTACT_STOP,
        FaultReason.SAFETY_CLAMP,
    }
)

#: Faults whose recovery ends in MOTION_INHIBITED — an explicit ``arm()`` is
#: required before the next motion. These are the human-in-the-loop faults:
#: someone pressed the E-stop or is standing at the pendant clearing a latched
#: alarm, so resuming motion the instant the fault clears would move the robot
#: with a person inside its envelope. Recovery must therefore hand the decision
#: back to the caller instead of auto-arming. E-stop and the latched in_error
#: that survives an e-stop release are those cases here; SYST-348
#: OPERATOR_REQUIRED arms the same gate via the supervisor's operator flag.
ARM_FAULTS: frozenset[FaultReason] = frozenset(
    {
        FaultReason.E_STOP,
        FaultReason.IN_ERROR,
    }
)


def needs_rmi_ladder(fault: FaultReason) -> bool:
    """True if clearing ``fault`` needs the RMI reset/relaunch ladder (vs a pure
    condition-exit the controller manages)."""
    return fault in FAULTED_FAULTS


def requires_arm(fault: FaultReason) -> bool:
    """True if recovery from ``fault`` must end in MOTION_INHIBITED, so motion
    stays refused until the caller explicitly ``arm()``s."""
    return fault in ARM_FAULTS


# --------------------------------------------------------------------------- #
# Observed-state classification.
# --------------------------------------------------------------------------- #

_STREAMING_MODES: frozenset[Mode] = frozenset(
    {Mode.HOLD, Mode.PREROLL, Mode.CAPTURE, Mode.TRAJECTORY, Mode.SERVO, Mode.BRAKE}
)


def classify(mode: Mode, fault: FaultReason, *, rmi_down: bool = False) -> LifecycleState:
    """Observed lifecycle state for a core ``(mode, fault)`` snapshot.

    Does NOT know about RECOVERING/SHUTTING_DOWN (supervisor-driven) — those are
    layered on by the caller. ``rmi_down`` promotes an RX-silent fault to LOST
    (RMI dead ∧ rx silent).
    """
    if mode == Mode.RX_SILENT or fault == FaultReason.RX_SILENT:
        return LifecycleState.LOST if rmi_down else LifecycleState.FAULTED
    if mode == Mode.STREAM_DOWN:
        return LifecycleState.LOST if rmi_down else LifecycleState.DISCONNECTED
    if mode == Mode.SAFE_FOLLOW or fault != FaultReason.NONE:
        if fault in DEGRADED_FAULTS:
            return LifecycleState.DEGRADED
        return LifecycleState.FAULTED
    if mode in _STREAMING_MODES:
        return LifecycleState.STREAMING
    return LifecycleState.DISCONNECTED


# --------------------------------------------------------------------------- #
# Motion result mapping.
# --------------------------------------------------------------------------- #

_MOTION_RESULT: dict[MotionStatus, MotionResult] = {
    MotionStatus.DONE: MotionResult.DONE,
    MotionStatus.SETTLE_TIMEOUT: MotionResult.SETTLE_TIMEOUT,
    MotionStatus.STOPPED: MotionResult.STOPPED,
    MotionStatus.PREEMPTED: MotionResult.PREEMPTED,
    MotionStatus.FAULTED: MotionResult.FAULTED,
    MotionStatus.REJECTED: MotionResult.REJECTED,
}


def motion_result_of(status: MotionStatus) -> MotionResult | None:
    """Terminal :class:`MotionStatus` → :class:`MotionResult`; ``None`` while the
    motion is still PENDING/RUNNING (not yet terminal)."""
    return _MOTION_RESULT.get(status)


# --------------------------------------------------------------------------- #
# Human-facing strings: for each fault, the action that clears it at the pendant.
# --------------------------------------------------------------------------- #

_FAULT_REASON_STRING: dict[FaultReason, str] = {
    FaultReason.NONE: "none",
    FaultReason.E_STOP: "e_stop",
    FaultReason.IN_ERROR: "in_error",
    FaultReason.MOTION_NOT_POSSIBLE: "motion_not_possible",
    FaultReason.TEACH_MODE: "teach_mode",
    FaultReason.CONTACT_STOP: "contact_stop",
    FaultReason.SAFETY_CLAMP: "safety_clamp",
    FaultReason.RX_SILENT: "rx_silent",
    FaultReason.RX_DEGRADED: "rx_degraded",
    FaultReason.WATCHDOG_EXPIRED: "watchdog_expired",
    FaultReason.FORCE_GUARD: "force_guard",
    FaultReason.REJECTED_START_MISMATCH: "rejected_start_mismatch",
    FaultReason.SUPERVISOR_LOST: "supervisor_lost",
    FaultReason.INTERNAL: "internal",
}

_OPERATOR_HINT: dict[FaultReason, str] = {
    FaultReason.E_STOP: "Release the E-stop; then press RESET on the TP if the fault persists.",
    FaultReason.IN_ERROR: "Press RESET on the teach pendant to clear the latched alarm.",
    FaultReason.MOTION_NOT_POSSIBLE: "Motion not possible — check drives powered / active alarms on the TP.",
    FaultReason.TEACH_MODE: "Keyswitch is in T1/T2 — return it to AUTO to resume.",
    FaultReason.CONTACT_STOP: "Clear the contact stop on the TP / move the robot away from the obstacle.",
    FaultReason.SAFETY_CLAMP: "Press RESET on the TP to release the residual DCS speed clamp.",
    FaultReason.RX_SILENT: "Stream Motion is silent — check the network link / power-cycle the controller.",
    FaultReason.RX_DEGRADED: "Intermittent Stream Motion RX — check the network link and host load.",
    FaultReason.FORCE_GUARD: "Force-guard tripped — clear the contact and replan.",
    FaultReason.WATCHDOG_EXPIRED: "Motion deadman expired — the collision supervisor stopped kicking.",
    FaultReason.SUPERVISOR_LOST: "Driver supervisor thread died — the core is holding; restart the process.",
    FaultReason.INTERNAL: "Internal driver fault — capture the diagnostic dump and report.",
}

#: SYST-348 payload-monitor operator flow. Distinct from the generic IN_ERROR
#: hint because RESET does NOT clear it — payload-confirm does.
OPERATOR_REQUIRED_HINT: str = (
    "Confirm the payload on the teach pendant. Do NOT jog while recovering in AUTO — it re-raises SYST-328."
)


def fault_reason_string(fault: FaultReason) -> str:
    """Short machine-friendly name of a fault reason (topic/log payload)."""
    return _FAULT_REASON_STRING.get(fault, "unknown")


def operator_hint_for(fault: FaultReason, *, operator_required: bool = False) -> str | None:
    """Actionable teach-pendant instruction for a fault, or ``None`` if the fault
    needs no operator action. ``operator_required`` (SYST-348) overrides."""
    if operator_required:
        return OPERATOR_REQUIRED_HINT
    return _OPERATOR_HINT.get(fault)
