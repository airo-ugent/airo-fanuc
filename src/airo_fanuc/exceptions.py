# SPDX-License-Identifier: Apache-2.0
"""Typed exception hierarchy for the ``airo_fanuc`` driver (PLAN §5.1).

Single-sourced here so every later phase (RmiClient, StreamCore Python layer,
FanucDriver, GripperWorker, ownership lock, FanucReceiveInterface) *imports*
these rather than redefining them — one ``isinstance`` tree across the package.

Two roots on purpose:

* :class:`FanucError` — the driver-owned base. Everything a caller of the
  public :class:`~airo_fanuc.FanucDriver` API might catch derives from it.
* :class:`RmiError` — kept a ``RuntimeError`` **exactly as the ``dries``
  ``rmi_client`` shipped it** (a controller-reported non-zero ErrorID). It is
  deliberately *not* a :class:`FanucError`: it is a low-level RMI-protocol
  signal, and callers that want to treat controller errors uniformly with
  transport/lifecycle errors already special-case it. Retaining the
  ``RuntimeError`` base preserves the ``dries`` catch semantics verbatim
  (``except (RmiError, OSError)`` in the Initialize recovery ladder).

``CommandEpochError`` is intentionally absent — it is DEAD per PLAN decision 9
(replaced by latched-FAULTED typed rejects). Do not add it.
"""

from __future__ import annotations

from typing import Any


class FanucError(Exception):
    """Base class for every driver-owned error (PLAN §5.1).

    Catch this to handle any error the ``airo_fanuc`` driver raises as a
    single family; catch a subclass for a specific failure mode.
    """


class FanucConnectionError(FanucError):
    """Transport/bring-up connection failure.

    Raised when the driver cannot reach the controller at all — TCP connect
    refused/timed out, preflight TCP-reach failure, or a fresh session that
    never establishes. Distinct from :class:`RmiSessionDown`, which means a
    *previously live* session has gone away.
    """


class FanucPreflightError(FanucError):
    """A preflight gate failed (PLAN §5.3 / design doc 08 §9).

    Wraps the structured preflight report (P-level/S636/AUTO-mode/drives/
    active-alarm classification) so the constructor can surface *why* bring-up
    was refused before any motion is attempted.
    """


class RobotFaultedError(FanucError):
    """A motion/command method was called while the robot is not STREAMING.

    Carries the human-facing ``reason`` (the ordered fault_reason string the
    lifecycle SM latched) and an ``operator_hint`` (the actionable TP-side
    instruction, e.g. "Release E-stop; then RESET on the TP").
    """

    def __init__(self, reason: str, operator_hint: str | None = None) -> None:
        self.reason = reason
        self.operator_hint = operator_hint
        msg = reason if operator_hint is None else f"{reason} — {operator_hint}"
        super().__init__(msg)


class RejectedStartMismatch(FanucError):
    """A trajectory was rejected because its first knot ≠ the commanded pose.

    The CAPTURE-or-REJECT contract (PLAN §5.1): a submitted trajectory whose
    ``q[0]`` is farther than the 5° capture window
    (``controller_facts.CAPTURE_TOL_DEG`` = 5.0°) from the robot's current
    commanded joints is refused rather than snapped to.
    """


class OwnershipError(FanucError):
    """Another process already holds the single-owner controller lock.

    Carries the holder's ``pid`` / ``mode`` (control|receive|tool) / ``since``
    (acquisition timestamp) parsed from the flock owner file so the message can
    name the blocker ("kill <PID>"). Also the mapped surfacing of a controller
    2556954 ("Robot is Already Connected.") at preflight (F24).
    """

    def __init__(
        self,
        pid: int | None = None,
        mode: str | None = None,
        since: float | None = None,
        message: str | None = None,
    ) -> None:
        self.pid = pid
        self.mode = mode
        self.since = since
        if message is None:
            message = f"controller already owned by pid={pid} mode={mode} since={since}"
        super().__init__(message)


class RmiSessionDown(FanucError):
    """The RMI command session is down and bounded auto-reopen failed.

    Raised to *workers* (gripper pokes, ext-status polls, register I/O) that hit
    a dead RMI session, in place of an implicit reopen. This is the F3+F5
    contract: transport auto-reopen is a bounded, single-flight concern of the
    :class:`~airo_fanuc.rmi_client.RmiClient`; a persistent failure surfaces
    here so the lifecycle supervisor can set the RMI_DOWN condition bit (F20)
    and drive recovery, rather than each worker racing to reconnect (which
    self-inflicts controller 2556954 and the gripper TOCTOU F32).
    """


class TrajectoryValidationError(FanucError):
    """A ``move_trajectory`` argument failed validation (PLAN §5.1).

    Per-violation typed error for the Python-side validation that mirrors the
    C++ defense: times not strictly increasing, <2 knots, non-finite q/qd,
    ``|s·qd| > v_lim``, or ``s > 1.0``. Raised synchronously so callers never
    submit an unsafe timeline.
    """


class CalibrationError(FanucError):
    """A :class:`~airo_fanuc.receive_interface.FanucReceiveInterface` calibration
    capture was rejected (PLAN R2 F30/F31, decision 14).

    Base for the hand-eye capture guards that structurally prevent the
    2026-05-17 T1-freeze corruption class. Catch this to handle any capture
    rejection; catch a subclass for the specific guard that fired. Raised (not
    returned as ``None``) so a bad sample is *loud*, never silently folded into
    a calibration dataset.
    """


class CalibrationSourceError(CalibrationError):
    """RMI-sourced joints were used for a calibration capture before the J2/J3
    representation was proven identical to Stream Motion (PLAN R2 F31).

    Carries the offending ``source`` tag and the name of the ``controller_facts``
    field that gates the reject (``rmi_joints_identical_to_stream``). While that
    fact is ``False`` a vendor J3+=J2 coupling would inject a silent J2-sized FK
    error, so RMI joints are HARD-rejected. Also raised on an attempt to mix RMI
    and Stream Motion joints in a single dataset ("never mix representations").
    """

    def __init__(
        self,
        message: str,
        *,
        source: str | None = None,
        fact: str | None = None,
    ) -> None:
        self.source = source
        self.fact = fact
        super().__init__(message)


class CalibrationVelocityUnavailable(CalibrationError):
    """A calibration capture was attempted while joint velocity was unavailable
    (PLAN R2 F30).

    The least-squares velocity estimate returned ``None`` (too few samples, an
    insufficient time base, or a stale feed). The capture is rejected rather than
    fabricating a zero velocity — publishing zeros for an unknown velocity is
    exactly the 2026-05-17 corruption (a frozen feed reads as "settled at 0°/s").
    """


class RmiError(RuntimeError):
    """A non-zero ErrorID returned by an RMI command (``dries`` verbatim).

    Kept a ``RuntimeError`` (not a :class:`FanucError`) to preserve the
    ``dries`` ``rmi_client`` catch contract. Beyond the formatted message it
    carries the raw ``error_id`` and its decoded ``text`` so callers can branch
    on the code (e.g. 2556954 → ownership hint, 2556943 → Init recovery ladder)
    without re-parsing the string.
    """

    def __init__(
        self,
        message: str,
        *,
        error_id: int | None = None,
        text: str | None = None,
        response: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.error_id = error_id
        self.text = text
        self.response = response


__all__ = [
    "FanucError",
    "FanucConnectionError",
    "FanucPreflightError",
    "RobotFaultedError",
    "RejectedStartMismatch",
    "OwnershipError",
    "RmiSessionDown",
    "TrajectoryValidationError",
    "CalibrationError",
    "CalibrationSourceError",
    "CalibrationVelocityUnavailable",
    "RmiError",
]
