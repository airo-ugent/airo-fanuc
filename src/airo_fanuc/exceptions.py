# SPDX-License-Identifier: Apache-2.0
"""Typed exception hierarchy for the ``airo_fanuc`` driver.

Single-sourced here so every module (RmiClient, the StreamCore Python layer,
FanucDriver, GripperWorker, the ownership lock, FanucReceiveInterface) *imports*
these rather than redefining them — one ``isinstance`` tree across the package.

Two roots on purpose:

* :class:`FanucError` — the driver-owned base. Everything a caller of the
  public :class:`~airo_fanuc.FanucDriver` API might catch derives from it.
* :class:`RmiError` — a controller-reported non-zero ErrorID, and a
  ``RuntimeError`` rather than a :class:`FanucError`. It sits deliberately
  *outside* the driver-owned tree: it is a low-level RMI-protocol signal rather
  than a driver verdict, and the RMI-level ladders need to catch it together
  with raw transport failures as ``except (RmiError, OSError)`` (the Initialize
  recovery ladder does exactly that). Callers that instead want to treat
  controller errors uniformly with transport/lifecycle errors special-case it.

``CommandEpochError`` is intentionally absent: a command issued against a stale
epoch is refused as a latched-FAULTED typed reject
(:class:`RobotFaultedError`, carrying the fault reason and operator hint), so a
dedicated epoch exception would only be a second, redundant path for a condition
the lifecycle state machine already reports. Do not add it.
"""

from __future__ import annotations

from typing import Any


class FanucError(Exception):
    """Base class for every driver-owned error.

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
    """A preflight gate failed.

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

    The CAPTURE-or-REJECT contract: a submitted trajectory whose ``q[0]`` is
    farther than the 5° capture window (``controller_facts.CAPTURE_TOL_DEG`` =
    5.0°) from the robot's current commanded joints is refused rather than
    snapped to — snapping would start the motion with an unplanned jump.
    """


class OwnershipError(FanucError):
    """Another process already holds the single-owner controller lock.

    Carries the holder's ``pid`` / ``mode`` (control|receive|tool) / ``since``
    (acquisition timestamp) parsed from the flock owner file so the message can
    name the blocker ("kill <PID>"). Also the mapped surfacing of a controller
    2556954 ("Robot is Already Connected.") at preflight — the controller's own
    report of the same single-owner conflict.
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
    a dead RMI session, in place of an implicit reopen. Transport auto-reopen is
    a bounded, single-flight concern of :class:`~airo_fanuc.rmi_client.RmiClient`;
    a persistent failure surfaces here so the supervisor can set the RMI_DOWN
    condition bit and drive recovery, rather than each worker racing to
    reconnect — concurrent reopens self-inflict controller 2556954 ("Robot is
    Already Connected.") and race a multi-register gripper command against a
    session rebuild.
    """


class TrajectoryValidationError(FanucError):
    """A ``move_trajectory`` argument failed validation.

    Per-violation typed error for the Python-side validation that mirrors the
    C++ defense: times not strictly increasing, <2 knots, non-finite q/qd,
    ``|qd| > v_lim``, a first knot above the capture envelope, a non-finite or
    non-positive ``force_stop_n``/``deadman_s``, and — from ``move_j`` — a target outside
    the position limits or an arm that is not at rest. Raised synchronously so callers
    never submit an unsafe timeline.
    """


class CalibrationError(FanucError):
    """A :class:`~airo_fanuc.receive_interface.FanucReceiveInterface` calibration
    capture was rejected.

    Base for the hand-eye capture guards that structurally prevent the
    frozen-feed corruption class: a stale or frozen joint feed reads exactly like
    a perfectly settled robot, so an unguarded capture silently folds a
    wrong-pose sample into the dataset. Catch this to handle any capture
    rejection; catch a subclass for the specific guard that fired. Raised (not
    returned as ``None``) so a bad sample is *loud*, never silently folded into
    a calibration dataset.
    """


class CalibrationSourceError(CalibrationError):
    """RMI-sourced joints were used for a calibration capture while the RMI→stream
    J3 conversion is unverified.

    Carries the offending ``source`` tag and the name of the ``controller_facts``
    field that gates the reject (``rmi_to_stream_j3_plus_j2_verified``). While that
    fact is ``False`` the uncorrected RMI J3 — reported one J2 below the Stream
    Motion one — would inject a silent J2-sized FK error, so RMI joints are
    HARD-rejected. Also raised on an attempt to mix RMI and Stream Motion joints in
    a single dataset ("never mix representations").
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
    """A calibration capture was attempted while joint velocity was unavailable.

    The least-squares velocity estimate returned ``None`` (too few samples, an
    insufficient time base, or a stale feed). The capture is rejected rather than
    fabricating a zero velocity — publishing zeros for an unknown velocity is
    precisely the frozen-feed corruption, because a frozen feed then reads as
    "settled at 0°/s" and passes every settle check.
    """


class RmiError(RuntimeError):
    """A non-zero ErrorID returned by an RMI command.

    A ``RuntimeError`` and not a :class:`FanucError` — the RMI-protocol layer is
    below the driver-owned error tree, so its ladders can catch a controller
    ErrorID and a raw socket failure in one ``except (RmiError, OSError)``.
    Beyond the formatted message it carries the raw ``error_id`` and its decoded
    ``text`` so callers can branch on the code (e.g. 2556954 → ownership hint,
    2556943 → Init recovery ladder) without re-parsing the string.
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
