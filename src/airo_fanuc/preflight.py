# SPDX-License-Identifier: Apache-2.0
"""Per-connect preflight gate.

The full gate ladder MINUS the recurring FTP checks: P-level / S636 / orderfil are
one-time bench facts, so re-fetching them over FTP on every connect would add a slow,
fallible step to every bring-up for facts that do not change. ``full=True`` reads them
from the controller via :mod:`airo_fanuc.controller_probe` — the controller's own order
file and version diagnostic rather than a transcription — and also cross-checks the
arm profile against the limits the controller says it is enforcing.

Every bring-up runs :func:`run_preflight` against an already-connected
commands-only :class:`~airo_fanuc.rmi_client.RmiClient` and gets a structured
:class:`PreflightReport` back — attached to the STARTUP event and, on a hard
block, wrapped in :class:`~airo_fanuc.exceptions.FanucPreflightError` so the
constructor surfaces *why* motion was refused before it is attempted.

Checks (per-connect):

* RMI handshake usable (``FRC_GetStatus`` returns) — else the caller's
  :meth:`RmiClient.start` already raised :class:`FanucConnectionError`.
* AUTO mode — T1/T2 is a *soft* warning (DEGRADED/TEACH), not a hard block.
* drives powered / e-stop — surfaced as warnings + the ReadError alarm text.
* active-alarm classification — SYST-348 / SYST-322 are **hard blocks** with
  operator instructions; OVR% < 100 and DCS speed-clamp < 100 are warnings.
* UI[2] — deliberately NOT read or gated: ``UI[2]=0`` is FANUC-normal on this
  SOP-less CRX (observed on every ``FRC_GetStatus``), so gating on it would block
  every bring-up.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from .controller_facts import MEASURED_FACTS
from .controller_probe import (
    OPTION_EXTERNAL_CONTROL,
    ControllerFacts,
    ControllerProbeError,
    p_level_key,
    probe_controller,
)
from .exceptions import FanucPreflightError, RmiError, RmiSessionDown
from .gripper import ROBOTIQ_2F85
from .rmi_client import RmiClient

if TYPE_CHECKING:
    from .robot_profile import RobotProfile

logger = logging.getLogger("airo_fanuc.preflight")

__all__ = ["PreflightReport", "run_preflight"]

# FRC_GetStatus.TPMode values that mean AUTO. The standard FANUC AUTO code is 2,
# but this SOP-less CRX reports 0 in its (permanent) AUTO — measured on the
# controller (docs/controller-notes.md §1.6). Both are AUTO here; only 1 (T1) /
# 3 (T2) are genuine TEACH modes. The supervisor imports this rather than restating it.
_TP_MODES_AUTO = (0, 2)

# Alarm-prefix classification: alarms that no amount of retrying will clear, so
# bring-up must refuse rather than loop.
_HARD_BLOCK_ALARMS: tuple[tuple[str, str], ...] = (
    (
        "SYST-348",
        "SYST-348 payload monitor active — confirm the payload on the TP "
        "(RESET does NOT clear it; do not jog in AUTO — re-raises SYST-328).",
    ),
    (
        "SYST-322",
        "SYST-322 auto status-check timeout — requires a controller power-cycle (+ DCS acknowledge).",
    ),
)


@dataclass
class PreflightReport:
    """Structured outcome of :func:`run_preflight`. ``ok`` iff no hard blocks."""

    auto_mode: bool = False
    tp_mode: int = 0
    servo_ready: bool = False
    drives_powered: bool = False
    gen_override_pct: int | None = None
    speed_clamp_limit_pct: float | None = None
    alarms: tuple[str, ...] = ()
    warnings: list[str] = field(default_factory=list)
    hard_blocks: list[str] = field(default_factory=list)
    #: What the controller said about itself over FTP. Only populated under
    #: ``full=True``; ``None`` otherwise, and also ``None`` if the probe failed (which
    #: is a warning, never a block — the driver runs fine without these facts).
    controller: ControllerFacts | None = None

    @property
    def ok(self) -> bool:
        return not self.hard_blocks

    def summary(self) -> str:
        parts = [
            f"auto_mode={self.auto_mode}",
            f"tp_mode={self.tp_mode}",
            f"servo_ready={self.servo_ready}",
            f"drives_powered={self.drives_powered}",
        ]
        if self.gen_override_pct is not None:
            parts.append(f"override={self.gen_override_pct}%")
        if self.controller is not None:
            parts.append(f"model={self.controller.model}")
            if self.controller.order is not None:
                parts.append(f"p_level={self.controller.order.deliver_version}")
        if self.alarms:
            parts.append(f"alarms={list(self.alarms)}")
        if self.warnings:
            parts.append(f"warnings={self.warnings}")
        if self.hard_blocks:
            parts.append(f"HARD_BLOCKS={self.hard_blocks}")
        return "preflight[" + ", ".join(parts) + "]"


#: The gripper path, for a caller that does not name its own pair. Checked only when
#: the caller says it wants a gripper, since a dispatcher lives on controller flash and
#: is a site-installation prerequisite that no wheel can supply — its absence is a real
#: block for that caller and irrelevant noise for everyone else. The names come from
#: :class:`~airo_fanuc.gripper.RegisterGripperProtocol`, so a site with its own
#: dispatcher is checked against its own programs rather than ours.
_DEFAULT_GRIPPER_PROGRAMS = ROBOTIQ_2F85.tp_programs


def run_preflight(
    rmi: RmiClient,
    *,
    full: bool = False,
    profile: RobotProfile | None = None,
    expect_gripper: bool = False,
    gripper_programs: tuple[str, ...] = _DEFAULT_GRIPPER_PROGRAMS,
    probe_timeout_s: float = 15.0,
) -> PreflightReport:
    """Run the per-connect preflight gate against an already-connected ``rmi``.

    Returns a :class:`PreflightReport`. Raises :class:`FanucPreflightError`
    (carrying the report) if any hard block is present.

    ``full=True`` adds the one-time FTP checks — P-level band, S636, the TP programs
    that back it — read off the controller rather than transcribed, plus a cross-check
    of ``profile`` against the joint limits the controller reports. The recurring
    per-connect path does not FTP.
    """
    report = PreflightReport()

    # RMI handshake usable + mode / drives / e-stop.
    try:
        status = rmi.get_status()
        ext = rmi.get_extended_status(quiet=True)
    except (RmiError, RmiSessionDown, OSError) as exc:
        report.hard_blocks.append(f"RMI status query failed: {exc}")
        raise FanucPreflightError(report.summary()) from exc

    report.tp_mode = status.tp_mode
    report.auto_mode = status.tp_mode in _TP_MODES_AUTO
    report.servo_ready = status.servo_ready
    report.drives_powered = ext.drives_powered
    report.gen_override_pct = ext.gen_override_pct
    report.speed_clamp_limit_pct = ext.speed_clamp_limit_pct

    if not report.auto_mode:
        report.warnings.append("teach pendant not in AUTO (T1/T2) — bring-up will DEGRADE to TEACH")
    if not report.servo_ready:
        report.warnings.append("servo not ready (drives off / e-stop / alarm)")
    if not report.drives_powered:
        report.warnings.append("drives not powered")
    if report.gen_override_pct is not None and report.gen_override_pct < 100:
        report.warnings.append(f"general override {report.gen_override_pct}% < 100% (set OVR=100)")
    if report.speed_clamp_limit_pct is not None and report.speed_clamp_limit_pct < 100:
        report.warnings.append(
            f"DCS speed clamp {report.speed_clamp_limit_pct:.0f}% < 100% (RESET on TP to release)"
        )

    # Active-alarm classification.
    try:
        err = rmi.read_error(5)
        report.alarms = err.messages
    except (RmiError, RmiSessionDown, OSError) as exc:
        report.warnings.append(f"ReadError failed during preflight: {exc}")

    for alarm in report.alarms:
        for prefix, hint in _HARD_BLOCK_ALARMS:
            if prefix in alarm:
                report.hard_blocks.append(hint)

    if full:
        _run_full_checks(
            report,
            rmi.controller_ip,
            profile=profile,
            expect_gripper=expect_gripper,
            gripper_programs=gripper_programs,
            probe_timeout_s=probe_timeout_s,
        )

    logger.info("%s", report.summary())
    if report.hard_blocks:
        raise FanucPreflightError(report.summary())
    return report


# ---------------------------------------------------------------------------
# The one-time FTP checks. Every finding here is a warning unless the controller
# positively states something disqualifying: absence of evidence (a file that would
# not fetch) must never read as evidence of absence, because that would turn an
# unreachable FTP server into a bring-up failure for a driver that does not need it.
# ---------------------------------------------------------------------------

#: Programs that must exist on the controller for the External Control Package to be
#: usable, whatever the order file says. They are functional evidence: J519 and R912
#: are bundled into S636 rather than ordered separately.
_EXTERNAL_CONTROL_PROGRAMS = ("stream_motn", "rmi_move")

#: How far a supplied profile may sit from the controller's own limits before the
#: divergence is worth a line in the report.
_LIMIT_TOL_DEG = 0.05
_VELOCITY_TOL_DEG_S = 0.5


def _run_full_checks(
    report: PreflightReport,
    ip: str,
    *,
    profile: RobotProfile | None,
    expect_gripper: bool,
    gripper_programs: tuple[str, ...],
    probe_timeout_s: float,
) -> None:
    try:
        facts = probe_controller(ip, timeout_s=probe_timeout_s)
    except ControllerProbeError as exc:
        report.warnings.append(f"full preflight: controller probe failed, checks skipped ({exc})")
        return

    report.controller = facts
    for warning in facts.warnings:
        report.warnings.append(f"controller probe: {warning}")

    _check_p_level(report, facts)
    _check_external_control(report, facts)
    if expect_gripper:
        missing = [p for p in gripper_programs if not facts.has_tp_program(p)]
        if missing and facts.tp_programs:
            report.hard_blocks.append(
                f"gripper enabled but TP program(s) {missing} are not installed on the controller. "
                f"A gripper dispatcher lives on controller flash and cannot be shipped in the wheel; "
                f"install it or run with DriverPolicy(enable_gripper=False)."
            )
    if profile is not None:
        _cross_check_profile(report, facts, profile)


def _check_p_level(report: PreflightReport, facts: ControllerFacts) -> None:
    """Band the controller's software P-level.

    Gates on the order file's "Deliver Ver", which is the field the recorded decision
    used. It is not always the running edition — this controller ships V9.40/P82 in the
    order file while ``version.dg`` reports V9.40P/84 — so when the two disagree, say
    so rather than quietly picking the more flattering one.
    """
    order = facts.order
    if order is None or not order.deliver_version:
        report.warnings.append("full preflight: no order-file Deliver Ver, P-level not banded")
        return

    got = p_level_key(order.deliver_version)
    hard = p_level_key(MEASURED_FACTS.p_level_min_hard)
    warn = p_level_key(MEASURED_FACTS.p_level_min_warn)
    if got is None:
        report.warnings.append(f"full preflight: unparseable Deliver Ver {order.deliver_version!r}")
        return

    if hard is not None and got < hard:
        report.hard_blocks.append(
            f"controller software {order.deliver_version} is below the hard floor "
            f"{MEASURED_FACTS.p_level_min_hard} — update the controller before streaming."
        )
    elif warn is not None and got < warn:
        report.warnings.append(
            f"controller software {order.deliver_version} is in the vibration-risk WARN band "
            f"(< {MEASURED_FACTS.p_level_min_warn}); non-blocking"
        )

    edition = facts.version.software_edition if facts.version else None
    if edition and p_level_key(edition) != got:
        report.warnings.append(
            f"order file says {order.deliver_version} but version.dg reports {edition}; "
            f"the gate used the order file"
        )


def _check_external_control(report: PreflightReport, facts: ControllerFacts) -> None:
    """S636 present? Order code first, TP programs as the functional backstop."""
    missing_programs = [p for p in _EXTERNAL_CONTROL_PROGRAMS if not facts.has_tp_program(p)]
    if facts.has_external_control:
        if missing_programs and facts.tp_programs:
            report.warnings.append(
                f"{OPTION_EXTERNAL_CONTROL} is ordered but TP program(s) {missing_programs} are "
                f"not on the controller"
            )
        return

    if facts.order is None:
        return  # no order file read — unknown, not absent
    if not missing_programs:
        report.warnings.append(
            f"{OPTION_EXTERNAL_CONTROL} not listed in the order file, but "
            f"{list(_EXTERNAL_CONTROL_PROGRAMS)} are installed — treating External Control as present"
        )
        return
    report.hard_blocks.append(
        f"External Control Package ({OPTION_EXTERNAL_CONTROL}) is not in the order file and "
        f"TP program(s) {missing_programs} are missing. Stream Motion and RMI are what this "
        f"driver speaks; without the option there is nothing to connect to."
    )


def _cross_check_profile(report: PreflightReport, facts: ControllerFacts, profile: RobotProfile) -> None:
    """Compare the arm profile against the limits the controller says it enforces.

    Divergence is reported, never adopted: the controller's numbers are authoritative
    about what it will allow, but the profile's are a deliberate choice by whoever
    configured this driver, and silently overwriting that at bring-up would change the
    clamps under a caller who never asked. The direction matters and is named — a
    profile *wider* than the controller lets commands through that the controller will
    answer with a limit alarm, while a narrower one is merely conservative.
    """
    limits = facts.limits
    if limits is None:
        return
    if limits.ndof != profile.ndof:
        report.warnings.append(
            f"controller reports {limits.ndof} joints, profile {profile.name!r} declares {profile.ndof}"
        )
        return

    import numpy as np

    n = profile.ndof
    v_profile = np.degrees(profile.velocity_limits)
    v_ctrl = np.asarray(limits.velocity_deg_s[:n], dtype=float)
    for j in np.where(np.abs(v_profile - v_ctrl) > _VELOCITY_TOL_DEG_S)[0]:
        direction = "ABOVE" if v_profile[j] > v_ctrl[j] else "below"
        report.warnings.append(
            f"profile velocity J{j + 1} {v_profile[j]:.2f}°/s is {direction} the controller's "
            f"{v_ctrl[j]:.2f}°/s"
        )

    for label, prof_deg, ctrl_deg, wide in (
        ("lower", profile.position_limits_lower_deg, limits.lower_deg[:n], "below"),
        ("upper", profile.position_limits_upper_deg, limits.upper_deg[:n], "above"),
    ):
        prof_arr = np.asarray(prof_deg, dtype=float)
        ctrl_arr = np.asarray(ctrl_deg, dtype=float)
        delta = prof_arr - ctrl_arr
        for j in np.where(np.abs(delta) > _LIMIT_TOL_DEG)[0]:
            outside = (delta[j] < 0) if label == "lower" else (delta[j] > 0)
            note = (
                f"WIDER than the controller ({wide} its own limit) — a planned pose there is a limit alarm"
                if outside
                else "narrower than the controller (conservative)"
            )
            report.warnings.append(
                f"profile {label} limit J{j + 1} {prof_arr[j]:.3f}° vs controller {ctrl_arr[j]:.3f}°: {note}"
            )
