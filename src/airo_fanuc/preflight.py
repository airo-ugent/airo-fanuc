# SPDX-License-Identifier: Apache-2.0
"""Per-connect preflight gate (PLAN.md §5.3 / design doc 08 §9).

The design-doc-08 D14 ladder MINUS the recurring FTP checks (P-level / S636 /
orderfil are one-time bench facts cached in ``docs/controller-notes.md`` and only
re-run with ``full=True``). Every bring-up runs :func:`run_preflight` against an
already-connected commands-only :class:`~airo_fanuc.rmi_client.RmiClient` and gets
a structured :class:`PreflightReport` back — attached to the STARTUP event and, on
a hard block, wrapped in :class:`~airo_fanuc.exceptions.FanucPreflightError` so the
constructor surfaces *why* motion was refused before it is attempted.

Checks (per-connect):

* RMI handshake usable (``FRC_GetStatus`` returns) — else the caller's
  :meth:`RmiClient.start` already raised :class:`FanucConnectionError`.
* AUTO mode — T1/T2 is a *soft* warning (DEGRADED/TEACH), not a hard block.
* drives powered / e-stop — surfaced as warnings + the ReadError alarm text.
* active-alarm classification — SYST-348 / SYST-322 are **hard blocks** with
  operator instructions; OVR% < 100 and DCS speed-clamp < 100 are warnings.
* UI[2] — deliberately NOT read or gated on this SOP-less CRX (``UI[2]=0`` is
  FANUC-normal here; fault-matrix row 22).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

from .exceptions import FanucPreflightError, RmiError, RmiSessionDown
from .rmi_client import RmiClient

logger = logging.getLogger("airo_fanuc.preflight")

__all__ = ["PreflightReport", "run_preflight"]

# FRC_GetStatus.TPMode values that mean AUTO. The standard FANUC AUTO code is 2,
# but this SOP-less CRX reports 0 in its (permanent) AUTO (P-1 measured;
# controller-notes §1.6). Both are AUTO here; only 1 (T1) / 3 (T2) are genuine
# TEACH modes. Mirrors supervisor._TP_MODES_AUTO.
_TP_MODES_AUTO = (0, 2)

# Alarm-prefix classification (design doc 08 §9 / fault matrix rows 6/8).
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
        if self.alarms:
            parts.append(f"alarms={list(self.alarms)}")
        if self.warnings:
            parts.append(f"warnings={self.warnings}")
        if self.hard_blocks:
            parts.append(f"HARD_BLOCKS={self.hard_blocks}")
        return "preflight[" + ", ".join(parts) + "]"


def run_preflight(rmi: RmiClient, *, full: bool = False) -> PreflightReport:
    """Run the per-connect preflight gate against an already-connected ``rmi``.

    Returns a :class:`PreflightReport`. Raises :class:`FanucPreflightError`
    (carrying the report) if any hard block is present. ``full=True`` is the hook
    for the one-time FTP checks (P-level / S636 / orderfil) — deferred to P-1 /
    ``--full``; the recurring path does not FTP.
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
        # One-time FTP facts (P-level >= V9.40P84, S636/orderfil, TP programs
        # present) are cached in docs/controller-notes.md and only re-verified on
        # demand. Wire the FTP fetch here at P-1; per-connect never FTPs (R4 cut).
        report.warnings.append(
            "full preflight (FTP P-level/S636/orderfil) not implemented — see controller-notes.md"
        )

    logger.info("%s", report.summary())
    if report.hard_blocks:
        raise FanucPreflightError(report.summary())
    return report
