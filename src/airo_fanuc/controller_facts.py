# SPDX-License-Identifier: Apache-2.0
"""Single source of truth for CRX-10iA/L kinematic limits and P-1-gated controller facts.

Every constant whose *true* value is measured on the physical controller lives here,
each carrying a ``MEASURED(P-1 Exx)`` marker naming the probe that confirmed it. The
**P-1 HIL probe day** (`docs/reference/airo-fanuc/p_minus_1/`) ran 2026-07-06 and the
values below were transcribed from its output (``confirmed=True``). Two facts remain
deferred and keep their safe defaults — **E1** (e-stop continuation path A unproven,
the SM stream was down) and **E3** (J2/J3 representation, not run on hardware). If a
value here ever changes, update it in ONE place (this module) and re-run the affected
tests.

Do NOT scatter these numbers across the C++ core, the FakeCRX plant, or the supervisor —
they flow from here into ``DriverConfig`` (Python) and are passed into the C++ ``StreamCore``
at construction. The C++ side holds only mirror-comment copies used by its own unit tests;
if you change a value here that a C++ test hardcodes, grep ``controller_facts`` in the C++
tree and update the mirror comment + test.

References: PLAN.md §5 (binding contracts), `docs/controller-notes.md` (P-1 fact table),
`docs/reference/hil-probe-runbook.md`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Stream Motion timing (fixed by the R-30iB controller class; NOT a P-1 unknown)
# ---------------------------------------------------------------------------

#: Interpolation period of the R-30iB controller in seconds (8 ms ITP = 125 Hz).
#: R-50iA + S647 would give 1 kHz; the CRX is fixed at 8 ms.
ITP_S: float = 0.008
STREAM_RATE_HZ: float = 125.0

#: Safety-critical: CommandPacket dataStyle field. FANUC's vendored struct calls this
#: field ``uint16_t unused``; we MUST write 0xFFFF (joint angles) — writing 0 makes the
#: controller read the payload as Cartesian XYZWPR → slew → E-stop (incident 2026-05-06).
COMMAND_DATA_STYLE: int = 0xFFFF

# ---------------------------------------------------------------------------
# CRX-10iA/L kinematic limits (rad, rad/s, rad/s², rad/s³)
#
# THE single source of truth for this arm's limits. Everything else derives from
# here: the C++ tick engine brakes with these values, and the curobo planner's
# cspace envelope in configs/curobo/crx10ial.yml is generated from them by
# scripts/fit_spheres.py (guarded by tests/test_robot_limits_parity.py).
#
# Provenance matters here because FANUC publishes only joint SPEEDS for the CRX
# series — acceleration and jerk are engineering derivations, so the reasoning is
# recorded rather than just the numbers:
#
#   * Velocity — FANUC Europe datasheet MDS-04018: J1,J2 = 120°/s; J3-J6 = 180°/s.
#   * Acceleration = 2× velocity (~1 s to reach max velocity, typical for cobots).
#     Universal Robots' joint-health guidance recommends ≤5.2 rad/s² (300°/s²) and
#     Franka Panda specs 15 rad/s², so these sit at the conservative end.
#   * Jerk = 8× acceleration (~0.125 s to max accel). Conservative; some aggressive
#     cobot configs target ~33× accel.
#
# Sources:
#   * FANUC Europe datasheet MDS-04018 (CRX-10iA family)
#   * mathemonads/fanuc_crx10ial_moveit_config — a community MoveIt config that
#     leaves acceleration disabled, confirming FANUC publishes no accel limits
#   * https://forum.universal-robots.com/t/maximum-axis-speed-acceleration/13338
#   * https://answers.ros.org/question/406533/how-to-make-fanuc-crx-10ial-move-faster/
#
# The PLANNER deliberately uses a softer jerk (3× accel, not 8×) — see
# scripts/fit_spheres.py. The CRX collaborative-stop monitor infers contact force
# from motor disturbance torque, so a sharp acceleration ramp reads as a phantom
# contact mid-transit. Jerk is the trip trigger; acceleration is not.
#
# P-1 cross-check (E5 / HIL-L9): the controller's $PARAM_GROUP / $MRR_GRP.$JNTVELLIM
# may report different active limits than the datasheet; flag divergence but do NOT
# auto-adopt.
# ---------------------------------------------------------------------------

CRX10IAL_VELOCITY_LIMITS: np.ndarray = np.array([2.094, 2.094, 3.142, 3.142, 3.142, 3.142], dtype=np.float64)
CRX10IAL_ACCELERATION_LIMITS: np.ndarray = 2.0 * CRX10IAL_VELOCITY_LIMITS
CRX10IAL_JERK_LIMITS: np.ndarray = 8.0 * CRX10IAL_ACCELERATION_LIMITS

# Brake / stop envelope scale factors (PLAN.md decision 5; dries B15).
# Split v/a vs jerk: the CRX collaborative-stop monitor estimates contact force from motor
# disturbance torque, so a high jerk ramp flags a phantom contact (observed 2026-05-16).
STOP_LIMIT_SCALE_VA: float = 0.4
STOP_LIMIT_SCALE_J: float = 0.15

# Per-tick slew clip: |Δq| ≤ SLEW_FACTOR × v_limit × ITP per joint (dries; last-line defense
# vs swap/merge discontinuities the CRX DCS reads as disturbance torque). Clip+count, never
# fault. The accel-cap on derived velocity stays BANNED (reverted anti-pattern, 22° drift).
SLEW_FACTOR: float = 1.2


@dataclass(frozen=True)
class P1Facts:
    """Controller facts measured at the P-1 HIL probe day.

    Values are the P-1-measured facts (probe day 2026-07-06); each field names the
    probe (Exx) that confirmed it. ``confirmed`` was flipped to ``True`` once the
    probe output was transcribed. The two deferred facts (E1 e-stop path A, E3 J2/J3
    representation) keep their safe defaults; code paths that would be unsafe under a
    wrong guess still assert on them (e.g. the calibration loader hard-rejects RMI
    joints while ``rmi_joints_identical_to_stream`` is False).
    """

    confirmed: bool = True  # P-1 HIL probe day 2026-07-06: values transcribed from probe output

    # --- E9 / H11: servo tracking lag (first-order) ---
    # Used by: drift guard (plan @ now − lag), FakeCRX plant τ, DriverConfig.tracking_lag_s.
    tracking_lag_s: float = 0.025  # MEASURED(P-1 E9): xcorr 25 ms (verify runs 20 ms); was 0.107 interim

    # --- E6 / H3 / HIL-L10: TX-silence backstop — THE go/no-go ---
    # MEASURED(P-1 E6): NO-GO. On TX silence the controller does NOT fast-decel within 2-3 ITPs; it
    # coasts at the last commanded velocity, then drops motion_possible (DEVIATION-triggered, not fixed-time):
    #   15.3 deg/s -> overrun 2.10 deg, motion_possible drop 121 ms, no clean decel onset (abrupt stop);
    #   49.9 deg/s -> overrun 4.63 deg, motion_possible drop 71 ms, decel onset 86.7 ms (10.8 ITPs).
    # No alarm, no runaway; stop within ~110-130 ms; overrun grows SUB-linearly with speed.
    # => in-process design may NOT rely on the controller as a FAST host-death backstop; the pre-committed
    #    fallback (external RMI-abort watchdog and/or DCS zone tightening) is REQUIRED. Deadman ~120 ms coast.
    tx_silence_backstop_ok: bool = False  # MEASURED(P-1 E6): NO-GO (decel onset ~10 ITPs, not <=3)
    # Deviation-watchdog threshold the controller uses to fault a frozen (un-ramped) command
    # stream; also calibrates the FakeCRX deviation-watchdog emulation. deg.
    deviation_watchdog_deg: float = 5.0  # MEASURED(P-1 E6): worst overrun 4.63 deg @ 49.9 deg/s

    # --- E1 / HIL-L1: e-stop continuation ---
    # "B" = full SM re-handshake (always-safe default). "A" = fast resume (behind policy,
    # only after HIL-L1 confirms the session survives e-stop).
    # P-1 E1: INCONCLUSIVE — SM status never streamed during the probe (STREAM_MOTN was aborted during an
    # earlier HOST-380 recovery), so path A could not be proven or refuted. Ship the safe default B.
    # (E-stop alarm text: SRVO-002 "Teach Pendant E-stop" + SRVO-289; FRC_Continue -> ErrorID 0 OK.)
    # Re-test path A once a live motion_possible session is available (post power-cycle it is).
    estop_continuation_path: str = "B"  # P-1 E1: default B retained (A unproven — SM stream was down)
    sm_session_survives_estop: bool = False  # P-1 E1: not proven (stream never came up during probe)

    # --- E3 / HIL-L7 / H4: J2/J3 representation ---
    # Until proven identical, RMI-sourced joints are tagged rmi_unconverted and calibration
    # HARD-REJECTS them (a wrong guess = silent J2-sized FK error). Vendor lib is known to
    # apply J3 += J2 on RMI reads; stream carries the coupled interaction angle unconverted.
    # P-1 E3: DEFERRED (not run on hardware) — single RMI session + single SM peer + AUTO-only (no T1 on
    # this CRX) + RMI-init locks hand-guidance made a clean simultaneous stream-vs-RMI capture impractical.
    # Safe default RETAINED (calibration still hard-rejects RMI joints). Resolve at P2'/L3 driver bring-up.
    rmi_joints_identical_to_stream: bool = False  # P-1 E3: deferred; keep hard-reject default
    rmi_j3_plus_j2_conversion: bool = True  # P-1 E3: deferred; vendor default kept (UNVERIFIED)

    # --- E8: RMI angle read quantization ---
    # Calibration stillness gate is 0.1 deg/s; quantization budget ≤ 0.0067 deg/read.
    rmi_angle_resolution_deg: float = 0.001  # MEASURED(P-1 E8): FRC_ReadJointAngles 3-decimal precision
    rmi_velocity_needs_lsq: bool = False  # MEASURED(P-1 E8): instantaneous OK (0 noise when still)

    # --- E5 / HIL-L9: static facts ---
    controller_p_level: str = "V9.40/P82"  # MEASURED(P-1 E5): orderfile (WARN band); TP fw P/84, boot P/77
    p_level_min_warn: str = "V9.40P84"
    p_level_min_hard: str = "V9.40P81"

    # --- E2 / HIL-L3: RMI single-session ---
    rmi_single_session: bool = True  # MEASURED(P-1 E2): confirmed (2nd times out on redirect port)

    # --- E4 / HIL-L8: RMI in T1 ---
    rmi_reads_ok_in_t1: bool = True  # MEASURED(P-1 E4): 450/450 reads track live motion @ 15 Hz (AUTO; no T1)


#: The P-1-measured fact set used throughout the codebase (single-sourced from here).
#: Name kept as ``INTERIM_FACTS`` for call-site stability; values are P-1 final (2026-07-06).
INTERIM_FACTS = P1Facts()


@dataclass(frozen=True)
class SettlePolicy:
    """End-of-trajectory settle criteria (PLAN.md §5.1). Production call sites override
    the defaults (dries used 1.5° / 1.5-3 s); H11 re-measures the constants."""

    tol_deg: float = 0.5
    vel_eps_deg_s: float = 2.0
    timeout_s: float = 2.0


# ---------------------------------------------------------------------------
# Capture / servo / brake / RX-silence / anti-flap constants (PLAN.md §5)
# ---------------------------------------------------------------------------

# CAPTURE-or-REJECT splice (decision 6, R3 A2): bridge commanded → new-trajectory (q0,qd0)
# via a deterministic Ruckig profile, collision-checked in Python before handoff.
CAPTURE_TOL_DEG: float = 5.0  # = dries re-anchor deadband; REJECT beyond, typed error
CAPTURE_RATE_DEG_S: float = 15.0  # = dries STARVATION_RE_ANCHOR_RATE; SAFE_FOLLOW envelope

# servo_j replace-not-queue window (R1 C3): typed reject if |q_target − q_cmd| > this.
SERVO_WINDOW_DEG: float = 5.0
SERVO_LIMIT_SCALE: float = 1.0

# Graduated RX-silence (decision 7, R1 A3). Mid-TRAJECTORY:
RX_SILENCE_BLIND_HOLD_MS: float = 100.0  # → kill-type entry: blind qd-ramp to hold
RX_SILENCE_QD_RAMP_MS: float = 60.0  # duration of the qd → 0 ramp
RX_SILENT_PARK_MS: float = 500.0  # → RX_SILENT fault, park TX

# Anti-flap dwell (R2 F7): DEGRADED → STREAMING requires all-clear sustained ≥ this
# (covers measured ~300 ms contact-stop / motion_possible bit-skew).
ANTIFLAP_DWELL_MS: float = 500.0

# qd_end blend (R1 B3): on trajectory exhaustion with non-zero terminal velocity, one
# Hermite blend (q_end,qd_end) → (q_end,0) over at least this long.
QD_END_BLEND_MIN_MS: float = 25.0

# Calibration stillness gate (R2 F30): hand_eye ground truth.
CALIB_STILLNESS_DEG_S: float = 0.1
CALIB_LSQ_WINDOW_S: float = 0.5

# ---------------------------------------------------------------------------
# In-process safety watchdogs wired at P-1 finalization (2026-07-06). Both
# FaultReasons (SUPERVISOR_LOST, DRIFT) existed but were never SET; P-1 E6 (host
# death rides a ~120 ms controller coast — no external watchdog) + E9 (25 ms lag)
# motivated wiring them in the C++ RT core. Mirrored in rt_core_config.hpp.
# ---------------------------------------------------------------------------

# SUPERVISOR_LOST: the RT core latches this + holds if the Python supervisor's
# heartbeat lapses for this long while streaming. The supervisor beats from a
# DEDICATED lightweight thread (no RMI, no locks), so this fires only on genuine
# supervisor/process death — not on a slow RMI round-trip. Generous vs the ~100 ms
# beat interval to tolerate GIL-storm / GC pauses without a false trip. seconds.
SUPERVISOR_LOST_S: float = 3.0

# DRIFT: sustained commanded↔measured divergence fault (the 22°-runaway guard;
# dries executor MAX_DRIFT). deg + consecutive ticks. The lag alignment uses
# tracking_lag_s (E9: 25 ms ≈ 3 ticks), so the threshold is genuine divergence,
# not the servo lag itself.
DRIFT_FAULT_DEG: float = 10.0
DRIFT_FAULT_TICKS: int = 5
