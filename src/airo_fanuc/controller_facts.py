# SPDX-License-Identifier: Apache-2.0
"""Single source of truth for CRX-10iA/L kinematic limits and measured controller facts.

Every constant whose *true* value is measured on the physical controller lives here,
each carrying a ``MEASURED`` marker and the observation that produced it. The values
below were transcribed from a hardware-in-the-loop probe run on 2026-07-06
(``confirmed=True``). Two facts are still UNVERIFIED and keep their safe defaults:
e-stop continuation path A (unprovable during the probe — the Stream Motion status
feed never came up) and the J2/J3 representation (never exercised on hardware). If a
value here ever changes, update it in ONE place (this module) and re-run the affected
tests.

Do NOT scatter these numbers across the C++ core, the FakeCRX plant, or the supervisor —
they flow from here into ``DriverConfig`` (Python) and are passed into the C++ ``StreamCore``
at construction. The C++ side holds only mirror-comment copies used by its own unit tests;
if you change a value here that a C++ test hardcodes, grep ``controller_facts`` in the C++
tree and update the mirror comment + test.

The narrative behind these facts — alarm texts, recovery procedures, the raw probe
observations — is in ``docs/controller-notes.md``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Stream Motion timing (fixed by the R-30iB controller class; not a measured unknown)
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
# THE single source of truth for this arm's limits. Everything downstream derives
# from here: the C++ tick engine brakes with these values, and any consumer that
# needs a motion envelope (a planner's cspace limits, say) should read them from this
# module rather than restating them.
#
# Only velocity comes straight off a FANUC datasheet; acceleration and jerk are
# engineering derivations, so the reasoning is recorded rather than just the numbers:
#
#   * Velocity — FANUC Europe datasheet MDS-04018: J1,J2 = 120°/s; J3-J6 = 180°/s.
#   * Acceleration = 2× velocity (~1 s to reach max velocity, typical for cobots).
#     Universal Robots' joint-health guidance recommends ≤5.2 rad/s² (300°/s²) and
#     Franka Panda specs 15 rad/s², so these sit at the conservative end.
#   * Jerk = 8× acceleration (~0.125 s to max accel). Conservative; some aggressive
#     cobot configs target ~33× accel.
#
# FANUC does also publish accelerations for this arm, and they are much lower than
# these: the vendored MoveIt config
# (`vendor/fanuc_driver/fanuc_moveit_config/config/joint_limits.yaml`, headed "Joint
# limits for CRX-10iA and CRX-10iA/L") gives max_acceleration 0.4 rad/s² for J1-J3 and
# 1.0 rad/s² for J4-J6, with velocities identical to ours — i.e. 6-16× below the
# derived values here. The two numbers are not the same kind of thing: FANUC's are
# *planning* limits, a target profile a planner shapes trajectories to, whereas these
# are *clamps* — the ceiling above which the RT core refuses to pass a command
# through — so they are deliberately looser, to avoid silently mangling a legitimate
# planned motion. Whether that gap is the right size is an OPEN QUESTION: what this
# controller actually tolerates has not been measured, and if a measurement lands near
# FANUC's planning figures then these clamps are too permissive to be a useful net.
# Resolve it with hardware measurement, not by picking one of the two numbers.
#
# A planner feeding this driver should shape trajectories with a SOFTER jerk than the
# clamp here (~3× accel rather than 8×). The CRX collaborative-stop monitor infers
# contact force from motor disturbance torque, so a sharp jerk ramp reads as a phantom
# contact mid-transit. Jerk is the trip trigger; acceleration is not.
#
# Sources:
#   * FANUC Europe datasheet MDS-04018 (CRX-10iA family)
#   * vendor/fanuc_driver/fanuc_moveit_config/config/joint_limits.yaml (FANUC's own
#     published planning velocity + acceleration limits)
#   * https://forum.universal-robots.com/t/maximum-axis-speed-acceleration/13338
#   * https://answers.ros.org/question/406533/how-to-make-fanuc-crx-10ial-move-faster/
#
# Worth cross-checking on any new controller: the active limits it reports in
# $PARAM_GROUP / $MRR_GRP.$JNTVELLIM may differ from the datasheet. Flag any
# divergence but do NOT auto-adopt it.
# ---------------------------------------------------------------------------

CRX10IAL_VELOCITY_LIMITS: np.ndarray = np.array([2.094, 2.094, 3.142, 3.142, 3.142, 3.142], dtype=np.float64)
CRX10IAL_ACCELERATION_LIMITS: np.ndarray = 2.0 * CRX10IAL_VELOCITY_LIMITS
CRX10IAL_JERK_LIMITS: np.ndarray = 8.0 * CRX10IAL_ACCELERATION_LIMITS

# Brake / stop envelope scale factors. Split v/a vs jerk: the CRX collaborative-stop
# monitor estimates contact force from motor disturbance torque, so a high jerk ramp reads
# as a contact and trips a phantom stop (observed 2026-05-16). Jerk is therefore scaled
# harder than v/a.
STOP_LIMIT_SCALE_VA: float = 0.4
STOP_LIMIT_SCALE_J: float = 0.15

# Per-tick slew clip: |Δq| ≤ SLEW_FACTOR × v_limit × ITP per joint — the last-line defense
# against swap/merge discontinuities the CRX DCS reads as disturbance torque. Clip+count,
# never fault. Capping acceleration on the *derived* velocity instead is BANNED: it
# integrates into a 22° position drift.
SLEW_FACTOR: float = 1.2


@dataclass(frozen=True)
class P1Facts:
    """Controller facts measured on the physical CRX.

    Each field carries the observation that produced it. ``confirmed`` is ``True``
    because these values are transcribed from a hardware probe (2026-07-06) rather
    than guessed. The two UNVERIFIED facts (e-stop continuation path A, the J2/J3
    representation) keep their safe defaults; code paths that would be unsafe under a
    wrong guess still assert on them (e.g. the calibration loader hard-rejects RMI
    joints while ``rmi_joints_identical_to_stream`` is False).
    """

    confirmed: bool = True  # values transcribed from the 2026-07-06 hardware probe output

    # --- servo tracking lag (first-order) ---
    # Used by: drift guard (plan @ now − lag), FakeCRX plant τ, DriverConfig.tracking_lag_s.
    tracking_lag_s: float = 0.025  # MEASURED: cross-correlation 25 ms (verification runs 20 ms)

    # --- TX-silence backstop — THE go/no-go for host-death safety ---
    # MEASURED: NO-GO. On TX silence the controller does NOT fast-decel within 2-3 ITPs; it
    # coasts at the last commanded velocity, then drops motion_possible (DEVIATION-triggered, not fixed-time):
    #   15.3 deg/s -> overrun 2.10 deg, motion_possible drop 121 ms, no clean decel onset (abrupt stop);
    #   49.9 deg/s -> overrun 4.63 deg, motion_possible drop 71 ms, decel onset 86.7 ms (10.8 ITPs).
    # No alarm, no runaway; stop within ~110-130 ms; overrun grows SUB-linearly with speed.
    # => in-process design may NOT rely on the controller as a FAST host-death backstop; an external
    #    fallback (an RMI-abort watchdog and/or DCS zone tightening) is REQUIRED. Deadman ~120 ms coast.
    tx_silence_backstop_ok: bool = False  # MEASURED: NO-GO (decel onset ~10 ITPs, not <=3)
    # Deviation-watchdog threshold the controller uses to fault a frozen (un-ramped) command
    # stream; also calibrates the FakeCRX deviation-watchdog emulation. deg.
    deviation_watchdog_deg: float = 5.0  # MEASURED: worst overrun 4.63 deg @ 49.9 deg/s

    # --- e-stop continuation ---
    # "B" = full SM re-handshake (always-safe default). "A" = fast resume (behind policy,
    # only once a measurement confirms the SM session survives an e-stop).
    # UNVERIFIED — INCONCLUSIVE on hardware: SM status never streamed during the probe (STREAM_MOTN
    # had been aborted during an earlier HOST-380 recovery), so path A could be neither proven nor
    # refuted. Ship the safe default B.
    # (E-stop alarm text: SRVO-002 "Teach Pendant E-stop" + SRVO-289; FRC_Continue -> ErrorID 0 OK.)
    # Re-test path A once a live motion_possible session is available (post power-cycle it is).
    estop_continuation_path: str = "B"  # UNVERIFIED: default B retained (A unproven — SM stream was down)
    sm_session_survives_estop: bool = False  # UNVERIFIED: not proven (stream never came up)

    # --- J2/J3 representation ---
    # Until proven identical, RMI-sourced joints are tagged rmi_unconverted and calibration
    # HARD-REJECTS them (a wrong guess = silent J2-sized FK error). Vendor lib is known to
    # apply J3 += J2 on RMI reads; stream carries the coupled interaction angle unconverted.
    # UNVERIFIED — never run on hardware: a single RMI session + a single SM peer + AUTO-only (no T1 on
    # this CRX) + RMI-init locking hand-guidance made a clean simultaneous stream-vs-RMI capture
    # impractical. Safe default RETAINED (calibration still hard-rejects RMI joints). Resolve during
    # on-hardware driver bring-up.
    rmi_joints_identical_to_stream: bool = False  # UNVERIFIED: keep hard-reject default
    rmi_j3_plus_j2_conversion: bool = True  # UNVERIFIED: vendor default kept

    # --- RMI angle read quantization ---
    # Calibration stillness gate is 0.1 deg/s; quantization budget ≤ 0.0067 deg/read.
    rmi_angle_resolution_deg: float = 0.001  # MEASURED: FRC_ReadJointAngles 3-decimal precision
    rmi_velocity_needs_lsq: bool = False  # MEASURED: instantaneous OK (0 noise when still)

    # --- static facts ---
    controller_p_level: str = "V9.40/P82"  # MEASURED: orderfile (WARN band); TP fw P/84, boot P/77
    p_level_min_warn: str = "V9.40P84"
    p_level_min_hard: str = "V9.40P81"

    # --- RMI single-session ---
    rmi_single_session: bool = True  # MEASURED: confirmed (2nd times out on redirect port)

    # --- RMI reads under live motion ---
    rmi_reads_ok_in_t1: bool = True  # MEASURED: 450/450 reads track live motion @ 15 Hz (AUTO; no T1)


#: The measured fact set used throughout the codebase (single-sourced from here).
#: Despite the ``INTERIM_FACTS`` name — stable because every call site imports it —
#: these values are the hardware-measured ones (2026-07-06), not placeholders.
INTERIM_FACTS = P1Facts()


@dataclass(frozen=True)
class SettlePolicy:
    """End-of-trajectory settle criteria. Production call sites routinely override the
    defaults — 1.5° with a 1.5-3 s timeout is a field-proven pair for coarser moves.
    These constants are worth re-measuring on hardware against the servo lag."""

    tol_deg: float = 0.5
    vel_eps_deg_s: float = 2.0
    timeout_s: float = 2.0


# ---------------------------------------------------------------------------
# Capture / servo / brake / RX-silence / anti-flap constants
# ---------------------------------------------------------------------------

# CAPTURE-or-REJECT splice: bridge commanded → new-trajectory (q0,qd0) via a
# deterministic Ruckig profile, collision-checked in Python before handoff.
CAPTURE_TOL_DEG: float = 5.0  # re-anchor deadband; REJECT beyond, typed error
CAPTURE_RATE_DEG_S: float = 15.0  # starvation re-anchor rate; also the SAFE_FOLLOW envelope

# servo_j replace-not-queue window: typed reject if |q_target − q_cmd| > this.
SERVO_WINDOW_DEG: float = 5.0
SERVO_LIMIT_SCALE: float = 1.0

# Graduated RX-silence response, mid-TRAJECTORY:
RX_SILENCE_BLIND_HOLD_MS: float = 100.0  # → kill-type entry: blind qd-ramp to hold
RX_SILENCE_QD_RAMP_MS: float = 60.0  # duration of the qd → 0 ramp
RX_SILENT_PARK_MS: float = 500.0  # → RX_SILENT fault, park TX

# Anti-flap dwell: DEGRADED → STREAMING requires all-clear sustained ≥ this
# (covers measured ~300 ms contact-stop / motion_possible bit-skew).
ANTIFLAP_DWELL_MS: float = 500.0

# qd_end blend: on trajectory exhaustion with non-zero terminal velocity, one
# Hermite blend (q_end,qd_end) → (q_end,0) over at least this long.
QD_END_BLEND_MIN_MS: float = 25.0

# Calibration stillness gate — the hand-eye capture ground truth.
CALIB_STILLNESS_DEG_S: float = 0.1
CALIB_LSQ_WINDOW_S: float = 0.5

# ---------------------------------------------------------------------------
# In-process safety watchdogs. Both FaultReasons (SUPERVISOR_LOST, DRIFT) are raised
# by the C++ RT core, and both exist because of the measurements above: host death
# rides a ~120 ms controller coast, so the controller is not a fast host-death
# backstop, and the 25 ms servo lag sets the drift alignment.
# Mirrored in rt_core_config.hpp.
# ---------------------------------------------------------------------------

# SUPERVISOR_LOST: the RT core latches this + holds if the Python supervisor's
# heartbeat lapses for this long while streaming. The supervisor beats from a
# DEDICATED lightweight thread (no RMI, no locks), so this fires only on genuine
# supervisor/process death — not on a slow RMI round-trip. Generous vs the ~100 ms
# beat interval to tolerate GIL-storm / GC pauses without a false trip. seconds.
SUPERVISOR_LOST_S: float = 3.0

# DRIFT: sustained commanded↔measured divergence fault — the guard against the 22°
# runaway. deg + consecutive ticks. The lag alignment uses tracking_lag_s (the measured
# 25 ms ≈ 3 ticks), so the threshold catches genuine divergence, not the servo lag itself.
DRIFT_FAULT_DEG: float = 10.0
DRIFT_FAULT_TICKS: int = 5
