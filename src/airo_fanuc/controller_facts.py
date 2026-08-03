# SPDX-License-Identifier: Apache-2.0
"""Single source of truth for measured controller facts and this driver's own tuning.

Two kinds of constant live here, and neither is a property of the *arm*:

* **Measured controller facts** — values whose true value was read off the physical
  controller, each carrying a ``MEASURED`` marker and the observation that produced
  it. They were transcribed from a hardware-in-the-loop probe run on 2026-07-06
  (``confirmed=True``). Two keep a safe default for want of a measurement: e-stop
  continuation path A, unprovable during the probe because the Stream Motion status
  feed never came up, and the RMI→stream J3 conversion, read at a single J2 value.
* **Driver tuning** — the brake scales, capture and servo windows, RX-silence ladder,
  anti-flap dwell and watchdog thresholds. Chosen against those measurements, but they
  are decisions rather than observations, and they carry across FANUC models unchanged.

The arm's own motion envelope is NOT here: velocity, acceleration and jerk clamps plus
joint position limits vary per robot model, so they are injected by the caller as an
:class:`airo_fanuc.robot_profile.RobotProfile`. ``examples/crx10ial.py`` builds one for
the CRX-10iA/L these examples run on.

If a value here ever changes, update it in ONE place (this module) and re-run the
affected tests. Do NOT scatter these numbers across the C++ core, the FakeCRX plant or
the supervisor — they flow from here into ``DriverConfig`` (Python) and are passed into
the C++ ``StreamCore`` at construction. The C++ side holds only mirror-comment copies
used by its own unit tests; if you change a value here that a C++ test hardcodes, grep
``controller_facts`` in the C++ tree and update the mirror comment + test.

The narrative behind these facts — alarm texts, recovery procedures, the raw probe
observations — is in ``docs/controller-notes.md``.
"""

from __future__ import annotations

from dataclasses import dataclass

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
# Brake / slew envelope
#
# Fractions of the arm's limits rather than limits themselves, which is why they live
# here and not in a RobotProfile: they express how hard THIS DRIVER is willing to push
# a given envelope, and that judgement carries across arms.
# ---------------------------------------------------------------------------

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
    because these values are transcribed from a hardware probe rather than guessed.
    Facts still UNVERIFIED (e-stop continuation path A) keep their safe defaults, and
    facts that are measured but INSTALLATION-SPECIFIC (whether this controller serves
    the J2/J3-coupled representation) stay off until the installation confirms them.
    Code paths that would be unsafe under a wrong guess assert on the facts they
    depend on — the calibration loader hard-rejects RMI joints while
    ``rmi_to_stream_j3_plus_j2_verified`` is False.
    """

    confirmed: bool = True  # values transcribed from the hardware probe output

    # --- servo tracking lag (first-order) ---
    # Used by: FakeCRX plant τ and the examples' lag reporting. Gates nothing.
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
    # MEASURED (docs/controller-notes.md §1.5): the two planes do NOT agree. RMI J3 =
    # SM J3 − J2, so the RMI→stream conversion is `q_stream[2] = q_rmi[2] + q_rmi[1]`.
    # The FORM is measured, not inferred from a single pose: both planes read at two
    # standstill poses in one session, 25 deg apart in J2, gave an offset that tracked J2
    # with a residual of 0.0000 deg, while a fixed offset mispredicted by the whole 25 deg.
    # Stream-plane J3 held still to 0.004 deg across the move, so it is the RMI plane that
    # carries the coupling. Every other joint agreed to 0.0000 at both poses.
    # Whether a given controller SERVES that representation is a controller CONFIGURATION,
    # not a property of the arm, so the conversion is OFF by default and the two failure
    # modes are deliberately asymmetric: off, RMI joints stay tagged rmi_unconverted and
    # calibration HARD-REJECTS them (loud); on where it does not apply, every derived pose
    # is silently one J2 out. Settle it per cell with
    # `examples/verify_j2j3_coupling.py --move`, then construct facts with
    # `dataclasses.replace(INTERIM_FACTS, rmi_to_stream_j3_plus_j2_verified=True)`.
    rmi_to_stream_j3_plus_j2_measured: bool = True  # MEASURED: the relation, and its form
    rmi_to_stream_j3_plus_j2_verified: bool = False  # per-installation: off until confirmed

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

# servo_j has NO distance window: a target is tracked however far away it is, bounded
# by the servo limits below rather than refused. A window was tried and removed — it is
# measured against the COMMANDED pose, which trails a streamed plan by the tracker's own
# response time, so at speed ordinary tracking lag ate it and the guard began discarding
# good setpoints (295 rejects on a 20 °/s ramp; total stall at 60 °/s). See NO DISTANCE
# GUARD in tick_engine/servo.hpp.
SERVO_LIMIT_SCALE: float = 1.0

# Point-to-point MoveJ planning envelope (FanucDriver.move_j). Like the brake scales
# above, these are FRACTIONS of the profile's limits rather than limits themselves, so
# they carry across arms unchanged.
#
# They are deliberately below 1.0. The profile's a/j are the ceiling the tick engine
# REFUSES to pass a command through, so a plan shaped at the ceiling gets clipped by
# ordinary numerical slop — and a clipped plan is no longer the profile that was
# checked. Jerk is scaled harder than acceleration for the same reason the brake scales
# it harder: the CRX collaborative-stop monitor infers contact force from motor
# disturbance torque, so a sharp jerk ramp reads as a phantom contact mid-transit. Jerk
# is the trip trigger; acceleration is not. 0.375 is the "~3x acceleration rather than
# 8x" that examples/crx10ial.py prescribes for a planner feeding this driver, expressed
# against the 8x jerk clamp that file derives.
MOVEJ_LIMIT_SCALE_A: float = 0.5
MOVEJ_LIMIT_SCALE_J: float = 0.375

# Default MoveJ leading-axis speed, as a fraction of the profile's SLOWEST joint
# velocity limit, when the caller names no speed. Matches airo-robots'
# PositionManipulator default of min(max_joint_speeds) / 4.
MOVEJ_DEFAULT_SPEED_FRACTION: float = 0.25

# Graduated RX-silence response, mid-TRAJECTORY:
RX_SILENCE_BLIND_HOLD_MS: float = 100.0  # → kill-type entry: blind qd-ramp to hold
RX_SILENCE_QD_RAMP_MS: float = 60.0  # duration of the qd → 0 ramp
RX_SILENT_PARK_MS: float = 500.0  # → RX_SILENT fault, park TX

# Anti-flap dwell: DEGRADED → STREAMING requires all-clear sustained ≥ this
# (covers measured ~300 ms contact-stop / motion_possible bit-skew).
ANTIFLAP_DWELL_MS: float = 500.0

# Plan staleness ceiling. A submission may declare the commanded tick its first knot was
# built from (StreamCore.submit_trajectory's plan_tick), and the core then joins the plan
# that many ticks in rather than splicing back to a knot the arm has already passed. This
# bounds how much of a plan that may skip: 200 ms is 25 ticks, an order of magnitude above
# the measured cost of the Python submission path (tens of microseconds) and still short
# enough that the skipped opening is motion the arm demonstrably just made. Beyond it the
# submission is refused instead.
MAX_PLAN_STALE_MS: float = 200.0

# qd_end blend: on trajectory exhaustion with non-zero terminal velocity, one
# Hermite blend (q_end,qd_end) → (q_end,0) over at least this long.
QD_END_BLEND_MIN_MS: float = 25.0

# Calibration stillness gate — the hand-eye capture ground truth.
CALIB_STILLNESS_DEG_S: float = 0.1
CALIB_LSQ_WINDOW_S: float = 0.5

# ---------------------------------------------------------------------------
# In-process safety watchdog. SUPERVISOR_LOST is raised by the C++ RT core, and it
# exists because of the measurements above: host death rides a ~120 ms controller
# coast, so the controller is not a fast host-death backstop.
# Mirrored in rt_core_config.hpp.
# ---------------------------------------------------------------------------

# SUPERVISOR_LOST: the RT core latches this + holds if the Python supervisor's
# heartbeat lapses for this long while streaming. The supervisor beats from a
# DEDICATED lightweight thread (no RMI, no locks), so this fires only on genuine
# supervisor/process death — not on a slow RMI round-trip. Generous vs the ~100 ms
# beat interval to tolerate GIL-storm / GC pauses without a false trip. seconds.
SUPERVISOR_LOST_S: float = 3.0
