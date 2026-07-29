// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — RT tick-engine configuration (P3a).
//
// This struct MIRRORS the single-sourced Python constants in
// `airo_fanuc.controller_facts` (see PLAN.md §5.1 "limits are single-sourced").
// The DEFAULTS below equal those Python values; each field carries a comment
// naming the `controller_facts` symbol it mirrors. In a later phase the Python
// `DriverConfig` will POPULATE this struct from `controller_facts` and pass it
// into the C++ core; for now the C++ defaults carry the numbers so the pure
// tick-engine math is testable stand-alone. If you change a value in
// `controller_facts.py` that a C++ test hardcodes, grep `controller_facts` in
// the C++ tree and update the mirror comment + test.
//
// UNIT CONVENTION (binding): the engine works entirely in RADIANS — the same
// unit as the limits in `controller_facts` (rad, rad/s, rad/s², rad/s³) and the
// unit of curobo trajectories. The Stream Motion wire is DEGREES; that
// conversion happens ONLY at the wire in P3b. Every L1 golden states rad.

#pragma once

#include <array>
#include <cstdint>

namespace airo_fanuc::tick_engine {

// Six joints on the CRX-10iA/L; the wire format is also fixed at 6 (+3 zero-pad
// applied at the wire in P3b). Mirrors interpolator.py::NDOF.
inline constexpr int kNumJoints = 6;

// A per-joint quantity in the engine's internal radian units. This is exactly
// ruckig's `StandardVector<double, 6>` (= std::array<double, 6>), so Vec6 can be
// assigned to/from ruckig InputParameter/OutputParameter fields directly.
using Vec6 = std::array<double, kNumJoints>;

// Interpolation period of the R-30iB controller: 8 ms ITP = 125 Hz.
// Mirrors controller_facts.ITP_S.
inline constexpr double kItpSeconds = 0.008;

// Degrees→radians at compile time (kept local so the header has no <cmath> dep
// for a constexpr; matches numpy.deg2rad to double precision).
inline constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double deg2rad(double deg) { return deg * (kPi / 180.0); }

// ---------------------------------------------------------------------------
// Kinematic limits (radians). Default = CRX-10iA/L datasheet-derived values,
// single-sourced in controller_facts. Provenance: FANUC Europe MDS-04018
// (velocity); accel = 2×vel, jerk = 8×accel (engineering-practice derivations).
//   v : controller_facts.CRX10IAL_VELOCITY_LIMITS
//   a : controller_facts.CRX10IAL_ACCELERATION_LIMITS (= 2·v)
//   j : controller_facts.CRX10IAL_JERK_LIMITS         (= 8·a)
// ---------------------------------------------------------------------------
struct Limits {
  Vec6 v{{2.094, 2.094, 3.142, 3.142, 3.142, 3.142}};
  Vec6 a{{4.188, 4.188, 6.284, 6.284, 6.284, 6.284}};        // 2·v
  Vec6 j{{33.504, 33.504, 50.272, 50.272, 50.272, 50.272}};  // 8·a
};

// ---------------------------------------------------------------------------
// TickEngineConfig — every knob the pure tick-engine math needs. Alloc-free;
// copied by value into the mode objects at construction.
// ---------------------------------------------------------------------------
struct TickEngineConfig {
  double itp_s{kItpSeconds};  // controller_facts.ITP_S (8 ms tick)

  Limits limits{};  // CRX-10iA/L defaults; DriverConfig overrides in a later phase.

  // Brake / stop envelope scales (PLAN.md decision 5; dries B15 v/a-vs-jerk split).
  double stop_scale_va{0.4};  // controller_facts.STOP_LIMIT_SCALE_VA
  double stop_scale_j{0.15};  // controller_facts.STOP_LIMIT_SCALE_J

  // Per-tick position slew clip factor: |Δq| ≤ slew_factor·v_lim·itp_s per joint.
  double slew_factor{1.2};  // controller_facts.SLEW_FACTOR
  // Number of consecutive clipping ticks that raise the "sustained-clip" flag.
  // NOT a controller_facts constant — chosen default; flagged for review. The
  // slew clip NEVER faults; this only sets a diagnostic condition bit (F35).
  int slew_sustained_ticks{3};

  // Brake settle / cap (decision 5 + R1 C1). Brake is done when Ruckig reports
  // Finished AND |qd_meas|∞ < brake_settle_vel_eps; hard cap brake_max_duration_s.
  double brake_settle_vel_eps_rad_s{deg2rad(1.0)};  // 1°/s (dries BrakeTarget eps)
  double brake_max_duration_s{2.0};                 // dries BrakeTarget.DEFAULT_MAX_DURATION_SEC

  // Capture-or-reject splice (decision 6 / R3 A2). Deterministic Ruckig from
  // commanded (q_cmd,qd_cmd) → (q0,qd0) at capture_rate per joint, brake-class a/j.
  double capture_rate_rad_s{deg2rad(15.0)};  // controller_facts.CAPTURE_RATE_DEG_S
  double capture_tol_rad{deg2rad(5.0)};      // controller_facts.CAPTURE_TOL_DEG (reject beyond)

  // Servo (R1 C2/C3). Ruckig online-position; limits × servo_limit_scale.
  double servo_limit_scale{1.0};           // controller_facts.SERVO_LIMIT_SCALE
  double servo_window_rad{deg2rad(5.0)};   // controller_facts.SERVO_WINDOW_DEG (distance guard)

  // qd_end blend (R1 B3): on trajectory exhaustion with |qd_end|>0, ramp
  // (q_end,qd_end)→rest over at least this long.
  double qd_end_blend_min_s{0.025};  // controller_facts.QD_END_BLEND_MIN_MS

  // End-of-trajectory settle (SettlePolicy; PLAN.md §5.1). Production call sites
  // override these (dries used 1.5°/1.5-3 s); H11 re-measures.
  double settle_tol_rad{deg2rad(0.5)};        // SettlePolicy.tol_deg
  double settle_vel_eps_rad_s{deg2rad(2.0)};  // SettlePolicy.vel_eps_deg_s
  double settle_timeout_s{2.0};               // SettlePolicy.timeout_s
};

}  // namespace airo_fanuc::tick_engine
