// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — RT tick-engine configuration.
//
// Limits are single-sourced in `airo_fanuc.controller_facts`; this struct MIRRORS
// those constants and each field carries a comment naming the symbol it mirrors.
// The DEFAULTS below are authoritative for the tick engine:
// `DriverConfig.to_rt_core_config` deliberately leaves these fields alone (it sets
// only the protocol + RT-hygiene knobs Python owns), so the numbers the RT loop
// executes are the same numbers the Python pre-flight capture synthesis sees
// through the binding. Keeping them here also makes the pure tick-engine math
// testable stand-alone. If you change a value in `controller_facts.py` that a C++ test
// hardcodes, grep `controller_facts` in the C++ tree and update the mirror
// comment + test.
//
// UNIT CONVENTION (binding): the engine works entirely in RADIANS — the same
// unit as the limits in `controller_facts` (rad, rad/s, rad/s², rad/s³) and the
// unit of curobo trajectories. The Stream Motion wire is DEGREES; that
// conversion happens ONLY at the wire, in the codec. Every C++ unit test states
// rad.

#pragma once

#include <array>
#include <cstdint>

namespace airo_fanuc::tick_engine {

// Six joints on the CRX-10iA/L. The Stream Motion wire carries 9 axis slots
// (kMaxAxisNumber); the trailing 3 are zero-filled by the caller that builds the
// command packet.
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

  Limits limits{};  // CRX-10iA/L defaults (mirror of controller_facts).

  // Brake / stop envelope scales. v/a and jerk are scaled SEPARATELY, and jerk is
  // scaled much harder (0.15 vs 0.4), because the CRX collaborative-stop monitor
  // infers contact force from motor disturbance torque: a steep jerk ramp produces
  // the same torque signature as a real contact and trips a phantom collaborative
  // stop mid-brake. Scaling jerk down while leaving v/a comparatively free gives a
  // brake that still stops quickly but never looks like a collision. Do not
  // collapse these into one scale factor.
  double stop_scale_va{0.4};  // controller_facts.STOP_LIMIT_SCALE_VA
  double stop_scale_j{0.15};  // controller_facts.STOP_LIMIT_SCALE_J

  // Per-tick position slew clip factor: |Δq| ≤ slew_factor·v_lim·itp_s per joint.
  double slew_factor{1.2};  // controller_facts.SLEW_FACTOR
  // Number of consecutive clipping ticks that raise the "sustained-clip" flag.
  // NOT a controller_facts constant — a local diagnostic threshold. The slew clip
  // NEVER faults; this only sets a condition bit in the snapshot.
  int slew_sustained_ticks{3};

  // Brake settle / cap. Brake is done when Ruckig reports Finished AND
  // |qd_meas|∞ < brake_settle_vel_eps; hard cap brake_max_duration_s so a brake
  // that never settles (e.g. against an obstruction) still terminates.
  double brake_settle_vel_eps_rad_s{deg2rad(1.0)};  // 1°/s
  double brake_max_duration_s{2.0};

  // Capture-or-reject splice. Deterministic Ruckig from commanded (q_cmd,qd_cmd) →
  // (q0,qd0) at capture_rate per joint, brake-class a/j.
  double capture_rate_rad_s{deg2rad(15.0)};  // controller_facts.CAPTURE_RATE_DEG_S
  double capture_tol_rad{deg2rad(5.0)};      // controller_facts.CAPTURE_TOL_DEG (reject beyond)

  // Servo. Ruckig online-position; limits × servo_limit_scale.
  double servo_limit_scale{1.0};           // controller_facts.SERVO_LIMIT_SCALE
  double servo_window_rad{deg2rad(5.0)};   // controller_facts.SERVO_WINDOW_DEG (distance guard)

  // qd_end blend: on trajectory exhaustion with |qd_end|>0, ramp
  // (q_end,qd_end)→rest over at least this long.
  double qd_end_blend_min_s{0.025};  // controller_facts.QD_END_BLEND_MIN_MS

  // End-of-trajectory settle (mirrors controller_facts.SettlePolicy). These are
  // defaults; `submit_trajectory` takes per-motion overrides, so a call site with a
  // looser tolerance or a longer wait passes its own.
  double settle_tol_rad{deg2rad(0.5)};        // SettlePolicy.tol_deg
  double settle_vel_eps_rad_s{deg2rad(2.0)};  // SettlePolicy.vel_eps_deg_s
  double settle_timeout_s{2.0};               // SettlePolicy.timeout_s
};

}  // namespace airo_fanuc::tick_engine
