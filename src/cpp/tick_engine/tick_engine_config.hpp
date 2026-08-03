// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — RT tick-engine configuration.
//
// Two kinds of knob live here, and the difference matters:
//
//   * The kinematic `Limits` belong to the ARM. Their defaults are synthetic (see
//     the note on the struct) and `DriverConfig.to_rt_core_config` overwrites them
//     from the caller's `airo_fanuc.robot_profile.RobotProfile`, along with the brake
//     scales and slew factor that multiply them.
//   * Everything else is this driver's tuning, single-sourced in
//     `airo_fanuc.controller_facts`; each such field carries a comment naming the
//     symbol it mirrors. If you change a value there that a C++ test hardcodes, grep
//     `controller_facts` in the C++ tree and update the mirror comment + test.
//
// The Python pre-flight capture synthesis is handed the same RtCoreConfig the core
// was constructed with, so the path it checks is bounded by the same numbers the RT
// loop executes.
//
// UNIT CONVENTION (binding): the engine works entirely in RADIANS — the same unit a
// `RobotProfile` stores (rad, rad/s, rad/s², rad/s³) and the unit of curobo
// trajectories. The Stream Motion wire is DEGREES; that conversion happens ONLY at
// the wire, in `RealtimeCore` either side of the codec call — the codec itself
// converts nothing. Every C++ unit test states rad.

#pragma once

#include <array>
#include <cstdint>
#include <limits>

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

// Degrees↔radians at compile time (kept local so the header has no <cmath> dep
// for a constexpr; matches numpy.deg2rad to double precision). `rad2deg` is the
// wire-boundary direction and belongs here beside its inverse rather than as a
// factor re-derived where it is used.
inline constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double deg2rad(double deg) { return deg * (kPi / 180.0); }
constexpr double rad2deg(double rad) { return rad * (180.0 / kPi); }

// ---------------------------------------------------------------------------
// Kinematic limits (radians) — the ceiling every stage clamps against.
//
// These belong to the ARM, and no arm's numbers are compiled in: the values below
// are a SYNTHETIC envelope, round and deliberately not any real robot's, so that the
// tick-engine math is exercisable by the stand-alone gtest suite without a Python
// layer. In a driver they are always overwritten —
// `DriverConfig.to_rt_core_config()` sets all three from the caller's
// `airo_fanuc.robot_profile.RobotProfile` before the core is constructed.
//
// The 2×/8× shape (a = 2·v, j = 8·a) matches the ratio a cobot profile tends to
// have, which keeps the Ruckig profiles here as well-conditioned as they are in
// production. C++ tests must derive their expectations from `cfg.limits` rather than
// restating numbers, so that changing this envelope cannot quietly invalidate them.
// ---------------------------------------------------------------------------
inline constexpr double kInf = std::numeric_limits<double>::infinity();

struct Limits {
  Vec6 v{{2.0, 2.0, 2.0, 2.0, 2.0, 2.0}};
  Vec6 a{{4.0, 4.0, 4.0, 4.0, 4.0, 4.0}};        // 2·v
  Vec6 j{{32.0, 32.0, 32.0, 32.0, 32.0, 32.0}};  // 8·a

  // Joint position limits (the arm's soft stops). Unlike v/a/j these default to
  // ±infinity — INERT — rather than to a synthetic envelope: a wrong position limit
  // is not a conservative error the way a wrong velocity limit is. Too narrow and the
  // arm silently refuses to reach poses it can reach; too wide and the clamp is a
  // no-op. There is no defensible stand-in for the real values, so an unconfigured
  // core does not pretend to have them. A driver sets them from its RobotProfile,
  // whose values are read from the controller's $PARAM_GROUP and cross-checked at
  // preflight, so in practice they are always the arm's own.
  Vec6 pos_lo{{-kInf, -kInf, -kInf, -kInf, -kInf, -kInf}};
  Vec6 pos_hi{{kInf, kInf, kInf, kInf, kInf, kInf}};
};

// ---------------------------------------------------------------------------
// TickEngineConfig — every knob the pure tick-engine math needs. Alloc-free;
// copied by value into the mode objects at construction.
// ---------------------------------------------------------------------------
struct TickEngineConfig {
  double itp_s{kItpSeconds};  // controller_facts.ITP_S (8 ms tick)

  Limits limits{};  // synthetic default; set from the caller's RobotProfile in a driver.

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
