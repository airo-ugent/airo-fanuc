// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — Ruckig-online brake (P3a). PLAN.md decision 5 + R1 C1.
//
// A universal preempt: Ruckig<6> in VELOCITY control interface driving every
// joint velocity → 0, ONE update() per 8 ms tick (µs, allocation-free — the
// Ruckig<6> objects are members, reused across ticks; StandardVector<double,6>
// is std::array so no heap).
//
// Envelope (radians):  a = STOP_LIMIT_SCALE_VA·a_lim,  j = STOP_LIMIT_SCALE_J·j_lim,
//   v_max = max(STOP_LIMIT_SCALE_VA·v_lim, 1.01·|qd_cmd|) per joint.
//
// SEED (R1 C1 — the important part): current_position/current_velocity from the
// COMMANDED (q_cmd, qd_cmd); current_acceleration from the interpolator's
// ANALYTIC second derivative at the brake tick (the WIRE qdd — see
// TrajectorySampler). dries braked at future knot boundaries and seeded accel=0;
// we brake MID-SEGMENT where qdd peaks, and an accel discontinuity latches the
// CRX collaborative contact-stop (~0.1° setpoint step). Seeding the analytic
// qdd makes the commanded acceleration continuous across the traj→brake handoff.
// (This SUPERSEDES the old dries "max-mag(planned, measured qd)" seed.)
//
// Done when Ruckig reports Finished AND |qd_meas|∞ < brake_settle_vel_eps
// (1°/s); hard cap brake_max_duration_s (2 s). Ruckig Result::Error is returned
// to the caller (P3b turns it into a slew-limited hold + fault) — never thrown.

#pragma once

#include <ruckig/ruckig.hpp>

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::tick_engine {

struct BrakeStep {
  Vec6 q{};
  Vec6 qd{};
  Vec6 qdd{};
  ruckig::Result result{ruckig::Result::Working};
  bool finished{false};  // Ruckig reached target velocity (=0) this tick
  bool error{false};     // Result::Error / ErrorInvalidInput on the RT path
};

class Brake {
 public:
  explicit Brake(const TickEngineConfig& cfg);

  // Seed the brake at entry from the COMMANDED state + analytic wire qdd.
  // Call once when transitioning into BRAKE; then call step() each tick.
  void seed(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& qdd_cmd);

  // One tick: advances the Ruckig velocity profile by itp_s. Alloc-free.
  BrakeStep step();

  // --- termination helpers (pure; caller supplies measured velocity) ---
  bool ruckig_finished() const { return finished_; }
  // |qd_meas|∞ < brake_settle_vel_eps.
  bool velocity_settled(const Vec6& qd_meas) const;
  double elapsed_s() const { return static_cast<double>(elapsed_ticks_) * cfg_.itp_s; }
  bool timed_out() const { return elapsed_s() >= cfg_.brake_max_duration_s; }
  // The full done predicate (decision 5): Finished ∧ |qd_meas|<eps, OR cap hit.
  bool done(const Vec6& qd_meas) const {
    return (finished_ && velocity_settled(qd_meas)) || timed_out();
  }
  int elapsed_ticks() const { return elapsed_ticks_; }

 private:
  TickEngineConfig cfg_;
  ruckig::Ruckig<kNumJoints> otg_;
  ruckig::InputParameter<kNumJoints> inp_;
  ruckig::OutputParameter<kNumJoints> out_;
  int elapsed_ticks_{0};
  bool finished_{false};
  bool seeded_{false};
};

}  // namespace airo_fanuc::tick_engine
