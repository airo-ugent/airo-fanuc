// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — Ruckig-online brake.
//
// A universal preempt: Ruckig<6> in VELOCITY control interface driving every
// joint velocity → 0, ONE update() per 8 ms tick (µs, allocation-free — the
// Ruckig<6> objects are members, reused across ticks; StandardVector<double,6>
// is std::array so no heap).
//
// Envelope (radians):  a = STOP_LIMIT_SCALE_VA·a_lim,  j = STOP_LIMIT_SCALE_J·j_lim,
//   v_max = max(STOP_LIMIT_SCALE_VA·v_lim, 1.01·|qd_cmd|) per joint.
// v/a and jerk are scaled separately on purpose — see the stop-scale note in
// tick_engine_config.hpp.
//
// SEED (the important part). Two rules, both safety-relevant:
//
//   1. PLAN FROM THE PLANNED STATE, NEVER THE MEASURED ONE.
//      current_position/current_velocity come from the COMMANDED state
//      (q_cmd, qd_cmd). The measured velocity lags the command by the servo lag,
//      so seeding the brake from it — or from the max-magnitude of planned and
//      measured — starts the deceleration profile at a velocity the controller is
//      not actually commanding. The result is a step in the commanded velocity at
//      the traj→brake handoff, i.e. the brake itself injects the discontinuity it
//      is supposed to smooth out. Do not mix q_meas/qd_meas into the seed;
//      measured velocity is used ONLY as a termination test (velocity_settled).
//
//   2. SEED THE ANALYTIC ACCELERATION, NOT ZERO.
//      current_acceleration comes from the interpolator's ANALYTIC second
//      derivative at the brake tick (the WIRE qdd — see TrajectorySampler). A
//      brake can be entered MID-SEGMENT, where |qdd| peaks, so seeding accel = 0
//      puts a step in the commanded acceleration. The CRX collaborative
//      contact-stop monitor infers contact force from motor disturbance torque,
//      and an acceleration discontinuity produces exactly that torque signature —
//      a setpoint step of ~0.1° is enough to latch a stop that never happened.
//      Seeding the analytic qdd makes the commanded acceleration continuous
//      across the handoff.
//
// Done when Ruckig reports Finished AND |qd_meas|∞ < brake_settle_vel_eps
// (1°/s); hard cap brake_max_duration_s (2 s). Ruckig Result::Error is returned
// to the caller (the tick core turns it into a slew-limited hold + fault) —
// never thrown, because this runs on the RT path.

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
  // |qd_meas|∞ < brake_settle_vel_eps.
  bool velocity_settled(const Vec6& qd_meas) const;
  double elapsed_s() const { return static_cast<double>(elapsed_ticks_) * cfg_.itp_s; }
  bool timed_out() const { return elapsed_s() >= cfg_.brake_max_duration_s; }
  // The full done predicate: Finished ∧ |qd_meas|<eps, OR cap hit.
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
