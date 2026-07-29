// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — end-of-motion settle detection (P3a). PLAN.md §5.1 SettlePolicy.
//
// After a trajectory (post qd_end blend) or brake reaches its hold position,
// declare the motion DONE when the MEASURED state has converged:
//   |q_meas − q_target|∞ < settle_tol_rad   (0.5°)  AND
//   |qd_meas|∞           < settle_vel_eps    (2°/s)
// or TIMEOUT after settle_timeout_s (2 s). Pure and clock-free: the caller
// drives one update() per tick; elapsed time is tick-counted.

#pragma once

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::tick_engine {

enum class SettleState {
  kSettling,
  kDone,
  kTimeout,
};

class SettleDetector {
 public:
  explicit SettleDetector(const TickEngineConfig& cfg);

  // Begin a settle wait; resets the tick counter.
  void reset();

  // One tick. Returns kDone once converged, kTimeout once the cap is hit,
  // else kSettling. Once kDone/kTimeout is returned the state latches until reset().
  SettleState update(const Vec6& q_target, const Vec6& q_meas, const Vec6& qd_meas);

  double elapsed_s() const { return static_cast<double>(elapsed_ticks_) * cfg_.itp_s; }
  int elapsed_ticks() const { return elapsed_ticks_; }
  SettleState state() const { return state_; }

  // Pure predicate (no side effects) — is the measured state within tolerance?
  bool converged(const Vec6& q_target, const Vec6& q_meas, const Vec6& qd_meas) const;

 private:
  TickEngineConfig cfg_;
  int elapsed_ticks_{0};
  SettleState state_{SettleState::kSettling};
};

}  // namespace airo_fanuc::tick_engine
