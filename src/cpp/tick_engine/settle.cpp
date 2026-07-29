// SPDX-License-Identifier: Apache-2.0
//
// End-of-motion settle detection — see settle.hpp.

#include "tick_engine/settle.hpp"

#include <cmath>

namespace airo_fanuc::tick_engine {

SettleDetector::SettleDetector(const TickEngineConfig& cfg) : cfg_(cfg) {}

void SettleDetector::reset() {
  elapsed_ticks_ = 0;
  state_ = SettleState::kSettling;
}

bool SettleDetector::converged(const Vec6& q_target, const Vec6& q_meas, const Vec6& qd_meas) const {
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    if (std::abs(q_meas[j] - q_target[j]) >= cfg_.settle_tol_rad) {
      return false;
    }
    if (std::abs(qd_meas[j]) >= cfg_.settle_vel_eps_rad_s) {
      return false;
    }
  }
  return true;
}

SettleState SettleDetector::update(const Vec6& q_target, const Vec6& q_meas, const Vec6& qd_meas) {
  if (state_ != SettleState::kSettling) {
    return state_;  // latched
  }
  ++elapsed_ticks_;
  if (converged(q_target, q_meas, qd_meas)) {
    state_ = SettleState::kDone;
  } else if (elapsed_s() >= cfg_.settle_timeout_s) {
    state_ = SettleState::kTimeout;
  }
  return state_;
}

}  // namespace airo_fanuc::tick_engine
