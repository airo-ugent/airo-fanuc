// SPDX-License-Identifier: Apache-2.0
//
// Ruckig-online brake — see brake.hpp.

#include "tick_engine/brake.hpp"

#include <algorithm>
#include <cmath>

namespace airo_fanuc::tick_engine {

namespace {
// 1% margin above |qd_cmd| for the velocity envelope. Mirrors the dries
// BrakeTarget comment: Ruckig's strict |current_velocity| < max_velocity
// precondition (kept for safety even though 0.17.3's update() does not check
// current-state limits by default; harmless and prescribed by decision 5).
constexpr double kVMaxSeedMargin = 1.01;
}  // namespace

Brake::Brake(const TickEngineConfig& cfg) : cfg_(cfg), otg_(cfg.itp_s) {}

void Brake::seed(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& qdd_cmd) {
  inp_.control_interface = ruckig::ControlInterface::Velocity;

  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    const double v_floor = cfg_.stop_scale_va * cfg_.limits.v[j];
    inp_.max_velocity[j] = std::max(v_floor, std::abs(qd_cmd[j]) * kVMaxSeedMargin);
    inp_.max_acceleration[j] = cfg_.stop_scale_va * cfg_.limits.a[j];
    inp_.max_jerk[j] = cfg_.stop_scale_j * cfg_.limits.j[j];
  }

  // Seed from COMMANDED state + analytic wire qdd (R1 C1).
  inp_.current_position = q_cmd;
  inp_.current_velocity = qd_cmd;
  inp_.current_acceleration = qdd_cmd;
  inp_.target_velocity = Vec6{};      // → 0
  inp_.target_acceleration = Vec6{};  // → 0

  elapsed_ticks_ = 0;
  finished_ = false;
  seeded_ = true;
}

BrakeStep Brake::step() {
  BrakeStep s{};
  if (!seeded_) {
    // No seed → hold nothing meaningful; report error so the caller faults.
    s.error = true;
    s.result = ruckig::Result::Error;
    return s;
  }

  const ruckig::Result r = otg_.update(inp_, out_);
  s.result = r;

  if (r == ruckig::Result::Working || r == ruckig::Result::Finished) {
    s.q = out_.new_position;
    s.qd = out_.new_velocity;
    s.qdd = out_.new_acceleration;
    s.finished = (r == ruckig::Result::Finished);
    finished_ = s.finished;
    out_.pass_to_input(inp_);
    ++elapsed_ticks_;
  } else {
    // RT-path error: do NOT throw. Hold the last commanded position at rest;
    // P3b converts this into a slew-limited hold + FAULTED.
    s.error = true;
    s.q = inp_.current_position;
    s.qd = Vec6{};
    s.qdd = Vec6{};
  }
  return s;
}

bool Brake::velocity_settled(const Vec6& qd_meas) const {
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    if (std::abs(qd_meas[j]) >= cfg_.brake_settle_vel_eps_rad_s) {
      return false;
    }
  }
  return true;
}

}  // namespace airo_fanuc::tick_engine
