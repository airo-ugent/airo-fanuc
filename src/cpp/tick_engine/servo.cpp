// SPDX-License-Identifier: Apache-2.0
//
// Ruckig-online servo — see servo.hpp.

#include "tick_engine/servo.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace airo_fanuc::tick_engine {

Servo::Servo(const TickEngineConfig& cfg) : cfg_(cfg), otg_(cfg.itp_s) {}

void Servo::start(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& qdd_cmd) {
  inp_.control_interface = ruckig::ControlInterface::Position;
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    inp_.max_velocity[j] = cfg_.servo_limit_scale * cfg_.limits.v[j];
    inp_.max_acceleration[j] = cfg_.servo_limit_scale * cfg_.limits.a[j];
    inp_.max_jerk[j] = cfg_.servo_limit_scale * cfg_.limits.j[j];
  }
  prev_target_ = q_cmd;

  // Initial state = current commanded; initial target = hold at q_cmd.
  inp_.current_position = q_cmd;
  inp_.current_velocity = qd_cmd;
  inp_.current_acceleration = qdd_cmd;
  inp_.target_position = q_cmd;
  inp_.target_velocity = Vec6{};
  inp_.target_acceleration = Vec6{};
  inp_.minimum_duration = std::nullopt;
  active_ = true;
}

ServoAccept Servo::set_target(const Vec6& q_target, double duration_s) {
  // Distance guard (R1 C3): compare against the CURRENT commanded position
  // (inp_.current_position is kept current by pass_to_input each step()).
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    if (std::abs(q_target[j] - inp_.current_position[j]) > cfg_.servo_window_rad) {
      return ServoAccept::kRejectedDistance;
    }
  }

  const double inv_dur = (duration_s > 0.0) ? (1.0 / duration_s) : 0.0;
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    // Velocity feedforward = clamp((q_new − q_prev)/duration, ±v_lim·scale).
    const double v_lim = cfg_.servo_limit_scale * cfg_.limits.v[j];
    double ff = (q_target[j] - prev_target_[j]) * inv_dur;
    ff = std::clamp(ff, -v_lim, v_lim);
    inp_.target_velocity[j] = ff;
  }
  inp_.target_position = q_target;
  inp_.target_acceleration = Vec6{};
  // minimum_duration = the call's duration (== servo update period == the
  // "servo period" of R1 C2 == PLAN.md §5.1). It BINDS only when the
  // time-optimal profile would be shorter than `duration` — i.e. exactly the
  // reach-early case it is meant to fix (kills the freeze-until-next-target
  // sawtooth). Replace-not-queue: this just updates the target; the next step()
  // re-plans from the current commanded state (Ruckig online).
  inp_.minimum_duration = (duration_s > 0.0) ? std::optional<double>(duration_s) : std::nullopt;

  prev_target_ = q_target;
  return ServoAccept::kOk;
}

ServoAccept Servo::set_target(const Vec6& q_target, const Vec6& qd_target,
                              const Vec6& qdd_target, double duration_s) {
  // Distance guard (R1 C3) — identical to the position-only overload.
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    if (std::abs(q_target[j] - inp_.current_position[j]) > cfg_.servo_window_rad) {
      return ServoAccept::kRejectedDistance;
    }
  }

  // Feed-forward directly from the caller's planned derivatives (clamped to the servo
  // limits), instead of the secant reconstruction + zero target acceleration. This is
  // what lets Ruckig reproduce a smooth externally-planned trajectory faithfully.
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    const double v_lim = cfg_.servo_limit_scale * cfg_.limits.v[j];
    const double a_lim = cfg_.servo_limit_scale * cfg_.limits.a[j];
    inp_.target_velocity[j] = std::clamp(qd_target[j], -v_lim, v_lim);
    inp_.target_acceleration[j] = std::clamp(qdd_target[j], -a_lim, a_lim);
  }
  inp_.target_position = q_target;
  inp_.minimum_duration = (duration_s > 0.0) ? std::optional<double>(duration_s) : std::nullopt;

  prev_target_ = q_target;
  return ServoAccept::kOk;
}

ServoStep Servo::step() {
  ServoStep s{};
  if (!active_) {
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
    // Feed the new state back as the current state (canonical Ruckig online
    // pattern). This is what advances the trajectory; without it Ruckig
    // re-plans from the same start every tick (frozen output).
    out_.pass_to_input(inp_);
  } else {
    s.error = true;
    s.q = inp_.current_position;  // hold last commanded; P3b faults + slew-limited hold
    s.qd = Vec6{};
    s.qdd = Vec6{};
  }
  return s;
}

}  // namespace airo_fanuc::tick_engine
