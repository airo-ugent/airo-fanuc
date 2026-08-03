// SPDX-License-Identifier: Apache-2.0
//
// Ruckig-online servo — see servo.hpp.

#include "tick_engine/servo.hpp"

#include <algorithm>
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

void Servo::set_target(const Vec6& q_target, double duration_s) {
  // Clamp the destination into the arm's soft limits. This is a real clamp, not a
  // reject: best effort means a setpoint past a stop is followed as far as the stop
  // and no further, which is also what makes it safe to stream IK solutions that
  // occasionally wander out of range. The commanded output is clamped again on the
  // way to the wire (tick_core), because a profile aiming AT a limit can still
  // overshoot it in transit.
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    inp_.target_position[j] = std::clamp(q_target[j], cfg_.limits.pos_lo[j], cfg_.limits.pos_hi[j]);
  }
  // No arrival state is demanded — see BEST EFFORT in servo.hpp. Written on every
  // target rather than once in start(), so the property holds for the whole stream
  // and not merely until something else touches the input.
  inp_.target_velocity = Vec6{};
  inp_.target_acceleration = Vec6{};
  // minimum_duration = the call's duration (== the servo update period, i.e. the
  // spacing between set_target calls). It BINDS whenever the time-optimal profile
  // would be shorter than `duration`, which for the small steps a stream is made of
  // is essentially always — and that is what keeps the command gliding between
  // targets instead of arriving early and dwelling (the sawtooth). Replace-not-queue:
  // this just updates the target, which is the ONLY thing that makes Ruckig
  // recalculate; the 2-3 ticks that follow sample the plan it produces.
  inp_.minimum_duration = (duration_s > 0.0) ? std::optional<double>(duration_s) : std::nullopt;
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
    // pattern), so the next set_target plans from where the command actually is.
    // It does NOT by itself force a recalculation — Ruckig reconciles the state it
    // produced itself, so a tick with no fresh target just advances the existing
    // plan (measured: recalculations == targets, not == ticks).
    out_.pass_to_input(inp_);
  } else {
    s.error = true;
    s.q = inp_.current_position;  // hold last commanded; the tick core faults + slew-limited hold
    s.qd = Vec6{};
    s.qdd = Vec6{};
  }
  return s;
}

}  // namespace airo_fanuc::tick_engine
