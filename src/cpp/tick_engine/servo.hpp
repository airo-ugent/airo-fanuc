// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — Ruckig-online servo.
//
// Replace-not-queue servoing: Ruckig<6> in POSITION control interface, ONE
// update() per tick (alloc-free — members reused). Each new target REPLACES the
// Ruckig target (no queue); starvation-safe by construction (with no new target
// the profile converges to the last target and rests).
//
//   limits          = (v_lim, a_lim, j_lim) × servo_limit_scale (default 1.0)
//   target_position = latest servo q
//   target_velocity = clamp((q_new − q_prev)/duration, ±v_lim·scale)  [feedforward]
//   minimum_duration = duration                              [see STUTTER FIX below]
//
// STUTTER FIX. A time-optimal profile reaches a `duration`-spaced target EARLY and
// then sits at rest until the next target arrives — a freeze/move sawtooth the arm
// renders as visible stutter. The velocity feedforward plus a minimum profile
// duration equal to `duration` remove the dwell: the profile is stretched to
// exactly fill the gap between targets and arrives moving.
//
// `duration` here is the servo UPDATE period — the spacing between successive
// set_target calls, i.e. 1/f in the `servo_j(q, 1/f)` call pattern. It is NOT the
// 8 ms RT tick: stretching a profile to the TARGET SPACING is what removes the
// sawtooth, stretching it to 8 ms would not.
//
// ONLINE PATTERN (important, verified empirically): the Ruckig target/limits/
// minimum_duration are set ONLY when a new target arrives (`set_target`); each
// `step()` then just advances `update()` along the trajectory Ruckig planned
// when the input last changed (Ruckig re-plans ONLY when the input changes).
// We do NOT call `pass_to_input` every tick — doing so mutates current_position
// each tick, forcing a fresh minimum_duration-stretched re-plan against the
// fixed target every tick, which contorts the profile (drives the wrong way).
// So `set_target` seeds current_* from the last commanded state; `step()` leaves
// the input untouched.
//
// DISTANCE GUARD. Typed reject if |q_target − q_cmd|∞ > servo_window (5°), which
// keeps the servo API consistent with the capture-or-reject policy. A servo call
// that silently accepts an arbitrarily distant target is a foot-gun: one mistaken
// or stale setpoint becomes a full-envelope lunge, planned at the servo limits,
// with no collision check anywhere in the path. Rejecting is the safe default —
// callers that genuinely want to travel that far submit a trajectory.

#pragma once

#include <ruckig/ruckig.hpp>

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::tick_engine {

enum class ServoAccept {
  kOk,
  kRejectedDistance,  // |q_target − q_cmd|∞ > servo_window_rad
};

struct ServoStep {
  Vec6 q{};
  Vec6 qd{};
  Vec6 qdd{};
  ruckig::Result result{ruckig::Result::Working};
  bool error{false};
};

class Servo {
 public:
  explicit Servo(const TickEngineConfig& cfg);

  // Enter SERVO from the current commanded state, so position and velocity are
  // continuous across the mode transition. Initial target
  // = q_cmd (hold) with zero feedforward until the first set_target.
  void start(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& qdd_cmd);

  // Replace the servo target. Rejects (no state change) if the target is farther
  // than servo_window from the CURRENT commanded position. `duration_s` shapes
  // both the velocity feedforward and Ruckig's minimum profile duration.
  ServoAccept set_target(const Vec6& q_target, double duration_s);

  // Feed-forward variant: the caller supplies the target velocity AND acceleration
  // (e.g. the qd/qdd of an MPC action-sequence knot) rather than the secant velocity +
  // zero acceleration the position-only overload reconstructs. Ruckig then plans to
  // arrive at (q_target, qd_target, qdd_target), so a smooth externally-planned
  // trajectory is rendered faithfully — no secant lag, no accel=0 forcing at each knot
  // (the servoing twitch). qd/qdd are clamped to ±(v_lim/a_lim)·servo_limit_scale.
  // Same distance guard + minimum_duration semantics as the position-only overload.
  ServoAccept set_target(const Vec6& q_target, const Vec6& qd_target,
                         const Vec6& qdd_target, double duration_s);

  // One tick. Alloc-free. Advances the cached trajectory; does NOT re-plan.
  ServoStep step();

  // Current commanded position (= what the distance guard compares against).
  // Kept current by pass_to_input each step().
  const Vec6& commanded() const { return inp_.current_position; }

 private:
  TickEngineConfig cfg_;
  ruckig::Ruckig<kNumJoints> otg_;
  ruckig::InputParameter<kNumJoints> inp_;
  ruckig::OutputParameter<kNumJoints> out_;
  Vec6 prev_target_{};
  bool active_{false};
};

}  // namespace airo_fanuc::tick_engine
