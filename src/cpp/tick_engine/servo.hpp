// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — Ruckig-online servo.
//
// Replace-not-queue servoing: Ruckig<6> in POSITION control interface, ONE
// update() per tick (alloc-free — members reused). Each new target REPLACES the
// Ruckig target (no queue); starvation-safe by construction (with no new target
// the profile converges to the last target and rests).
//
//   limits           = (v_lim, a_lim, j_lim) × servo_limit_scale (default 1.0)
//   target_position  = latest servo q
//   target_velocity  = 0                                     [see BEST EFFORT below]
//   minimum_duration = duration                              [see STUTTER FIX below]
//
// BEST EFFORT, NOT POINT-TO-POINT. A servo command means "head for this pose, and
// you have `duration` to do it" — not "be at this pose, at this velocity, exactly
// then". So no arrival velocity is demanded: target_velocity and target_acceleration
// are always zero and the caller's qd/qdd, if any, are advisory only. This mirrors
// UR's `servoj(q, a, v, t, lookahead_time, gain)`, whose `a`/`v` are likewise unused
// — a servo target is a position plus a deadline plus a tracking law, and nothing
// else.
//
// Demanding an arrival velocity would be unstable against clock drift, which is why the
// target velocity is always zero. Targets arrive on the CALLER's clock while ticks run on the
// controller's PLL-locked one (~7.95 ms, not a round 8), so the ticks-per-target count
// alternates between 2 and 3 and its phase walks. When the phase puts the target nearer
// than the demanded arrival velocity has runway for,
// the only way to reach that position AT that velocity is to back up first — so the
// command decelerated, REVERSED against a still-advancing target, lost ~0.75° and
// recovered at ~1.8× the streamed speed, once every ~1.3 s. Measured on hardware;
// invisible whenever the two clocks are commensurate, which is why every unit test
// below (integer ticks per target) missed it. Dropping the arrival velocity removed
// it outright and cut peak overspeed from 1.75× to 1.01× and tracking error from
// 0.79° to 0.22° on a 5 °/s ramp. Regression: StreamedRampNoReversalUnderClockDrift.
//
// STUTTER FIX. A time-optimal profile reaches a `duration`-spaced target EARLY and
// then sits at rest until the next target arrives — a freeze/move sawtooth the arm
// renders as visible stutter. `minimum_duration` alone removes it, and it is what
// keeps the profile gliding now that there is no velocity feedforward to do it: each
// plan is stretched to last `duration`, which is very nearly exactly the time until
// the next target replaces it, so the profile is always mid-flight and never
// decelerates into the target. Measured at 20 °/s with 20 ms targets on ~7.95 ms
// ticks: 2.51 ticks per target, velocity ripple ±5%, no zero-velocity dwell.
//
// ONE PLAN PER TARGET, NOT PER TICK. Ruckig recalculates only when its input changes,
// and `pass_to_input` does not count — Ruckig reconciles the state it produced itself.
// Measured: 319 recalculations across 800 ticks for exactly 319 targets. So each
// target buys one plan and the 2-3 ticks that follow sample it. That is why the
// stitch is seamless rather than a splice: the per-tick position step, velocity step
// and acceleration step on a tick where a fresh target lands are indistinguishable
// from one where none does (0.1746° vs 0.1750°, 0.152 vs 0.156 °/s, both jerk-bounded
// at j_lim*itp) — a new plan starts from the state the old one just emitted.
//
// `duration` here is the servo UPDATE period — the spacing between successive
// set_target calls, i.e. 1/f in the `servo_j(q, 1/f)` call pattern. It is NOT the
// 8 ms RT tick: stretching a profile to the TARGET SPACING is what removes the
// sawtooth, stretching it to 8 ms would not.
//
// NO DISTANCE GUARD. A far target is tracked, not refused. A window on the distance from
// the commanded pose sounds protective and is not, and the numbers say so: the window would
// be measured against the COMMANDED pose, which trails a streamed plan by the tracker's own
// response time, so at speed ordinary tracking lag consumes it and the guard throws away
// perfectly good setpoints — measured at a 5° window, 295 rejects on a
// 20 °/s ramp, and at 60 °/s the arm stalled outright and never recovered. Lookahead
// cannot buy headroom either, because it moves the target away from the commanded
// pose by exactly what it removes from the lag. Rejecting also contradicts best
// effort: falling behind should mean "keep going", not "stop".
//
// What bounds a bad setpoint instead is the same thing that bounds it on a UR: the
// velocity, acceleration and jerk limits above. A stale or wrong target produces
// motion toward it at up to servo_limit_scale · v_lim until a fresh target arrives —
// fast, but bounded and jerk-shaped. Note what this does NOT bound: there is no
// collision check anywhere on the servo path (unlike move_trajectory, whose capture
// splice goes through policy.capture_check), so a caller streaming into an occupied
// workspace is responsible for its own setpoints.

#pragma once

#include <ruckig/ruckig.hpp>

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::tick_engine {

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
  // = q_cmd (hold) until the first set_target.
  void start(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& qdd_cmd);

  // Replace the servo target: head for `q_target`, with `duration_s` to get there.
  // Always accepted — there is no distance guard, and no state in which a target is
  // refused. `duration_s` sets Ruckig's minimum profile duration.
  void set_target(const Vec6& q_target, double duration_s);

  // Current commanded position. Kept current by pass_to_input each step().
  const Vec6& commanded() const { return inp_.current_position; }

  // One tick. Alloc-free.
  ServoStep step();

 private:
  TickEngineConfig cfg_;
  ruckig::Ruckig<kNumJoints> otg_;
  ruckig::InputParameter<kNumJoints> inp_;
  ruckig::OutputParameter<kNumJoints> out_;
  bool active_{false};
};

}  // namespace airo_fanuc::tick_engine
