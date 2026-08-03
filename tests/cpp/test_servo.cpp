// SPDX-License-Identifier: Apache-2.0
//
// Unit test — Ruckig-online servo. Asserts the minimum_duration behaviour (no
// reach-early freeze), no zero-velocity dwell on a streamed ramp, bounded qdd on
// a sine + a direction reversal, best-effort tracking of a far target, no reversal
// against a forward-moving stream under clock drift, the joint position clamp, and
// starvation rest via re-issue.
//
// Several of these step a whole number of ticks per target for readability. That
// pairing is the benign one — StreamedRampNoReversalUnderClockDrift deliberately
// does not, because the incommensurate case is where the interesting failures live.

#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "tick_engine/servo.hpp"
#include "tick_engine/tick_engine_config.hpp"

using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::Servo;
using airo_fanuc::tick_engine::ServoStep;
using airo_fanuc::tick_engine::TickEngineConfig;
using airo_fanuc::tick_engine::Vec6;

namespace {
Vec6 axis0(double x) { return Vec6{x, 0, 0, 0, 0, 0}; }
constexpr double kTwoPi = 2.0 * airo_fanuc::tick_engine::kPi;
}  // namespace

// minimum_duration = duration prevents the reach-early freeze sawtooth.
// A tiny move whose time-optimal profile is ≪ duration is spread across the full
// duration: at ~half the duration the command is still en route, and arrival
// takes many ticks (not 1-2).
TEST(Servo, MinimumDurationNoEarlyArrival) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  const double target = 0.002;  // rad — tiny; time-optimal ≪ duration
  const double duration = 0.05;
  servo.set_target(axis0(target), duration);

  int arrival_tick = -1;
  for (int i = 1; i <= 20; ++i) {
    const ServoStep s = servo.step();
    ASSERT_FALSE(s.error);
    if (i == 3) {  // ~24 ms, roughly half the 50 ms duration
      EXPECT_GT(s.q[0], 0.0);
      EXPECT_LT(s.q[0], 0.8 * target) << "still en route at mid-duration (no early arrival)";
      EXPECT_GT(s.qd[0], 0.0) << "still moving (no freeze)";
    }
    if (arrival_tick < 0 && std::abs(s.q[0] - target) < 2e-4) {
      arrival_tick = i;
    }
  }
  EXPECT_GE(arrival_tick, 5) << "target reached near `duration`, not in 1-2 ticks (the sawtooth bug)";
  EXPECT_LE(arrival_tick, 15);
}

// No zero-velocity dwell on a gentle constant-velocity stream. After warmup the
// commanded velocity holds near V and never collapses toward zero.
TEST(Servo, ConstantVelocityStreamNoDwell) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  const double V = 0.15;         // rad/s (gentle → feasible feedforward)
  const double duration = 0.04;  // 5 ticks
  const int ticks_per = 5;
  const double delta = V * duration;  // a small step per target

  int global_tick = 0;
  for (int m = 1; m <= 40; ++m) {
    servo.set_target(axis0(static_cast<double>(m) * delta), duration);
    for (int k = 0; k < ticks_per; ++k) {
      const ServoStep s = servo.step();
      ASSERT_FALSE(s.error);
      ++global_tick;
      if (global_tick > 60) {  // steady state
        EXPECT_GT(s.qd[0], 0.10) << "never freezes mid-stream (no sawtooth dwell), tick " << global_tick;
        EXPECT_NEAR(s.qd[0], V, 0.06) << "tracks the stream velocity, tick " << global_tick;
      }
    }
  }
}

// Bounded velocity/acceleration + tracking on a gentle sine of targets.
TEST(Servo, SineTrackingBounded) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  const double amp = 0.03;
  const double freq = 0.3;        // Hz
  const double duration = 0.04;   // target spacing
  const int ticks_per = 5;

  double t = 0.0;
  double max_err = 0.0;
  int gt = 0;
  for (int m = 1; m <= 120; ++m) {
    const double q_target = amp * std::sin(kTwoPi * freq * static_cast<double>(m) * duration);
    servo.set_target(axis0(q_target), duration);
    for (int k = 0; k < ticks_per; ++k) {
      const ServoStep s = servo.step();
      ASSERT_FALSE(s.error);
      t += cfg.itp_s;
      ++gt;
      for (int j = 0; j < kNumJoints; ++j) {
        const auto jj = static_cast<std::size_t>(j);
        EXPECT_LE(std::abs(s.qd[jj]), cfg.limits.v[jj] * 1.001) << "velocity bounded, joint " << j;
        EXPECT_LE(std::abs(s.qdd[jj]), cfg.limits.a[jj] * 1.001) << "acceleration bounded, joint " << j;
      }
      if (gt > 40) {
        max_err = std::max(max_err, std::abs(s.q[0] - amp * std::sin(kTwoPi * freq * t)));
      }
    }
  }
  EXPECT_LT(max_err, 0.01) << "servo tracks the sine (small lag)";
}

// Direction reversal (gentle triangle): bounded qdd through the reversal, no
// large overshoot past the target extrema.
TEST(Servo, DirectionReversalBounded) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  const double duration = 0.06;
  const int ticks_per = 7;
  const double step = 0.01;
  std::vector<double> targets;
  for (int i = 1; i <= 5; ++i) targets.push_back(static_cast<double>(i) * step);  // up to 0.05
  for (int i = 4; i >= 0; --i) targets.push_back(static_cast<double>(i) * step);  // reverse to 0

  double q_min = 0.0;
  double q_max = 0.0;
  for (double q_target : targets) {
    servo.set_target(axis0(q_target), duration);
    for (int k = 0; k < ticks_per; ++k) {
      const ServoStep s = servo.step();
      ASSERT_FALSE(s.error);
      for (int j = 0; j < kNumJoints; ++j) {
        const auto jj = static_cast<std::size_t>(j);
        EXPECT_LE(std::abs(s.qdd[jj]), cfg.limits.a[jj] * 1.001) << "qdd bounded through reversal, joint " << j;
      }
      q_min = std::min(q_min, s.q[0]);
      q_max = std::max(q_max, s.q[0]);
    }
  }
  EXPECT_LE(q_max, 0.05 + 0.02) << "no large overshoot above the peak target";
  EXPECT_GE(q_min, 0.0 - 0.02) << "no large overshoot below the bottom target";
}

// Best effort: a FAR target is tracked, not refused. There is no distance guard —
// what bounds the motion is the servo velocity limit, so the command heads toward the
// target monotonically and no single tick moves more than v_lim*itp.
TEST(Servo, FarTargetIsTrackedNotRefused) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  const double far = 0.9;  // rad, ~51° — far outside the window earlier revisions enforced
  servo.set_target(axis0(far), 0.05);

  double prev = servo.commanded()[0];
  const double max_step = cfg.servo_limit_scale * cfg.limits.v[0] * cfg.itp_s;
  for (int i = 0; i < 200; ++i) {
    const ServoStep s = servo.step();
    ASSERT_FALSE(s.error);
    EXPECT_GE(s.q[0], prev - 1e-12) << "moves toward the far target, tick " << i;
    EXPECT_LE(s.q[0] - prev, max_step * 1.001) << "no jump: bounded by v_lim*itp, tick " << i;
    for (int j = 0; j < kNumJoints; ++j) {
      const auto jj = static_cast<std::size_t>(j);
      EXPECT_LE(std::abs(s.qd[jj]), cfg.servo_limit_scale * cfg.limits.v[jj] * 1.001);
      EXPECT_LE(std::abs(s.qdd[jj]), cfg.servo_limit_scale * cfg.limits.a[jj] * 1.001);
    }
    prev = s.q[0];
  }
  EXPECT_GT(prev, 0.1) << "made real progress toward the far target rather than refusing it";
}

// A streamed ramp with the caller's clock and the tick clock DRIFTING against each
// other — the case every other test here misses, and the one that caught the arrival-
// velocity bug (see BEST EFFORT in servo.hpp).
//
// The caller streams on its own clock (20 ms) while the tick runs on the controller's,
// PLL-locked to ~7.95 ms rather than a round 8, so the ticks between targets alternate
// 2 and 3 and the phase walks through a whole-tick boundary periodically. Against a
// target stream that only ever moves forward, the command must only ever move forward.
TEST(Servo, StreamedRampNoReversalUnderClockDrift) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  const double V = 0.0873;      // rad/s (~5 °/s) — gentle, far under every limit
  const double dt = 0.020;      // caller's target spacing
  const double tick = 0.00795;  // controller's tick, deliberately not dt/2.5
  double next_target_t = 0.0;
  double worst_neg = 0.0;
  double peak = 0.0;

  for (int n = 1; n <= 1000; ++n) {
    const double t = static_cast<double>(n) * tick;
    while (next_target_t <= t) {  // a caller slot came due since the last tick
      servo.set_target(axis0(V * (next_target_t + dt)), dt);
      next_target_t += dt;
    }
    const ServoStep s = servo.step();
    ASSERT_FALSE(s.error);
    if (t > 0.4) {  // past the standing-start transient
      worst_neg = std::min(worst_neg, s.qd[0]);
      peak = std::max(peak, s.qd[0]);
    }
  }
  EXPECT_GE(worst_neg, 0.0) << "commanded velocity must never reverse against a "
                               "monotonically advancing target stream";
  EXPECT_LT(peak, 1.25 * V) << "and must not overshoot the streamed speed to catch up";
}

// Joint position clamp: a target past the soft limit is followed AS FAR AS the limit
// and no further — clamped, not refused, and never overshot past the stop.
TEST(Servo, TargetBeyondPositionLimitIsClampedToTheLimit) {
  TickEngineConfig cfg;
  cfg.limits.pos_lo = Vec6{-0.5, -0.5, -0.5, -0.5, -0.5, -0.5};
  cfg.limits.pos_hi = Vec6{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  servo.set_target(axis0(2.0), 0.05);  // well past the 0.5 rad stop
  double q_max = 0.0;
  for (int i = 0; i < 400; ++i) {
    const ServoStep s = servo.step();
    ASSERT_FALSE(s.error);
    q_max = std::max(q_max, s.q[0]);
    EXPECT_LE(s.q[0], cfg.limits.pos_hi[0] + 1e-9) << "never commands past the stop, tick " << i;
  }
  EXPECT_NEAR(q_max, cfg.limits.pos_hi[0], 1e-3) << "goes all the way to the stop, not short of it";

  // And the same on the way back down.
  servo.set_target(axis0(-2.0), 0.05);
  for (int i = 0; i < 600; ++i) {
    const ServoStep s = servo.step();
    ASSERT_FALSE(s.error);
    EXPECT_GE(s.q[0], cfg.limits.pos_lo[0] - 1e-9) << "never commands past the lower stop, tick " << i;
  }
}

// Unset position limits are inert: the shipped default is ±inf, so a core that was
// never handed a profile must not silently clamp motion to a made-up envelope.
TEST(Servo, DefaultPositionLimitsDoNotClamp) {
  TickEngineConfig cfg;  // defaults
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});
  servo.set_target(axis0(1.5), 0.05);  // ~86°, far outside any plausible synthetic box
  double q = 0.0;
  for (int i = 0; i < 600; ++i) {
    const ServoStep s = servo.step();
    ASSERT_FALSE(s.error);
    q = s.q[0];
  }
  EXPECT_NEAR(q, 1.5, 1e-3) << "reaches the target when no position limits are configured";
}

// Starvation-safety via re-issue. The servo holds its last target instead of
// requiring a fresh one every tick, so a stalled command stream is safe: re-sending
// the SAME target zeroes the velocity feedforward → the commanded position rests.
TEST(Servo, RestsWhenLastTargetReissued) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  const double q_hold = 0.04;
  const double duration = 0.05;
  servo.set_target(axis0(q_hold), duration);  // transit
  for (int i = 0; i < 8; ++i) servo.step();
  for (int i = 0; i < 80; ++i) {  // stream "stops" → re-issue the same target
    servo.set_target(axis0(q_hold), duration);  // feedforward (q-q)/dur = 0
    servo.step();
  }
  const ServoStep s = servo.step();
  EXPECT_NEAR(s.q[0], q_hold, 1e-4) << "commanded position rests at the held target";
  EXPECT_NEAR(s.qd[0], 0.0, 1e-3) << "velocity rests (starvation-safe)";
}
