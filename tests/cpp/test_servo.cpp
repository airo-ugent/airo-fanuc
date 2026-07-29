// SPDX-License-Identifier: Apache-2.0
//
// L1 golden — Ruckig-online servo (R1 C2/C3). Asserts the minimum_duration fix
// (no reach-early freeze), no zero-velocity dwell on a streamed ramp, bounded
// qdd on a sine + a direction reversal, the 5° distance guard, and starvation
// rest via re-issue. Parameters are deliberately gentle: an aggressive velocity
// feedforward toward a nearby target from rest is infeasible without runway and
// makes Ruckig reverse to gain it (the "overshoot-reverse" C2 also names) — that
// is correct Ruckig behaviour, not a servo bug, but it is not what these
// property tests are probing.

#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "tick_engine/servo.hpp"
#include "tick_engine/tick_engine_config.hpp"

using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::Servo;
using airo_fanuc::tick_engine::ServoAccept;
using airo_fanuc::tick_engine::ServoStep;
using airo_fanuc::tick_engine::TickEngineConfig;
using airo_fanuc::tick_engine::Vec6;

namespace {
Vec6 axis0(double x) { return Vec6{x, 0, 0, 0, 0, 0}; }
constexpr double kTwoPi = 2.0 * 3.141592653589793;
}  // namespace

// R1 C2 — minimum_duration = duration prevents the reach-early freeze sawtooth.
// A tiny move whose time-optimal profile is ≪ duration is spread across the full
// duration: at ~half the duration the command is still en route, and arrival
// takes many ticks (not 1-2).
TEST(Servo, MinimumDurationNoEarlyArrival) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  const double target = 0.002;  // rad — tiny; time-optimal ≪ duration
  const double duration = 0.05;
  ASSERT_EQ(servo.set_target(axis0(target), duration), ServoAccept::kOk);

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
  const double delta = V * duration;  // within the 5° window

  int global_tick = 0;
  for (int m = 1; m <= 40; ++m) {
    ASSERT_EQ(servo.set_target(axis0(static_cast<double>(m) * delta), duration), ServoAccept::kOk);
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
    ASSERT_EQ(servo.set_target(axis0(q_target), duration), ServoAccept::kOk);
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
    ASSERT_EQ(servo.set_target(axis0(q_target), duration), ServoAccept::kOk);
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

// R1 C3 — distance guard: reject a target farther than the servo window (5°)
// from the current commanded position; no state change on reject; the accepted
// target still executes (forward progress).
TEST(Servo, DistanceGuardRejectsFarTarget) {
  TickEngineConfig cfg;
  Servo servo(cfg);
  servo.start(Vec6{}, Vec6{}, Vec6{});

  // Gentle accepted target (long duration → feasible feedforward, forward motion).
  EXPECT_EQ(servo.set_target(axis0(0.03), 0.2), ServoAccept::kOk);  // ~1.7°
  const Vec6 cmd_before = servo.commanded();
  EXPECT_EQ(servo.set_target(axis0(0.20), 0.2), ServoAccept::kRejectedDistance);  // ~11.5°
  for (int j = 0; j < kNumJoints; ++j) {
    EXPECT_EQ(servo.commanded()[static_cast<std::size_t>(j)], cmd_before[static_cast<std::size_t>(j)])
        << "rejected target must not mutate commanded state, joint " << j;
  }
  // The accepted target (0.03) still executes: monotone forward progress.
  double prev = servo.commanded()[0];
  for (int i = 0; i < 6; ++i) {
    const ServoStep s = servo.step();
    ASSERT_FALSE(s.error);
    EXPECT_GT(s.q[0], prev - 1e-12) << "moving toward the accepted target, tick " << i;
    EXPECT_GT(s.q[0], 0.0) << "moving in the +direction of the accepted target 0.03";
    prev = s.q[0];
  }
}

// Starvation-safety via re-issue (ur_rtde hold-last-target model): re-sending the
// SAME target zeroes the velocity feedforward → the commanded position rests.
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
