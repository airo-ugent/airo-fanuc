// SPDX-License-Identifier: Apache-2.0
//
// Unit test — per-tick position slew clip. Verifies the clip + count + never-
// fault behaviour AND the NO-ACCEL-CAP regression (the clip is a pure function
// of (prev_q, q_desired); it caps |Δq| per tick only, NEVER Δ(derived velocity)).
// An accel cap here is a known anti-pattern: it withholds part of every commanded
// step, and because the limiter feeds its own output back as prev_q, the withheld
// error accumulates instead of being caught up — measured as a 22° runaway drift
// away from the commanded path.

#include <cmath>

#include <gtest/gtest.h>

#include "tick_engine/slew.hpp"
#include "tick_engine/tick_engine_config.hpp"

using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::SlewLimiter;
using airo_fanuc::tick_engine::SlewResult;
using airo_fanuc::tick_engine::TickEngineConfig;
using airo_fanuc::tick_engine::Vec6;

TEST(Slew, ClipsToPerJointBound) {
  TickEngineConfig cfg;
  SlewLimiter lim(cfg);
  lim.reset(Vec6{});

  const SlewResult r = lim.apply(Vec6{10, -10, 10, -10, 10, -10});  // huge jump every joint
  EXPECT_EQ(r.clipped_joints, kNumJoints);
  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    const double step = cfg.slew_factor * cfg.limits.v[jj] * cfg.itp_s;
    const double expect = (j % 2 == 0) ? step : -step;
    EXPECT_DOUBLE_EQ(r.q[jj], expect) << "clipped to ± slew_factor·v_lim·itp, joint " << j;
    EXPECT_DOUBLE_EQ(lim.max_step(j), step);
  }
  EXPECT_EQ(lim.total_clips(), static_cast<std::uint64_t>(kNumJoints));
}

TEST(Slew, PassesThroughWithinBound) {
  TickEngineConfig cfg;
  SlewLimiter lim(cfg);
  lim.reset(Vec6{});
  const double small = 0.5 * cfg.slew_factor * cfg.limits.v[0] * cfg.itp_s;
  const SlewResult r = lim.apply(Vec6{small, 0, 0, 0, 0, 0});
  EXPECT_EQ(r.clipped_joints, 0);
  EXPECT_DOUBLE_EQ(r.q[0], small) << "within-bound command passes through unchanged";
}

TEST(Slew, FirstCommandAdoptedNotClipped) {
  TickEngineConfig cfg;
  SlewLimiter lim(cfg);  // no reset → no reference yet
  const SlewResult r = lim.apply(Vec6{5, 5, 5, 5, 5, 5});
  EXPECT_EQ(r.clipped_joints, 0) << "first command establishes the reference, never clipped";
  EXPECT_DOUBLE_EQ(r.q[0], 5.0);
}

TEST(Slew, NeverFaultsUnderSustainedHugeJumps) {
  TickEngineConfig cfg;
  SlewLimiter lim(cfg);
  lim.reset(Vec6{});
  for (int i = 0; i < 100; ++i) {
    const double sign = (i % 2 == 0) ? 1000.0 : -1000.0;
    const SlewResult r = lim.apply(Vec6{sign, sign, sign, sign, sign, sign});
    for (int j = 0; j < kNumJoints; ++j) {
      const double step = lim.max_step(j);
      EXPECT_LE(std::abs(r.q[static_cast<std::size_t>(j)]), step + 1e-12)
          << "output always within one step of prev";
    }
  }
  EXPECT_TRUE(lim.sustained_clip()) << "sustained clipping raises the diagnostic flag";
  EXPECT_GE(lim.consecutive_clip_ticks(), cfg.slew_sustained_ticks);
  EXPECT_EQ(lim.total_clips(), static_cast<std::uint64_t>(100 * kNumJoints));
}

TEST(Slew, SustainedFlagClearsWhenClippingStops) {
  TickEngineConfig cfg;
  SlewLimiter lim(cfg);
  lim.reset(Vec6{});
  SlewResult last{};
  for (int i = 0; i < 5; ++i) last = lim.apply(Vec6{1000, 0, 0, 0, 0, 0});
  EXPECT_TRUE(lim.sustained_clip());
  // Re-issue the last (clipped) command → delta 0 → no clip → run resets.
  const SlewResult r = lim.apply(last.q);
  EXPECT_EQ(r.clipped_joints, 0);
  EXPECT_EQ(lim.consecutive_clip_ticks(), 0);
  EXPECT_FALSE(lim.sustained_clip());
}

// NO-ACCEL-CAP regression. After a +max_step tick, a reversing command is
// clipped to −max_step (a full-magnitude position step in the opposite
// direction) — the commanded velocity reverses by 2·max_step/itp in one tick.
// An accel-cap on derived velocity would FORBID this; a pure position-slew
// ALLOWS it because |Δq| == max_step ≤ bound. Output == 0 proves position-only.
TEST(Slew, NoAccelCapAllowsFullVelocityReversal) {
  TickEngineConfig cfg;
  SlewLimiter lim(cfg);
  lim.reset(Vec6{});
  const double step0 = lim.max_step(0);

  const SlewResult up = lim.apply(Vec6{10, 0, 0, 0, 0, 0});  // → +max_step
  ASSERT_DOUBLE_EQ(up.q[0], step0);

  const SlewResult down = lim.apply(Vec6{-10, 0, 0, 0, 0, 0});  // reverse
  // Pure position slew: prev(+step0) + clamp(-10-step0) = +step0 − step0 = 0.
  EXPECT_DOUBLE_EQ(down.q[0], 0.0)
      << "position-slew permits a full-magnitude reversal in one tick (no accel-cap)";
}

// The clip is a pure function of (prev_q, q_desired): different prior motion
// histories, same (reset prev, desired) → identical output. Proves the limiter
// carries NO velocity/acceleration state.
TEST(Slew, ClipIsHistoryIndependent) {
  TickEngineConfig cfg;
  const Vec6 P{0.1, -0.2, 0.3, -0.1, 0.2, -0.3};
  const Vec6 D{0.5, -0.5, 0.5, -0.5, 0.5, -0.5};

  SlewLimiter a(cfg);
  a.reset(P);
  const SlewResult ra = a.apply(D);

  SlewLimiter b(cfg);
  // Give b a totally different prior history before pinning the same reference.
  b.reset(Vec6{});
  for (int i = 0; i < 7; ++i) b.apply(Vec6{static_cast<double>(i), 0, 0, 0, 0, 0});
  b.reset(P);  // same reference as a
  const SlewResult rb = b.apply(D);

  for (int j = 0; j < kNumJoints; ++j) {
    EXPECT_DOUBLE_EQ(ra.q[static_cast<std::size_t>(j)], rb.q[static_cast<std::size_t>(j)])
        << "clip depends only on (prev_q, q_desired), joint " << j;
  }
}
