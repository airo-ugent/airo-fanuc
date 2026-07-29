// SPDX-License-Identifier: Apache-2.0
//
// L1 golden — Ruckig-online brake. The headline assertion is R1 C1: seeding the
// analytic Hermite qdd makes the commanded ACCELERATION continuous across the
// trajectory→brake transition (an accel step latches the CRX contact-stop).

#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "tick_engine/brake.hpp"
#include "tick_engine/hermite.hpp"
#include "tick_engine/tick_engine_config.hpp"

using airo_fanuc::tick_engine::Brake;
using airo_fanuc::tick_engine::BrakeStep;
using airo_fanuc::tick_engine::HermiteSample;
using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::TickEngineConfig;
using airo_fanuc::tick_engine::TrajectorySampler;
using airo_fanuc::tick_engine::Vec6;

namespace {
double max_abs(const Vec6& v) {
  double m = 0.0;
  for (double x : v) {
    m = std::max(m, std::abs(x));
  }
  return m;
}
}  // namespace

// R1 C1 — accel continuity across traj→brake. A realistic mid-segment sample
// gives (q_cmd, qd_cmd, qdd_cmd) with qdd within the brake accel envelope;
// seeding qdd_cmd keeps the first brake accel within one jerk-limited step of it
// (continuous), whereas seeding qdd=0 (the old dries behaviour) produces a large
// accel STEP.
TEST(Brake, QddSeedGivesAccelContinuity) {
  TickEngineConfig cfg;

  // Gentle 1 s S-curve segment on joint 0; sample mid-segment (t = 250 ms).
  const std::array<std::int64_t, 2> times{0, 1'000'000'000};
  const std::array<Vec6, 2> q{Vec6{}, Vec6{0.2, 0, 0, 0, 0, 0}};
  const std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  TrajectorySampler samp;
  samp.bind(times.data(), q.data(), qd.data(), 2);
  const HermiteSample mid = samp.sample(250'000'000, 1.0);

  // Precondition: the mid-segment accel is within the brake accel envelope, so
  // Ruckig tracks it and the continuity comparison is meaningful.
  const double a_env0 = cfg.stop_scale_va * cfg.limits.a[0];
  ASSERT_GT(std::abs(mid.qdd[0]), 0.2) << "need a meaningfully non-zero mid-segment accel";
  ASSERT_LT(std::abs(mid.qdd[0]), a_env0) << "mid-segment accel must be within the brake envelope";

  const double j_env0 = cfg.stop_scale_j * cfg.limits.j[0];
  const double one_jerk_step = j_env0 * cfg.itp_s;

  // Correct seed: analytic qdd.
  Brake brake(cfg);
  brake.seed(mid.q, mid.qd, mid.qdd);
  const BrakeStep s1 = brake.step();
  ASSERT_FALSE(s1.error);
  const double disc_seeded = std::abs(s1.qdd[0] - mid.qdd[0]);

  // Wrong seed: qdd = 0 (old dries behaviour).
  Brake brake0(cfg);
  brake0.seed(mid.q, mid.qd, Vec6{});
  const BrakeStep s0 = brake0.step();
  ASSERT_FALSE(s0.error);
  const double disc_zero = std::abs(s0.qdd[0] - mid.qdd[0]);

  // Seeded: acceleration continuous → within one jerk-limited step.
  EXPECT_LE(disc_seeded, one_jerk_step + 1e-9)
      << "seeded-qdd brake accel is continuous with the trajectory accel";
  // Zero-seed: a clear acceleration STEP (much larger than one jerk step).
  EXPECT_GT(disc_zero, 10.0 * one_jerk_step)
      << "qdd=0 seed produces an accel discontinuity (the CSTOP-latching step)";
  // And the seed materially reduces the discontinuity.
  EXPECT_LT(disc_seeded, 0.1 * disc_zero);
}

// Brake drives velocity to rest under the stop envelope, reports Finished, and
// `done()` fires once measured velocity settles.
TEST(Brake, DeceleratesToRest) {
  TickEngineConfig cfg;
  Brake brake(cfg);
  const Vec6 q0{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  const Vec6 qd0{0.5, -0.4, 0.3, -0.2, 0.1, -0.05};
  brake.seed(q0, qd0, Vec6{});  // accel=0 → clean monotone decel

  double prev_speed = max_abs(qd0);
  bool finished = false;
  Vec6 last_qd{};
  const int max_ticks = static_cast<int>(cfg.brake_max_duration_s / cfg.itp_s) + 4;
  for (int i = 0; i < max_ticks; ++i) {
    const BrakeStep s = brake.step();
    ASSERT_FALSE(s.error);
    const double speed = max_abs(s.qd);
    EXPECT_LE(speed, prev_speed + 1e-9) << "|qd| monotonically non-increasing, tick " << i;
    prev_speed = speed;
    last_qd = s.qd;
    if (s.finished) {
      finished = true;
      break;
    }
  }
  EXPECT_TRUE(finished) << "brake reaches target velocity within the cap";
  EXPECT_LT(max_abs(last_qd), cfg.brake_settle_vel_eps_rad_s) << "final velocity below settle eps";
  // With measured velocity ~0, done() is satisfied.
  EXPECT_TRUE(brake.done(Vec6{}));
}

// The velocity envelope is expanded to encompass |qd_cmd| (max(0.4·v_lim,
// 1.01·|qd_cmd|)) so the seed velocity is NOT clipped down to the floor.
TEST(Brake, VelocityEnvelopeEncompassesSeed) {
  TickEngineConfig cfg;
  const double floor0 = cfg.stop_scale_va * cfg.limits.v[0];  // ~0.838 rad/s
  const double seed_v = 1.5;                                   // > floor, < v_lim
  ASSERT_GT(seed_v, floor0);
  ASSERT_LT(seed_v, cfg.limits.v[0]);

  Brake brake(cfg);
  brake.seed(Vec6{}, Vec6{seed_v, 0, 0, 0, 0, 0}, Vec6{});
  const BrakeStep s = brake.step();
  ASSERT_FALSE(s.error);
  // First-tick velocity stays near the seed (changes by ≤ a_env·dt), i.e. it was
  // NOT clipped to the floor.
  const double a_env0 = cfg.stop_scale_va * cfg.limits.a[0];
  EXPECT_NEAR(s.qd[0], seed_v, a_env0 * cfg.itp_s + 1e-9)
      << "seed velocity above the floor is preserved (no envelope clip)";
  EXPECT_GT(s.qd[0], floor0) << "definitely not clipped to the STOP_LIMIT_SCALE_VA floor";
}

// Hard cap: after brake_max_duration_s of ticks, done() latches even if the
// (fake) measured velocity never settles.
TEST(Brake, HardDurationCap) {
  TickEngineConfig cfg;
  Brake brake(cfg);
  brake.seed(Vec6{}, Vec6{0.3, 0, 0, 0, 0, 0}, Vec6{});
  const int ticks = static_cast<int>(cfg.brake_max_duration_s / cfg.itp_s) + 1;
  for (int i = 0; i < ticks; ++i) {
    brake.step();
  }
  EXPECT_TRUE(brake.timed_out());
  // done() true via the cap even with a spurious non-zero measured velocity.
  EXPECT_TRUE(brake.done(Vec6{5.0, 0, 0, 0, 0, 0}));
}

TEST(Brake, StepWithoutSeedIsErrorNotThrow) {
  TickEngineConfig cfg;
  Brake brake(cfg);
  const BrakeStep s = brake.step();  // must not throw on the RT path
  EXPECT_TRUE(s.error);
}
