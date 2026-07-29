// SPDX-License-Identifier: Apache-2.0
//
// L1 golden — cubic-Hermite resampling. Bit-agreement with the dries oracle
// (`src/grocery_bot/robot/fanuc/interpolator.py::_cubic_hermite` +
// `tests/test_waypoint_interp.py`), analytic-derivative correctness, speed_scale
// halving, and the qd_end blend (R1 B3).
//
// The expected position hexfloats were emitted by running the VERBATIM Python
// oracle (see the reference generator in the P3a session). Bit-exactness holds
// because hermite.cpp is compiled -ffp-contract=off and reproduces the oracle's
// op order; this TU is compiled the same way.

#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "tick_engine/hermite.hpp"

using airo_fanuc::tick_engine::HermiteSample;
using airo_fanuc::tick_engine::hermite_at_ns;
using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::QdEndBlend;
using airo_fanuc::tick_engine::TrajectorySampler;
using airo_fanuc::tick_engine::Vec6;

namespace {

// --- CASE_A: rich, non-knot query (matches the reference generator) ---
constexpr std::int64_t kT0 = 0;
constexpr std::int64_t kT1 = 100'000'000;  // 0.1 s
constexpr std::int64_t kTq = 37'000'000;   // s = 0.37
const Vec6 kQ0{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
const Vec6 kQd0{1.0, -1.0, 0.5, -0.5, 0.25, -0.25};
const Vec6 kQ1{0.6, 0.3, -0.1, 0.2, -0.3, 0.4};
const Vec6 kQd1{-0.5, 0.5, -1.0, 1.0, -0.25, 0.25};

// Position at kTq, hexfloat from the Python oracle — EXACT.
const Vec6 kQExpectedA{
    0x1.1843690b5c549p-2, -0x1.07601e4dc8e10p-4, 0x1.89a543f1c7582p-3,
    -0x1.d7b7c0cc0d8a6p-3, 0x1.088305029e400p-2, -0x1.2f8c436fc1590p-2,
};

}  // namespace

// interpolator.py::_cubic_hermite → bit-for-bit position agreement.
TEST(Hermite, BitExactPositionCaseA) {
  const HermiteSample s = hermite_at_ns(kT0, kQ0, kQd0, kT1, kQ1, kQd1, kTq);
  for (int j = 0; j < kNumJoints; ++j) {
    EXPECT_EQ(s.q[static_cast<std::size_t>(j)], kQExpectedA[static_cast<std::size_t>(j)])
        << "joint " << j << " position must match the Python oracle bit-for-bit";
  }
}

// test_waypoint_interp::test_cubic_hermite_at_knots_is_exact — q at the knots.
TEST(Hermite, ExactAtKnots) {
  const HermiteSample at0 = hermite_at_ns(kT0, kQ0, kQd0, kT1, kQ1, kQd1, kT0);
  const HermiteSample at1 = hermite_at_ns(kT0, kQ0, kQd0, kT1, kQ1, kQd1, kT1);
  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    EXPECT_EQ(at0.q[jj], kQ0[jj]) << "q(t0) == q0 exactly, joint " << j;
    EXPECT_EQ(at1.q[jj], kQ1[jj]) << "q(t1) == q1 exactly, joint " << j;
    // Velocity at the knots equals the knot tangents exactly (the h_s cancels).
    EXPECT_DOUBLE_EQ(at0.qd[jj], kQd0[jj]) << "qd(t0) == qd0, joint " << j;
    EXPECT_DOUBLE_EQ(at1.qd[jj], kQd1[jj]) << "qd(t1) == qd1, joint " << j;
  }
}

// test_waypoint_interp::test_cubic_hermite_midpoint_for_zero_velocity_knots.
TEST(Hermite, ZeroVelocityMidpointIsHalf) {
  const Vec6 z{};
  const Vec6 one{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const HermiteSample s = hermite_at_ns(0, z, z, 100'000'000, one, z, 50'000'000);
  for (int j = 0; j < kNumJoints; ++j) {
    EXPECT_EQ(s.q[static_cast<std::size_t>(j)], 0.5) << "midpoint of zero-vel knots, joint " << j;
  }
}

// Analytic qd / qdd equal a central finite difference of the (exact) position.
TEST(Hermite, AnalyticDerivativesMatchFiniteDifference) {
  const HermiteSample s = hermite_at_ns(kT0, kQ0, kQd0, kT1, kQ1, kQd1, kTq);
  const std::int64_t d = 1000;  // 1 µs
  const double dt = 2.0 * static_cast<double>(d) / 1e9;
  const HermiteSample lo = hermite_at_ns(kT0, kQ0, kQd0, kT1, kQ1, kQd1, kTq - d);
  const HermiteSample hi = hermite_at_ns(kT0, kQ0, kQd0, kT1, kQ1, kQd1, kTq + d);
  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    const double fd_v = (hi.q[jj] - lo.q[jj]) / dt;
    const double fd_a = (hi.qd[jj] - lo.qd[jj]) / dt;
    EXPECT_NEAR(s.qd[jj], fd_v, 1e-5) << "analytic velocity, joint " << j;
    EXPECT_NEAR(s.qdd[jj], fd_a, 1e-5) << "analytic acceleration, joint " << j;
  }
}

// TrajectorySampler at speed_scale=1.0 reproduces the oracle (bit-exact q).
TEST(TrajectorySampler, MatchesOracleAtUnitScale) {
  const std::array<std::int64_t, 2> times{kT0, kT1};
  const std::array<Vec6, 2> q{kQ0, kQ1};
  const std::array<Vec6, 2> qd{kQd0, kQd1};
  TrajectorySampler samp;
  samp.bind(times.data(), q.data(), qd.data(), 2);
  const HermiteSample s = samp.sample(kTq, 1.0);
  const HermiteSample oracle = hermite_at_ns(kT0, kQ0, kQd0, kT1, kQ1, kQd1, kTq);
  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    EXPECT_EQ(s.q[jj], kQExpectedA[jj]) << "sampler position bit-exact, joint " << j;
    EXPECT_DOUBLE_EQ(s.qd[jj], oracle.qd[jj]) << "sampler velocity, joint " << j;
  }
}

// speed_scale = 0.5 halves velocities and quarters accelerations; positions are
// evaluated at scaled time (q_wire(τ) = q(0.5·τ)). At τ=2·kTq, 0.5·τ = kTq.
TEST(TrajectorySampler, SpeedScaleHalvesVelocity) {
  const std::array<std::int64_t, 2> times{kT0, kT1};
  const std::array<Vec6, 2> q{kQ0, kQ1};
  const std::array<Vec6, 2> qd{kQd0, kQd1};
  TrajectorySampler samp;
  samp.bind(times.data(), q.data(), qd.data(), 2);

  const HermiteSample full = samp.sample(kTq, 1.0);        // trajectory time kTq
  const HermiteSample half = samp.sample(2 * kTq, 0.5);    // trajectory time 0.5·2kTq = kTq
  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    // Same trajectory point → identical position (bit-exact).
    EXPECT_EQ(half.q[jj], full.q[jj]) << "position at scaled time, joint " << j;
    EXPECT_EQ(half.q[jj], kQExpectedA[jj]) << "position bit-exact vs oracle, joint " << j;
    // Velocity halved, acceleration quartered.
    EXPECT_DOUBLE_EQ(half.qd[jj], 0.5 * full.qd[jj]) << "velocity halved, joint " << j;
    EXPECT_DOUBLE_EQ(half.qdd[jj], 0.25 * full.qdd[jj]) << "accel quartered, joint " << j;
  }
}

TEST(TrajectorySampler, HoldsBeforeStartAndAfterEnd) {
  const std::array<std::int64_t, 2> times{kT0, kT1};
  const std::array<Vec6, 2> q{kQ0, kQ1};
  const std::array<Vec6, 2> qd{kQd0, kQd1};
  TrajectorySampler samp;
  samp.bind(times.data(), q.data(), qd.data(), 2);

  const HermiteSample before = samp.sample(0, 1.0);
  const HermiteSample after = samp.sample(kT1 + 5'000'000, 1.0);
  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    EXPECT_EQ(before.q[jj], kQ0[jj]) << "hold first knot before start, joint " << j;
    EXPECT_EQ(after.q[jj], kQ1[jj]) << "hold last knot after end, joint " << j;
    EXPECT_EQ(after.qd[jj], 0.0) << "no velocity past end (blend/hold take over), joint " << j;
    EXPECT_EQ(after.qdd[jj], 0.0) << "no accel past end, joint " << j;
  }
}

// R1 B3 — qd_end blend: monotone velocity ramp to 0, no step, ≥ 25 ms.
TEST(QdEndBlend, MonotoneRampToRest) {
  const Vec6 q_end{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  const Vec6 qd_end{0.5, -0.4, 0.3, -0.2, 0.1, -0.05};  // wire terminal velocity
  const double itp = 0.008;
  QdEndBlend blend;
  blend.plan(q_end, qd_end, 0.025, itp);

  ASSERT_TRUE(blend.active());
  EXPECT_GE(blend.duration_s(), 0.025) << "blend must span >= 25 ms";
  EXPECT_GE(blend.num_ticks(), 4) << "25 ms / 8 ms rounds up to >= 4 ticks";
  EXPECT_DOUBLE_EQ(blend.max_abs_qd_end(), 0.5) << "records |qd_end|_inf";

  const std::int64_t step_ns = static_cast<std::int64_t>(itp * 1e9);

  // Tick 0 starts EXACTLY at (q_end, qd_end) — no step at entry.
  const HermiteSample first = blend.sample(0);
  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    EXPECT_EQ(first.q[jj], q_end[jj]) << "blend starts at q_end, joint " << j;
    EXPECT_EQ(first.qd[jj], qd_end[jj]) << "blend starts at qd_end (no step), joint " << j;
  }

  // Per-joint |velocity| monotonically non-increasing across ticks; ends at ~0.
  Vec6 prev_abs{};
  for (int j = 0; j < kNumJoints; ++j) {
    prev_abs[static_cast<std::size_t>(j)] = std::abs(qd_end[static_cast<std::size_t>(j)]);
  }
  const int n = blend.num_ticks();
  for (int k = 1; k <= n; ++k) {
    const HermiteSample s = blend.sample(static_cast<std::int64_t>(k) * step_ns);
    for (int j = 0; j < kNumJoints; ++j) {
      const auto jj = static_cast<std::size_t>(j);
      const double a = std::abs(s.qd[jj]);
      EXPECT_LE(a, prev_abs[jj] + 1e-12) << "velocity magnitude monotone, joint " << j << " tick " << k;
      prev_abs[jj] = a;
    }
  }
  // At the end: at rest.
  const HermiteSample last = blend.sample(blend.duration_ns());
  for (int j = 0; j < kNumJoints; ++j) {
    EXPECT_NEAR(last.qd[static_cast<std::size_t>(j)], 0.0, 1e-9) << "ends at rest, joint " << j;
  }
}

TEST(QdEndBlend, InactiveWhenTerminalVelocityZero) {
  const Vec6 q_end{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const Vec6 zero{};
  QdEndBlend blend;
  blend.plan(q_end, zero, 0.025, 0.008);
  EXPECT_FALSE(blend.active()) << "no ramp warranted when |qd_end| == 0";
  EXPECT_DOUBLE_EQ(blend.max_abs_qd_end(), 0.0);
}
