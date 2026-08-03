// SPDX-License-Identifier: Apache-2.0
//
// Unit test — joining a submitted plan at the phase the caller's staleness implies.
//
// A caller plans from a commanded state it read at tick S and the core consumes the
// submission at tick C > S, so the plan's first knot describes where the arm WAS. Three
// properties are asserted, all on the SEQUENCE of commanded positions returned by
// TickCore::tick, because that is what reaches the wire:
//
//   (1) a plan that continues the arm's current motion is joined with NO CAPTURE PHASE
//       and no splice — the executed path is the caller's trajectory and nothing else;
//   (2) the join lands on the plan's own phase, so the commanded stream neither jumps
//       nor stalls across the submission;
//   (3) a submission whose declared anchor is older than the staleness ceiling is
//       refused, and one that declares nothing still joins at knot 0.
//
// (1) is the one that matters most, and it is asserted as "mode never became CAPTURE"
// rather than by inspecting the splice: a splice asked to bridge zero distance at an
// unchanged velocity is precisely the geometry a fixed-point position profile cannot
// express, so routing a zero-residual join back through the generator would reintroduce
// the excursion this exists to remove. Expectations are derived from `cfg`.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"

using namespace airo_fanuc::rt_core;
using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::Vec6;

namespace {

RxSample clean_rx(const Vec6& q, const Vec6& qd = Vec6{}) {
  RxSample rx{};
  rx.q_meas = q;
  rx.qd_est = qd;
  rx.gates.motion_possible = true;
  rx.safety_scale = 1.0;
  return rx;
}

// An ITP-spaced constant-velocity plan on joint 0, the shape a replan of an ongoing
// cruise produces: knot k is the state the arm should be in k ticks after the plan's
// anchor. Long enough that a join several ticks in still leaves plenty to play.
struct RampPlan {
  static constexpr int kN = 200;
  std::array<std::int64_t, kN> times{};
  std::array<Vec6, kN> q{};
  std::array<Vec6, kN> qd{};
};

RampPlan make_plan(double q_start, double v, double itp) {
  RampPlan p{};
  const std::int64_t itp_ns = static_cast<std::int64_t>(std::llround(itp * 1e9));
  for (int k = 0; k < RampPlan::kN; ++k) {
    p.times[static_cast<std::size_t>(k)] = static_cast<std::int64_t>(k) * itp_ns;
    p.q[static_cast<std::size_t>(k)] = Vec6{q_start + v * static_cast<double>(k) * itp, 0, 0, 0, 0, 0};
    p.qd[static_cast<std::size_t>(k)] = Vec6{v, 0, 0, 0, 0, 0};
  }
  return p;
}

Target plan_target(const RampPlan& p, std::uint64_t epoch, std::uint64_t plan_tick) {
  Target g{};
  g.kind = TargetKind::kTrajectory;
  g.epoch = epoch;
  g.times_ns = p.times.data();
  g.q = p.q.data();
  g.qd = p.qd.data();
  g.n = RampPlan::kN;
  g.speed_scale = 1.0;
  g.plan_tick = plan_tick;
  return g;
}

// Drive a cruise at `v` on joint 0 and stop with the core in TRAJECTORY, mid-cruise.
struct Cruising {
  std::array<std::int64_t, 4> times{};
  std::array<Vec6, 4> q{};
  std::array<Vec6, 4> qd{};
};

void start_cruise(TickCore& tc, Cruising& c, double v) {
  const double r1 = 0.4, r2 = 2.0, r3 = 0.4;
  c.times = {0, static_cast<std::int64_t>(r1 * 1e9), static_cast<std::int64_t>((r1 + r2) * 1e9),
             static_cast<std::int64_t>((r1 + r2 + r3) * 1e9)};
  const double q1 = 0.5 * v * r1, q2 = q1 + v * r2, q3 = q2 + 0.5 * v * r3;
  c.q = {Vec6{}, Vec6{q1, 0, 0, 0, 0, 0}, Vec6{q2, 0, 0, 0, 0, 0}, Vec6{q3, 0, 0, 0, 0, 0}};
  c.qd = {Vec6{}, Vec6{v, 0, 0, 0, 0, 0}, Vec6{v, 0, 0, 0, 0, 0}, Vec6{}};
  tc.begin_streaming(Vec6{});
  {
    RxSample rx = clean_rx(Vec6{});
    tc.tick(&rx, nullptr);
  }
  Target g{};
  g.kind = TargetKind::kTrajectory;
  g.epoch = tc.epoch();
  g.times_ns = c.times.data();
  g.q = c.q.data();
  g.qd = c.qd.data();
  g.n = 4;
  g.speed_scale = 1.0;
  RxSample rx0 = clean_rx(Vec6{});
  tc.tick(&rx0, &g);
  for (int i = 0; i < 80; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
}

}  // namespace

// ===========================================================================
// (1) + (2) A continuation replan, declared, consumed n_stale ticks later: joined with
// no CAPTURE phase, and the commanded stream keeps its cadence across the submission.
// ===========================================================================
TEST(JoinAtPhase, ContinuationReplanJoinsWithoutASplice) {
  for (int n_stale : {1, 5, 10}) {
    RtCoreConfig cfg;
    const double itp = cfg.tick.itp_s;
    const double v = 0.6 * cfg.tick.capture_rate_rad_s;
    TickCore tc(cfg);
    Cruising c{};
    start_cruise(tc, c, v);
    ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);
    ASSERT_NEAR(tc.qd_cmd()[0], v, 1e-6) << "the replan must be taken from a flat cruise";

    // The caller's snapshot: this tick's commanded state, and the plan it builds from it.
    const std::uint64_t plan_tick = tc.tick_no();
    RampPlan plan = make_plan(tc.q_cmd()[0], v, itp);

    // n_stale - 1 ticks pass before the submission is consumed.
    for (int i = 0; i < n_stale - 1; ++i) {
      RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
      tc.tick(&rx, nullptr);
    }
    const double d_cruise = v * itp;
    const Vec6 before = tc.q_cmd();
    Target g2 = plan_target(plan, tc.epoch(), plan_tick);
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    const Command c1 = tc.tick(&rx, &g2);
    ASSERT_TRUE(tc.last_consume().accepted) << "n_stale = " << n_stale;

    // THE POINT (1). No CAPTURE phase: nothing was synthesized, because there was nothing
    // between the commanded state and the plan.
    EXPECT_EQ(tc.mode(), Mode::TRAJECTORY)
        << "n_stale = " << n_stale
        << ": a plan that continues the current motion was routed through the splice "
           "generator — a bridge of zero distance at an unchanged velocity is the one "
           "geometry a fixed-point position profile cannot express";

    // THE POINT (2). And the accept tick carries exactly one tick of travel: the join
    // landed on the plan's own phase, so the stream neither jumped nor stalled.
    EXPECT_NEAR((c1.q_rad[0] - before[0]) / d_cruise, 1.0, 0.02)
        << "n_stale = " << n_stale << ": the join was off phase";

    // Cadence holds afterwards too, and the plan runs to DONE.
    double prev = c1.q_rad[0];
    for (int i = 0; i < 40; ++i) {
      RxSample r = clean_rx(tc.q_cmd(), tc.qd_cmd());
      const Command cc = tc.tick(&r, nullptr);
      EXPECT_NEAR((cc.q_rad[0] - prev) / d_cruise, 1.0, 0.02)
          << "n_stale = " << n_stale << ", tick " << i;
      prev = cc.q_rad[0];
      EXPECT_NE(tc.mode(), Mode::CAPTURE) << "no splice may appear later either";
    }
  }
}

// ===========================================================================
// A gap the splice CAN bridge monotonically still goes through the splice: the
// dispatcher chooses a mechanism, it does not disable one.
// ===========================================================================
TEST(JoinAtPhase, AReachableGapStillSplices) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;
  const double v = 0.6 * cfg.tick.capture_rate_rad_s;
  TickCore tc(cfg);
  Cruising c{};
  start_cruise(tc, c, v);
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);

  // Declared, consumed immediately, but the plan starts well AHEAD of the commanded pose
  // at the same velocity — a gap a monotone splice can cover.
  const std::uint64_t plan_tick = tc.tick_no();
  RampPlan plan = make_plan(tc.q_cmd()[0] + 20.0 * v * itp, v, itp);
  Target g2 = plan_target(plan, tc.epoch(), plan_tick);
  RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
  tc.tick(&rx, &g2);
  ASSERT_TRUE(tc.last_consume().accepted);
  EXPECT_EQ(tc.mode(), Mode::CAPTURE) << "a reachable gap is what the splice is for";

  // And it is monotone: the splice never reverses on the way.
  double prev = tc.q_cmd()[0];
  int capture_ticks = 0;
  for (int i = 0; i < 300 && tc.mode() == Mode::CAPTURE; ++i) {
    RxSample r = clean_rx(tc.q_cmd(), tc.qd_cmd());
    const Command cc = tc.tick(&r, nullptr);
    EXPECT_GE(cc.q_rad[0] - prev, -1e-12) << "splice reversed at tick " << i;
    prev = cc.q_rad[0];
    ++capture_ticks;
  }
  EXPECT_GT(capture_ticks, 0);
  EXPECT_EQ(tc.mode(), Mode::TRAJECTORY) << "the splice hands over to playback";
}

// ===========================================================================
// (3) The staleness ceiling, and the undeclared default.
// ===========================================================================
TEST(JoinAtPhase, StaleBeyondTheCeilingIsRefused) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;
  const double v = 0.6 * cfg.tick.capture_rate_rad_s;
  TickCore tc(cfg);
  Cruising c{};
  start_cruise(tc, c, v);

  const int ceiling_ticks = static_cast<int>(std::ceil(cfg.max_plan_stale_ms / 1000.0 / itp - 1e-9));
  const std::uint64_t plan_tick = tc.tick_no();
  RampPlan plan = make_plan(tc.q_cmd()[0], v, itp);
  for (int i = 0; i < ceiling_ticks + 1; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
  Target g2 = plan_target(plan, tc.epoch(), plan_tick);
  RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
  tc.tick(&rx, &g2);
  EXPECT_TRUE(tc.last_consume().rejected_capture)
      << "a plan " << (ceiling_ticks + 1) << " ticks out of date must be refused, not "
      << "joined that far in";
  EXPECT_FALSE(tc.last_consume().accepted);
}

// An undeclared submission is unchanged: knot 0 is the join point, and a rest-to-rest
// splice over a real gap is generated exactly as before.
TEST(JoinAtPhase, UndeclaredSubmissionJoinsAtKnotZero) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;
  TickCore tc(cfg);
  tc.begin_streaming(Vec6{});
  {
    RxSample rx = clean_rx(Vec6{});
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::HOLD);

  // From rest, 3 degrees away, no plan_tick: the classic capture splice.
  RampPlan plan = make_plan(0.05, 0.0, itp);
  Target g = plan_target(plan, tc.epoch(), /*plan_tick=*/0);
  RxSample rx = clean_rx(Vec6{});
  tc.tick(&rx, &g);
  ASSERT_TRUE(tc.last_consume().accepted);
  EXPECT_EQ(tc.mode(), Mode::CAPTURE) << "a rest-to-rest gap is spliced, as ever";

  // The first CAPTURE command advances toward the plan, and the anchor knot is not
  // re-commanded.
  EXPECT_GT(tc.q_cmd()[0], 0.0);
  EXPECT_LT(tc.q_cmd()[0], 0.05);
}

// The tick counter is the thing plan_tick is compared against, so it must count every
// tick, including ones that transmit nothing.
TEST(JoinAtPhase, TickNumberCountsEveryTick) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  EXPECT_EQ(tc.tick_no(), 0u) << "nothing has been commanded yet";
  tc.begin_streaming(Vec6{});
  for (std::uint64_t i = 1; i <= 5; ++i) {
    RxSample rx = clean_rx(Vec6{});
    tc.tick(&rx, nullptr);
    EXPECT_EQ(tc.tick_no(), i);
  }
  // RX-silent ticks count too: the commanded state advances whether or not a packet
  // arrived, so a caller's snapshot age has to be measured in ticks, not packets.
  for (std::uint64_t i = 6; i <= 10; ++i) {
    tc.tick(nullptr, nullptr);
    EXPECT_EQ(tc.tick_no(), i);
  }
}
