// SPDX-License-Identifier: Apache-2.0
//
// Unit test — epoch bump-event table COMPLETENESS. test_epoch.cpp triggers each
// of the eight bump reasons in isolation; this file locks down the table as a
// whole: (1) the enum has exactly the eight reasons (a new reason without a test
// breaks the static_assert), (2) the happy path (HOLD / trajectory-to-DONE /
// stop_j preempt) bumps ZERO times — no spurious epoch churn, and (3) every
// epoch increment is ATTRIBUTED to a table entry: Σ bump_counts == epoch (no
// untracked bump can slip in). kSupervisorLost is exercised by test_epoch.cpp's
// SupervisorLost case.

#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"

using namespace airo_fanuc::rt_core;
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

void init_holding(TickCore& tc, const Vec6& q0) {
  tc.begin_streaming(q0);
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, nullptr);
}

std::uint64_t sum_bumps(const TickCore& tc) {
  std::uint64_t s = 0;
  for (std::size_t r = 0; r < static_cast<std::size_t>(BumpReason::kCount); ++r) {
    s += tc.bump_count(static_cast<BumpReason>(r));
  }
  return s;
}

}  // namespace

// The table is exactly {kNone, + 8 bump reasons}. Adding a BumpReason without a
// test (and without updating this count) fails to compile.
static_assert(static_cast<int>(BumpReason::kCount) == 9, "epoch bump-event table must have 8 reasons");

// Happy path: HOLD steady state, a full trajectory to DONE, and a stop_j clean
// preempt all bump the epoch ZERO times.
TEST(BumpTable, HappyPathNeverBumps) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  const std::uint64_t e_after_stream = tc.epoch();

  // 300 clean HOLD ticks.
  for (int i = 0; i < 300; ++i) {
    RxSample rx = clean_rx(q0);
    tc.tick(&rx, nullptr);
  }

  // A short trajectory to DONE.
  std::array<std::int64_t, 2> times{0, 300'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.1, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  t.speed_scale = 1.0;
  {
    RxSample rx = clean_rx(q0);
    tc.tick(&rx, &t);
  }
  bool done = false;
  for (int i = 0; i < 200 && !done; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
    Event e{};
    while (tc.pop_event(e)) {
      if (e.type == EventType::kMotionDone) done = true;
    }
  }
  ASSERT_TRUE(done);

  // A second trajectory, preempted mid-flight by stop_j (clean, non-bumping).
  std::array<std::int64_t, 2> times2{0, 2'000'000'000};
  std::array<Vec6, 2> q2{tc.q_cmd(), Vec6{0.4, 0, 0, 0, 0, 0}};
  Target t2 = t;
  t2.epoch = tc.epoch();
  t2.times_ns = times2.data();
  t2.q = q2.data();
  {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, &t2);
  }
  for (int i = 0; i < 20; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }
  tc.request_stop();
  for (int i = 0; i < 400 && tc.mode() != Mode::HOLD; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }

  EXPECT_EQ(tc.epoch(), e_after_stream) << "no epoch bump on the happy path";
  EXPECT_EQ(sum_bumps(tc), 0u) << "no bump-table entry incremented on the happy path";
}

// Accounting invariant: after a fault + recover + fault + recover sequence,
// every epoch increment is attributed to a table entry (Σ bump_counts == epoch).
TEST(BumpTable, EveryEpochIncrementIsAttributed) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  const int dwell = static_cast<int>(cfg.antiflap_dwell_ms / 1000.0 / cfg.tick.itp_s) + 8;

  auto fault_then_recover = [&](void (*raise)(RxSample&)) {
    {
      RxSample rx = clean_rx(tc.q_cmd());
      raise(rx);
      tc.tick(&rx, nullptr);
    }
    tc.request_recover();
    for (int i = 0; i < dwell; ++i) {
      RxSample rx = clean_rx(tc.q_cmd());  // all-clear → dwell → recover completes
      tc.tick(&rx, nullptr);
    }
  };

  fault_then_recover([](RxSample& r) { r.gates.e_stopped = true; });
  ASSERT_EQ(tc.mode(), Mode::HOLD);
  fault_then_recover([](RxSample& r) {
    r.gates.contact_stop_active = true;
    r.contact_stop_status = 4;
  });
  ASSERT_EQ(tc.mode(), Mode::HOLD);

  // e-stop edge (1) + recovery (1) + kill-degraded (1) + recovery (1) = 4 bumps.
  EXPECT_EQ(tc.epoch(), 4u);
  EXPECT_EQ(sum_bumps(tc), tc.epoch()) << "no untracked epoch increment";
  EXPECT_EQ(tc.bump_count(BumpReason::kEStopEdge), 1u);
  EXPECT_EQ(tc.bump_count(BumpReason::kKillDegradedEntry), 1u);
  EXPECT_EQ(tc.bump_count(BumpReason::kRecoveryComplete), 2u);
}
