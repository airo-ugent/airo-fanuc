// SPDX-License-Identifier: Apache-2.0
//
// Unit test — epoch-at-CONSUME + the bump-event table. Each of the nine
// bump-causing scenarios is triggered in isolation and asserted to bump the core
// epoch exactly once with the right BumpReason. A target tagged with a pre-bump
// epoch is structurally unexecutable (rejected at consume).

#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"

using namespace airo_fanuc::rt_core;
using airo_fanuc::tick_engine::Vec6;

namespace {

RxSample clean_rx(const Vec6& q) {
  RxSample rx{};
  rx.q_meas = q;
  rx.gates.motion_possible = true;  // all other gates clear
  rx.safety_scale = 1.0;
  return rx;
}

// Drive the core to steady HOLD (streaming, one clean RX processed).
void init_holding(TickCore& tc, const Vec6& q0) {
  tc.begin_streaming(q0);
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, nullptr);  // establishes clean gate baseline; no bump
}

// A long trajectory (stays in TRAJECTORY for the whole test window).
struct Traj {
  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{};
  std::array<Vec6, 2> qd{};
};

}  // namespace

TEST(Epoch, EStopEdgeBumps) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  const std::uint64_t e0 = tc.epoch();
  RxSample rx = clean_rx(q0);
  rx.gates.e_stopped = true;
  tc.tick(&rx, nullptr);
  EXPECT_EQ(tc.epoch(), e0 + 1);
  EXPECT_EQ(tc.bump_count(BumpReason::kEStopEdge), 1u);
  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  EXPECT_EQ(tc.fault(), FaultReason::E_STOP);
}

TEST(Epoch, InErrorBumpsFaultEntry) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  const std::uint64_t e0 = tc.epoch();
  RxSample rx = clean_rx(q0);
  rx.gates.in_error = true;
  tc.tick(&rx, nullptr);
  EXPECT_EQ(tc.epoch(), e0 + 1);
  EXPECT_EQ(tc.bump_count(BumpReason::kFaultEntry), 1u);
  EXPECT_EQ(tc.fault(), FaultReason::IN_ERROR);
}

TEST(Epoch, ContactStopBumpsKillDegraded) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  const std::uint64_t e0 = tc.epoch();
  RxSample rx = clean_rx(q0);
  rx.gates.contact_stop_active = true;  // contact_stop ∈ {2,4}
  rx.contact_stop_status = 2;
  tc.tick(&rx, nullptr);
  EXPECT_EQ(tc.epoch(), e0 + 1);
  EXPECT_EQ(tc.bump_count(BumpReason::kKillDegradedEntry), 1u);
  EXPECT_EQ(tc.fault(), FaultReason::CONTACT_STOP);
}

TEST(Epoch, RxSilentEntryBumpsFromHold) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  const std::uint64_t e0 = tc.epoch();
  // From HOLD (not an active motion): no 100 ms kill; only the 500 ms park bumps.
  const int ticks = static_cast<int>(cfg.rx_silent_park_ms / 1000.0 / cfg.tick.itp_s) + 5;
  for (int i = 0; i < ticks; ++i) {
    tc.tick(nullptr, nullptr);  // RX silence
  }
  EXPECT_EQ(tc.mode(), Mode::RX_SILENT);
  EXPECT_EQ(tc.bump_count(BumpReason::kRxSilentEntry), 1u);
  EXPECT_EQ(tc.bump_count(BumpReason::kSafeFollowEntry), 0u) << "no 100 ms kill from HOLD";
  EXPECT_EQ(tc.epoch(), e0 + 1);
}

TEST(Epoch, RxDegradedBumpsSafeFollowFromTrajectory) {
  RtCoreConfig cfg;
  Vec6 q0{};
  Traj tr;
  tr.q[0] = q0;
  tr.q[1] = Vec6{0.2, 0, 0, 0, 0, 0};
  TickCore tc(cfg);
  init_holding(tc, q0);
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = tr.times.data();
  t.q = tr.q.data();
  t.qd = tr.qd.data();
  t.n = 2;
  t.speed_scale = 1.0;
  // Get into TRAJECTORY (capture completes in ~2 ticks).
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, &t);
  for (int i = 0; i < 5; ++i) {
    RxSample r = clean_rx(tc.q_cmd());
    tc.tick(&r, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);
  const std::uint64_t e0 = tc.epoch();
  // 100 ms of RX silence mid-TRAJECTORY → kill-type SAFE_FOLLOW entry.
  const int ticks = static_cast<int>(cfg.rx_silence_blind_hold_ms / 1000.0 / cfg.tick.itp_s) + 2;
  for (int i = 0; i < ticks; ++i) {
    tc.tick(nullptr, nullptr);
  }
  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  EXPECT_EQ(tc.fault(), FaultReason::RX_DEGRADED);
  EXPECT_EQ(tc.bump_count(BumpReason::kSafeFollowEntry), 1u);
  EXPECT_EQ(tc.epoch(), e0 + 1);
}

TEST(Epoch, DeadmanTripBumps) {
  RtCoreConfig cfg;
  Vec6 q0{};
  Traj tr;
  tr.q[0] = q0;
  tr.q[1] = Vec6{0.2, 0, 0, 0, 0, 0};
  TickCore tc(cfg);
  init_holding(tc, q0);
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = tr.times.data();
  t.q = tr.q.data();
  t.qd = tr.qd.data();
  t.n = 2;
  t.speed_scale = 1.0;
  t.deadman_s = 0.1;  // 100 ms deadman, never kicked
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, &t);
  const std::uint64_t e0 = tc.epoch();
  const int ticks = static_cast<int>(0.1 / cfg.tick.itp_s) + 20;
  for (int i = 0; i < ticks; ++i) {
    RxSample r = clean_rx(tc.q_cmd());
    tc.tick(&r, nullptr);  // never kick()
  }
  EXPECT_GE(tc.bump_count(BumpReason::kDeadmanTrip), 1u);
  EXPECT_EQ(tc.fault(), FaultReason::WATCHDOG_EXPIRED);
  EXPECT_GT(tc.epoch(), e0);
}

TEST(Epoch, DeadmanKickPreventsTrip) {
  RtCoreConfig cfg;
  Vec6 q0{};
  Traj tr;
  tr.q[0] = q0;
  tr.q[1] = Vec6{0.2, 0, 0, 0, 0, 0};
  TickCore tc(cfg);
  init_holding(tc, q0);
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = tr.times.data();
  t.q = tr.q.data();
  t.qd = tr.qd.data();
  t.n = 2;
  t.speed_scale = 1.0;
  t.deadman_s = 0.1;
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, &t);
  const int ticks = static_cast<int>(0.1 / cfg.tick.itp_s) + 20;
  for (int i = 0; i < ticks; ++i) {
    RxSample r = clean_rx(tc.q_cmd());
    tc.kick();  // supervision alive
    tc.tick(&r, nullptr);
  }
  EXPECT_EQ(tc.bump_count(BumpReason::kDeadmanTrip), 0u);
  EXPECT_NE(tc.fault(), FaultReason::WATCHDOG_EXPIRED);
}

// SUPERVISOR_LOST: once the supervisor has beaten at least
// once, a heartbeat lapse > supervisor_lost_s while streaming latches the fault
// and drops to SAFE_FOLLOW (the core holds). Armed only after the first beat.
TEST(Epoch, SupervisorLostBumps) {
  RtCoreConfig cfg;
  cfg.supervisor_lost_s = 0.1;  // 100 ms for a fast test
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  // Arm the watchdog with one heartbeat + tick.
  tc.heartbeat();
  {
    RxSample rx = clean_rx(q0);
    tc.tick(&rx, nullptr);
  }
  const std::uint64_t e0 = tc.epoch();
  // Stop beating; tick past the threshold.
  const int ticks = static_cast<int>(cfg.supervisor_lost_s / cfg.tick.itp_s) + 5;
  for (int i = 0; i < ticks; ++i) {
    RxSample rx = clean_rx(q0);
    tc.tick(&rx, nullptr);  // never heartbeat()
  }
  EXPECT_EQ(tc.fault(), FaultReason::SUPERVISOR_LOST);
  EXPECT_EQ(tc.bump_count(BumpReason::kSupervisorLost), 1u) << "latched once, not per-tick";
  EXPECT_EQ(tc.epoch(), e0 + 1);
  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
}

// A supervisor that keeps beating never trips (parity with DeadmanKickPreventsTrip).
TEST(Epoch, SupervisorHeartbeatPreventsTrip) {
  RtCoreConfig cfg;
  cfg.supervisor_lost_s = 0.1;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  const int ticks = static_cast<int>(cfg.supervisor_lost_s / cfg.tick.itp_s) + 20;
  for (int i = 0; i < ticks; ++i) {
    tc.heartbeat();  // supervisor alive
    RxSample rx = clean_rx(q0);
    tc.tick(&rx, nullptr);
  }
  EXPECT_EQ(tc.bump_count(BumpReason::kSupervisorLost), 0u);
  EXPECT_NE(tc.fault(), FaultReason::SUPERVISOR_LOST);
}

TEST(Epoch, RecoveryCompleteBumpsAfterDwell) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  // Fault → SAFE_FOLLOW.
  {
    RxSample rx = clean_rx(q0);
    rx.gates.contact_stop_active = true;
    rx.contact_stop_status = 2;
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  tc.request_recover();
  const std::uint64_t e0 = tc.epoch();
  // Feed all-clear RX; recovery completes only after the anti-flap dwell.
  const int dwell = static_cast<int>(cfg.antiflap_dwell_ms / 1000.0 / cfg.tick.itp_s);
  bool recovered_early = false;
  for (int i = 0; i < dwell - 3; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());  // all gates clear
    tc.tick(&rx, nullptr);
    if (tc.mode() == Mode::HOLD) recovered_early = true;
  }
  EXPECT_FALSE(recovered_early) << "recover() must wait out the anti-flap dwell";
  for (int i = 0; i < 10; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }
  EXPECT_EQ(tc.mode(), Mode::HOLD);
  EXPECT_EQ(tc.fault(), FaultReason::NONE);
  EXPECT_EQ(tc.bump_count(BumpReason::kRecoveryComplete), 1u);
  EXPECT_EQ(tc.epoch(), e0 + 1);
}

TEST(Epoch, DwellResetsIfConditionReappears) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  {
    RxSample rx = clean_rx(q0);
    rx.gates.contact_stop_active = true;
    rx.contact_stop_status = 2;
    tc.tick(&rx, nullptr);
  }
  tc.request_recover();
  const int dwell = static_cast<int>(cfg.antiflap_dwell_ms / 1000.0 / cfg.tick.itp_s);
  // Clear for most of the dwell, then flap the condition back once.
  for (int i = 0; i < dwell - 5; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }
  {
    RxSample rx = clean_rx(tc.q_cmd());
    rx.gates.contact_stop_active = true;  // flap
    rx.contact_stop_status = 2;
    tc.tick(&rx, nullptr);
  }
  // A few more clear ticks: NOT enough to satisfy a fresh full dwell.
  for (int i = 0; i < 10; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }
  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW) << "flap resets the dwell; no premature recovery";
}

// Epoch-at-CONSUME: a target tagged with a pre-fault epoch is structurally
// unexecutable — rejected at the mailbox pop.
TEST(Epoch, StaleTargetRejectedAtConsume) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  const std::uint64_t stale_epoch = tc.epoch();

  // Fault bumps the epoch.
  {
    RxSample rx = clean_rx(q0);
    rx.gates.e_stopped = true;
    tc.tick(&rx, nullptr);
  }
  ASSERT_GT(tc.epoch(), stale_epoch);

  // Recover so a new motion could otherwise run.
  {
    RxSample rx = clean_rx(q0);
    tc.tick(&rx, nullptr);  // clears e-stop condition
  }
  tc.request_recover();
  for (int i = 0; i < 100; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::HOLD);

  // A trajectory tagged with the STALE (pre-fault) epoch must be rejected.
  Traj tr;
  tr.q[0] = q0;
  tr.q[1] = Vec6{0.1, 0, 0, 0, 0, 0};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = stale_epoch;  // stale
  t.times_ns = tr.times.data();
  t.q = tr.q.data();
  t.qd = tr.qd.data();
  t.n = 2;
  RxSample rx = clean_rx(tc.q_cmd());
  tc.tick(&rx, &t);
  EXPECT_TRUE(tc.last_consume().rejected_stale);
  EXPECT_EQ(tc.mode(), Mode::HOLD) << "stale target does not start a motion";

  // The same target tagged with the LIVE epoch is accepted.
  t.epoch = tc.epoch();
  RxSample rx2 = clean_rx(tc.q_cmd());
  tc.tick(&rx2, &t);
  EXPECT_TRUE(tc.last_consume().accepted);
  EXPECT_EQ(tc.mode(), Mode::CAPTURE);
}
