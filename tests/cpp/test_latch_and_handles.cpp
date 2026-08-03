// SPDX-License-Identifier: Apache-2.0
//
// Three "exactly once" invariants of the tick core, each of which fails silently
// (no fault, no log) when it breaks:
//
//  1. SUPERVISOR_LOST faults ONCE per lapse. The trip drives SAFE_FOLLOW and does
//     not reset ticks_since_heartbeat_, so the trip condition stays true until the
//     next beat; the per-lapse latch is what stops it re-firing. Re-firing would
//     bump the epoch every tick (rejecting every submitted Target), reset the
//     SAFE_FOLLOW ramp before it can re-anchor, un-park the 500 ms RX-silence park
//     (tx back to 1 into a dead link) and overwrite a higher-severity fault reason.
//
//  2. stop_j is answered in BRAKE. The brake is already decelerating to rest so the
//     profile is left alone, but a submitted Brake target IS the tracked motion and
//     must resolve STOPPED, not DONE.
//
//  3. consume resolves the motion it displaces. The active-motion slot holds ONE
//     motion; every path that overwrites it must emit a terminal event first, or the
//     displaced MotionHandle polls PENDING forever (motion_status finds the id
//     neither in the resolution table nor as the active id). The servo stream is the
//     ordinary case: one target per tick means nothing coalesces in the mailbox, so
//     RealtimeCore's synthetic preempt — which covers only targets that never reach
//     consume — never fires.

#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"

using namespace airo_fanuc::rt_core;
using airo_fanuc::tick_engine::Vec6;

namespace {

// supervisor_lost_s = 3.0 s / itp_s = 8 ms → 375 ticks; round up for margin.
constexpr int kBeatlessTicks = 400;

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

// Feed the core its own command back as the measurement: a robot that tracks
// perfectly, which makes every run below bit-for-bit reproducible.
void tick_tracking(TickCore& tc, int n) {
  for (int i = 0; i < n; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
}

// Both counters DRAIN the event ring as they scan it.
int count_events(TickCore& tc, EventType type, std::uint64_t motion_id) {
  Event e{};
  int n = 0;
  while (tc.pop_event(e)) {
    if (e.type == type && e.motion_id == motion_id) ++n;
  }
  return n;
}

bool is_terminal(EventType t) {
  switch (t) {
    case EventType::kMotionDone:
    case EventType::kMotionSettleTimeout:
    case EventType::kMotionStopped:
    case EventType::kMotionPreempted:
    case EventType::kMotionFaulted:
    case EventType::kMotionRejected:
      return true;
    default:
      return false;
  }
}

int count_terminal(TickCore& tc, std::uint64_t motion_id) {
  Event e{};
  int n = 0;
  while (tc.pop_event(e)) {
    if (e.motion_id == motion_id && is_terminal(e.type)) ++n;
  }
  return n;
}

void drain(TickCore& tc) {
  Event e{};
  while (tc.pop_event(e)) {
  }
}

// A non-owning rest-to-rest trajectory target on J0.
struct TrajTarget {
  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{};
  std::array<Vec6, 2> qd{};
  Target t{};
  TrajTarget(const Vec6& start, const Vec6& end, std::uint64_t epoch, std::uint64_t id) {
    q[0] = start;
    q[1] = end;
    t.kind = TargetKind::kTrajectory;
    t.epoch = epoch;
    t.motion_id = id;
    t.times_ns = times.data();
    t.q = q.data();
    t.qd = qd.data();
    t.n = 2;
    t.speed_scale = 1.0;
  }
};

Target servo_target(const Vec6& q, std::uint64_t epoch, std::uint64_t id) {
  Target t{};
  t.kind = TargetKind::kServo;
  t.epoch = epoch;
  t.motion_id = id;
  t.servo_q = q;
  t.servo_duration_s = 0.05;
  return t;
}

// Drive to a moving TRAJECTORY, then submit a Brake target so the BRAKE is the
// tracked motion. Returns the brake's motion_id.
std::uint64_t enter_tracked_brake(TickCore& tc, const Vec6& q0) {
  TrajTarget a(q0, Vec6{q0[0] + 0.5, 0, 0, 0, 0, 0}, tc.epoch(), 101);
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &a.t); }
  tick_tracking(tc, 30);
  Target br{};
  br.kind = TargetKind::kBrake;
  br.epoch = tc.epoch();
  br.motion_id = 301;
  { RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd()); tc.tick(&rx, &br); }
  return br.motion_id;
}

}  // namespace

// ===========================================================================
// 1. SUPERVISOR_LOST faults exactly once per lapse.
// ===========================================================================

TEST(SupervisorLostLatch, FaultsOncePerLapse) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  Vec6 q0{};
  init_holding(tc, q0);
  tc.heartbeat();
  tick_tracking(tc, 1);

  tick_tracking(tc, kBeatlessTicks);
  ASSERT_EQ(tc.fault(), FaultReason::SUPERVISOR_LOST);
  ASSERT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  EXPECT_EQ(tc.bump_count(BumpReason::kSupervisorLost), 1u);

  // Keep ticking with the beat still absent: the trip condition is still true
  // (ticks_since_heartbeat_ is never reset by the trip) and must not re-fire.
  const std::uint64_t epoch_after_trip = tc.epoch();
  tick_tracking(tc, 200);
  EXPECT_EQ(tc.bump_count(BumpReason::kSupervisorLost), 1u) << "one fault per lapse";
  EXPECT_EQ(tc.epoch(), epoch_after_trip) << "a latched supervisor lapse bumps no further epochs";
  EXPECT_EQ(tc.fault(), FaultReason::SUPERVISOR_LOST);
}

// The 500 ms RX-silence park must HOLD while the supervisor is also lost: tx=0 is
// the whole point of the park, and it is the fault entry (which sets SAFE_FOLLOW)
// that un-parks it.
TEST(SupervisorLostLatch, RxSilenceParkHolds) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  Vec6 q0{};
  init_holding(tc, q0);
  tc.heartbeat();
  tick_tracking(tc, 1);

  for (int i = 0; i < kBeatlessTicks; ++i) {
    tc.tick(nullptr, nullptr);  // no RX and no beat
  }
  const std::uint64_t epoch_after = tc.epoch();
  int tx_ticks = 0;
  for (int i = 0; i < 100; ++i) {
    if (tc.tick(nullptr, nullptr).tx) ++tx_ticks;
  }
  EXPECT_EQ(tc.mode(), Mode::RX_SILENT) << "the park holds";
  EXPECT_EQ(tx_ticks, 0) << "a parked core must not stream into a dead link";
  EXPECT_EQ(tc.epoch(), epoch_after) << "no epoch churn, so a submitted Target can still be accepted";
  EXPECT_EQ(tc.bump_count(BumpReason::kSupervisorLost), 1u);
}

// A latched supervisor lapse must not overwrite a higher-severity fault reason:
// process_gates orders e-stop above everything, and it runs earlier in the same
// tick than the liveness check.
TEST(SupervisorLostLatch, EStopReasonSurvives) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  Vec6 q0{};
  init_holding(tc, q0);
  tc.heartbeat();
  tick_tracking(tc, 1);
  tick_tracking(tc, kBeatlessTicks);
  ASSERT_EQ(tc.fault(), FaultReason::SUPERVISOR_LOST);

  RxSample rx = clean_rx(tc.q_cmd());
  rx.gates.e_stopped = true;
  tc.tick(&rx, nullptr);
  EXPECT_EQ(tc.fault(), FaultReason::E_STOP) << "the operator-visible reason is the e-stop";
  EXPECT_NE(tc.conditions() & kCondEStop, 0u);
}

// The latch is per-LAPSE, not once-per-process: a beat clears it so a second lapse
// faults again.
TEST(SupervisorLostLatch, SecondLapseFaultsAgain) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  Vec6 q0{};
  init_holding(tc, q0);
  tc.heartbeat();
  tick_tracking(tc, 1);
  tick_tracking(tc, kBeatlessTicks);
  ASSERT_EQ(tc.fault(), FaultReason::SUPERVISOR_LOST);

  // Supervisor comes back and the caller recovers (antiflap_dwell_ms = 500 ms).
  tc.request_recover();
  for (int i = 0; i < 100; ++i) {
    tc.heartbeat();
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.fault(), FaultReason::NONE);
  ASSERT_EQ(tc.mode(), Mode::HOLD);

  // Second lapse.
  tick_tracking(tc, kBeatlessTicks);
  EXPECT_EQ(tc.fault(), FaultReason::SUPERVISOR_LOST);
  EXPECT_EQ(tc.bump_count(BumpReason::kSupervisorLost), 2u);
}

// ===========================================================================
// 2. stop_j is answered in BRAKE — outcome only, profile untouched.
// ===========================================================================

TEST(StopInBrake, SubmittedBrakeResolvesStoppedNotDone) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  Vec6 q0{};
  init_holding(tc, q0);
  const std::uint64_t brake_id = enter_tracked_brake(tc, q0);
  ASSERT_EQ(tc.mode(), Mode::BRAKE);
  ASSERT_EQ(tc.active_motion_id(), brake_id);
  ASSERT_EQ(tc.active_motion_status(), MotionStatus::RUNNING);
  drain(tc);

  tc.request_stop();
  tick_tracking(tc, 1);
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::STOPPED) << "the caller stopped it";
  EXPECT_EQ(count_events(tc, EventType::kMotionStopped, brake_id), 1);
  EXPECT_EQ(tc.mode(), Mode::BRAKE) << "still decelerating; the stop changes the outcome, not the mode";

  bool reached_hold = false;
  int done_events = 0;
  for (int i = 0; i < 400 && !reached_hold; ++i) {
    tick_tracking(tc, 1);
    done_events += count_events(tc, EventType::kMotionDone, brake_id);
    if (tc.mode() == Mode::HOLD) reached_hold = true;
  }
  EXPECT_TRUE(reached_hold) << "the brake still reaches rest";
  EXPECT_EQ(done_events, 0) << "a stopped motion never reports DONE";
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::STOPPED);
}

// The stop must not replan the deceleration: Brake::seed resets elapsed_ticks_
// (restarting the brake_max_duration_s cap) and recomputes max_velocity from the
// decayed qd_cmd, so answering the stop by re-seeding would change the profile.
TEST(StopInBrake, DecelerationProfileIsUnchanged) {
  auto run = [](bool with_stop) {
    RtCoreConfig cfg;
    TickCore tc(cfg);
    Vec6 q0{};
    init_holding(tc, q0);
    enter_tracked_brake(tc, q0);
    if (with_stop) tc.request_stop();
    std::vector<double> q0_trace;
    for (int i = 0; i < 400; ++i) {
      tick_tracking(tc, 1);
      q0_trace.push_back(tc.q_cmd()[0]);
      if (tc.mode() == Mode::HOLD) break;
    }
    return q0_trace;
  };
  const std::vector<double> plain = run(false);
  const std::vector<double> stopped = run(true);
  ASSERT_FALSE(plain.empty());
  ASSERT_EQ(plain.size(), stopped.size()) << "the stop must not lengthen or shorten the brake";
  for (std::size_t i = 0; i < plain.size(); ++i) {
    EXPECT_DOUBLE_EQ(plain[i], stopped[i]) << "commanded J0 diverged at brake tick " << i;
  }
}

// ===========================================================================
// 3. consume resolves the motion it displaces.
// ===========================================================================

TEST(ConsumeResolvesDisplaced, ServoOverServo) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  Vec6 q0{};
  init_holding(tc, q0);

  Target s1 = servo_target(Vec6{0.01, 0, 0, 0, 0, 0}, tc.epoch(), 101);
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &s1); }
  ASSERT_EQ(tc.mode(), Mode::SERVO);
  ASSERT_EQ(tc.active_motion_id(), 101u);
  // The FIRST servo displaces nothing, so it must not synthesise a preempt (an
  // unset active slot carries motion_id 0).
  EXPECT_EQ(count_events(tc, EventType::kMotionPreempted, 0u), 0);

  Target s2 = servo_target(Vec6{0.02, 0, 0, 0, 0, 0}, tc.epoch(), 102);
  { RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd()); tc.tick(&rx, &s2); }
  EXPECT_EQ(tc.active_motion_id(), 102u);
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::RUNNING);
  EXPECT_EQ(count_terminal(tc, 101u), 1) << "the displaced servo id resolves here or nowhere";
}

TEST(ConsumeResolvesDisplaced, ServoOverTrajectory) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  Vec6 q0{};
  init_holding(tc, q0);
  TrajTarget a(q0, Vec6{0.5, 0, 0, 0, 0, 0}, tc.epoch(), 101);
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &a.t); }
  tick_tracking(tc, 30);
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);
  drain(tc);

  Target sv = servo_target(tc.q_cmd(), tc.epoch(), 202);
  { RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd()); tc.tick(&rx, &sv); }
  ASSERT_EQ(tc.mode(), Mode::SERVO) << "the servo takes over";
  EXPECT_EQ(tc.active_motion_id(), 202u);
  EXPECT_EQ(count_terminal(tc, 101u), 1) << "the trajectory the servo replaced resolves";
}

TEST(ConsumeResolvesDisplaced, HoldOverASubmittedBrake) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  Vec6 q0{};
  init_holding(tc, q0);
  const std::uint64_t brake_id = enter_tracked_brake(tc, q0);
  ASSERT_EQ(tc.mode(), Mode::BRAKE);
  drain(tc);

  Target h{};
  h.kind = TargetKind::kHold;
  h.epoch = tc.epoch();
  h.motion_id = 401;
  { RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd()); tc.tick(&rx, &h); }
  EXPECT_EQ(tc.active_motion_id(), 401u);
  EXPECT_EQ(count_terminal(tc, brake_id), 1) << "the brake the hold replaced resolves";
}
