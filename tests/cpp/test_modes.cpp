// SPDX-License-Identifier: Apache-2.0
//
// Unit test — TickCore mode machine. HOLD→CAPTURE→TRAJECTORY→settle→
// DONE; stop_j→BRAKE→HOLD; kill-type DEGRADED→SAFE_FOLLOW; graduated RX-silence
// (100 ms kill / 500 ms park); CAPTURE-or-REJECT; slew clip counted-not-faulted;
// SAFE_FOLLOW bounded re-anchor.

#include <array>
#include <cmath>
#include <cstdint>

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

void init_holding(TickCore& tc, const Vec6& q0) {
  tc.begin_streaming(q0);
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, nullptr);
}

bool drain_for(TickCore& tc, EventType want) {
  Event e{};
  bool found = false;
  while (tc.pop_event(e)) {
    if (e.type == want) found = true;
  }
  return found;
}

double max_abs(const Vec6& v) {
  double m = 0.0;
  for (double x : v) m = std::max(m, std::abs(x));
  return m;
}

}  // namespace

TEST(Modes, HoldAfterBeginStreaming) {
  RtCoreConfig cfg;
  Vec6 q0{0.1, -0.2, 0.3, 0, 0, 0};
  TickCore tc(cfg);
  init_holding(tc, q0);
  RxSample rx = clean_rx(q0);
  const Command c = tc.tick(&rx, nullptr);
  EXPECT_EQ(tc.mode(), Mode::HOLD);
  EXPECT_TRUE(c.tx);
  EXPECT_NEAR(c.q_rad[0], q0[0], 1e-9);
}

TEST(Modes, TxParkedBeforeStreaming) {
  RtCoreConfig cfg;
  TickCore tc(cfg);  // not begin_streaming'd
  RxSample rx = clean_rx(Vec6{});
  const Command c = tc.tick(&rx, nullptr);
  EXPECT_FALSE(c.tx) << "no CommandPacket before STREAMING";
}

TEST(Modes, HoldToCaptureToTrajectoryToDone) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  std::array<std::int64_t, 2> times{0, 400'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.1, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};  // rest-to-rest → no qd_end blend
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  t.speed_scale = 1.0;

  bool saw_capture = false, saw_trajectory = false, done = false;
  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);
  if (tc.mode() == Mode::CAPTURE) saw_capture = true;
  for (int i = 0; i < 400 && !done; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());  // perfect tracking → settle converges
    tc.tick(&rx, nullptr);
    if (tc.mode() == Mode::TRAJECTORY) saw_trajectory = true;
    if (drain_for(tc, EventType::kMotionDone)) done = true;
  }
  EXPECT_TRUE(saw_capture);
  EXPECT_TRUE(saw_trajectory);
  EXPECT_TRUE(done) << "trajectory settles to DONE";
  EXPECT_EQ(tc.mode(), Mode::HOLD);
  EXPECT_NEAR(tc.q_cmd()[0], 0.1, 0.01);
}

TEST(Modes, StopJToBrakeToHold) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.5, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  t.speed_scale = 1.0;
  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);
  for (int i = 0; i < 30; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);

  tc.request_stop();
  bool stopped = false;
  for (int i = 0; i < 400; ++i) {
    // measured tracks commanded but with the commanded velocity settling to 0.
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
    if (drain_for(tc, EventType::kMotionStopped)) stopped = true;
    if (tc.mode() == Mode::HOLD) break;
  }
  EXPECT_TRUE(stopped) << "stop_j resolves the motion STOPPED";
  EXPECT_EQ(tc.mode(), Mode::HOLD);
  EXPECT_EQ(tc.epoch(), t.epoch) << "stop_j does NOT bump epoch (clean preempt)";
}

TEST(Modes, ContactStopKillsToSafeFollow) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.5, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);
  for (int i = 0; i < 20; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);

  RxSample rx = clean_rx(tc.q_cmd());
  rx.gates.contact_stop_active = true;
  rx.contact_stop_status = 4;  // ESCP
  tc.tick(&rx, nullptr);
  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  EXPECT_EQ(tc.fault(), FaultReason::CONTACT_STOP);
  EXPECT_TRUE(tc.conditions() & kCondContactStop);
}

TEST(Modes, GraduatedRxSilence) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.5, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);
  for (int i = 0; i < 20; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);

  // 100 ms silence → kill-type SAFE_FOLLOW (RX_DEGRADED); TX keeps flowing.
  const int t100 = static_cast<int>(cfg.rx_silence_blind_hold_ms / 1000.0 / cfg.tick.itp_s) + 2;
  Command c{};
  for (int i = 0; i < t100; ++i) {
    c = tc.tick(nullptr, nullptr);
  }
  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  EXPECT_TRUE(c.tx) << "hold-TX continues at cadence during RX_DEGRADED";

  // Continue to 500 ms total → RX_SILENT park (TX suppressed).
  const int t500 = static_cast<int>(cfg.rx_silent_park_ms / 1000.0 / cfg.tick.itp_s) + 4;
  for (int i = 0; i < t500; ++i) {
    c = tc.tick(nullptr, nullptr);
  }
  EXPECT_EQ(tc.mode(), Mode::RX_SILENT);
  EXPECT_FALSE(c.tx) << "TX parked at RX_SILENT";
}

TEST(Modes, CaptureRejectsFarStart) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  // Trajectory starting > capture_tol (5°) from the commanded pose.
  std::array<std::int64_t, 2> times{0, 400'000'000};
  std::array<Vec6, 2> q{Vec6{0.5, 0, 0, 0, 0, 0}, Vec6{0.6, 0, 0, 0, 0, 0}};  // starts 0.5 rad ≈ 28.6° away
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, &t);
  EXPECT_TRUE(tc.last_consume().rejected_capture);
  EXPECT_EQ(tc.mode(), Mode::HOLD) << "rejected splice does not start a motion";
}

TEST(Modes, SlewClipCountedNotFaulted) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  // A deliberately too-fast trajectory: 0.6 rad on J0 over 24 ms (3 ticks) — the
  // per-tick Hermite step vastly exceeds the slew cap (~0.02 rad/tick) → the
  // clip fires every tick but NEVER faults.
  std::array<std::int64_t, 2> times{0, 24'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.6, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);
  int clipped_ticks = 0;
  bool ever_sustained = false;  // the diagnostic bit is transient (clears once the command catches up)
  for (int i = 0; i < 40; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    const Command c = tc.tick(&rx, nullptr);
    if (c.slew_clipped_joints > 0) ++clipped_ticks;
    if (tc.conditions() & kCondSustainedSlew) ever_sustained = true;
    EXPECT_NE(tc.mode(), Mode::SAFE_FOLLOW) << "slew clip must NEVER fault";
  }
  EXPECT_GT(clipped_ticks, 0);
  EXPECT_GT(tc.total_slew_clips(), 0u);
  EXPECT_TRUE(ever_sustained) << "sustained-clip diagnostic bit set during clipping";
}

TEST(Modes, SafeFollowReanchorBounded) {
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
  const double max_step = cfg.safe_follow_rate_rad_s * cfg.tick.itp_s;
  const Vec6 q_far{0.3, 0, 0, 0, 0, 0};  // measured 0.3 rad away (> 5° deadband)
  Vec6 prev = tc.q_cmd();
  // Skip the 60 ms ramp, then verify bounded re-anchor steps toward measured.
  const int ramp_ticks = static_cast<int>(cfg.rx_silence_qd_ramp_ms / 1000.0 / cfg.tick.itp_s) + 2;
  for (int i = 0; i < ramp_ticks; ++i) {
    RxSample rx = clean_rx(q_far);  // contact-stop condition cleared; still SAFE_FOLLOW
    rx.gates.contact_stop_active = false;
    tc.tick(&rx, nullptr);
  }
  prev = tc.q_cmd();
  for (int i = 0; i < 30; ++i) {
    RxSample rx = clean_rx(q_far);
    tc.tick(&rx, nullptr);
    const double step = std::abs(tc.q_cmd()[0] - prev[0]);
    EXPECT_LE(step, max_step + 1e-9) << "re-anchor step bounded to 15°/s·itp";
    EXPECT_GE(tc.q_cmd()[0], prev[0]) << "commanded moves toward measured";
    prev = tc.q_cmd();
  }
  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW) << "SAFE_FOLLOW exits only via recover()";
}

TEST(Modes, TrajectoryQdEndBlendCompletes) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);
  // Trajectory ending at a NON-zero terminal velocity (grasp-descent style).
  std::array<std::int64_t, 2> times{0, 400'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.1, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{0.25, 0, 0, 0, 0, 0}};  // start at rest, qd_end ≠ 0
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);
  bool done = false;
  double prev_qd = 0.0;
  bool velocity_stepped = false;
  for (int i = 0; i < 400 && !done; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
    const double qd = tc.qd_cmd()[0];
    if (std::abs(qd - prev_qd) > 0.15) velocity_stepped = true;  // no one-tick velocity step
    prev_qd = qd;
    if (drain_for(tc, EventType::kMotionDone)) done = true;
  }
  EXPECT_TRUE(done);
  EXPECT_FALSE(velocity_stepped) << "qd_end blend ramps velocity to 0 (no step at trajectory end)";
  EXPECT_NE(tc.mode(), Mode::SAFE_FOLLOW);
}

// A SUBMITTED brake is a motion, not the universal preempt — `request_stop()` is that, and
// it takes a different path in. So this branch is refused in the same states kServo and
// kTrajectory are refused in, and the RX_SILENT case is the one that matters most: TX is
// gated on `mode_ != RX_SILENT`, so accepting a brake there would resume transmitting from
// the stale anchor the park exists to stop sending.
TEST(Modes, SubmittedBrakeIsRefusedInTheRxSilentPark) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  // Starve RX past rx_silent_park_ms so the core parks TX.
  const int park_ticks = static_cast<int>(cfg.rx_silent_park_ms / (cfg.tick.itp_s * 1000.0)) + 4;
  for (int i = 0; i < park_ticks; ++i) tc.tick(nullptr, nullptr);
  ASSERT_EQ(tc.mode(), Mode::RX_SILENT);
  const Command parked = tc.tick(nullptr, nullptr);
  ASSERT_FALSE(parked.tx) << "precondition: the park must have TX off";

  Target t{};
  t.kind = TargetKind::kBrake;
  t.epoch = tc.epoch();
  t.motion_id = 4242;
  const Command cmd = tc.tick(nullptr, &t);

  EXPECT_EQ(tc.mode(), Mode::RX_SILENT) << "a submitted brake must not move the mode off the park";
  EXPECT_FALSE(cmd.tx) << "and must not resume TX from the stale anchor";
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::REJECTED);
  EXPECT_TRUE(drain_for(tc, EventType::kMotionRejected));
}

// Same gate, out of SAFE_FOLLOW: that state is left only by recover(), which carries the
// anti-flap dwell with it.
TEST(Modes, SubmittedBrakeIsRefusedWhileFaulted) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  RxSample rx = clean_rx(q0);
  rx.gates.contact_stop_active = true;
  rx.contact_stop_status = 4;  // ESCP
  tc.tick(&rx, nullptr);
  ASSERT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  ASSERT_EQ(tc.fault(), FaultReason::CONTACT_STOP);

  Target t{};
  t.kind = TargetKind::kBrake;
  t.epoch = tc.epoch();
  t.motion_id = 99;
  RxSample rx2 = clean_rx(tc.q_cmd());
  tc.tick(&rx2, &t);

  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW) << "the brake must not escape SAFE_FOLLOW";
  EXPECT_EQ(tc.fault(), FaultReason::CONTACT_STOP) << "and must not clear the fault";
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::REJECTED);
}

// The force guard is armed per-motion by a trajectory, with a threshold chosen for it. A
// servo takeover resolves that trajectory, so the guard must not survive into the stream —
// it would police a streamed setpoint against a number belonging to a finished motion.
TEST(Modes, AServoTakeoverDisarmsTheTrajectorysForceGuard) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.5, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target traj{};
  traj.kind = TargetKind::kTrajectory;
  traj.epoch = tc.epoch();
  traj.times_ns = times.data();
  traj.q = q.data();
  traj.qd = qd.data();
  traj.n = 2;
  traj.force_stop_n = 20.0;  // arms the guard
  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &traj);
  for (int i = 0; i < 10; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
  }

  // Preempt with a servo target, then present a wrench well past the trajectory's threshold.
  Target sv{};
  sv.kind = TargetKind::kServo;
  sv.epoch = tc.epoch();
  sv.servo_q = tc.q_cmd();
  sv.servo_duration_s = 0.05;
  RxSample rx1 = clean_rx(tc.q_cmd());
  tc.tick(&rx1, &sv);
  ASSERT_EQ(tc.mode(), Mode::SERVO);

  RxSample hot = clean_rx(tc.q_cmd());
  hot.wrench_valid = true;
  hot.fx = 500.0;  // 25x the trajectory's 20 N
  tc.tick(&hot, nullptr);

  EXPECT_EQ(tc.mode(), Mode::SERVO) << "a disarmed guard must not trip the servo into a brake";
  EXPECT_NE(tc.fault(), FaultReason::FORCE_GUARD);
}

// hold() is the documented way to end a servo stream: a stream never terminates itself,
// so without this the core holds the last streamed target in SERVO indefinitely and never
// reaches HOLD. It is a PREEMPT, not a stop — the distinction the caller sees is the
// motion's terminal status, and it is the only difference from stop_j at this level.
TEST(Modes, HoldEndsAServoStreamAsAPreempt) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  Target sv{};
  sv.kind = TargetKind::kServo;
  sv.epoch = tc.epoch();
  sv.servo_q = Vec6{0.05, 0, 0, 0, 0, 0};
  sv.servo_duration_s = 0.1;
  sv.motion_id = 77;
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &sv); }
  ASSERT_EQ(tc.mode(), Mode::SERVO);
  drain_for(tc, EventType::kMotionDone);  // clear the queue

  tc.request_hold();
  bool preempted = false;
  for (int i = 0; i < 400; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
    if (drain_for(tc, EventType::kMotionPreempted)) preempted = true;
    if (tc.mode() == Mode::HOLD) break;
  }
  EXPECT_TRUE(preempted) << "hold() resolves the stream's target PREEMPTED, not STOPPED";
  EXPECT_EQ(tc.mode(), Mode::HOLD) << "the stream ends at rest";
  EXPECT_EQ(tc.epoch(), sv.epoch) << "hold() does NOT bump epoch (clean preempt)";
}

// The one mode hold() must leave alone. A submitted brake IS the tracked motion, so
// answering a hold there would resolve it a second time and re-seed the deceleration —
// restarting its duration cap and recomputing the decel from an already-decayed velocity.
// Both are silent, because the arm still reaches rest either way; the caller sees it only
// as a motion that reported twice, or reported PREEMPTED for a brake nothing preempted.
TEST(Modes, HoldLeavesABrakeAlreadyHeadingToRestAlone) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  // Build real velocity first: a brake seeded from rest is over in one tick and would not
  // stay in BRAKE long enough for the hold to land on it.
  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.5, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target traj{};
  traj.kind = TargetKind::kTrajectory;
  traj.epoch = tc.epoch();
  traj.times_ns = times.data();
  traj.q = q.data();
  traj.qd = qd.data();
  traj.n = 2;
  traj.motion_id = 11;
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &traj); }
  for (int i = 0; i < 40; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);

  Target brake{};
  brake.kind = TargetKind::kBrake;
  brake.epoch = tc.epoch();
  brake.motion_id = 22;
  { RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd()); tc.tick(&rx, &brake); }
  ASSERT_EQ(tc.mode(), Mode::BRAKE);
  ASSERT_EQ(tc.active_motion_id(), 22u);
  { Event e{}; while (tc.pop_event(e)) {} }

  tc.request_hold();
  { RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd()); tc.tick(&rx, nullptr); }
  EXPECT_EQ(tc.mode(), Mode::BRAKE) << "the brake keeps its own profile";
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::RUNNING) << "and is not resolved early";
  EXPECT_FALSE(drain_for(tc, EventType::kMotionPreempted));

  // It still resolves exactly once, on its own terms, when it reaches rest.
  int terminals = 0;
  for (int i = 0; i < 400; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
    Event e{};
    while (tc.pop_event(e)) {
      if (e.motion_id == 22u && e.type != EventType::kMotionRunning) ++terminals;
    }
    if (tc.mode() == Mode::HOLD) break;
  }
  EXPECT_EQ(tc.mode(), Mode::HOLD);
  EXPECT_EQ(terminals, 1) << "one submitted brake, one terminal event";
}

// A trajectory whose commanded timeline finishes but whose MEASURED joints never arrive.
// The settle detector's timeout is the only thing that ends such a motion: without it the
// motion stays RUNNING forever and every caller blocked in wait() blocks forever. The
// distinct terminal status is what tells the caller the path was commanded but the arm
// did not confirm it — reporting DONE here would be a lie, and FAULTED would be wrong
// because nothing in the core failed.
TEST(Modes, SettleTimeoutEndsAMotionTheArmNeverConfirms) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  std::array<std::int64_t, 2> times{0, 200'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.05, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  t.motion_id = 55;
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &t); }

  // The measured feed lags the command by more than settle_tol_rad and stays there —
  // the shape of a servo that never closes the gap, and indistinguishable from a frozen
  // feed. qd_est reads zero throughout, so only the POSITION term withholds convergence.
  const Vec6 lag{cfg.tick.settle_tol_rad * 10.0, 0, 0, 0, 0, 0};
  bool timed_out = false;
  const int ticks = static_cast<int>(cfg.tick.settle_timeout_s / cfg.tick.itp_s) + 400;
  for (int i = 0; i < ticks; ++i) {
    Vec6 q_meas = tc.q_cmd();
    for (std::size_t j = 0; j < kNumJoints; ++j) q_meas[j] -= lag[j];
    RxSample rx = clean_rx(q_meas);
    tc.tick(&rx, nullptr);
    if (drain_for(tc, EventType::kMotionSettleTimeout)) {
      timed_out = true;
      break;
    }
  }
  EXPECT_TRUE(timed_out) << "a motion the arm never confirms must still resolve";
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::SETTLE_TIMEOUT);
  EXPECT_EQ(tc.fault(), FaultReason::NONE) << "an unconfirmed arrival is not a fault";
}
