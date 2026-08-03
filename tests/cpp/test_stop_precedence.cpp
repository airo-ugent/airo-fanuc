// SPDX-License-Identifier: Apache-2.0
//
// STOP precedence over a same-tick trajectory/servo. stop_j is the universal
// preempt: it takes effect within ≤1 tick and ALWAYS wins. TickCore::tick handles a drained stop_j (step 2) BEFORE it
// consumes the mailbox target (step 7); request_stop() does NOT bump the motion
// epoch, so absent a guard a trajectory/servo that raced the stop into the SAME
// 8 ms window (e.g. a direct FanucDriver caller: TrajectoryMonitor.stop_j() vs a
// concurrent move_trajectory) would activate right after the stop's brake — the
// STOP would be structurally swallowed.
//
// The fix is CAUSAL-ORDER, not "same tick": RealtimeCore stamps every submitted
// Target with the caller-side stop generation and, at consume, flags a target
// superseded when a stop_j was issued AFTER it was submitted. That distinguishes
//   * submit-then-stop (submit → stop_j) → superseded → REJECTED, the stop wins;
//   * brake-then-submit (stop_j → submit) → NOT superseded → accepted, even from
//     an already-steady pose where the (no-op) stop and the submit land in the
//     SAME tick — the contract a caller relies on when it defensively stops
//     before every move.
//
// Tests 1/1b/2 drive TickCore directly (deterministic) to prove the consume
// gate; tests 3/4 drive the full RealtimeCore over UDP loopback to prove the
// stop_gen causal-order wiring end-to-end.

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "rt_core/realtime_core.hpp"
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

// A non-owning rest-to-rest trajectory target from `start` to `end` on J0.
struct TrajTarget {
  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{};
  std::array<Vec6, 2> qd{};
  Target t{};
  TrajTarget(const Vec6& start, const Vec6& end, std::uint64_t epoch) {
    q[0] = start;
    q[1] = end;
    t.kind = TargetKind::kTrajectory;
    t.epoch = epoch;
    t.times_ns = times.data();
    t.q = q.data();
    t.qd = qd.data();
    t.n = 2;
    t.speed_scale = 1.0;
  }
};

bool saw_event(TickCore& tc, EventType type, std::uint64_t motion_id) {
  Event e{};
  bool found = false;
  while (tc.pop_event(e)) {
    if (e.type == type && e.motion_id == motion_id) found = true;
  }
  return found;
}

}  // namespace

// ===========================================================================
// TickCore consume-gate tests (deterministic).
// ===========================================================================

// TEST 1 — a stop_j + a superseded TRAJECTORY consumed in the SAME tick: the
// STOP wins. The trajectory must NOT activate; it resolves REJECTED. The new
// trajectory heads the OPPOSITE way (−J0) from the in-flight one (+J0), so "the
// robot ran the raced trajectory" would show as J0 crossing negative.
TEST(StopPrecedence, SameTickStopSupersedesTrajectory) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  // Motion A: drive +J0 and get moving. Distinct non-zero motion_ids so per-id
  // event scans don't collide (TickCore doesn't assign ids; RealtimeCore does).
  TrajTarget a(q0, Vec6{0.5, 0, 0, 0, 0, 0}, tc.epoch());
  a.t.motion_id = 101;
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &a.t); }
  for (int i = 0; i < 30; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);
  ASSERT_GT(tc.qd_cmd()[0], 0.0) << "A is actively moving +J0";

  // Motion B: a VALID splice (starts at the current commanded pose) heading −J0.
  TrajTarget b(tc.q_cmd(), Vec6{-0.5, 0, 0, 0, 0, 0}, tc.epoch());
  b.t.motion_id = 202;

  // THE RACE: stop_j() is drained AND B is consumed-as-superseded in ONE tick
  // (exactly what RealtimeCore produces for a submit-then-stop_j race).
  tc.request_stop();
  {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, &b.t, /*consume_superseded=*/true);
  }

  // The STOP wins: B did NOT activate. Mode is BRAKE (never CAPTURE/TRAJECTORY).
  EXPECT_EQ(tc.mode(), Mode::BRAKE) << "stop's brake wins; the raced trajectory must not activate";
  EXPECT_NE(tc.mode(), Mode::CAPTURE);

  // B resolved terminally REJECTED (not PENDING/RUNNING/DONE) — no hung handle.
  EXPECT_FALSE(tc.last_consume().accepted);
  EXPECT_EQ(tc.active_motion_id(), b.t.motion_id);
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::REJECTED);

  bool b_rejected = false, b_running = false, a_stopped = false;
  Event e{};
  while (tc.pop_event(e)) {
    if (e.motion_id == b.t.motion_id && e.type == EventType::kMotionRejected) b_rejected = true;
    if (e.motion_id == b.t.motion_id && e.type == EventType::kMotionRunning) b_running = true;
    if (e.motion_id == a.t.motion_id && e.type == EventType::kMotionStopped) a_stopped = true;
  }
  EXPECT_TRUE(b_rejected) << "raced trajectory resolves REJECTED";
  EXPECT_FALSE(b_running) << "raced trajectory must never announce RUNNING";
  EXPECT_TRUE(a_stopped) << "the in-flight motion resolves STOPPED";
  EXPECT_EQ(tc.epoch(), a.t.epoch) << "stop_j does not bump epoch";

  // Drive to rest: the robot brakes and NEVER heads toward B's (−J0) target.
  double min_q0 = tc.q_cmd()[0];
  bool reached_hold = false;
  for (int i = 0; i < 400; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
    min_q0 = std::min(min_q0, tc.q_cmd()[0]);
    if (tc.mode() == Mode::HOLD) { reached_hold = true; break; }
  }
  EXPECT_TRUE(reached_hold) << "brake settles to HOLD";
  EXPECT_GT(min_q0, -0.05) << "J0 never heads toward the superseded (−J0) target";
}

// TEST 1b — same precedence covers a superseded SERVO racing the stop.
TEST(StopPrecedence, SameTickStopSupersedesServo) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  Target sv{};
  sv.kind = TargetKind::kServo;
  sv.epoch = tc.epoch();
  sv.motion_id = 4242;
  sv.servo_q = Vec6{0.01, 0, 0, 0, 0, 0};  // a small step from q_cmd (0)
  sv.servo_duration_s = 0.1;

  tc.request_stop();
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &sv, /*consume_superseded=*/true); }

  EXPECT_NE(tc.mode(), Mode::SERVO) << "the raced servo must not activate";
  EXPECT_TRUE(tc.mode() == Mode::HOLD || tc.mode() == Mode::BRAKE) << "stop keeps the robot at rest";
  EXPECT_FALSE(tc.last_consume().accepted);
  EXPECT_EQ(tc.active_motion_id(), sv.motion_id);
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::REJECTED);
  EXPECT_TRUE(saw_event(tc, EventType::kMotionRejected, sv.motion_id));
}

// TEST 2 — the LEGITIMATE brake-then-submit pattern: stop_j, wait several ticks
// to reach HOLD, THEN submit a NON-superseded trajectory in a LATER tick. It
// MUST be accepted (the submit carries the post-stop generation → not
// superseded). Guards against over-rejecting.
TEST(StopPrecedence, BrakeThenSubmitLaterTickIsAccepted) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  TrajTarget a(q0, Vec6{0.3, 0, 0, 0, 0, 0}, tc.epoch());
  a.t.motion_id = 111;
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &a.t); }
  for (int i = 0; i < 20; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);

  tc.request_stop();
  { RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd()); tc.tick(&rx, nullptr); }
  EXPECT_EQ(tc.mode(), Mode::BRAKE);

  bool reached_hold = false;
  for (int i = 0; i < 400; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
    if (tc.mode() == Mode::HOLD) { reached_hold = true; break; }
  }
  ASSERT_TRUE(reached_hold) << "robot brakes to HOLD before the later submit";

  // A LATER tick submits a fresh (non-superseded) trajectory — MUST be accepted.
  const Vec6 q_hold = tc.q_cmd();
  TrajTarget b(q_hold, Vec6{q_hold[0] + 0.2, 0, 0, 0, 0, 0}, tc.epoch());
  b.t.motion_id = 222;
  { RxSample rx = clean_rx(q_hold); tc.tick(&rx, &b.t, /*consume_superseded=*/false); }

  EXPECT_TRUE(tc.last_consume().accepted) << "brake-then-submit (later tick) must be accepted";
  EXPECT_EQ(tc.mode(), Mode::CAPTURE) << "the later trajectory activates (CAPTURE → TRAJECTORY)";
  EXPECT_EQ(tc.active_motion_id(), b.t.motion_id);
  EXPECT_EQ(tc.active_motion_status(), MotionStatus::RUNNING);

  bool done = false, saw_traj = false;
  for (int i = 0; i < 600 && !done; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
    if (tc.mode() == Mode::TRAJECTORY) saw_traj = true;
    if (saw_event(tc, EventType::kMotionDone, b.t.motion_id)) done = true;
  }
  EXPECT_TRUE(saw_traj);
  EXPECT_TRUE(done) << "the accepted trajectory runs to DONE";
  EXPECT_NEAR(tc.q_cmd()[0], q_hold[0] + 0.2, 0.01);
}

// ===========================================================================
// RealtimeCore causal-order (stop_gen) loopback tests — exercise the REAL
// caller-side stop-generation wiring, not the hand-fed boolean.
// ===========================================================================
namespace {

std::int64_t now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}
void be_store_u32(std::uint8_t* p, std::uint32_t v) {
  p[0] = static_cast<std::uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<std::uint8_t>(v & 0xFF);
}
std::uint32_t be_load_u32(const std::uint8_t* p) {
  return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
         (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
}
// Read a big-endian float64 from a wire buffer (test host is x86-64 LE).
double be_read_f64(const std::uint8_t* p) {
  std::uint8_t tmp[8];
  for (int i = 0; i < 8; ++i) tmp[i] = p[7 - i];
  double out;
  std::memcpy(&out, tmp, 8);
  return out;
}
// Store a native float32 as big-endian (test host LE → BE wire).
void be_store_f32(std::uint8_t* p, float f) {
  std::uint8_t tmp[4];
  std::memcpy(tmp, &f, 4);
  p[0] = tmp[3];
  p[1] = tmp[2];
  p[2] = tmp[1];
  p[3] = tmp[0];
}
// Valid 416 B type-204: all gates clear, motion_possible=1, safety_scale=1.0.
// joint_angle[0] echoes the last commanded J0 (deg) so the fake models a robot
// that TRACKS its command, which is what the brake/settle assertions read back.
// Other joints stay 0 (unused by tests).
void build_status(std::uint8_t* buf, std::uint32_t seq, double j0_deg = 0.0) {
  std::memset(buf, 0, 416);
  be_store_u32(buf + 0, 204);
  be_store_u32(buf + 4, 3);
  be_store_u32(buf + 8, seq);
  buf[12] = 0x01;                              // motion_possible
  be_store_f32(buf + 56, static_cast<float>(j0_deg));  // joint_angle[0] echoes the command
  be_store_u32(buf + 128, 0x3F800000);         // safety_scale = 1.0f
}
void sleep_ms(int ms) {
  struct timespec ts { ms / 1000, (ms % 1000) * 1'000'000L };
  nanosleep(&ts, nullptr);
}

// Streams type-204 status to the core at ~8 ms until stopped.
struct LoopbackRig {
  int peer{-1};
  std::uint16_t port{0};
  std::atomic<bool> peer_stop{false};
  std::thread peer_thr;
  RealtimeCore* core{nullptr};

  LoopbackRig() {
    peer = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ::bind(peer, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    socklen_t alen = sizeof(addr);
    ::getsockname(peer, reinterpret_cast<sockaddr*>(&addr), &alen);
    port = ntohs(addr.sin_port);
    peer_thr = std::thread([this] {
      std::uint8_t rxbuf[2048];
      std::uint8_t status[416];
      sockaddr_in from{};
      socklen_t fl = sizeof(from);
      bool have_addr = false;
      sockaddr_in core_addr{};
      std::int64_t last_send = 0;
      std::uint32_t seq = 1;
      double cmd_j0_deg = 0.0;  // last commanded J0 (deg) — echoed back as measured
      while (!peer_stop.load(std::memory_order_relaxed)) {
        for (;;) {
          const ssize_t r = ::recvfrom(peer, rxbuf, sizeof(rxbuf), MSG_DONTWAIT,
                                       reinterpret_cast<sockaddr*>(&from), &fl);
          if (r < 0) break;
          core_addr = from;
          have_addr = true;
          // GetCapability (type 7, 28 B): reply with a type-7 result advertising
          // v3 / 8 ms so the driver's handshake proceeds to Start (the real
          // controller requires this exchange; HIL-measured).
          if (r >= 28 && be_load_u32(rxbuf) == 7u) {
            std::uint8_t cap[28] = {};
            be_store_u32(cap + 0, 7u);   // packet_type
            be_store_u32(cap + 4, 3u);   // version_no
            be_store_u32(cap + 8, 1u);   // id
            be_store_u32(cap + 12, 8u);  // sampling_rate [ms]
            be_store_u32(cap + 20, 3u);  // available_version
            (void)::sendto(peer, cap, sizeof(cap), 0, reinterpret_cast<sockaddr*>(&from), fl);
          }
          // A 344 B type-201 CommandPacket carries command_pos[0] (BE f64, deg) at
          // offset 16; track it so the status echoes a tracking robot.
          if (r >= 344) cmd_j0_deg = be_read_f64(rxbuf + 16);
        }
        const std::int64_t now = now_ns();
        if (have_addr && (now - last_send) >= 8'000'000) {
          build_status(status, seq++, cmd_j0_deg);
          (void)::sendto(peer, status, sizeof(status), 0,
                         reinterpret_cast<sockaddr*>(&core_addr), sizeof(core_addr));
          last_send = now;
        }
        struct timespec ts { 0, 500'000 };
        nanosleep(&ts, nullptr);
      }
    });
  }
  ~LoopbackRig() {
    peer_stop.store(true, std::memory_order_relaxed);
    if (peer_thr.joinable()) peer_thr.join();
    if (peer >= 0) ::close(peer);
  }
};

std::vector<Vec6> j0_line(double a, double b) {
  return {Vec6{a, 0, 0, 0, 0, 0}, Vec6{b, 0, 0, 0, 0, 0}};
}

// wait_ready() returns as soon as streaming_ready_ is set (on the RX-ingest
// path, where begin_streaming → HOLD runs); the published SNAPSHOT only updates
// on the next timer tick. So poll the snapshot for the mode rather than reading
// it immediately.
bool poll_mode(RealtimeCore& core, Mode want, int timeout_ms) {
  const std::int64_t deadline = now_ns() + static_cast<std::int64_t>(timeout_ms) * 1'000'000LL;
  while (now_ns() < deadline) {
    if (core.snapshot().mode == want) return true;
    sleep_ms(2);
  }
  return core.snapshot().mode == want;
}

}  // namespace

// TEST 3 (submit-then-stop, end-to-end): submit a trajectory, THEN immediately stop_j.
// The trajectory was stamped with the pre-stop generation, so the later stop_j
// supersedes it — the robot must NOT run toward the target and the motion must
// resolve terminally non-DONE (REJECTED if caught same-tick, STOPPED if it
// briefly activated before the stop preempted it — either way the STOP holds).
TEST(StopPrecedence, LoopbackSubmitThenStopHoldsTheStop) {
  LoopbackRig rig;
  RtCoreConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.sm_port = rig.port;
  RealtimeCore core(cfg);
  ASSERT_TRUE(core.start());
  rig.core = &core;
  ASSERT_TRUE(core.wait_ready(3.0)) << "core prerolls to STREAMING/HOLD";
  ASSERT_TRUE(poll_mode(core, Mode::HOLD, 500)) << "steady HOLD before submit";

  const std::vector<std::int64_t> times{0, 1'000'000'000};
  const std::uint64_t mid = core.submit_trajectory(times, j0_line(0.0, 0.4), j0_line(0.0, 0.0),
                                                    1.0, 0.0087, 0.035, 2.0, 0.0, 0.0);
  core.stop_j();  // issued AFTER the submit → supersedes it
  ASSERT_GT(mid, 0u);

  // The motion resolves terminally non-DONE; the robot never runs toward 0.4.
  MotionStatus st = MotionStatus::PENDING;
  double max_q0 = 0.0;
  const std::int64_t deadline = now_ns() + 1'500'000'000LL;
  bool terminal = false;
  while (now_ns() < deadline) {
    st = core.motion_status(mid);
    max_q0 = std::max(max_q0, std::abs(core.snapshot().q_cmd[0]));
    if (st == MotionStatus::REJECTED || st == MotionStatus::STOPPED) { terminal = true; break; }
    sleep_ms(2);
  }
  // Let a few more ticks pass to catch any late commanded motion.
  for (int i = 0; i < 50; ++i) { max_q0 = std::max(max_q0, std::abs(core.snapshot().q_cmd[0])); sleep_ms(2); }

  core.stop();
  EXPECT_TRUE(terminal) << "superseded motion resolved terminally (status=" << static_cast<int>(st) << ")";
  EXPECT_NE(st, MotionStatus::DONE) << "the stop must prevent the trajectory from completing";
  EXPECT_LT(max_q0, 0.05) << "robot must not run toward the superseded (0.4 rad) target";
}

// TEST 4 (brake-then-submit, end-to-end): stop_j, wait to steady HOLD, THEN
// submit. The submit carries the post-stop generation → NOT superseded → it must
// be accepted and RUN (q_cmd advances toward the target). This is the contract a
// defensively-stopping caller relies on, and it guards the supersede rule against
// over-rejecting.
TEST(StopPrecedence, LoopbackStopThenSubmitRuns) {
  LoopbackRig rig;
  RtCoreConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.sm_port = rig.port;
  RealtimeCore core(cfg);
  ASSERT_TRUE(core.start());
  rig.core = &core;
  ASSERT_TRUE(core.wait_ready(3.0));
  ASSERT_TRUE(poll_mode(core, Mode::HOLD, 500));

  core.stop_j();  // quiesce first (a no-op from rest) — the brake-then-submit order
  sleep_ms(60);   // let the (no-op) stop drain over a few ticks; robot stays HOLD
  ASSERT_TRUE(poll_mode(core, Mode::HOLD, 500));

  const std::vector<std::int64_t> times{0, 1'000'000'000};
  const std::uint64_t mid = core.submit_trajectory(times, j0_line(0.0, 0.4), j0_line(0.0, 0.0),
                                                    1.0, 0.0087, 0.035, 2.0, 0.0, 0.0);
  ASSERT_GT(mid, 0u);

  // The trajectory must activate and drive q_cmd toward 0.4 — never REJECTED.
  double max_q0 = 0.0;
  bool ever_rejected = false;
  const std::int64_t deadline = now_ns() + 2'000'000'000LL;
  while (now_ns() < deadline) {
    const MotionStatus st = core.motion_status(mid);
    if (st == MotionStatus::REJECTED) ever_rejected = true;
    max_q0 = std::max(max_q0, core.snapshot().q_cmd[0]);
    if (max_q0 > 0.3) break;
    sleep_ms(2);
  }

  core.stop();
  EXPECT_FALSE(ever_rejected) << "a trajectory submitted AFTER the stop must NOT be superseded";
  EXPECT_GT(max_q0, 0.3) << "the accepted trajectory ran (q_cmd advanced toward 0.4)";
}
