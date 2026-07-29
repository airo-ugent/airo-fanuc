// SPDX-License-Identifier: Apache-2.0
//
// L1 — TickCore basics + the zero-allocation-on-the-hot-path proof (PLAN.md
// §5.2). A global operator-new hook counts EVERY heap allocation; the pure tick
// path (HOLD and TRAJECTORY playback) must add ZERO after warmup.

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <new>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"

using namespace airo_fanuc::rt_core;
using airo_fanuc::tick_engine::Vec6;

// ---------------------------------------------------------------------------
// Global allocation counter (active only inside the measurement window).
// ---------------------------------------------------------------------------
namespace {
std::atomic<std::uint64_t> g_alloc{0};
std::atomic<bool> g_count{false};
}  // namespace

void* operator new(std::size_t n) {
  if (g_count.load(std::memory_order_relaxed)) g_alloc.fetch_add(1, std::memory_order_relaxed);
  void* p = std::malloc(n ? n : 1);
  if (!p) throw std::bad_alloc();
  return p;
}
void* operator new[](std::size_t n) {
  if (g_count.load(std::memory_order_relaxed)) g_alloc.fetch_add(1, std::memory_order_relaxed);
  void* p = std::malloc(n ? n : 1);
  if (!p) throw std::bad_alloc();
  return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

namespace {

RxSample clean_rx(const Vec6& q, const Vec6& qd = Vec6{}) {
  RxSample rx{};
  rx.q_meas = q;
  rx.qd_est = qd;
  rx.gates.motion_possible = true;
  rx.safety_scale = 1.0;
  return rx;
}

}  // namespace

TEST(TickCore, BeginStreamingHolds) {
  RtCoreConfig cfg;
  Vec6 q0{0.1, 0.2, -0.3, 0.4, -0.5, 0.6};
  TickCore tc(cfg);
  tc.begin_streaming(q0);
  RxSample rx = clean_rx(q0);
  const Command c = tc.tick(&rx, nullptr);
  EXPECT_EQ(tc.mode(), Mode::HOLD);
  EXPECT_TRUE(c.tx);
  for (int j = 0; j < 6; ++j) EXPECT_NEAR(c.q_rad[j], q0[j], 1e-12);
}

TEST(TickCore, ForceGuardTripsToStoppedNoEpochBump) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  tc.begin_streaming(q0);
  { RxSample rx = clean_rx(q0); tc.tick(&rx, nullptr); }

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
  t.force_stop_n = 10.0;  // arm the force-guard at 10 N
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &t); }
  for (int i = 0; i < 20; ++i) { RxSample rx = clean_rx(tc.q_cmd()); tc.tick(&rx, nullptr); }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);
  const std::uint64_t e0 = tc.epoch();

  // A wrench above the threshold trips the guard within one tick.
  RxSample rx = clean_rx(tc.q_cmd());
  rx.fs_type = 1;  // EMBEDDED → wrench valid
  rx.wrench_valid = true;
  rx.fx = 15.0;
  rx.fz = 12.0;  // |F| = sqrt(15²+12²) ≈ 19.2 N > 10
  tc.tick(&rx, nullptr);

  bool force_evt = false, stopped = false;
  Event e{};
  while (tc.pop_event(e)) {
    if (e.type == EventType::kForceGuardTrip) force_evt = true;
    if (e.type == EventType::kMotionStopped) stopped = true;
  }
  EXPECT_TRUE(force_evt);
  EXPECT_TRUE(stopped);
  EXPECT_EQ(tc.mode(), Mode::BRAKE) << "force trip → internal stop_j → BRAKE";
  EXPECT_EQ(tc.epoch(), e0) << "force-guard trip does NOT bump epoch (expected grasp stop)";
}

TEST(TickCore, ForceGuardDisarmedWithoutThreshold) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  tc.begin_streaming(q0);
  { RxSample rx = clean_rx(q0); tc.tick(&rx, nullptr); }
  std::array<std::int64_t, 2> times{0, 2'000'000'000};
  std::array<Vec6, 2> q{q0, Vec6{0.5, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;  // no force_stop_n → disarmed
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &t); }
  for (int i = 0; i < 20; ++i) { RxSample rx = clean_rx(tc.q_cmd()); tc.tick(&rx, nullptr); }
  RxSample rx = clean_rx(tc.q_cmd());
  rx.fs_type = 1;
  rx.wrench_valid = true;
  rx.fx = 500.0;  // huge, but guard disarmed
  tc.tick(&rx, nullptr);
  EXPECT_EQ(tc.mode(), Mode::TRAJECTORY) << "disarmed force-guard ignores the wrench";
}

// The headline zero-alloc proof: HOLD and TRAJECTORY playback add 0 heap allocs.
TEST(TickCore, ZeroAllocOnHotPath) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  tc.begin_streaming(q0);

  // ---- HOLD steady state ----
  for (int i = 0; i < 200; ++i) { RxSample rx = clean_rx(q0); tc.tick(&rx, nullptr); }  // warmup
  {
    const std::uint64_t a0 = g_alloc.load();
    g_count.store(true, std::memory_order_relaxed);
    for (int i = 0; i < 2000; ++i) {
      RxSample rx = clean_rx(q0);
      tc.tick(&rx, nullptr);
    }
    g_count.store(false, std::memory_order_relaxed);
    EXPECT_EQ(g_alloc.load() - a0, 0u) << "HOLD tick path allocated";
  }

  // ---- TRAJECTORY playback steady state ----
  std::array<std::int64_t, 2> times{0, 30'000'000'000LL};  // 30 s: stays in kPlaying the whole window
  std::array<Vec6, 2> q{q0, Vec6{0.5, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  { RxSample rx = clean_rx(q0); tc.tick(&rx, &t); }
  for (int i = 0; i < 100; ++i) { RxSample rx = clean_rx(tc.q_cmd()); tc.tick(&rx, nullptr); }  // warmup into playing
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);
  {
    const std::uint64_t a0 = g_alloc.load();
    g_count.store(true, std::memory_order_relaxed);
    for (int i = 0; i < 2000; ++i) {
      RxSample rx = clean_rx(tc.q_cmd());
      tc.tick(&rx, nullptr);
    }
    g_count.store(false, std::memory_order_relaxed);
    ASSERT_EQ(tc.mode(), Mode::TRAJECTORY) << "still playing (window < trajectory duration)";
    EXPECT_EQ(g_alloc.load() - a0, 0u) << "TRAJECTORY tick path allocated";
  }
}

// The event ring is fixed-capacity and drops (does not allocate/grow) when full.
TEST(TickCore, EventRingBounded) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  tc.begin_streaming(q0);
  // Flap the e-stop many times to emit many events without draining.
  for (int i = 0; i < 5000; ++i) {
    RxSample rx = clean_rx(q0);
    rx.gates.e_stopped = (i % 2 == 0);
    tc.tick(&rx, nullptr);
  }
  // Draining never hangs / overflows; count is bounded by the ring capacity.
  int n = 0;
  Event e{};
  while (tc.pop_event(e)) ++n;
  EXPECT_LE(n, 256);
}
