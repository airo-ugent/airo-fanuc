// SPDX-License-Identifier: Apache-2.0
//
// Unit test — PLL-clocked TX phase servo. The headline
// invariants: correction is clamped to ±100 µs/tick; free-run at nominal cadence
// without a fresh RX; the next tick is always strictly ahead of the fired one AND
// ahead of the caller's clock, so a window lost to preemption is dropped rather
// than replayed; the phase converges to (rx + lead).

#include <cstdint>

#include <gtest/gtest.h>

#include "rt_core/pll.hpp"

using airo_fanuc::rt_core::Pll;

namespace {
constexpr std::int64_t kTick = 8'000'000;   // 8 ms
constexpr std::int64_t kCap = 100'000;      // 100 µs
constexpr std::int64_t kLead = 300'000;     // 300 µs
}  // namespace

TEST(Pll, FreeRunsWithoutRx) {
  Pll pll(kLead, 0.20, kTick, kCap);
  const std::int64_t s = 1'000'000'000;
  EXPECT_EQ(pll.next_tick(s, /*have_fresh_rx=*/false, 0, /*now_mono_ns=*/s), s + kTick);
  EXPECT_EQ(pll.last_correction_ns(), 0);
  EXPECT_EQ(pll.last_skipped_windows(), 0);
}

TEST(Pll, CorrectionClampedToCap) {
  Pll pll(kLead, 0.20, kTick, kCap);
  // Huge positive phase error (tick far ahead of rx+lead) → correction saturates
  // at +cap; huge negative error → −cap.
  const std::int64_t rx = 1'000'000'000;
  const std::int64_t s_ahead = rx + kLead + 50'000'000;  // 50 ms ahead
  const std::int64_t next = pll.next_tick(s_ahead, true, rx, /*now_mono_ns=*/s_ahead);
  EXPECT_EQ(pll.last_correction_ns(), kCap);
  EXPECT_EQ(next, s_ahead + kTick - kCap);

  const std::int64_t s_behind = rx + kLead - 50'000'000;
  pll.next_tick(s_behind, true, rx, /*now_mono_ns=*/s_behind);
  EXPECT_EQ(pll.last_correction_ns(), -kCap);
}

TEST(Pll, NextTickAlwaysAhead) {
  Pll pll(kLead, 5.0 /*aggressive gain*/, kTick, kCap);
  const std::int64_t rx = 500'000'000;
  // Even with a correction pulling the tick back, next > scheduled (8 ms − 100 µs).
  for (std::int64_t off = -20'000'000; off <= 20'000'000; off += 1'000'000) {
    const std::int64_t s = rx + kLead + off;
    EXPECT_GT(pll.next_tick(s, true, rx, /*now_mono_ns=*/s), s);
  }
}

TEST(Pll, RebasesADeadlineTheClockHasAlreadyPassed) {
  Pll pll(kLead, 0.20, kTick, kCap);
  const std::int64_t rx = 1'000'000'000;
  const std::int64_t s = rx + kLead;  // in phase → correction 0
  // The loop lost 60 ms (7.5 windows) to preemption, so by the time it computes the
  // next deadline the clock is far past it.
  const std::int64_t now = s + 60'000'000;
  const std::int64_t next = pll.next_tick(s, true, rx, now);
  EXPECT_EQ(next, now + kTick) << "re-based onto the clock";
  EXPECT_GT(next, now) << "an absolute timer armed here cannot fire immediately";
  // Windows dropped: the computed deadline s+kTick plus every window between it
  // and `now`.
  EXPECT_EQ(pll.last_skipped_windows(), (now - (s + kTick)) / kTick + 1);
  // The next window is a full cadence away, so the send it produces cannot land in
  // the same 8 ms as this one.
  EXPECT_GE(next - now, kTick);
}

TEST(Pll, OnTimeTickIsNeverRebased) {
  Pll pll(kLead, 0.20, kTick, kCap);
  std::int64_t rx = 1'000'000'000;
  std::int64_t sched = rx + kLead + 5'000'000;  // 5 ms off phase → correction saturates
  for (int i = 0; i < 200; ++i) {
    // `now` is the fired deadline plus wake latency. 400 µs is 4x the correction
    // cap, so neither wake latency at that scale nor a saturated correction can on
    // its own look like a lost window.
    const std::int64_t now = sched + 400'000;
    const std::int64_t next = pll.next_tick(sched, true, rx, now);
    EXPECT_EQ(pll.last_skipped_windows(), 0) << "no window dropped on an on-time tick";
    EXPECT_GT(next, now);
    sched = next;
    rx += kTick;  // controller advances one ITP
  }
}

TEST(Pll, ConvergesToLead) {
  Pll pll(kLead, 0.20, kTick, kCap);
  // Controller RX ticks at exactly 8 ms; the scheduled tick starts 3 ms off phase.
  std::int64_t rx = 1'000'000'000;
  std::int64_t sched = rx + 3'000'000;  // 3 ms late
  std::int64_t err = 0;
  for (int i = 0; i < 2000; ++i) {
    err = sched - (rx + kLead);  // phase error at the start of this period
    sched = pll.next_tick(sched, true, rx, /*now_mono_ns=*/sched);
    rx += kTick;  // controller advances one ITP
  }
  // Steady state: the per-period phase error → 0 (locked to the target lead).
  EXPECT_LT(std::llabs(err), 5'000) << "phase locked to within 5 µs of the target lead";
}
