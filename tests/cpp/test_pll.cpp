// SPDX-License-Identifier: Apache-2.0
//
// Unit test — PLL-clocked TX phase servo. The headline
// invariants: correction is clamped to ±100 µs/tick; free-run at nominal cadence
// without a fresh RX; the next tick is always strictly ahead of the fired one;
// the phase converges to (rx + lead).

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
  EXPECT_EQ(pll.next_tick(s, /*have_fresh_rx=*/false, 0), s + kTick);
  EXPECT_EQ(pll.last_correction_ns(), 0);
}

TEST(Pll, CorrectionClampedToCap) {
  Pll pll(kLead, 0.20, kTick, kCap);
  // Huge positive phase error (tick far ahead of rx+lead) → correction saturates
  // at +cap; huge negative error → −cap.
  const std::int64_t rx = 1'000'000'000;
  const std::int64_t s_ahead = rx + kLead + 50'000'000;  // 50 ms ahead
  const std::int64_t next = pll.next_tick(s_ahead, true, rx);
  EXPECT_EQ(pll.last_correction_ns(), kCap);
  EXPECT_EQ(next, s_ahead + kTick - kCap);

  const std::int64_t s_behind = rx + kLead - 50'000'000;
  pll.next_tick(s_behind, true, rx);
  EXPECT_EQ(pll.last_correction_ns(), -kCap);
}

TEST(Pll, NextTickAlwaysAhead) {
  Pll pll(kLead, 5.0 /*aggressive gain*/, kTick, kCap);
  const std::int64_t rx = 500'000'000;
  // Even with a correction pulling the tick back, next > scheduled (8 ms − 100 µs).
  for (std::int64_t off = -20'000'000; off <= 20'000'000; off += 1'000'000) {
    const std::int64_t s = rx + kLead + off;
    EXPECT_GT(pll.next_tick(s, true, rx), s);
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
    sched = pll.next_tick(sched, true, rx);
    rx += kTick;  // controller advances one ITP
  }
  // Steady state: the per-period phase error → 0 (locked to the target lead).
  EXPECT_LT(std::llabs(err), 5'000) << "phase locked to within 5 µs of the target lead";
}
