// SPDX-License-Identifier: Apache-2.0
//
// Unit test — big-anchor int64 timestamp invariant: absolute timestamps are
// int64-only, doubles are used for differences only. The path that actually
// carries ABSOLUTE monotonic timestamps is JointsAtRing (the FK-at-shutter ring
// that matches a camera frame to the joint pose at its exposure): it stores
// mono_ns as int64 and finds the
// nearest sample by int64 subtraction. This test proves that lookup stays EXACT
// when the anchor exceeds 2^53 ns (a double can no longer represent every integer
// there, so a "simplify mono_ns to double" regression would collapse neighbours
// / mis-rank the nearest — this is the wall against that).

#include <cstdint>

#include <gtest/gtest.h>

#include "rt_core/rings.hpp"

using airo_fanuc::rt_core::JointsAtRing;
using airo_fanuc::rt_core::Vec6;

namespace {
Vec6 tag(double v) { return Vec6{v, 0, 0, 0, 0, 0}; }
}  // namespace

// 2^53 = 9'007'199'254'740'992 ns ≈ 104 days of CLOCK_MONOTONIC uptime — a real,
// reachable anchor magnitude. Entries 8 ms apart above it are retrieved exactly.
TEST(BigAnchor, NearestExactAbove2Pow53) {
  const std::int64_t base = (std::int64_t{1} << 54) + 123'456'789;  // ~1.8e16, > 2^53
  JointsAtRing ring;
  for (int k = 0; k < 10; ++k) {
    ring.push(base + static_cast<std::int64_t>(k) * 8'000'000, tag(static_cast<double>(k)));
  }

  Vec6 out{};
  // Exact hit on entry 3.
  ASSERT_TRUE(ring.nearest(base + 3 * 8'000'000, out));
  EXPECT_EQ(out[0], 3.0);
  // A 100 ns nudge (≪ half the 8 ms spacing) still resolves to entry 3.
  ASSERT_TRUE(ring.nearest(base + 3 * 8'000'000 + 100, out));
  EXPECT_EQ(out[0], 3.0);
  // Just past the midpoint toward entry 4.
  ASSERT_TRUE(ring.nearest(base + 3 * 8'000'000 + 4'000'001, out));
  EXPECT_EQ(out[0], 4.0);
  // Anchor endpoints.
  ASSERT_TRUE(ring.nearest(base, out));
  EXPECT_EQ(out[0], 0.0);
  ASSERT_TRUE(ring.nearest(base + 9 * 8'000'000, out));
  EXPECT_EQ(out[0], 9.0);
}

// Sub-ULP separation: two entries 1 ns apart at ~2^55 (where a double's ULP is
// 8 ns) are still distinguished. Under a double representation both stamps would
// round to the SAME value and the nearest lookup would be ambiguous / wrong;
// int64 resolves the 1 ns difference exactly.
TEST(BigAnchor, ResolvesOneNsSeparationAboveDoubleUlp) {
  const std::int64_t big = std::int64_t{1} << 55;  // ~3.6e16; double ULP here = 8 ns
  JointsAtRing ring;
  ring.push(big, tag(1.0));
  ring.push(big + 1, tag(2.0));

  Vec6 out{};
  ASSERT_TRUE(ring.nearest(big + 1, out));
  EXPECT_EQ(out[0], 2.0) << "1 ns apart above 2^54 must stay distinguishable (int64, not double)";
  ASSERT_TRUE(ring.nearest(big, out));
  EXPECT_EQ(out[0], 1.0);
}

// Wraparound past the ring capacity keeps big-anchor lookups correct (the newest
// kCap entries are searched; older ones are overwritten).
TEST(BigAnchor, WraparoundKeepsBigAnchorLookupCorrect) {
  const std::int64_t base = (std::int64_t{1} << 54);
  JointsAtRing ring;
  const int total = static_cast<int>(JointsAtRing::kCap) + 50;
  for (int k = 0; k < total; ++k) {
    ring.push(base + static_cast<std::int64_t>(k) * 8'000'000, tag(static_cast<double>(k)));
  }
  Vec6 out{};
  // The most recent entry is exact.
  ASSERT_TRUE(ring.nearest(base + static_cast<std::int64_t>(total - 1) * 8'000'000, out));
  EXPECT_EQ(out[0], static_cast<double>(total - 1));
}
