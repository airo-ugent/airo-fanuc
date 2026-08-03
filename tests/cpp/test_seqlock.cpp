// SPDX-License-Identifier: Apache-2.0
//
// Unit test — the Seqlock that carries every published snapshot from the RT thread to
// the Python side. Every getter in the driver reads core state through it, and its
// failure mode is silent: a torn read returns a struct whose fields come from two
// different ticks — a well-formed snapshot that never existed. Nothing downstream can
// detect that, because each field is individually in range.
//
// So the assertion is on the PROTOCOL, not the payload. The writer's odd/even sequence
// discipline is the only thing that makes a torn copy observable to the reader, and it
// is invisible to a single-threaded round-trip: dropping both `seq_` stores leaves
// write()/read() behaving identically until a reader and the writer actually overlap.
// This test overlaps them.

#include <atomic>
#include <cstdint>
#include <thread>

#include <gtest/gtest.h>

#include "rt_core/snapshot.hpp"

using airo_fanuc::rt_core::Seqlock;

namespace {

// Wide enough that copying it is many machine words, so a writer interrupted mid-copy
// leaves the halves disagreeing. Every word carries the same generation number, which
// makes "torn" checkable without knowing which generation a reader was aiming at.
struct Generation {
  static constexpr int kWords = 64;
  std::uint64_t word[kWords];
};

bool torn(const Generation& g) {
  for (int i = 1; i < Generation::kWords; ++i) {
    if (g.word[i] != g.word[0]) return true;
  }
  return false;
}

}  // namespace

TEST(SeqlockTest, RoundTripsAValueWithNoWriterContention) {
  Seqlock<Generation> sl;
  Generation g{};
  for (std::uint64_t& w : g.word) w = 7;
  sl.write(g);
  const Generation out = sl.read();
  EXPECT_FALSE(torn(out));
  EXPECT_EQ(out.word[0], 7u);
}

TEST(SeqlockTest, AReaderNeverObservesAHalfWrittenPayload) {
  Seqlock<Generation> sl;
  sl.write(Generation{});

  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> written{0};
  std::thread writer([&] {
    Generation g{};
    for (std::uint64_t n = 1; !stop.load(std::memory_order_relaxed); ++n) {
      for (std::uint64_t& w : g.word) w = n;
      sl.write(g);
      std::this_thread::yield();
      written.store(n, std::memory_order_relaxed);
    }
  });

  // Overlap is the whole point, so do not start measuring until the writer is live:
  // a reader that raced ahead of thread startup would only ever see the pre-loop write.
  while (written.load(std::memory_order_relaxed) == 0) {
  }

  std::uint64_t tears = 0;
  std::uint64_t first = 0;
  std::uint64_t last = 0;
  for (int i = 0; i < 20'000; ++i) {
    const Generation out = sl.read();
    if (torn(out)) ++tears;
    if (i == 0) first = out.word[0];
    last = out.word[0];
  }
  stop.store(true, std::memory_order_relaxed);
  writer.join();

  EXPECT_EQ(tears, 0u) << "a read assembled a payload out of two generations";
  // Without this the tear check passes trivially on a Seqlock that never returned
  // anything but its initial value — a reader that observes nothing cannot observe a tear.
  EXPECT_GT(last, first) << "the reader never observed the writer advance";
}
