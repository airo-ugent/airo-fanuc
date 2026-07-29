// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — lock-free rings for the RT core (P3b). PLAN.md §5.2.
//
// The RT tick path must never allocate, free, or take a blocking lock. These
// fixed-capacity, pre-allocated rings carry data across the RT ↔ Python boundary
// with only atomics:
//   * SpscRing<T, Cap>       — single-producer / single-consumer bounded ring
//       - mailbox   : Python producer → RT consumer (Target)
//       - events    : RT producer → Python consumer (Event)
//       - retire    : RT producer → Python consumer (buffer pointers to free)
//     Multiple Python producers are serialised by RealtimeCore's submit mutex
//     (OFF the RT path), so the RT side sees a single producer.
//   * JointsAtRing           — RT writer, many readers; per-slot seqlock so a
//     reader never observes a torn (mono, q) pair (camera FK-at-shutter, R3 C2).

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::rt_core {

using tick_engine::Vec6;

// ---------------------------------------------------------------------------
// SpscRing — bounded single-producer/single-consumer ring. `Cap` MUST be a power
// of two. push() returns false when full (caller decides: retire the dropped
// buffer / count the drop). pop() returns false when empty. Both are wait-free.
// ---------------------------------------------------------------------------
template <typename T, std::size_t Cap>
class SpscRing {
  static_assert((Cap & (Cap - 1)) == 0, "SpscRing capacity must be a power of two");

 public:
  bool push(const T& v) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t next = (tail + 1) & (Cap - 1);
    if (next == head_.load(std::memory_order_acquire)) {
      return false;  // full
    }
    buf_[tail] = v;
    tail_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T& out) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head == tail_.load(std::memory_order_acquire)) {
      return false;  // empty
    }
    out = buf_[head];
    head_.store((head + 1) & (Cap - 1), std::memory_order_release);
    return true;
  }

  bool empty() const { return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire); }

  std::size_t size() const {
    const std::size_t t = tail_.load(std::memory_order_acquire);
    const std::size_t h = head_.load(std::memory_order_acquire);
    return (t - h) & (Cap - 1);
  }

 private:
  std::array<T, Cap> buf_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};

// ---------------------------------------------------------------------------
// JointsAtRing — 256-deep ring of (mono_ns, q_rad) written by the RT thread each
// RX ingest. Readers look up the entry nearest a queried monotonic timestamp
// (camera↔FK sync). Each slot carries a per-slot seqlock so a reader never
// observes a mono paired with a different tick's q. `wall_ns` conversion is done
// by the caller (RealtimeCore holds an off-RT-sampled mono↔wall offset — R3 C2).
// ---------------------------------------------------------------------------
class JointsAtRing {
 public:
  static constexpr std::size_t kCap = 256;

  // RT-thread writer (per RX). Alloc-free.
  void push(std::int64_t mono_ns, const Vec6& q) {
    const std::size_t i = write_idx_ & (kCap - 1);
    const std::uint64_t s = slots_[i].seq.load(std::memory_order_relaxed);
    slots_[i].seq.store(s + 1, std::memory_order_relaxed);  // odd = writing
    std::atomic_thread_fence(std::memory_order_release);
    slots_[i].mono_ns = mono_ns;
    slots_[i].q = q;
    std::atomic_thread_fence(std::memory_order_release);
    slots_[i].seq.store(s + 2, std::memory_order_relaxed);  // even = stable
    ++write_idx_;
    count_.store(write_idx_ < kCap ? write_idx_ : kCap, std::memory_order_release);
  }

  // Reader (any thread). Finds the stored joint state whose mono_ns is nearest
  // `query_mono_ns`. Returns false if the ring is empty. Torn-free per slot.
  bool nearest(std::int64_t query_mono_ns, Vec6& out) const {
    const std::size_t n = count_.load(std::memory_order_acquire);
    if (n == 0) {
      return false;
    }
    std::int64_t best_dt = INT64_MAX;
    Vec6 best{};
    bool found = false;
    for (std::size_t k = 0; k < n; ++k) {
      std::int64_t mono;
      Vec6 q;
      if (!read_slot(k, mono, q)) {
        continue;  // caught mid-write; skip
      }
      const std::int64_t dt = mono > query_mono_ns ? (mono - query_mono_ns) : (query_mono_ns - mono);
      if (dt < best_dt) {
        best_dt = dt;
        best = q;
        found = true;
      }
    }
    if (found) {
      out = best;
    }
    return found;
  }

 private:
  struct Slot {
    std::atomic<std::uint64_t> seq{0};
    std::int64_t mono_ns{0};
    Vec6 q{};
  };

  bool read_slot(std::size_t k, std::int64_t& mono, Vec6& q) const {
    const Slot& sl = slots_[k & (kCap - 1)];
    const std::uint64_t s1 = sl.seq.load(std::memory_order_acquire);
    if (s1 & 1u) {
      return false;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    mono = sl.mono_ns;
    q = sl.q;
    std::atomic_thread_fence(std::memory_order_acquire);
    return sl.seq.load(std::memory_order_acquire) == s1;
  }

  std::array<Slot, kCap> slots_{};
  std::size_t write_idx_{0};  // RT-owned
  std::atomic<std::size_t> count_{0};
};

}  // namespace airo_fanuc::rt_core
