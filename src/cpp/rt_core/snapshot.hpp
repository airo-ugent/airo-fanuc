// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — seqlock state snapshot (P3b). PLAN.md §5.2 / design doc 07 §4.
//
// The RT thread publishes a POD `StateSnapshot` every tick via a seqlock; any
// number of Python-side readers get a torn-free read WITHOUT blocking the RT
// writer (poll-only — NO RT-thread callbacks into Python, ever). Staleness is
// annotated (rx_mono_ns / ages), never suppressed: a getter reports the age so a
// stale value is never presented as fresh (T1 keeps flowing with honest ages).

#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "rt_core/rt_core_config.hpp"

namespace airo_fanuc::rt_core {

// POD published by the RT thread. Radians internally (matches TickEngineConfig);
// RealtimeCore also carries the raw wire scalars a Python getter needs.
struct StateSnapshot {
  // Measured (from the status packet) and commanded (from the tick).
  Vec6 q_meas{};    // radians
  Vec6 qd_est{};    // radians/s (5-sample finite-difference slope, computed at ingest)
  Vec6 q_cmd{};     // radians (last commanded position)
  Vec6 qd_cmd{};    // radians/s (last commanded velocity)
  std::array<double, 9> cart{};  // Cartesian XYZWPR + ext (deg / mm), raw from the status packet

  // Sequence / timing.
  std::uint32_t rx_seq{0};
  std::uint32_t tx_seq{0};
  std::uint32_t ctrl_time_stamp_ms{0};
  std::int64_t rx_mono_ns{0};    // CLOCK_MONOTONIC at last RX ingest
  std::int64_t tick_mono_ns{0};  // CLOCK_MONOTONIC at snapshot publish

  // Status bits (type-202/204).
  bool e_stopped{false};
  bool in_error{false};
  bool tp_enabled{false};
  bool motion_possible{false};
  bool motion_in_progress{false};
  std::uint8_t contact_stop_status{0};
  double safety_scale{0.0};

  // Force / torque (tool frame; fs_type-gated).
  double fx{0.0}, fy{0.0}, fz{0.0}, mx{0.0}, my{0.0}, mz{0.0};
  std::uint32_t fs_type{0};
  bool wrench_valid{false};

  // Core state.
  Mode mode{Mode::STREAM_DOWN};
  FaultReason fault{FaultReason::NONE};
  std::uint32_t conditions{0};  // Condition bitmask
  std::uint64_t active_motion_id{0};
  MotionStatus active_motion_status{MotionStatus::PENDING};
  std::uint64_t epoch{0};

  // Diagnostics.
  std::uint64_t total_slew_clips{0};
  bool rx_fresh{false};  // was there a fresh RX at this publish?
};

// A single-writer / many-reader seqlock over a POD. The writer is the RT thread;
// readers are Python getters. `read` never blocks the writer and returns a
// torn-free copy (retries while the writer is mid-update). Never raises.
template <typename T>
class Seqlock {
 public:
  Seqlock() { seq_.store(0, std::memory_order_relaxed); }

  // RT-thread writer. Alloc-free (T is a POD; copy is a memcpy).
  void write(const T& v) {
    const std::uint64_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);  // → odd: writing
    std::atomic_thread_fence(std::memory_order_release);
    value_ = v;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_relaxed);  // → even: stable
  }

  // Reader (any thread). Retries until it observes a stable even sequence with a
  // consistent copy. Bounded-spin in practice (writer holds the lock for a
  // single memcpy); never blocks the writer.
  T read() const {
    T out;
    for (;;) {
      const std::uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1u) {
        continue;  // writer mid-update
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      out = value_;
      std::atomic_thread_fence(std::memory_order_acquire);
      const std::uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2) {
        return out;
      }
    }
  }

 private:
  std::atomic<std::uint64_t> seq_;
  T value_{};
};

}  // namespace airo_fanuc::rt_core
