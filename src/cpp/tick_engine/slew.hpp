// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — per-tick position slew clip (P3a). PLAN.md §5.2; dries B15.
//
// Last-line defense against swap/merge discontinuities the CRX DCS reads as
// disturbance torque: clip |Δq| ≤ SLEW_FACTOR·v_lim·ITP_S per joint, IN PLACE,
// count the clips, and raise a diagnostic "sustained-clip" bit. This NEVER
// FAULTS — it is a clamp, not a gate (P3b decides what to do with the counters).
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ ACCEL-CAP IS BANNED. This limiter clips ONLY the per-tick POSITION delta. │
// │ It holds NO velocity/acceleration state and NEVER caps Δ(derived          │
// │ velocity)/dt. The reverted accel-cap anti-pattern (Δv/dt cap on derived   │
// │ velocity) caused a runaway 22° drift into the DCS envelope (2026-05-16,   │
// │ dries B15). Do NOT add velocity/accel members here. `test_slew` asserts   │
// │ the clip is a pure function of (prev_q, q_desired) — history-independent. │
// └─────────────────────────────────────────────────────────────────────────┘

#pragma once

#include <cstdint>

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::tick_engine {

struct SlewResult {
  Vec6 q{};             // clipped command
  int clipped_joints{0};  // how many joints were clipped this tick
};

class SlewLimiter {
 public:
  explicit SlewLimiter(const TickEngineConfig& cfg);

  // Set the reference position (e.g. at PREROLL / mode entry) WITHOUT clipping.
  // The next apply() clips relative to this.
  void reset(const Vec6& q);

  // Clip q_desired so no joint moves more than max_step[j] = slew_factor·v_lim·itp_s
  // from the previous commanded position. Updates the reference to the clipped
  // value, counts clips, and updates the sustained-clip run. NEVER faults.
  SlewResult apply(const Vec6& q_desired);

  // Per-joint max step (radians): slew_factor·v_lim·itp_s.
  double max_step(int joint) const;

  // --- diagnostics ---
  std::uint64_t total_clips() const { return total_clips_; }
  int consecutive_clip_ticks() const { return consecutive_clip_ticks_; }
  bool sustained_clip() const { return consecutive_clip_ticks_ >= cfg_.slew_sustained_ticks; }
  bool has_reference() const { return has_prev_; }

 private:
  TickEngineConfig cfg_;
  Vec6 prev_q_{};
  bool has_prev_{false};
  std::uint64_t total_clips_{0};
  int consecutive_clip_ticks_{0};
};

}  // namespace airo_fanuc::tick_engine
