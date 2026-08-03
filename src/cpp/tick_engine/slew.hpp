// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — per-tick position slew clip.
//
// Last-line defense against swap/merge discontinuities the CRX DCS reads as
// disturbance torque: clip |Δq| ≤ SLEW_FACTOR·v_lim·ITP_S per joint, IN PLACE,
// count the clips, and raise a diagnostic "sustained-clip" bit. This NEVER
// FAULTS — it is a clamp, not a gate (the tick core decides what to do with the
// counters).
//
// ┌───────────────────────────────────────────────────────────────────────────┐
// │ ACCEL-CAP IS BANNED — this is a safety constraint, not a style choice.    │
// │                                                                           │
// │ This limiter clips ONLY the per-tick POSITION delta. It holds NO velocity │
// │ or acceleration state and NEVER caps Δ(derived velocity)/dt.              │
// │                                                                           │
// │ Why: an accel cap has to differentiate the command and carry velocity     │
// │ state, so the emitted position becomes a function of the limiter's own    │
// │ integrated velocity instead of the desired position. Once it starts       │
// │ clipping, the output no longer tracks the input — it keeps generating     │
// │ motion of its own and the error accumulates every tick. That is a         │
// │ runaway, and it happened: a Δv/dt cap on derived velocity drove a 22°     │
// │ drift off the commanded path and into the DCS envelope.                   │
// │                                                                           │
// │ A stateless position clip cannot do this: the output is at most one       │
// │ max-step away from the last command, so the error is bounded by           │
// │ construction and vanishes as soon as the input comes back in range.       │
// │ Do NOT add velocity/accel members here. `test_slew` asserts the clip is a │
// │ pure function of (prev_q, q_desired) — history-independent.               │
// └───────────────────────────────────────────────────────────────────────────┘

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

 private:
  TickEngineConfig cfg_;
  Vec6 prev_q_{};
  bool has_prev_{false};
  std::uint64_t total_clips_{0};
  int consecutive_clip_ticks_{0};
};

}  // namespace airo_fanuc::tick_engine
