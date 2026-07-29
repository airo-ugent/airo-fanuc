// SPDX-License-Identifier: Apache-2.0
//
// Per-tick position slew clip — see slew.hpp. POSITION-ONLY; accel-cap banned.

#include "tick_engine/slew.hpp"

#include <algorithm>

namespace airo_fanuc::tick_engine {

SlewLimiter::SlewLimiter(const TickEngineConfig& cfg) : cfg_(cfg) {}

void SlewLimiter::reset(const Vec6& q) {
  prev_q_ = q;
  has_prev_ = true;
  consecutive_clip_ticks_ = 0;
}

double SlewLimiter::max_step(int joint) const {
  return cfg_.slew_factor * cfg_.limits.v[static_cast<std::size_t>(joint)] * cfg_.itp_s;
}

SlewResult SlewLimiter::apply(const Vec6& q_desired) {
  SlewResult r{};
  if (!has_prev_) {
    // No reference yet → pass through and adopt it. First command is never clipped.
    r.q = q_desired;
    prev_q_ = q_desired;
    has_prev_ = true;
    return r;
  }

  int clipped = 0;
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    const double step = cfg_.slew_factor * cfg_.limits.v[j] * cfg_.itp_s;  // ≥ 0
    const double delta = q_desired[j] - prev_q_[j];
    if (delta > step) {
      r.q[j] = prev_q_[j] + step;
      ++clipped;
    } else if (delta < -step) {
      r.q[j] = prev_q_[j] - step;
      ++clipped;
    } else {
      r.q[j] = q_desired[j];
    }
  }

  r.clipped_joints = clipped;
  total_clips_ += static_cast<std::uint64_t>(clipped);
  if (clipped > 0) {
    ++consecutive_clip_ticks_;
  } else {
    consecutive_clip_ticks_ = 0;
  }

  prev_q_ = r.q;  // reference advances to the CLIPPED command (never the raw desired)
  return r;
}

}  // namespace airo_fanuc::tick_engine
