// SPDX-License-Identifier: Apache-2.0
//
// PLL-clocked TX phase servo — see pll.hpp.

#include "rt_core/pll.hpp"

namespace airo_fanuc::rt_core {

namespace {
inline std::int64_t clamp_i64(std::int64_t v, std::int64_t lo, std::int64_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

Pll::Pll(std::int64_t rx_lead_ns, double kp, std::int64_t tick_ns, std::int64_t cap_ns)
    : rx_lead_ns_(rx_lead_ns), kp_(kp), tick_ns_(tick_ns), cap_ns_(cap_ns) {}

std::int64_t Pll::next_tick(std::int64_t scheduled_tick_ns, bool have_fresh_rx, std::int64_t last_rx_mono_ns) {
  std::int64_t next;
  if (have_fresh_rx) {
    // Phase error: how far the scheduled tick is AHEAD of (rx + lead). Pulling
    // the tick toward that target keeps TX phase-locked to the controller ITP.
    // The clamp is what bounds the resulting tick spacing to tick_ns ± cap_ns, so
    // two sends can never land in one window no matter what `err` comes out as.
    const std::int64_t err = scheduled_tick_ns - (last_rx_mono_ns + rx_lead_ns_);
    const std::int64_t corr = clamp_i64(static_cast<std::int64_t>(kp_ * static_cast<double>(err)), -cap_ns_, cap_ns_);
    last_correction_ns_ = corr;
    next = scheduled_tick_ns + tick_ns_ - corr;
  } else {
    last_correction_ns_ = 0;
    next = scheduled_tick_ns + tick_ns_;
  }
  // Never schedule at/behind the just-fired deadline.
  if (next <= scheduled_tick_ns) {
    next = scheduled_tick_ns + 1;
  }
  return next;
}

}  // namespace airo_fanuc::rt_core
