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

std::int64_t Pll::next_tick(std::int64_t scheduled_tick_ns, bool have_fresh_rx, std::int64_t last_rx_mono_ns,
                            std::int64_t now_mono_ns) {
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
  // Re-base onto the clock when the loop is already a whole window late. Both branches
  // above are relative to `scheduled_tick_ns`, so they carry the lateness forward
  // unchanged: a deadline behind `now_mono_ns` means the loop lost at least one full
  // tick_ns, and arming an absolute timer there re-enters the loop immediately.
  // Re-basing keeps consecutive sends tick_ns apart — the passed-over windows are
  // dropped, so the motion clock advances once per window actually served rather than
  // once per iteration of a catch-up burst. This cannot fire on an on-time tick: `next`
  // is a full tick_ns ± cap_ns ahead of the deadline that just fired, so `now_mono_ns`
  // reaching it requires that much wake latency. Re-locking the phase afterwards costs
  // at most cap_ns per tick, the same slew a reconnect takes.
  last_skipped_windows_ = 0;
  if (next <= now_mono_ns) {
    last_skipped_windows_ = static_cast<int>((now_mono_ns - next) / tick_ns_) + 1;
    next = now_mono_ns + tick_ns_;
  }
  return next;
}

}  // namespace airo_fanuc::rt_core
