// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — PLL-clocked TX phase servo (P3b). PLAN.md decision 4 / R1 A1+A2.
//
// Ported verbatim from the proven P2' spike (spike/rt_loop.cpp lines ~451-467):
// a timerfd sets the 8 ms cadence; its phase is servoed toward RX arrival with a
// per-tick correction HARD-CLAMPED to ±100 µs. This kills the double-send /
// velocity-step class by construction (raw reply-on-receive can fire two commands
// <1 ms apart, each advancing τ 8 ms → one ITP at 2× velocity → CRX contact-stop).
//
// Pure and clock-free (takes the just-fired scheduled tick + RX arrival, returns
// the next absolute tick time). RealtimeCore owns the timerfd and the wall clock;
// this class owns only the phase-servo math, so it is unit-testable stand-alone.

#pragma once

#include <cstdint>

namespace airo_fanuc::rt_core {

class Pll {
 public:
  // rx_lead_ns : target lead of the tick after RX arrival (fire tick this long
  //              after the RX that precedes it).
  // kp         : proportional phase-servo gain (spike default 0.20).
  // tick_ns    : nominal cadence (8 ms).
  // cap_ns     : per-tick correction clamp (spike invariant: 100 µs).
  Pll(std::int64_t rx_lead_ns, double kp, std::int64_t tick_ns, std::int64_t cap_ns = 100'000);

  // Compute the next absolute tick time from the just-fired scheduled tick.
  //   have_fresh_rx : did a fresh RX arrive since the previous tick?
  //   last_rx_mono_ns : monotonic arrival time of that RX (ignored if !fresh).
  // When no fresh RX arrived, free-runs at nominal cadence (no phase correction).
  // The result is guaranteed strictly greater than `scheduled_tick_ns`.
  std::int64_t next_tick(std::int64_t scheduled_tick_ns, bool have_fresh_rx, std::int64_t last_rx_mono_ns);

  // Last applied phase correction (ns), signed, clamped to ±cap. Diagnostic.
  std::int64_t last_correction_ns() const { return last_correction_ns_; }

  std::int64_t tick_ns() const { return tick_ns_; }
  std::int64_t cap_ns() const { return cap_ns_; }

 private:
  std::int64_t rx_lead_ns_;
  double kp_;
  std::int64_t tick_ns_;
  std::int64_t cap_ns_;
  std::int64_t last_correction_ns_{0};
};

}  // namespace airo_fanuc::rt_core
