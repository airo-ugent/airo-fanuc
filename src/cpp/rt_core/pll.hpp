// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — PLL-clocked TX phase servo.
//
// A timerfd sets the 8 ms TX cadence; this class servos that timer's PHASE so
// every command is emitted a fixed lead after the status packet it answers, while
// the cadence ITSELF is never allowed to wander far. That separation is what makes
// the double-send / velocity-step class of failure structurally impossible.
// Replying directly to each received status does not: two statuses can land <1 ms
// apart (a late datagram followed by an on-time one), so two commands go out inside
// a single 8 ms window, and because every command advances the motion clock by one
// full window, the controller then executes one ITP at 2× the intended joint
// velocity — a velocity step large enough to read as a collision and trip the CRX
// contact stop. A phase servo with a clamped correction cannot produce that:
// consecutive sends are separated by at least tick_ns − cap_ns by construction.
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
  // kp         : proportional phase-servo gain; the tuned value is 0.20. It takes
  //              out a fifth of the observed phase error per tick, i.e. a
  //              first-order lag with a ~5-tick (40 ms) time constant. The gain is
  //              deliberately slack: at 0.20 the loop averages over per-packet
  //              network jitter instead of chasing a single late datagram — a stiff
  //              gain would feed that jitter straight back out as TX-cadence jitter
  //              — and it still re-locks a phase step within a few hundred ms.
  // tick_ns    : nominal cadence (8 ms).
  // cap_ns     : hard clamp on the per-tick phase correction, 100 µs. This is the
  //              load-bearing invariant of the loop rather than a tuning knob: it
  //              pins consecutive TX spacing inside 8 ms ± 100 µs (±1.25 %), so no
  //              phase error — however large, and even one computed from a garbage
  //              RX timestamp — can pull two sends into the same 8 ms window. The
  //              same 100 µs is far too small for the controller to see the stream
  //              as anything but steady, yet still buys a full 8 ms of phase slew
  //              in ~80 ticks (0.64 s), which is fast enough to re-lock after a
  //              reconnect. The clamp binds once |error| exceeds 500 µs
  //              (kp · 500 µs = cap); below that the servo is purely proportional.
  Pll(std::int64_t rx_lead_ns, double kp, std::int64_t tick_ns, std::int64_t cap_ns = 100'000);

  // Compute the next absolute tick time from the just-fired scheduled tick.
  //   have_fresh_rx : did a fresh RX arrive since the previous tick?
  //   last_rx_mono_ns : monotonic arrival time of that RX (ignored if !fresh).
  //   now_mono_ns : CLOCK_MONOTONIC as read by the caller. This class reads no clock
  //              of its own; it needs the caller's `now` because the two terms above
  //              are relative to the previous deadline alone, so a preemption longer
  //              than one tick yields a deadline that is already in the past. An
  //              absolute timer armed there fires immediately, and the loop then
  //              ticks — and sends — once per elapsed window with no spacing, which
  //              is the velocity step the correction cap exists to prevent. A deadline
  //              at or behind `now_mono_ns` is therefore re-based to now + tick_ns:
  //              the windows whose deadlines went by are dropped and counted (see
  //              last_skipped_windows), never replayed.
  // When no fresh RX arrived, free-runs at nominal cadence (no phase correction).
  // The result is guaranteed strictly greater than both `scheduled_tick_ns` and
  // `now_mono_ns`.
  std::int64_t next_tick(std::int64_t scheduled_tick_ns, bool have_fresh_rx, std::int64_t last_rx_mono_ns,
                         std::int64_t now_mono_ns);

  // Last applied phase correction (ns), signed, clamped to ±cap. Diagnostic.
  std::int64_t last_correction_ns() const { return last_correction_ns_; }

  // Whole tick windows the last next_tick() dropped by re-basing onto the clock; 0
  // whenever the computed deadline was still ahead of `now_mono_ns`. Diagnostic.
  int last_skipped_windows() const { return last_skipped_windows_; }

  std::int64_t tick_ns() const { return tick_ns_; }
  std::int64_t cap_ns() const { return cap_ns_; }

 private:
  std::int64_t rx_lead_ns_;
  double kp_;
  std::int64_t tick_ns_;
  std::int64_t cap_ns_;
  std::int64_t last_correction_ns_{0};
  int last_skipped_windows_{0};
};

}  // namespace airo_fanuc::rt_core
