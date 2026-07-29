// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — CAPTURE-or-REJECT splice generator (P3a). PLAN.md decision 6 /
// R3 A2.
//
// When a new trajectory arrives, the bridge must splice the COMMANDED state
// (q_cmd, qd_cmd) to the new trajectory start (q0, qd0). This generator emits a
// DETERMINISTIC Ruckig<6> position-mode profile at ITP resolution:
//   target        = (q0, qd0)
//   v_max         = capture_rate_rad_s (15°/s) per joint
//   a / j         = brake-class (STOP_LIMIT_SCALE_VA·a_lim / STOP_LIMIT_SCALE_J·j_lim)
//
// "THE CHECKED PATH IS THE EXECUTED PATH" (PLAN.md §5.1): this is ONE function.
// The Python pre-flight collision check (via the P4 binding) and the C++ RT
// execution BOTH call `generate_capture_path`; determinism is structural (one
// code path, fixed inputs, fixed delta_time → identical Ruckig output). The L1
// golden `test_capture` asserts (a) identical output on repeated calls and
// (b) that replaying the generated knots reproduces them exactly. No separate
// "synthesis vs execution" implementation exists to drift.
//
// The generator allocates NOTHING on the heap: `CapturePath` owns a fixed-
// capacity std::array buffer and a local Ruckig<6> (StandardVector = std::array)
// is stack-allocated. It is called ONCE at CAPTURE entry (a mode transition),
// not per steady tick.
//
// The acceptance/REJECT decision (|q_cmd − q0|∞ > capture_tol_rad) is a policy
// gate applied by the caller BEFORE generating (typed error in Python). This
// module exposes the tolerance check as a free helper for reuse.

#pragma once

#include <array>
#include <cstdint>

#include <ruckig/ruckig.hpp>

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::tick_engine {

// Deterministic capture profile. Knots are ITP-spaced (index k = tick k·itp_s).
// knot[0] == (q_cmd, qd_cmd); the last knot reaches (q0, qd0).
struct CapturePath {
  // 2 s @ 125 Hz is the brake-class hard ceiling; capture (5° @ 15°/s + jerk
  // ramp) is far shorter, but size generously and fault on overflow.
  static constexpr int kMaxKnots = 300;
  std::array<Vec6, kMaxKnots> q{};
  std::array<Vec6, kMaxKnots> qd{};
  // Analytic wire acceleration at each knot (Ruckig new_acceleration; knot[0] is
  // the seeded current_acceleration = 0). R1 C1: braking OUT of CAPTURE seeds the
  // Ruckig brake with this so the commanded accel is continuous across the
  // capture→brake handoff (an accel STEP latches the CRX contact-stop).
  std::array<Vec6, kMaxKnots> qdd{};
  int count{0};
  ruckig::Result result{ruckig::Result::Working};
  bool overflow{false};  // profile exceeded kMaxKnots (should never happen at 15°/s over 5°)
  bool finished{false};  // Ruckig reached (q0, qd0)

  void clear() {
    count = 0;
    result = ruckig::Result::Working;
    overflow = false;
    finished = false;
  }
  bool ok() const { return finished && !overflow; }
};

// The SINGLE capture-path generator (see header note). Fills `out`. Pure and
// deterministic: same inputs → byte-identical `out`.
void generate_capture_path(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& q0, const Vec6& qd0,
                           const TickEngineConfig& cfg, CapturePath& out);

// Policy helper: would this splice be REJECTED? True iff |q_cmd − q0|∞ exceeds
// the capture tolerance (5°). Callers turn `true` into a typed reject.
bool capture_would_reject(const Vec6& q_cmd, const Vec6& q0, const TickEngineConfig& cfg);

}  // namespace airo_fanuc::tick_engine
