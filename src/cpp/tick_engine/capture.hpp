// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — CAPTURE-or-REJECT splice generator.
//
// When a new trajectory arrives, the driver must splice the COMMANDED state
// (q_cmd, qd_cmd) to the new trajectory start (q0, qd0). This generator emits a
// DETERMINISTIC Ruckig<6> position-mode profile at ITP resolution:
//   target        = (q0, qd0)
//   v_max         = capture_rate_rad_s (15°/s) per joint
//   a / j         = brake-class (STOP_LIMIT_SCALE_VA·a_lim / STOP_LIMIT_SCALE_J·j_lim)
//
// ═══ INVARIANT: THE CHECKED PATH *IS* THE EXECUTED PATH ═══════════════════
// The splice is real arm motion that no planner produced, so it must be
// collision-checked before it runs — and the path that was checked must be,
// provably, the path that executes. A path that is merely *similar* to the
// checked one is not good enough: the splice sweeps through free space nobody
// planned, and any divergence between check and execution is arm motion through
// geometry the checker never saw.
//
// The guarantee is structural rather than argued: there is exactly ONE splice
// generator. The Python pre-flight collision check (through the pybind binding)
// and the C++ RT execution BOTH call `generate_capture_path`, with the same
// inputs and the same fixed delta_time, so Ruckig emits identical output. No
// separate "synthesis" and "execution" implementations exist that could drift
// apart. The C++ unit test `test_capture` pins this: (a) repeated calls produce
// identical output, and (b) replaying the generated knots reproduces them
// exactly.
//
// Consequences to respect when editing: keep this function PURE (no clock, no
// RNG, no ambient state), keep the Ruckig instance local so no internal state is
// carried between calls, and never add a code path that only one of the two
// callers takes.
// ══════════════════════════════════════════════════════════════════════════
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
// knot[0] == (q_cmd, qd_cmd). The LAST knot is (q0, qd0) advanced by `residue_ns`
// (see there): the knot grid is whole ticks and the profile's duration is not, so the
// final sample lands past the target rather than on it.
struct CapturePath {
  // 2 s @ 125 Hz is the brake-class hard ceiling; capture (5° @ 15°/s + jerk
  // ramp) is far shorter, but size generously and fault on overflow.
  static constexpr int kMaxKnots = 300;
  std::array<Vec6, kMaxKnots> q{};
  std::array<Vec6, kMaxKnots> qd{};
  // Analytic wire acceleration at each knot (Ruckig new_acceleration; knot[0] is
  // the seeded current_acceleration = 0). Braking OUT of CAPTURE seeds the Ruckig
  // brake with this so the commanded accel is continuous across the capture→brake
  // handoff — an accel STEP reads as motor disturbance torque and latches the CRX
  // collaborative contact-stop.
  std::array<Vec6, kMaxKnots> qdd{};
  int count{0};
  ruckig::Result result{ruckig::Result::Working};
  bool overflow{false};  // profile exceeded kMaxKnots (should never happen at 15°/s over 5°)
  bool finished{false};  // Ruckig reached (q0, qd0)
  // How far the LAST knot lies past the target, in trajectory time: (0, itp_s].
  //
  // The profile's duration is not a whole number of ticks, and the knots are. Ruckig
  // reports Finished on the first update strictly past the duration and evaluates that
  // sample by extrapolating from the target state, so
  //     q[count-1] == q0 + qd0·residue      (exactly, to double precision)
  // — i.e. the last knot is the state the TRAJECTORY itself has at its own time
  // `residue`. Playback must therefore resume one tick after that, at
  // tau = itp_s + residue, or the handoff tick re-covers ground this knot already
  // covered and carries only (itp_s − residue) of a tick's travel. The velocity STEP
  // that under-step puts on the wire is up to the whole commanded velocity, which
  // dwarfs anything else at this seam, and its size depends only on where the duration
  // happens to fall between two ticks.
  std::int64_t residue_ns{0};

  void clear() {
    count = 0;
    result = ruckig::Result::Working;
    overflow = false;
    finished = false;
    residue_ns = 0;
  }
  bool ok() const { return finished && !overflow; }
};

// The SINGLE capture-path generator (see header note). Fills `out`. Pure and
// deterministic: same inputs → byte-identical `out`.
void generate_capture_path(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& q0, const Vec6& qd0,
                           const TickEngineConfig& cfg, CapturePath& out);

// ---------------------------------------------------------------------------
// THE CAPTURE ACCEPTANCE GATE
//
// Three things have to be true for a splice to be a bounded bridge rather than a motion of
// its own, and the endpoint window is only the first:
//
//   (1) |q_cmd − q0|∞ ≤ capture_tol_rad — the endpoint gap.
//   (2) The velocity change must be SHEDDABLE INSIDE THAT WINDOW. The splice runs at the
//       brake-class a/j (see generate_capture_path), so changing a joint's velocity by
//       |qd0 − qd_cmd| costs a fixed distance; if that distance exceeds the window's own
//       width, no profile can absorb it inside the window — it must leave and come back,
//       sweeping travel that (1) never bounded. Measured against the CRX-10iA/L
//       envelope: a first knot 4.900° away at rest, with the arm commanded at 50 °/s,
//       passes (1) and sweeps 20.910° over 2.104 s, because shedding 50 °/s at
//       96 °/s² / 288 °/s³ costs 21.354°. Term (2) is what makes the 5° number mean
//       something about the motion rather than only about its endpoint.
//
// (2) is DIRECTION-FREE on purpose. A test on the SIGN of (q0 − q_cmd) would refuse every
// stale mid-flight replan rather than every infeasible one: a caller anchors knot 0 at
// the q_cmd it read from a snapshot, and by the time consume() runs the core's q_cmd_ has
// advanced, so the gap is negative by construction. Likewise there is no lower floor on
// the gap, because a splice whose target IS the current commanded state has a gap of
// exactly zero and nothing to shed, and refusing that would refuse a pure continuation.
// The reversal a negative gap causes is real and separately tracked; it is not converted
// into a refusal here.
//
// (2) is a CLOSED FORM in the seed state, so it is evaluated BEFORE generating. That is
// what makes an infeasible splice a typed REJECTED_START_MISMATCH instead of reaching the
// generator and failing its 300-knot ceiling, which the `!ok()` branch can only report as
// INTERNAL. It is also what lets the Python pre-flight reach the same verdict with no
// second implementation.
//
// No allocation, no loop beyond the six joints, one std::sqrt per joint. Called ONCE at
// CAPTURE entry, never per tick.
//
// The gate returns its PER-JOINT NUMBERS, not just a verdict, so this one derivation also
// feeds the Python typed error: the pybind binding exports `shed_travel` /
// `reject_joints` / `tol_exceeded` and `airo_fanuc.driver` formats them. There is
// deliberately no second implementation of this arithmetic anywhere in the tree.
// ---------------------------------------------------------------------------
struct CaptureGate {
  bool reject{false};
  bool tol_exceeded{false};      // term (1) failed on at least one joint
  std::uint32_t reject_mask{0};  // bit j set ⇔ term (2) failed on joint j
  // Travel that shedding |qd0 − qd_cmd| costs on joint j, at the brake-class clamps.
  // Zero when the endpoint velocities match, whatever their magnitude.
  Vec6 shed_travel{};
};

// Evaluate the gate. Pure and deterministic: same inputs → same result.
CaptureGate capture_gate(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& q0, const Vec6& qd0,
                         const TickEngineConfig& cfg);

// Policy helper: would this splice be REJECTED? The RT path needs only the verdict, and
// this is the same single evaluation — not a second, cheaper test.
inline bool capture_would_reject(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& q0,
                                 const Vec6& qd0, const TickEngineConfig& cfg) {
  return capture_gate(q_cmd, qd_cmd, q0, qd0, cfg).reject;
}

}  // namespace airo_fanuc::tick_engine
