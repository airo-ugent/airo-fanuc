// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — cubic-Hermite trajectory resampling.
//
// The cubic-Hermite basis, plus analytic first/second derivatives (the brake qdd
// seed needs them) and the qd_end blend.
//
// BASIS (canonical s ∈ [0, 1], tangents scaled by interval length h_s in s):
//     s   = (t_query_ns - t0_ns) / (t1_ns - t0_ns)
//     h00 = 2s³ - 3s² + 1;  h10 = s³ - 2s² + s
//     h01 = -2s³ + 3s²;     h11 = s³ - s²
//     q(t) = h00·q0 + h10·h_s·qd0 + h01·q1 + h11·h_s·qd1     (per joint)
// with h_s = (t1_ns - t0_ns) / 1e9 (seconds).
//
// BIT-EXACTNESS: the position expression is PINNED — `hermite_at_ns` evaluates it
// in exactly the operation order and associativity written above, and the
// tick_engine translation unit is compiled with `-ffp-contract=off` so no
// `a*b + c` fuses into an FMA (which would re-round and shift the result by an
// ULP). This is what makes resampling reproducible to the bit: the same knots and
// query time yield the same commanded position on every host and in every build,
// so a replay of a trajectory is the same motion the RT loop commanded. The C++
// unit test `test_hermite` therefore pins the position against hexfloat values
// with an EQUALITY assertion rather than a tolerance — which is what catches any
// reassociation of the expression. Do not "tidy up" the arithmetic below into a
// factored or Horner form, and do not drop `-ffp-contract=off` from the target.
//
// UNITS: radians / rad·s⁻¹ / rad·s⁻² throughout (see tick_engine_config.hpp).

#pragma once

#include <cstdint>

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::tick_engine {

// A sampled kinematic state (per joint): position, velocity, acceleration.
// For `sample`/`hermite_at_ns` in NATURAL trajectory time these are dq/dt and
// d²q/dt². `TrajectorySampler::sample` applies speed_scale so its qd/qdd are the
// WIRE quantities (see below).
struct HermiteSample {
  Vec6 q{};
  Vec6 qd{};
  Vec6 qdd{};
};

// Evaluate the cubic-Hermite basis for a given canonical parameter `s` and
// interval length `h_s` (seconds). Returns natural-time q, qd = dq/dt,
// qdd = d²q/dt². This is the shared core; `s`/`h_s` are computed by the callers.
// The position uses the pinned operation order (see the header note).
HermiteSample hermite_eval(const Vec6& q0, const Vec6& qd0, const Vec6& q1, const Vec6& qd1, double s,
                           double h_s);

// Evaluation at an integer query time (ns). The position is bit-reproducible (see
// the header note); qd/qdd are the analytic derivatives. If (t1_ns - t0_ns) <= 0
// the interval is degenerate: returns q1 with zero qd/qdd rather than dividing by
// zero.
HermiteSample hermite_at_ns(std::int64_t t0_ns, const Vec6& q0, const Vec6& qd0, std::int64_t t1_ns,
                            const Vec6& q1, const Vec6& qd1, std::int64_t t_query_ns);

// ---------------------------------------------------------------------------
// TrajectorySampler — resamples ONE whole trajectory (times, q, qd) against a
// captured start anchor. Every sample is evaluated directly from the bracketing
// knot pair; there is no chunk merge-buffer to keep consistent.
//
// The view is NON-OWNING: `times_ns`, `q`, `qd` are allocated/filled by the
// calling thread and must outlive the sampler — the RT thread never allocates.
// Times are int64 ns, strictly increasing, n ≥ 2.
//
// SPEED SCALE s_v: the trajectory is evaluated at scaled time. For a wire-elapsed
// time τ (ns since playback start, i.e. since
// times[0]), the trajectory time is  t = times[0] + s_v·τ, and
//     q_wire(τ)   = q(t)
//     qd_wire(τ)  = s_v · qd(t)      (a 0.5× scale halves velocities)
//     qdd_wire(τ) = s_v² · qdd(t)    (chain rule; used for the brake qdd seed)
// τ is clamped to [0, duration/s_v]: before start → first knot at rest; at/after
// end → last knot with qd_wire = qdd_wire = 0 (the qd_end blend + hold take over).
// ---------------------------------------------------------------------------
class TrajectorySampler {
 public:
  TrajectorySampler() = default;

  // Bind a trajectory view. n ≥ 2 and strictly-increasing times are the caller's
  // contract (validated in Python and again at the C++ API boundary); `bind`
  // asserts n ≥ 2 in debug builds and clamps to a hold otherwise.
  void bind(const std::int64_t* times_ns, const Vec6* q, const Vec6* qd, int n);
  void clear();
  bool bound() const { return n_ >= 2; }
  int size() const { return n_; }

  // Trajectory span in natural time (ns): times[n-1] - times[0].
  std::int64_t duration_ns() const;

  // Sample at wire-elapsed tau_ns (ns since times[0]) under speed_scale.
  HermiteSample sample(std::int64_t tau_ns, double speed_scale) const;

  // Terminal knot (for the qd_end blend / hold). qd is the NATURAL terminal
  // velocity; multiply by speed_scale to get the wire terminal velocity.
  const Vec6& q_last() const { return q_[static_cast<std::size_t>(n_ - 1)]; }
  const Vec6& qd_last() const { return qd_[static_cast<std::size_t>(n_ - 1)]; }

 private:
  const std::int64_t* times_{nullptr};
  const Vec6* q_{nullptr};
  const Vec6* qd_{nullptr};
  int n_{0};
};

// ---------------------------------------------------------------------------
// QdEndBlend — on trajectory exhaustion with a non-zero terminal velocity, blend
// (q_end, qd_end) → REST over ≥ qd_end_blend_min_s via ONE cubic-Hermite segment.
// `qd_end` here is the WIRE terminal velocity (already speed-scaled).
//
// WHY THE BLEND COASTS FORWARD (do not "simplify" it back). The tempting
// formulation is a Hermite from (q_end, qd_end) to (q_end, 0) — hold the endpoint,
// zero the velocity. Between EQUAL endpoints that yields
//     qd(s) = (3s² − 4s + 1)·qd_end,
// which dips NEGATIVE at s = 2/3: the arm overshoots q_end, then reverses
// direction to crawl back onto it at the end of every trajectory. That is not
// monotone and not the motion anyone asked for.
//
// Coasting to q_coast = q_end + (T/2)·qd_end instead yields the exact LINEAR
// velocity ramp qd(t) = (1 − s)·qd_end (derivation in hermite.cpp): monotone by
// construction, zero at s = 1, and with constant acceleration −qd_end/T so there
// is no jerk spike at either end. Handing the terminal velocity to a Ruckig
// velocity→0 brake coasts forward for the same reason, so the two agree on the
// physics. `QdEndBlend.MonotoneRampToRest` pins the monotone, step-free ramp.
// ---------------------------------------------------------------------------
class QdEndBlend {
 public:
  // Plan a blend. `T` is snapped UP to a whole number of ticks ≥ min_blend_s.
  // If every |qd_end| is ~0, `active()` is false and no blend is warranted.
  void plan(const Vec6& q_end, const Vec6& qd_end, double min_blend_s, double itp_s);
  void clear();

  bool active() const { return active_; }
  double duration_s() const;
  std::int64_t duration_ns() const { return dur_ns_; }
  int num_ticks() const { return num_ticks_; }
  // The recorded |qd_end|∞, for the submit helper's diagnostics.
  double max_abs_qd_end() const { return max_abs_qd_end_; }

  // Sample the blend at tau_ns ∈ [0, duration]. Wire (q, qd) with qd monotone
  // → 0. Past the end returns (q_coast, 0).
  HermiteSample sample(std::int64_t tau_ns) const;
  const Vec6& q_coast() const { return q_coast_; }

 private:
  Vec6 q_end_{};
  Vec6 qd_end_{};
  Vec6 q_coast_{};
  std::int64_t dur_ns_{0};
  int num_ticks_{0};
  double max_abs_qd_end_{0.0};
  bool active_{false};
};

}  // namespace airo_fanuc::tick_engine
