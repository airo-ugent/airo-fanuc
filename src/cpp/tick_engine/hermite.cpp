// SPDX-License-Identifier: Apache-2.0
//
// Cubic-Hermite resampling — see hermite.hpp. Compiled with -ffp-contract=off
// (set on the `tick_engine` target) so nothing in the position expression fuses
// into an FMA and the result stays bit-reproducible.

#include "tick_engine/hermite.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace airo_fanuc::tick_engine {

HermiteSample hermite_eval(const Vec6& q0, const Vec6& qd0, const Vec6& q1, const Vec6& qd1, double s,
                           double h_s) {
  const double s2 = s * s;
  const double s3 = s2 * s;

  // Basis (pinned order/literals).
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;

  // First derivatives of the basis w.r.t. s.
  const double h00d = 6.0 * s2 - 6.0 * s;
  const double h10d = 3.0 * s2 - 4.0 * s + 1.0;
  const double h01d = -6.0 * s2 + 6.0 * s;
  const double h11d = 3.0 * s2 - 2.0 * s;

  // Second derivatives of the basis w.r.t. s.
  const double h00dd = 12.0 * s - 6.0;
  const double h10dd = 6.0 * s - 4.0;
  const double h01dd = -12.0 * s + 6.0;
  const double h11dd = 6.0 * s - 2.0;

  const double inv_h = 1.0 / h_s;
  const double inv_h2 = inv_h * inv_h;

  HermiteSample out{};
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    // Position — PINNED op order; do not reassociate or factor (see hermite.hpp).
    out.q[j] = h00 * q0[j] + h10 * h_s * qd0[j] + h01 * q1[j] + h11 * h_s * qd1[j];

    // Natural velocity dq/dt. The h_s in the tangent terms cancels 1/h_s, so at
    // the knots qd(0)=qd0, qd(1)=qd1 exactly.
    out.qd[j] = (h00d * q0[j] + h01d * q1[j]) * inv_h + h10d * qd0[j] + h11d * qd1[j];

    // Natural acceleration d²q/dt².
    out.qdd[j] = (h00dd * q0[j] + h01dd * q1[j]) * inv_h2 + (h10dd * qd0[j] + h11dd * qd1[j]) * inv_h;
  }
  return out;
}

HermiteSample hermite_at_ns(std::int64_t t0_ns, const Vec6& q0, const Vec6& qd0, std::int64_t t1_ns,
                            const Vec6& q1, const Vec6& qd1, std::int64_t t_query_ns) {
  const std::int64_t h_ns = t1_ns - t0_ns;
  if (h_ns <= 0) {
    // Degenerate-interval guard: return q1 for the position; the derivatives are
    // undefined there (division by a zero-length interval) — report zero.
    HermiteSample out{};
    out.q = q1;
    return out;
  }
  // Pinned: take the int64 differences first, widen to double, then one IEEE
  // divide — reordering these changes the last bit of `s`.
  const double h_s = static_cast<double>(h_ns) / 1e9;
  const double s = static_cast<double>(t_query_ns - t0_ns) / static_cast<double>(h_ns);
  return hermite_eval(q0, qd0, q1, qd1, s, h_s);
}

// ---------------------------------------------------------------------------
// TrajectorySampler
// ---------------------------------------------------------------------------
void TrajectorySampler::bind(const std::int64_t* times_ns, const Vec6* q, const Vec6* qd, int n) {
  assert(n >= 2 && "TrajectorySampler requires >= 2 knots");
  times_ = times_ns;
  q_ = q;
  qd_ = qd;
  n_ = n;
}

void TrajectorySampler::clear() {
  times_ = nullptr;
  q_ = nullptr;
  qd_ = nullptr;
  n_ = 0;
}

std::int64_t TrajectorySampler::duration_ns() const {
  if (n_ < 2) {
    return 0;
  }
  return times_[n_ - 1] - times_[0];
}

HermiteSample TrajectorySampler::sample(std::int64_t tau_ns, double speed_scale) const {
  if (n_ < 1) {
    return HermiteSample{};
  }
  if (n_ == 1) {
    HermiteSample out{};
    out.q = q_[0];
    return out;
  }

  const double t0d = static_cast<double>(times_[0]);
  const double t_end_d = static_cast<double>(times_[n_ - 1]);

  // Trajectory query time (double ns) = times[0] + s_v · τ. Clamp to the span.
  double t_query = t0d + speed_scale * static_cast<double>(tau_ns);

  // Before start → hold first knot at rest.
  if (t_query <= t0d) {
    HermiteSample out{};
    out.q = q_[0];
    return out;
  }
  // At/after end → hold last knot; qd/qdd forced to 0 (qd_end blend/hold take over).
  if (t_query >= t_end_d) {
    HermiteSample out{};
    out.q = q_[n_ - 1];
    return out;
  }

  // Locate the bracketing segment [times[i], times[i+1]] containing t_query.
  // Logarithmic per sample; trajectories are short and the tick core caches its
  // own cursor, so a std::upper_bound over the int64 times is enough here.
  int hi = static_cast<int>(std::upper_bound(times_, times_ + n_, static_cast<std::int64_t>(t_query)) - times_);
  // t_query is strictly inside (t0, t_end) so 1 <= hi <= n-1.
  if (hi < 1) {
    hi = 1;
  }
  if (hi > n_ - 1) {
    hi = n_ - 1;
  }
  const int lo = hi - 1;

  const double seg_ns = static_cast<double>(times_[hi] - times_[lo]);
  const double h_s = seg_ns / 1e9;
  const double s = (t_query - static_cast<double>(times_[lo])) / seg_ns;

  HermiteSample nat = hermite_eval(q_[lo], qd_[lo], q_[hi], qd_[hi], s, h_s);

  // Apply speed_scale to get WIRE kinematics (position unscaled).
  const double sv = speed_scale;
  const double sv2 = sv * sv;
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    nat.qd[j] = sv * nat.qd[j];
    nat.qdd[j] = sv2 * nat.qdd[j];
  }
  return nat;
}

// ---------------------------------------------------------------------------
// QdEndBlend
//
// Blend (q_end, qd_end) → rest with a MONOTONE velocity ramp. We choose the
// coast endpoint q_coast = q_end + (T/2)·qd_end and target velocity 0. With
// q1 - q0 = (h_s/2)·qd_end and qd1 = 0, the Hermite velocity is:
//   qd(t) = (1/h_s)(h00'·q0 + h01'·q1) + h10'·qd0
//         = h01'·(q1-q0)/h_s + h10'·qd_end        [h00'+h01' = 0]
//         = (3s - 3s²)·qd_end + (3s² - 4s + 1)·qd_end
//         = (1 - s)·qd_end                         [LINEAR, monotone → 0]
// and the position is q(t) = q_end + h_s·qd_end·(s - 0.5 s²) (glides to q_coast).
// ---------------------------------------------------------------------------
void QdEndBlend::plan(const Vec6& q_end, const Vec6& qd_end, double min_blend_s, double itp_s) {
  q_end_ = q_end;
  qd_end_ = qd_end;

  double max_abs = 0.0;
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    max_abs = std::max(max_abs, std::abs(qd_end[j]));
  }
  max_abs_qd_end_ = max_abs;

  // Below the settle velocity there is nothing to ramp; treat as inactive.
  // (The caller/hold path handles a near-zero terminal velocity directly.)
  if (max_abs <= 0.0) {
    active_ = false;
    num_ticks_ = 0;
    dur_ns_ = 0;
    q_coast_ = q_end;
    return;
  }

  // Snap the blend duration UP to a whole number of ticks ≥ min_blend_s.
  const int min_ticks = static_cast<int>(std::ceil(min_blend_s / itp_s - 1e-9));
  num_ticks_ = std::max(1, min_ticks);
  const double dur_s = static_cast<double>(num_ticks_) * itp_s;
  dur_ns_ = static_cast<std::int64_t>(std::llround(dur_s * 1e9));

  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    q_coast_[j] = q_end[j] + 0.5 * dur_s * qd_end[j];
  }
  active_ = true;
}

void QdEndBlend::clear() {
  active_ = false;
  num_ticks_ = 0;
  dur_ns_ = 0;
  max_abs_qd_end_ = 0.0;
  q_end_ = Vec6{};
  qd_end_ = Vec6{};
  q_coast_ = Vec6{};
}

double QdEndBlend::duration_s() const { return static_cast<double>(dur_ns_) / 1e9; }

HermiteSample QdEndBlend::sample(std::int64_t tau_ns) const {
  if (!active_ || tau_ns >= dur_ns_) {
    HermiteSample out{};
    out.q = q_coast_;
    return out;
  }
  if (tau_ns <= 0) {
    // Entry sample: exact (q_end, qd_end). The velocity ramp qd(t)=(1−t/T)·qd_end
    // is LINEAR, so its acceleration is the constant −qd_end/T at every instant,
    // INCLUDING t=0 — the interior hermite path returns exactly this. Populate it
    // here too so braking on the FIRST blend tick seeds the analytic accel rather
    // than a spurious 0: a zero seed would step the commanded acceleration and the
    // CRX contact-stop monitor reads that step as contact.
    // (dur_ns_ > 0 whenever active_, so no div-by-zero.)
    HermiteSample out{};
    out.q = q_end_;
    out.qd = qd_end_;
    const double h_s = static_cast<double>(dur_ns_) / 1e9;
    for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
      out.qdd[j] = -qd_end_[j] / h_s;
    }
    return out;
  }
  const Vec6 zero{};
  return hermite_at_ns(0, q_end_, qd_end_, dur_ns_, q_coast_, zero, tau_ns);
}

}  // namespace airo_fanuc::tick_engine
