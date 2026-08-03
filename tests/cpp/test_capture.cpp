// SPDX-License-Identifier: Apache-2.0
//
// Unit test — CAPTURE-or-REJECT splice generator. Asserts
// (a) determinism: the ONE generator produces byte-identical output for
// identical inputs — this is what makes "the checked path IS the executed path"
// (Python pre-flight collision-check and C++ execution call this same function),
// (b) it reaches the target under the 15°/s + brake-class envelope, and
// (c) the reject tolerance helper.

#include <cmath>
#include <cstring>

#include <gtest/gtest.h>

#include "tick_engine/capture.hpp"
#include "tick_engine/tick_engine_config.hpp"

using airo_fanuc::tick_engine::capture_gate;
using airo_fanuc::tick_engine::CaptureGate;
using airo_fanuc::tick_engine::capture_would_reject;
using airo_fanuc::tick_engine::CapturePath;
using airo_fanuc::tick_engine::generate_capture_path;
using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::TickEngineConfig;
using airo_fanuc::tick_engine::Vec6;

namespace {
const Vec6 kQCmd{0.00, -0.10, 0.20, -0.05, 0.15, -0.02};
const Vec6 kQdCmd{};
const Vec6 kQ0{0.05, -0.07, 0.22, -0.02, 0.18, 0.01};  // ~3° from q_cmd, within 5° window
const Vec6 kQd0{};
}  // namespace

// The determinism property — same inputs → identical CapturePath, bit-for-bit.
// A single deterministic generator is what guarantees the Python-checked path
// equals the C++-executed path.
TEST(Capture, DeterministicSameInputsIdenticalOutput) {
  TickEngineConfig cfg;
  CapturePath a;
  CapturePath b;
  generate_capture_path(kQCmd, kQdCmd, kQ0, kQd0, cfg, a);
  generate_capture_path(kQCmd, kQdCmd, kQ0, kQd0, cfg, b);

  ASSERT_EQ(a.count, b.count);
  ASSERT_TRUE(a.ok());
  for (int k = 0; k < a.count; ++k) {
    for (int j = 0; j < kNumJoints; ++j) {
      const auto jj = static_cast<std::size_t>(j);
      EXPECT_EQ(a.q[static_cast<std::size_t>(k)][jj], b.q[static_cast<std::size_t>(k)][jj])
          << "q knot " << k << " joint " << j;
      EXPECT_EQ(a.qd[static_cast<std::size_t>(k)][jj], b.qd[static_cast<std::size_t>(k)][jj])
          << "qd knot " << k << " joint " << j;
    }
  }
}

// The anchor (knot 0) is exactly (q_cmd, qd_cmd); the final knot reaches
// (q0, qd0). "Replaying" the generated knots (indexing) reproduces them — there
// is exactly one code path, so checked == executed by construction.
TEST(Capture, AnchorAndTargetExact) {
  TickEngineConfig cfg;
  CapturePath path;
  generate_capture_path(kQCmd, kQdCmd, kQ0, kQd0, cfg, path);
  ASSERT_TRUE(path.ok());
  ASSERT_GE(path.count, 2);

  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    EXPECT_EQ(path.q[0][jj], kQCmd[jj]) << "knot 0 == q_cmd exactly, joint " << j;
    EXPECT_EQ(path.qd[0][jj], kQdCmd[jj]) << "knot 0 == qd_cmd exactly, joint " << j;
    const auto last = static_cast<std::size_t>(path.count - 1);
    EXPECT_NEAR(path.q[last][jj], kQ0[jj], 1e-6) << "final knot reaches q0, joint " << j;
    EXPECT_NEAR(path.qd[last][jj], kQd0[jj], 1e-6) << "final knot reaches qd0, joint " << j;
  }
}

// The profile respects the 15°/s per-joint rate cap (velocity + per-tick delta).
TEST(Capture, RespectsRateEnvelope) {
  TickEngineConfig cfg;
  CapturePath path;
  generate_capture_path(kQCmd, kQdCmd, kQ0, kQd0, cfg, path);
  ASSERT_TRUE(path.ok());

  const double step_cap = cfg.capture_rate_rad_s * cfg.itp_s;
  for (int k = 0; k < path.count; ++k) {
    const auto kk = static_cast<std::size_t>(k);
    for (int j = 0; j < kNumJoints; ++j) {
      const auto jj = static_cast<std::size_t>(j);
      EXPECT_LE(std::abs(path.qd[kk][jj]), cfg.capture_rate_rad_s + 1e-9)
          << "velocity within 15°/s, knot " << k << " joint " << j;
      if (k > 0) {
        const double d = std::abs(path.q[kk][jj] - path.q[kk - 1][jj]);
        EXPECT_LE(d, step_cap + 1e-9) << "per-tick step within rate·itp, knot " << k << " joint " << j;
      }
    }
  }
}

// Non-zero target velocity (q0, qd0) is reached.
TEST(Capture, ReachesNonZeroTargetVelocity) {
  TickEngineConfig cfg;
  const Vec6 qd0{0.05, -0.05, 0.05, -0.05, 0.05, -0.05};  // within 15°/s
  CapturePath path;
  generate_capture_path(kQCmd, kQdCmd, kQ0, qd0, cfg, path);
  ASSERT_TRUE(path.ok());
  const auto last = static_cast<std::size_t>(path.count - 1);
  for (int j = 0; j < kNumJoints; ++j) {
    const auto jj = static_cast<std::size_t>(j);
    EXPECT_NEAR(path.qd[last][jj], qd0[jj], 1e-6) << "reaches qd0, joint " << j;
  }
}

// Gate term (1), the endpoint window: within 5° → accept; beyond → reject. From REST —
// which is every submission out of HOLD — the endpoint velocities match at zero, so term
// (2) contributes nothing and this is the whole gate, exactly as it always was.
TEST(Capture, WouldRejectBeyondTolerance) {
  TickEngineConfig cfg;
  EXPECT_FALSE(capture_would_reject(kQCmd, kQdCmd, kQ0, kQd0, cfg))
      << "3° splice is within the 5° window";

  Vec6 far = kQCmd;
  far[2] += cfg.capture_tol_rad + 0.01;  // just over 5° on joint 2
  EXPECT_TRUE(capture_would_reject(kQCmd, kQdCmd, far, kQd0, cfg))
      << ">5° splice must be rejected";

  const CaptureGate g = capture_gate(kQCmd, kQdCmd, far, kQd0, cfg);
  EXPECT_TRUE(g.tol_exceeded);
  EXPECT_EQ(g.reject_mask, 0u) << "nothing to shed between two zero velocities";
  for (int j = 0; j < kNumJoints; ++j) {
    EXPECT_EQ(g.shed_travel[static_cast<std::size_t>(j)], 0.0) << "joint " << j;
  }
}

// Gate term (2). A first knot INSIDE the 5° window, at rest, while the joint is commanded
// fast: term (1) passes and the deceleration cannot fit in the window. This is the
// 20.9°-swept-against-a-5°-gate case, in the synthetic envelope.
TEST(Capture, GateRejectsAVelocityTheWindowCannotAbsorb) {
  TickEngineConfig cfg;
  const double v = 0.8;  // rad/s on joint 0
  Vec6 qdc{};
  qdc[0] = v;
  Vec6 q0 = kQCmd;
  q0[0] += 0.08;  // ~4.58°, comfortably inside capture_tol
  ASSERT_LT(std::abs(q0[0] - kQCmd[0]), cfg.capture_tol_rad);

  const CaptureGate g = capture_gate(kQCmd, qdc, q0, Vec6{}, cfg);
  EXPECT_TRUE(g.reject) << "an infeasible splice must be refused, not swept";
  EXPECT_FALSE(g.tol_exceeded) << "the endpoint gap is inside the window; term (2) fails";
  EXPECT_EQ(g.reject_mask, 1u) << "joint 0 only";

  // Closed form, restated from the envelope: trapezoidal shed (|dv| >= a_b^2/j_b), travel
  // = mean speed x duration.
  const double a_b = cfg.stop_scale_va * cfg.limits.a[0];
  const double j_b = cfg.stop_scale_j * cfg.limits.j[0];
  ASSERT_GE(v, a_b * a_b / j_b);
  const double need = 0.5 * v * (v / a_b + a_b / j_b);
  EXPECT_NEAR(g.shed_travel[0], need, 1e-12);
  EXPECT_GT(need, cfg.capture_tol_rad)
      << "the shed travel exceeds the window the old gate measured — the whole point";
}

// The gate is DIRECTION-FREE. A first knot behind the commanded pose is not refused on
// that ground: a caller's snapshot is always at least one tick stale, so a negative gap is
// the normal case for a mid-flight replan, not an error. (The reversal such a gap causes is
// a separate defect with a separate fix; this gate does not turn it into a refusal.)
TEST(Capture, GateIsDirectionFree) {
  TickEngineConfig cfg;
  const double v = 0.2;
  Vec6 qdc{};
  qdc[0] = v;
  Vec6 qd0m{};
  qd0m[0] = v;  // matched: nothing to shed

  for (const double delta : {-3.0 * v * cfg.itp_s, -v * cfg.itp_s, 0.0, v * cfg.itp_s}) {
    Vec6 q0 = kQCmd;
    q0[0] += delta;  // behind, level, and ahead — all inside the window
    const CaptureGate g = capture_gate(kQCmd, qdc, q0, qd0m, cfg);
    EXPECT_FALSE(g.reject) << "refused a gap of " << delta << " rad on sign alone";
    EXPECT_EQ(g.shed_travel[0], 0.0) << "matched endpoint velocities cost no travel";
  }
}

// *** COMPATIBILITY INVARIANT — read before adding any term to this gate. ***
// A join-at-phase submission targets the trajectory at the phase the arm has actually
// reached, so its target IS the current commanded state: gap 0.000 and |dqd| 0.000, by
// construction. The gate must not refuse it. Any directional term, and any lower floor on
// the gap, breaks this — that is why neither is here.
TEST(Capture, GateDoesNotRefuseAJoinAtPhaseSubmission) {
  TickEngineConfig cfg;
  for (const double v : {0.0, 0.05, 0.2, 0.8, 2.0}) {
    Vec6 qdc{};
    qdc[0] = v;
    // Target == current state, exactly.
    const CaptureGate g = capture_gate(kQCmd, qdc, kQCmd, qdc, cfg);
    EXPECT_FALSE(g.reject)
        << "a continuation whose target is the current commanded state was refused at "
        << v << " rad/s — this is the submission a phase join exists to make work";
    EXPECT_FALSE(g.tol_exceeded);
    EXPECT_EQ(g.reject_mask, 0u);
    EXPECT_EQ(g.shed_travel[0], 0.0);
  }
}
