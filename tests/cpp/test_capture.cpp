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

// Reject tolerance helper: within 5° → accept; beyond → reject.
TEST(Capture, WouldRejectBeyondTolerance) {
  TickEngineConfig cfg;
  EXPECT_FALSE(capture_would_reject(kQCmd, kQ0, cfg)) << "3° splice is within the 5° window";

  Vec6 far = kQCmd;
  far[2] += cfg.capture_tol_rad + 0.01;  // just over 5° on joint 2
  EXPECT_TRUE(capture_would_reject(kQCmd, far, cfg)) << ">5° splice must be rejected";
}
