// SPDX-License-Identifier: Apache-2.0
//
// Unit test — end-of-motion settle detection (SettlePolicy: 0.5°, 2°/s, 2 s).

#include <gtest/gtest.h>

#include "tick_engine/settle.hpp"
#include "tick_engine/tick_engine_config.hpp"

using airo_fanuc::tick_engine::deg2rad;
using airo_fanuc::tick_engine::SettleDetector;
using airo_fanuc::tick_engine::SettleState;
using airo_fanuc::tick_engine::TickEngineConfig;
using airo_fanuc::tick_engine::Vec6;

TEST(Settle, DoneWhenWithinTolerance) {
  TickEngineConfig cfg;
  SettleDetector det(cfg);
  det.reset();
  const Vec6 target{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  // Measured within 0.5° and < 2°/s.
  const Vec6 q_meas{0.1 + deg2rad(0.2), -0.2, 0.3, -0.4, 0.5, -0.6};
  const Vec6 qd_meas{deg2rad(1.0), 0, 0, 0, 0, 0};
  EXPECT_EQ(det.update(target, q_meas, qd_meas), SettleState::kDone);
}

TEST(Settle, NotConvergedAtToleranceBoundary) {
  TickEngineConfig cfg;
  SettleDetector det(cfg);
  const Vec6 target{};
  // Exactly at tol / eps → strict "<" means NOT converged.
  const Vec6 q_at_tol{cfg.settle_tol_rad, 0, 0, 0, 0, 0};
  EXPECT_FALSE(det.converged(target, q_at_tol, Vec6{}));
  const Vec6 qd_at_eps{cfg.settle_vel_eps_rad_s, 0, 0, 0, 0, 0};
  EXPECT_FALSE(det.converged(target, Vec6{}, qd_at_eps));
  // Just inside → converged.
  const Vec6 q_in{cfg.settle_tol_rad * 0.99, 0, 0, 0, 0, 0};
  const Vec6 qd_in{cfg.settle_vel_eps_rad_s * 0.99, 0, 0, 0, 0, 0};
  EXPECT_TRUE(det.converged(target, q_in, qd_in));
}

TEST(Settle, TimesOutWhenNeverConverging) {
  TickEngineConfig cfg;
  SettleDetector det(cfg);
  det.reset();
  const Vec6 target{};
  const Vec6 q_far{1.0, 0, 0, 0, 0, 0};  // way outside tol forever
  const Vec6 qd_still{};
  SettleState st = SettleState::kSettling;
  const int ticks = static_cast<int>(cfg.settle_timeout_s / cfg.itp_s) + 2;
  int done_tick = -1;
  for (int i = 0; i < ticks; ++i) {
    st = det.update(target, q_far, qd_still);
    if (st != SettleState::kSettling && done_tick < 0) {
      done_tick = i;
    }
  }
  EXPECT_EQ(st, SettleState::kTimeout);
  // Timeout fires at ~settle_timeout_s / itp ticks (250 for 2 s @ 8 ms).
  EXPECT_GE(done_tick, static_cast<int>(cfg.settle_timeout_s / cfg.itp_s) - 1);
}

TEST(Settle, LatchesOnceResolved) {
  TickEngineConfig cfg;
  SettleDetector det(cfg);
  det.reset();
  const Vec6 target{};
  // Converge immediately.
  EXPECT_EQ(det.update(target, Vec6{}, Vec6{}), SettleState::kDone);
  // Subsequent updates with a wildly-out-of-tolerance state stay kDone (latched).
  EXPECT_EQ(det.update(target, Vec6{5, 0, 0, 0, 0, 0}, Vec6{9, 0, 0, 0, 0, 0}), SettleState::kDone);
  EXPECT_EQ(det.state(), SettleState::kDone);
}
