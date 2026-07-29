// SPDX-License-Identifier: Apache-2.0
//
// L1 — brake qdd-seed continuity ACROSS THE ACTUAL TickCore TRAJECTORY→BRAKE
// transition (R1 C1). test_brake proves the brake PRIMITIVE is accel-continuous
// when seeded with an analytic qdd; this proves the CORE actually wires the live
// analytic acceleration (`qdd_cmd_` from the Hermite sampler) into that seed at a
// real mid-segment stop_j — i.e. a regression that seeds qdd=0 in begin_brake
// (the old dries behaviour that latched the CRX contact-stop) would be caught
// here even though the primitive test stays green.
//
// Observable: TickCore exposes qd_cmd() only, so acceleration is recovered as the
// per-tick finite difference of the commanded velocity. Continuity = the accel
// does not STEP across the splice (a qdd=0 seed would step the decel in fully).

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"

using namespace airo_fanuc::rt_core;
using airo_fanuc::tick_engine::Vec6;

namespace {

RxSample clean_rx(const Vec6& q, const Vec6& qd = Vec6{}) {
  RxSample rx{};
  rx.q_meas = q;
  rx.qd_est = qd;
  rx.gates.motion_possible = true;
  rx.safety_scale = 1.0;
  return rx;
}

void init_holding(TickCore& tc, const Vec6& q0) {
  tc.begin_streaming(q0);
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, nullptr);
}

}  // namespace

TEST(TrajBrakeContinuity, AccelContinuousAcrossMidSegmentStopJ) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;

  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  // A GENTLE 1 s rest-to-rest segment (J0: 0 → 0.2 rad). Peak |qd| ≈ 0.3 rad/s
  // and peak |qdd| ≈ 1.2 rad/s² — both well under the slew cap and the brake
  // accel envelope, so qd_cmd == the analytic Hermite value (no slew clip) and
  // the brake can track the seed accel.
  const std::array<std::int64_t, 2> times{0, 1'000'000'000};
  const std::array<Vec6, 2> q{q0, Vec6{0.2, 0, 0, 0, 0, 0}};
  const std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  t.speed_scale = 1.0;

  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);

  // Play ~250 ms into the segment (s ≈ 0.25, where |qdd| is meaningfully
  // non-zero), recording the commanded velocity each tick.
  std::vector<double> qd0_hist;
  qd0_hist.push_back(tc.qd_cmd()[0]);
  int traj_ticks = 0;
  for (int i = 0; i < 40; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
    qd0_hist.push_back(tc.qd_cmd()[0]);
    if (tc.mode() == Mode::TRAJECTORY) ++traj_ticks;
    if (traj_ticks >= 30) break;  // ~240 ms of trajectory playback
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);

  // Acceleration just before the stop (finite diff of the last two traj samples).
  const std::size_t n = qd0_hist.size();
  ASSERT_GE(n, 3u);
  const double qd_last = qd0_hist[n - 1];
  const double accel_traj_last = (qd0_hist[n - 1] - qd0_hist[n - 2]) / itp;

  // Precondition: the mid-segment acceleration is meaningfully non-zero, so
  // "no accel step at the splice" is a real constraint (not vacuously true).
  ASSERT_GT(std::abs(accel_traj_last), 0.3) << "need a meaningfully non-zero pre-brake accel";

  // stop_j → next tick is the FIRST BRAKE command.
  tc.request_stop();
  RxSample rxb = clean_rx(tc.q_cmd(), tc.qd_cmd());
  tc.tick(&rxb, nullptr);
  ASSERT_EQ(tc.mode(), Mode::BRAKE);
  const double qd_brake0 = tc.qd_cmd()[0];
  const double accel_brake0 = (qd_brake0 - qd_last) / itp;

  // R1 C1 — acceleration is CONTINUOUS across the splice: the first brake accel
  // is within a few jerk-limited steps of the trajectory accel it seeded from.
  const double j_env0 = cfg.tick.stop_scale_j * cfg.tick.limits.j[0];
  const double jerk_step = j_env0 * itp;  // ~0.04 rad/s²
  const double accel_step = std::abs(accel_brake0 - accel_traj_last);
  EXPECT_LE(accel_step, 4.0 * jerk_step)
      << "accel step across TRAJECTORY→BRAKE within jerk-limited continuity "
      << "(seed=" << accel_traj_last << " brake0=" << accel_brake0 << ")";
  // And the step is a small FRACTION of the accel magnitude — an unseeded (qdd=0)
  // begin_brake would step the full ~0.6 rad/s² decel in immediately.
  EXPECT_LT(accel_step, 0.25 * std::abs(accel_traj_last))
      << "no acceleration discontinuity at the splice";

  // Velocity is continuous too (brake starts from the trajectory's qd_cmd).
  EXPECT_LT(std::abs(qd_brake0 - qd_last), cfg.tick.stop_scale_va * cfg.tick.limits.a[0] * itp + 1e-9);

  // The brake then jerk-limits its way to rest: no single-tick accel jump beyond
  // the jerk envelope over the next several ticks.
  double prev_accel = accel_brake0;
  double prev_qd = qd_brake0;
  for (int i = 0; i < 8; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
    const double qd_now = tc.qd_cmd()[0];
    const double accel_now = (qd_now - prev_qd) / itp;
    EXPECT_LE(std::abs(accel_now - prev_accel), 2.0 * jerk_step + 1e-6)
        << "brake accel stays jerk-limited (tick " << i << ")";
    prev_accel = accel_now;
    prev_qd = qd_now;
  }
}

namespace {

// Shared continuity assertion for a <mode>→BRAKE splice. `accel_pre` is the
// commanded accel (finite diff of qd_cmd) on the last pre-brake tick; the core
// is left with `qd_cmd()==qd_last` and its qdd_cmd_ analytic seed. Issues stop_j,
// steps ONE tick (the first BRAKE command) and asserts the first brake accel is
// continuous with `accel_pre` — the R1 C1 invariant a qdd=0 seed would violate.
void assert_brake_seed_continuous(TickCore& tc, double itp, double accel_pre, double qd_last,
                                  const char* what) {
  // Precondition: the pre-brake accel is meaningfully non-zero, so "no step" is a
  // real constraint (a qdd=0 seed would step this full magnitude in immediately).
  ASSERT_GT(std::abs(accel_pre), 0.3) << what << ": need a meaningfully non-zero pre-brake accel";

  tc.request_stop();
  RxSample rxb = clean_rx(tc.q_cmd(), tc.qd_cmd());
  tc.tick(&rxb, nullptr);
  ASSERT_EQ(tc.mode(), Mode::BRAKE) << what;
  const double qd_brake0 = tc.qd_cmd()[0];
  const double accel_brake0 = (qd_brake0 - qd_last) / itp;

  RtCoreConfig cfg;
  const double j_env0 = cfg.tick.stop_scale_j * cfg.tick.limits.j[0];
  const double jerk_step = j_env0 * itp;
  const double accel_step = std::abs(accel_brake0 - accel_pre);
  // R1 C1: continuous within a few jerk-limited steps AND a small FRACTION of the
  // accel magnitude (an unseeded qdd=0 begin_brake would step the full decel in).
  EXPECT_LE(accel_step, 4.0 * jerk_step)
      << what << ": accel step across the splice within jerk-limited continuity "
      << "(seed=" << accel_pre << " brake0=" << accel_brake0 << ")";
  EXPECT_LT(accel_step, 0.25 * std::abs(accel_pre)) << what << ": no acceleration discontinuity";

  // The brake then jerk-limits to rest: no single-tick accel jump beyond the env.
  double prev_accel = accel_brake0;
  double prev_qd = qd_brake0;
  for (int i = 0; i < 6; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
    const double qd_now = tc.qd_cmd()[0];
    const double accel_now = (qd_now - prev_qd) / itp;
    EXPECT_LE(std::abs(accel_now - prev_accel), 2.0 * jerk_step + 1e-6)
        << what << ": brake accel stays jerk-limited (tick " << i << ")";
    prev_accel = accel_now;
    prev_qd = qd_now;
  }
}

}  // namespace

// R1 C1 — braking OUT OF CAPTURE must seed the analytic capture acceleration, not
// 0. Regression: begin_brake used qdd_cmd_==Vec6{} out of CAPTURE (capture.qdd was
// not stored), stepping the full capture-class decel into the brake in one tick.
TEST(TrajBrakeContinuity, AccelContinuousAcrossCaptureStopJ) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;

  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  // Trajectory whose START q0 is ~4° (0.07 rad) from the commanded pose → within
  // the 5° window → a multi-tick CAPTURE splice (15°/s, brake-class a/j) runs
  // first, where the commanded acceleration ramps up meaningfully.
  const std::array<std::int64_t, 2> times{0, 1'000'000'000};
  const std::array<Vec6, 2> q{Vec6{0.07, 0, 0, 0, 0, 0}, Vec6{0.27, 0, 0, 0, 0, 0}};
  const std::array<Vec6, 2> qd{Vec6{}, Vec6{}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  t.speed_scale = 1.0;

  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);
  ASSERT_EQ(tc.mode(), Mode::CAPTURE) << "a > tol-rate splice enters CAPTURE";

  // Play CAPTURE ticks until the commanded acceleration (finite diff of qd_cmd)
  // is clearly non-zero, then stop there — still inside CAPTURE.
  double qd_prev = tc.qd_cmd()[0];
  double accel_pre = 0.0;
  double qd_last = 0.0;
  bool ready = false;
  for (int i = 0; i < 40 && !ready; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    tc.tick(&rx, nullptr);
    const double qd_now = tc.qd_cmd()[0];
    const double accel = (qd_now - qd_prev) / itp;
    if (tc.mode() == Mode::CAPTURE && std::abs(accel) > 0.35) {
      accel_pre = accel;
      qd_last = qd_now;
      ready = true;
    }
    qd_prev = qd_now;
  }
  ASSERT_TRUE(ready) << "never observed a meaningfully-accelerating CAPTURE tick";
  ASSERT_EQ(tc.mode(), Mode::CAPTURE);

  assert_brake_seed_continuous(tc, itp, accel_pre, qd_last, "CAPTURE→BRAKE");
}

// R1 C1 — braking OUT OF the qd_end BLEND must seed the blend's analytic accel
// (constant −qd_end/T), not 0. Regression: begin_brake used qdd_cmd_==Vec6{} in
// kEndBlend, stepping the blend decel into the brake.
TEST(TrajBrakeContinuity, AccelContinuousAcrossEndBlendStopJ) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;

  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  // A CONSTANT-velocity segment (qd0==qd1==0.045 rad/s, q1==0.045·0.2 s) so the
  // playback velocity is a flat cruise; on exhaustion the non-zero terminal
  // velocity triggers the qd_end blend (0.045 > settle_vel_eps ≈ 0.035 rad/s) and
  // its constant blend accel (−0.045/T ≈ −1.4 rad/s²) is within the brake env.
  const double v = 0.045;
  const std::int64_t dur = 200'000'000;
  const double q1 = v * (static_cast<double>(dur) / 1e9);  // 0.009 rad → exactly linear
  const std::array<std::int64_t, 2> times{0, dur};
  const std::array<Vec6, 2> q{Vec6{0, 0, 0, 0, 0, 0}, Vec6{q1, 0, 0, 0, 0, 0}};
  const std::array<Vec6, 2> qd{Vec6{v, 0, 0, 0, 0, 0}, Vec6{v, 0, 0, 0, 0, 0}};
  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.epoch = tc.epoch();
  t.times_ns = times.data();
  t.q = q.data();
  t.qd = qd.data();
  t.n = 2;
  t.speed_scale = 1.0;

  RxSample rx0 = clean_rx(q0);
  tc.tick(&rx0, &t);

  // Play until we detect the qd_end blend: velocity has cruised at ~v and then
  // dropped below it while still moving (the linear ramp toward rest).
  double qd_prev = tc.qd_cmd()[0];
  bool reached_cruise = false;
  double accel_pre = 0.0;
  double qd_last = 0.0;
  bool in_blend = false;
  for (int i = 0; i < 200 && !in_blend; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
    const double qd_now = tc.qd_cmd()[0];
    if (tc.mode() == Mode::TRAJECTORY && qd_now >= 0.98 * v) reached_cruise = true;
    if (reached_cruise && tc.mode() == Mode::TRAJECTORY && qd_now < 0.9 * v && qd_now > 0.2 * v) {
      in_blend = true;
      accel_pre = (qd_now - qd_prev) / itp;
      qd_last = qd_now;
    }
    qd_prev = qd_now;
  }
  ASSERT_TRUE(in_blend) << "never observed the qd_end blend ramp";
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY) << "blend runs under Mode::TRAJECTORY";

  assert_brake_seed_continuous(tc, itp, accel_pre, qd_last, "kEndBlend→BRAKE");
}
