// SPDX-License-Identifier: Apache-2.0
//
// Unit test — commanded-position PROFILES across the four one-tick discontinuities that
// a single-state assertion cannot see: the CAPTURE splice head, the SAFE_FOLLOW fault
// ramp head, the qd_end blend handoff, and the RX_SILENT park exit.
//
// Every assertion here is on the SEQUENCE of commanded positions returned by
// TickCore::tick — the quantity that actually reaches the wire — rather than on an
// internal phase variable, because each defect is "one tick commanded the pose it had
// already commanded", which is invisible in every scalar the snapshot exposes.
//
// Expectations are derived from `cfg` throughout (the envelope here is the synthetic
// tick-engine default, not any arm's) so changing that envelope cannot quietly
// invalidate them.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"

using namespace airo_fanuc::rt_core;
using airo_fanuc::tick_engine::kNumJoints;
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

bool drain_for(TickCore& tc, EventType want) {
  Event e{};
  bool found = false;
  while (tc.pop_event(e)) {
    if (e.type == want) found = true;
  }
  return found;
}

// Whole ticks a blend of `min_s` occupies — QdEndBlend::plan's own expression, restated
// so these tests do not hardcode 8 or 4.
int blend_ticks(double min_s, double itp) {
  return std::max(1, static_cast<int>(std::ceil(min_s / itp - 1e-9)));
}

// A 4-knot trajectory whose MIDDLE segment is exactly linear at `v`, so the commanded
// velocity there is a flat cruise and one tick of travel is exactly v*itp. Starts and
// ends at REST, so the CAPTURE splice into it is a pair of identical rest states and
// nothing in these tests depends on splice shape or on the capture gate.
struct CruiseTraj {
  std::array<std::int64_t, 4> times{};
  std::array<Vec6, 4> q{};
  std::array<Vec6, 4> qd{};
};

CruiseTraj make_cruise(double v) {
  CruiseTraj t{};
  const double r1 = 0.4;  // s, rest -> v
  const double r2 = 0.8;  // s, cruise
  const double r3 = 0.4;  // s, v -> rest
  t.times = {0, static_cast<std::int64_t>(r1 * 1e9),
             static_cast<std::int64_t>((r1 + r2) * 1e9),
             static_cast<std::int64_t>((r1 + r2 + r3) * 1e9)};
  // Segment 1's tangents give qd(s) = v*s (monotone, no overshoot); segment 2 satisfies
  // q2 - q1 == h*v exactly, which is what makes the Hermite there exactly linear.
  const double q1 = 0.5 * v * r1;
  const double q2 = q1 + v * r2;
  const double q3 = q2 + 0.5 * v * r3;
  t.q = {Vec6{}, Vec6{q1, 0, 0, 0, 0, 0}, Vec6{q2, 0, 0, 0, 0, 0}, Vec6{q3, 0, 0, 0, 0, 0}};
  t.qd = {Vec6{}, Vec6{v, 0, 0, 0, 0, 0}, Vec6{v, 0, 0, 0, 0, 0}, Vec6{}};
  return t;
}

Target traj_target(const CruiseTraj& t, std::uint64_t epoch) {
  Target g{};
  g.kind = TargetKind::kTrajectory;
  g.epoch = epoch;
  g.times_ns = t.times.data();
  g.q = t.q.data();
  g.qd = t.qd.data();
  g.n = 4;
  g.speed_scale = 1.0;
  return g;
}

}  // namespace

// ===========================================================================
// C3-i — the fault ramp does not re-command the pose it just commanded.
//
// enter_fault plans a qd-ramp from (q_cmd_, qd_cmd_) and used to sample it at tau=0,
// where QdEndBlend returns (q_end, qd_end) = the pose already commanded last tick. The
// wire therefore carried the same position twice and the commanded velocity notched to
// zero for one tick before the ramp began.
// ===========================================================================
TEST(FaultRamp, NoZeroVelocityTickAtEntryAndSameCoastDistance) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;
  const double v = 0.20;  // rad/s cruise
  TickCore tc(cfg);
  init_holding(tc, Vec6{});

  CruiseTraj traj = make_cruise(v);
  Target g = traj_target(traj, tc.epoch());
  RxSample rx0 = clean_rx(Vec6{});
  tc.tick(&rx0, &g);

  // Reach the flat middle segment.
  for (int i = 0; i < 80; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);
  ASSERT_NEAR(tc.qd_cmd()[0], v, 1e-6) << "must fault from a flat cruise, not a ramp";
  const double d_cruise = v * itp;
  const Vec6 q_pre = tc.q_cmd();
  const double qd_entry = tc.qd_cmd()[0];

  // Fault on the next tick (contact-stop rising edge -> enter_fault -> SAFE_FOLLOW).
  std::vector<double> cmd;
  {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    rx.gates.contact_stop_active = true;
    rx.contact_stop_status = 2;
    const Command c = tc.tick(&rx, nullptr);
    cmd.push_back(c.q_rad[0]);
  }
  ASSERT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  ASSERT_EQ(tc.fault(), FaultReason::CONTACT_STOP);

  // THE POINT. The fault-entry tick must ADVANCE, not repeat the previous position.
  const double d_entry = cmd.front() - q_pre[0];
  EXPECT_GT(d_entry, 0.8 * d_cruise)
      << "fault entry re-commanded the previous position: a one-tick velocity notch to "
         "zero, i.e. |qd|/itp of commanded acceleration at the worst possible moment";

  // Ramp out. kReanchor is the first tick with qd_cmd == 0 (the ramp's last sample is
  // still moving), and the measured pose is fed as the commanded one so the re-anchor
  // sits inside its deadband and cannot move anything afterwards.
  int ramp_ticks = 1;
  for (int i = 0; i < 40; ++i) {
    RxSample rx = clean_rx(tc.q_cmd());
    const Command c = tc.tick(&rx, nullptr);
    if (tc.qd_cmd()[0] == 0.0) break;
    cmd.push_back(c.q_rad[0]);
    ++ramp_ticks;
  }
  ASSERT_EQ(tc.mode(), Mode::SAFE_FOLLOW) << "SAFE_FOLLOW exits only via recover()";

  // No tick inside the ramp stands still, and the velocity decreases monotonically.
  double prev_d = d_entry;
  for (std::size_t k = 1; k < cmd.size(); ++k) {
    const double d = cmd[k] - cmd[k - 1];
    EXPECT_GT(d, 0.0) << "ramp tick " << k << " commanded no motion (velocity notch)";
    EXPECT_LE(d, prev_d + 1e-12) << "ramp velocity must decrease monotonically, tick " << k;
    prev_d = d;
  }

  // The ramp is one tick SHORTER than the blend's whole-tick span, because the sample at
  // tau=0 is no longer taken.
  const int n_blend = blend_ticks(cfg.rx_silence_qd_ramp_ms / 1000.0, itp);
  EXPECT_EQ(ramp_ticks, n_blend - 1)
      << "ramp spans blend_ticks-1 samples (tau = itp .. (n-1)*itp)";

  // AND THE COAST DISTANCE IS UNCHANGED. Closed form for the linear velocity ramp
  // qd(t) = (1 - t/T)*qd_entry, at the ramp's last sample tau = (n-1)*itp:
  //   q(tau) - q_pre = T*qd_entry*(s - s^2/2),  s = tau/T.
  // This value is identical with and without the fix — the fix removes a REPEATED
  // sample, it does not shorten the profile — which is exactly what must stay true, so
  // that a fault stop is not quietly traded for a longer one.
  const double T = static_cast<double>(n_blend) * itp;
  const double s = static_cast<double>(n_blend - 1) * itp / T;
  const double expect_coast = T * qd_entry * (s - 0.5 * s * s);
  EXPECT_NEAR(cmd.back() - q_pre[0], expect_coast, 1e-8)
      << "the fault stop must cover the same distance as before, not less and not more";
}

// ===========================================================================
// C2(a) — the CAPTURE splice does not re-command the anchor.
//
// generate_capture_path writes (q_cmd, qd_cmd) into knot 0 verbatim; playback used to
// start there, re-sending the pose already commanded last tick.
// ===========================================================================
TEST(CaptureSplice, FirstTickAdvancesFromTheAnchor) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;
  const double v = 0.20;
  TickCore tc(cfg);
  init_holding(tc, Vec6{});

  CruiseTraj traj = make_cruise(v);
  Target g = traj_target(traj, tc.epoch());
  RxSample rx0 = clean_rx(Vec6{});
  tc.tick(&rx0, &g);
  for (int i = 0; i < 80; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::TRAJECTORY);
  ASSERT_NEAR(tc.qd_cmd()[0], v, 1e-6);
  const double d_cruise = v * itp;
  const Vec6 q_anchor = tc.q_cmd();

  // Replan: knot 0 AHEAD of the commanded pose at the SAME velocity, which is the one
  // splice geometry measured to be monotone — a positive gap with matched endpoint
  // velocities gives a 2-knot splice. This test is about the splice HEAD, so it uses the
  // clean geometry deliberately and asserts nothing about acceptance.
  const double gap = 2.0 * d_cruise;
  std::array<std::int64_t, 2> times2{0, 800'000'000};
  std::array<Vec6, 2> q2{Vec6{q_anchor[0] + gap, 0, 0, 0, 0, 0},
                         Vec6{q_anchor[0] + gap + v * 0.8, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd2{Vec6{v, 0, 0, 0, 0, 0}, Vec6{v, 0, 0, 0, 0, 0}};
  Target g2{};
  g2.kind = TargetKind::kTrajectory;
  g2.epoch = tc.epoch();
  g2.times_ns = times2.data();
  g2.q = q2.data();
  g2.qd = qd2.data();
  g2.n = 2;
  g2.speed_scale = 1.0;

  // consume() arms CAPTURE and dispatch_mode runs it in the SAME tick, so the Command
  // returned here IS the first CAPTURE command.
  RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
  const Command c = tc.tick(&rx, &g2);
  ASSERT_EQ(tc.mode(), Mode::CAPTURE) << "splice accepted (gap well inside capture_tol)";

  // THE POINT.
  const double d_first = c.q_rad[0] - q_anchor[0];
  EXPECT_GT(d_first, 0.5 * d_cruise)
      << "first CAPTURE tick re-commanded the anchor: a one-tick velocity notch to zero "
         "at the head of every splice taken while the arm is moving";
  EXPECT_LT(d_first, 2.0 * d_cruise) << "and it must not double-step either";

  // Sanity: the replan still completes.
  bool done = false;
  for (int i = 0; i < 400 && !done; ++i) {
    RxSample r = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&r, nullptr);
    if (drain_for(tc, EventType::kMotionDone)) done = true;
  }
  EXPECT_TRUE(done) << "replan runs to DONE";
}

// ===========================================================================
// C3-ii — the trajectory -> qd_end blend handoff is one tick of travel at every grid
// phase.
//
// traj_end_tau_ns_ is generally not a whole number of ticks, so the last playing tick
// covered only a fraction of a tick and starting the blend at tau=0 re-commanded q_last
// on top of it. The duration below is DELIBERATELY not a tick multiple.
// ===========================================================================
TEST(EndBlend, HandoffKeepsOneTickOfTravelOffGrid) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;
  const double v = 0.20;  // > settle_vel_eps, so the blend engages
  ASSERT_GT(v, cfg.tick.settle_vel_eps_rad_s) << "terminal velocity must trigger the blend";

  const std::int64_t itp_ns = static_cast<std::int64_t>(std::llround(itp * 1e9));
  const std::int64_t t1 = 200'000'000;
  const std::int64_t t2 = 333'000'000;  // 41.625 ticks — off grid on purpose
  ASSERT_NE(t2 % itp_ns, 0) << "this test is only meaningful for an off-grid duration";

  const double h1 = static_cast<double>(t1) / 1e9;
  const double h2 = static_cast<double>(t2 - t1) / 1e9;
  const double q1 = 0.5 * v * h1;  // qd(s) = v*s on segment 1
  const double q2 = q1 + v * h2;   // exactly linear on segment 2
  std::array<std::int64_t, 3> times{0, t1, t2};
  std::array<Vec6, 3> q{Vec6{}, Vec6{q1, 0, 0, 0, 0, 0}, Vec6{q2, 0, 0, 0, 0, 0}};
  std::array<Vec6, 3> qd{Vec6{}, Vec6{v, 0, 0, 0, 0, 0}, Vec6{v, 0, 0, 0, 0, 0}};
  Target g{};
  g.kind = TargetKind::kTrajectory;
  g.times_ns = times.data();
  g.q = q.data();
  g.qd = qd.data();
  g.n = 3;
  g.speed_scale = 1.0;

  TickCore tc(cfg);
  init_holding(tc, Vec6{});
  g.epoch = tc.epoch();
  RxSample rx0 = clean_rx(Vec6{});
  Command c = tc.tick(&rx0, &g);

  std::vector<double> cmd{c.q_rad[0]};
  bool done = false;
  for (int i = 0; i < 400 && !done; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    c = tc.tick(&rx, nullptr);
    cmd.push_back(c.q_rad[0]);
    if (drain_for(tc, EventType::kMotionDone)) done = true;
  }
  ASSERT_TRUE(done) << "trajectory + blend + settle completes";

  // Per-tick commanded travel, and the last tick still at full cruise. Everything after
  // that index is the blend and then the settle hold, so it must only ever slow down.
  const double d_cruise = v * itp;
  std::vector<double> d(cmd.size() - 1);
  for (std::size_t k = 1; k < cmd.size(); ++k) {
    d[k - 1] = cmd[k] - cmd[k - 1];
  }
  int cruise_end = -1;
  int at_cruise = 0;
  for (std::size_t i = 0; i < d.size(); ++i) {
    if (d[i] >= 0.999 * d_cruise) {
      cruise_end = static_cast<int>(i);
      ++at_cruise;
    }
  }
  ASSERT_GT(at_cruise, 10) << "the cruise segment must actually have been observed";
  ASSERT_LT(cruise_end + 1, static_cast<int>(d.size())) << "and must be followed by the blend";

  // THE POINT (1). The handoff tick itself must carry a whole tick of travel. Off grid
  // it used to carry only (traj_end - tau_last)/itp of one — 0.625 for this duration —
  // because the trajectory's last tick stopped short of the end and the blend then
  // re-commanded the terminal knot instead of covering the remainder.
  const double d_handoff = d[static_cast<std::size_t>(cruise_end) + 1];
  EXPECT_GE(d_handoff, 0.8 * d_cruise)
      << "trajectory->blend handoff under-stepped (d/d_cruise = " << (d_handoff / d_cruise)
      << "): the grid-phase residue was dropped and the terminal knot re-commanded";

  // THE POINT (2). And from the end of cruise onward the commanded velocity only ever
  // decreases. The dropped residue was not merely a small step — it was a DIP followed
  // by a recovery (0.625 then 0.875 of a tick here), which is a commanded acceleration
  // that reverses sign inside a deceleration.
  for (std::size_t i = static_cast<std::size_t>(cruise_end) + 2; i < d.size(); ++i) {
    EXPECT_LE(d[i], d[i - 1] + 1e-15)
        << "commanded velocity increased at tick " << (i + 1)
        << " while decelerating to rest (d/d_cruise = " << (d[i] / d_cruise) << " after "
        << (d[i - 1] / d_cruise) << ")";
  }
}

// ===========================================================================
// C5-i (a) — while TX is parked, the anchor tracks the measured pose.
// ===========================================================================
TEST(RxSilentPark, AnchorTracksMeasuredWhileTxIsOff) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  init_holding(tc, Vec6{});

  const int park =
      static_cast<int>(std::ceil(cfg.rx_silent_park_ms / 1000.0 / cfg.tick.itp_s)) + 3;
  Command c{};
  for (int i = 0; i < park; ++i) {
    c = tc.tick(nullptr, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::RX_SILENT);
  ASSERT_FALSE(c.tx) << "TX parked at RX_SILENT";

  // RX comes back but nobody has called recover(), so the park continues. The arm moved
  // during the outage; the anchor must follow it while nothing is transmitted.
  const double drift = 0.10;
  const Vec6 q_far{drift, 0, 0, 0, 0, 0};
  const double slew_step = cfg.tick.slew_factor * cfg.tick.limits.v[0] * cfg.tick.itp_s;
  const int need = static_cast<int>(std::ceil(drift / slew_step)) + 4;
  for (int i = 0; i < need; ++i) {
    RxSample rx = clean_rx(q_far);
    c = tc.tick(&rx, nullptr);
    EXPECT_FALSE(c.tx) << "the park must not un-park without recover()";
  }
  ASSERT_EQ(tc.mode(), Mode::RX_SILENT);

  // THE POINT.
  EXPECT_NEAR(tc.q_cmd()[0], drift, 1e-9)
      << "the parked anchor stayed at the pre-silence pose, so the first transmitted "
         "position after recovery would step to somewhere the arm has left";
}

// ===========================================================================
// C5-i (b) — the first transmitted position after recover() IS the measured pose, even
// when the drift exceeds what the dwell's slew-bounded walk can cover.
// ===========================================================================
TEST(RxSilentRecover, FirstTransmittedPositionIsTheMeasuredPose) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  init_holding(tc, Vec6{});

  const int park =
      static_cast<int>(std::ceil(cfg.rx_silent_park_ms / 1000.0 / cfg.tick.itp_s)) + 3;
  for (int i = 0; i < park; ++i) {
    tc.tick(nullptr, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::RX_SILENT);

  // Drift deliberately LARGER than dwell_ticks * slew_step, so converging by the slew
  // walk alone cannot get there and only the dwell's own re-anchor can.
  const double slew_step = cfg.tick.slew_factor * cfg.tick.limits.v[0] * cfg.tick.itp_s;
  const int dwell =
      static_cast<int>(std::ceil(cfg.antiflap_dwell_ms / 1000.0 / cfg.tick.itp_s - 1e-9));
  const double drift = 1.25 * static_cast<double>(dwell) * slew_step;
  const Vec6 q_far{drift, 0, 0, 0, 0, 0};

  tc.request_recover();
  bool saw_tx = false;
  Command first_tx{};
  for (int i = 0; i < dwell + 20 && !saw_tx; ++i) {
    RxSample rx = clean_rx(q_far);
    const Command c = tc.tick(&rx, nullptr);
    if (c.tx) {
      saw_tx = true;
      first_tx = c;
    }
  }
  ASSERT_TRUE(saw_tx) << "recover() completed and un-parked TX";
  EXPECT_EQ(tc.mode(), Mode::HOLD);
  EXPECT_EQ(tc.fault(), FaultReason::NONE);

  // THE POINT.
  EXPECT_NEAR(first_tx.q_rad[0], drift, 1e-12)
      << "the first transmitted position after leaving the park must be the measured "
         "pose, not the pre-silence anchor";
}

// ===========================================================================
// C5-ii — the SAFE_FOLLOW re-anchor holds when the measured pose is STALE.
//
// have_meas_ latches true on the first packet and is never cleared, so on its own the
// existing guard never fired and the re-anchor walked the commanded pose toward a
// pre-gap measurement for the whole gap.
// ===========================================================================
TEST(SafeFollowReanchor, HoldsWhileTheMeasurementIsStale) {
  RtCoreConfig cfg;
  TickCore tc(cfg);
  init_holding(tc, Vec6{});

  // Fault from HOLD: qd_cmd_ == 0, so the ramp is inactive and kReanchor starts at once.
  {
    RxSample rx = clean_rx(Vec6{});
    rx.gates.contact_stop_active = true;
    rx.contact_stop_status = 2;
    tc.tick(&rx, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::SAFE_FOLLOW);

  // Fresh measurements well outside the deadband: the re-anchor walks, as it should.
  const Vec6 q_far{0.5, 0, 0, 0, 0, 0};
  ASSERT_GT(q_far[0], cfg.safe_follow_deadband_rad);
  const double max_step = cfg.safe_follow_rate_rad_s * cfg.tick.itp_s;
  for (int i = 0; i < 6; ++i) {
    RxSample rx = clean_rx(q_far);
    tc.tick(&rx, nullptr);
  }
  const double walked = tc.q_cmd()[0];
  ASSERT_GT(walked, 0.5 * max_step) << "the re-anchor must be walking before we stall RX";
  ASSERT_GT(std::abs(q_far[0] - walked), cfg.safe_follow_deadband_rad)
      << "and must still be outside the deadband, so a hold is a real decision";

  // Now stall RX, staying under the 500 ms park. have_meas_ is still true and q_meas_
  // still holds q_far.
  const int silent = 10;
  ASSERT_LT(static_cast<double>(silent) * cfg.tick.itp_s * 1000.0, cfg.rx_silent_park_ms);
  for (int i = 0; i < silent; ++i) {
    tc.tick(nullptr, nullptr);
  }
  ASSERT_EQ(tc.mode(), Mode::SAFE_FOLLOW);

  // THE POINT.
  EXPECT_NEAR(tc.q_cmd()[0], walked, 1e-15)
      << "the re-anchor kept walking against a measurement taken before the gap: "
         "commanded motion toward where the arm no longer is";
}
