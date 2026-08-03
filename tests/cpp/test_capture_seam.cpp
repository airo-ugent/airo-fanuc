// SPDX-License-Identifier: Apache-2.0
//
// Unit test — the CAPTURE -> TRAJECTORY seam.
//
// The splice's knot grid is whole ticks and its Ruckig duration is not, so its last
// knot lands PAST the trajectory start by a sub-tick residue. Two assertions:
//
//   (1) the generator reports that residue, and it accounts exactly for where the
//       terminal knot sits — q[count-1] == q0 + qd0*residue;
//   (2) the handoff tick carries a WHOLE tick of commanded travel at every grid phase,
//       which is only true if playback resumes at tau = itp + residue.
//
// (2) is asserted on the SEQUENCE of commanded positions returned by TickCore::tick —
// the quantity that reaches the wire — because the wire carries positions only and the
// controller infers velocity by differencing them. A knot's qd field being right is
// not the same thing as the position stream being continuous.
//
// The grid phases used below are gaps that are NOT whole multiples of a tick's travel,
// because those are the phases where the residue is a large and stable fraction of a
// tick. A gap that IS a whole multiple puts the profile's duration within float dust of
// a tick boundary, so its residue is either ~0 or a whole tick depending on which side
// the dust falls — a real case, but not one to assert on. Every expectation is derived
// from `cfg`.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"
#include "tick_engine/capture.hpp"

using namespace airo_fanuc::rt_core;
using airo_fanuc::tick_engine::CapturePath;
using airo_fanuc::tick_engine::generate_capture_path;
using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::TickEngineConfig;
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

// A 4-knot trajectory whose MIDDLE segment is exactly linear at `v` (segment 1's
// tangents give qd(s) = v*s, and q2 - q1 == h*v makes segment 2 exactly linear), so the
// commanded velocity there is a flat cruise and one tick of travel is exactly v*itp.
struct CruiseTraj {
  std::array<std::int64_t, 4> times{};
  std::array<Vec6, 4> q{};
  std::array<Vec6, 4> qd{};
};

CruiseTraj make_cruise(double v) {
  CruiseTraj t{};
  const double r1 = 0.4, r2 = 1.2, r3 = 0.4;  // s: rest->v, cruise, v->rest
  t.times = {0, static_cast<std::int64_t>(r1 * 1e9),
             static_cast<std::int64_t>((r1 + r2) * 1e9),
             static_cast<std::int64_t>((r1 + r2 + r3) * 1e9)};
  const double q1 = 0.5 * v * r1;
  const double q2 = q1 + v * r2;
  const double q3 = q2 + 0.5 * v * r3;
  t.q = {Vec6{}, Vec6{q1, 0, 0, 0, 0, 0}, Vec6{q2, 0, 0, 0, 0, 0}, Vec6{q3, 0, 0, 0, 0, 0}};
  t.qd = {Vec6{}, Vec6{v, 0, 0, 0, 0, 0}, Vec6{v, 0, 0, 0, 0, 0}, Vec6{}};
  return t;
}

// Ratio of the handoff tick's commanded travel to a cruise tick's, for a replan whose
// first knot sits `gap_ticks` ticks of travel ahead at the SAME velocity — the geometry
// a monotone splice needs, and at least a whole tick of it, which is the least a fixed
// target can be ahead of a moving anchor and still be reachable without leaving and
// coming back. 1.0 means the seam is continuous; 0.0 means the handoff tick stood still
// while the stream was moving.
double handoff_ratio(double v, double gap_ticks) {
  RtCoreConfig cfg;
  const double itp = cfg.tick.itp_s;
  const double d_cruise = v * itp;
  TickCore tc(cfg);
  tc.begin_streaming(Vec6{});
  {
    RxSample rx = clean_rx(Vec6{});
    tc.tick(&rx, nullptr);
  }

  CruiseTraj traj = make_cruise(v);
  Target g{};
  g.kind = TargetKind::kTrajectory;
  g.epoch = tc.epoch();
  g.times_ns = traj.times.data();
  g.q = traj.q.data();
  g.qd = traj.qd.data();
  g.n = 4;
  g.speed_scale = 1.0;
  RxSample rx0 = clean_rx(Vec6{});
  tc.tick(&rx0, &g);

  // Into the flat middle segment.
  for (int i = 0; i < 80; ++i) {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    tc.tick(&rx, nullptr);
  }
  EXPECT_EQ(tc.mode(), Mode::TRAJECTORY);
  EXPECT_NEAR(tc.qd_cmd()[0], v, 1e-6) << "the replan must be taken from a flat cruise";

  // The replan: a 2-knot constant-velocity segment starting `gap` ahead at velocity v.
  const double gap = gap_ticks * d_cruise;
  const double q_start = tc.q_cmd()[0] + gap;
  std::array<std::int64_t, 2> times2{0, 1'000'000'000};
  std::array<Vec6, 2> q2{Vec6{q_start, 0, 0, 0, 0, 0}, Vec6{q_start + v * 1.0, 0, 0, 0, 0, 0}};
  std::array<Vec6, 2> qd2{Vec6{v, 0, 0, 0, 0, 0}, Vec6{v, 0, 0, 0, 0, 0}};
  Target g2{};
  g2.kind = TargetKind::kTrajectory;
  g2.epoch = tc.epoch();
  g2.times_ns = times2.data();
  g2.q = q2.data();
  g2.qd = qd2.data();
  g2.n = 2;
  g2.speed_scale = 1.0;

  // consume() arms CAPTURE and dispatch runs it in the same tick, so this Command is
  // the first CAPTURE command. Record the whole run through the seam.
  std::vector<double> cmd{tc.q_cmd()[0]};
  {
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    const Command c = tc.tick(&rx, &g2);
    cmd.push_back(c.q_rad[0]);
  }
  EXPECT_EQ(tc.mode(), Mode::CAPTURE) << "gap is well inside capture_tol";

  int seam = -1;
  for (int i = 0; i < 40; ++i) {
    const bool was_capture = (tc.mode() == Mode::CAPTURE);
    RxSample rx = clean_rx(tc.q_cmd(), tc.qd_cmd());
    const Command c = tc.tick(&rx, nullptr);
    cmd.push_back(c.q_rad[0]);
    if (was_capture && tc.mode() == Mode::TRAJECTORY) {
      seam = static_cast<int>(cmd.size()) - 1;  // the first TRAJECTORY-dispatched sample
      break;
    }
  }
  EXPECT_GT(seam, 0) << "the splice never handed over to trajectory playback";
  if (seam <= 0) return 0.0;
  return (cmd[static_cast<std::size_t>(seam)] - cmd[static_cast<std::size_t>(seam) - 1]) / d_cruise;
}

}  // namespace

// ===========================================================================
// The generator reports the sub-tick residue, and it explains the terminal knot.
// ===========================================================================
TEST(CaptureSeam, ResidueAccountsForTheTerminalKnot) {
  TickEngineConfig cfg;
  const double itp = cfg.itp_s;
  const std::int64_t itp_ns = static_cast<std::int64_t>(std::llround(itp * 1e9));

  for (double vf : {1.0, 0.6}) {
    const double v = vf * cfg.capture_rate_rad_s;
    for (double gap_ticks : {0.3, 0.7, 1.2, 1.5, 2.5, 3.3, 7.4}) {
      const double gap = gap_ticks * v * itp;
      const Vec6 qd_cmd{v, 0, 0, 0, 0, 0};
      const Vec6 q0{gap, 0, 0, 0, 0, 0};
      CapturePath path;
      generate_capture_path(Vec6{}, qd_cmd, q0, qd_cmd, cfg, path);
      ASSERT_TRUE(path.ok()) << "matched-velocity splice, gap " << gap_ticks << " ticks";

      // The interval, which is what Ruckig's own Finished test bounds it to.
      EXPECT_GT(path.residue_ns, 0) << "gap " << gap_ticks << " ticks at " << vf << "x rate";
      EXPECT_LE(path.residue_ns, itp_ns) << "gap " << gap_ticks << " ticks";

      // THE LOAD-BEARING PROPERTY. The residue accounts for where the terminal knot
      // sits: the target advanced by the residue at the target velocity. That is what
      // makes tau = itp + residue the right place to resume — the knot IS the
      // trajectory's own state at its time `residue`, so the next sample is one tick on.
      const double residue_s = static_cast<double>(path.residue_ns) * 1e-9;
      const double q_last = path.q[static_cast<std::size_t>(path.count - 1)][0];
      EXPECT_NEAR(q_last, q0[0] + qd_cmd[0] * residue_s, 1e-9)
          << "terminal knot is not (q0 + qd0*residue), gap " << gap_ticks << " ticks";

      // A gap off the tick grid leaves a substantial residue, which is what makes this
      // worth carrying: it is not a rounding term.
      EXPECT_GT(static_cast<double>(path.residue_ns) / static_cast<double>(itp_ns), 0.1)
          << "gap " << gap_ticks << " ticks sits off the tick grid; residue should be "
             "a sizeable fraction of a tick";

      // The other joints are at rest and must not have moved.
      for (int j = 1; j < kNumJoints; ++j) {
        EXPECT_NEAR(path.q[static_cast<std::size_t>(path.count - 1)][static_cast<std::size_t>(j)],
                    0.0, 1e-12);
      }
    }
  }
}

// A rest-to-rest splice has no residue to report in position terms, but the field must
// still be well defined and inside its interval.
TEST(CaptureSeam, ResidueIsBoundedForARestSplice) {
  TickEngineConfig cfg;
  const std::int64_t itp_ns = static_cast<std::int64_t>(std::llround(cfg.itp_s * 1e9));
  const Vec6 q0{0.05, -0.02, 0.03, 0.0, 0.0, 0.0};
  CapturePath path;
  generate_capture_path(Vec6{}, Vec6{}, q0, Vec6{}, cfg, path);
  ASSERT_TRUE(path.ok());
  EXPECT_GT(path.residue_ns, 0);
  EXPECT_LE(path.residue_ns, itp_ns);
  for (int j = 0; j < kNumJoints; ++j) {
    EXPECT_NEAR(path.q[static_cast<std::size_t>(path.count - 1)][static_cast<std::size_t>(j)],
                q0[static_cast<std::size_t>(j)], 1e-9)
        << "at rest the terminal knot IS the target, joint " << j;
  }
}

// ===========================================================================
// THE POINT. The handoff tick carries a whole tick of travel at every grid phase.
//
// The velocity the stream carries into the seam is v; the velocity it carries out is
// (handoff travel)/itp. A ratio of r means a commanded-acceleration step of
// (r-1)*v/itp, which at capture_rate and r = 0 is 19.5x the brake acceleration clamp.
// ===========================================================================
TEST(CaptureSeam, HandoffCarriesOneTickAtEveryGridPhase) {
  RtCoreConfig cfg;
  for (double vf : {0.95, 0.6}) {
    const double v = vf * cfg.tick.capture_rate_rad_s;
    for (double gap_ticks : {1.2, 1.5, 1.7, 2.5, 3.3, 7.4}) {
      const double r = handoff_ratio(v, gap_ticks);
      EXPECT_NEAR(r, 1.0, 0.02)
          << "at " << vf << "x capture_rate with a " << gap_ticks
          << "-tick gap the handoff carried " << r << " of a tick, i.e. a commanded "
          << "acceleration of " << ((r - 1.0) * v / cfg.tick.itp_s) << " rad/s^2 against a "
          << (cfg.tick.stop_scale_va * cfg.tick.limits.a[0]) << " rad/s^2 brake clamp: the "
             "splice's sub-tick terminal residue was dropped and the seam tick re-covered "
             "ground the splice had already covered";
    }
  }
}
