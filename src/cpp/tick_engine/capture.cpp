// SPDX-License-Identifier: Apache-2.0
//
// CAPTURE-or-REJECT splice generator — see capture.hpp.

#include "tick_engine/capture.hpp"

#include <algorithm>
#include <cmath>

namespace airo_fanuc::tick_engine {

void generate_capture_path(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& q0, const Vec6& qd0,
                           const TickEngineConfig& cfg, CapturePath& out) {
  out.clear();

  // Local Ruckig<6> — StandardVector = std::array, so stack-allocated / no heap.
  // Fresh instance per call guarantees determinism (no carried internal state).
  ruckig::Ruckig<kNumJoints> otg(cfg.itp_s);
  ruckig::InputParameter<kNumJoints> inp;
  ruckig::OutputParameter<kNumJoints> ruck_out;

  inp.control_interface = ruckig::ControlInterface::Position;
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    inp.max_velocity[j] = cfg.capture_rate_rad_s;                  // 15°/s per joint
    inp.max_acceleration[j] = cfg.stop_scale_va * cfg.limits.a[j];  // brake-class
    inp.max_jerk[j] = cfg.stop_scale_j * cfg.limits.j[j];           // brake-class
  }
  inp.current_position = q_cmd;
  inp.current_velocity = qd_cmd;
  inp.current_acceleration = Vec6{};
  inp.target_position = q0;
  inp.target_velocity = qd0;
  inp.target_acceleration = Vec6{};

  // knot[0] = the commanded anchor exactly; knot[0] accel = the seeded
  // current_acceleration (0) so the CAPTURE brake-seed is well-defined from the
  // very first knot.
  out.q[0] = q_cmd;
  out.qd[0] = qd_cmd;
  out.qdd[0] = inp.current_acceleration;
  out.count = 1;

  for (int k = 1; k < CapturePath::kMaxKnots; ++k) {
    const ruckig::Result r = otg.update(inp, ruck_out);
    out.result = r;
    if (r != ruckig::Result::Working && r != ruckig::Result::Finished) {
      // Ruckig error (e.g. infeasible target) — stop; caller faults / retries.
      return;
    }
    out.q[static_cast<std::size_t>(k)] = ruck_out.new_position;
    out.qd[static_cast<std::size_t>(k)] = ruck_out.new_velocity;
    out.qdd[static_cast<std::size_t>(k)] = ruck_out.new_acceleration;
    out.count = k + 1;
    if (r == ruckig::Result::Finished) {
      out.finished = true;
      // Finished means ruck_out.time is past the profile's duration; the difference is
      // how far THIS knot sits beyond (q0, qd0) in the trajectory's own time, which is
      // what playback needs to resume one tick on. Clamped to [0, itp_s]
      // because that is the interval Ruckig's own Finished test bounds it to, so a
      // clamp can only ever absorb float dust.
      const double residue_s =
          std::clamp(ruck_out.time - ruck_out.trajectory.get_duration(), 0.0, cfg.itp_s);
      out.residue_ns = static_cast<std::int64_t>(std::llround(residue_s * 1e9));
      return;
    }
    ruck_out.pass_to_input(inp);
  }
  // Ran out of buffer before Finished.
  out.overflow = true;
}

namespace {

// Duration of a jerk-limited change of |dv| in one joint's velocity, bounded by (a, j):
// triangular in acceleration while |dv| < a²/j (a is never reached), trapezoidal above
// it. The standard S-curve result, under the SAME envelope generate_capture_path hands
// Ruckig above — which is why it predicts THAT generator and not some other one.
double shed_time(double dv, double a, double j) {
  if (!(a > 0.0) || !(j > 0.0) || !(dv > 0.0)) {
    return 0.0;
  }
  return (dv < a * a / j) ? 2.0 * std::sqrt(dv / j) : dv / a + a / j;
}

}  // namespace

CaptureGate capture_gate(const Vec6& q_cmd, const Vec6& qd_cmd, const Vec6& q0, const Vec6& qd0,
                         const TickEngineConfig& cfg) {
  CaptureGate g{};
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    // (1) endpoint gap vs the capture window.
    if (std::abs(q_cmd[j] - q0[j]) > cfg.capture_tol_rad) {
      g.tol_exceeded = true;
      g.reject = true;
    }

    // (1b) arrival velocity vs the rate the splice itself may run at. capture_rate_rad_s
    // is the generator's own max_velocity, and Ruckig refuses a target_velocity above
    // max_velocity as invalid input — so without this test the gate passes a submission
    // the generator then fails, and a failed generation can only be reported as INTERNAL.
    // Testing it here is what makes the arrival rate a typed REJECTED_START_MISMATCH.
    if (std::abs(qd0[j]) > cfg.capture_rate_rad_s) {
      g.reject_mask |= (1u << static_cast<unsigned>(j));
      g.reject = true;
    }

    // (2) can the window absorb the velocity change? Mean speed over a monotone velocity
    // change is (|v0| + |v1|)/2, so the travel it costs is that times its duration. Zero
    // whenever the endpoint velocities match — including the case where both are zero,
    // which is every submission out of HOLD, and the case where both are large and equal,
    // which is a continuation replan.
    const double a_b = cfg.stop_scale_va * cfg.limits.a[j];
    const double j_b = cfg.stop_scale_j * cfg.limits.j[j];
    const double t_shed = shed_time(std::abs(qd0[j] - qd_cmd[j]), a_b, j_b);
    g.shed_travel[j] = 0.5 * (std::abs(qd_cmd[j]) + std::abs(qd0[j])) * t_shed;

    if (g.shed_travel[j] > cfg.capture_tol_rad) {
      g.reject_mask |= (1u << static_cast<unsigned>(j));
      g.reject = true;
    }
  }
  return g;
}

}  // namespace airo_fanuc::tick_engine
