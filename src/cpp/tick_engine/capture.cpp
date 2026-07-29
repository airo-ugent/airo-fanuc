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
      return;
    }
    ruck_out.pass_to_input(inp);
  }
  // Ran out of buffer before Finished.
  out.overflow = true;
}

bool capture_would_reject(const Vec6& q_cmd, const Vec6& q0, const TickEngineConfig& cfg) {
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    if (std::abs(q_cmd[j] - q0[j]) > cfg.capture_tol_rad) {
      return true;
    }
  }
  return false;
}

}  // namespace airo_fanuc::tick_engine
