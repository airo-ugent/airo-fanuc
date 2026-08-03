// SPDX-License-Identifier: Apache-2.0
//
// pybind11 bindings for airo_fanuc._core.
//
// Binds two layers: the Stream Motion packet codec (so the Python `wire.py`
// oracle can byte-compare the C++ encoder/decoder against it) AND the real-time
// StreamCore + RtCoreConfig + capture-path generator.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ruckig/ruckig.hpp>

#include "codec/codec.hpp"
#include "rt_core/realtime_core.hpp"
#include "rt_core/rt_core_config.hpp"
#include "rt_core/snapshot.hpp"
#include "tick_engine/capture.hpp"
#include "tick_engine/tick_engine_config.hpp"

#ifndef AIRO_FANUC_CORE_VERSION
#define AIRO_FANUC_CORE_VERSION "0.0.0+unknown"
#endif

namespace py = pybind11;

namespace {

py::bytes py_encode_command_packet(std::uint32_t sequence_no, bool is_last_command, std::uint8_t do_motn_ctrl,
                                   const std::vector<double>& command_pos_deg) {
  if (command_pos_deg.size() != static_cast<std::size_t>(airo_fanuc::codec::kMaxAxes)) {
    throw std::invalid_argument("pos_deg must have length " + std::to_string(airo_fanuc::codec::kMaxAxes) +
                                " (got " + std::to_string(command_pos_deg.size()) + ")");
  }
  std::array<double, 9> pos{};
  for (std::size_t i = 0; i < pos.size(); ++i) {
    pos[i] = command_pos_deg[i];
  }
  const auto buf = airo_fanuc::codec::encode_command_packet(sequence_no, is_last_command, do_motn_ctrl, pos);
  return py::bytes(reinterpret_cast<const char*>(buf.data()), buf.size());
}

// Expose the deterministic CAPTURE-splice generator so the Python driver can
// collision-check the EXACT knots the RT core will execute ("the checked path IS
// the executed path" — see the capture.hpp header note and
// docs/invariants.md, "Collision-check hook"). The caller passes the same
// RtCoreConfig the core was built from, so the splice is synthesized under the
// arm's own limits rather than the C++ fallback defaults.
py::dict py_generate_capture_path(const std::vector<double>& q_cmd, const std::vector<double>& qd_cmd,
                                  const std::vector<double>& q0, const std::vector<double>& qd0,
                                  const std::optional<airo_fanuc::rt_core::RtCoreConfig>& config) {
  namespace te = airo_fanuc::tick_engine;
  auto to_vec6 = [](const std::vector<double>& v, const char* name) {
    if (v.size() != static_cast<std::size_t>(te::kNumJoints)) {
      throw std::invalid_argument(std::string(name) + " must have length " +
                                  std::to_string(te::kNumJoints));
    }
    te::Vec6 out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
      out[i] = v[i];
    }
    return out;
  };
  const te::Vec6 qc = to_vec6(q_cmd, "q_cmd");
  const te::Vec6 qdc = to_vec6(qd_cmd, "qd_cmd");
  const te::Vec6 q0v = to_vec6(q0, "q0");
  const te::Vec6 qd0v = to_vec6(qd0, "qd0");
  // The tick-engine knobs MUST be the ones the RT core will run with, not the
  // shipped defaults: a caller that changes itp_s or a limit and then checks a
  // default-configured splice would be collision-checking a path the core never
  // executes. Passing no config means "the defaults", which is what a core
  // constructed from a default RtCoreConfig runs.
  const te::TickEngineConfig cfg = config ? config->tick : te::TickEngineConfig{};

  py::dict d;
  // ONE evaluation of the gate, exported whole. The Python typed error is FORMATTED from
  // these numbers rather than recomputed, so no second derivation of the feasibility
  // arithmetic exists to drift from tick_engine/capture.cpp.
  const te::CaptureGate gate = te::capture_gate(qc, qdc, q0v, qd0v, cfg);
  d["would_reject"] = gate.reject;
  d["tol_exceeded"] = gate.tol_exceeded;
  py::list reject_joints;
  for (int j = 0; j < te::kNumJoints; ++j) {
    if (((gate.reject_mask >> static_cast<unsigned>(j)) & 1u) != 0u) {
      reject_joints.append(j);
    }
  }
  d["reject_joints"] = reject_joints;
  d["shed_travel"] = std::vector<double>(gate.shed_travel.begin(), gate.shed_travel.end());

  auto path = std::make_unique<te::CapturePath>();
  te::generate_capture_path(qc, qdc, q0v, qd0v, cfg, *path);
  d["count"] = path->count;
  d["finished"] = path->finished;
  d["overflow"] = path->overflow;
  d["residue_ns"] = path->residue_ns;
  py::list q_knots;
  py::list qd_knots;
  for (int k = 0; k < path->count; ++k) {
    q_knots.append(std::vector<double>(path->q[k].begin(), path->q[k].end()));
    qd_knots.append(std::vector<double>(path->qd[k].begin(), path->qd[k].end()));
  }
  d["q"] = q_knots;
  d["qd"] = qd_knots;
  return d;
}

// Offline point-to-point joint plan, for `FanucDriver.move_j`.
//
// The RT core plays a submitted trajectory back with cubic Hermite between the knots
// it is given (see tick_engine/hermite.hpp) — it never re-times them. So a MoveJ is
// entirely a matter of producing feasible knots, and this produces them with the SAME
// Ruckig version, the SAME `Ruckig<6>` template and the SAME `cfg.limits` the brake
// and servo run under. That is the `generate_capture_path` argument applied to the
// planning side: a profile shaped here is one the tick engine can pass through
// unclipped, rather than one shaped against a second, separately-maintained envelope.
//
// `max_velocity_rad_s` is the LEADING-AXIS speed: it caps every joint, and Ruckig's
// default time-synchronization then lands them all together, so the joint with the
// furthest to travel runs at this speed and the rest scale down. <= 0 means "the
// config's own velocity limits". `accel_scale` / `jerk_scale` are FRACTIONS of
// `cfg.limits.a` / `cfg.limits.j` (airo_fanuc.controller_facts.MOVEJ_LIMIT_SCALE_A/_J).
py::dict py_plan_joint_move(const std::vector<double>& q0, const std::vector<double>& qd0,
                            const std::vector<double>& q_target,
                            const std::optional<airo_fanuc::rt_core::RtCoreConfig>& config,
                            double max_velocity_rad_s, double accel_scale, double jerk_scale,
                            const std::vector<double>& qdd0) {
  namespace te = airo_fanuc::tick_engine;
  auto to_vec6 = [](const std::vector<double>& v, const char* name) {
    if (v.size() != static_cast<std::size_t>(te::kNumJoints)) {
      throw std::invalid_argument(std::string(name) + " must have length " +
                                  std::to_string(te::kNumJoints));
    }
    te::Vec6 out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
      out[i] = v[i];
    }
    return out;
  };
  const te::Vec6 p0 = to_vec6(q0, "q0");
  const te::Vec6 v0 = to_vec6(qd0, "qd0");
  const te::Vec6 pt = to_vec6(q_target, "q_target");
  // Same rule as generate_capture_path: pass the config the core was built from, or
  // the plan is shaped by the synthetic C++ fallback envelope instead of the arm's.
  const te::TickEngineConfig cfg = config ? config->tick : te::TickEngineConfig{};

  // Both are FRACTIONS of cfg.limits, so 1.0 is the ceiling, not just the default. The
  // core replays these knots with cubic Hermite and never re-times them, and the only
  // per-tick clip on the command is positional (an acceleration cap there is banned —
  // slew.hpp), so a profile planned above the arm's a/j reaches the wire unreshaped.
  // NaN fails both bounds.
  if (!(accel_scale > 0.0 && accel_scale <= 1.0) || !(jerk_scale > 0.0 && jerk_scale <= 1.0)) {
    throw std::invalid_argument(
        "accel_scale and jerk_scale must be in (0, 1]: they are fractions of the config's "
        "acceleration / jerk limits");
  }
  if (!(cfg.itp_s > 0.0)) {
    throw std::invalid_argument("config.itp_s must be > 0");
  }

  ruckig::Ruckig<te::kNumJoints> otg(cfg.itp_s);
  ruckig::InputParameter<te::kNumJoints> inp;
  ruckig::Trajectory<te::kNumJoints> traj;

  inp.control_interface = ruckig::ControlInterface::Position;
  inp.synchronization = ruckig::Synchronization::Time;  // the leading-axis semantics
  inp.current_position = p0;
  inp.current_velocity = v0;
  // Seeded from the caller's own commanded acceleration when it supplies one (empty =
  // zeros). A plan anchored at the commanded state but seeded at zero acceleration starts
  // with a curvature the arm does not have, and the capture splice is then asked to absorb
  // that difference; passing the snapshot's qdd_cmd makes the plan continue the motion
  // instead. Ruckig validates the TARGET state against the limits, not this one.
  inp.current_acceleration = qdd0.empty() ? te::Vec6{} : to_vec6(qdd0, "qdd0");
  inp.target_position = pt;
  inp.target_velocity = te::Vec6{};
  inp.target_acceleration = te::Vec6{};
  for (std::size_t j = 0; j < static_cast<std::size_t>(te::kNumJoints); ++j) {
    double v_max = cfg.limits.v[j];
    if (max_velocity_rad_s > 0.0) {
      v_max = std::min(v_max, max_velocity_rad_s);
    }
    // The ceiling is the REQUESTED speed even when the arm already exceeds it — a
    // MoveJ issued while the arm still coasts must decelerate into the speed the
    // caller asked for, not inherit the entry speed as its cruise. Ruckig permits
    // this: `calculate` validates the TARGET state against the limits but not the
    // current one, so an over-speed start is planned down rather than refused.
    inp.max_velocity[j] = v_max;
    inp.max_acceleration[j] = accel_scale * cfg.limits.a[j];
    inp.max_jerk[j] = jerk_scale * cfg.limits.j[j];
  }

  const ruckig::Result r = otg.calculate(inp, traj);
  if (r != ruckig::Result::Working && r != ruckig::Result::Finished) {
    throw std::runtime_error("ruckig could not plan this joint move (Result=" +
                             std::to_string(static_cast<int>(r)) + ")");
  }

  const double duration_s = traj.get_duration();
  const std::int64_t itp_ns = static_cast<std::int64_t>(std::llround(cfg.itp_s * 1e9));
  const std::int64_t plan_ns = static_cast<std::int64_t>(std::llround(duration_s * 1e9));
  // The core cannot execute less than one tick, so that is the floor on the emitted
  // timeline. A sub-tick plan (already at the target, or a hair away) is stretched
  // over one ITP, which errs slow, and guarantees the strictly-increasing int64 times
  // move_trajectory validates.
  const std::int64_t exec_ns = std::max(plan_ns, itp_ns);
  const int n_intervals = std::max(1, static_cast<int>(std::ceil(static_cast<double>(exec_ns) /
                                                                 static_cast<double>(itp_ns))));

  py::list times_ns;
  py::list q_knots;
  py::list qd_knots;
  te::Vec6 p{};
  te::Vec6 v{};
  te::Vec6 a{};
  for (int k = 0; k <= n_intervals; ++k) {
    const double frac = static_cast<double>(k) / static_cast<double>(n_intervals);
    traj.at_time(duration_s * frac, p, v, a);
    times_ns.append(static_cast<std::int64_t>(std::llround(static_cast<double>(exec_ns) * frac)));
    q_knots.append(std::vector<double>(p.begin(), p.end()));
    qd_knots.append(std::vector<double>(v.begin(), v.end()));
  }
  // Pin the endpoint. Sampling at t=duration lands on the target to ~1e-12 rad, but
  // the settle tolerance is measured against the pose the CALLER asked for, so the
  // last knot states it exactly rather than to within float dust.
  q_knots[n_intervals] = std::vector<double>(pt.begin(), pt.end());
  qd_knots[n_intervals] = std::vector<double>(te::kNumJoints, 0.0);

  py::dict d;
  d["times_ns"] = times_ns;
  d["q"] = q_knots;
  d["qd"] = qd_knots;
  d["count"] = n_intervals + 1;
  d["duration_s"] = static_cast<double>(exec_ns) / 1e9;
  return d;
}

py::dict status_view_to_dict(const airo_fanuc::codec::RobotStatusView& view) {
  py::dict d;
  d["packet_type"] = view.packet_type;
  d["version_no"] = view.version_no;
  d["sequence_no"] = view.sequence_no;
  d["status"] = view.status;
  d["robot_status"] = view.robot_status;
  d["contact_stop_status"] = view.contact_stop_status;
  d["time_stamp"] = view.time_stamp;
  d["position"] = view.position;
  d["joint_angle"] = view.joint_angle;
  d["current"] = view.current;
  d["safety_scale"] = view.safety_scale;
  d["force_x"] = view.force_x;
  d["force_y"] = view.force_y;
  d["force_z"] = view.force_z;
  d["moment_x"] = view.moment_x;
  d["moment_y"] = view.moment_y;
  d["moment_z"] = view.moment_z;
  d["fs_type"] = view.fs_type;
  d["io_status"] = py::bytes(reinterpret_cast<const char*>(view.io_status.data()), view.io_status.size());
  return d;
}

py::dict py_decode_status_204(const std::string& data) {
  return status_view_to_dict(airo_fanuc::codec::decode_status_204(
      reinterpret_cast<const std::uint8_t*>(data.data()), data.size()));
}

py::dict py_decode_status_v3(const std::string& data) {
  return status_view_to_dict(airo_fanuc::codec::decode_status_v3(
      reinterpret_cast<const std::uint8_t*>(data.data()), data.size()));
}

// ---------------------------------------------------------------------------
// StreamCore — thin pybind wrapper over rt_core::RealtimeCore. This is the
// production RT-core surface the shipped FanucDriver drives (and the FakeCRX
// integration tests exercise). GIL discipline: released on start/stop/blocking
// calls; the RT thread NEVER calls back into Python.
// ---------------------------------------------------------------------------
namespace rt = airo_fanuc::rt_core;
using airo_fanuc::tick_engine::kNumJoints;
using airo_fanuc::tick_engine::Vec6;

Vec6 to_vec6(const std::vector<double>& v) {
  if (v.size() != static_cast<std::size_t>(kNumJoints)) {
    throw std::invalid_argument("expected a length-6 joint vector (got " + std::to_string(v.size()) + ")");
  }
  Vec6 out{};
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) out[j] = v[j];
  return out;
}

py::list vec6_to_list(const Vec6& v) {
  py::list out;
  for (double x : v) out.append(x);
  return out;
}

// Same as to_vec6, naming the field in the error. Used by the RtCoreConfig limit
// setters, where three same-shaped vectors are assignable and "a length-6 vector" on
// its own would not say which one was wrong. A short list raises instead of copying
// what fits, which would leave the trailing joints clamped by a default the caller
// never chose.
Vec6 to_vec6_named(const std::vector<double>& v, const char* what) {
  if (v.size() != static_cast<std::size_t>(kNumJoints)) {
    throw std::invalid_argument(std::string(what) + " needs " + std::to_string(kNumJoints) +
                                " values (one per joint), got " + std::to_string(v.size()));
  }
  return to_vec6(v);
}

class StreamCore {
 public:
  StreamCore(const std::string& host, std::uint16_t port, const rt::RtCoreConfig& cfg) {
    cfg_ = cfg;
    cfg_.host = host;
    cfg_.sm_port = port;
    core_ = std::make_unique<rt::RealtimeCore>(cfg_);
  }

  bool start() {
    py::gil_scoped_release release;
    return core_->start();
  }
  void stop() {
    py::gil_scoped_release release;
    core_->stop();
  }
  bool wait_ready(double timeout_s) {
    py::gil_scoped_release release;
    return core_->wait_ready(timeout_s);
  }
  bool running() const { return core_->running(); }
  std::uint32_t sm_negotiated_version() const { return core_->sm_negotiated_version(); }
  std::uint32_t sm_sampling_rate_ms() const { return core_->sm_sampling_rate_ms(); }

  std::uint64_t submit_trajectory(const std::vector<std::int64_t>& times_ns,
                                  const std::vector<std::vector<double>>& q,
                                  const std::vector<std::vector<double>>& qd, double speed_scale,
                                  double settle_tol_rad, double settle_vel_eps_rad_s, double settle_timeout_s,
                                  double force_stop_n, double deadman_s) {
    if (times_ns.size() != q.size() || q.size() != qd.size()) {
      throw std::invalid_argument("times_ns, q, qd must have equal length");
    }
    if (q.size() < 2) {
      throw std::invalid_argument("trajectory needs >= 2 knots");
    }
    std::vector<Vec6> qv, qdv;
    qv.reserve(q.size());
    qdv.reserve(qd.size());
    for (std::size_t i = 0; i < q.size(); ++i) {
      qv.push_back(to_vec6(q[i]));
      qdv.push_back(to_vec6(qd[i]));
    }
    return core_->submit_trajectory(times_ns, qv, qdv, speed_scale, settle_tol_rad, settle_vel_eps_rad_s,
                                    settle_timeout_s, force_stop_n, deadman_s);
  }

  std::uint64_t submit_servo(const std::vector<double>& q, double duration_s) {
    return core_->submit_servo(to_vec6(q), duration_s);
  }
  std::uint64_t submit_servo_ff(const std::vector<double>& q, const std::vector<double>& qd,
                                const std::vector<double>& qdd, double duration_s) {
    return core_->submit_servo(to_vec6(q), to_vec6(qd), to_vec6(qdd), duration_s);
  }
  std::uint64_t submit_brake() { return core_->submit_brake(); }
  std::uint64_t submit_hold() { return core_->submit_hold(); }
  void stop_j() { core_->stop_j(); }
  void hold() { core_->hold(); }
  void recover() { core_->recover(); }
  void kick() { core_->kick(); }
  void heartbeat() { core_->heartbeat(); }

  int motion_status(std::uint64_t id) const {
    return static_cast<int>(core_->motion_status(id));
  }

  py::object joints_at_wall(std::int64_t wall_ns) const {
    Vec6 out{};
    if (!core_->joints_at_wall(wall_ns, out)) {
      return py::none();
    }
    return vec6_to_list(out);
  }

  py::dict get_snapshot() const {
    const rt::StateSnapshot s = core_->snapshot();  // seqlock read; never raises
    py::dict d;
    d["mode"] = static_cast<int>(s.mode);
    d["fault"] = static_cast<int>(s.fault);
    d["conditions"] = s.conditions;
    d["epoch"] = s.epoch;
    d["q_meas"] = vec6_to_list(s.q_meas);
    d["qd_est"] = vec6_to_list(s.qd_est);
    d["q_cmd"] = vec6_to_list(s.q_cmd);
    d["qd_cmd"] = vec6_to_list(s.qd_cmd);
    d["qdd_cmd"] = vec6_to_list(s.qdd_cmd);
    d["cmd_tick"] = s.cmd_tick;
    py::list cart;
    for (double x : s.cart) cart.append(x);
    d["cart"] = cart;
    d["rx_seq"] = s.rx_seq;
    d["tx_seq"] = s.tx_seq;
    d["ctrl_time_stamp_ms"] = s.ctrl_time_stamp_ms;
    d["rx_mono_ns"] = s.rx_mono_ns;
    d["tick_mono_ns"] = s.tick_mono_ns;
    const double age_ms = (s.rx_mono_ns > 0 && s.tick_mono_ns >= s.rx_mono_ns)
                              ? static_cast<double>(s.tick_mono_ns - s.rx_mono_ns) * 1e-6
                              : 0.0;
    d["rx_age_ms"] = age_ms;
    d["e_stopped"] = s.e_stopped;
    d["in_error"] = s.in_error;
    d["tp_enabled"] = s.tp_enabled;
    d["motion_possible"] = s.motion_possible;
    d["motion_in_progress"] = s.motion_in_progress;
    d["contact_stop_status"] = s.contact_stop_status;
    d["safety_scale"] = s.safety_scale;
    d["fx"] = s.fx;
    d["fy"] = s.fy;
    d["fz"] = s.fz;
    d["mx"] = s.mx;
    d["my"] = s.my;
    d["mz"] = s.mz;
    d["fs_type"] = s.fs_type;
    d["wrench_valid"] = s.wrench_valid;
    d["active_motion_id"] = s.active_motion_id;
    d["active_motion_status"] = static_cast<int>(s.active_motion_status);
    d["total_slew_clips"] = s.total_slew_clips;
    d["rx_fresh"] = s.rx_fresh;
    return d;
  }

  py::list poll_events() {
    py::list out;
    rt::Event buf[256];
    std::size_t n = 0;
    while ((n = core_->drain_events(buf, 256)) > 0) {
      for (std::size_t i = 0; i < n; ++i) {
        py::dict e;
        e["type"] = static_cast<int>(buf[i].type);
        e["reason"] = static_cast<int>(buf[i].reason);
        e["motion_id"] = buf[i].motion_id;
        e["epoch"] = buf[i].epoch;
        e["value"] = buf[i].value;
        out.append(e);
      }
      if (n < 256) break;
    }
    return out;
  }

  py::dict timing_stats() const {
    const rt::TimingStats t = core_->timing();
    py::dict d;
    d["tx_interval_p50_ms"] = t.tx_interval_p50_ms;
    d["tx_interval_p99_ms"] = t.tx_interval_p99_ms;
    d["tx_interval_p999_ms"] = t.tx_interval_p999_ms;
    d["tx_interval_max_ms"] = t.tx_interval_max_ms;
    d["rx2tx_p50_us"] = t.rx2tx_p50_us;
    d["rx2tx_p99_us"] = t.rx2tx_p99_us;
    d["rx2tx_p999_us"] = t.rx2tx_p999_us;
    d["rx2tx_max_us"] = t.rx2tx_max_us;
    d["tick_count"] = t.tick_count;
    d["tx_count"] = t.tx_count;
    d["tau_advance_count"] = t.tau_advance_count;
    d["parked_ticks"] = t.parked_ticks;
    d["missed_rx_ticks"] = t.missed_rx_ticks;
    d["rx_seq_gaps"] = t.rx_seq_gaps;
    d["rx_nonfinite_drops"] = t.rx_nonfinite_drops;
    d["skipped_tick_windows"] = t.skipped_tick_windows;
    d["double_send_guard"] = t.double_send_guard;
    d["cpu_migrations"] = t.cpu_migrations;
    return d;
  }

 private:
  rt::RtCoreConfig cfg_;
  std::unique_ptr<rt::RealtimeCore> core_;
};

}  // namespace

PYBIND11_MODULE(_core, m) {
  m.doc() = "airo_fanuc._core — C++17 Stream Motion packet codec + real-time StreamCore.";
  m.attr("__version__") = AIRO_FANUC_CORE_VERSION;

  // Wire sizes, exposed so Python tests can assert layout without magic numbers.
  m.attr("COMMAND_PACKET_SIZE") = py::int_(airo_fanuc::codec::kCommandPacketSize);
  m.attr("STATUS_204_PACKET_SIZE") = py::int_(airo_fanuc::codec::kStatus204PacketSize);
  m.attr("STATUS_V3_PACKET_SIZE") = py::int_(airo_fanuc::codec::kStatusV3PacketSize);
  m.attr("FORCE_SENSOR_CONFIG_PACKET_SIZE") = py::int_(airo_fanuc::codec::kForceSensorConfigPacketSize);

  m.def("encode_command_packet", &py_encode_command_packet, py::arg("seq"), py::arg("is_last"),
        py::arg("do_motn_ctrl"), py::arg("pos_deg"),
        "Encode a Stream Motion CommandPacket (type 201, 344 B, big-endian). "
        "dataStyle is pinned to 0xFFFF (joint angles). Returns 344 raw bytes.");

  m.def("decode_status_204", &py_decode_status_204, py::arg("data"),
        "Decode a Stream Motion type-204 RobotStatusPacket (416 B, big-endian) into a dict.");

  m.def("decode_status_v3", &py_decode_status_v3, py::arg("data"),
        "Decode a Stream Motion v3 type-202 RobotStatusPacket (388 B, big-endian; no force block) into a "
        "dict. force_x..moment_z are 0 and fs_type is 0xFFFFFFFF (Unavailable).");

  m.def("generate_capture_path", &py_generate_capture_path, py::arg("q_cmd"), py::arg("qd_cmd"),
        py::arg("q0"), py::arg("qd0"), py::arg("config") = py::none(),
        "Synthesize the deterministic CAPTURE splice (q_cmd,qd_cmd)->(q0,qd0) the RT core will "
        "execute. Returns {would_reject, count, finished, overflow, q, qd} — the same code path "
        "as the RT execution so the Python collision check IS the executed path. Pass the same "
        "RtCoreConfig the core was constructed with — the splice is bounded by the arm limits it "
        "carries, so omitting it synthesizes under the synthetic C++ fallback envelope instead and "
        "matches only a core built from a default RtCoreConfig.");

  m.def("plan_joint_move", &py_plan_joint_move, py::arg("q0"), py::arg("qd0"), py::arg("q_target"),
        py::arg("config") = py::none(), py::arg("max_velocity_rad_s") = 0.0,
        py::arg("accel_scale") = 1.0, py::arg("jerk_scale") = 1.0,
        py::arg("qdd0") = std::vector<double>{},
        "Plan a point-to-point joint move offline with Ruckig and return ITP-spaced knots: "
        "{times_ns, q, qd, count, duration_s}, ready for FanucDriver.move_trajectory. "
        "max_velocity_rad_s is the LEADING-AXIS speed (<=0 = the config's velocity limits); "
        "accel_scale / jerk_scale are fractions of the config's acceleration / jerk limits, "
        "in (0, 1]. "
        "Pass the same RtCoreConfig the core was constructed with, so the profile is shaped by "
        "the limits the tick engine actually enforces.");

  // -------------------------------------------------------------------------
  // RT core: StreamCore + RtCoreConfig + mode/fault/status enums.
  // -------------------------------------------------------------------------
  py::enum_<rt::Mode>(m, "Mode")
      .value("STREAM_DOWN", rt::Mode::STREAM_DOWN)
      .value("PREROLL", rt::Mode::PREROLL)
      .value("HOLD", rt::Mode::HOLD)
      .value("CAPTURE", rt::Mode::CAPTURE)
      .value("TRAJECTORY", rt::Mode::TRAJECTORY)
      .value("SERVO", rt::Mode::SERVO)
      .value("BRAKE", rt::Mode::BRAKE)
      .value("SAFE_FOLLOW", rt::Mode::SAFE_FOLLOW)
      .value("RX_SILENT", rt::Mode::RX_SILENT);

  // The snapshot's `conditions` is a BITMASK, and the primary FaultReason it travels with
  // cannot decode it: FaultReason is a dense ordinal and these are disjoint bits. Bound
  // with py::arithmetic() so the bitwise tests a mask exists to support actually typecheck
  // and work. Without this the driver published an integer no caller could read.
  py::enum_<rt::Condition>(m, "Condition", py::arithmetic())
      .value("NONE", rt::Condition::kCondNone)
      .value("E_STOP", rt::Condition::kCondEStop)
      .value("IN_ERROR", rt::Condition::kCondInError)
      .value("MOTION_NOT_POSSIBLE", rt::Condition::kCondMotionNotPossible)
      .value("TEACH", rt::Condition::kCondTeach)
      .value("CONTACT_STOP", rt::Condition::kCondContactStop)
      .value("SAFETY_CLAMP", rt::Condition::kCondSafetyClamp)
      .value("RX_DEGRADED", rt::Condition::kCondRxDegraded)
      .value("RX_SILENT", rt::Condition::kCondRxSilent)
      // Diagnostic only — set when the slew clip has been active for
      // `slew_sustained_ticks` in a row. It never faults, so a caller watching for
      // trouble must not treat a non-zero mask as one.
      .value("SUSTAINED_SLEW", rt::Condition::kCondSustainedSlew);

  py::enum_<rt::FaultReason>(m, "FaultReason")
      .value("NONE", rt::FaultReason::NONE)
      .value("E_STOP", rt::FaultReason::E_STOP)
      .value("IN_ERROR", rt::FaultReason::IN_ERROR)
      .value("MOTION_NOT_POSSIBLE", rt::FaultReason::MOTION_NOT_POSSIBLE)
      .value("TEACH_MODE", rt::FaultReason::TEACH_MODE)
      .value("CONTACT_STOP", rt::FaultReason::CONTACT_STOP)
      .value("SAFETY_CLAMP", rt::FaultReason::SAFETY_CLAMP)
      .value("RX_SILENT", rt::FaultReason::RX_SILENT)
      .value("RX_DEGRADED", rt::FaultReason::RX_DEGRADED)
      .value("WATCHDOG_EXPIRED", rt::FaultReason::WATCHDOG_EXPIRED)
      .value("FORCE_GUARD", rt::FaultReason::FORCE_GUARD)
      .value("REJECTED_START_MISMATCH", rt::FaultReason::REJECTED_START_MISMATCH)
      .value("SUPERVISOR_LOST", rt::FaultReason::SUPERVISOR_LOST)
      .value("INTERNAL", rt::FaultReason::INTERNAL);

  py::enum_<rt::MotionStatus>(m, "MotionStatus")
      .value("PENDING", rt::MotionStatus::PENDING)
      .value("RUNNING", rt::MotionStatus::RUNNING)
      .value("DONE", rt::MotionStatus::DONE)
      .value("SETTLE_TIMEOUT", rt::MotionStatus::SETTLE_TIMEOUT)
      .value("STOPPED", rt::MotionStatus::STOPPED)
      .value("PREEMPTED", rt::MotionStatus::PREEMPTED)
      .value("FAULTED", rt::MotionStatus::FAULTED)
      .value("REJECTED", rt::MotionStatus::REJECTED);

  py::class_<rt::RtCoreConfig>(m, "RtCoreConfig")
      .def(py::init<>())
      // Controller interpolation period. Lives in the embedded TickEngineConfig
      // because every per-tick limit is expressed against it; surfaced here so a
      // caller on a controller with a different ITP can set it, and so the CAPTURE
      // splice check can be handed the same value the core runs with.
      .def_property(
          "itp_s", [](const rt::RtCoreConfig& c) { return c.tick.itp_s; },
          [](rt::RtCoreConfig& c, double v) { c.tick.itp_s = v; })
      // The arm's motion envelope (rad/s, rad/s², rad/s³), one value per joint. The
      // C++ defaults are a synthetic envelope for the stand-alone tick-engine tests;
      // `DriverConfig.to_rt_core_config` overwrites all three from the caller's
      // RobotProfile, and these are the values the trajectory, servo, brake, capture
      // and slew stages all clamp against.
      .def_property(
          "velocity_limits", [](const rt::RtCoreConfig& c) { return vec6_to_list(c.tick.limits.v); },
          [](rt::RtCoreConfig& c, const std::vector<double>& v) {
            c.tick.limits.v = to_vec6_named(v, "velocity_limits");
          })
      .def_property(
          "acceleration_limits", [](const rt::RtCoreConfig& c) { return vec6_to_list(c.tick.limits.a); },
          [](rt::RtCoreConfig& c, const std::vector<double>& v) {
            c.tick.limits.a = to_vec6_named(v, "acceleration_limits");
          })
      // Joint position limits (rad). Default ±inf in the C++ core — a driver sets the
      // arm's real values from its RobotProfile; see Limits::pos_lo.
      .def_property(
          "position_limits_lower",
          [](const rt::RtCoreConfig& c) { return vec6_to_list(c.tick.limits.pos_lo); },
          [](rt::RtCoreConfig& c, const std::vector<double>& v) {
            c.tick.limits.pos_lo = to_vec6_named(v, "position_limits_lower");
          })
      .def_property(
          "position_limits_upper",
          [](const rt::RtCoreConfig& c) { return vec6_to_list(c.tick.limits.pos_hi); },
          [](rt::RtCoreConfig& c, const std::vector<double>& v) {
            c.tick.limits.pos_hi = to_vec6_named(v, "position_limits_upper");
          })
      .def_property(
          "jerk_limits", [](const rt::RtCoreConfig& c) { return vec6_to_list(c.tick.limits.j); },
          [](rt::RtCoreConfig& c, const std::vector<double>& v) {
            c.tick.limits.j = to_vec6_named(v, "jerk_limits");
          })
      // Fractions of those limits, not limits themselves: the brake envelope and the
      // per-tick slew clip. Arm-independent, but they multiply the limits above, so
      // they travel with them.
      .def_property(
          "stop_scale_va", [](const rt::RtCoreConfig& c) { return c.tick.stop_scale_va; },
          [](rt::RtCoreConfig& c, double v) { c.tick.stop_scale_va = v; })
      .def_property(
          "stop_scale_j", [](const rt::RtCoreConfig& c) { return c.tick.stop_scale_j; },
          [](rt::RtCoreConfig& c, double v) { c.tick.stop_scale_j = v; })
      .def_property(
          "slew_factor", [](const rt::RtCoreConfig& c) { return c.tick.slew_factor; },
          [](rt::RtCoreConfig& c, double v) { c.tick.slew_factor = v; })
      // Exposed because move_trajectory REJECTS a caller's trajectory against these two
      // (the capture window, and the envelope the splice into knot 0 runs at). They must be
      // settable so the refusal and the gate that executes are decided by ONE number: as
      // C++-only defaults they would be a second, independently editable copy of the Python
      // constants the rejection is written in terms of, and "the checked path IS the
      // executed path" would not hold for them.
      .def_property(
          "capture_rate_rad_s", [](const rt::RtCoreConfig& c) { return c.tick.capture_rate_rad_s; },
          [](rt::RtCoreConfig& c, double v) { c.tick.capture_rate_rad_s = v; })
      .def_property(
          "capture_tol_rad", [](const rt::RtCoreConfig& c) { return c.tick.capture_tol_rad; },
          [](rt::RtCoreConfig& c, double v) { c.tick.capture_tol_rad = v; })
      // Exposed for the weaker version of the same reason: nothing REJECTS against these,
      // but `controller_facts` names itself their single source and the C++ mirrors name it
      // back. Unbound, they are values that can be edited in the one place claiming to own
      // them without anything changing.
      .def_property(
          "servo_limit_scale", [](const rt::RtCoreConfig& c) { return c.tick.servo_limit_scale; },
          [](rt::RtCoreConfig& c, double v) { c.tick.servo_limit_scale = v; })
      .def_property(
          "qd_end_blend_min_s", [](const rt::RtCoreConfig& c) { return c.tick.qd_end_blend_min_s; },
          [](rt::RtCoreConfig& c, double v) { c.tick.qd_end_blend_min_s = v; })
      .def_readwrite("rx_silence_blind_hold_ms", &rt::RtCoreConfig::rx_silence_blind_hold_ms)
      .def_readwrite("rx_silence_qd_ramp_ms", &rt::RtCoreConfig::rx_silence_qd_ramp_ms)
      .def_readwrite("rx_silent_park_ms", &rt::RtCoreConfig::rx_silent_park_ms)
      .def_readwrite("antiflap_dwell_ms", &rt::RtCoreConfig::antiflap_dwell_ms)
      .def_readwrite("safe_follow_rate_rad_s", &rt::RtCoreConfig::safe_follow_rate_rad_s)
      .def_readwrite("safe_follow_deadband_rad", &rt::RtCoreConfig::safe_follow_deadband_rad)
      .def_readwrite("safety_scale_min", &rt::RtCoreConfig::safety_scale_min)
      .def_readwrite("supervisor_lost_s", &rt::RtCoreConfig::supervisor_lost_s)
      .def_readwrite("preroll_timeout_s", &rt::RtCoreConfig::preroll_timeout_s)
      .def_readwrite("rt_priority", &rt::RtCoreConfig::rt_priority)
      .def_readwrite("sched_fifo", &rt::RtCoreConfig::sched_fifo)
      .def_readwrite("mlock", &rt::RtCoreConfig::mlock)
      .def_readwrite("pll_rx_lead_us", &rt::RtCoreConfig::pll_rx_lead_us)
      .def_readwrite("pll_kp", &rt::RtCoreConfig::pll_kp)
      .def_readwrite("sm_version", &rt::RtCoreConfig::sm_version);

  py::class_<StreamCore>(m, "StreamCore")
      .def(py::init<const std::string&, std::uint16_t, const rt::RtCoreConfig&>(), py::arg("host"),
           py::arg("port"), py::arg("config") = rt::RtCoreConfig{})
      .def("start", &StreamCore::start)
      .def("stop", &StreamCore::stop)
      .def("wait_ready", &StreamCore::wait_ready, py::arg("timeout_s") = 5.0)
      .def_property_readonly("running", &StreamCore::running)
      .def("submit_trajectory", &StreamCore::submit_trajectory, py::arg("times_ns"), py::arg("q"),
           py::arg("qd"), py::arg("speed_scale") = 1.0, py::arg("settle_tol_rad") = airo_fanuc::tick_engine::deg2rad(0.5),
           py::arg("settle_vel_eps_rad_s") = airo_fanuc::tick_engine::deg2rad(2.0), py::arg("settle_timeout_s") = 2.0,
           py::arg("force_stop_n") = 0.0, py::arg("deadman_s") = 0.0)
      .def("submit_servo", &StreamCore::submit_servo, py::arg("q"), py::arg("duration"))
      .def("submit_servo_ff", &StreamCore::submit_servo_ff, py::arg("q"), py::arg("qd"),
           py::arg("qdd"), py::arg("duration"))
      .def("submit_brake", &StreamCore::submit_brake)
      .def("submit_hold", &StreamCore::submit_hold)
      .def("stop_j", &StreamCore::stop_j)
      .def("hold", &StreamCore::hold)
      .def("recover", &StreamCore::recover)
      .def("kick", &StreamCore::kick)
      .def("heartbeat", &StreamCore::heartbeat)
      .def("motion_status", &StreamCore::motion_status, py::arg("motion_id"))
      .def("joints_at_wall", &StreamCore::joints_at_wall, py::arg("wall_ns"))
      .def("get_snapshot", &StreamCore::get_snapshot)
      .def("poll_events", &StreamCore::poll_events)
      .def("timing_stats", &StreamCore::timing_stats)
      .def_property_readonly("sm_negotiated_version", &StreamCore::sm_negotiated_version,
                             "Stream Motion version the controller reported it will serve "
                             "(GetCapability type-7 reply); 0 until a reply is seen.")
      .def_property_readonly("sm_sampling_rate_ms", &StreamCore::sm_sampling_rate_ms,
                             "Interpolation period in milliseconds as reported by the controller "
                             "(GetCapability type-7 reply); 0 until a reply is seen. Compare "
                             "against the configured itp_s to catch a controller whose period is "
                             "not the one the driver was configured for.");
}
