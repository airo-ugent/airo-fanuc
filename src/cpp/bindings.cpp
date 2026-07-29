// SPDX-License-Identifier: Apache-2.0
//
// pybind11 bindings for airo_fanuc._core.
//
// Binds two layers: the Stream Motion packet codec (so the Python `wire.py`
// oracle can byte-compare the C++ encoder/decoder against it) AND the real-time
// StreamCore + RtCoreConfig + capture-path generator.

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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

// P4b: expose the deterministic CAPTURE-splice generator so the Python driver can
// collision-check the EXACT knots the RT core will execute ("the checked path IS
// the executed path", PLAN.md §5.1 / capture.hpp header note). Uses a default
// TickEngineConfig, whose capture/limit fields mirror controller_facts and equal
// the RT core's embedded cfg.tick (RtCoreConfig does not expose the tick config to
// Python, so both sides use the same defaults → byte-identical output).
py::dict py_generate_capture_path(const std::vector<double>& q_cmd, const std::vector<double>& qd_cmd,
                                  const std::vector<double>& q0, const std::vector<double>& qd0) {
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
  const te::TickEngineConfig cfg{};

  py::dict d;
  d["would_reject"] = te::capture_would_reject(qc, q0v, cfg);

  auto path = std::make_unique<te::CapturePath>();
  te::generate_capture_path(qc, qdc, q0v, qd0v, cfg, *path);
  d["count"] = path->count;
  d["finished"] = path->finished;
  d["overflow"] = path->overflow;
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
// production RT-core surface the shipped FanucDriver drives (and the FakeCRX L2
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
        "Decode a legacy V3 type-202 RobotStatusPacket (388 B, big-endian; no force block) into a "
        "dict. force_x..moment_z are 0 and fs_type is 0xFFFFFFFF (Unavailable).");

  m.def("generate_capture_path", &py_generate_capture_path, py::arg("q_cmd"), py::arg("qd_cmd"),
        py::arg("q0"), py::arg("qd0"),
        "Synthesize the deterministic CAPTURE splice (q_cmd,qd_cmd)->(q0,qd0) the RT core will "
        "execute. Returns {would_reject, count, finished, overflow, q, qd} — the same code path "
        "as the RT execution so the Python collision check IS the executed path (PLAN.md §5.1).");

  // -------------------------------------------------------------------------
  // RT core (P3b): StreamCore + RtCoreConfig + mode/fault/status enums.
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
      .value("DRIFT", rt::FaultReason::DRIFT)
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
      .def_readwrite("rx_silence_blind_hold_ms", &rt::RtCoreConfig::rx_silence_blind_hold_ms)
      .def_readwrite("rx_silence_qd_ramp_ms", &rt::RtCoreConfig::rx_silence_qd_ramp_ms)
      .def_readwrite("rx_silent_park_ms", &rt::RtCoreConfig::rx_silent_park_ms)
      .def_readwrite("antiflap_dwell_ms", &rt::RtCoreConfig::antiflap_dwell_ms)
      .def_readwrite("safe_follow_rate_rad_s", &rt::RtCoreConfig::safe_follow_rate_rad_s)
      .def_readwrite("safe_follow_deadband_rad", &rt::RtCoreConfig::safe_follow_deadband_rad)
      .def_readwrite("safety_scale_min", &rt::RtCoreConfig::safety_scale_min)
      .def_readwrite("supervisor_lost_s", &rt::RtCoreConfig::supervisor_lost_s)
      .def_readwrite("drift_lag_ticks", &rt::RtCoreConfig::drift_lag_ticks)
      .def_readwrite("drift_fault_rad", &rt::RtCoreConfig::drift_fault_rad)
      .def_readwrite("drift_fault_ticks", &rt::RtCoreConfig::drift_fault_ticks)
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
           py::arg("qd"), py::arg("speed_scale") = 1.0, py::arg("settle_tol_rad") = 0.008726646259971648,
           py::arg("settle_vel_eps_rad_s") = 0.03490658503988659, py::arg("settle_timeout_s") = 2.0,
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
      .def("timing_stats", &StreamCore::timing_stats);
}
