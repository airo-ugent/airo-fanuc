// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — RT core configuration, enums and POD command/gate/target types
// (P3b). PLAN.md §5.2 + design doc 07 §3.5.
//
// `RtCoreConfig` MIRRORS the single-sourced Python constants in
// `airo_fanuc.controller_facts` (see PLAN.md §5.1). Every RX-silence / dwell /
// re-anchor field carries a comment naming its `controller_facts` symbol. The
// Python `DriverConfig.to_rt_core_config()` POPULATES the protocol + watchdog
// fields from `controller_facts` and `FanucDriver` passes the struct into the C++
// `StreamCore` at construction; the C++ defaults still mirror the rest so the pure
// `TickCore` logic is testable stand-alone. If you change a value in
// `controller_facts.py` that a C++ test hardcodes, grep `controller_facts` in the
// C++ tree and update the mirror comment + test.
//
// UNIT CONVENTION (binding): the core works entirely in RADIANS — the same unit
// as `TickEngineConfig` and curobo trajectories. The Stream Motion wire is
// DEGREES; that conversion happens ONLY at the wire (encode) in RealtimeCore.

#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "tick_engine/tick_engine_config.hpp"

namespace airo_fanuc::rt_core {

using tick_engine::deg2rad;
using tick_engine::kNumJoints;
using tick_engine::Vec6;

// ---------------------------------------------------------------------------
// Motion mode (design doc 07 §3.5). Orthogonal to health (FaultReason).
//   STREAM_DOWN → PREROLL → HOLD ⇄ {CAPTURE → TRAJECTORY, SERVO, BRAKE} → HOLD
//   + SAFE_FOLLOW (re-anchor commanded→measured ≤15°/s, 5° deadband; entered on
//     fault/DEGRADED; exited only via recover()).
//   + RX_SILENT (500 ms no RX → park TX).
// ---------------------------------------------------------------------------
enum class Mode : std::uint8_t {
  STREAM_DOWN,
  PREROLL,
  HOLD,
  CAPTURE,
  TRAJECTORY,
  SERVO,
  BRAKE,
  SAFE_FOLLOW,
  RX_SILENT,
};

// Latched health cause (design doc 07 §3.5). NONE = OK.
enum class FaultReason : std::uint8_t {
  NONE,
  E_STOP,
  IN_ERROR,
  MOTION_NOT_POSSIBLE,
  TEACH_MODE,
  CONTACT_STOP,
  SAFETY_CLAMP,
  RX_SILENT,
  RX_DEGRADED,
  DRIFT,
  WATCHDOG_EXPIRED,
  FORCE_GUARD,
  REJECTED_START_MISMATCH,
  SUPERVISOR_LOST,
  INTERNAL,
};

// Per-motion result (mirrors MotionResult / MotionStatus in PLAN.md §5.1).
enum class MotionStatus : std::uint8_t {
  PENDING,
  RUNNING,
  DONE,
  SETTLE_TIMEOUT,
  STOPPED,
  PREEMPTED,
  FAULTED,
  REJECTED,
};

// Condition bitmask (F8: conditions are a SET). Snapshot exposes this alongside
// the primary latched FaultReason.
enum Condition : std::uint32_t {
  kCondNone = 0,
  kCondEStop = 1u << 0,
  kCondInError = 1u << 1,
  kCondMotionNotPossible = 1u << 2,
  kCondTeach = 1u << 3,
  kCondContactStop = 1u << 4,
  kCondSafetyClamp = 1u << 5,
  kCondRxDegraded = 1u << 6,
  kCondRxSilent = 1u << 7,
  kCondSustainedSlew = 1u << 8,  // F35 diagnostic (never faults)
};

// ---------------------------------------------------------------------------
// Epoch bump-event table (R2 F4). Exactly these nine scenarios bump the core
// epoch; each is L1-tested in isolation (test_epoch.cpp). A stale target (tagged
// with a pre-bump epoch) is structurally unexecutable — rejected at CONSUME.
// (kSupervisorLost + kDriftFault added at P-1 finalization; see the P-1
// controller-notes E6/E9 gaps — both were defined-but-never-set fault sources.)
// ---------------------------------------------------------------------------
enum class BumpReason : std::uint8_t {
  kNone = 0,
  kFaultEntry = 1,          // in_error / internal fault entry
  kSafeFollowEntry = 2,     // entry into SAFE_FOLLOW for a non-kill cause
  kKillDegradedEntry = 3,   // kill-type DEGRADED: TEACH / contact∈{2,4} / clamp / motion_possible=false
  kRecoveryComplete = 4,    // recover() completed (anti-flap dwell satisfied)
  kEStopEdge = 5,           // e-stop rising edge
  kRxSilentEntry = 6,       // 500 ms RX silence → RX_SILENT park
  kDeadmanTrip = 7,         // caller-fed deadman expired
  kSupervisorLost = 8,      // supervisor heartbeat lapsed → FAULTED(SUPERVISOR_LOST), core holds
  kDriftFault = 9,          // commanded↔measured divergence > threshold, sustained
  kCount = 10,
};

// ---------------------------------------------------------------------------
// Gates — pure decode of a type-204 status (the six autonomy gates that run
// GIL-free in C++, R2 F29). NOTE: OVERRIDE_LOW is NOT here — TP override % is
// absent from type-204 (it arrives via 1 Hz RMI ext-status), so the Python
// supervisor lowers it and calls stop_j(). See PLAN.md §5.2 / design doc §1.3.
// ---------------------------------------------------------------------------
struct Gates {
  bool e_stopped{false};        // robot_status & 0x4
  bool tp_enabled{false};       // robot_status & 0x2  (TEACH)
  bool in_error{false};         // robot_status & 0x1
  bool motion_possible{false};  // status & 0x1
  bool motion_in_progress{false};  // status & 0x8
  bool contact_stop_active{false};  // contact_stop_status ∈ {2 (STOP), 4 (ESCP)}
  bool safety_clamp{false};        // safety_scale < safety_scale_min
};

// ---------------------------------------------------------------------------
// RtCoreConfig — every knob the RT core needs beyond the pure tick-math knobs in
// TickEngineConfig (which it embeds). Trivially copyable POD.
// ---------------------------------------------------------------------------
struct RtCoreConfig {
  tick_engine::TickEngineConfig tick{};  // all cubic-Hermite / brake / servo / slew / settle knobs

  // --- Graduated RX-silence (decision 7 / R1 A3), controller_facts ---
  double rx_silence_blind_hold_ms{100.0};  // controller_facts.RX_SILENCE_BLIND_HOLD_MS
  double rx_silence_qd_ramp_ms{60.0};      // controller_facts.RX_SILENCE_QD_RAMP_MS
  double rx_silent_park_ms{500.0};         // controller_facts.RX_SILENT_PARK_MS

  // --- Anti-flap dwell (R2 F7): recover() succeeds only once all kill gates
  //     have been clear this long. ---
  double antiflap_dwell_ms{500.0};  // controller_facts.ANTIFLAP_DWELL_MS

  // --- SAFE_FOLLOW re-anchor (dries starvation stage-2) ---
  double safe_follow_rate_rad_s{deg2rad(15.0)};   // controller_facts.CAPTURE_RATE_DEG_S (= STARVATION_RE_ANCHOR_RATE)
  double safe_follow_deadband_rad{deg2rad(5.0)};  // dries STARVATION_RE_ANCHOR_DRIFT_RAD

  // --- Supervisor liveness (SUPERVISOR_LOST, P-1 finalization) ---
  // The Python supervisor beats from a DEDICATED lightweight thread (no RMI, no
  // locks); if the core sees no heartbeat for this long *while streaming* (after
  // the first beat arms it), it latches FAULTED(SUPERVISOR_LOST) and holds — the
  // RT core is independent of the supervisor (F28/F29), so this makes "the
  // supervisor thread/process died" observable + safe. Generous vs the beat
  // interval (~100 ms) to tolerate GIL-storm / GC pauses without a false trip.
  double supervisor_lost_s{3.0};  // controller_facts.SUPERVISOR_LOST_S

  // --- Drift guard (DRIFT, P-1 finalization) ---
  // Lag-aligned commanded↔measured divergence: q_meas(now) vs q_cmd(now − lag).
  // Sustained divergence > drift_fault_rad for drift_fault_ticks → FAULTED(DRIFT)
  // → SAFE_FOLLOW (re-anchor + recover). This is the 22°-runaway protection the
  // dries executor had (MAX_DRIFT); E9 fixed the lag to 25 ms (≈3 ticks) so the
  // prediction is accurate. drift_lag_ticks is set from tracking_lag_s / ITP.
  int drift_lag_ticks{3};                     // round(controller_facts.tracking_lag_s / ITP_S) = round(0.025/0.008)
  double drift_fault_rad{deg2rad(10.0)};      // controller_facts.DRIFT_FAULT_DEG
  int drift_fault_ticks{5};                   // controller_facts.DRIFT_FAULT_TICKS

  // --- Gates ---
  double safety_scale_min{0.05};  // dries executor SAFETY_CLAMP threshold (<0.05)

  // --- Preroll (SM handshake wait for motion_possible) ---
  double preroll_timeout_s{5.0};  // dries STREAM_MOTION_PREROLL_TIMEOUT_S

  // --- RealtimeCore I/O + PLL (not used by pure TickCore) ---
  std::string host{"127.0.0.1"};
  std::uint16_t sm_port{60015};
  int rt_core{31};       // logical core to self-pin (PLAN.md §5.5); SMT sibling 15 reserved idle
  int rt_priority{80};   // SCHED_FIFO priority
  bool pin_core{false};  // request affinity (graceful on EPERM/EINVAL)
  bool sched_fifo{false};
  bool mlock{false};
  double pll_rx_lead_us{300.0};  // PLL target: fire tick this long after RX
  double pll_kp{0.20};           // PLL proportional gain
  bool reply_on_receive{false};  // H2 comparison flag (raw reply-on-receive instead of PLL-clocked)
  std::uint32_t sm_version{3};   // fallback default; GetCapability negotiation adopts the controller's available_version at handshake
};

// ---------------------------------------------------------------------------
// Command — the pure output of TickCore::tick. Positions are RADIANS; the wire
// (deg, 9-pad, 0xFFFF) conversion happens in RealtimeCore::encode.
// ---------------------------------------------------------------------------
struct Command {
  Vec6 q_rad{};              // commanded joint position (radians), post-slew-clip
  bool tx{true};             // transmit this window? false only in RX_SILENT / SHUTDOWN park
  bool is_last{false};       // terminal CommandPacket flag (Stop after)
  int slew_clipped_joints{0};  // diagnostics (never faults)
};

// ---------------------------------------------------------------------------
// Target — a POD mailbox payload (latest-wins single slot). Trajectory arrays
// are allocated/owned by the submitter and referenced by pointer (RT thread
// never allocates/frees — retire ring frees off-RT). `epoch` is tagged at submit
// and re-checked at CONSUME (R2 F4).
// ---------------------------------------------------------------------------
enum class TargetKind : std::uint8_t { kNone, kHold, kTrajectory, kServo, kBrake };

struct Target {
  TargetKind kind{TargetKind::kNone};
  std::uint64_t motion_id{0};
  std::uint64_t epoch{0};  // core epoch at submit time (CONSUME re-checks vs live epoch)
  std::uint64_t stop_gen{0};  // caller stop-generation at submit (finding-1: a later stop_j supersedes this target)
  const void* buffer_owner{nullptr};  // opaque handle to the owning TrajBuffer (RealtimeCore retire; TickCore ignores)

  // Trajectory (non-owning view; must outlive the motion).
  const std::int64_t* times_ns{nullptr};
  const Vec6* q{nullptr};
  const Vec6* qd{nullptr};
  int n{0};
  double speed_scale{1.0};
  // per-motion settle override (defaults mirror TickEngineConfig / SettlePolicy)
  double settle_tol_rad{deg2rad(0.5)};
  double settle_vel_eps_rad_s{deg2rad(2.0)};
  double settle_timeout_s{2.0};
  double force_stop_n{0.0};  // 0 = force-guard disarmed; >0 arms |F| threshold (N)
  double deadman_s{0.0};     // 0 = no deadman; >0 arms caller-fed watchdog (TRAJECTORY only)

  // Servo.
  Vec6 servo_q{};
  double servo_duration_s{0.0};
  // Feed-forward servo (optional): when servo_has_ff, the RT servo uses these as
  // Ruckig target_velocity/target_acceleration instead of reconstructing a secant
  // velocity + zero acceleration from consecutive position targets.  Lets a caller
  // stream an externally-planned smooth trajectory (e.g. an MPC action-sequence knot)
  // faithfully — no secant lag, no accel=0 forcing at every knot (the servoing twitch).
  Vec6 servo_qd{};
  Vec6 servo_qdd{};
  bool servo_has_ff{false};
};

// ---------------------------------------------------------------------------
// Event — POD pushed to the lock-free event ring, drained by Python (never a
// callback into Python from the RT thread). Fixed size, trivially copyable.
// ---------------------------------------------------------------------------
enum class EventType : std::uint16_t {
  kModeChange,       // value = new Mode ordinal
  kEpochBump,        // reason = mapped FaultReason where meaningful; value = BumpReason ordinal
  kSafeFollowEntry,
  kFaultEntry,
  kRecoveryComplete,
  kRxDegraded,
  kRxSilentEntry,
  kDeadmanTrip,
  kForceGuardTrip,
  kCaptureRejected,
  kSustainedSlew,
  kMotionDone,
  kMotionSettleTimeout,
  kMotionStopped,
  kMotionPreempted,
  kMotionFaulted,
  kMotionRejected,
  kMotionRunning,
  kGateEdge,  // value = Condition bit that changed; reason = mapped fault
  kSupervisorLost,  // supervisor heartbeat lapsed (P-1 finalization)
  kDrift,           // commanded↔measured divergence fault (P-1 finalization)
};

struct Event {
  EventType type{EventType::kModeChange};
  FaultReason reason{FaultReason::NONE};
  std::uint64_t motion_id{0};
  std::uint64_t epoch{0};
  std::int64_t value{0};
};

// ---------------------------------------------------------------------------
// Gate decode (pure). robot_status/status bit layout from PLAN.md §A (verified
// against dries packets.py): robot_status 0x1 in_error / 0x2 tp_enabled /
// 0x4 e_stopped; status 0x1 motion_possible / 0x8 motion_in_progress.
// ---------------------------------------------------------------------------
inline Gates decode_gates(std::uint8_t status, std::uint8_t robot_status, std::uint8_t contact_stop_status,
                          double safety_scale, double safety_scale_min) {
  Gates g{};
  g.in_error = (robot_status & 0x1u) != 0u;
  g.tp_enabled = (robot_status & 0x2u) != 0u;
  g.e_stopped = (robot_status & 0x4u) != 0u;
  g.motion_possible = (status & 0x1u) != 0u;
  g.motion_in_progress = (status & 0x8u) != 0u;
  g.contact_stop_active = (contact_stop_status == 2u || contact_stop_status == 4u);
  g.safety_clamp = (safety_scale < safety_scale_min);
  return g;
}

}  // namespace airo_fanuc::rt_core
