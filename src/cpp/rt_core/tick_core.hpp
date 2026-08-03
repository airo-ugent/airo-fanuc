// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — TickCore: the PURE, I/O-free 125 Hz motion tick.
// NO threads, sockets, clocks, or heap.
//
// This is where ALL safety logic lives so it is unit-testable in plain C++ by
// feeding hand-crafted RX samples and asserting the emitted command + events +
// exposed state: the mode machine, epoch checks, brake/capture/servo/Hermite
// selection, slew clip, force-guard, RX-silence escalation, SAFE_FOLLOW
// re-anchor, settle, and the qd_end blend. `RealtimeCore` is the thin I/O
// wrapper (sockets/timerfd/PLL/threads/snapshot) that calls `tick()` once per
// 8 ms window; ONE tick() call ⇒ ONE motion-τ advance ⇒ (at most) ONE TX.
//
// CLOCK-FREE: TickCore never reads a clock. Elapsed time is TICK-COUNTED (each
// tick = itp_s). RX-silence, the anti-flap dwell, the deadman, brake/settle caps
// are all measured in ticks. RealtimeCore owns the wall clock (PLL / snapshot
// ages) — that is the only place a clock is read.
//
// UNITS: radians throughout; the deg/9-pad/0xFFFF wire conversion is in
// RealtimeCore. `q`/`qd` marked "wire" are speed-scaled (the value we command),
// still in radians.

#pragma once

#include <array>
#include <cstdint>

#include "rt_core/rings.hpp"
#include "rt_core/rt_core_config.hpp"
#include "tick_engine/brake.hpp"
#include "tick_engine/capture.hpp"
#include "tick_engine/hermite.hpp"
#include "tick_engine/servo.hpp"
#include "tick_engine/settle.hpp"
#include "tick_engine/slew.hpp"

namespace airo_fanuc::rt_core {

// A decoded RX sample for one tick. RealtimeCore builds this from a status packet
// (type-202 at the v3 default, type-204 at v4: deg→rad q_meas, 5-sample
// finite-difference-slope qd_est, decoded gates, wrench); unit tests build it by
// hand. `nullptr` into tick() means "no fresh RX this window" (an RX-silence tick).
struct RxSample {
  Vec6 q_meas{};   // radians
  Vec6 qd_est{};   // radians/s (finite-difference slope over the 5-tap window)
  Gates gates{};   // decoded autonomy gates
  double safety_scale{1.0};
  std::uint8_t contact_stop_status{0};
  // Force/torque (tool frame).
  double fx{0.0}, fy{0.0}, fz{0.0}, mx{0.0}, my{0.0}, mz{0.0};
  std::uint32_t fs_type{0};
  bool wrench_valid{false};
  // Pass-throughs for the snapshot (not used by the tick math).
  std::uint32_t rx_seq{0};
  std::uint32_t ctrl_time_stamp_ms{0};
  std::array<double, 9> cart{};
  std::int64_t rx_mono_ns{0};
};

// Result of consuming a mailbox target (for RealtimeCore's off-RT retire logic).
struct ConsumeResult {
  bool accepted{false};
  bool rejected_stale{false};     // tagged epoch < live epoch (structurally unexecutable)
  bool rejected_capture{false};   // |q_cmd − q0| > capture_tol (REJECTED_START_MISMATCH)
  std::uint64_t motion_id{0};
};

class TickCore {
 public:
  explicit TickCore(const RtCoreConfig& cfg);

  TickCore(const TickCore&) = delete;
  TickCore& operator=(const TickCore&) = delete;

  // ---- lifecycle (called on the RT thread / directly in tests) ----
  // Enter streaming: seed the commanded anchor + slew reference from the first
  // measured pose (PREROLL → HOLD). Call once after the SM handshake.
  void begin_streaming(const Vec6& q_meas);

  // ---- the one entry point ----
  // `rx`      : freshest RX this window, or nullptr (RX-silence tick).
  // `pending` : the latest target drained from the mailbox this window, or
  //             nullptr. Consumed AFTER gate/fault processing so a target that
  //             raced a fault (tagged the pre-bump epoch) is rejected.
  // `consume_superseded` : true iff `pending` was superseded by a stop_j issued
  //             AFTER it was submitted (caller-causal stop-generation check;
  //             RealtimeCore computes it). A superseded TRAJECTORY/SERVO does NOT
  //             activate — the stop wins. Defaults false so tests that do not
  //             exercise stop precedence need not thread the flag through.
  Command tick(const RxSample* rx, const Target* pending, bool consume_superseded = false);

  // ---- control ops (RT-thread; RealtimeCore forwards its cross-thread atomics
  //      here at tick start; tests call directly) ----
  void request_stop();   // stop_j — universal preempt, checked FIRST each tick → BRAKE
  void request_hold();   // hold() — brake to rest → HOLD (active motion → PREEMPTED)
  void request_recover();  // recover() — leave SAFE_FOLLOW/RX_SILENT after the anti-flap dwell
  void kick();           // caller-fed deadman reset
  void heartbeat();      // supervisor-liveness beat (SUPERVISOR_LOST watchdog reset)

  // ---- state exposure (for the seqlock snapshot + tests) ----
  Mode mode() const { return mode_; }
  FaultReason fault() const { return fault_; }
  std::uint32_t conditions() const { return conditions_; }
  std::uint64_t epoch() const { return epoch_; }
  const Vec6& q_cmd() const { return q_cmd_; }
  const Vec6& qd_cmd() const { return qd_cmd_; }
  const Vec6& qdd_cmd() const { return qdd_cmd_; }
  // Ticks since construction, one per tick(), TX or not. Published as
  // StateSnapshot::cmd_tick and echoed back as Target::plan_tick.
  std::uint64_t tick_no() const { return tick_no_; }
  std::uint64_t active_motion_id() const { return active_motion_id_; }
  MotionStatus active_motion_status() const { return active_status_; }
  std::uint64_t total_slew_clips() const { return slew_.total_clips(); }

  // ---- epoch bump-event table (one C++ unit test per reason) ----
  std::uint64_t bump_count(BumpReason r) const { return bump_counts_[static_cast<std::size_t>(r)]; }

  // ---- events (RT-thread single consumer: RealtimeCore or the test) ----
  bool pop_event(Event& out) { return events_.pop(out); }

  const ConsumeResult& last_consume() const { return last_consume_; }

 private:
  // Sub-phase of a trajectory playback.
  enum class TrajPhase : std::uint8_t { kPlaying, kEndBlend, kSettling };
  // Sub-phase of SAFE_FOLLOW.
  enum class FollowPhase : std::uint8_t { kRamp, kReanchor };

  // --- helpers ---
  void process_gates(const Gates& g);           // edge detect + kill-vs-suspend + condition bits
  void handle_rx_silence();                      // graduated 100 ms / 500 ms escalation
  void handle_deadman();                         // TRAJECTORY-only caller-fed watchdog
  void handle_supervisor_liveness();             // heartbeat-fed watchdog → FAULTED(SUPERVISOR_LOST)
  void handle_force_guard(const RxSample& rx);   // armed-per-motion |F| threshold
  ConsumeResult consume(const Target& t, bool superseded_by_stop);  // epoch-checked mailbox pop
  Vec6 dispatch_mode();                          // produce the desired (pre-slew) command

  void bump_epoch(BumpReason r);
  void enter_fault(FaultReason cause, BumpReason bump, MotionStatus motion_result);  // → SAFE_FOLLOW (60 ms ramp)
  void enter_rx_silent();
  void begin_brake(MotionStatus motion_result, FaultReason reason);  // stop_j / hold / force / deadman-less
  void resolve_active(MotionStatus st);
  void go_hold();
  void install_trajectory(const Target& t);
  void emit(EventType t, FaultReason reason = FaultReason::NONE, std::int64_t value = 0, std::uint64_t motion_id = 0);
  void set_condition(std::uint32_t bit, bool on);
  Vec6 dispatch_trajectory();
  Vec6 dispatch_servo();
  Vec6 dispatch_brake();
  Vec6 dispatch_safe_follow();
  int servo_stale_ticks() const;

  double itp() const { return cfg_.tick.itp_s; }
  int ms_to_ticks(double ms) const;

  // --- config ---
  RtCoreConfig cfg_;

  // --- tick-engine objects (constructed from cfg_.tick) ---
  tick_engine::TrajectorySampler sampler_;
  tick_engine::Brake brake_;
  tick_engine::Servo servo_;
  tick_engine::SlewLimiter slew_;
  tick_engine::SettleDetector settle_;
  tick_engine::QdEndBlend qd_blend_;
  tick_engine::CapturePath capture_;

  // --- mode / health ---
  Mode mode_{Mode::STREAM_DOWN};
  FaultReason fault_{FaultReason::NONE};
  std::uint32_t conditions_{0};
  std::uint64_t epoch_{0};
  std::array<std::uint64_t, static_cast<std::size_t>(BumpReason::kCount)> bump_counts_{};

  // --- commanded anchor (continuity by construction) ---
  Vec6 q_cmd_{};    // last commanded position (radians, post-slew) — the hold/continuity anchor
  Vec6 qd_cmd_{};   // last commanded velocity (wire, radians/s) — brake seed
  Vec6 qdd_cmd_{};  // last commanded acceleration (wire) — brake seed
  bool streaming_{false};

  // --- measured (from last RX) ---
  Vec6 q_meas_{};
  Vec6 qd_est_{};
  bool have_meas_{false};

  // --- gates ---
  Gates last_gates_{};

  // --- active motion ---
  std::uint64_t active_motion_id_{0};
  MotionStatus active_status_{MotionStatus::PENDING};

  // --- trajectory playback ---
  TrajPhase traj_phase_{TrajPhase::kPlaying};
  std::int64_t tau_ns_{0};          // motion-relative time (advanced by itp each playing tick)
  std::int64_t traj_end_tau_ns_{0}; // wire-elapsed exhaustion point (= duration / speed_scale)
  double speed_scale_{1.0};
  Vec6 qd_end_wire_{};              // terminal velocity × speed_scale (for the qd_end blend)
  Vec6 q_hold_{};                   // settle target
  std::int64_t tau_blend_ns_{0};    // qd_end-blend time
  tick_engine::TickEngineConfig motion_tick_cfg_{};  // cfg.tick + per-motion settle overrides

  // --- capture ---
  int capture_idx_{0};
  Target captured_target_{};        // the trajectory to install after CAPTURE completes
  // Wire-elapsed time of the knot the pending trajectory is JOINED at — 0 for knot 0.
  // install_trajectory starts playback one tick past it.
  std::int64_t join_tau_ns_{0};

  // --- tick counter (published as StateSnapshot::cmd_tick) ---
  std::uint64_t tick_no_{0};

  // --- servo ---
  Vec6 servo_last_q_{};
  double servo_last_dur_{0.0};
  int ticks_since_servo_set_{0};
  bool servo_held_{false};

  // --- brake ---
  FaultReason brake_dest_reason_{FaultReason::NONE};  // if != NONE, brake ends in fault handling
  bool brake_is_motion_{false};  // true if the brake IS the tracked motion (submitted Brake target)

  // --- SAFE_FOLLOW ---
  FollowPhase follow_phase_{FollowPhase::kRamp};
  std::int64_t tau_follow_ns_{0};

  // --- RX-silence ---
  int ticks_since_rx_{0};
  bool rx_degraded_entered_{false};

  // --- deadman ---
  bool deadman_armed_{false};
  double deadman_s_{0.0};
  int ticks_since_kick_{0};

  // --- supervisor liveness (SUPERVISOR_LOST) ---
  bool heartbeated_{false};          // beat drained at tick start
  bool supervisor_hb_armed_{false};  // armed after the supervisor's FIRST beat
  int ticks_since_heartbeat_{0};
  bool supervisor_lost_latched_{false};  // this lapse has already faulted (cleared by a beat)

  // --- force-guard ---
  bool force_armed_{false};
  double force_stop_n_{0.0};

  // --- recovery / anti-flap dwell ---
  bool recover_pending_{false};
  int all_clear_ticks_{0};

  // --- control-op flags (drained at tick start) ---
  bool stop_requested_{false};
  bool hold_requested_{false};
  bool kicked_{false};

  // --- events + last consume ---
  SpscRing<Event, 256> events_;
  ConsumeResult last_consume_{};
  bool sustained_slew_latched_{false};
};

}  // namespace airo_fanuc::rt_core
