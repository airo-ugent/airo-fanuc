// SPDX-License-Identifier: Apache-2.0
//
// TickCore — the pure 125 Hz motion tick. See tick_core.hpp.

#include "rt_core/tick_core.hpp"

#include <algorithm>
#include <cmath>

namespace airo_fanuc::rt_core {

namespace {

double max_abs_diff(const Vec6& a, const Vec6& b) {
  double m = 0.0;
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    m = std::max(m, std::abs(a[j] - b[j]));
  }
  return m;
}

double max_abs(const Vec6& v) {
  double m = 0.0;
  for (double x : v) {
    m = std::max(m, std::abs(x));
  }
  return m;
}

constexpr std::uint32_t kKillMask = kCondEStop | kCondInError | kCondMotionNotPossible | kCondTeach |
                                    kCondContactStop | kCondSafetyClamp;

EventType motion_event_for(MotionStatus st) {
  switch (st) {
    case MotionStatus::DONE:
      return EventType::kMotionDone;
    case MotionStatus::SETTLE_TIMEOUT:
      return EventType::kMotionSettleTimeout;
    case MotionStatus::STOPPED:
      return EventType::kMotionStopped;
    case MotionStatus::PREEMPTED:
      return EventType::kMotionPreempted;
    case MotionStatus::FAULTED:
      return EventType::kMotionFaulted;
    case MotionStatus::REJECTED:
      return EventType::kMotionRejected;
    default:
      return EventType::kMotionDone;
  }
}

}  // namespace

TickCore::TickCore(const RtCoreConfig& cfg)
    : cfg_(cfg),
      brake_(cfg.tick),
      servo_(cfg.tick),
      slew_(cfg.tick),
      settle_(cfg.tick),
      motion_tick_cfg_(cfg.tick) {}

int TickCore::ms_to_ticks(double ms) const {
  const int t = static_cast<int>(std::ceil((ms / 1000.0) / itp() - 1e-9));
  return t < 1 ? 1 : t;
}

int TickCore::servo_stale_ticks() const {
  const int dur_ticks = static_cast<int>(std::ceil(servo_last_dur_ / itp() - 1e-9));
  return std::max(1, dur_ticks) + 2;
}

void TickCore::emit(EventType t, FaultReason reason, std::int64_t value, std::uint64_t motion_id) {
  Event e{};
  e.type = t;
  e.reason = reason;
  e.value = value;
  e.motion_id = motion_id;
  e.epoch = epoch_;
  events_.push(e);  // drop-if-full (fixed ring; never blocks/allocs)
}

void TickCore::set_condition(std::uint32_t bit, bool on) {
  if (on) {
    conditions_ |= bit;
  } else {
    conditions_ &= ~bit;
  }
}

void TickCore::bump_epoch(BumpReason r) {
  ++epoch_;
  ++bump_counts_[static_cast<std::size_t>(r)];
  emit(EventType::kEpochBump, FaultReason::NONE, static_cast<std::int64_t>(r));
}

void TickCore::begin_streaming(const Vec6& q_meas) {
  q_meas_ = q_meas;
  qd_est_ = Vec6{};
  have_meas_ = true;
  q_cmd_ = q_meas;
  qd_cmd_ = Vec6{};
  qdd_cmd_ = Vec6{};
  slew_.reset(q_meas);
  mode_ = Mode::HOLD;
  fault_ = FaultReason::NONE;
  conditions_ = 0;
  streaming_ = true;
  ticks_since_rx_ = 0;
  // Supervisor-liveness state (fresh streaming session). Do NOT clear
  // supervisor_hb_armed_: once the supervisor has beaten it stays armed across
  // re-handshakes; we only reset the elapsed counter so a re-stream never trips
  // on a stale count.
  ticks_since_heartbeat_ = 0;
}

// ---------------------------------------------------------------------------
// control ops
// ---------------------------------------------------------------------------
void TickCore::request_stop() { stop_requested_ = true; }
void TickCore::request_hold() { hold_requested_ = true; }
void TickCore::request_recover() {
  if (mode_ == Mode::SAFE_FOLLOW || mode_ == Mode::RX_SILENT) {
    recover_pending_ = true;
    all_clear_ticks_ = 0;
  }
}
void TickCore::kick() { kicked_ = true; }
void TickCore::heartbeat() { heartbeated_ = true; }

// ---------------------------------------------------------------------------
// motion resolution / brake / hold
// ---------------------------------------------------------------------------
void TickCore::resolve_active(MotionStatus st) {
  if (active_status_ != MotionStatus::RUNNING) {
    return;
  }
  active_status_ = st;
  emit(motion_event_for(st), fault_, 0, active_motion_id_);
}

void TickCore::begin_brake(MotionStatus motion_result, FaultReason reason) {
  resolve_active(motion_result);
  brake_.seed(q_cmd_, qd_cmd_, qdd_cmd_);  // analytic wire qdd → acceleration-continuous brake entry
  mode_ = Mode::BRAKE;
  brake_dest_reason_ = reason;
  brake_is_motion_ = false;
  deadman_armed_ = false;
  force_armed_ = false;
}

void TickCore::go_hold() {
  mode_ = Mode::HOLD;
  brake_dest_reason_ = FaultReason::NONE;
  brake_is_motion_ = false;
}

// ---------------------------------------------------------------------------
// fault / SAFE_FOLLOW / RX_SILENT entry
// ---------------------------------------------------------------------------
void TickCore::enter_fault(FaultReason cause, BumpReason bump, MotionStatus motion_result) {
  bump_epoch(bump);
  fault_ = cause;
  resolve_active(motion_result);
  // 60 ms blind qd-ramp from the current commanded velocity, coasting forward to
  // rest. A linear ramp coasts exactly half the distance the current velocity
  // would have covered over T (∫qd(1−t/T)dt = qd·T/2), so the stopping distance is
  // known in closed form without needing a fresh measurement to plan against.
  qd_blend_.plan(q_cmd_, qd_cmd_, cfg_.rx_silence_qd_ramp_ms / 1000.0, itp());
  follow_phase_ = FollowPhase::kRamp;
  tau_follow_ns_ = 0;
  mode_ = Mode::SAFE_FOLLOW;
  recover_pending_ = false;
  all_clear_ticks_ = 0;
  deadman_armed_ = false;
  force_armed_ = false;
  emit(EventType::kSafeFollowEntry, cause);
  emit(EventType::kFaultEntry, cause);
}

void TickCore::enter_rx_silent() {
  bump_epoch(BumpReason::kRxSilentEntry);
  fault_ = FaultReason::RX_SILENT;
  set_condition(kCondRxSilent, true);
  resolve_active(MotionStatus::FAULTED);
  mode_ = Mode::RX_SILENT;
  deadman_armed_ = false;
  force_armed_ = false;
  emit(EventType::kRxSilentEntry, FaultReason::RX_SILENT);
}

// ---------------------------------------------------------------------------
// gates (edge detection + kill-vs-suspend) — decided here in C++, so no autonomy
// decision ever waits on the Python GIL
// ---------------------------------------------------------------------------
void TickCore::process_gates(const Gates& g) {
  // Rising edges vs the last observed gates.
  const bool e_rise = g.e_stopped && !last_gates_.e_stopped;
  const bool err_rise = g.in_error && !last_gates_.in_error;
  const bool teach_rise = g.tp_enabled && !last_gates_.tp_enabled;
  const bool contact_rise = g.contact_stop_active && !last_gates_.contact_stop_active;
  const bool clamp_rise = g.safety_clamp && !last_gates_.safety_clamp;
  const bool mp_drop = !g.motion_possible && last_gates_.motion_possible;

  // Level → condition bits (clear when the physical condition clears; the
  // primary fault_ stays latched until recover()).
  set_condition(kCondEStop, g.e_stopped);
  set_condition(kCondInError, g.in_error);
  set_condition(kCondTeach, g.tp_enabled);
  set_condition(kCondContactStop, g.contact_stop_active);
  set_condition(kCondSafetyClamp, g.safety_clamp);
  set_condition(kCondMotionNotPossible, !g.motion_possible && streaming_);

  // Observability: one edge event per rising edge.
  if (e_rise) emit(EventType::kGateEdge, FaultReason::E_STOP, kCondEStop);
  if (err_rise) emit(EventType::kGateEdge, FaultReason::IN_ERROR, kCondInError);
  if (teach_rise) emit(EventType::kGateEdge, FaultReason::TEACH_MODE, kCondTeach);
  if (contact_rise) emit(EventType::kGateEdge, FaultReason::CONTACT_STOP, kCondContactStop);
  if (clamp_rise) emit(EventType::kGateEdge, FaultReason::SAFETY_CLAMP, kCondSafetyClamp);
  if (mp_drop) emit(EventType::kGateEdge, FaultReason::MOTION_NOT_POSSIBLE, kCondMotionNotPossible);

  last_gates_ = g;

  // Kill-vs-suspend: every condition under which the controller ignores or
  // rescales our commands KILLS the active motion + bumps epoch. It must not
  // merely suspend it: while the controller was ignoring the stream the arm fell
  // behind τ, so resuming at the stored τ would command a jump of however far the
  // motion advanced in the meantime — tens of degrees for a long stall. Only react
  // while streaming; before begin_streaming the gates are informational.
  if (!streaming_) {
    return;
  }
  // Severity order: e-stop > in_error > kill-DEGRADED (TEACH/contact/clamp/mp).
  if (e_rise) {
    enter_fault(FaultReason::E_STOP, BumpReason::kEStopEdge, MotionStatus::FAULTED);
  } else if (err_rise) {
    enter_fault(FaultReason::IN_ERROR, BumpReason::kFaultEntry, MotionStatus::FAULTED);
  } else if (teach_rise) {
    enter_fault(FaultReason::TEACH_MODE, BumpReason::kKillDegradedEntry, MotionStatus::FAULTED);
  } else if (contact_rise) {
    enter_fault(FaultReason::CONTACT_STOP, BumpReason::kKillDegradedEntry, MotionStatus::FAULTED);
  } else if (clamp_rise) {
    enter_fault(FaultReason::SAFETY_CLAMP, BumpReason::kKillDegradedEntry, MotionStatus::FAULTED);
  } else if (mp_drop) {
    enter_fault(FaultReason::MOTION_NOT_POSSIBLE, BumpReason::kKillDegradedEntry, MotionStatus::FAULTED);
  }
}

// ---------------------------------------------------------------------------
// graduated RX-silence
// ---------------------------------------------------------------------------
void TickCore::handle_rx_silence() {
  if (ticks_since_rx_ == 0) {
    return;  // fresh RX this window
  }
  if (mode_ == Mode::RX_SILENT) {
    return;  // already parked
  }
  const double age_ms = static_cast<double>(ticks_since_rx_) * itp() * 1000.0;
  // 500 ms → RX_SILENT park (any mode).
  if (age_ms >= cfg_.rx_silent_park_ms) {
    enter_rx_silent();
    return;
  }
  // 100 ms mid-motion → kill-type: 60 ms blind qd-ramp → SAFE_FOLLOW; in-flight
  // handle resolves FAULTED(RX_DEGRADED).
  const bool active = (mode_ == Mode::TRAJECTORY || mode_ == Mode::CAPTURE || mode_ == Mode::SERVO ||
                       mode_ == Mode::BRAKE);
  if (active && age_ms >= cfg_.rx_silence_blind_hold_ms && !rx_degraded_entered_) {
    rx_degraded_entered_ = true;
    set_condition(kCondRxDegraded, true);
    emit(EventType::kRxDegraded, FaultReason::RX_DEGRADED);
    enter_fault(FaultReason::RX_DEGRADED, BumpReason::kSafeFollowEntry, MotionStatus::FAULTED);
  }
}

// ---------------------------------------------------------------------------
// deadman (TRAJECTORY only; caller-fed)
// ---------------------------------------------------------------------------
void TickCore::handle_deadman() {
  if (!deadman_armed_ || mode_ != Mode::TRAJECTORY) {
    return;
  }
  ++ticks_since_kick_;
  if (static_cast<double>(ticks_since_kick_) * itp() >= deadman_s_) {
    // Supervision died → auto-stop_j + latch fault (do NOT auto-resume): SAFE_FOLLOW.
    enter_fault(FaultReason::WATCHDOG_EXPIRED, BumpReason::kDeadmanTrip, MotionStatus::STOPPED);
    emit(EventType::kDeadmanTrip, FaultReason::WATCHDOG_EXPIRED);
  }
}

// ---------------------------------------------------------------------------
// supervisor liveness (SUPERVISOR_LOST). The RT core keeps ticking whether or not
// its supervisor is alive, which is precisely why a lapsed heartbeat must fault:
// otherwise the core would stream on with nothing watching the conditions only the
// supervisor can see. A lapse latches FAULTED(SUPERVISOR_LOST) and drives to a safe
// hold (SAFE_FOLLOW) so a watchdog further up can act. Armed only after the
// supervisor's first beat, and the threshold exceeds a single RMI round-trip so a
// slow supervisor tick never false-trips.
// ---------------------------------------------------------------------------
void TickCore::handle_supervisor_liveness() {
  if (!streaming_ || !supervisor_hb_armed_) {
    return;
  }
  if (fault_ == FaultReason::SUPERVISOR_LOST) {
    return;  // already latched — do not re-fault every tick
  }
  ++ticks_since_heartbeat_;
  if (static_cast<double>(ticks_since_heartbeat_) * itp() >= cfg_.supervisor_lost_s) {
    enter_fault(FaultReason::SUPERVISOR_LOST, BumpReason::kSupervisorLost, MotionStatus::FAULTED);
    emit(EventType::kSupervisorLost, FaultReason::SUPERVISOR_LOST);
  }
}

// ---------------------------------------------------------------------------
// force-guard (armed per-motion) — deterministic: trips on the tick that sees the
// over-threshold wrench, never later
// ---------------------------------------------------------------------------
void TickCore::handle_force_guard(const RxSample& rx) {
  if (!force_armed_ || !rx.wrench_valid) {
    return;
  }
  const double f = std::sqrt(rx.fx * rx.fx + rx.fy * rx.fy + rx.fz * rx.fz);
  if (f > force_stop_n_) {
    // Trip → internal stop_j + handle STOPPED + reason (no epoch bump: an expected
    // grasp stop, not a fault). Brake to rest → HOLD; caller replans.
    begin_brake(MotionStatus::STOPPED, FaultReason::NONE);
    emit(EventType::kForceGuardTrip, FaultReason::FORCE_GUARD);
  }
}

// ---------------------------------------------------------------------------
// consume (epoch-checked mailbox pop) — called AFTER fault processing so a
// target that raced a fault (tagged the pre-bump epoch) is structurally rejected.
// `superseded_by_stop` is the causal-order stop-precedence signal (see below).
// ---------------------------------------------------------------------------
ConsumeResult TickCore::consume(const Target& t, bool superseded_by_stop) {
  ConsumeResult r{};
  r.motion_id = t.motion_id;
  if (t.kind == TargetKind::kNone) {
    return r;
  }
  // Epoch-at-CONSUME: tagged epoch must equal the LIVE core epoch. A fault this
  // tick already bumped epoch → stale.
  if (t.epoch != epoch_) {
    r.rejected_stale = true;
    active_motion_id_ = t.motion_id;
    active_status_ = MotionStatus::REJECTED;
    emit(EventType::kMotionRejected, FaultReason::NONE, 0, t.motion_id);
    return r;
  }

  // Stop precedence. stop_j is the universal preempt: it takes effect within one
  // tick and ALWAYS wins. It is handled BEFORE consume (step 2), so absent this
  // guard a trajectory/servo that raced a stop into the SAME 8 ms window (e.g.
  // TrajectoryMonitor.stop_j() vs a concurrent move_trajectory) would activate
  // right after the stop's brake and run — the stop would be structurally
  // swallowed. request_stop() deliberately does NOT bump the motion epoch (a clean
  // preempt, not a fault), so the epoch check above cannot catch it.
  //
  // `superseded_by_stop` is the CAUSAL-ORDER signal RealtimeCore computes: it is
  // true iff a stop_j was issued by the caller AFTER this target was submitted
  // (target's tagged stop-generation < the live stop-generation). That precisely
  // distinguishes the two same-tick cases a bare "stop this tick" flag cannot:
  //   * submit-then-stop — trajectory submitted, THEN stop_j → target predates
  //     the stop → superseded → REJECTED (the stop wins). ✓
  //   * brake-then-submit — stop_j (quiesce), THEN move_trajectory → target
  //     carries the post-stop generation → NOT superseded → accepted, even when
  //     the robot was already at rest and the submit lands in the SAME tick as
  //     the (no-op) stop. A trajectory issued from a braked state MUST run — a
  //     bare per-tick "a stop happened" flag would wrongly reject it. ✓
  // When superseded, the brake/hold the stop installed MUST win — leave mode_
  // untouched and resolve the raced motion terminally (REJECTED, reusing the
  // consume-time reject machinery) so its MotionHandle doesn't hang PENDING.
  // kHold/kBrake are themselves quiescent (brake/hold to rest) and are compatible
  // with the stop — allowed through.
  if (superseded_by_stop && (t.kind == TargetKind::kTrajectory || t.kind == TargetKind::kServo)) {
    active_motion_id_ = t.motion_id;
    active_status_ = MotionStatus::REJECTED;
    emit(EventType::kMotionRejected, FaultReason::NONE, 0, t.motion_id);
    return r;
  }

  switch (t.kind) {
    case TargetKind::kHold: {
      if (mode_ == Mode::CAPTURE || mode_ == Mode::TRAJECTORY || mode_ == Mode::SERVO) {
        begin_brake(MotionStatus::PREEMPTED, FaultReason::NONE);
      }
      active_motion_id_ = t.motion_id;
      active_status_ = MotionStatus::DONE;
      emit(EventType::kMotionDone, FaultReason::NONE, 0, t.motion_id);
      r.accepted = true;
      return r;
    }
    case TargetKind::kBrake: {
      resolve_active(MotionStatus::PREEMPTED);
      active_motion_id_ = t.motion_id;
      active_status_ = MotionStatus::RUNNING;
      brake_.seed(q_cmd_, qd_cmd_, qdd_cmd_);
      mode_ = Mode::BRAKE;
      brake_dest_reason_ = FaultReason::NONE;
      brake_is_motion_ = true;
      deadman_armed_ = false;
      force_armed_ = false;
      emit(EventType::kMotionRunning, FaultReason::NONE, 0, t.motion_id);
      r.accepted = true;
      return r;
    }
    case TargetKind::kServo: {
      // No servo/trajectory motion while faulted / following / parked.
      if (fault_ != FaultReason::NONE || mode_ == Mode::SAFE_FOLLOW || mode_ == Mode::RX_SILENT) {
        active_motion_id_ = t.motion_id;
        active_status_ = MotionStatus::REJECTED;
        emit(EventType::kMotionRejected, fault_, 0, t.motion_id);
        return r;
      }
      // No distance guard: a servo target is tracked however far away it is, bounded
      // by the servo limits rather than refused (servo.hpp, NO DISTANCE GUARD). The
      // only servo rejection left is the faulted/following/parked case above.
      if (mode_ != Mode::SERVO) {
        servo_.start(q_cmd_, qd_cmd_, qdd_cmd_);
        mode_ = Mode::SERVO;
      }
      // Position + deadline is the whole command. Any qd/qdd the caller supplied is
      // advisory and deliberately unused — demanding an arrival velocity is what made
      // the command reverse against a forward-moving stream (servo.hpp, BEST EFFORT).
      servo_.set_target(t.servo_q, t.servo_duration_s);
      servo_last_q_ = t.servo_q;
      servo_last_dur_ = t.servo_duration_s;
      ticks_since_servo_set_ = 0;
      servo_held_ = false;
      active_motion_id_ = t.motion_id;
      active_status_ = MotionStatus::RUNNING;
      emit(EventType::kMotionRunning, FaultReason::NONE, 0, t.motion_id);
      r.accepted = true;
      return r;
    }
    case TargetKind::kTrajectory: {
      if (t.n < 2 || t.times_ns == nullptr || t.q == nullptr || t.qd == nullptr) {
        active_motion_id_ = t.motion_id;
        active_status_ = MotionStatus::REJECTED;
        emit(EventType::kMotionRejected, FaultReason::INTERNAL, 0, t.motion_id);
        return r;
      }
      // No new trajectory while faulted / following / parked ("converging, retry").
      if (fault_ != FaultReason::NONE || mode_ == Mode::SAFE_FOLLOW || mode_ == Mode::RX_SILENT) {
        active_motion_id_ = t.motion_id;
        active_status_ = MotionStatus::REJECTED;
        emit(EventType::kMotionRejected, fault_, 0, t.motion_id);
        return r;
      }
      const Vec6& q0 = t.q[0];
      const Vec6& qd0 = t.qd[0];
      // CAPTURE-or-REJECT: |q_cmd − q0|∞ > capture_tol → typed reject. Either the
      // gap to the trajectory start is small enough to be bridged by a bounded
      // capture splice, or the caller planned from a stale pose and must replan —
      // there is no third option that silently moves the arm to meet the plan.
      if (tick_engine::capture_would_reject(q_cmd_, q0, cfg_.tick)) {
        r.rejected_capture = true;
        active_motion_id_ = t.motion_id;
        active_status_ = MotionStatus::REJECTED;
        emit(EventType::kCaptureRejected, FaultReason::REJECTED_START_MISMATCH, 0, t.motion_id);
        emit(EventType::kMotionRejected, FaultReason::REJECTED_START_MISMATCH, 0, t.motion_id);
        return r;
      }
      // Generate the bounded capture splice (commanded → (q0, qd0)).
      tick_engine::generate_capture_path(q_cmd_, qd_cmd_, q0, qd0, cfg_.tick, capture_);
      if (!capture_.ok()) {
        r.rejected_capture = true;
        active_motion_id_ = t.motion_id;
        active_status_ = MotionStatus::REJECTED;
        emit(EventType::kMotionRejected, FaultReason::INTERNAL, 0, t.motion_id);
        return r;
      }
      // Preempt any current motion, then arm capture → trajectory.
      resolve_active(MotionStatus::PREEMPTED);
      captured_target_ = t;
      speed_scale_ = t.speed_scale;
      // per-motion settle overrides (fold into a TickEngineConfig copy for SettleDetector).
      motion_tick_cfg_ = cfg_.tick;
      motion_tick_cfg_.settle_tol_rad = t.settle_tol_rad;
      motion_tick_cfg_.settle_vel_eps_rad_s = t.settle_vel_eps_rad_s;
      motion_tick_cfg_.settle_timeout_s = t.settle_timeout_s;
      // terminal wire velocity (for the qd_end blend).
      for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
        qd_end_wire_[j] = t.qd[t.n - 1][j] * t.speed_scale;
      }
      // arm deadman / force-guard.
      deadman_armed_ = (t.deadman_s > 0.0);
      deadman_s_ = t.deadman_s;
      ticks_since_kick_ = 0;
      force_armed_ = (t.force_stop_n > 0.0);
      force_stop_n_ = t.force_stop_n;
      mode_ = Mode::CAPTURE;
      capture_idx_ = 0;
      active_motion_id_ = t.motion_id;
      active_status_ = MotionStatus::RUNNING;
      emit(EventType::kMotionRunning, FaultReason::NONE, 0, t.motion_id);
      r.accepted = true;
      return r;
    }
    default:
      return r;
  }
}

// ---------------------------------------------------------------------------
// trajectory install (after CAPTURE completes)
// ---------------------------------------------------------------------------
void TickCore::install_trajectory(const Target& t) {
  sampler_.bind(t.times_ns, t.q, t.qd, t.n);
  const std::int64_t dur = sampler_.duration_ns();
  traj_end_tau_ns_ = speed_scale_ > 0.0 ? static_cast<std::int64_t>(static_cast<double>(dur) / speed_scale_) : dur;
  // Start playback ONE tick in, not at tau=0. The CAPTURE path already delivered
  // the trajectory start (q0, qd0) exactly; sampling at tau=0 would hit the
  // sampler's before-start branch (q0 at REST, qd=0), stepping the commanded
  // velocity to 0 for one tick when qd0 ≠ 0 (mid-flight replan). Sampling at
  // tau=itp continues from (≈q0, ≈qd0) — continuous in both q and qd.
  const std::int64_t itp_ns = static_cast<std::int64_t>(std::llround(itp() * 1e9));
  tau_ns_ = itp_ns;
  traj_phase_ = TrajPhase::kPlaying;
  mode_ = Mode::TRAJECTORY;
}

// ---------------------------------------------------------------------------
// mode dispatch → desired (pre-slew) command; sets qd_cmd_/qdd_cmd_ (analytic)
// ---------------------------------------------------------------------------
Vec6 TickCore::dispatch_trajectory() {
  const std::int64_t itp_ns = static_cast<std::int64_t>(std::llround(itp() * 1e9));
  if (traj_phase_ == TrajPhase::kPlaying) {
    const tick_engine::HermiteSample s = sampler_.sample(tau_ns_, speed_scale_);
    qd_cmd_ = s.qd;
    qdd_cmd_ = s.qdd;
    const Vec6 q = s.q;
    tau_ns_ += itp_ns;  // the ONE motion-τ advance
    if (tau_ns_ >= traj_end_tau_ns_) {
      if (max_abs(qd_end_wire_) > cfg_.tick.settle_vel_eps_rad_s) {
        qd_blend_.plan(sampler_.q_last(), qd_end_wire_, cfg_.tick.qd_end_blend_min_s, itp());
        traj_phase_ = TrajPhase::kEndBlend;
        tau_blend_ns_ = 0;
      } else {
        traj_phase_ = TrajPhase::kSettling;
        q_hold_ = sampler_.q_last();
        settle_ = tick_engine::SettleDetector(motion_tick_cfg_);
        settle_.reset();
      }
    }
    return q;
  }
  if (traj_phase_ == TrajPhase::kEndBlend) {
    const tick_engine::HermiteSample s = qd_blend_.sample(tau_blend_ns_);
    qd_cmd_ = s.qd;
    // The blend's analytic accel (constant −qd_end/T), not 0, so a brake OUT of
    // the qd_end blend is acceleration-continuous.
    qdd_cmd_ = s.qdd;
    const Vec6 q = s.q;
    tau_blend_ns_ += itp_ns;
    if (tau_blend_ns_ >= qd_blend_.duration_ns()) {
      traj_phase_ = TrajPhase::kSettling;
      q_hold_ = qd_blend_.q_coast();
      settle_ = tick_engine::SettleDetector(motion_tick_cfg_);
      settle_.reset();
    }
    return q;
  }
  // kSettling: hold q_hold_ and wait for the measured state to converge.
  qd_cmd_ = Vec6{};
  qdd_cmd_ = Vec6{};
  const tick_engine::SettleState st = settle_.update(q_hold_, q_meas_, qd_est_);
  if (st == tick_engine::SettleState::kDone) {
    resolve_active(MotionStatus::DONE);
    go_hold();
  } else if (st == tick_engine::SettleState::kTimeout) {
    resolve_active(MotionStatus::SETTLE_TIMEOUT);
    go_hold();
  }
  return q_hold_;
}

Vec6 TickCore::dispatch_servo() {
  ++ticks_since_servo_set_;
  // Staleness re-issue. INERT as of the best-effort servo, and kept only until it is
  // deliberately removed: it re-sends a target identical to the one already set, which
  // leaves the Ruckig input unchanged and so changes nothing. It existed to zero the
  // velocity feedforward when the caller stopped refreshing, because the feedforward
  // would otherwise extrapolate past the last target and overshoot. There is no
  // feedforward now — a starved stream converges to its last target and rests on its
  // own, since the target velocity is always zero (Servo.RestsWhenLastTargetReissued).
  if (!servo_held_ && ticks_since_servo_set_ > servo_stale_ticks()) {
    servo_.set_target(servo_last_q_, servo_last_dur_);
    servo_held_ = true;
  }
  const tick_engine::ServoStep s = servo_.step();
  if (s.error) {
    enter_fault(FaultReason::INTERNAL, BumpReason::kFaultEntry, MotionStatus::FAULTED);
    return q_cmd_;  // slew-limited hold
  }
  qd_cmd_ = s.qd;
  qdd_cmd_ = s.qdd;
  return s.q;
}

Vec6 TickCore::dispatch_brake() {
  const tick_engine::BrakeStep s = brake_.step();
  if (s.error) {
    enter_fault(FaultReason::INTERNAL, BumpReason::kFaultEntry, MotionStatus::FAULTED);
    return q_cmd_;  // slew-limited hold
  }
  qd_cmd_ = s.qd;
  qdd_cmd_ = s.qdd;
  const Vec6 q = s.q;
  if (brake_.done(qd_est_)) {
    if (brake_is_motion_) {
      resolve_active(MotionStatus::DONE);
    }
    go_hold();
  }
  return q;
}

Vec6 TickCore::dispatch_safe_follow() {
  const std::int64_t itp_ns = static_cast<std::int64_t>(std::llround(itp() * 1e9));
  if (follow_phase_ == FollowPhase::kRamp) {
    const tick_engine::HermiteSample s = qd_blend_.sample(tau_follow_ns_);
    qd_cmd_ = s.qd;
    qdd_cmd_ = Vec6{};
    const Vec6 q = s.q;
    tau_follow_ns_ += itp_ns;
    if (tau_follow_ns_ >= qd_blend_.duration_ns()) {
      follow_phase_ = FollowPhase::kReanchor;
    }
    return q;
  }
  // kReanchor: step commanded → measured at ≤ rate·itp, with a 5° deadband so the
  // re-anchor does not chatter against ordinary tracking error.
  qd_cmd_ = Vec6{};
  qdd_cmd_ = Vec6{};
  if (!have_meas_) {
    return q_cmd_;  // no fresh measured (e.g. RX degraded) → hold
  }
  const double drift = max_abs_diff(q_cmd_, q_meas_);
  if (drift < cfg_.safe_follow_deadband_rad) {
    return q_cmd_;  // within the deadband → hold
  }
  const double max_step = cfg_.safe_follow_rate_rad_s * itp();
  Vec6 q{};
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    double delta = q_meas_[j] - q_cmd_[j];
    delta = std::clamp(delta, -max_step, max_step);
    q[j] = q_cmd_[j] + delta;
  }
  return q;
}

Vec6 TickCore::dispatch_mode() {
  switch (mode_) {
    case Mode::STREAM_DOWN:
    case Mode::PREROLL:
      qd_cmd_ = Vec6{};
      qdd_cmd_ = Vec6{};
      return have_meas_ ? q_meas_ : q_cmd_;
    case Mode::HOLD:
    case Mode::RX_SILENT:
      qd_cmd_ = Vec6{};
      qdd_cmd_ = Vec6{};
      return q_cmd_;
    case Mode::CAPTURE: {
      if (capture_idx_ < capture_.count) {
        const Vec6 q = capture_.q[static_cast<std::size_t>(capture_idx_)];
        qd_cmd_ = capture_.qd[static_cast<std::size_t>(capture_idx_)];
        // Seed the analytic capture accel so a brake OUT of CAPTURE is
        // acceleration-continuous; a zero here puts a curvature STEP into the brake.
        qdd_cmd_ = capture_.qdd[static_cast<std::size_t>(capture_idx_)];
        ++capture_idx_;
        return q;
      }
      // Capture complete → install and sample the trajectory this same tick.
      install_trajectory(captured_target_);
      return dispatch_trajectory();
    }
    case Mode::TRAJECTORY:
      return dispatch_trajectory();
    case Mode::SERVO:
      return dispatch_servo();
    case Mode::BRAKE:
      return dispatch_brake();
    case Mode::SAFE_FOLLOW:
      return dispatch_safe_follow();
  }
  qd_cmd_ = Vec6{};
  qdd_cmd_ = Vec6{};
  return q_cmd_;
}

// ---------------------------------------------------------------------------
// the one entry point
// ---------------------------------------------------------------------------
Command TickCore::tick(const RxSample* rx, const Target* pending, bool consume_superseded) {
  // 1) Ingest RX or count silence.
  if (rx != nullptr) {
    q_meas_ = rx->q_meas;
    qd_est_ = rx->qd_est;
    have_meas_ = true;
    ticks_since_rx_ = 0;
    // Fresh RX → clear the RX-silence conditions so recover()'s dwell can proceed.
    rx_degraded_entered_ = false;
    set_condition(kCondRxDegraded, false);
    process_gates(rx->gates);  // may enter_fault (bump epoch) BEFORE consume
  } else {
    ++ticks_since_rx_;
  }

  // 2) stop_j checked FIRST (universal preempt), then hold(), then kick().
  if (stop_requested_) {
    stop_requested_ = false;
    if (mode_ == Mode::HOLD || mode_ == Mode::CAPTURE || mode_ == Mode::TRAJECTORY || mode_ == Mode::SERVO) {
      begin_brake(MotionStatus::STOPPED, FaultReason::NONE);
    }
  }
  if (hold_requested_) {
    hold_requested_ = false;
    if (mode_ == Mode::CAPTURE || mode_ == Mode::TRAJECTORY || mode_ == Mode::SERVO) {
      begin_brake(MotionStatus::PREEMPTED, FaultReason::NONE);
    }
  }
  if (kicked_) {
    kicked_ = false;
    ticks_since_kick_ = 0;
  }
  if (heartbeated_) {
    heartbeated_ = false;
    supervisor_hb_armed_ = true;  // arm the SUPERVISOR_LOST watchdog on the first beat
    ticks_since_heartbeat_ = 0;
  }

  // 3) graduated RX-silence escalation.
  handle_rx_silence();

  // 4) deadman (TRAJECTORY only).
  handle_deadman();

  // 4b) supervisor liveness (any streaming mode; armed after the first beat).
  handle_supervisor_liveness();

  // 5) force-guard (needs a fresh RX sample).
  if (rx != nullptr) {
    handle_force_guard(*rx);
  }

  // 6) recovery anti-flap dwell.
  if (recover_pending_) {
    const bool clear = (conditions_ & kKillMask) == 0;
    const bool rx_ok = (mode_ != Mode::RX_SILENT) || (ticks_since_rx_ == 0);
    if (clear && rx_ok) {
      ++all_clear_ticks_;
      if (all_clear_ticks_ >= ms_to_ticks(cfg_.antiflap_dwell_ms)) {
        bump_epoch(BumpReason::kRecoveryComplete);
        fault_ = FaultReason::NONE;
        set_condition(kCondRxSilent, false);
        set_condition(kCondRxDegraded, false);
        recover_pending_ = false;
        all_clear_ticks_ = 0;
        rx_degraded_entered_ = false;
        go_hold();
        emit(EventType::kRecoveryComplete);
      }
    } else {
      all_clear_ticks_ = 0;
    }
  }

  // 7) consume the mailbox target (epoch-checked; AFTER fault processing).
  // `consume_superseded` is set by RealtimeCore when a stop_j was issued by the
  // caller AFTER this target was submitted — such a target is structurally
  // superseded by the stop and must not activate.
  if (pending != nullptr) {
    last_consume_ = consume(*pending, consume_superseded);
  } else {
    last_consume_ = ConsumeResult{};
  }

  // 8) mode dispatch → desired (pre-clamp, pre-slew) command.
  Vec6 q_desired = dispatch_mode();

  // 8b) Joint position clamp — the last thing between any mode and a soft-limit
  // overrun. Applied to EVERY mode rather than at submission, because a target
  // inside the limits can still be approached by a profile that overshoots, and
  // because the servo path takes setpoints straight from a caller with no
  // validation between. Clamp, never fault: the arm stops at its own stop and keeps
  // streaming, which the controller tolerates, whereas commanding past it is a servo
  // alarm needing a reset at the pendant. Defaults are ±inf, so this is inert until a
  // driver supplies the arm's real limits from its RobotProfile.
  //
  // Ordered BEFORE the slew clip on purpose: clamping can produce a step (if the
  // commanded pose is already outside, e.g. limits narrowed under a running core),
  // and the slew clip is what turns that step into a ramp.
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    q_desired[j] =
        std::clamp(q_desired[j], cfg_.tick.limits.pos_lo[j], cfg_.tick.limits.pos_hi[j]);
  }

  // 9) slew clip (never faults; count; sustained-clip diagnostic bit).
  const tick_engine::SlewResult sr = slew_.apply(q_desired);
  const bool sustained = slew_.sustained_clip();
  set_condition(kCondSustainedSlew, sustained);
  if (sustained && !sustained_slew_latched_) {
    emit(EventType::kSustainedSlew);
    sustained_slew_latched_ = true;
  } else if (!sustained) {
    sustained_slew_latched_ = false;
  }

  // 10) build the command; update the commanded anchor.
  Command cmd{};
  cmd.q_rad = sr.q;
  cmd.slew_clipped_joints = sr.clipped_joints;
  cmd.tx = streaming_ && (mode_ != Mode::RX_SILENT);
  cmd.is_last = false;
  q_cmd_ = sr.q;  // post-slew position anchor (qd_cmd_/qdd_cmd_ stay analytic for the brake seed)
  return cmd;
}

}  // namespace airo_fanuc::rt_core
