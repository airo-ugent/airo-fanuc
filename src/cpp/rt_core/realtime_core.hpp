// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — RealtimeCore: the thin I/O wrapper around TickCore.
//
// The RT thread: epoll{connected UDP 60015, timerfd(CLOCK_MONOTONIC, absolute)}
// → PLL-clocked TX (one send + one τ-advance per 8 ms window) → decode status
// (type-202 v3 no-force / type-204 v4 +force) → build RxSample (deg→rad, 5-sample
// finite-difference-slope qd_est, decoded gates, wrench) → TickCore::tick → encode
// type-201 (rad→deg, 9-pad, dataStyle 0xFFFF)
// → send → publish seqlock snapshot + drain events → rings. RT hygiene behind
// flags (SCHED_FIFO, mlockall) degrades gracefully without privilege; no CPU
// affinity is set, so no host core reservation is required. ZERO heap
// allocation on the tick path.
//
// This is the ONLY place a wall clock is read (PLL / snapshot ages). NO Python
// callbacks from the RT thread — Python polls the snapshot and drains events.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rt_core/pll.hpp"
#include "rt_core/rings.hpp"
#include "rt_core/rt_core_config.hpp"
#include "rt_core/snapshot.hpp"
#include "rt_core/tick_core.hpp"

namespace airo_fanuc::rt_core {

// Fixed-bucket latency histogram (alloc-free). Buckets are `bucket_ns` wide up to
// `kBuckets`; anything larger lands in the overflow bucket and updates max.
struct Histogram {
  static constexpr int kBuckets = 512;
  std::int64_t bucket_ns{50'000};  // 50 µs default
  std::array<std::uint64_t, kBuckets + 1> counts{};
  std::uint64_t total{0};
  std::int64_t max_ns{0};

  void add(std::int64_t v) {
    if (v > max_ns) max_ns = v;
    std::int64_t idx = v / bucket_ns;
    if (idx < 0) idx = 0;
    if (idx > kBuckets) idx = kBuckets;
    counts[static_cast<std::size_t>(idx)]++;
    total++;
  }
  // Approximate percentile (upper edge of the containing bucket, ns).
  std::int64_t percentile(double p) const {
    if (total == 0) return 0;
    const std::uint64_t target = static_cast<std::uint64_t>(p * static_cast<double>(total) + 0.5);
    std::uint64_t cum = 0;
    for (int i = 0; i <= kBuckets; ++i) {
      cum += counts[static_cast<std::size_t>(i)];
      if (cum >= target) return static_cast<std::int64_t>(i + 1) * bucket_ns;
    }
    return max_ns;
  }
};

struct TimingStats {
  double tx_interval_p50_ms{0}, tx_interval_p99_ms{0}, tx_interval_p999_ms{0}, tx_interval_max_ms{0};
  double rx2tx_p50_us{0}, rx2tx_p99_us{0}, rx2tx_p999_us{0}, rx2tx_max_us{0};
  std::uint64_t tick_count{0};
  std::uint64_t tx_count{0};
  std::uint64_t tau_advance_count{0};
  std::uint64_t parked_ticks{0};
  std::uint64_t missed_rx_ticks{0};
  std::uint64_t rx_seq_gaps{0};
  std::uint64_t double_send_guard{0};
  std::uint64_t cpu_migrations{0};
};

class RealtimeCore {
 public:
  explicit RealtimeCore(const RtCoreConfig& cfg);
  ~RealtimeCore();

  RealtimeCore(const RealtimeCore&) = delete;
  RealtimeCore& operator=(const RealtimeCore&) = delete;

  // Open the socket, do the GetCapability-negotiated SM handshake (defensive Stop
  // → GetCapability type-7 → adopt available_version + sampling_rate → Start), spawn
  // the RT thread. Returns false if the socket could not be opened. Non-blocking
  // w.r.t. PREROLL: use wait_ready() to block until motion_possible (STREAMING).
  bool start();
  // Signal stop → terminal is_last command → join RT thread → close fds. Wedge-free.
  void stop();
  bool running() const { return running_.load(std::memory_order_acquire); }

  // Block until the core reaches STREAMING (HOLD) or timeout. Returns ready.
  bool wait_ready(double timeout_s);

  // --- submission (any thread; serialised by submit_mu_) ---
  std::uint64_t submit_trajectory(const std::vector<std::int64_t>& times_ns, const std::vector<Vec6>& q,
                                  const std::vector<Vec6>& qd, double speed_scale, double settle_tol_rad,
                                  double settle_vel_eps_rad_s, double settle_timeout_s, double force_stop_n,
                                  double deadman_s);
  std::uint64_t submit_servo(const Vec6& q, double duration_s);
  // Feed-forward servo: qd/qdd become the Ruckig target velocity/acceleration (see the
  // Target::servo_has_ff note). Same replace-not-queue / distance-guard semantics.
  std::uint64_t submit_servo(const Vec6& q, const Vec6& qd, const Vec6& qdd, double duration_s);
  std::uint64_t submit_brake();
  std::uint64_t submit_hold();
  void stop_j();
  void hold();
  void recover();
  void kick();
  void heartbeat();  // supervisor-liveness beat → TickCore SUPERVISOR_LOST watchdog reset

  // --- getters (never raise) ---
  StateSnapshot snapshot() const { return snap_.read(); }
  std::size_t drain_events(Event* out, std::size_t max);
  int event_fd() const { return event_fd_; }
  TimingStats timing() const;
  bool joints_at_wall(std::int64_t wall_ns, Vec6& out) const;
  MotionStatus motion_status(std::uint64_t id) const;

 private:
  // Owned trajectory buffer (allocated off-RT in submit_trajectory).
  struct TrajBuffer {
    std::vector<std::int64_t> times;
    std::vector<Vec6> q;
    std::vector<Vec6> qd;
  };

  void rt_main_();
  void apply_rt_hygiene_();
  bool open_socket_();
  void send_handshake_();
  void send_stop_packet_();
  bool arm_timer_abs_(std::int64_t abs_ns);
  void publish_snapshot_(const RxSample* rx, const Command& cmd, std::int64_t tick_mono_ns, bool rx_fresh);
  void drain_tick_events_();  // move TickCore events → Python-facing ring + resolution table
  void reap_retired_();         // free retired trajectory buffers (off-RT); locks submit_mu_
  void reap_retired_locked_();  // same, but caller already holds submit_mu_ (submit paths)
  std::uint64_t enqueue_(const Target& t, const void* owner);
  void record_resolution_(std::uint64_t id, MotionStatus st);

  RtCoreConfig cfg_;
  TickCore tick_core_;
  Pll pll_;

  int sockfd_{-1};
  int timerfd_{-1};
  int epollfd_{-1};
  int event_fd_{-1};
  int wake_fd_{-1};  // eventfd to wake the RT thread out of epoll at stop

  std::thread rt_thread_;
  std::atomic<bool> stop_flag_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> streaming_ready_{false};

  // control-op atomics (drained at tick start on the RT thread).
  std::atomic<bool> stop_req_{false};
  std::atomic<bool> hold_req_{false};
  std::atomic<bool> recover_req_{false};
  std::atomic<bool> kick_req_{false};
  std::atomic<bool> heartbeat_req_{false};

  // Stream Motion capability negotiated at handshake (GetCapability type-7 reply).
  // 0 = no reply seen (fell back to cfg_.sm_version). Set once in send_handshake_.
  std::atomic<std::uint32_t> sm_negotiated_version_{0};
  std::atomic<std::uint32_t> sm_sampling_rate_ms_{0};

  // Caller-side stop generation. Incremented in stop_j() ON THE CALLER THREAD (so
  // a subsequent submit on the caller side sees it immediately) and stamped onto
  // every submitted Target. At consume the RT thread compares the target's stamp
  // to the live value: a mismatch means a stop_j was issued AFTER the target was
  // submitted → the stop supersedes it. This captures caller-causal order, which
  // distinguishes submit-then-stop (the stop must win → reject the target) from
  // legitimate brake-then-submit (stop, then a fresh trajectory → accept) even
  // when both land in the same 8 ms tick. A caller-thread counter is REQUIRED: a
  // counter bumped on the RT thread (in request_stop) lags the caller by up to one
  // tick and would wrongly reject a brake-then-submit issued from an
  // already-steady pose.
  std::atomic<std::uint64_t> stop_gen_{0};

  // mailbox (Python → RT); producers serialised by submit_mu_.
  SpscRing<Target, 16> mailbox_;
  std::mutex submit_mu_;
  std::atomic<std::uint64_t> next_motion_id_{1};
  std::atomic<std::uint64_t> published_epoch_{0};  // mirrors TickCore epoch for submit tagging

  // trajectory buffer ownership.
  std::vector<std::unique_ptr<TrajBuffer>> live_buffers_;  // guarded by submit_mu_
  SpscRing<const void*, 64> retire_ring_;                  // RT → off-RT (pointers to free)
  const void* active_traj_owner_{nullptr};                 // RT-owned

  // events (RT → Python).
  SpscRing<Event, 1024> py_events_;

  // qd_est finite-difference-slope history (RT-owned).
  static constexpr int kVelTaps = 5;
  std::array<std::int64_t, kVelTaps> vel_t_{};
  std::array<Vec6, kVelTaps> vel_q_{};
  int vel_count_{0};
  int vel_head_{0};

  // TX sequence (independent monotonic, reset per handshake).
  std::uint32_t tx_seq_{0};
  std::uint32_t last_rx_seq_{0};

  // timing.
  Histogram hist_tx_interval_{};
  Histogram hist_rx2tx_{};
  std::atomic<std::uint64_t> tick_count_{0};
  std::atomic<std::uint64_t> tx_count_{0};
  std::atomic<std::uint64_t> tau_advance_count_{0};
  std::atomic<std::uint64_t> parked_ticks_{0};
  std::atomic<std::uint64_t> missed_rx_ticks_{0};
  std::atomic<std::uint64_t> rx_seq_gaps_{0};
  std::atomic<std::uint64_t> double_send_guard_{0};
  std::atomic<std::uint64_t> cpu_migrations_{0};
  // Histograms are RT-thread-owned; timing() reads them racily (a torn bucket
  // count is harmless for diagnostics — no lock on the hot path).

  // seqlock snapshot.
  Seqlock<StateSnapshot> snap_;

  // joints_at ring.
  JointsAtRing joints_ring_;

  // motion resolution table (RT writer, Python readers).
  struct ResolutionSlot {
    std::atomic<std::uint64_t> id{0};
    std::atomic<std::uint32_t> status{0};
  };
  static constexpr int kResolutions = 64;
  std::array<ResolutionSlot, kResolutions> resolutions_{};
  int resolution_head_{0};  // RT-owned
  std::atomic<std::uint64_t> active_id_pub_{0};
  std::atomic<std::uint32_t> active_status_pub_{0};
};

}  // namespace airo_fanuc::rt_core
