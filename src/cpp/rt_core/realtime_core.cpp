// SPDX-License-Identifier: Apache-2.0
//
// RealtimeCore — the RT thread / I/O wrapper. See realtime_core.hpp.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // sched_getcpu, MCL_ONFAULT
#endif

#include "rt_core/realtime_core.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>

#include "codec/codec.hpp"
#include "stream_motion/packets.hpp"

namespace airo_fanuc::rt_core {

namespace {


inline std::int64_t now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

inline std::int64_t realtime_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// Big-endian u32 store/load (Stream Motion is big-endian on the wire).
inline void be_store_u32(std::uint8_t* p, std::uint32_t v) {
  p[0] = static_cast<std::uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<std::uint8_t>(v & 0xFF);
}
inline std::uint32_t be_load_u32(const std::uint8_t* p) {
  return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
         (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
}

MotionStatus event_to_status(EventType t) {
  switch (t) {
    case EventType::kMotionDone:
      return MotionStatus::DONE;
    case EventType::kMotionSettleTimeout:
      return MotionStatus::SETTLE_TIMEOUT;
    case EventType::kMotionStopped:
      return MotionStatus::STOPPED;
    case EventType::kMotionPreempted:
      return MotionStatus::PREEMPTED;
    case EventType::kMotionFaulted:
      return MotionStatus::FAULTED;
    case EventType::kMotionRejected:
      return MotionStatus::REJECTED;
    default:
      return MotionStatus::RUNNING;
  }
}

// Finiteness gate for a decoded status packet, applied before any field reaches the
// control path. Checks exactly the wire floats the core consumes, over the six axes the
// arm has: what a controller writes into the three unused axis slots or into `current`
// is not something this driver reads, so it is not made a condition for using a packet.
//
// This is the only place a non-finite value can be stopped. std::clamp, std::min,
// std::max and a bare `>` comparison all pass a NaN through: the +-position clamp does
// not clip it, the slew clip's two comparisons are both false so it emits the value and
// adopts it as the next reference, `safety_scale < safety_scale_min` is false so the
// SAFETY_CLAMP kill gate reads clear, and `|F| > force_stop_n` is false so the force
// guard reads untripped.
bool rx_floats_finite(const codec::RobotStatusView& v) {
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    if (!std::isfinite(v.joint_angle[j]) || !std::isfinite(v.position[j])) {
      return false;
    }
  }
  if (!std::isfinite(v.safety_scale)) {
    return false;
  }
  if (v.fs_type == 1u || v.fs_type == 2u) {
    return std::isfinite(v.force_x) && std::isfinite(v.force_y) && std::isfinite(v.force_z) &&
           std::isfinite(v.moment_x) && std::isfinite(v.moment_y) && std::isfinite(v.moment_z);
  }
  return true;
}

bool is_resolution_event(EventType t) {
  switch (t) {
    case EventType::kMotionDone:
    case EventType::kMotionSettleTimeout:
    case EventType::kMotionStopped:
    case EventType::kMotionPreempted:
    case EventType::kMotionFaulted:
    case EventType::kMotionRejected:
      return true;
    default:
      return false;
  }
}

}  // namespace

RealtimeCore::RealtimeCore(const RtCoreConfig& cfg)
    : cfg_(cfg),
      tick_core_(cfg),
      pll_(static_cast<std::int64_t>(cfg.pll_rx_lead_us * 1000.0), cfg.pll_kp,
           static_cast<std::int64_t>(std::llround(cfg.tick.itp_s * 1e9))) {
  hist_tx_interval_.bucket_ns = 50'000;   // 50 µs buckets over ~25 ms range
  hist_rx2tx_.bucket_ns = 10'000;         // 10 µs buckets over ~5 ms range
}

RealtimeCore::~RealtimeCore() { stop(); }

// ---------------------------------------------------------------------------
// socket + handshake
// ---------------------------------------------------------------------------
bool RealtimeCore::open_socket_() {
  sockfd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (sockfd_ < 0) {
    return false;
  }
  sockaddr_in peer{};
  peer.sin_family = AF_INET;
  peer.sin_port = htons(cfg_.sm_port);
  if (inet_pton(AF_INET, cfg_.host.c_str(), &peer.sin_addr) != 1) {
    ::close(sockfd_);
    sockfd_ = -1;
    return false;
  }
  if (::connect(sockfd_, reinterpret_cast<sockaddr*>(&peer), sizeof(peer)) < 0) {
    ::close(sockfd_);
    sockfd_ = -1;
    return false;
  }
  return true;
}

void RealtimeCore::send_stop_packet_() {
  std::uint8_t buf[8];
  be_store_u32(buf, stream_motion::kStopPacketType);
  be_store_u32(buf + 4, cfg_.sm_version);
  (void)::send(sockfd_, buf, sizeof(buf), 0);
}

void RealtimeCore::send_handshake_() {
  // Handshake: defensive Stop → drain → GetCapability → adopt the controller's
  // available_version + sampling_rate → Start. The real R-30iB requires the
  // GetCapability (type-7) exchange before it honours a StartPacket and streams
  // status: a bare Stop→Start with the version pinned is enough for a software
  // stand-in, but on hardware it leaves motion_possible unasserted forever, so
  // preroll never completes. FSConfig(v≥4) is not sent (v3 = no force).
  send_stop_packet_();
  // Drain any stale datagrams (including a stale capability reply).
  std::uint8_t scratch[1024];
  while (::recv(sockfd_, scratch, sizeof(scratch), 0) > 0) {
  }

  // GetCapability request (type 7, 28 B big-endian): id=1 matches FANUC's default;
  // every read-only field is zero on the request (see stream_motion::
  // ControllerCapabilityPacket / wire.encode_get_capability_packet).
  std::uint32_t negotiated = cfg_.sm_version;
  {
    std::uint8_t req[28] = {};
    be_store_u32(req + 0, stream_motion::kGetCapabilityPacketType);  // 7
    be_store_u32(req + 4, cfg_.sm_version);
    be_store_u32(req + 8, 1u);  // id
    (void)::send(sockfd_, req, sizeof(req), 0);

    // Bounded wait for the type-7 reply; adopt available_version + sampling_rate.
    // Best-effort: if no reply arrives we fall back to cfg_.sm_version and let the
    // preroll timeout report the failure, rather than failing start() here — the
    // timeout is the one place that already reports "the controller never became
    // ready", and it covers every cause, not just a missing capability reply.
    const std::int64_t deadline = now_ns() + 1'500'000'000LL;  // 1.5 s
    std::uint8_t rep[64];
    while (now_ns() < deadline) {
      const ssize_t r = ::recv(sockfd_, rep, sizeof(rep), 0);
      if (r >= 28 && be_load_u32(rep) == stream_motion::kGetCapabilityPacketType) {
        const std::uint32_t avail = be_load_u32(rep + 20);  // available_version
        const std::uint32_t srate = be_load_u32(rep + 12);  // sampling_rate [ms]
        if (avail != 0u) {
          negotiated = avail;
        }
        sm_negotiated_version_.store(negotiated, std::memory_order_relaxed);
        sm_sampling_rate_ms_.store(srate, std::memory_order_relaxed);
        break;
      }
      if (r < 0) {
        const timespec ts{0, 2'000'000L};  // 2 ms
        nanosleep(&ts, nullptr);
      }
    }
  }

  // Start with the negotiated version.
  std::uint8_t buf[8];
  be_store_u32(buf, stream_motion::kStartPacketType);
  be_store_u32(buf + 4, negotiated);
  (void)::send(sockfd_, buf, sizeof(buf), 0);
  tx_seq_ = 0;  // independent monotonic TX seq, reset per handshake
}

// ---------------------------------------------------------------------------
// start / stop / wait_ready
// ---------------------------------------------------------------------------
bool RealtimeCore::start() {
  if (running_.load(std::memory_order_acquire)) {
    return true;
  }
  if (!open_socket_()) {
    return false;
  }
  timerfd_ = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
  if (timerfd_ < 0) {
    ::close(sockfd_);
    sockfd_ = -1;
    return false;
  }
  event_fd_ = eventfd(0, EFD_NONBLOCK);
  wake_fd_ = eventfd(0, EFD_NONBLOCK);
  epollfd_ = epoll_create1(0);
  epoll_event ev{};
  ev.events = EPOLLIN;
  ev.data.fd = sockfd_;
  epoll_ctl(epollfd_, EPOLL_CTL_ADD, sockfd_, &ev);
  ev.data.fd = timerfd_;
  epoll_ctl(epollfd_, EPOLL_CTL_ADD, timerfd_, &ev);
  ev.data.fd = wake_fd_;
  epoll_ctl(epollfd_, EPOLL_CTL_ADD, wake_fd_, &ev);

  send_handshake_();

  stop_flag_.store(false, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  rt_thread_ = std::thread([this] { rt_main_(); });
  return true;
}

void RealtimeCore::stop() {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }
  stop_flag_.store(true, std::memory_order_release);
  if (wake_fd_ >= 0) {
    std::uint64_t one = 1;
    { ssize_t wr = ::write(wake_fd_, &one, sizeof(one)); (void)wr; }
  }
  if (rt_thread_.joinable()) {
    rt_thread_.join();
  }
  running_.store(false, std::memory_order_release);

  if (epollfd_ >= 0) {
    ::close(epollfd_);
    epollfd_ = -1;
  }
  if (timerfd_ >= 0) {
    ::close(timerfd_);
    timerfd_ = -1;
  }
  if (sockfd_ >= 0) {
    ::close(sockfd_);
    sockfd_ = -1;
  }
  if (event_fd_ >= 0) {
    ::close(event_fd_);
    event_fd_ = -1;
  }
  if (wake_fd_ >= 0) {
    ::close(wake_fd_);
    wake_fd_ = -1;
  }
  reap_retired_();
  std::lock_guard<std::mutex> lk(submit_mu_);
  live_buffers_.clear();
}

bool RealtimeCore::wait_ready(double timeout_s) {
  const std::int64_t deadline = now_ns() + static_cast<std::int64_t>(timeout_s * 1e9);
  while (!streaming_ready_.load(std::memory_order_acquire)) {
    if (!running_.load(std::memory_order_acquire)) {
      return false;
    }
    if (now_ns() >= deadline) {
      return false;
    }
    struct timespec ts {
      0, 1'000'000
    };  // 1 ms
    nanosleep(&ts, nullptr);
  }
  return true;
}

bool RealtimeCore::arm_timer_abs_(std::int64_t abs_ns) {
  itimerspec its{};
  its.it_value.tv_sec = abs_ns / 1'000'000'000LL;
  its.it_value.tv_nsec = abs_ns % 1'000'000'000LL;
  return timerfd_settime(timerfd_, TFD_TIMER_ABSTIME, &its, nullptr) == 0;
}

// ---------------------------------------------------------------------------
// RT hygiene. Both are best-effort: an unprivileged process simply keeps the
// default scheduling class and swappable pages, so bring-up never depends on
// having been granted CAP_SYS_NICE / MEMLOCK. No CPU affinity is set — this
// driver requires no host core reservation.
// ---------------------------------------------------------------------------
void RealtimeCore::apply_rt_hygiene_() {
  if (cfg_.sched_fifo) {
    struct sched_param sp;
    std::memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg_.rt_priority;
    (void)pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);  // tolerate failure
  }
  if (cfg_.mlock) {
    (void)mlockall(MCL_CURRENT | MCL_ONFAULT);  // tolerate failure
  }
}

// ---------------------------------------------------------------------------
// velocity estimate (5-sample finite-difference slope over ~32 ms)
// ---------------------------------------------------------------------------
namespace {
Vec6 slope_estimate(const std::array<std::int64_t, 5>& t, const std::array<Vec6, 5>& q, int count, int head) {
  Vec6 out{};
  if (count < 2) {
    return out;
  }
  // oldest = the entry `count` back from head; newest = head-1.
  const int newest = (head - 1 + 5) % 5;
  const int oldest = (head - count + 5 * 5) % 5;
  const double dt = static_cast<double>(t[static_cast<std::size_t>(newest)] - t[static_cast<std::size_t>(oldest)]) / 1e9;
  if (dt <= 0.0) {
    return out;
  }
  for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
    out[j] = (q[static_cast<std::size_t>(newest)][j] - q[static_cast<std::size_t>(oldest)][j]) / dt;
  }
  return out;
}
}  // namespace

// ---------------------------------------------------------------------------
// the RT thread
// ---------------------------------------------------------------------------
void RealtimeCore::rt_main_() {
  apply_rt_hygiene_();

  const std::int64_t tick_ns = static_cast<std::int64_t>(std::llround(cfg_.tick.itp_s * 1e9));

  // Prefault a chunk of stack (page faults happen now, not mid-tick).
  {
    volatile char stackbuf[256 * 1024];
    for (std::size_t i = 0; i < sizeof(stackbuf); i += 4096) stackbuf[i] = 0;
    (void)stackbuf[0];
  }

  std::uint8_t rxbuf[1024];
  RxSample rx{};
  bool have_rx = false;      // fresh RX since last tick
  bool ever_rx = false;      // any RX ever
  std::int64_t last_rx_mono = 0;
  std::int64_t last_send_mono = 0;
  int last_cpu = -1;

  std::int64_t scheduled_tick = now_ns() + tick_ns;
  arm_timer_abs_(scheduled_tick);

  epoll_event events[8];
  while (!stop_flag_.load(std::memory_order_relaxed)) {
    const int n = epoll_wait(epollfd_, events, 8, -1);
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }

    bool timer_ready = false;
    for (int i = 0; i < n; ++i) {
      const int fd = events[i].data.fd;
      if (fd == sockfd_) {
        const std::int64_t rx_mono = now_ns();
        int got = 0;
        for (;;) {
          const ssize_t r = ::recv(sockfd_, rxbuf, sizeof(rxbuf), 0);
          if (r < 0) break;  // EAGAIN → drained
          // Dispatch on packet type + size (pre-checked so decode never throws).
          // A Stream Motion v3 controller streams type-202 (388 B, no force); the
          // newer type-204 (416 B, +force) is accepted too, for a v4 / firmware-P84
          // controller. Anything else (short / unknown) is dropped.
          const std::size_t rlen = static_cast<std::size_t>(r);
          const std::uint32_t rx_ptype = (rlen >= 4) ? be_load_u32(rxbuf) : 0u;
          codec::RobotStatusView v{};
          if (rx_ptype == 204u && rlen >= codec::kStatus204PacketSize) {
            v = codec::decode_status_204(rxbuf, rlen);
          } else if (rx_ptype == 202u && rlen >= codec::kStatusV3PacketSize) {
            v = codec::decode_status_v3(rxbuf, rlen);
          } else {
            continue;  // not a status packet we understand
          }
          // A status carrying a non-finite float is DROPPED, exactly like a status that
          // never arrived, and never latches a fault. Dropping is already handled end to
          // end: the graduated RX-silence ladder absorbs an isolated loss at no cost and
          // escalates a persistent one to SAFE_FOLLOW and then to a parked TX. If the
          // very FIRST packet is affected, begin_streaming simply never runs and the
          // preroll timeout reports that the controller never became ready. Faulting
          // instead would turn one corrupt datagram — an IPv4 UDP checksum is optional
          // and commonly zero — into an operator-required recovery, and on that first
          // packet it would enter SAFE_FOLLOW with the commanded anchor still at its
          // default zero vector and no measurement to re-anchor onto.
          if (!rx_floats_finite(v)) {
            rx_nonfinite_drops_.fetch_add(1, std::memory_order_relaxed);
            continue;
          }
          // deg → rad measured.
          Vec6 q_meas{};
          for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
            q_meas[j] = tick_engine::deg2rad(v.joint_angle[j]);
          }
          // velocity history.
          vel_t_[static_cast<std::size_t>(vel_head_)] = rx_mono;
          vel_q_[static_cast<std::size_t>(vel_head_)] = q_meas;
          vel_head_ = (vel_head_ + 1) % kVelTaps;
          if (vel_count_ < kVelTaps) ++vel_count_;
          const Vec6 qd_est = slope_estimate(vel_t_, vel_q_, vel_count_, vel_head_);

          rx = RxSample{};
          rx.q_meas = q_meas;
          rx.qd_est = qd_est;
          rx.gates = decode_gates(v.status, v.robot_status, v.contact_stop_status, v.safety_scale,
                                  cfg_.safety_scale_min);
          rx.safety_scale = v.safety_scale;
          rx.contact_stop_status = v.contact_stop_status;
          rx.fx = v.force_x;
          rx.fy = v.force_y;
          rx.fz = v.force_z;
          rx.mx = v.moment_x;
          rx.my = v.moment_y;
          rx.mz = v.moment_z;
          rx.fs_type = v.fs_type;
          rx.wrench_valid = (v.fs_type == 1u || v.fs_type == 2u);
          rx.rx_seq = v.sequence_no;
          rx.ctrl_time_stamp_ms = v.time_stamp;
          rx.cart = v.position;
          rx.rx_mono_ns = rx_mono;

          if (ever_rx && v.sequence_no != last_rx_seq_ + 1) {
            rx_seq_gaps_.fetch_add(1, std::memory_order_relaxed);
          }
          last_rx_seq_ = v.sequence_no;
          ever_rx = true;
          have_rx = true;
          last_rx_mono = rx_mono;

          // joints_at ring (mono, q) for camera FK-at-shutter.
          joints_ring_.push(rx_mono, q_meas);

          // PREROLL → HOLD: begin streaming on the first motion_possible RX.
          if (!streaming_ready_.load(std::memory_order_relaxed) && rx.gates.motion_possible) {
            tick_core_.begin_streaming(q_meas);
            published_epoch_.store(tick_core_.epoch(), std::memory_order_release);
            streaming_ready_.store(true, std::memory_order_release);
          }
          ++got;
        }
        (void)got;
      } else if (fd == timerfd_) {
        timer_ready = true;
      } else if (fd == wake_fd_) {
        std::uint64_t drain;
        { ssize_t rr = ::read(wake_fd_, &drain, sizeof(drain)); (void)rr; }
      }
    }

    if (!timer_ready) {
      continue;
    }

    // TIMER TICK — exactly one per firing.
    std::uint64_t expirations = 0;
    if (::read(timerfd_, &expirations, sizeof(expirations)) != sizeof(expirations)) {
      continue;
    }

    // PLL phase servo → next tick. `now` is what lets the servo re-base a deadline the
    // clock has already passed, instead of handing arm_timer_abs_ a past time, which
    // fires immediately and turns one lost window into a catch-up burst.
    const std::int64_t next_tick = pll_.next_tick(scheduled_tick, have_rx, last_rx_mono, now_ns());
    if (pll_.last_skipped_windows() > 0) {
      skipped_tick_windows_.fetch_add(static_cast<std::uint64_t>(pll_.last_skipped_windows()),
                                     std::memory_order_relaxed);
    }
    scheduled_tick = next_tick;
    arm_timer_abs_(next_tick);

    // Forward control-op atomics to TickCore (RT thread).
    if (stop_req_.exchange(false, std::memory_order_acq_rel)) tick_core_.request_stop();
    if (hold_req_.exchange(false, std::memory_order_acq_rel)) tick_core_.request_hold();
    if (recover_req_.exchange(false, std::memory_order_acq_rel)) tick_core_.request_recover();
    if (kick_req_.exchange(false, std::memory_order_acq_rel)) tick_core_.kick();
    if (heartbeat_req_.exchange(false, std::memory_order_acq_rel)) tick_core_.heartbeat();

    // Drain the mailbox → latest-wins pending; retire superseded trajectory
    // buffers AND resolve superseded motion_ids. A coalesced-away target never
    // reaches TickCore::consume (only `last` does), so it has NO other resolution
    // path — record it PREEMPTED here + emit a synthetic kMotionPreempted event,
    // otherwise its MotionHandle hangs PENDING forever.
    Target last{};
    bool have_pending = false;
    bool superseded_events = false;
    Target t{};
    while (mailbox_.pop(t)) {
      if (have_pending) {
        // `last` is superseded by the just-popped `t` (any kind).
        if (last.kind == TargetKind::kTrajectory && last.buffer_owner != nullptr) {
          retire_ring_.push(last.buffer_owner);  // RT-thread producer — the ONLY producer
        }
        if (last.motion_id != 0) {
          record_resolution_(last.motion_id, MotionStatus::PREEMPTED);
          Event ev{};
          ev.type = EventType::kMotionPreempted;
          ev.motion_id = last.motion_id;
          ev.epoch = tick_core_.epoch();
          py_events_.push(ev);  // drop-if-full (RT-thread producer)
          superseded_events = true;
        }
      }
      last = t;
      have_pending = true;
    }
    if (superseded_events && event_fd_ >= 0) {
      std::uint64_t one = 1;
      { ssize_t wr = ::write(event_fd_, &one, sizeof(one)); (void)wr; }
    }
    const Target* pending = have_pending ? &last : nullptr;

    // Stop precedence: a stop_j issued AFTER this target was submitted (its
    // stamped stop_gen is behind the live counter) supersedes it — the stop must
    // win, so the target must not activate. Computed here (RealtimeCore owns the
    // caller-side counter) and passed into consume via tick().
    const bool consume_superseded =
        have_pending && (last.stop_gen != stop_gen_.load(std::memory_order_acquire));

    // THE ONE tick + THE ONE τ-advance for this window.
    const RxSample* rx_ptr = have_rx ? &rx : nullptr;
    const Command cmd = tick_core_.tick(rx_ptr, pending, consume_superseded);
    tau_advance_count_.fetch_add(1, std::memory_order_relaxed);
    tick_count_.fetch_add(1, std::memory_order_relaxed);
    published_epoch_.store(tick_core_.epoch(), std::memory_order_release);

    // Retire trajectory buffers per the consume result.
    if (have_pending) {
      const ConsumeResult& cr = tick_core_.last_consume();
      if (last.kind == TargetKind::kTrajectory) {
        if (cr.accepted) {
          if (active_traj_owner_ != nullptr) retire_ring_.push(active_traj_owner_);
          active_traj_owner_ = last.buffer_owner;
        } else if (last.buffer_owner != nullptr) {
          retire_ring_.push(last.buffer_owner);
        }
      } else if (cr.accepted && active_traj_owner_ != nullptr &&
                 (last.kind == TargetKind::kServo || last.kind == TargetKind::kBrake ||
                  last.kind == TargetKind::kHold)) {
        retire_ring_.push(active_traj_owner_);
        active_traj_owner_ = nullptr;
      }
    }

    // Encode + send (only when TickCore says TX; one send per window).
    const std::int64_t send_mono = now_ns();
    if (cmd.tx) {
      std::array<double, 9> pos_deg{};
      for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
        pos_deg[j] = tick_engine::rad2deg(cmd.q_rad[j]);
      }
      ++tx_seq_;  // independent monotonic TX seq
      const auto pkt = codec::encode_command_packet(tx_seq_, cmd.is_last, /*do_motn_ctrl=*/1, pos_deg);
      (void)::send(sockfd_, pkt.data(), pkt.size(), 0);
      tx_count_.fetch_add(1, std::memory_order_relaxed);
      // Double-send guard: two sends must never fall inside one 8 ms window.
      if (last_send_mono != 0 && (send_mono - last_send_mono) < (tick_ns / 4)) {
        double_send_guard_.fetch_add(1, std::memory_order_relaxed);
      }
      // Metrics (post-first-send).
      if (last_send_mono != 0) {
        hist_tx_interval_.add(send_mono - last_send_mono);
      }
      last_send_mono = send_mono;
      if (ever_rx) {
        hist_rx2tx_.add(send_mono - last_rx_mono);
      }
    } else {
      parked_ticks_.fetch_add(1, std::memory_order_relaxed);
    }

    if (!have_rx) {
      missed_rx_ticks_.fetch_add(1, std::memory_order_relaxed);
    }
    {
      const int cpu = sched_getcpu();
      if (last_cpu >= 0 && cpu != last_cpu) cpu_migrations_.fetch_add(1, std::memory_order_relaxed);
      last_cpu = cpu;
    }

    // Publish snapshot + drain events.
    publish_snapshot_(rx_ptr, cmd, send_mono, have_rx);
    drain_tick_events_();

    have_rx = false;  // reset per-window freshness
  }

  // Wedge-free teardown: one terminal is_last command (if streaming), then Stop.
  if (streaming_ready_.load(std::memory_order_relaxed)) {
    std::array<double, 9> pos_deg{};
    const Vec6& q = tick_core_.q_cmd();
    for (std::size_t j = 0; j < static_cast<std::size_t>(kNumJoints); ++j) {
      pos_deg[j] = tick_engine::rad2deg(q[j]);
    }
    ++tx_seq_;
    const auto pkt = codec::encode_command_packet(tx_seq_, /*is_last=*/true, /*do_motn_ctrl=*/1, pos_deg);
    (void)::send(sockfd_, pkt.data(), pkt.size(), 0);
  }
  send_stop_packet_();
}

// ---------------------------------------------------------------------------
// snapshot + events
// ---------------------------------------------------------------------------
void RealtimeCore::publish_snapshot_(const RxSample* rx, const Command& cmd, std::int64_t tick_mono_ns,
                                     bool rx_fresh) {
  StateSnapshot s{};
  s.q_cmd = cmd.q_rad;
  s.qd_cmd = tick_core_.qd_cmd();
  s.qdd_cmd = tick_core_.qdd_cmd();
  s.cmd_tick = tick_core_.tick_no();
  if (rx != nullptr) {
    s.q_meas = rx->q_meas;
    s.qd_est = rx->qd_est;
    s.cart = rx->cart;
    s.rx_seq = rx->rx_seq;
    s.ctrl_time_stamp_ms = rx->ctrl_time_stamp_ms;
    s.rx_mono_ns = rx->rx_mono_ns;
    s.e_stopped = rx->gates.e_stopped;
    s.in_error = rx->gates.in_error;
    s.tp_enabled = rx->gates.tp_enabled;
    s.motion_possible = rx->gates.motion_possible;
    s.motion_in_progress = rx->gates.motion_in_progress;
    s.contact_stop_status = rx->contact_stop_status;
    s.safety_scale = rx->safety_scale;
    s.fx = rx->fx;
    s.fy = rx->fy;
    s.fz = rx->fz;
    s.mx = rx->mx;
    s.my = rx->my;
    s.mz = rx->mz;
    s.fs_type = rx->fs_type;
    s.wrench_valid = rx->wrench_valid;
  } else {
    // no fresh RX this window: keep last measured from the prior snapshot.
    const StateSnapshot prev = snap_.read();
    s.q_meas = prev.q_meas;
    s.qd_est = prev.qd_est;
    s.cart = prev.cart;
    s.rx_seq = prev.rx_seq;
    s.ctrl_time_stamp_ms = prev.ctrl_time_stamp_ms;
    s.rx_mono_ns = prev.rx_mono_ns;  // stale — age exposed via tick_mono_ns − rx_mono_ns
    s.e_stopped = prev.e_stopped;
    s.in_error = prev.in_error;
    s.tp_enabled = prev.tp_enabled;
    s.motion_possible = prev.motion_possible;
    s.motion_in_progress = prev.motion_in_progress;
    s.contact_stop_status = prev.contact_stop_status;
    s.safety_scale = prev.safety_scale;
    s.wrench_valid = false;  // no fresh wrench
  }
  s.tx_seq = tx_seq_;
  s.tick_mono_ns = tick_mono_ns;
  s.mode = tick_core_.mode();
  s.fault = tick_core_.fault();
  s.conditions = tick_core_.conditions();
  s.active_motion_id = tick_core_.active_motion_id();
  s.active_motion_status = tick_core_.active_motion_status();
  s.epoch = tick_core_.epoch();
  s.total_slew_clips = tick_core_.total_slew_clips();
  s.rx_fresh = rx_fresh;
  snap_.write(s);

  active_id_pub_.store(tick_core_.active_motion_id(), std::memory_order_release);
  active_status_pub_.store(static_cast<std::uint32_t>(tick_core_.active_motion_status()),
                           std::memory_order_release);
}

void RealtimeCore::record_resolution_(std::uint64_t id, MotionStatus st) {
  ResolutionSlot& slot = resolutions_[static_cast<std::size_t>(resolution_head_)];
  slot.status.store(static_cast<std::uint32_t>(st), std::memory_order_relaxed);
  slot.id.store(id, std::memory_order_release);
  resolution_head_ = (resolution_head_ + 1) % kResolutions;
}

void RealtimeCore::drain_tick_events_() {
  Event e{};
  bool any = false;
  while (tick_core_.pop_event(e)) {
    if (is_resolution_event(e.type) && e.motion_id != 0) {
      record_resolution_(e.motion_id, event_to_status(e.type));
    }
    py_events_.push(e);  // drop-if-full
    any = true;
  }
  if (any && event_fd_ >= 0) {
    std::uint64_t one = 1;
    { ssize_t wr = ::write(event_fd_, &one, sizeof(one)); (void)wr; }
  }
}

// Body of the reclaim. Caller MUST already hold submit_mu_ (live_buffers_ is
// guarded by it). Kept separate from reap_retired_() so the submit_* paths — which
// all run holding submit_mu_ — have a way to reap without re-locking a
// non-recursive mutex and self-deadlocking. That deadlock is not hypothetical: it
// needs only a non-empty retire_ring_, i.e. any submit after one trajectory has
// completed and had its buffer retired.
void RealtimeCore::reap_retired_locked_() {
  const void* owner = nullptr;
  while (retire_ring_.pop(owner)) {
    for (auto it = live_buffers_.begin(); it != live_buffers_.end(); ++it) {
      if (it->get() == owner) {
        live_buffers_.erase(it);
        break;
      }
    }
  }
}

// For callers that do NOT already hold submit_mu_ (kick(), stop()).
void RealtimeCore::reap_retired_() {
  std::lock_guard<std::mutex> lk(submit_mu_);
  reap_retired_locked_();
}

// ---------------------------------------------------------------------------
// submission (off-RT; serialised)
// ---------------------------------------------------------------------------
std::uint64_t RealtimeCore::enqueue_(const Target& t, const void* owner) {
  Target tt = t;
  tt.motion_id = next_motion_id_.fetch_add(1, std::memory_order_relaxed);
  tt.epoch = published_epoch_.load(std::memory_order_acquire);
  // Stamp the caller-side stop generation. A stop_j that runs BEFORE this submit
  // (brake-then-submit) is already reflected here → the target is NOT superseded;
  // a stop_j that runs AFTER this submit bumps stop_gen_ past this stamp →
  // superseded at consume. submit_mu_ (held across every submit)
  // serialises the two submit-side reads; stop_j() bumps stop_gen_ lock-free but
  // its increment is release-ordered so a same-thread submit sees it.
  tt.stop_gen = stop_gen_.load(std::memory_order_acquire);
  tt.buffer_owner = owner;
  if (!mailbox_.push(tt)) {
    // Full (should not happen at these rates) — reclaim the buffer DIRECTLY under
    // the already-held submit_mu_ and drop the target. Do NOT push to
    // retire_ring_: that ring is strictly single-producer = the RT thread, and a
    // second producer here (the submit thread, which never takes any RT lock)
    // would be a data race on the SPSC slot AND could lose the owner pointer
    // (never reaped → live_buffers_ grows unbounded). `owner` is the raw
    // TrajBuffer that submit_trajectory just push_back'd, so erase it here.
    if (owner != nullptr) {
      for (auto it = live_buffers_.begin(); it != live_buffers_.end(); ++it) {
        if (it->get() == owner) {
          live_buffers_.erase(it);
          break;
        }
      }
    }
    return 0;
  }
  return tt.motion_id;
}

std::uint64_t RealtimeCore::submit_trajectory(const std::vector<std::int64_t>& times_ns,
                                              const std::vector<Vec6>& q, const std::vector<Vec6>& qd,
                                              double speed_scale, double settle_tol_rad,
                                              double settle_vel_eps_rad_s, double settle_timeout_s,
                                              double force_stop_n, double deadman_s) {
  std::lock_guard<std::mutex> lk(submit_mu_);
  reap_retired_locked_();
  auto buf = std::make_unique<TrajBuffer>();
  buf->times = times_ns;
  buf->q = q;
  buf->qd = qd;
  TrajBuffer* raw = buf.get();
  live_buffers_.push_back(std::move(buf));

  Target t{};
  t.kind = TargetKind::kTrajectory;
  t.times_ns = raw->times.data();
  t.q = raw->q.data();
  t.qd = raw->qd.data();
  t.n = static_cast<int>(raw->q.size());
  t.speed_scale = speed_scale;
  t.settle_tol_rad = settle_tol_rad;
  t.settle_vel_eps_rad_s = settle_vel_eps_rad_s;
  t.settle_timeout_s = settle_timeout_s;
  t.force_stop_n = force_stop_n;
  t.deadman_s = deadman_s;
  return enqueue_(t, raw);
}

std::uint64_t RealtimeCore::submit_servo(const Vec6& q, double duration_s) {
  std::lock_guard<std::mutex> lk(submit_mu_);
  reap_retired_locked_();
  Target t{};
  t.kind = TargetKind::kServo;
  t.servo_q = q;
  t.servo_duration_s = duration_s;
  return enqueue_(t, nullptr);
}

std::uint64_t RealtimeCore::submit_servo(const Vec6& q, const Vec6& qd, const Vec6& qdd,
                                         double duration_s) {
  std::lock_guard<std::mutex> lk(submit_mu_);
  reap_retired_locked_();
  Target t{};
  t.kind = TargetKind::kServo;
  t.servo_q = q;
  t.servo_qd = qd;
  t.servo_qdd = qdd;
  t.servo_has_ff = true;
  t.servo_duration_s = duration_s;
  return enqueue_(t, nullptr);
}

std::uint64_t RealtimeCore::submit_brake() {
  std::lock_guard<std::mutex> lk(submit_mu_);
  reap_retired_locked_();
  Target t{};
  t.kind = TargetKind::kBrake;
  return enqueue_(t, nullptr);
}

std::uint64_t RealtimeCore::submit_hold() {
  std::lock_guard<std::mutex> lk(submit_mu_);
  reap_retired_locked_();
  Target t{};
  t.kind = TargetKind::kHold;
  return enqueue_(t, nullptr);
}

void RealtimeCore::stop_j() {
  // Bump the caller-side stop generation FIRST: a submit sequenced
  // after this stop_j will stamp the new value and so is NOT superseded, while a
  // target already submitted keeps the old stamp and is superseded at consume.
  stop_gen_.fetch_add(1, std::memory_order_acq_rel);
  stop_req_.store(true, std::memory_order_release);
}
void RealtimeCore::hold() { hold_req_.store(true, std::memory_order_release); }
void RealtimeCore::recover() { recover_req_.store(true, std::memory_order_release); }
void RealtimeCore::kick() {
  kick_req_.store(true, std::memory_order_release);
  reap_retired_();  // opportunistic off-RT reclaim (kick is called frequently)
}
void RealtimeCore::heartbeat() { heartbeat_req_.store(true, std::memory_order_release); }

// ---------------------------------------------------------------------------
// getters
// ---------------------------------------------------------------------------
std::size_t RealtimeCore::drain_events(Event* out, std::size_t max) {
  // Drain the eventfd count (best effort).
  if (event_fd_ >= 0) {
    std::uint64_t drain;
    { ssize_t rr = ::read(event_fd_, &drain, sizeof(drain)); (void)rr; }
  }
  std::size_t k = 0;
  Event e{};
  while (k < max && py_events_.pop(e)) {
    out[k++] = e;
  }
  return k;
}

TimingStats RealtimeCore::timing() const {
  TimingStats s{};
  s.tx_interval_p50_ms = static_cast<double>(hist_tx_interval_.percentile(0.50)) * 1e-6;
  s.tx_interval_p99_ms = static_cast<double>(hist_tx_interval_.percentile(0.99)) * 1e-6;
  s.tx_interval_p999_ms = static_cast<double>(hist_tx_interval_.percentile(0.999)) * 1e-6;
  s.tx_interval_max_ms = static_cast<double>(hist_tx_interval_.max_ns) * 1e-6;
  s.rx2tx_p50_us = static_cast<double>(hist_rx2tx_.percentile(0.50)) * 1e-3;
  s.rx2tx_p99_us = static_cast<double>(hist_rx2tx_.percentile(0.99)) * 1e-3;
  s.rx2tx_p999_us = static_cast<double>(hist_rx2tx_.percentile(0.999)) * 1e-3;
  s.rx2tx_max_us = static_cast<double>(hist_rx2tx_.max_ns) * 1e-3;
  s.tick_count = tick_count_.load(std::memory_order_relaxed);
  s.tx_count = tx_count_.load(std::memory_order_relaxed);
  s.tau_advance_count = tau_advance_count_.load(std::memory_order_relaxed);
  s.parked_ticks = parked_ticks_.load(std::memory_order_relaxed);
  s.missed_rx_ticks = missed_rx_ticks_.load(std::memory_order_relaxed);
  s.rx_seq_gaps = rx_seq_gaps_.load(std::memory_order_relaxed);
  s.rx_nonfinite_drops = rx_nonfinite_drops_.load(std::memory_order_relaxed);
  s.skipped_tick_windows = skipped_tick_windows_.load(std::memory_order_relaxed);
  s.double_send_guard = double_send_guard_.load(std::memory_order_relaxed);
  s.cpu_migrations = cpu_migrations_.load(std::memory_order_relaxed);
  return s;
}

bool RealtimeCore::joints_at_wall(std::int64_t wall_ns, Vec6& out) const {
  // Establish the mono↔wall offset off-RT on the caller's thread (both clocks
  // read now; the offset is stable over the short lookup). mono = wall + offset.
  const std::int64_t offset = now_ns() - realtime_ns();
  const std::int64_t query_mono = wall_ns + offset;
  return joints_ring_.nearest(query_mono, out);
}

MotionStatus RealtimeCore::motion_status(std::uint64_t id) const {
  if (id == 0) {
    return MotionStatus::PENDING;
  }
  // Resolved?
  for (const auto& slot : resolutions_) {
    if (slot.id.load(std::memory_order_acquire) == id) {
      return static_cast<MotionStatus>(slot.status.load(std::memory_order_relaxed));
    }
  }
  // Currently active?
  if (active_id_pub_.load(std::memory_order_acquire) == id) {
    return static_cast<MotionStatus>(active_status_pub_.load(std::memory_order_relaxed));
  }
  return MotionStatus::PENDING;
}

}  // namespace airo_fanuc::rt_core
