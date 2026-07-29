// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc P2' timing spike -- RT-loop skeleton.
//
// A self-contained, minimal-but-faithful model of the in-process 125 Hz Stream
// Motion RT loop from PLAN.md §5.2 / design doc 07 §3.2, built to answer ONE
// question: can an in-process C++ RT thread hold 8 ms TX cadence under
// adversarial CUDA/GIL/voxel load on olifant core 31?
//
// This program depends on NOTHING else in the airo_fanuc package (per the P2'
// self-containment decision): it talks to the throwaway `status_gen.py` over
// loopback UDP using packet SIZES that match the real Stream Motion structs
// (RobotStatusPacket = 416 B, CommandPacket = 344 B, see
// vendor/.../stream_motion/packets.hpp) but is otherwise a standalone spike.
//
// Design invariants exercised here (all normative in PLAN.md):
//   * PLL-clocked TX (R1 A1/A2): the timerfd sets the 8 ms cadence; its phase is
//     servoed toward RX arrival timestamps with a per-tick correction clamped to
//     <= 100 us. NOT raw reply-on-receive.
//   * Exactly ONE send() + ONE tau-advance per 8 ms window (hard invariant).
//   * No wall clock in the hot path: trajectory tau is relative (8 ms/tick);
//     CLOCK_MONOTONIC is used only for stamping / PLL / metrics.
//   * Zero-allocation hot path: all buffers pre-allocated; a global operator-new
//     counter asserts zero allocations after warmup.
//
// What this program CANNOT do without privilege (see README.md): SCHED_FIFO,
// mlockall, and core-pinning all require root / CAP_SYS_NICE / rtprio limits.
// They are requested via flags and degrade gracefully (warn + continue) so the
// harness still runs an unprivileged plumbing sanity check. The go/no-go VERDICT
// is only meaningful under the full isolated-host conditions in the README.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // sched_getcpu, pthread_setaffinity_np, RUSAGE_THREAD, MCL_ONFAULT
#endif

#include <array>
#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <vector>
#include <algorithm>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Global allocation counter (zero-alloc-hot-path proof).
// Overriding the global operators lets us count EVERY heap allocation, then
// diff the counter across the measurement window. The RT hot path must add 0.
// ---------------------------------------------------------------------------
static std::atomic<uint64_t> g_alloc_count{0};

void* operator new(std::size_t n) {
  g_alloc_count.fetch_add(1, std::memory_order_relaxed);
  void* p = std::malloc(n ? n : 1);
  if (!p) throw std::bad_alloc();
  return p;
}
void* operator new[](std::size_t n) {
  g_alloc_count.fetch_add(1, std::memory_order_relaxed);
  void* p = std::malloc(n ? n : 1);
  if (!p) throw std::bad_alloc();
  return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

// ---------------------------------------------------------------------------
// Constants matching the real Stream Motion wire (sizes only; loopback is
// native-endian -- byte-swap is a controller-facing concern, out of scope here).
// ---------------------------------------------------------------------------
static constexpr size_t STATUS_BYTES = 416;   // RobotStatusPacket (type 204)
static constexpr size_t COMMAND_BYTES = 344;  // CommandPacket     (type 201)
static constexpr int NJ = 6;                  // 6-DOF arm (wire pads to 9)
static constexpr int64_t TICK_NS = 8'000'000; // 125 Hz -> 8 ms
static constexpr double TICK_S = 0.008;
static constexpr int64_t PLL_CAP_NS = 100'000; // <= 100 us/tick slew cap (invariant)

// Field byte offsets inside RobotStatusPacket (see packets.hpp, #pragma pack 1):
//   sequence_no @ 8 (u32), time_stamp @ 16 (u32), joint_angle[9] @ 56 (f32).
static constexpr size_t OFF_STATUS_SEQ = 8;
static constexpr size_t OFF_STATUS_TS = 16;
static constexpr size_t OFF_STATUS_JA = 56;
// Field byte offsets inside CommandPacket:
//   sequence_no @ 8 (u32), is_last @ 12, do_motn_ctrl @ 13, command_pos[9] @ 16.
static constexpr size_t OFF_CMD_SEQ = 8;
static constexpr size_t OFF_CMD_ISLAST = 12;
static constexpr size_t OFF_CMD_DOMOTN = 13;
static constexpr size_t OFF_CMD_POS = 16;

// ---------------------------------------------------------------------------
static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop.store(true, std::memory_order_relaxed); }

static inline int64_t now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

static inline int64_t clamp_i64(int64_t v, int64_t lo, int64_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Sink to defeat dead-code elimination of the decode + Hermite work.
static volatile double g_sink = 0.0;

// ---------------------------------------------------------------------------
// Canned trajectory: NK knots x NJ joints of (position, velocity), evaluated
// with cubic Hermite. Filled once at startup (off the hot path).
// ---------------------------------------------------------------------------
static constexpr int NK = 6;
static const double KT[NK] = {0.0, 0.40, 0.90, 1.50, 2.20, 3.00};  // knot times [s]
static double KQ[NK][NJ];  // knot positions
static double KV[NK][NJ];  // knot velocities
static double TRAJ_DUR = KT[NK - 1];

static void build_canned_trajectory() {
  for (int k = 0; k < NK; ++k) {
    for (int j = 0; j < NJ; ++j) {
      const double w = 0.7 + 0.11 * j;
      KQ[k][j] = 0.30 * std::sin(w * KT[k] + 0.5 * j);
      KV[k][j] = 0.30 * w * std::cos(w * KT[k] + 0.5 * j);
    }
  }
}

// Evaluate the cubic-Hermite segment covering trajectory-time tau (seconds),
// writing NJ joint positions into out. Allocation-free; linear segment scan
// over NK knots (NK is tiny). This is the "trivial-but-real" per-tick work.
static inline void hermite_eval(double tau, double* out) {
  double t = std::fmod(tau, TRAJ_DUR);
  if (t < 0.0) t += TRAJ_DUR;
  int i = 0;
  while (i < NK - 2 && t >= KT[i + 1]) ++i;  // segment [i, i+1]
  const double h = KT[i + 1] - KT[i];
  const double s = (t - KT[i]) / h;
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  for (int j = 0; j < NJ; ++j) {
    out[j] = h00 * KQ[i][j] + h10 * h * KV[i][j] +
             h01 * KQ[i + 1][j] + h11 * h * KV[i + 1][j];
  }
}

// ---------------------------------------------------------------------------
struct Config {
  int pin_core = -1;        // --pin-core N  (pthread affinity)
  int sched_fifo = -1;      // --sched-fifo PRIO
  bool mlock = false;       // --mlock
  double duration_s = 30.0; // --duration-s
  double warmup_s = 5.0;    // --warmup-s
  std::string host = "127.0.0.1";  // --status-host
  int port = 60115;         // --status-port  (matches status_gen.py default)
  int64_t rx_lead_ns = 300'000;  // --rx-lead-us : target (tick - rx) lead
  double pll_kp = 0.20;     // --pll-kp : proportional phase-servo gain
};

static void usage(const char* argv0) {
  std::printf(
      "usage: %s [options]\n"
      "  --pin-core N       pin RT thread to logical core N (needs cpuset membership)\n"
      "  --sched-fifo PRIO  SCHED_FIFO priority (needs CAP_SYS_NICE / rtprio limit)\n"
      "  --mlock            mlockall(MCL_CURRENT|MCL_ONFAULT) (needs privilege/memlock)\n"
      "  --duration-s S     measurement duration after warmup (default 30)\n"
      "  --warmup-s S       warmup duration, metrics reset at end (default 5)\n"
      "  --status-host H    status_gen host (default 127.0.0.1)\n"
      "  --status-port P    status_gen UDP port (default 60115)\n"
      "  --rx-lead-us U     PLL target: fire tick U us after RX (default 300)\n"
      "  --pll-kp K         PLL proportional gain (default 0.20)\n"
      "  -h, --help\n",
      argv0);
}

static bool parse_args(int argc, char** argv, Config& c) {
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", name); std::exit(2); }
      return argv[++i];
    };
    if (a == "--pin-core") c.pin_core = std::atoi(need("--pin-core"));
    else if (a == "--sched-fifo") c.sched_fifo = std::atoi(need("--sched-fifo"));
    else if (a == "--mlock") c.mlock = true;
    else if (a == "--duration-s") c.duration_s = std::atof(need("--duration-s"));
    else if (a == "--warmup-s") c.warmup_s = std::atof(need("--warmup-s"));
    else if (a == "--status-host") c.host = need("--status-host");
    else if (a == "--status-port") c.port = std::atoi(need("--status-port"));
    else if (a == "--rx-lead-us") c.rx_lead_ns = static_cast<int64_t>(std::atof(need("--rx-lead-us")) * 1000.0);
    else if (a == "--pll-kp") c.pll_kp = std::atof(need("--pll-kp"));
    else if (a == "-h" || a == "--help") { usage(argv[0]); return false; }
    else { std::fprintf(stderr, "unknown arg: %s\n", a.c_str()); usage(argv[0]); std::exit(2); }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Percentile over a copy (post-loop; allocation here is fine).
static int64_t pct(std::vector<int64_t>& v, double p) {
  if (v.empty()) return 0;
  size_t idx = static_cast<size_t>(p * (v.size() - 1) + 0.5);
  if (idx >= v.size()) idx = v.size() - 1;
  std::nth_element(v.begin(), v.begin() + idx, v.end());
  return v[idx];
}

int main(int argc, char** argv) {
  Config cfg;
  if (!parse_args(argc, argv, cfg)) return 0;

  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  build_canned_trajectory();

  // ---- RT hygiene (all graceful on EPERM/EINVAL) ------------------------
  bool pinned = false, fifo = false, locked = false;
  std::string hygiene_notes;

  if (cfg.pin_core >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cfg.pin_core, &set);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (rc == 0) { pinned = true; }
    else hygiene_notes += "  [warn] pin-core " + std::to_string(cfg.pin_core) +
                          " failed: " + std::strerror(rc) + " (need cpuset membership)\n";
  }
  if (cfg.sched_fifo >= 0) {
    struct sched_param sp;
    std::memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.sched_fifo;
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc == 0) { fifo = true; }
    else hygiene_notes += "  [warn] SCHED_FIFO " + std::to_string(cfg.sched_fifo) +
                          " failed: " + std::strerror(rc) + " (need CAP_SYS_NICE / rtprio limit)\n";
  }
  if (cfg.mlock) {
    if (mlockall(MCL_CURRENT | MCL_ONFAULT) == 0) { locked = true; }
    else hygiene_notes += std::string("  [warn] mlockall failed: ") + std::strerror(errno) +
                          " (need privilege / RLIMIT_MEMLOCK)\n";
  }

  // Stack prefault: touch a chunk of stack so page faults happen now, not mid-tick.
  {
    volatile char stackbuf[512 * 1024];
    for (size_t i = 0; i < sizeof(stackbuf); i += 4096) stackbuf[i] = 0;
    g_sink += stackbuf[0];
  }

  // ---- Socket: connected UDP to the local status generator --------------
  int sockfd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (sockfd < 0) { std::perror("socket"); return 1; }
  sockaddr_in peer;
  std::memset(&peer, 0, sizeof(peer));
  peer.sin_family = AF_INET;
  peer.sin_port = htons(static_cast<uint16_t>(cfg.port));
  if (inet_pton(AF_INET, cfg.host.c_str(), &peer.sin_addr) != 1) {
    std::fprintf(stderr, "bad --status-host %s\n", cfg.host.c_str());
    return 1;
  }
  if (::connect(sockfd, reinterpret_cast<sockaddr*>(&peer), sizeof(peer)) < 0) {
    std::perror("connect");
    return 1;
  }
  // Kick the generator so it learns our source address (StartPacket-ish, type 200).
  {
    uint8_t start[8] = {0};
    uint32_t t = 200, v = 3;
    std::memcpy(start + 0, &t, 4);
    std::memcpy(start + 4, &v, 4);
    if (::send(sockfd, start, sizeof(start), 0) < 0) std::perror("send(start)");
  }

  // ---- timerfd (CLOCK_MONOTONIC, absolute one-shot, re-armed by the PLL) --
  int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
  if (tfd < 0) { std::perror("timerfd_create"); return 1; }

  int epfd = epoll_create1(0);
  if (epfd < 0) { std::perror("epoll_create1"); return 1; }
  epoll_event ev{};
  ev.events = EPOLLIN;
  ev.data.fd = sockfd;
  epoll_ctl(epfd, EPOLL_CTL_ADD, sockfd, &ev);
  ev.data.fd = tfd;
  epoll_ctl(epfd, EPOLL_CTL_ADD, tfd, &ev);

  auto arm_timer_abs = [&](int64_t abs_ns) {
    itimerspec its{};
    its.it_value.tv_sec = abs_ns / 1'000'000'000LL;
    its.it_value.tv_nsec = abs_ns % 1'000'000'000LL;
    // it_interval stays 0: one-shot, so the PLL controls every deadline.
    timerfd_settime(tfd, TFD_TIMER_ABSTIME, &its, nullptr);
  };

  // ---- pre-allocated hot-path buffers -----------------------------------
  static uint8_t rxbuf[1024];
  static uint8_t txbuf[COMMAND_BYTES];
  std::memset(txbuf, 0, sizeof(txbuf));
  {
    uint32_t t = 201, v = 3;
    std::memcpy(txbuf + 0, &t, 4);
    std::memcpy(txbuf + 4, &v, 4);
    txbuf[OFF_CMD_DOMOTN] = 1;  // do_motn_ctrl = 1 (pinned)
  }
  double cmd_q[NJ];

  const int64_t warmup_ticks = static_cast<int64_t>(cfg.warmup_s * 125.0 + 0.5);
  const int64_t total_ticks = static_cast<int64_t>((cfg.warmup_s + cfg.duration_s) * 125.0 + 0.5);
  const int64_t meas_ticks = total_ticks - warmup_ticks;

  // Reserve exactly-once so push_back never reallocates in the hot path.
  std::vector<int64_t> ivl_ns;   // TX-to-TX interval samples
  std::vector<int64_t> rx2tx_ns; // send - last_rx (state freshness at TX)
  ivl_ns.reserve(static_cast<size_t>(meas_ticks) + 16);
  rx2tx_ns.reserve(static_cast<size_t>(meas_ticks) + 16);

  // ---- hot-path state ---------------------------------------------------
  int64_t tau_ns = 0;              // relative trajectory time (NO wall clock)
  uint32_t tx_seq = 0;             // independent monotonic TX seq (from 1)
  int64_t scheduled_tick_ns = now_ns() + TICK_NS;
  arm_timer_abs(scheduled_tick_ns);

  int64_t last_rx_mono = 0;
  int64_t last_send_mono = 0;
  bool have_fresh_rx = false;
  uint32_t last_rx_seq = 0;

  // counters
  uint64_t tx_count = 0, tau_advance_count = 0;
  uint64_t rx_total = 0, rx_bad = 0, rx_oversize = 0, rx_seq_gaps = 0, rx_coalesced = 0;
  uint64_t missed_ticks = 0;       // timerfd reported >1 expiration
  uint64_t missed_rx_ticks = 0;    // ticks with no fresh RX since last tick
  uint64_t double_send_guard = 0;  // MUST stay 0: sends within same window
  int64_t min_ivl_ns = INT64_MAX;
  int last_cpu = -1, init_cpu = -1;
  uint64_t migrations = 0;

  bool recording = (warmup_ticks == 0);
  int64_t tick_count = 0;

  // rusage snapshots (RUSAGE_THREAD == this thread only)
  struct rusage ru_warm{}, ru_end{};
  uint64_t alloc_at_warmup = 0;

  auto snapshot_warmup = [&]() {
    getrusage(RUSAGE_THREAD, &ru_warm);
    alloc_at_warmup = g_alloc_count.load(std::memory_order_relaxed);
    ivl_ns.clear();
    rx2tx_ns.clear();
    last_send_mono = 0;   // don't record the interval straddling the boundary
    min_ivl_ns = INT64_MAX;
    migrations = 0;
    last_cpu = -1;
    recording = true;
  };
  if (warmup_ticks == 0) snapshot_warmup();

  init_cpu = sched_getcpu();

  std::printf("[rt_loop] starting: warmup=%.1fs measure=%.1fs (%lld ticks), "
              "peer=%s:%d, rx_lead=%lldus, kp=%.3f\n",
              cfg.warmup_s, cfg.duration_s, (long long)meas_ticks,
              cfg.host.c_str(), cfg.port, (long long)(cfg.rx_lead_ns / 1000), cfg.pll_kp);
  std::printf("[rt_loop] hygiene: pin=%s fifo=%s mlock=%s (init cpu=%d)\n",
              pinned ? "on" : "off", fifo ? "on" : "off", locked ? "on" : "off", init_cpu);
  if (!hygiene_notes.empty()) std::fputs(hygiene_notes.c_str(), stdout);
  std::fflush(stdout);

  // =========================== HOT LOOP ==================================
  epoll_event events[8];
  while (tick_count < total_ticks && !g_stop.load(std::memory_order_relaxed)) {
    int n = epoll_wait(epfd, events, 8, -1);
    if (n < 0) {
      if (errno == EINTR) continue;
      std::perror("epoll_wait");
      break;
    }

    bool timer_ready = false;
    // Drain socket events FIRST so the tick acts on the freshest RX.
    for (int i = 0; i < n; ++i) {
      if (events[i].data.fd == sockfd) {
        int64_t rx_mono = now_ns();
        int got = 0;
        for (;;) {
          ssize_t r = ::recv(sockfd, rxbuf, sizeof(rxbuf), 0);
          if (r < 0) break;  // EAGAIN/EWOULDBLOCK -> drained
          rx_total++;
          if (static_cast<size_t>(r) != STATUS_BYTES) {
            rx_bad++;
            if (static_cast<size_t>(r) > STATUS_BYTES) rx_oversize++;
            continue;
          }
          // Real decode work: seq, controller timestamp, 6 joint angles.
          uint32_t seq, cts;
          float ja[NJ];
          std::memcpy(&seq, rxbuf + OFF_STATUS_SEQ, 4);
          std::memcpy(&cts, rxbuf + OFF_STATUS_TS, 4);
          std::memcpy(ja, rxbuf + OFF_STATUS_JA, sizeof(ja));
          if (last_rx_seq != 0 && seq != last_rx_seq + 1) rx_seq_gaps++;
          last_rx_seq = seq;
          g_sink += ja[0] + static_cast<double>(cts);
          got++;
        }
        if (got > 0) {
          last_rx_mono = rx_mono;
          have_fresh_rx = true;
          if (got > 1) rx_coalesced += static_cast<uint64_t>(got - 1);
        }
      } else if (events[i].data.fd == tfd) {
        timer_ready = true;
      }
    }

    if (!timer_ready) continue;

    // ---- TIMER TICK: exactly ONE per firing --------------------------
    uint64_t expirations = 0;
    if (::read(tfd, &expirations, sizeof(expirations)) != sizeof(expirations)) {
      // spurious wake without a real expiration; nothing to do
      continue;
    }
    if (expirations > 1) missed_ticks += (expirations - 1);

    // PLL phase servo: pull scheduled tick toward (last_rx + rx_lead).
    // Correction clamped to +-100us/tick (the hard slew cap). Only servo on a
    // fresh RX; otherwise free-run at nominal cadence.
    int64_t next_tick_ns;
    if (have_fresh_rx) {
      int64_t err = scheduled_tick_ns - (last_rx_mono + cfg.rx_lead_ns);
      int64_t corr = clamp_i64(static_cast<int64_t>(cfg.pll_kp * static_cast<double>(err)),
                               -PLL_CAP_NS, PLL_CAP_NS);
      next_tick_ns = scheduled_tick_ns + TICK_NS - corr;
    } else {
      next_tick_ns = scheduled_tick_ns + TICK_NS;
      missed_rx_ticks++;
    }
    // never schedule in the past relative to the just-fired deadline
    if (next_tick_ns <= scheduled_tick_ns) next_tick_ns = scheduled_tick_ns + 1;
    scheduled_tick_ns = next_tick_ns;
    arm_timer_abs(next_tick_ns);

    // ---- the ONE tau-advance for this window -------------------------
    tau_ns += TICK_NS;
    tau_advance_count++;

    // real per-tick work: cubic-Hermite eval, then encode 344-byte command
    hermite_eval(static_cast<double>(tau_ns) * 1e-9, cmd_q);
    tx_seq++;  // independent monotonic TX seq from 1
    std::memcpy(txbuf + OFF_CMD_SEQ, &tx_seq, 4);
    txbuf[OFF_CMD_ISLAST] = 0;
    for (int j = 0; j < NJ; ++j) {
      double d = cmd_q[j];  // (would be deg here on the wire; irrelevant to timing)
      std::memcpy(txbuf + OFF_CMD_POS + static_cast<size_t>(j) * 8, &d, 8);
    }

    // ---- the ONE send for this window --------------------------------
    int64_t send_mono = now_ns();
    (void)::send(sockfd, txbuf, sizeof(txbuf), 0);
    tx_count++;

    // Invariant guard: two sends must never fall inside the same 8 ms window.
    if (last_send_mono != 0 && (send_mono - last_send_mono) < (TICK_NS / 4)) {
      double_send_guard++;
    }

    // ---- metrics (post-warmup only) ----------------------------------
    if (recording) {
      if (last_send_mono != 0) {
        int64_t ivl = send_mono - last_send_mono;
        if (ivl_ns.size() < ivl_ns.capacity()) ivl_ns.push_back(ivl);
        if (ivl < min_ivl_ns) min_ivl_ns = ivl;
      }
      // rx->tx = freshness of the state we acted on (age of the last RX at send).
      int64_t age = send_mono - last_rx_mono;
      if (last_rx_mono != 0 && rx2tx_ns.size() < rx2tx_ns.capacity()) rx2tx_ns.push_back(age);
      int cpu = sched_getcpu();
      if (last_cpu >= 0 && cpu != last_cpu) migrations++;
      last_cpu = cpu;
    }
    last_send_mono = send_mono;
    have_fresh_rx = false;

    tick_count++;
    if (tick_count == warmup_ticks && warmup_ticks > 0) snapshot_warmup();
  }
  // ========================= END HOT LOOP ================================

  getrusage(RUSAGE_THREAD, &ru_end);
  const uint64_t alloc_after = g_alloc_count.load(std::memory_order_relaxed);
  const uint64_t hotpath_allocs = alloc_after - alloc_at_warmup;

  ::close(tfd);
  ::close(sockfd);
  ::close(epfd);

  // ------------------------------ REPORT ---------------------------------
  auto ms = [](int64_t ns) { return static_cast<double>(ns) * 1e-6; };
  auto us = [](int64_t ns) { return static_cast<double>(ns) * 1e-3; };

  int64_t ivl_p50 = pct(ivl_ns, 0.50);
  int64_t ivl_p99 = pct(ivl_ns, 0.99);
  int64_t ivl_p999 = pct(ivl_ns, 0.999);
  int64_t ivl_max = ivl_ns.empty() ? 0 : *std::max_element(ivl_ns.begin(), ivl_ns.end());
  int64_t r2t_p50 = pct(rx2tx_ns, 0.50);
  int64_t r2t_p99 = pct(rx2tx_ns, 0.99);
  int64_t r2t_p999 = pct(rx2tx_ns, 0.999);
  int64_t r2t_max = rx2tx_ns.empty() ? 0 : *std::max_element(rx2tx_ns.begin(), rx2tx_ns.end());

  const int64_t majflt = ru_end.ru_majflt - ru_warm.ru_majflt;
  const int64_t minflt = ru_end.ru_minflt - ru_warm.ru_minflt;
  const int64_t nivcsw = ru_end.ru_nivcsw - ru_warm.ru_nivcsw;
  const int64_t nvcsw = ru_end.ru_nvcsw - ru_warm.ru_nvcsw;

  std::printf("\n========================= RT-LOOP SPIKE REPORT =========================\n");
  std::printf("samples: interval=%zu  rx2tx=%zu   ticks: total=%lld warmup=%lld\n",
              ivl_ns.size(), rx2tx_ns.size(), (long long)tick_count, (long long)warmup_ticks);
  std::printf("RX: total=%llu bad=%llu oversize=%llu seq_gaps=%llu coalesced=%llu\n",
              (unsigned long long)rx_total, (unsigned long long)rx_bad,
              (unsigned long long)rx_oversize, (unsigned long long)rx_seq_gaps,
              (unsigned long long)rx_coalesced);
  std::printf("missed_ticks(>1 exp)=%llu  missed_rx_ticks(no fresh rx)=%llu\n",
              (unsigned long long)missed_ticks, (unsigned long long)missed_rx_ticks);
  std::printf("\n-- TX interval (target 8.000 ms) --\n");
  std::printf("  p50=%.4f ms  p99=%.4f ms  p999=%.4f ms  max=%.4f ms  min=%.4f ms\n",
              ms(ivl_p50), ms(ivl_p99), ms(ivl_p999), ms(ivl_max),
              ms(min_ivl_ns == INT64_MAX ? 0 : min_ivl_ns));
  std::printf("-- rx -> tx latency (state freshness at TX) --\n");
  std::printf("  p50=%.1f us  p99=%.1f us  p999=%.1f us  max=%.1f us\n",
              us(r2t_p50), us(r2t_p99), us(r2t_p999), us(r2t_max));
  std::printf("-- scheduling / memory (RUSAGE_THREAD deltas, post-warmup) --\n");
  std::printf("  cpu_migrations=%llu  init_cpu=%d\n", (unsigned long long)migrations, init_cpu);
  std::printf("  majflt=%lld  minflt=%lld  nonvol_ctxsw=%lld  vol_ctxsw=%lld\n",
              (long long)majflt, (long long)minflt, (long long)nivcsw, (long long)nvcsw);
  std::printf("  hotpath_heap_allocs=%llu  (MUST be 0)\n", (unsigned long long)hotpath_allocs);
  std::printf("-- invariants --\n");
  std::printf("  tx_count=%llu  tau_advance_count=%llu  (MUST be equal)\n",
              (unsigned long long)tx_count, (unsigned long long)tau_advance_count);
  std::printf("  double_send_guard=%llu  (MUST be 0: one TX per 8ms window)\n",
              (unsigned long long)double_send_guard);
  std::printf("  sink=%.6f (anti-DCE)\n", g_sink);

  // --------------------------- VERDICT -----------------------------------
  // Gates (PLAN.md §9 P2' row / design doc 07 §3.4). These are ONLY meaningful
  // under the full isolated-host conditions in README.md (core 31 pinned,
  // SCHED_FIFO, mlock, sibling 15 idle, IRQ pinned, >=1 h, real adversarial
  // load). An unprivileged / unloaded run is a PLUMBING check, not a verdict.
  auto gate = [](bool ok) { return ok ? "PASS" : "FAIL"; };
  bool g_ivl_p99 = ivl_p99 <= 8'500'000;
  bool g_ivl_p999 = ivl_p999 <= 9'000'000;
  bool g_ivl_max = ivl_max < 16'000'000;
  bool g_r2t = r2t_p99 < 500'000;
  bool g_majflt = majflt == 0;
  bool g_migr = migrations == 0;
  bool g_alloc = hotpath_allocs == 0;
  bool g_inv = (tx_count == tau_advance_count) && (double_send_guard == 0);
  bool all = g_ivl_p99 && g_ivl_p999 && g_ivl_max && g_r2t && g_majflt && g_migr && g_alloc && g_inv;

  const bool privileged = pinned && fifo && locked;

  std::printf("\n--------------------------- VERDICT ---------------------------\n");
  std::printf("  [%s] TX interval p99  <= 8.5 ms   (%.4f ms)\n", gate(g_ivl_p99), ms(ivl_p99));
  std::printf("  [%s] TX interval p999 <= 9.0 ms   (%.4f ms)\n", gate(g_ivl_p999), ms(ivl_p999));
  std::printf("  [%s] TX interval max  <  16 ms    (%.4f ms)\n", gate(g_ivl_max), ms(ivl_max));
  std::printf("  [%s] rx->tx p99       <  500 us   (%.1f us)\n", gate(g_r2t), us(r2t_p99));
  std::printf("  [%s] majflt post-warmup == 0      (%lld)\n", gate(g_majflt), (long long)majflt);
  std::printf("  [%s] cpu migrations    == 0       (%llu)\n", gate(g_migr), (unsigned long long)migrations);
  std::printf("  [%s] hot-path allocs   == 0       (%llu)\n", gate(g_alloc), (unsigned long long)hotpath_allocs);
  std::printf("  [%s] one-TX/one-tau per window     (dbl=%llu)\n", gate(g_inv), (unsigned long long)double_send_guard);
  std::printf("  ---------------------------------------------------------\n");
  std::printf("  OVERALL: %s\n", all ? "PASS" : "FAIL");
  if (!privileged) {
    std::printf("  *** NOT A GO/NO-GO VERDICT ***\n");
    std::printf("  This run lacks pin/fifo/mlock and/or real load. Gates above are a\n");
    std::printf("  PLUMBING sanity check only. See README.md for the real soak procedure.\n");
  }
  std::printf("===============================================================\n");

  return all ? 0 : 1;
}
