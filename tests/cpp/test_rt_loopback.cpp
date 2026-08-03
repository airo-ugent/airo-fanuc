// SPDX-License-Identifier: Apache-2.0
//
// RealtimeCore UDP loopback smoke test — one step up from a unit test: it runs
// the real RT thread and socket, against a stub peer rather than the full fake. A
// throwaway in-process peer streams hand-built 416 B type-204 status; the core
// prerolls to HOLD and TXes valid 344 B type-201 commands. Asserts: valid wire
// framing (dataStyle 0xFFFF, do_motn_ctrl=1), the PLL invariants (one TX + one
// τ-advance per window, double_send=0), and ZERO heap allocs on the hot path
// after warmup.

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "rt_core/realtime_core.hpp"

using namespace airo_fanuc::rt_core;

// ---------------------------------------------------------------------------
namespace {
std::atomic<std::uint64_t> g_alloc{0};
std::atomic<bool> g_count{false};

std::int64_t now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}
void be_store_u32(std::uint8_t* p, std::uint32_t v) {
  p[0] = static_cast<std::uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<std::uint8_t>(v & 0xFF);
}
std::uint32_t be_load_u32(const std::uint8_t* p) {
  return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
         (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
}

// Build a valid 416 B type-204 status: all gates clear, motion_possible=1,
// safety_scale=1.0, joints=0.
void build_status(std::uint8_t* buf, std::uint32_t seq) {
  std::memset(buf, 0, 416);
  be_store_u32(buf + 0, 204);  // packet_type
  be_store_u32(buf + 4, 3);    // version_no
  be_store_u32(buf + 8, seq);  // sequence_no
  buf[12] = 0x01;              // status: motion_possible
  buf[13] = 0x00;              // robot_status: no estop/error/teach
  buf[14] = 0x00;              // contact_stop_status: None
  be_store_u32(buf + 128, 0x3F800000);  // safety_scale = 1.0f (big-endian)
  // fs_type (offset 156) = 0 → wrench invalid; joints (offset 56) = 0.
}

void sleep_ms(int ms) {
  struct timespec ts {
    ms / 1000, (ms % 1000) * 1'000'000L
  };
  nanosleep(&ts, nullptr);
}

}  // namespace

void* operator new(std::size_t n) {
  if (g_count.load(std::memory_order_relaxed)) g_alloc.fetch_add(1, std::memory_order_relaxed);
  void* p = std::malloc(n ? n : 1);
  if (!p) throw std::bad_alloc();
  return p;
}
void* operator new[](std::size_t n) {
  if (g_count.load(std::memory_order_relaxed)) g_alloc.fetch_add(1, std::memory_order_relaxed);
  void* p = std::malloc(n ? n : 1);
  if (!p) throw std::bad_alloc();
  return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

TEST(RtLoopback, StreamsValidCommandsPllInvariantsZeroAlloc) {
  // ---- peer ("controller") socket: bind to an ephemeral port ----
  int peer = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(peer, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  ASSERT_EQ(::bind(peer, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
  socklen_t alen = sizeof(addr);
  ASSERT_EQ(::getsockname(peer, reinterpret_cast<sockaddr*>(&addr), &alen), 0);
  const std::uint16_t port = ntohs(addr.sin_port);
  // The peer drains with MSG_DONTWAIT (non-blocking) so its loop never stalls.

  RtCoreConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.sm_port = port;
  RealtimeCore core(cfg);
  ASSERT_TRUE(core.start());

  std::atomic<bool> peer_stop{false};
  std::atomic<std::uint64_t> cmd_count{0};
  std::array<std::uint8_t, 344> last_cmd{};
  std::atomic<int> last_cmd_len{0};

  std::thread peer_thr([&] {
    std::uint8_t rxbuf[2048];
    std::uint8_t status[416];
    sockaddr_in from{};
    socklen_t fl = sizeof(from);
    bool have_addr = false;
    sockaddr_in core_addr{};
    std::int64_t last_send = 0;
    std::uint32_t seq = 1;
    while (!peer_stop.load(std::memory_order_relaxed)) {
      for (;;) {
        const ssize_t r = ::recvfrom(peer, rxbuf, sizeof(rxbuf), MSG_DONTWAIT,
                                     reinterpret_cast<sockaddr*>(&from), &fl);
        if (r < 0) break;
        core_addr = from;
        have_addr = true;
        if (r == 344) {
          cmd_count.fetch_add(1, std::memory_order_relaxed);
          std::memcpy(last_cmd.data(), rxbuf, 344);
          last_cmd_len.store(344, std::memory_order_release);
        }
      }
      const std::int64_t now = now_ns();
      if (have_addr && (now - last_send) >= 8'000'000) {
        build_status(status, seq++);
        (void)::sendto(peer, status, sizeof(status), 0, reinterpret_cast<sockaddr*>(&core_addr),
                       sizeof(core_addr));
        last_send = now;
      }
      struct timespec ts {
        0, 500'000
      };  // 500 µs
      nanosleep(&ts, nullptr);
    }
  });

  EXPECT_TRUE(core.wait_ready(3.0)) << "core prerolls to STREAMING on motion_possible";

  // Warm up, then measure a zero-alloc window (main thread idle; only RT + peer run).
  sleep_ms(300);
  const std::uint64_t a0 = g_alloc.load();
  g_count.store(true, std::memory_order_relaxed);
  sleep_ms(400);
  g_count.store(false, std::memory_order_relaxed);
  const std::uint64_t window_allocs = g_alloc.load() - a0;

  sleep_ms(100);
  const TimingStats st = core.timing();
  const StateSnapshot snap = core.snapshot();

  peer_stop.store(true, std::memory_order_relaxed);
  peer_thr.join();
  core.stop();
  ::close(peer);

  // ---- assertions ----
  EXPECT_EQ(snap.mode, Mode::HOLD) << "steady HOLD (no motion submitted)";
  EXPECT_GT(cmd_count.load(), 50u) << "core TXed commands";
  EXPECT_GT(st.tx_count, 50u);
  EXPECT_EQ(st.double_send_guard, 0u) << "no two sends within a quarter tick of each other";
  EXPECT_EQ(st.tau_advance_count, st.tick_count) << "one τ-advance per tick";
  EXPECT_EQ(window_allocs, 0u) << "zero heap allocations on the hot path after warmup";
  // TX cadence sane (loopback, unprivileged: generous bound).
  EXPECT_LT(st.tx_interval_p99_ms, 20.0);

  // ---- validate the last captured command's wire framing ----
  ASSERT_EQ(last_cmd_len.load(), 344);
  EXPECT_EQ(be_load_u32(last_cmd.data()), 201u) << "type-201 CommandPacket";
  EXPECT_EQ(last_cmd[13], 1) << "do_motn_ctrl pinned to 1";
  EXPECT_EQ(last_cmd[14], 0xFF) << "dataStyle 0xFFFF (byte 0)";
  EXPECT_EQ(last_cmd[15], 0xFF) << "dataStyle 0xFFFF (byte 1)";
}
