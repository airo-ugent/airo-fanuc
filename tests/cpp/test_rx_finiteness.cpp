// SPDX-License-Identifier: Apache-2.0
//
// RealtimeCore ingest finiteness gate, over UDP loopback with a stub peer that can
// poison individual wire floats. A non-finite field passes every clamp and every
// threshold downstream of ingest (std::clamp / std::min / std::max / `>` all return
// or reject nothing for a NaN), so the gate at ingest is the only thing between a
// corrupt datagram and a NaN joint position on the wire.
//
// Asserts, in order: a poisoned FIRST status does not start streaming; a clean
// stream still prerolls to HOLD afterwards; a poisoned safety_scale does not read
// as a cleared SAFETY_CLAMP gate; a poisoned Cartesian block never reaches the
// published snapshot; sustained poisoning escalates through the ordinary
// RX-silence ladder to a parked TX rather than wedging; and no command the core
// ever emitted carried a non-finite joint position.

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "rt_core/realtime_core.hpp"

using namespace airo_fanuc::rt_core;

namespace {

// Which wire float the peer poisons on the status packets it emits.
enum class Poison { kNone, kJointAngle, kSafetyScale, kCartesian };

// Big-endian float32 NaN (quiet, sign clear).
constexpr std::uint32_t kNanF32Bits = 0x7FC00000u;
// Type-204 field offsets, from the #pragma pack(1) vendored struct:
// position[9] at 20, joint_angle[9] at 56, current[9] at 92, safety_scale at 128.
constexpr std::size_t kOffPosition = 20;
constexpr std::size_t kOffJointAngle = 56;
constexpr std::size_t kOffSafetyScale = 128;
// CommandPacket: command_pos[9] doubles at offset 16.
constexpr std::size_t kOffCommandPos = 16;

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

double be_load_f64(const std::uint8_t* p) {
  std::uint64_t bits = 0;
  for (int i = 0; i < 8; ++i) {
    bits = (bits << 8) | static_cast<std::uint64_t>(p[i]);
  }
  double out = 0.0;
  std::memcpy(&out, &bits, sizeof(out));
  return out;
}

// A valid 416 B type-204 status (gates clear, motion_possible=1, safety_scale=1.0,
// joints 0), with one float optionally replaced by a NaN.
void build_status(std::uint8_t* buf, std::uint32_t seq, Poison poison) {
  std::memset(buf, 0, 416);
  be_store_u32(buf + 0, 204);  // packet_type
  be_store_u32(buf + 4, 3);    // version_no
  be_store_u32(buf + 8, seq);  // sequence_no
  buf[12] = 0x01;              // status: motion_possible
  buf[13] = 0x00;              // robot_status: no estop/error/teach
  buf[14] = 0x00;              // contact_stop_status: None
  be_store_u32(buf + kOffSafetyScale, 0x3F800000);  // safety_scale = 1.0f
  switch (poison) {
    case Poison::kJointAngle:
      be_store_u32(buf + kOffJointAngle, kNanF32Bits);  // J1
      break;
    case Poison::kSafetyScale:
      be_store_u32(buf + kOffSafetyScale, kNanF32Bits);
      break;
    case Poison::kCartesian:
      be_store_u32(buf + kOffPosition, kNanF32Bits);  // X
      break;
    case Poison::kNone:
      break;
  }
}

void sleep_ms(int ms) {
  struct timespec ts {
    ms / 1000, (ms % 1000) * 1'000'000L
  };
  nanosleep(&ts, nullptr);
}

bool all_finite(const Vec6& v) {
  for (double x : v) {
    if (!std::isfinite(x)) return false;
  }
  return true;
}

}  // namespace

TEST(RxFiniteness, NonFiniteStatusIsDroppedAndNeverCommanded) {
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

  RtCoreConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.sm_port = port;
  RealtimeCore core(cfg);
  ASSERT_TRUE(core.start());

  std::atomic<bool> peer_stop{false};
  std::atomic<int> poison{static_cast<int>(Poison::kJointAngle)};
  std::atomic<std::uint64_t> cmd_count{0};
  std::atomic<std::uint64_t> cmd_nonfinite{0};

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
          for (int j = 0; j < 9; ++j) {
            const double q = be_load_f64(rxbuf + kOffCommandPos + static_cast<std::size_t>(j) * 8);
            if (!std::isfinite(q)) cmd_nonfinite.fetch_add(1, std::memory_order_relaxed);
          }
        }
      }
      const std::int64_t now = now_ns();
      if (have_addr && (now - last_send) >= 8'000'000) {
        build_status(status, seq++, static_cast<Poison>(poison.load(std::memory_order_relaxed)));
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

  // ---- 1) the FIRST status carries a non-finite joint angle ----
  // 300 ms is ~37 status packets and stays under the 500 ms RX-silence park, so the
  // core is still in its pre-streaming mode when this is read.
  EXPECT_FALSE(core.wait_ready(0.3)) << "a poisoned status does not start streaming";
  {
    const TimingStats st = core.timing();
    const StateSnapshot snap = core.snapshot();
    EXPECT_GT(st.rx_nonfinite_drops, 0u) << "dropped at ingest and counted";
    EXPECT_EQ(snap.mode, Mode::STREAM_DOWN) << "begin_streaming never ran";
    EXPECT_TRUE(all_finite(snap.q_cmd)) << "the commanded anchor was never seeded from it";
    EXPECT_TRUE(all_finite(snap.q_meas));
    EXPECT_EQ(snap.fault, FaultReason::NONE) << "a dropped status is not a fault";
  }

  // ---- 2) a clean stream still prerolls ----
  poison.store(static_cast<int>(Poison::kNone), std::memory_order_relaxed);
  EXPECT_TRUE(core.wait_ready(3.0)) << "streaming starts on the first usable status";
  sleep_ms(150);
  {
    const StateSnapshot snap = core.snapshot();
    EXPECT_EQ(snap.mode, Mode::HOLD);
    EXPECT_TRUE(all_finite(snap.q_cmd));
    EXPECT_TRUE(all_finite(snap.q_meas));
  }

  // ---- 3) a non-finite safety_scale does not read as a cleared kill gate ----
  // 200 ms of drops: under the 500 ms park, and HOLD is not one of the modes the
  // 100 ms blind-hold escalation applies to.
  const std::uint64_t drops_before_scale = core.timing().rx_nonfinite_drops;
  poison.store(static_cast<int>(Poison::kSafetyScale), std::memory_order_relaxed);
  sleep_ms(200);
  {
    const TimingStats st = core.timing();
    const StateSnapshot snap = core.snapshot();
    EXPECT_GT(st.rx_nonfinite_drops, drops_before_scale);
    EXPECT_EQ(snap.conditions & kCondSafetyClamp, 0u) << "no clamp condition latched from a NaN";
    EXPECT_EQ(snap.fault, FaultReason::NONE);
    EXPECT_EQ(snap.mode, Mode::HOLD);
    EXPECT_TRUE(std::isfinite(snap.safety_scale)) << "published safety_scale stays usable";
  }

  // ---- 4) a non-finite Cartesian value never reaches the published snapshot ----
  poison.store(static_cast<int>(Poison::kCartesian), std::memory_order_relaxed);
  sleep_ms(150);
  {
    const StateSnapshot snap = core.snapshot();
    for (std::size_t i = 0; i < 6; ++i) {
      EXPECT_TRUE(std::isfinite(snap.cart[i])) << "cart[" << i << "]";
    }
  }

  // ---- 5) sustained poisoning escalates through the RX-silence ladder ----
  // Nothing usable for 800 ms > the 500 ms park threshold: the core parks TX, which
  // is the same terminal state as a stream that stopped arriving.
  poison.store(static_cast<int>(Poison::kJointAngle), std::memory_order_relaxed);
  sleep_ms(800);
  {
    const StateSnapshot snap = core.snapshot();
    EXPECT_EQ(snap.mode, Mode::RX_SILENT) << "parked, not wedged and not streaming NaN";
    EXPECT_TRUE(all_finite(snap.q_cmd));
  }

  peer_stop.store(true, std::memory_order_relaxed);
  peer_thr.join();
  core.stop();
  ::close(peer);

  EXPECT_GT(cmd_count.load(), 20u) << "the core did TX while the stream was usable";
  EXPECT_EQ(cmd_nonfinite.load(), 0u) << "no commanded joint position was ever non-finite";
}
