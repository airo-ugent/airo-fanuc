// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — thin, testable C++ codec over FANUC's vendored Stream Motion
// packet structs (fanuc_libs/stream_motion/include/stream_motion/packets.hpp).
//
// We vendor ONLY the packet structs + the byte-swap template; we do NOT use
// their socket layer (stream.cpp / sockpp). This header exposes the three codec
// operations (encode command, decode type-204, decode legacy type-202), plus a
// plain-old-data view of a decoded status packet (type-202 or type-204) so both
// the pybind bindings and the gtest suite exercise the same code.
//
// Byte order (ground truth: dries `src/grocery_bot/robot/fanuc/packets.py`):
//   * Stream Motion is BIG-ENDIAN for every multi-byte scalar on the wire.
//   * The io_command / io_status byte arrays are byte-oriented (no swap).
//   * Joint angles are DEGREES on the wire; length-9 (kMaxAxisNumber), the CRX
//     is 6-DOF so the trailing 3 entries are zero-padded by the caller.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "stream_motion/packets.hpp"

namespace airo_fanuc::codec {

// Wire sizes, derived from the vendored #pragma pack(1) structs. Kept as
// compile-time constants so callers/tests never hard-code magic numbers.
inline constexpr std::size_t kCommandPacketSize = sizeof(stream_motion::CommandPacket);              // 344
inline constexpr std::size_t kStatus204PacketSize = sizeof(stream_motion::RobotStatusPacket);        // 416
inline constexpr std::size_t kStatusV3PacketSize = sizeof(stream_motion::V3RobotStatusPacket);       // 388
inline constexpr std::size_t kForceSensorConfigPacketSize = sizeof(stream_motion::ForceSensorConfigPacket);  // 16
inline constexpr int kMaxAxes = stream_motion::kMaxAxisNumber;   // 9
inline constexpr int kMaxIoBytes = stream_motion::kMaxIOSize;    // 256

// Decoded, host-native view of a type-204 RobotStatusPacket. Force/moment are
// widened to double for convenience; the wire carries float32 (exactly
// representable in double, matching the Python oracle's struct '>f' unpack).
struct RobotStatusView {
  std::uint32_t packet_type{};
  std::uint32_t version_no{};
  std::uint32_t sequence_no{};
  std::uint8_t status{};
  std::uint8_t robot_status{};
  std::uint8_t contact_stop_status{};
  std::uint32_t time_stamp{};
  std::array<double, 9> position{};      // Cartesian XYZWPR + ext (deg / mm)
  std::array<double, 9> joint_angle{};   // degrees
  std::array<double, 9> current{};       // controller units
  double safety_scale{};
  double force_x{};
  double force_y{};
  double force_z{};
  double moment_x{};
  double moment_y{};
  double moment_z{};
  std::uint32_t fs_type{};
  std::array<std::uint8_t, 256> io_status{};
};

// Encode a Stream Motion CommandPacket (type 201, 344 B, big-endian).
//
// SAFETY-CRITICAL: the uint16 at offset 14 (FANUC names it `unused`) is the
// `dataStyle` selector and MUST be 0xFFFF (joint angles). Writing 0 makes the
// controller read command_pos as Cartesian XYZWPR → slew → E-stop (incident
// 2026-05-06). See `airo_fanuc.controller_facts.COMMAND_DATA_STYLE`.
//
// version_no is pinned to stream_motion::kVersion (3), matching the dries
// `encode_command_packet` default; a negotiated version is a P3+ concern.
std::array<std::uint8_t, kCommandPacketSize> encode_command_packet(std::uint32_t sequence_no, bool is_last_command,
                                                                   std::uint8_t do_motn_ctrl,
                                                                   const std::array<double, 9>& command_pos_deg);

// Decode a Stream Motion type-204 RobotStatusPacket (416 B, big-endian).
// `len` must be >= kStatus204PacketSize and buf[0..3] must decode to packet
// type 204; throws std::invalid_argument otherwise.
RobotStatusView decode_status_204(const std::uint8_t* buf, std::size_t len);

// Decode a legacy V3 type-202 RobotStatusPacket (388 B, big-endian). This is
// what the P-1 controller actually streams (V9.40/P82, Stream Motion v3 — the
// pcap proves 388-byte type-202): identical header through `safety_scale`, but
// NO force/torque block. `len` must be >= kStatusV3PacketSize and buf[0..3] must
// decode to packet type 202; throws std::invalid_argument otherwise. The
// returned view carries force_x..moment_z = 0 and fs_type = 0xFFFFFFFF
// (Unavailable) so `wrench_valid` stays false and get_wrench() returns None.
RobotStatusView decode_status_v3(const std::uint8_t* buf, std::size_t len);

}  // namespace airo_fanuc::codec
