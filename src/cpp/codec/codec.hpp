// SPDX-License-Identifier: Apache-2.0
//
// airo_fanuc — thin, testable C++ codec over FANUC's vendored Stream Motion
// packet structs (vendor/fanuc_driver/fanuc_libs/stream_motion/include/
// stream_motion/packets.hpp).
//
// We vendor ONLY the packet structs + the byte-swap template; we do NOT use
// their socket layer (stream.cpp / sockpp). This header exposes the three codec
// operations (encode command, decode type-204, decode type-202), plus a
// plain-old-data view of a decoded status packet (type-202 or type-204) so both
// the pybind bindings and the gtest suite exercise the same code.
//
// WIRE ENCODING (the byte-for-byte contract; `airo_fanuc.testing.wire` implements
// the same layout in Python and `tests/goldens/sm/*.bin` pins the bytes, so the
// two implementations are cross-checked against fixed captures):
//   * Stream Motion is BIG-ENDIAN for every multi-byte scalar on the wire.
//   * The io_command / io_status byte arrays are byte-oriented (no swap).
//   * Joint angles are DEGREES on the wire; length-9 (kMaxAxisNumber), the CRX
//     is 6-DOF so the trailing 3 entries are zero-padded by the caller.
//   * The uint16 at CommandPacket offset 14 is the dataStyle selector and MUST be
//     0xFFFF. See the encode_command_packet note below — this one is capable of
//     damaging the robot.

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
// widened to double for convenience; the wire carries float32, which is exactly
// representable in double, so this matches the `>f` unpack in
// `airo_fanuc.testing.wire` value-for-value.
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
  std::array<std::uint8_t, static_cast<std::size_t>(kMaxIoBytes)> io_status{};
};

// Encode a Stream Motion CommandPacket (type 201, 344 B, big-endian).
//
// ╔════════════════════════════════════════════════════════════════════════════╗
// ║ SAFETY-CRITICAL — dataStyle MUST be 0xFFFF.                                ║
// ║                                                                            ║
// ║ The uint16 at CommandPacket offset 14 is named `unused` in FANUC's struct, ║
// ║ but it is NOT unused: it is the `dataStyle` selector that tells the        ║
// ║ controller how to interpret command_pos. 0xFFFF means "joint angles".      ║
// ║                                                                            ║
// ║ Write 0 — which is what a zero-initialised struct gives you — and the      ║
// ║ controller reads the same six doubles as a Cartesian XYZWPR pose           ║
// ║ instead of joint angles. Joint values interpreted as millimetres and       ║
// ║ Euler degrees describe a pose nowhere near the current one, so the arm     ║
// ║ takes a full-speed slew toward it and the controller E-stops. This has     ║
// ║ happened on real hardware. It is a hardware-damage-level mistake, not a    ║
// ║ protocol nicety.                                                           ║
// ║                                                                            ║
// ║ The value is single-sourced as COMMAND_DATA_STYLE in                       ║
// ║ `airo_fanuc.controller_facts`, mirrored by kCommandDataStyle in codec.cpp. ║
// ║ Never hardcode it at a call site, and never drop the write because the     ║
// ║ field is declared `unused` — this driver requires it written.              ║
// ╚════════════════════════════════════════════════════════════════════════════╝
//
// version_no in the COMMAND packet is pinned to stream_motion::kVersion (3), whatever
// version the session negotiated. Status decoding is not pinned: the 388-byte type-202 and
// the 416-byte type-204 both decode (see below), and RealtimeCore dispatches on the packet
// it actually received. What no version does is stream a force block, because this driver
// sends no ForceSensorConfig packet — that is the piece a force-capable session would need.
std::array<std::uint8_t, kCommandPacketSize> encode_command_packet(std::uint32_t sequence_no, bool is_last_command,
                                                                   std::uint8_t do_motn_ctrl,
                                                                   const std::array<double, 9>& command_pos_deg);

// Decode a Stream Motion type-204 RobotStatusPacket (416 B, big-endian).
// `len` must be >= kStatus204PacketSize and buf[0..3] must decode to packet
// type 204; throws std::invalid_argument otherwise.
RobotStatusView decode_status_204(const std::uint8_t* buf, std::size_t len);

// Decode a Stream Motion v3 type-202 RobotStatusPacket (388 B, big-endian). This
// is what a V9.40/P82 controller at Stream Motion v3 streams — a packet capture
// confirms the 388-byte type-202: identical header through `safety_scale`, but
// NO force/torque block. `len` must be >= kStatusV3PacketSize and buf[0..3] must
// decode to packet type 202; throws std::invalid_argument otherwise. The
// returned view carries force_x..moment_z = 0 and fs_type = 0xFFFFFFFF
// (Unavailable) so `wrench_valid` stays false and get_wrench() returns None.
RobotStatusView decode_status_v3(const std::uint8_t* buf, std::size_t len);

}  // namespace airo_fanuc::codec
