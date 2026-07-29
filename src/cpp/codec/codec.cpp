// SPDX-License-Identifier: Apache-2.0

#include "codec/codec.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

#include "stream_motion/byte_ops.hpp"
#include "stream_motion/packets.hpp"

namespace airo_fanuc::codec {

namespace {

// Local endianness probe. Mirrors stream_motion::isLittleEndian() but is
// defined here so we never ODR-use the vendored symbol (byte_ops.cpp is not
// compiled — it carries an unused <fmt/format.h> include we decline to pull
// in). Stream Motion is big-endian on the wire, so we swap on LE hosts.
inline bool host_is_little_endian() {
  const std::uint16_t probe = 0x1;
  return *reinterpret_cast<const std::uint8_t*>(&probe) == 0x1;
}

// SAFETY-CRITICAL dataStyle value for the CommandPacket `unused` u16 at offset
// 14. Mirror of airo_fanuc.controller_facts.COMMAND_DATA_STYLE. 0xFFFF selects
// JOINT ANGLES; any other value (notably 0) makes the controller read command_pos
// as a Cartesian XYZWPR pose and slew the arm at full speed toward the garbage
// pose that implies. Never change this value. See codec.hpp for the full note.
constexpr std::uint16_t kCommandDataStyle = 0xFFFF;

// The vendored struct comment marks RobotStatusPacket as "// 204"; keep a named
// constant here rather than a bare literal at the validation site.
constexpr std::uint32_t kRobotStatusPacketType = 204;

// The vendored V3RobotStatusPacket comment marks it as "// 202" — the Stream
// Motion v3 status packet, which carries no force block. This is what a
// controller streams at v3.
constexpr std::uint32_t kV3RobotStatusPacketType = 202;

// fs_type sentinel meaning "no force sensor block on the wire" (matches the
// vendored RobotStatusPacket.fs_type doc: 0xFFFFFFFF = Unavailable). A type-202
// packet carries no force fields, so decode_status_v3 stamps this — it keeps
// wrench_valid (fs_type ∈ {1,2}) false end-to-end.
constexpr std::uint32_t kForceSensorUnavailable = 0xFFFFFFFFu;

}  // namespace

std::array<std::uint8_t, kCommandPacketSize> encode_command_packet(std::uint32_t sequence_no, bool is_last_command,
                                                                   std::uint8_t do_motn_ctrl,
                                                                   const std::array<double, 9>& command_pos_deg) {
  static_assert(sizeof(stream_motion::CommandPacket) == 344, "CommandPacket must be 344 B on the wire");

  const bool swap = host_is_little_endian();

  // Value-initialise so io_command[256] and every unset byte is zero, then set
  // every scalar with the wire (big-endian) byte order. Writing byte-swapped
  // values into a native-layout #pragma pack(1) struct and memcpy'ing the whole
  // struct out reproduces the big-endian wire packet exactly.
  stream_motion::CommandPacket cp{};
  cp.packet_type = stream_motion::swapBytesIfNeeded<std::uint32_t>(stream_motion::kCommandPacketType, swap);
  cp.version_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(stream_motion::kVersion, swap);
  cp.sequence_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(sequence_no, swap);
  cp.is_last_command = is_last_command ? 1u : 0u;
  cp.do_motn_ctrl = do_motn_ctrl;
  // dataStyle (0xFFFF = joint angles; MUST NOT be 0 — see kCommandDataStyle).
  // Byte-order-independent as a value, but routed through the swap for uniformity.
  cp.unused = stream_motion::swapBytesIfNeeded<std::uint16_t>(kCommandDataStyle, swap);
  for (int i = 0; i < kMaxAxes; ++i) {
    cp.command_pos[static_cast<std::size_t>(i)] = stream_motion::swapBytesIfNeeded<double>(command_pos_deg[static_cast<std::size_t>(i)], swap);
  }
  // io_command stays all-zero (we do not drive GPIO over Stream Motion).

  std::array<std::uint8_t, kCommandPacketSize> out{};
  std::memcpy(out.data(), &cp, kCommandPacketSize);
  return out;
}

RobotStatusView decode_status_204(const std::uint8_t* buf, std::size_t len) {
  static_assert(sizeof(stream_motion::RobotStatusPacket) == 416, "RobotStatusPacket must be 416 B on the wire");

  if (buf == nullptr) {
    throw std::invalid_argument("decode_status_204: null buffer");
  }
  if (len < kStatus204PacketSize) {
    throw std::invalid_argument("decode_status_204: buffer too short (need " + std::to_string(kStatus204PacketSize) +
                                " bytes, got " + std::to_string(len) + ")");
  }

  const bool swap = host_is_little_endian();

  stream_motion::RobotStatusPacket p{};
  std::memcpy(&p, buf, kStatus204PacketSize);

  const std::uint32_t packet_type = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.packet_type, swap);
  if (packet_type != kRobotStatusPacketType) {
    throw std::invalid_argument("decode_status_204: not a type-204 packet (got type " + std::to_string(packet_type) +
                                ")");
  }

  RobotStatusView v{};
  v.packet_type = packet_type;
  v.version_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.version_no, swap);
  v.sequence_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.sequence_no, swap);
  v.status = p.status;              // u8, no swap
  v.robot_status = p.robot_status;  // u8, no swap
  v.contact_stop_status = static_cast<std::uint8_t>(p.contact_stop_status);  // u8 enum, no swap
  v.time_stamp = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.time_stamp, swap);
  for (int i = 0; i < kMaxAxes; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    v.position[idx] = stream_motion::swapBytesIfNeeded<float>(p.position[idx], swap);
    v.joint_angle[idx] = stream_motion::swapBytesIfNeeded<float>(p.joint_angle[idx], swap);
    v.current[idx] = stream_motion::swapBytesIfNeeded<float>(p.current[idx], swap);
  }
  v.safety_scale = stream_motion::swapBytesIfNeeded<float>(p.safety_scale, swap);
  v.force_x = stream_motion::swapBytesIfNeeded<float>(p.force_x, swap);
  v.force_y = stream_motion::swapBytesIfNeeded<float>(p.force_y, swap);
  v.force_z = stream_motion::swapBytesIfNeeded<float>(p.force_z, swap);
  v.moment_x = stream_motion::swapBytesIfNeeded<float>(p.moment_x, swap);
  v.moment_y = stream_motion::swapBytesIfNeeded<float>(p.moment_y, swap);
  v.moment_z = stream_motion::swapBytesIfNeeded<float>(p.moment_z, swap);
  v.fs_type = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.fs_type, swap);
  v.io_status = p.io_status;  // byte-oriented, no swap

  return v;
}

RobotStatusView decode_status_v3(const std::uint8_t* buf, std::size_t len) {
  static_assert(sizeof(stream_motion::V3RobotStatusPacket) == 388, "V3RobotStatusPacket must be 388 B on the wire");

  if (buf == nullptr) {
    throw std::invalid_argument("decode_status_v3: null buffer");
  }
  if (len < kStatusV3PacketSize) {
    throw std::invalid_argument("decode_status_v3: buffer too short (need " + std::to_string(kStatusV3PacketSize) +
                                " bytes, got " + std::to_string(len) + ")");
  }

  const bool swap = host_is_little_endian();

  stream_motion::V3RobotStatusPacket p{};
  std::memcpy(&p, buf, kStatusV3PacketSize);

  const std::uint32_t packet_type = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.packet_type, swap);
  if (packet_type != kV3RobotStatusPacketType) {
    throw std::invalid_argument("decode_status_v3: not a type-202 packet (got type " + std::to_string(packet_type) +
                                ")");
  }

  RobotStatusView v{};
  v.packet_type = packet_type;
  v.version_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.version_no, swap);
  v.sequence_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.sequence_no, swap);
  v.status = p.status;              // u8, no swap
  v.robot_status = p.robot_status;  // u8, no swap
  v.contact_stop_status = static_cast<std::uint8_t>(p.contact_stop_status);  // u8 enum, no swap
  v.time_stamp = stream_motion::swapBytesIfNeeded<std::uint32_t>(p.time_stamp, swap);
  for (int i = 0; i < kMaxAxes; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    v.position[idx] = stream_motion::swapBytesIfNeeded<float>(p.position[idx], swap);
    v.joint_angle[idx] = stream_motion::swapBytesIfNeeded<float>(p.joint_angle[idx], swap);
    v.current[idx] = stream_motion::swapBytesIfNeeded<float>(p.current[idx], swap);
  }
  v.safety_scale = stream_motion::swapBytesIfNeeded<float>(p.safety_scale, swap);
  // No force block on the wire — leave force_x..moment_z at 0 and mark the force
  // sensor Unavailable so wrench_valid (fs_type ∈ {1,2}) is false everywhere.
  v.force_x = 0.0;
  v.force_y = 0.0;
  v.force_z = 0.0;
  v.moment_x = 0.0;
  v.moment_y = 0.0;
  v.moment_z = 0.0;
  v.fs_type = kForceSensorUnavailable;
  v.io_status = p.io_status;  // byte-oriented, no swap

  return v;
}

}  // namespace airo_fanuc::codec
