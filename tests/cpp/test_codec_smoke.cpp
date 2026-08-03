// SPDX-License-Identifier: Apache-2.0
//
// Smoke suite for the vendored Stream Motion codec + our wrapper. The sizes and
// byte offsets asserted here are pinned goldens of this implementation: the
// controller reads fields by fixed offset, so a struct that silently grows or
// reorders would put valid-looking garbage on the wire. Every sizeof and offset
// below must therefore be treated as load-bearing, not as documentation.

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "codec/codec.hpp"
#include "stream_motion/byte_ops.hpp"
#include "stream_motion/packets.hpp"

namespace {

bool host_le() {
  const std::uint16_t probe = 0x1;
  return *reinterpret_cast<const std::uint8_t*>(&probe) == 0x1;
}

// Read a big-endian float64 out of a wire buffer (reverses bytes on LE hosts).
double be_read_f64(const std::uint8_t* p) {
  std::uint8_t tmp[8];
  if (host_le()) {
    for (int i = 0; i < 8; ++i) {
      tmp[i] = p[7 - i];
    }
  } else {
    std::memcpy(tmp, p, 8);
  }
  double out;
  std::memcpy(&out, tmp, 8);
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Struct layout — the whole point of vendoring FANUC's #pragma pack(1) structs.
// ---------------------------------------------------------------------------
TEST(CodecLayout, PackedStructSizes) {
  EXPECT_EQ(sizeof(stream_motion::CommandPacket), 344u);
  EXPECT_EQ(sizeof(stream_motion::RobotStatusPacket), 416u);
  EXPECT_EQ(sizeof(stream_motion::V3RobotStatusPacket), 388u);  // 416 − 28 B force block
  EXPECT_EQ(sizeof(stream_motion::ForceSensorConfigPacket), 16u);
  EXPECT_EQ(sizeof(stream_motion::StartPacket), 8u);
  EXPECT_EQ(sizeof(stream_motion::StopPacket), 8u);
  EXPECT_EQ(airo_fanuc::codec::kCommandPacketSize, 344u);
  EXPECT_EQ(airo_fanuc::codec::kStatus204PacketSize, 416u);
  EXPECT_EQ(airo_fanuc::codec::kStatusV3PacketSize, 388u);
}

// ---------------------------------------------------------------------------
// CommandPacket encode: dataStyle safety pin + field bytes + position round trip.
// ---------------------------------------------------------------------------
TEST(CodecEncode, CommandPacketFields) {
  std::array<double, 9> pos{};
  for (int i = 0; i < 9; ++i) {
    pos[static_cast<std::size_t>(i)] = static_cast<double>(i) - 4.0;  // -4 .. +4
  }
  const std::uint32_t seq = 42;
  const std::uint8_t do_motn = 1;
  const auto buf = airo_fanuc::codec::encode_command_packet(seq, /*is_last=*/false, do_motn, pos);

  ASSERT_EQ(buf.size(), 344u);

  // packet_type (offset 0, BE u32) == 201.
  EXPECT_EQ(buf[0], 0x00);
  EXPECT_EQ(buf[1], 0x00);
  EXPECT_EQ(buf[2], 0x00);
  EXPECT_EQ(buf[3], 201);

  // version_no (offset 4, BE u32) == 3.
  EXPECT_EQ(buf[7], 3);

  // sequence_no (offset 8, BE u32) == 42.
  EXPECT_EQ(buf[11], 42);

  // is_last_command (offset 12) == 0.
  EXPECT_EQ(buf[12], 0);

  // do_motn_ctrl (offset 13) == 1.
  EXPECT_EQ(buf[13], 1);

  // SAFETY-CRITICAL: dataStyle (offset 14-15) MUST be 0xFFFF (joint angles).
  // Mirror of airo_fanuc.controller_facts.COMMAND_DATA_STYLE. Writing 0 here
  // makes the controller read command_pos as Cartesian XYZWPR (a real E-stop).
  EXPECT_EQ(buf[14], 0xFF);
  EXPECT_EQ(buf[15], 0xFF);

  // A couple of command_pos doubles decode back (BE f64 at offset 16 + 8*i).
  EXPECT_DOUBLE_EQ(be_read_f64(buf.data() + 16 + 8 * 0), -4.0);
  EXPECT_DOUBLE_EQ(be_read_f64(buf.data() + 16 + 8 * 4), 0.0);
  EXPECT_DOUBLE_EQ(be_read_f64(buf.data() + 16 + 8 * 8), 4.0);

  // io_command (offset 88..343) is all zero.
  for (std::size_t i = 88; i < 344; ++i) {
    EXPECT_EQ(buf[i], 0) << "io_command byte " << i << " must be zero";
  }
}

TEST(CodecEncode, IsLastCommandFlag) {
  std::array<double, 9> pos{};
  const auto buf = airo_fanuc::codec::encode_command_packet(1, /*is_last=*/true, /*do_motn=*/0, pos);
  EXPECT_EQ(buf[12], 1);  // is_last_command
  EXPECT_EQ(buf[13], 0);  // do_motn_ctrl passthrough
  // dataStyle pinned regardless of the io-only packet.
  EXPECT_EQ(buf[14], 0xFF);
  EXPECT_EQ(buf[15], 0xFF);
}

// ---------------------------------------------------------------------------
// Status-204 decode: hand-build a wire buffer, decode, assert native values.
// ---------------------------------------------------------------------------
namespace {

// Build a big-endian type-204 status buffer from native values, mirroring what
// the controller places on the wire (byte-swapped fields in a packed struct).
std::array<std::uint8_t, 416> build_status_204() {
  const bool swap = host_le();
  stream_motion::RobotStatusPacket p{};
  p.packet_type = stream_motion::swapBytesIfNeeded<std::uint32_t>(204, swap);
  p.version_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(3, swap);
  p.sequence_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(12345, swap);
  p.status = 0x01;                                        // motion_possible
  p.robot_status = 0x02;                                  // tp_enabled
  p.contact_stop_status = stream_motion::ContactStopStatus::SAFE;  // 1
  p.time_stamp = stream_motion::swapBytesIfNeeded<std::uint32_t>(987654, swap);
  for (int i = 0; i < 9; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    p.position[idx] = stream_motion::swapBytesIfNeeded<float>(10.0F + static_cast<float>(i), swap);
    p.joint_angle[idx] = stream_motion::swapBytesIfNeeded<float>(20.0F + static_cast<float>(i), swap);
    p.current[idx] = stream_motion::swapBytesIfNeeded<float>(30.0F + static_cast<float>(i), swap);
  }
  p.safety_scale = stream_motion::swapBytesIfNeeded<float>(0.875F, swap);
  p.force_x = stream_motion::swapBytesIfNeeded<float>(11.0F, swap);
  p.force_y = stream_motion::swapBytesIfNeeded<float>(12.0F, swap);
  p.force_z = stream_motion::swapBytesIfNeeded<float>(13.0F, swap);
  p.moment_x = stream_motion::swapBytesIfNeeded<float>(21.0F, swap);
  p.moment_y = stream_motion::swapBytesIfNeeded<float>(22.0F, swap);
  p.moment_z = stream_motion::swapBytesIfNeeded<float>(23.0F, swap);
  p.fs_type = stream_motion::swapBytesIfNeeded<std::uint32_t>(2, swap);  // EXTERNAL

  std::array<std::uint8_t, 416> buf{};
  std::memcpy(buf.data(), &p, buf.size());
  return buf;
}

}  // namespace

TEST(CodecDecode, Status204RoundTrip) {
  const auto buf = build_status_204();
  const auto v = airo_fanuc::codec::decode_status_204(buf.data(), buf.size());

  EXPECT_EQ(v.packet_type, 204u);
  EXPECT_EQ(v.version_no, 3u);
  EXPECT_EQ(v.sequence_no, 12345u);
  EXPECT_EQ(v.status, 0x01);
  EXPECT_EQ(v.robot_status, 0x02);
  EXPECT_EQ(v.contact_stop_status, 1);  // SAFE
  EXPECT_EQ(v.time_stamp, 987654u);

  for (int i = 0; i < 9; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_FLOAT_EQ(static_cast<float>(v.position[idx]), 10.0F + static_cast<float>(i));
    EXPECT_FLOAT_EQ(static_cast<float>(v.joint_angle[idx]), 20.0F + static_cast<float>(i));
    EXPECT_FLOAT_EQ(static_cast<float>(v.current[idx]), 30.0F + static_cast<float>(i));
  }
  EXPECT_DOUBLE_EQ(v.safety_scale, 0.875);  // exactly representable in float32
  EXPECT_FLOAT_EQ(static_cast<float>(v.force_x), 11.0F);
  EXPECT_FLOAT_EQ(static_cast<float>(v.force_z), 13.0F);
  EXPECT_FLOAT_EQ(static_cast<float>(v.moment_x), 21.0F);
  EXPECT_FLOAT_EQ(static_cast<float>(v.moment_z), 23.0F);
  EXPECT_EQ(v.fs_type, 2u);
}

TEST(CodecDecode, RejectsShortBuffer) {
  std::array<std::uint8_t, 100> tiny{};
  EXPECT_THROW(airo_fanuc::codec::decode_status_204(tiny.data(), tiny.size()), std::invalid_argument);
}

TEST(CodecDecode, RejectsWrongPacketType) {
  auto buf = build_status_204();
  // Corrupt the packet type (BE u32 at offset 0-3) to 202 (legacy V3).
  buf[3] = 202;
  EXPECT_THROW(airo_fanuc::codec::decode_status_204(buf.data(), buf.size()), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Status-202 (legacy V3, 388 B, NO force block) decode — what a controller
// streams when the negotiated Stream Motion version is 3. Same header as 204;
// force fields must decode to 0 and fs_type to 0xFFFFFFFF (Unavailable).
// ---------------------------------------------------------------------------
namespace {

std::array<std::uint8_t, 388> build_status_v3() {
  const bool swap = host_le();
  stream_motion::V3RobotStatusPacket p{};
  p.packet_type = stream_motion::swapBytesIfNeeded<std::uint32_t>(202, swap);
  p.version_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(3, swap);
  p.sequence_no = stream_motion::swapBytesIfNeeded<std::uint32_t>(6789, swap);
  p.status = 0x01;                                                 // motion_possible
  p.robot_status = 0x04;                                           // e_stopped
  p.contact_stop_status = stream_motion::ContactStopStatus::STOP;  // 2
  p.time_stamp = stream_motion::swapBytesIfNeeded<std::uint32_t>(555, swap);
  for (int i = 0; i < 9; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    p.position[idx] = stream_motion::swapBytesIfNeeded<float>(40.0F + static_cast<float>(i), swap);
    p.joint_angle[idx] = stream_motion::swapBytesIfNeeded<float>(50.0F + static_cast<float>(i), swap);
    p.current[idx] = stream_motion::swapBytesIfNeeded<float>(60.0F + static_cast<float>(i), swap);
  }
  p.safety_scale = stream_motion::swapBytesIfNeeded<float>(0.5F, swap);
  // No force block on the wire.
  std::array<std::uint8_t, 388> buf{};
  std::memcpy(buf.data(), &p, buf.size());
  return buf;
}

}  // namespace

TEST(CodecDecode, StatusV3RoundTrip) {
  const auto buf = build_status_v3();
  const auto v = airo_fanuc::codec::decode_status_v3(buf.data(), buf.size());

  EXPECT_EQ(v.packet_type, 202u);
  EXPECT_EQ(v.version_no, 3u);
  EXPECT_EQ(v.sequence_no, 6789u);
  EXPECT_EQ(v.status, 0x01);
  EXPECT_EQ(v.robot_status, 0x04);
  EXPECT_EQ(v.contact_stop_status, 2);  // STOP — collaborative contact stop
  EXPECT_EQ(v.time_stamp, 555u);
  for (int i = 0; i < 9; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    EXPECT_FLOAT_EQ(static_cast<float>(v.joint_angle[idx]), 50.0F + static_cast<float>(i));
  }
  EXPECT_DOUBLE_EQ(v.safety_scale, 0.5);
  // No force block → zeros, and fs_type Unavailable so wrench_valid stays false.
  EXPECT_DOUBLE_EQ(v.force_x, 0.0);
  EXPECT_DOUBLE_EQ(v.force_y, 0.0);
  EXPECT_DOUBLE_EQ(v.force_z, 0.0);
  EXPECT_DOUBLE_EQ(v.moment_x, 0.0);
  EXPECT_DOUBLE_EQ(v.moment_z, 0.0);
  EXPECT_EQ(v.fs_type, 0xFFFFFFFFu);
}

TEST(CodecDecode, StatusV3RejectsWrongType) {
  auto buf = build_status_v3();
  buf[3] = 204;  // wrong type for the v3 decoder
  EXPECT_THROW(airo_fanuc::codec::decode_status_v3(buf.data(), buf.size()), std::invalid_argument);
}

TEST(CodecDecode, StatusV3RejectsShortBuffer) {
  std::array<std::uint8_t, 100> tiny{};
  EXPECT_THROW(airo_fanuc::codec::decode_status_v3(tiny.data(), tiny.size()), std::invalid_argument);
}
