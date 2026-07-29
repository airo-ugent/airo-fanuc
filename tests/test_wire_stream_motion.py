# SPDX-License-Identifier: Apache-2.0
"""L0 byte-exact conformance tests for the Stream Motion wire codec.

Two layers of assurance:

1. **Golden fixtures** (``goldens/sm/*.bin``): each packet is encoded from
   fixed known inputs and byte-compared against a committed ``.bin``. These
   goldens double as the cross-language oracle for the future C++ codec — a
   ``.bin`` that changes without an intended layout change is a regression.
   Regenerate with ``AIRO_FANUC_REGEN_GOLDENS=1 pytest ...`` (writes the
   ``.bin`` files instead of asserting).

2. **Property / structural tests**: sizes, the safety-critical 0xFFFF
   dataStyle at its exact offset, do_motn_ctrl / io zero-pad, and
   encode→decode round-trips over a spread of joint configs.
"""

from __future__ import annotations

import os
import struct
from pathlib import Path

import numpy as np
import pytest

from airo_fanuc.controller_facts import COMMAND_DATA_STYLE
from airo_fanuc.testing import wire

_GOLDENS_DIR = Path(__file__).resolve().parent / "goldens" / "sm"
_REGEN = os.environ.get("AIRO_FANUC_REGEN_GOLDENS") == "1"

# ---------------------------------------------------------------------------
# Canonical fixed inputs (single source of truth for the goldens).
# ---------------------------------------------------------------------------

# CommandPacket nominal: 6-DOF arm command, axes 6-8 zero-padded, recognisable
# per-axis magnitudes so a botched offset shows up clearly.
_CMD_SEQ = 42
_CMD_POS_DEG = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0, 0.0, 0.0, 0.0]


def _sm_goldens() -> dict[str, bytes]:
    """Name -> canonical bytes for every Stream Motion golden.

    Encoded from the fixed inputs above. This is the ONE place inputs are
    defined; both the regen path and the verify path read it.
    """
    return {
        "start.bin": wire.encode_start_packet(),
        "stop.bin": wire.encode_stop_packet(),
        "get_capability.bin": wire.encode_get_capability_packet(),
        "capability_result.bin": wire.encode_capability_result_packet(
            sampling_rate_ms=8, available_version=3
        ),
        "force_sensor_config.bin": wire.encode_force_sensor_config_packet(
            do_reset=1, fs_type=wire.FORCE_SENSOR_TYPE_EMBEDDED
        ),
        "command_nominal.bin": wire.encode_command_packet(
            _CMD_SEQ, _CMD_POS_DEG, is_last_command=False, do_motn_ctrl=True
        ),
        "command_last.bin": wire.encode_command_packet(
            _CMD_SEQ, _CMD_POS_DEG, is_last_command=True, do_motn_ctrl=True
        ),
        "command_io_only.bin": wire.encode_command_packet(
            _CMD_SEQ, _CMD_POS_DEG, is_last_command=False, do_motn_ctrl=False
        ),
        "status_204.bin": wire._build_synthetic_status_204(),
        "status_v3.bin": wire._build_synthetic_status_v3(),
    }


def _load_or_regen(name: str, data: bytes) -> bytes:
    """Read a committed golden, or (in regen mode) write it and return it."""
    path = _GOLDENS_DIR / name
    if _REGEN:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return data
    assert path.is_file(), (
        f"missing golden {path} — regenerate with AIRO_FANUC_REGEN_GOLDENS=1 pytest {Path(__file__).name}"
    )
    return path.read_bytes()


# ---------------------------------------------------------------------------
# Golden regeneration guard.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(_sm_goldens().keys()))
def test_sm_golden_matches(name: str) -> None:
    """Re-encode from fixed inputs and byte-compare against the committed .bin."""
    data = _sm_goldens()[name]
    committed = _load_or_regen(name, data)
    assert data == committed, (
        f"{name}: encoder output diverged from committed golden "
        f"({len(data)} B vs {len(committed)} B). If the layout change is "
        f"intentional, regenerate with AIRO_FANUC_REGEN_GOLDENS=1."
    )


# ---------------------------------------------------------------------------
# Byte-exact sizes.
# ---------------------------------------------------------------------------


def test_packet_sizes() -> None:
    assert len(wire.encode_start_packet()) == wire.START_PACKET_SIZE == 8
    assert len(wire.encode_stop_packet()) == wire.STOP_PACKET_SIZE == 8
    assert len(wire.encode_get_capability_packet()) == wire.CAPABILITY_PACKET_SIZE == 28
    assert len(wire.encode_force_sensor_config_packet()) == wire.FORCE_SENSOR_CONFIG_PACKET_SIZE == 16
    assert len(wire.encode_command_packet(0, [0.0] * wire.MAX_AXES)) == wire.COMMAND_PACKET_SIZE == 344
    assert len(wire._build_synthetic_status_204()) == wire.STATUS_204_PACKET_SIZE == 416
    assert len(wire._build_synthetic_status_v3()) == wire.STATUS_V3_PACKET_SIZE == 388


def test_start_stop_type_codes() -> None:
    pt, ver = struct.unpack(">II", wire.encode_start_packet(version_no=3))
    assert (pt, ver) == (wire.PACKET_TYPE_START, 3)
    pt, ver = struct.unpack(">II", wire.encode_stop_packet(version_no=3))
    assert (pt, ver) == (wire.PACKET_TYPE_STOP, 3)


# ---------------------------------------------------------------------------
# SAFETY-CRITICAL: dataStyle == 0xFFFF at the correct offset.
# ---------------------------------------------------------------------------


def test_command_datastyle_is_ffff_at_offset() -> None:
    """The uint16 dataStyle MUST serialize as 0xFFFF (big-endian) at offset 14.

    Writing 0 makes the controller read joint values as Cartesian XYZWPR and
    slew to the implied pose (E-stop save, 2026-05-06). This asserts both the
    value and its exact wire position.
    """
    assert COMMAND_DATA_STYLE == 0xFFFF  # sourced from controller_facts
    cmd = wire.encode_command_packet(1, _CMD_POS_DEG)
    off = wire.COMMAND_DATA_STYLE_OFFSET
    assert off == 14
    assert cmd[off : off + 2] == b"\xff\xff"
    (datastyle,) = struct.unpack_from(">H", cmd, off)
    assert datastyle == 0xFFFF == COMMAND_DATA_STYLE


def test_command_do_motn_ctrl_and_is_last_bytes() -> None:
    cmd = wire.encode_command_packet(1, _CMD_POS_DEG, is_last_command=True, do_motn_ctrl=True)
    assert cmd[wire.COMMAND_IS_LAST_OFFSET] == 1
    assert cmd[wire.COMMAND_DO_MOTN_CTRL_OFFSET] == 1

    cmd = wire.encode_command_packet(1, _CMD_POS_DEG, is_last_command=False, do_motn_ctrl=False)
    assert cmd[wire.COMMAND_IS_LAST_OFFSET] == 0
    assert cmd[wire.COMMAND_DO_MOTN_CTRL_OFFSET] == 0


def test_command_io_is_zero_padded() -> None:
    """The 256-byte io_command block is all zeros in the nominal command."""
    cmd = wire.encode_command_packet(_CMD_SEQ, _CMD_POS_DEG)
    io = cmd[wire.COMMAND_IO_OFFSET :]
    assert len(io) == wire.MAX_IO_BYTES == 256
    assert io == b"\x00" * 256


def test_command_axes_6_to_8_zero_padded() -> None:
    """A 6-DOF command fills axes 0-5; axes 6-8 are zero on the wire."""
    cmd = wire.encode_command_packet(_CMD_SEQ, _CMD_POS_DEG)
    positions = struct.unpack_from(">9d", cmd, wire.COMMAND_POS_OFFSET)
    assert list(positions) == _CMD_POS_DEG
    assert positions[6] == positions[7] == positions[8] == 0.0


def test_command_header_fields() -> None:
    cmd = wire.encode_command_packet(_CMD_SEQ, _CMD_POS_DEG, version_no=3)
    pt, ver, seq = struct.unpack_from(">III", cmd, 0)
    assert pt == wire.PACKET_TYPE_COMMAND == 201
    assert ver == 3
    assert seq == _CMD_SEQ


def test_command_length_validation() -> None:
    with pytest.raises(ValueError):
        wire.encode_command_packet(0, [0.0] * (wire.MAX_AXES - 1))
    with pytest.raises(ValueError):
        wire.encode_command_packet(-1, [0.0] * wire.MAX_AXES)  # seq out of range
    with pytest.raises(ValueError):
        wire.encode_command_packet(0x1_0000_0000, [0.0] * wire.MAX_AXES)


# ---------------------------------------------------------------------------
# Force sensor config field order.
# ---------------------------------------------------------------------------


def test_force_sensor_config_fields() -> None:
    buf = wire.encode_force_sensor_config_packet(
        version_no=4, do_reset=1, fs_type=wire.FORCE_SENSOR_TYPE_EXTERNAL
    )
    pt, ver, do_reset, fs_type = struct.unpack(">IIII", buf)
    assert pt == wire.PACKET_TYPE_FORCE_SENSOR_CONFIG == 205
    assert ver == 4
    assert do_reset == 1
    assert fs_type == wire.FORCE_SENSOR_TYPE_EXTERNAL == 2


# ---------------------------------------------------------------------------
# Capability request / result.
# ---------------------------------------------------------------------------


def test_capability_request_fields() -> None:
    buf = wire.encode_get_capability_packet(version_no=3)
    fields = struct.unpack(">IIIIIII", buf)
    assert fields[0] == wire.PACKET_TYPE_GET_CAPABILITY == 7
    assert fields[1] == 3  # version_no
    assert fields[2] == 1  # id
    assert fields[3:] == (0, 0, 0, 0)  # sampling_rate, start_move, avail_ver, use_tcp


def test_capability_result_decode() -> None:
    buf = wire.encode_capability_result_packet(
        packet_type=wire.PACKET_TYPE_GET_CAPABILITY,
        version_no=3,
        id=1,
        sampling_rate_ms=8,
        start_move=2,
        available_version=4,
        rob_status_use_tcp=1,
    )
    res = wire.decode_capability_result_packet(buf)
    assert res is not None
    assert res.sampling_rate_ms == 8
    assert res.available_version == 4
    assert res.start_move == 2
    assert res.rob_status_use_tcp == 1
    # SET type code (8) also accepted.
    buf_set = wire.encode_capability_result_packet(packet_type=wire.PACKET_TYPE_SET_CAPABILITY)
    assert wire.decode_capability_result_packet(buf_set) is not None
    # Wrong type / too short rejected.
    assert wire.decode_capability_result_packet(struct.pack(">IIIIIII", 999, 0, 0, 0, 0, 0, 0)) is None
    assert wire.decode_capability_result_packet(b"\x00" * 10) is None


# ---------------------------------------------------------------------------
# Status-204 decode of a hand-built buffer: bits, enum, force/moment, fs_type.
# ---------------------------------------------------------------------------


def test_status_204_decode_bits_and_fields() -> None:
    buf = wire.build_status_204_packet(
        version_no=3,
        sequence_no=555,
        status=0x1 | 0x8,  # motion_possible | motion_in_progress
        robot_status=0x1 | 0x2 | 0x4,  # in_error | tp_enabled | e_stopped
        contact_stop_status=int(wire.ContactStopStatus.STOP),
        time_stamp=13579,
        position_deg=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0],
        joint_angle_deg=[-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, 0.0, 0.0, 0.0],
        current=[0.1 * i for i in range(wire.MAX_AXES)],
        safety_scale=0.5,  # exactly representable in float32
        force_x=1.5,
        force_y=-2.5,
        force_z=3.5,
        moment_x=-0.25,
        moment_y=0.75,
        moment_z=-1.25,
        fs_type=wire.FORCE_SENSOR_TYPE_EMBEDDED,
    )
    assert len(buf) == wire.STATUS_204_PACKET_SIZE
    pkt = wire.decode_status_packet(buf)
    assert pkt is not None
    assert pkt.packet_type == 204
    assert pkt.sequence_no == 555
    assert pkt.time_stamp == 13579

    # status bits.
    assert pkt.motion_possible is True
    assert pkt.motion_in_progress is True
    # robot_status bits.
    assert pkt.in_error is True
    assert pkt.tp_enabled is True
    assert pkt.e_stopped is True
    # contact_stop enum.
    assert pkt.contact_stop_status is wire.ContactStopStatus.STOP

    # safety_scale (float32-exact).
    assert pkt.safety_scale == 0.5

    # force/moment (float32-exact values chosen above).
    assert (pkt.force_x, pkt.force_y, pkt.force_z) == (1.5, -2.5, 3.5)
    assert (pkt.moment_x, pkt.moment_y, pkt.moment_z) == (-0.25, 0.75, -1.25)
    assert pkt.fs_type == wire.FORCE_SENSOR_TYPE_EMBEDDED == 1

    # Positions/joints float32-exact for these small integers.
    assert pkt.position_deg == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0]
    assert pkt.joint_angle_deg == [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, 0.0, 0.0, 0.0]


def test_status_204_zero_bits() -> None:
    """All-zero status bytes decode to all-False."""
    buf = wire.build_status_204_packet(
        status=0x0,
        robot_status=0x0,
        contact_stop_status=int(wire.ContactStopStatus.NONE),
        position_deg=[0.0] * wire.MAX_AXES,
        joint_angle_deg=[0.0] * wire.MAX_AXES,
        current=[0.0] * wire.MAX_AXES,
        safety_scale=1.0,
        force_x=0.0,
        force_y=0.0,
        force_z=0.0,
        moment_x=0.0,
        moment_y=0.0,
        moment_z=0.0,
        fs_type=wire.FORCE_SENSOR_TYPE_UNSELECTED,
    )
    pkt = wire.decode_status_packet(buf)
    assert pkt is not None
    assert not pkt.motion_possible
    assert not pkt.motion_in_progress
    assert not pkt.in_error
    assert not pkt.tp_enabled
    assert not pkt.e_stopped
    assert pkt.contact_stop_status is wire.ContactStopStatus.NONE


def test_status_v3_has_no_force_block() -> None:
    buf = wire.build_status_v3_packet(
        sequence_no=222,
        status=0x1,
        robot_status=0x2,
        contact_stop_status=int(wire.ContactStopStatus.SAFE),
        time_stamp=99,
        position_deg=[0.0] * wire.MAX_AXES,
        joint_angle_deg=[0.0] * wire.MAX_AXES,
        current=[0.0] * wire.MAX_AXES,
        safety_scale=1.0,
    )
    assert len(buf) == wire.STATUS_V3_PACKET_SIZE
    pkt = wire.decode_status_packet(buf)
    assert pkt is not None
    assert pkt.packet_type == 202
    assert pkt.force_x is None and pkt.moment_z is None and pkt.fs_type is None
    assert pkt.motion_possible is True
    assert pkt.tp_enabled is True


def test_contact_stop_enum_all_values() -> None:
    for raw, expected in (
        (0, wire.ContactStopStatus.NONE),
        (1, wire.ContactStopStatus.SAFE),
        (2, wire.ContactStopStatus.STOP),
        (3, wire.ContactStopStatus.DSBL),
        (4, wire.ContactStopStatus.ESCP),
        (99, wire.ContactStopStatus.NONE),  # forward-compat: unknown -> NONE
    ):
        buf = wire.build_status_204_packet(
            contact_stop_status=raw,
            position_deg=[0.0] * wire.MAX_AXES,
            joint_angle_deg=[0.0] * wire.MAX_AXES,
            current=[0.0] * wire.MAX_AXES,
            safety_scale=1.0,
            force_x=0.0,
            force_y=0.0,
            force_z=0.0,
            moment_x=0.0,
            moment_y=0.0,
            moment_z=0.0,
            fs_type=0,
        )
        pkt = wire.decode_status_packet(buf)
        assert pkt is not None and pkt.contact_stop_status is expected


def test_decode_rejects_garbage() -> None:
    assert wire.decode_status_packet(b"") is None
    assert wire.decode_status_packet(b"\x00" * 16) is None
    bad = struct.pack(">I", 999) + b"\x00" * (wire.STATUS_204_PACKET_SIZE - 4)
    assert wire.decode_status_packet(bad) is None
    assert wire.decode_status_packet(wire._build_synthetic_status_204()[:-1]) is None


# ---------------------------------------------------------------------------
# Property-style round-trips.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("seed", range(8))
def test_command_roundtrip_random_degrees(seed: int) -> None:
    """encode CommandPacket -> parse fields -> recover joint degrees exactly.

    command_pos is float64 (double) on the wire, so a Python float round-trips
    bit-exactly; we assert exact equality (tolerance would only hide a bug).
    """
    rng = np.random.default_rng(seed)
    q6 = rng.uniform(-180.0, 180.0, size=6)
    q9 = [float(v) for v in q6] + [0.0, 0.0, 0.0]
    seq = int(rng.integers(0, 0xFFFF_FFFF, dtype=np.uint32))
    cmd = wire.encode_command_packet(seq, q9)
    fields = struct.unpack(wire._FMT_COMMAND, cmd)
    assert fields[2] == seq
    assert fields[5] == COMMAND_DATA_STYLE
    recovered = list(fields[6 : 6 + wire.MAX_AXES])
    assert recovered == q9  # bit-exact (double)


@pytest.mark.parametrize("seed", range(8))
def test_status_204_roundtrip_random(seed: int) -> None:
    """build status-204 -> decode -> recover arrays within float32 tolerance.

    position/joint/current/force/moment are float32 on the wire, so decode
    returns float32-rounded values; compare against the float32 projection.
    """
    rng = np.random.default_rng(1000 + seed)
    pos = rng.uniform(-360.0, 360.0, size=wire.MAX_AXES)
    joint = rng.uniform(-360.0, 360.0, size=wire.MAX_AXES)
    current = rng.uniform(-10.0, 10.0, size=wire.MAX_AXES)
    fx, fy, fz, mx, my, mz = rng.uniform(-50.0, 50.0, size=6)
    safety = float(rng.uniform(0.0, 1.0))

    buf = wire.build_status_204_packet(
        sequence_no=seed,
        status=0x1,
        robot_status=0x2,
        contact_stop_status=int(wire.ContactStopStatus.SAFE),
        time_stamp=seed * 1000,
        position_deg=[float(v) for v in pos],
        joint_angle_deg=[float(v) for v in joint],
        current=[float(v) for v in current],
        safety_scale=safety,
        force_x=float(fx),
        force_y=float(fy),
        force_z=float(fz),
        moment_x=float(mx),
        moment_y=float(my),
        moment_z=float(mz),
        fs_type=wire.FORCE_SENSOR_TYPE_EMBEDDED,
    )
    pkt = wire.decode_status_packet(buf)
    assert pkt is not None

    np.testing.assert_allclose(pkt.position_deg, pos.astype(np.float32), rtol=0, atol=1e-2)
    np.testing.assert_allclose(pkt.joint_angle_deg, joint.astype(np.float32), rtol=0, atol=1e-2)
    np.testing.assert_allclose(pkt.current, current.astype(np.float32), rtol=0, atol=1e-4)
    np.testing.assert_allclose(
        [pkt.force_x, pkt.force_y, pkt.force_z],
        np.array([fx, fy, fz], dtype=np.float32),
        rtol=0,
        atol=1e-3,
    )
    np.testing.assert_allclose(
        [pkt.moment_x, pkt.moment_y, pkt.moment_z],
        np.array([mx, my, mz], dtype=np.float32),
        rtol=0,
        atol=1e-3,
    )
    np.testing.assert_allclose(pkt.safety_scale, np.float32(safety), rtol=0, atol=1e-6)
