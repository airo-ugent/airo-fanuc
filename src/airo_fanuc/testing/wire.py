# SPDX-License-Identifier: Apache-2.0
"""Byte-exact wire-protocol oracle for the FANUC CRX-10iA/L driver.

Pure data layer, **no I/O**. This module is the language-independent ground
truth for every byte the driver puts on (or reads off) the wire:

* **Stream Motion (UDP 60015)** — Start / Stop / GetCapability / Command /
  RobotStatus (204 + legacy V3 202) / ForceSensorConfig packet codecs.
* **RMI JSON (TCP 16001)** — canonical JSON request serializers;
  :class:`~airo_fanuc.rmi_client.RmiClient` builds every request through them,
  so the client inherits their exact string form.

Byte layout
-----------
The encoders and decoders below reproduce the byte layout the controller
requires: FANUC's Apache-2.0 ``#pragma pack(push, 1)`` structs as declared in
the vendored header
``vendor/fanuc_driver/fanuc_libs/stream_motion/include/stream_motion/packets.hpp``.
RMI requests are one JSON object per line, ``json.dumps(req) + "\\r\\n"``
ASCII-encoded.

That layout is **locked by byte-exact goldens**: ``tests/goldens/sm/`` holds the
expected Stream Motion buffers and ``tests/goldens/rmi/`` the expected RMI wire
strings. A changed struct format string, a reordered field, or a flipped
endianness therefore fails a golden comparison in the test suite instead of
surfacing as a controller alarm.

Dependency-light: **stdlib ``struct`` / ``json`` only** for the codec itself;
the single external import is ``COMMAND_DATA_STYLE`` from
:mod:`airo_fanuc.controller_facts` (the 0xFFFF dataStyle safety constant —
imported, never hardcoded, so it stays single-sourced).

Conventions
-----------
- **Big-endian on the wire.** FANUC Stream Motion is BE for every multi-byte
  field (uint32 / uint16 / float / double); the vendored C++ driver applies
  ``swapBytesIfNeeded`` to every scalar on send / receive. The ``io_command``
  / ``io_status`` arrays are byte-oriented (``uint8``) and endian-agnostic —
  we encode them as plain ``B`` within the BE struct. (The C++ header's
  "io always little endian" note concerns how to interpret *multi-byte values
  packed inside* that byte array; the array storage is just bytes and we
  zero-fill it, so it does not affect any byte we emit.)
- Joint angles are **degrees on the wire**; callers convert rad↔deg at the
  API boundary.
- Position arrays are length-9 (``MAX_AXES``); the CRX-10iA/L is 6-DOF, so
  callers zero-pad the trailing 3 entries.

Run ``python -m airo_fanuc.testing.wire`` to execute the encode → decode →
assert round-trip smoke test at the bottom of the file.
"""

from __future__ import annotations

import json
import struct
from collections.abc import Sequence
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

from airo_fanuc.controller_facts import COMMAND_DATA_STYLE

# ---------------------------------------------------------------------------
# Wire constants — keep in sync with stream_motion::packets.hpp.
# ---------------------------------------------------------------------------

STREAM_MOTION_VERSION: int = 3

# Packet type codes.
PACKET_TYPE_STOP: int = 2
PACKET_TYPE_GET_CAPABILITY: int = 7
PACKET_TYPE_SET_CAPABILITY: int = 8
PACKET_TYPE_START: int = 200
PACKET_TYPE_COMMAND: int = 201
PACKET_TYPE_STATUS_V3: int = 202  # legacy V3 status packet (no force data)
PACKET_TYPE_STATUS_204: int = 204  # current status packet (force/torque + fs_type)
PACKET_TYPE_FORCE_SENSOR_CONFIG: int = 205  # FS-config request (v >= 4 only)

# Force sensor type IDs (mirrors stream_motion::FORCE_SENSOR_*).
FORCE_SENSOR_TYPE_UNSELECTED: int = 0
FORCE_SENSOR_TYPE_EMBEDDED: int = 1
FORCE_SENSOR_TYPE_EXTERNAL: int = 2
FORCE_SENSOR_TYPE_UNAVAILABLE: int = 0xFFFFFFFF

# Sizing constants (``kMaxAxisNumber``, ``kMaxIOSize``).
MAX_AXES: int = 9
MAX_IO_BYTES: int = 256

# Default UDP port for Stream Motion.
UDP_PORT: int = 60015

# Wire-level packet sizes (verified against pragma-pack(push,1) layouts).
START_PACKET_SIZE: int = 8
STOP_PACKET_SIZE: int = 8
CAPABILITY_PACKET_SIZE: int = 28  # ControllerCapability {Request,Result} both 7×uint32
COMMAND_PACKET_SIZE: int = 344
STATUS_204_PACKET_SIZE: int = 416
STATUS_V3_PACKET_SIZE: int = 388
FORCE_SENSOR_CONFIG_PACKET_SIZE: int = 16  # type(4) + version_no(4) + do_reset(4) + fs_type(4)

# Byte offset of the dataStyle uint16 inside the CommandPacket:
#   uint32 packet_type (0) | uint32 version_no (4) | uint32 sequence_no (8)
#   uint8 is_last_command (12) | uint8 do_motn_ctrl (13) | uint16 dataStyle (14)
COMMAND_DATA_STYLE_OFFSET: int = 14
COMMAND_DO_MOTN_CTRL_OFFSET: int = 13
COMMAND_IS_LAST_OFFSET: int = 12
COMMAND_POS_OFFSET: int = 16  # first byte of command_pos[0] (double)
COMMAND_IO_OFFSET: int = 16 + 8 * MAX_AXES  # 88; first byte of io_command[0]


class ContactStopStatus(IntEnum):
    """Mirror of ``stream_motion::ContactStopStatus``."""

    NONE = 0
    SAFE = 1
    STOP = 2
    DSBL = 3
    ESCP = 4

    @classmethod
    def from_wire(cls, raw: int) -> ContactStopStatus:
        try:
            return cls(raw)
        except ValueError:
            # Forward-compat: unknown values map to NONE rather than raising,
            # so a controller firmware bump can't crash the RX thread.
            return cls.NONE


# ---------------------------------------------------------------------------
# Struct format strings (verified against packets.hpp; see module docstring).
#
#   StartPacket / StopPacket:
#       uint32 packet_type, uint32 version_no                   →   8 B
#
#   CommandPacket (type 201):
#       uint32 packet_type, uint32 version_no, uint32 sequence_no,
#       uint8  is_last_command, uint8 do_motn_ctrl, uint16 dataStyle,
#       double[9] command_pos, uint8[256] io_command            → 344 B
#
#   RobotStatusPacket (type 204):
#       uint32 packet_type, uint32 version_no, uint32 sequence_no,
#       uint8  status, uint8 robot_status, uint8 contact_stop_status,
#       uint8  unused, uint32 time_stamp,
#       float[9] position, float[9] joint_angle, float[9] current,
#       float  safety_scale,
#       float  force_x, force_y, force_z,
#       float  moment_x, moment_y, moment_z,
#       uint32 fs_type, uint8[256] io_status                    → 416 B
#
#   V3RobotStatusPacket (type 202): same up through safety_scale,
#   then uint8[256] io_status                                   → 388 B
# ---------------------------------------------------------------------------

_FMT_START_OR_STOP = ">II"

_FMT_COMMAND = (
    ">"
    "III"  # packet_type, version_no, sequence_no
    "BBH"  # is_last_command, do_motn_ctrl, dataStyle
    "9d"  # command_pos[9]
    "256B"  # io_command[256]  (byte-oriented; endianness irrelevant)
)

# Header shared by 204 and V3-202, up to and including safety_scale.
_FMT_STATUS_HEADER = (
    ">"
    "III"  # packet_type, version_no, sequence_no
    "BBBB"  # status, robot_status, contact_stop_status, unused
    "I"  # time_stamp
    "9f"  # position[9]
    "9f"  # joint_angle[9]
    "9f"  # current[9]
    "f"  # safety_scale
)

_FMT_STATUS_204_TAIL = (
    ">"
    "ffffff"  # force_x, force_y, force_z, moment_x, moment_y, moment_z
    "I"  # fs_type
    "256B"  # io_status[256]  (byte-oriented; endianness irrelevant)
)

_FMT_STATUS_V3_TAIL = ">256B"  # io_status[256]

# ControllerCapability {Request,Result}: 7 × uint32 = 28 B.
_FMT_CAPABILITY = ">IIIIIII"

# ForceSensorConfigPacket (type 205): 4 × uint32 = 16 B.
#   uint32 packet_type, uint32 version_no, uint32 do_reset, uint32 fs_type
_FMT_FORCE_SENSOR_CONFIG = ">IIII"

_STATUS_HEADER_SIZE = struct.calcsize(_FMT_STATUS_HEADER)  # 132
_STATUS_204_TAIL_SIZE = struct.calcsize(_FMT_STATUS_204_TAIL)  # 284
_STATUS_V3_TAIL_SIZE = struct.calcsize(_FMT_STATUS_V3_TAIL)  # 256

# Sanity-check the layout sizes at import time so a typo here surfaces
# immediately rather than as a "received N bytes, expected M" runtime error.
assert struct.calcsize(_FMT_START_OR_STOP) == START_PACKET_SIZE
assert struct.calcsize(_FMT_COMMAND) == COMMAND_PACKET_SIZE
assert struct.calcsize(_FMT_CAPABILITY) == CAPABILITY_PACKET_SIZE
assert struct.calcsize(_FMT_FORCE_SENSOR_CONFIG) == FORCE_SENSOR_CONFIG_PACKET_SIZE
assert _STATUS_HEADER_SIZE + _STATUS_204_TAIL_SIZE == STATUS_204_PACKET_SIZE
assert _STATUS_HEADER_SIZE + _STATUS_V3_TAIL_SIZE == STATUS_V3_PACKET_SIZE


# ---------------------------------------------------------------------------
# Decoded form.
# ---------------------------------------------------------------------------


@dataclass
class RobotStatusPacket:
    """Decoded Stream Motion status packet (type 204 or legacy V3 type 202).

    All multi-axis arrays are length-9 (``MAX_AXES``). The CRX-10iA/L is
    6-DOF — the trailing three entries are zero in practice.

    Force / moment / ``fs_type`` are ``None`` when the packet is the legacy
    V3 type-202 layout, which omits the force-sensor block.
    """

    packet_type: int
    version_no: int
    sequence_no: int
    status: int
    robot_status: int
    contact_stop_status: ContactStopStatus
    time_stamp: int  # controller-local ms timestamp (uint32 wraps after ~49.7 d)
    position_deg: list[float]  # length MAX_AXES, Cartesian XYZWPR + ext
    joint_angle_deg: list[float]  # length MAX_AXES
    current: list[float]  # length MAX_AXES, motor current (controller units)
    safety_scale: float
    # type-204 only — None on the legacy V3 type-202 layout.
    force_x: float | None
    force_y: float | None
    force_z: float | None
    moment_x: float | None
    moment_y: float | None
    moment_z: float | None
    fs_type: int | None  # 0=Unselected, 1=EMBEDDED, 2=EXTERNAL, 0xFFFFFFFF=Unavailable

    # ------------------------------------------------------------------
    # Decoded status bits (FANUC Stream Motion status-bitfield convention).
    # ------------------------------------------------------------------

    @property
    def in_error(self) -> bool:
        return bool(self.robot_status & 0x1)

    @property
    def tp_enabled(self) -> bool:
        return bool(self.robot_status & 0x2)

    @property
    def e_stopped(self) -> bool:
        return bool(self.robot_status & 0x4)

    @property
    def motion_possible(self) -> bool:
        return bool(self.status & 0x1)

    @property
    def motion_in_progress(self) -> bool:
        return bool(self.status & 0x8)


@dataclass(frozen=True)
class CapabilityResult:
    """Decoded ControllerCapability response (type 7 result).

    ``available_version`` is what the caller MUST use for all subsequent
    Start / Stop / Command packets. ``sampling_rate_ms`` is the controller's
    output cycle in milliseconds (typically 8 = 125 Hz on R-30iB).
    """

    packet_type: int
    version_no: int
    id: int
    sampling_rate_ms: int
    start_move: int
    available_version: int
    rob_status_use_tcp: int


# ---------------------------------------------------------------------------
# Stream Motion encoders.
# ---------------------------------------------------------------------------


def encode_start_packet(*, version_no: int = STREAM_MOTION_VERSION) -> bytes:
    """Encode a Stream Motion StartPacket (type 200).

    ``version_no`` must match what the controller advertised in the
    GetCapability response.
    """
    return struct.pack(_FMT_START_OR_STOP, PACKET_TYPE_START, int(version_no))


def encode_stop_packet(*, version_no: int = STREAM_MOTION_VERSION) -> bytes:
    """Encode a Stream Motion StopPacket (type 2)."""
    return struct.pack(_FMT_START_OR_STOP, PACKET_TYPE_STOP, int(version_no))


def encode_get_capability_packet(*, version_no: int = STREAM_MOTION_VERSION) -> bytes:
    """Encode a ControllerCapability request (type 7).

    All non-version fields are zero on the request except ``id=1`` (matches
    FANUC C++ default); the controller fills the read-only fields on the
    response.
    """
    return struct.pack(
        _FMT_CAPABILITY,
        PACKET_TYPE_GET_CAPABILITY,
        int(version_no),
        1,  # id (matches FANUC C++ default)
        0,  # sampling_rate (read-only, ignored on request)
        0,  # start_move
        0,  # available_version (read-only)
        0,  # rob_status_use_tcp
    )


def decode_capability_result_packet(buf: bytes) -> CapabilityResult | None:
    """Decode a ControllerCapability response (type 7 or 8).

    Returns ``None`` if ``buf`` is too short or not a capability packet.
    Accepts both the GET and legacy SET type codes — they share a layout.
    """
    if len(buf) < CAPABILITY_PACKET_SIZE:
        return None
    fields = struct.unpack_from(_FMT_CAPABILITY, buf, 0)
    if fields[0] not in (PACKET_TYPE_GET_CAPABILITY, PACKET_TYPE_SET_CAPABILITY):
        return None
    return CapabilityResult(*fields)


def encode_force_sensor_config_packet(
    *,
    version_no: int = STREAM_MOTION_VERSION,
    do_reset: int = 0,
    fs_type: int = FORCE_SENSOR_TYPE_EMBEDDED,
) -> bytes:
    """Encode a ForceSensorConfigPacket (type 205).

    Sent by the host on v >= 4 to tell the controller which force sensor to
    stream. ``do_reset=1`` re-zeros (tares) the sensor; ``do_reset=0`` just
    selects ``fs_type``. Sending this on v < 4 raises controller alarm
    HOST-380 — callers MUST gate on the negotiated protocol version.
    """
    return struct.pack(
        _FMT_FORCE_SENSOR_CONFIG,
        PACKET_TYPE_FORCE_SENSOR_CONFIG,
        int(version_no) & 0xFFFF_FFFF,
        int(do_reset) & 0xFFFF_FFFF,
        int(fs_type) & 0xFFFF_FFFF,
    )


def encode_command_packet(
    sequence_no: int,
    command_pos_deg: Sequence[float],
    *,
    is_last_command: bool = False,
    do_motn_ctrl: bool = True,
    version_no: int = STREAM_MOTION_VERSION,
) -> bytes:
    """Encode a Stream Motion CommandPacket (type 201).

    Parameters
    ----------
    sequence_no:
        Monotonically increasing per-TX counter (uint32); the controller
        echoes it in the next status packet.
    command_pos_deg:
        Length-``MAX_AXES`` sequence of joint angles in **degrees**. Callers
        with fewer DOF (CRX has 6) zero-pad the trailing entries.
    is_last_command:
        Set on the final command of a stream so the controller knows the
        upcoming StopPacket is intentional.
    do_motn_ctrl:
        ``True`` (default) → command joints; ``False`` → I/O-only packet.

    SAFETY: the uint16 dataStyle selector is written ``COMMAND_DATA_STYLE``
    (= 0xFFFF), imported from :mod:`airo_fanuc.controller_facts`. Writing 0
    makes the controller interpret the 6 joint-angle values as Cartesian
    XYZWPR and slew toward the implied absolute pose (a real E-stop
    save). This value is NEVER hardcoded here.
    """
    if len(command_pos_deg) != MAX_AXES:
        raise ValueError(f"command_pos_deg must have length {MAX_AXES} (got {len(command_pos_deg)})")
    if not (0 <= sequence_no <= 0xFFFF_FFFF):
        raise ValueError(f"sequence_no out of uint32 range: {sequence_no}")

    # I/O bytes are zero-filled by default — we don't drive GPIO over
    # Stream Motion at the moment.
    io = (0,) * MAX_IO_BYTES
    return struct.pack(
        _FMT_COMMAND,
        PACKET_TYPE_COMMAND,
        int(version_no),
        sequence_no & 0xFFFF_FFFF,
        1 if is_last_command else 0,
        1 if do_motn_ctrl else 0,
        COMMAND_DATA_STYLE,  # dataStyle = joint mode (0xFFFF); imported, never hardcoded
        *(float(v) for v in command_pos_deg),
        *io,
    )


# ---------------------------------------------------------------------------
# Stream Motion decoder.
# ---------------------------------------------------------------------------


def decode_status_packet(buf: bytes) -> RobotStatusPacket | None:
    """Decode a Stream Motion status packet (type 204 or legacy V3 type 202).

    Returns ``None`` if ``buf`` is too short or the packet type is not one we
    understand. Deliberately swallows malformed packets — UDP can deliver
    garbage and the RX thread should keep running.
    """
    if len(buf) < _STATUS_HEADER_SIZE:
        return None

    (packet_type,) = struct.unpack_from(">I", buf, 0)
    if packet_type not in (PACKET_TYPE_STATUS_204, PACKET_TYPE_STATUS_V3):
        return None

    expected_size = STATUS_204_PACKET_SIZE if packet_type == PACKET_TYPE_STATUS_204 else STATUS_V3_PACKET_SIZE
    if len(buf) < expected_size:
        return None

    header = struct.unpack_from(_FMT_STATUS_HEADER, buf, 0)
    (
        _packet_type,
        version_no,
        sequence_no,
        status,
        robot_status,
        contact_stop_status_raw,
        _unused,
        time_stamp,
    ) = header[:8]

    pos = list(header[8 : 8 + MAX_AXES])
    joint = list(header[8 + MAX_AXES : 8 + 2 * MAX_AXES])
    current = list(header[8 + 2 * MAX_AXES : 8 + 3 * MAX_AXES])
    safety_scale = float(header[8 + 3 * MAX_AXES])

    force_x: float | None = None
    force_y: float | None = None
    force_z: float | None = None
    moment_x: float | None = None
    moment_y: float | None = None
    moment_z: float | None = None
    fs_type: int | None = None
    if packet_type == PACKET_TYPE_STATUS_204:
        tail = struct.unpack_from(_FMT_STATUS_204_TAIL, buf, _STATUS_HEADER_SIZE)
        force_x, force_y, force_z, moment_x, moment_y, moment_z = (float(v) for v in tail[:6])
        fs_type = int(tail[6])
        # io bytes (tail[7:]) are intentionally dropped — no GPIO surface yet.

    return RobotStatusPacket(
        packet_type=int(packet_type),
        version_no=int(version_no),
        sequence_no=int(sequence_no),
        status=int(status),
        robot_status=int(robot_status),
        contact_stop_status=ContactStopStatus.from_wire(int(contact_stop_status_raw)),
        time_stamp=int(time_stamp),
        position_deg=[float(v) for v in pos],
        joint_angle_deg=[float(v) for v in joint],
        current=[float(v) for v in current],
        safety_scale=safety_scale,
        force_x=force_x,
        force_y=force_y,
        force_z=force_z,
        moment_x=moment_x,
        moment_y=moment_y,
        moment_z=moment_z,
        fs_type=fs_type,
    )


# ---------------------------------------------------------------------------
# Status-packet builders (used to synthesise hand-packed golden buffers and by
# the round-trip smoke test — the codec itself only *decodes* status packets).
# ---------------------------------------------------------------------------


def build_status_204_packet(
    *,
    version_no: int = STREAM_MOTION_VERSION,
    sequence_no: int = 0,
    status: int = 0,
    robot_status: int = 0,
    contact_stop_status: int = ContactStopStatus.NONE,
    time_stamp: int = 0,
    position_deg: Sequence[float],
    joint_angle_deg: Sequence[float],
    current: Sequence[float],
    safety_scale: float,
    force_x: float,
    force_y: float,
    force_z: float,
    moment_x: float,
    moment_y: float,
    moment_z: float,
    fs_type: int,
) -> bytes:
    """Hand-pack a type-204 status packet (BE), zero-filled io_status."""
    for name, arr in (
        ("position_deg", position_deg),
        ("joint_angle_deg", joint_angle_deg),
        ("current", current),
    ):
        if len(arr) != MAX_AXES:
            raise ValueError(f"{name} must have length {MAX_AXES} (got {len(arr)})")
    header = struct.pack(
        _FMT_STATUS_HEADER,
        PACKET_TYPE_STATUS_204,
        int(version_no),
        int(sequence_no),
        int(status),
        int(robot_status),
        int(contact_stop_status),
        0,  # unused
        int(time_stamp),
        *(float(v) for v in position_deg),
        *(float(v) for v in joint_angle_deg),
        *(float(v) for v in current),
        float(safety_scale),
    )
    tail = struct.pack(
        _FMT_STATUS_204_TAIL,
        float(force_x),
        float(force_y),
        float(force_z),
        float(moment_x),
        float(moment_y),
        float(moment_z),
        int(fs_type) & 0xFFFF_FFFF,
        *((0,) * MAX_IO_BYTES),
    )
    return header + tail


def build_status_v3_packet(
    *,
    version_no: int = STREAM_MOTION_VERSION,
    sequence_no: int = 0,
    status: int = 0,
    robot_status: int = 0,
    contact_stop_status: int = ContactStopStatus.NONE,
    time_stamp: int = 0,
    position_deg: Sequence[float],
    joint_angle_deg: Sequence[float],
    current: Sequence[float],
    safety_scale: float,
) -> bytes:
    """Hand-pack a legacy V3 type-202 status packet (BE), zero-filled io_status."""
    for name, arr in (
        ("position_deg", position_deg),
        ("joint_angle_deg", joint_angle_deg),
        ("current", current),
    ):
        if len(arr) != MAX_AXES:
            raise ValueError(f"{name} must have length {MAX_AXES} (got {len(arr)})")
    header = struct.pack(
        _FMT_STATUS_HEADER,
        PACKET_TYPE_STATUS_V3,
        int(version_no),
        int(sequence_no),
        int(status),
        int(robot_status),
        int(contact_stop_status),
        0,  # unused
        int(time_stamp),
        *(float(v) for v in position_deg),
        *(float(v) for v in joint_angle_deg),
        *(float(v) for v in current),
        float(safety_scale),
    )
    tail = struct.pack(_FMT_STATUS_V3_TAIL, *((0,) * MAX_IO_BYTES))
    return header + tail


def encode_capability_result_packet(
    *,
    packet_type: int = PACKET_TYPE_GET_CAPABILITY,
    version_no: int = STREAM_MOTION_VERSION,
    id: int = 1,
    sampling_rate_ms: int = 8,
    start_move: int = 0,
    available_version: int = STREAM_MOTION_VERSION,
    rob_status_use_tcp: int = 0,
) -> bytes:
    """Encode a ControllerCapability *result* buffer (for decode-side goldens)."""
    return struct.pack(
        _FMT_CAPABILITY,
        int(packet_type),
        int(version_no),
        int(id),
        int(sampling_rate_ms),
        int(start_move),
        int(available_version),
        int(rob_status_use_tcp),
    )


# ---------------------------------------------------------------------------
# RMI JSON request serializers.
#
# Each builder returns the request dict for one RMI command; ``rmi_serialize`` /
# ``rmi_wire_bytes`` turn it into the exact on-wire byte string
# (``json.dumps(req) + "\r\n"``, ASCII).
#
# **Field order is the contract.** ``json.dumps`` emits keys in dict insertion
# order, and the ``tests/goldens/rmi/*.json`` fixtures pin the resulting wire
# strings byte for byte — so reordering the keys in any builder below, or adding
# one, fails a golden. Build every request through these functions rather than
# hand-assembling a dict, and the wire form stays pinned.
# ---------------------------------------------------------------------------

RMI_LINE_TERMINATOR: str = "\r\n"


def rmi_serialize(request: dict[str, Any]) -> str:
    """Serialise an RMI request dict to its exact on-wire string form.

    ``json.dumps(request)`` with the stdlib default separators (``", "`` /
    ``": "``, insertion order preserved) plus the ``\\r\\n`` terminator the
    controller's line-oriented parser expects.
    """
    return json.dumps(request) + RMI_LINE_TERMINATOR


def rmi_wire_bytes(request: dict[str, Any]) -> bytes:
    """Exact wire bytes: :func:`rmi_serialize` ASCII-encoded (RMI JSON is ASCII)."""
    return rmi_serialize(request).encode("ascii")


def rmi_connect_stmo_request() -> dict[str, Any]:
    """``FRC_Connect_STMO``: a lone ``Communication`` key, no payload fields.

    The bootstrap connect. Its reply carries the ``PortNumber`` the client must
    reconnect to for every subsequent request.
    """
    return {"Communication": "FRC_Connect_STMO"}


def rmi_disconnect_request() -> dict[str, Any]:
    """``FRC_Disconnect``: a lone ``Communication`` key, no payload fields."""
    return {"Communication": "FRC_Disconnect"}


def rmi_command_request(command: str) -> dict[str, Any]:
    """A bare ``{"Command": <command>}`` request — one key, no payload fields.

    Covers ``FRC_Initialize``, ``FRC_Reset``, ``FRC_GetStatus``,
    ``FRC_GetExtStatus``, ``FRC_Abort`` — every RMI command whose request
    carries no extra fields.
    """
    return {"Command": command}


def rmi_read_register_request(register_number: int) -> dict[str, Any]:
    """``FRC_ReadRegister``: ``Command``, then ``RegisterNumber``."""
    return {"Command": "FRC_ReadRegister", "RegisterNumber": int(register_number)}


def rmi_write_register_request(register_number: int, value: float | int | bool) -> dict[str, Any]:
    """``FRC_WriteRegister``: ``Command``, ``RegisterNumber``, ``RegisterValue``, ``DataType``.

    ``DataType`` is derived from the Python type: ``bool`` / ``int`` → lowercase
    ``"integer"``, ``float`` → lowercase ``"float"``. **Lowercase DataType is
    a pinned invariant** — capitalised "Integer"/"Float" is silently coerced
    to Integer by the controller (FANUC reference ``rmi.cpp:462``), so a float
    written with "Float" would land truncated.
    ``bool`` is handled before ``int`` because ``bool`` subclasses ``int``.
    """
    if isinstance(value, bool) or isinstance(value, int):
        data_type = "integer"
        register_value: int | float = int(value)
    elif isinstance(value, float):
        data_type = "float"
        register_value = float(value)
    else:
        raise TypeError(f"write_register: value must be int or float, got {type(value).__name__}")
    return {
        "Command": "FRC_WriteRegister",
        "RegisterNumber": int(register_number),
        "RegisterValue": register_value,
        "DataType": data_type,
    }


def rmi_read_error_request(count: int = 1) -> dict[str, Any]:
    """``FRC_ReadError``: ``Command``, then ``Count`` (1..5)."""
    if not 1 <= int(count) <= 5:
        raise ValueError(f"read_error: count must be 1..5 (got {count})")
    return {"Command": "FRC_ReadError", "Count": int(count)}


def rmi_read_joint_angles_request() -> dict[str, Any]:
    """``FRC_ReadJointAngles`` (RMI §2.3.15): ``Command`` only, no payload fields.

    Reads the controller's current joint angles. The vendored rmi lib
    (``rmi/packets.hpp`` ``ReadJointAnglesPacket::Request``) carries only the
    ``Command`` plus an *optional* ``Group``; a single-group arm omits it (the
    C++ default), so the request is the bare ``{"Command": "FRC_ReadJointAngles"}``.

    The reply's ``JointAngle`` block is UNCONVERTED — RMI reports J3 one J2 below
    the Stream Motion value (``controller_facts.MEASURED_FACTS
    .rmi_to_stream_j3_plus_j2_measured``), so joints read this way are tagged
    :data:`~airo_fanuc.receive_interface.SOURCE_RMI_UNCONVERTED` and hard-rejected
    for calibration until the ``J3 += J2`` conversion is verified at a second J2.
    """
    return {"Command": "FRC_ReadJointAngles"}


def rmi_read_cartesian_position_request() -> dict[str, Any]:
    """``FRC_ReadCartesianPosition`` (RMI §2.3.14): ``Command`` only, no payload.

    Matches the vendored ``rmi/packets.hpp``
    ``GetCartesianPositionPacket::Request``: ``Command`` plus an *optional*
    ``Group`` that a single-group arm omits (the C++ default).

    The reply pairs a ``Position`` block (X/Y/Z mm, W/P/R deg, + 3 ext axes) with a
    ``Configuration`` block naming ``UFrameNumber`` / ``UToolNumber``. That pairing
    is the point: the Stream Motion status packet carries a Cartesian pose with no
    frame tag, so this is the only read that says which frame a pose is in.
    """
    return {"Command": "FRC_ReadCartesianPosition"}


def rmi_continue_request() -> dict[str, Any]:
    """``FRC_Continue`` (RMI §2.3.4): ``Command`` only, no payload fields.

    Resumes a paused Remote-Motion TP program. Matches the vendored rmi lib
    (``rmi/packets.hpp`` ``ContinuePacket::Request``): a bare
    ``{"Command": "FRC_Continue"}``. ErrorID ``2556938`` ("TP Program is Not
    Paused.") is tolerated as a no-op by the client — the program already
    running is the desired end state.
    """
    return {"Command": "FRC_Continue"}


def rmi_call_request(sequence_id: int, program_name: str) -> dict[str, Any]:
    """``FRC_Call`` instruction: ``Instruction``, ``SequenceID``, ``ProgramName``.

    Note the top-level key is ``Instruction`` (not ``Command``) — that is also
    the field the controller echoes on the ack. The ``SequenceID`` must be
    re-anchored to the controller's ``NextSequenceID`` after every
    Initialize/Reset, or the controller silently drops the call as a duplicate.
    """
    return {
        "Instruction": "FRC_Call",
        "SequenceID": int(sequence_id),
        "ProgramName": str(program_name),
    }


# ---------------------------------------------------------------------------
# Round-trip smoke test (byte-exact self-check).
#
# Run via ``python -m airo_fanuc.testing.wire``. A typo in a struct format
# string trips here, not on the wire against the controller.
# ---------------------------------------------------------------------------


def _build_synthetic_status_204() -> bytes:
    """Hand-pack a type-204 status packet for round-trip testing."""
    return build_status_204_packet(
        version_no=STREAM_MOTION_VERSION,
        sequence_no=12345,
        status=0x01,  # motion_possible bit set
        robot_status=0x02,  # tp_enabled bit set
        contact_stop_status=int(ContactStopStatus.SAFE),
        time_stamp=987654,
        position_deg=[10.0 + i for i in range(MAX_AXES)],
        joint_angle_deg=[20.0 + i for i in range(MAX_AXES)],
        current=[30.0 + i for i in range(MAX_AXES)],
        safety_scale=0.875,  # exactly representable in float32
        force_x=11.0,
        force_y=12.0,
        force_z=13.0,
        moment_x=21.0,
        moment_y=22.0,
        moment_z=23.0,
        fs_type=FORCE_SENSOR_TYPE_EXTERNAL,
    )


def _build_synthetic_status_v3() -> bytes:
    """Hand-pack a legacy V3 type-202 status packet for round-trip testing."""
    return build_status_v3_packet(
        version_no=STREAM_MOTION_VERSION,
        sequence_no=7777,
        status=0x05,  # bit 0 = motion_possible
        robot_status=0x07,  # in_error | tp_enabled | e_stopped
        contact_stop_status=int(ContactStopStatus.STOP),
        time_stamp=424242,
        position_deg=[40.0 + i for i in range(MAX_AXES)],
        joint_angle_deg=[50.0 + i for i in range(MAX_AXES)],
        current=[60.0 + i for i in range(MAX_AXES)],
        safety_scale=1.0,
    )


def _smoke_test() -> None:
    """Run encode → decode → assert round trips for every packet type."""

    # --- StartPacket ------------------------------------------------------
    start = encode_start_packet()
    assert len(start) == START_PACKET_SIZE, len(start)
    pt, ver = struct.unpack(_FMT_START_OR_STOP, start)
    assert pt == PACKET_TYPE_START
    assert ver == STREAM_MOTION_VERSION
    print(f"[ok] StartPacket ({len(start)} B): type={pt}, ver={ver}")

    # --- StopPacket -------------------------------------------------------
    stop = encode_stop_packet()
    assert len(stop) == STOP_PACKET_SIZE, len(stop)
    pt, ver = struct.unpack(_FMT_START_OR_STOP, stop)
    assert pt == PACKET_TYPE_STOP
    assert ver == STREAM_MOTION_VERSION
    print(f"[ok] StopPacket ({len(stop)} B): type={pt}, ver={ver}")

    # --- GetCapability ----------------------------------------------------
    cap_req = encode_get_capability_packet()
    assert len(cap_req) == CAPABILITY_PACKET_SIZE, len(cap_req)
    cap_res = encode_capability_result_packet(sampling_rate_ms=8, available_version=3)
    decoded_cap = decode_capability_result_packet(cap_res)
    assert decoded_cap is not None
    assert decoded_cap.sampling_rate_ms == 8
    assert decoded_cap.available_version == 3
    print(f"[ok] ControllerCapability ({len(cap_req)} B) get/set round-trip")

    # --- ForceSensorConfig ------------------------------------------------
    fsc = encode_force_sensor_config_packet(do_reset=1, fs_type=FORCE_SENSOR_TYPE_EMBEDDED)
    assert len(fsc) == FORCE_SENSOR_CONFIG_PACKET_SIZE, len(fsc)
    pt, ver, do_reset, fs_type = struct.unpack(_FMT_FORCE_SENSOR_CONFIG, fsc)
    assert pt == PACKET_TYPE_FORCE_SENSOR_CONFIG
    assert do_reset == 1 and fs_type == FORCE_SENSOR_TYPE_EMBEDDED
    print(f"[ok] ForceSensorConfigPacket ({len(fsc)} B): do_reset={do_reset}, fs_type={fs_type}")

    # --- CommandPacket ----------------------------------------------------
    q_deg = [float(i) - 4.0 for i in range(MAX_AXES)]  # -4..+4 deg
    cmd = encode_command_packet(
        sequence_no=42,
        command_pos_deg=q_deg,
        is_last_command=False,
        do_motn_ctrl=True,
    )
    assert len(cmd) == COMMAND_PACKET_SIZE, len(cmd)
    fields = struct.unpack(_FMT_COMMAND, cmd)
    assert fields[0] == PACKET_TYPE_COMMAND
    assert fields[1] == STREAM_MOTION_VERSION
    assert fields[2] == 42
    assert fields[3] == 0  # is_last_command
    assert fields[4] == 1  # do_motn_ctrl
    assert fields[5] == 0xFFFF == COMMAND_DATA_STYLE  # dataStyle = joint mode
    # Verify the dataStyle bytes at the exact wire offset (0xFF 0xFF, BE).
    assert cmd[COMMAND_DATA_STYLE_OFFSET : COMMAND_DATA_STYLE_OFFSET + 2] == b"\xff\xff"
    decoded_pos = list(fields[6 : 6 + MAX_AXES])
    assert decoded_pos == q_deg, (decoded_pos, q_deg)
    decoded_io = list(fields[6 + MAX_AXES :])
    assert all(b == 0 for b in decoded_io)
    print(f"[ok] CommandPacket ({len(cmd)} B): seq={fields[2]}, q_deg[0]={decoded_pos[0]}")

    # Flag round-trips.
    assert struct.unpack(_FMT_COMMAND, encode_command_packet(1, q_deg, is_last_command=True))[3] == 1
    assert struct.unpack(_FMT_COMMAND, encode_command_packet(2, q_deg, do_motn_ctrl=False))[4] == 0

    # Length validation.
    try:
        encode_command_packet(0, [0.0] * (MAX_AXES - 1))
    except ValueError as exc:
        print(f"[ok] CommandPacket length validation: {exc}")
    else:
        raise AssertionError("expected ValueError for short command_pos_deg")

    # --- RobotStatusPacket type 204 --------------------------------------
    raw_204 = _build_synthetic_status_204()
    assert len(raw_204) == STATUS_204_PACKET_SIZE, len(raw_204)
    pkt = decode_status_packet(raw_204)
    assert pkt is not None
    assert pkt.packet_type == PACKET_TYPE_STATUS_204
    assert pkt.sequence_no == 12345
    assert pkt.time_stamp == 987654
    assert pkt.contact_stop_status is ContactStopStatus.SAFE
    assert pkt.motion_possible is True
    assert pkt.e_stopped is False
    assert pkt.tp_enabled is True
    assert pkt.in_error is False
    assert pkt.position_deg == [10.0 + i for i in range(MAX_AXES)]
    assert pkt.joint_angle_deg == [20.0 + i for i in range(MAX_AXES)]
    assert pkt.current == [30.0 + i for i in range(MAX_AXES)]
    assert pkt.safety_scale == 0.875
    assert pkt.force_x == 11.0 and pkt.force_y == 12.0 and pkt.force_z == 13.0
    assert pkt.moment_x == 21.0 and pkt.moment_y == 22.0 and pkt.moment_z == 23.0
    assert pkt.fs_type == 2
    print(
        f"[ok] RobotStatusPacket-204 ({len(raw_204)} B): seq={pkt.sequence_no}, "
        f"motion_possible={pkt.motion_possible}, force_x={pkt.force_x}"
    )

    # --- RobotStatusPacket type 202 (V3 legacy) --------------------------
    raw_v3 = _build_synthetic_status_v3()
    assert len(raw_v3) == STATUS_V3_PACKET_SIZE, len(raw_v3)
    pkt_v3 = decode_status_packet(raw_v3)
    assert pkt_v3 is not None
    assert pkt_v3.packet_type == PACKET_TYPE_STATUS_V3
    assert pkt_v3.sequence_no == 7777
    assert pkt_v3.time_stamp == 424242
    assert pkt_v3.contact_stop_status is ContactStopStatus.STOP
    assert pkt_v3.motion_possible is True
    assert pkt_v3.in_error is True
    assert pkt_v3.tp_enabled is True
    assert pkt_v3.e_stopped is True
    assert pkt_v3.safety_scale == 1.0
    assert pkt_v3.force_x is None
    assert pkt_v3.fs_type is None
    print(f"[ok] RobotStatusPacket-202 ({len(raw_v3)} B): seq={pkt_v3.sequence_no}, force_x={pkt_v3.force_x}")

    # --- Decoder rejects garbage / wrong types ---------------------------
    assert decode_status_packet(b"") is None
    assert decode_status_packet(b"\x00" * 16) is None  # type=0, way too small
    bad_type = struct.pack(">I", 999) + b"\x00" * (STATUS_204_PACKET_SIZE - 4)
    assert decode_status_packet(bad_type) is None
    assert decode_status_packet(raw_204[:-1]) is None  # truncated
    print("[ok] decode_status_packet rejects garbage / truncated / wrong type")

    # --- RMI JSON serialisation ------------------------------------------
    assert rmi_serialize(rmi_command_request("FRC_Reset")) == '{"Command": "FRC_Reset"}\r\n'
    wr = rmi_write_register_request(2, 1)
    assert wr["DataType"] == "integer"
    assert rmi_write_register_request(1, 1.5)["DataType"] == "float"
    assert rmi_write_register_request(3, True)["DataType"] == "integer"  # bool → integer
    call = rmi_call_request(7, "STREAM_MOTN")
    assert list(call.keys()) == ["Instruction", "SequenceID", "ProgramName"]
    assert rmi_serialize(rmi_read_joint_angles_request()) == '{"Command": "FRC_ReadJointAngles"}\r\n'
    assert rmi_serialize(rmi_continue_request()) == '{"Command": "FRC_Continue"}\r\n'
    print("[ok] RMI JSON serialisers (lowercase DataType, key order)")

    print("\nAll round-trip checks passed.")


if __name__ == "__main__":
    _smoke_test()
