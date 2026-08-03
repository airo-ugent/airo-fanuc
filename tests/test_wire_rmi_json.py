# SPDX-License-Identifier: Apache-2.0
"""Codec goldens for the RMI JSON request wire format (TCP 16001).

These tests pin the exact on-wire JSON *serialization* of every request the
recovery/gripper ladders issue. They go through the builders in
``airo_fanuc.testing.wire`` rather than through :class:`RmiClient`, so the wire
format is nailed down independently of the client that sends it: a refactor
inside the client cannot silently move a byte without a golden failing here.

Each ``goldens/rmi/<name>.json`` fixture records:
  * ``builder`` + ``args`` — the ``airo_fanuc.testing.wire`` request builder and
    its inputs (everything needed to reconstruct the request).
  * ``provenance`` — a one-line description of what the request is: which
    ``FRC_`` command/instruction it carries and what it is used for.
  * ``request`` — the request dict (insertion order == wire key order).
  * ``wire`` — the exact on-wire string: ``json.dumps(request) + "\\r\\n"``.

The verify path rebuilds the request from ``builder``+``args`` (NOT from the
stored dict), re-serializes, and asserts it equals the committed ``wire`` and
``request``. Regenerate with ``AIRO_FANUC_REGEN_GOLDENS=1 pytest ...``.

**Pinned invariant**: ``FRC_WriteRegister`` ``DataType`` is LOWERCASE
(``"integer"`` / ``"float"``). The controller silently coerces a capitalised
value to Integer (FANUC ``rmi.cpp:462``), so ``"Float"`` would write a truncated
integer into the register with no error reported — hence the exact-case pin.
"""

from __future__ import annotations

import json
import os
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

from airo_fanuc.testing import wire

_GOLDENS_DIR = Path(__file__).resolve().parent / "goldens" / "rmi"
_REGEN = os.environ.get("AIRO_FANUC_REGEN_GOLDENS") == "1"

# ---------------------------------------------------------------------------
# Golden registry.  name -> (builder, kwargs, provenance).
#
# The builder + kwargs are the single source of truth; the .json fixture is
# derived from them (regen) and byte-compared against them (verify).
# ---------------------------------------------------------------------------

_BUILDERS: dict[str, Callable[..., dict[str, Any]]] = {
    "rmi_connect_stmo_request": wire.rmi_connect_stmo_request,
    "rmi_disconnect_request": wire.rmi_disconnect_request,
    "rmi_command_request": wire.rmi_command_request,
    "rmi_read_register_request": wire.rmi_read_register_request,
    "rmi_write_register_request": wire.rmi_write_register_request,
    "rmi_read_error_request": wire.rmi_read_error_request,
    "rmi_read_joint_angles_request": wire.rmi_read_joint_angles_request,
    "rmi_read_cartesian_position_request": wire.rmi_read_cartesian_position_request,
    "rmi_continue_request": wire.rmi_continue_request,
    "rmi_call_request": wire.rmi_call_request,
}

# name -> (builder_name, kwargs, provenance)
_RMI_GOLDENS: dict[str, tuple[str, dict[str, Any], str]] = {
    "connect_stmo": (
        "rmi_connect_stmo_request",
        {},
        "Session open — {'Communication': 'FRC_Connect_STMO'} on TCP 16001; the "
        "reply carries the port the rest of the session runs on.",
    ),
    "disconnect": (
        "rmi_disconnect_request",
        {},
        "Session close — {'Communication': 'FRC_Disconnect'}, sent best-effort at "
        "shutdown so the controller frees the RMI session slot.",
    ),
    "initialize": (
        "rmi_command_request",
        {"command": "FRC_Initialize"},
        "{'Command': 'FRC_Initialize'} — takes RMI control of the controller; "
        "required before any motion instruction is accepted.",
    ),
    "reset": (
        "rmi_command_request",
        {"command": "FRC_Reset"},
        "{'Command': 'FRC_Reset'} — clears the latched controller fault, the first "
        "rung of the recovery ladder.",
    ),
    "get_status": (
        "rmi_command_request",
        {"command": "FRC_GetStatus"},
        "{'Command': 'FRC_GetStatus'} — scheduler-relevant state: authoritative "
        "NextSequenceID, TP mode, program status, override percentage.",
    ),
    "get_ext_status": (
        "rmi_command_request",
        {"command": "FRC_GetExtStatus"},
        "{'Command': 'FRC_GetExtStatus'} — extended status: in_motion, "
        "drives_powered, overrides, and the FANUC alarm code.",
    ),
    "abort": (
        "rmi_command_request",
        {"command": "FRC_Abort"},
        "{'Command': 'FRC_Abort'} — aborts all running TP programs and flushes the "
        "controller-side instruction queue (the pendant's FCTN → ABORT ALL).",
    ),
    "read_register_r1": (
        "rmi_read_register_request",
        {"register_number": 1},
        "FRC_ReadRegister on R[1] — the gripper handshake poll.",
    ),
    "write_register_integer": (
        "rmi_write_register_request",
        {"register_number": 2, "value": 1},
        "FRC_WriteRegister R[2] = 1 — an int value carries DataType 'integer' "
        "(lowercase).",
    ),
    "write_register_float": (
        "rmi_write_register_request",
        {"register_number": 1, "value": 1.5},
        "FRC_WriteRegister R[1] = 1.5 — a float value carries DataType 'float' "
        "(lowercase).",
    ),
    "read_error": (
        "rmi_read_error_request",
        {"count": 1},
        "{'Command': 'FRC_ReadError', 'Count': 1} — the most recent alarm text, "
        "read on motion_possible→False to capture the real FANUC alarm.",
    ),
    "read_joint_angles": (
        "rmi_read_joint_angles_request",
        {},
        "FRC_ReadJointAngles (RMI §2.3.15, ReadJointAnglesPacket) — current joint "
        "angles over the RMI plane.",
    ),
    "read_cartesian_position": (
        "rmi_read_cartesian_position_request",
        {},
        "FRC_ReadCartesianPosition (RMI §2.3.14, GetCartesianPositionPacket) — the "
        "controller's own FK, tagged with the active UFRAME/UTOOL numbers that the "
        "Stream Motion pose block carries no equivalent of.",
    ),
    "continue": (
        "rmi_continue_request",
        {},
        "FRC_Continue (RMI §2.3.4, ContinuePacket) — resumes a paused "
        "Remote-Motion TP program.",
    ),
    "call_stream_motn": (
        "rmi_call_request",
        {"sequence_id": 7, "program_name": "STREAM_MOTN"},
        "FRC_Call('STREAM_MOTN') at SequenceID 7 — an Instruction (not a Command), "
        "fire-and-forget: no synchronous reply is read.",
    ),
}


def _build(builder_name: str, kwargs: dict[str, Any]) -> dict[str, Any]:
    return _BUILDERS[builder_name](**kwargs)


def _load_or_regen(name: str, builder_name: str, kwargs: dict[str, Any], provenance: str) -> dict[str, Any]:
    """Read a committed golden, or (in regen mode) build + write it."""
    path = _GOLDENS_DIR / f"{name}.json"
    if _REGEN:
        request = _build(builder_name, kwargs)
        golden = {
            "provenance": provenance,
            "builder": builder_name,
            "args": kwargs,
            "request": request,
            "wire": wire.rmi_serialize(request),
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(golden, indent=2) + "\n", encoding="utf-8")
        return golden
    assert path.is_file(), f"missing golden {path} — regenerate with AIRO_FANUC_REGEN_GOLDENS=1 pytest"
    return json.loads(path.read_text(encoding="utf-8"))


@pytest.mark.parametrize("name", sorted(_RMI_GOLDENS.keys()))
def test_rmi_golden_matches(name: str) -> None:
    """Rebuild request from builder+args, re-serialize, compare to committed golden."""
    builder_name, kwargs, provenance = _RMI_GOLDENS[name]
    golden = _load_or_regen(name, builder_name, kwargs, provenance)

    rebuilt = _build(builder_name, kwargs)
    wire_str = wire.rmi_serialize(rebuilt)

    # The committed wire string is the pinned golden.
    assert wire_str == golden["wire"], (
        f"{name}: serialization diverged from committed golden.\n"
        f"  got:      {wire_str!r}\n  expected: {golden['wire']!r}"
    )
    # Request dict matches, including key order (json preserves insertion order).
    assert rebuilt == golden["request"]
    assert list(rebuilt.keys()) == list(golden["request"].keys())
    # The committed builder/provenance metadata is self-consistent.
    assert golden["builder"] == builder_name


# ---------------------------------------------------------------------------
# Explicit invariant assertions (independent of the goldens).
# ---------------------------------------------------------------------------


def test_serialization_uses_stdlib_defaults_and_crlf() -> None:
    """json.dumps default separators (', ' / ': ') + trailing CRLF, ASCII."""
    s = wire.rmi_serialize({"Command": "FRC_Reset"})
    assert s == '{"Command": "FRC_Reset"}\r\n'
    assert s.endswith("\r\n")
    assert wire.rmi_wire_bytes({"Command": "FRC_Reset"}) == b'{"Command": "FRC_Reset"}\r\n'


def test_write_register_datatype_is_lowercase() -> None:
    """PINNED: DataType is lowercase for both int and float paths."""
    int_req = wire.rmi_write_register_request(2, 1)
    assert int_req["DataType"] == "integer"
    assert '"DataType": "integer"' in wire.rmi_serialize(int_req)

    float_req = wire.rmi_write_register_request(1, 1.5)
    assert float_req["DataType"] == "float"
    assert '"DataType": "float"' in wire.rmi_serialize(float_req)

    # bool is an int subclass, so True normalizes to DataType 'integer' with a
    # plain int 1 as the value — never a JSON `true`, which the controller rejects.
    bool_req = wire.rmi_write_register_request(3, True)
    assert bool_req["DataType"] == "integer"
    assert bool_req["RegisterValue"] == 1
    assert isinstance(bool_req["RegisterValue"], int) and not isinstance(bool_req["RegisterValue"], bool)


def test_write_register_key_order() -> None:
    """Command, RegisterNumber, RegisterValue, DataType — in that wire order."""
    req = wire.rmi_write_register_request(5, 2)
    assert list(req.keys()) == ["Command", "RegisterNumber", "RegisterValue", "DataType"]
    assert req["Command"] == "FRC_WriteRegister"
    assert req["RegisterNumber"] == 5
    assert req["RegisterValue"] == 2


def test_write_register_rejects_bad_type() -> None:
    with pytest.raises(TypeError):
        wire.rmi_write_register_request(1, "nope")  # type: ignore[arg-type]


def test_call_request_uses_instruction_key() -> None:
    """FRC_Call is an Instruction (not a Command) with SequenceID + ProgramName."""
    req = wire.rmi_call_request(7, "STREAM_MOTN")
    assert list(req.keys()) == ["Instruction", "SequenceID", "ProgramName"]
    assert req["Instruction"] == "FRC_Call"
    assert req["SequenceID"] == 7
    assert req["ProgramName"] == "STREAM_MOTN"
    assert "Command" not in req


def test_read_register_shape() -> None:
    req = wire.rmi_read_register_request(3)
    assert req == {"Command": "FRC_ReadRegister", "RegisterNumber": 3}
    assert list(req.keys()) == ["Command", "RegisterNumber"]


def test_read_error_count_bounds() -> None:
    assert wire.rmi_read_error_request(1) == {"Command": "FRC_ReadError", "Count": 1}
    assert wire.rmi_read_error_request(5)["Count"] == 5
    for bad in (0, 6, -1):
        with pytest.raises(ValueError):
            wire.rmi_read_error_request(bad)


def test_connect_and_disconnect_use_communication_key() -> None:
    assert wire.rmi_connect_stmo_request() == {"Communication": "FRC_Connect_STMO"}
    assert wire.rmi_disconnect_request() == {"Communication": "FRC_Disconnect"}
