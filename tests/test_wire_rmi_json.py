# SPDX-License-Identifier: Apache-2.0
"""L0 goldens for the RMI JSON request wire format (TCP 16001).

We are NOT building the RMI client here (PLAN §P4a). We pin the exact on-wire
JSON *serialization* of every request the ladders issue, so the future
``RmiClient`` can be validated against these goldens.

Each ``goldens/rmi/<name>.json`` fixture records:
  * ``builder`` + ``args`` — the ``airo_fanuc.testing.wire`` request builder and
    its inputs (provenance / how to reconstruct).
  * ``provenance`` — the dries ``rmi_client.py`` method it mirrors.
  * ``request`` — the request dict (insertion order == wire key order).
  * ``wire`` — the exact on-wire string: ``json.dumps(request) + "\\r\\n"``.

The verify path rebuilds the request from ``builder``+``args`` (NOT from the
stored dict), re-serializes, and asserts it equals the committed ``wire`` and
``request``. Regenerate with ``AIRO_FANUC_REGEN_GOLDENS=1 pytest ...``.

**Pinned invariant**: ``FRC_WriteRegister`` ``DataType`` is LOWERCASE
(``"integer"`` / ``"float"``). Capitalised values are silently coerced to
Integer by the controller (dries ``write_register`` / FANUC ``rmi.cpp:462``).
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
    "rmi_continue_request": wire.rmi_continue_request,
    "rmi_call_request": wire.rmi_call_request,
}

# name -> (builder_name, kwargs, provenance)
_RMI_GOLDENS: dict[str, tuple[str, dict[str, Any], str]] = {
    "connect_stmo": (
        "rmi_connect_stmo_request",
        {},
        "rmi_client._open_session_locked: send {'Communication': 'FRC_Connect_STMO'}",
    ),
    "disconnect": (
        "rmi_disconnect_request",
        {},
        "rmi_client.stop: best-effort {'Communication': 'FRC_Disconnect'}",
    ),
    "initialize": (
        "rmi_command_request",
        {"command": "FRC_Initialize"},
        "rmi_client._initialize_with_recovery_locked: {'Command': 'FRC_Initialize'}",
    ),
    "reset": (
        "rmi_command_request",
        {"command": "FRC_Reset"},
        "rmi_client.reset: {'Command': 'FRC_Reset'}",
    ),
    "get_status": (
        "rmi_command_request",
        {"command": "FRC_GetStatus"},
        "rmi_client.get_status / reseed_sequence_id_from_controller",
    ),
    "get_ext_status": (
        "rmi_command_request",
        {"command": "FRC_GetExtStatus"},
        "rmi_client.get_extended_status: {'Command': 'FRC_GetExtStatus'}",
    ),
    "abort": (
        "rmi_command_request",
        {"command": "FRC_Abort"},
        "rmi_client.abort / recovery pass-2: {'Command': 'FRC_Abort'}",
    ),
    "read_register_r1": (
        "rmi_read_register_request",
        {"register_number": 1},
        "rmi_client.read_register(1) — gripper R[1] poll",
    ),
    "write_register_integer": (
        "rmi_write_register_request",
        {"register_number": 2, "value": 1},
        "rmi_client.write_register(2, 1) — int -> DataType 'integer' (lowercase)",
    ),
    "write_register_float": (
        "rmi_write_register_request",
        {"register_number": 1, "value": 1.5},
        "rmi_client.write_register(1, 1.5) — float -> DataType 'float' (lowercase)",
    ),
    "read_error": (
        "rmi_read_error_request",
        {"count": 1},
        "rmi_client.read_error(1): {'Command': 'FRC_ReadError', 'Count': 1}",
    ),
    "read_joint_angles": (
        "rmi_read_joint_angles_request",
        {},
        "rmi_client.read_joint_angles — FRC_ReadJointAngles (RMI §2.3.15, ReadJointAnglesPacket)",
    ),
    "continue": (
        "rmi_continue_request",
        {},
        "rmi_client.program_continue — FRC_Continue (RMI §2.3.4, ContinuePacket)",
    ),
    "call_stream_motn": (
        "rmi_call_request",
        {"sequence_id": 7, "program_name": "STREAM_MOTN"},
        "rmi_client.program_call('STREAM_MOTN') — fire-and-forget FRC_Call",
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

    # Wire string is the pinned oracle.
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

    # bool is an int subclass -> integer (matches dries write_register).
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
