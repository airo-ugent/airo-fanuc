# SPDX-License-Identifier: Apache-2.0
"""Integration tests for :class:`airo_fanuc.rmi_client.RmiClient`.

Drives the RMI client against a *real socket* served by
:class:`~airo_fanuc.testing.FakeCRXController`'s RMI emulator. The central
structural property under test is the split between the **commands-only
session** any caller may use (status polls, register reads/writes) and the
**supervisor-only session lifecycle** (``FRC_Initialize`` / ``FRC_Abort``), which
must never be reached from the transport's own recovery paths. Covered:

* commands-only session (Connect_STMO → redirect hop → GetStatus / registers
  work pre-Initialize, and no FRC_Initialize is ever sent);
* emitted request JSON byte-matches the committed RMI wire goldens;
* :class:`RmiError` carries the correct ErrorID + decoded text;
* the Init recovery ladder (pass 1 GetStatus→Reset→GetStatus→Init; pass 2
  Abort→Reset→GetStatus→Init on 2556943) + SequenceID reseed, and a stale
  SequenceID FRC_Call dropped by the fake then fixed by the reseed;
* async FRC_SystemFault push surfaced via ``poll_async_packets``;
* transport auto-reopen re-establishes the socket WITHOUT Initialize, and a
  persistent failure raises :class:`RmiSessionDown`;
* single-session: a second connect gets 2556954 (RmiError) with no reopen storm;
* :meth:`abort` is supervisor-only (never on the auto-reopen path);
* the GRIPDISP register happy path (R[3]→R[2]→R[1]=1, poll R[1] until 0).
"""

from __future__ import annotations

import json
import socket
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

from airo_fanuc.exceptions import FanucConnectionError, RmiError, RmiSessionDown
from airo_fanuc.rmi_client import RmiClient
from airo_fanuc.testing import FakeCRXConfig, FakeCRXController
from airo_fanuc.testing.fake_crx_rmi import (
    ERR_ALREADY_CONNECTED,
    ERR_INVALID_CONTROLLER_STATE,
)

_GOLDENS_DIR = Path(__file__).resolve().parent / "goldens" / "rmi"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _client(c: FakeCRXController, **kwargs: Any) -> RmiClient:
    """An RmiClient pointed at the fake controller's RMI bootstrap port."""
    return RmiClient(c.sm.host, c.rmi_port, **kwargs)


def _golden_wire(name: str) -> bytes:
    """The exact on-wire bytes pinned by the RMI wire golden ``name``."""
    data = json.loads((_GOLDENS_DIR / f"{name}.json").read_text(encoding="utf-8"))
    return data["wire"].encode("ascii")


def _spy_dispatch(c: FakeCRXController) -> list[str | None]:
    """Record the identifier of every redirect-session command the fake dispatches.

    Connect_STMO / Disconnect on the bootstrap port go through a different
    handler; this only sees traffic on the live redirect session — exactly the
    commands the client issues after the port-hop.
    """
    seen: list[str | None] = []
    orig = c.rmi._dispatch

    def spy(req: dict[str, Any]) -> dict[str, Any] | None:
        seen.append(req.get("Command") or req.get("Instruction") or req.get("Communication"))
        return orig(req)

    c.rmi._dispatch = spy  # type: ignore[method-assign]
    return seen


def _spy_bootstrap(c: FakeCRXController) -> dict[str, int]:
    """Count Connect_STMO bootstrap handshakes (to detect a reopen storm)."""
    count = {"n": 0}
    orig = c.rmi._handle_bootstrap

    def spy(conn: socket.socket, req: dict[str, Any]) -> None:
        count["n"] += 1
        orig(conn, req)

    c.rmi._handle_bootstrap = spy  # type: ignore[method-assign]
    return count


def _wait_until(pred: Callable[[], bool], timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


def _drain_async_until(
    rmi: RmiClient, pred: Callable[[dict[str, Any]], bool], timeout: float = 2.0
) -> list[dict[str, Any]]:
    """Poll ``poll_async_packets`` until one packet satisfies ``pred`` (or timeout)."""
    collected: list[dict[str, Any]] = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        collected.extend(rmi.poll_async_packets())
        if any(pred(p) for p in collected):
            return collected
        time.sleep(0.01)
    collected.extend(rmi.poll_async_packets())
    return collected


class _RecordingSocket:
    """Transparent socket proxy that records every ``sendall`` payload."""

    def __init__(self, sock: socket.socket, sink: list[bytes]) -> None:
        self._sock = sock
        self._sink = sink

    def sendall(self, data: bytes, *args: Any, **kwargs: Any) -> None:
        self._sink.append(bytes(data))
        self._sock.sendall(data, *args, **kwargs)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._sock, name)


class _RecordingRmiClient(RmiClient):
    """RmiClient whose sockets record every outgoing byte into ``sink``."""

    def __init__(self, *args: Any, sink: list[bytes], **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._sink = sink

    def _tcp_connect(self, ip: str, port: int) -> socket.socket:
        raw = super()._tcp_connect(ip, port)
        return _RecordingSocket(raw, self._sink)  # type: ignore[return-value]


# ---------------------------------------------------------------------------
# Session open + commands work pre-Initialize (the commands-only session)
# ---------------------------------------------------------------------------


def test_commands_only_session_works_before_initialize() -> None:
    with FakeCRXController() as c:
        seen = _spy_dispatch(c)
        with _client(c) as rmi:
            # GetStatus / registers work on the commands-only session — no
            # FRC_Initialize has been sent (that is initialize(), supervisor-only).
            status = rmi.get_status()
            assert status.servo_ready is True
            assert status.next_sequence_id == 1

            assert rmi.read_register(1) == 0.0
            rmi.write_register(2, 1)  # int -> DataType "integer"
            rmi.write_register(3, 1.5)  # float -> DataType "float"
            assert c.rmi.read_register_value(2) == 1
            assert c.rmi.read_register_value(3) == 1.5

            ext = rmi.get_extended_status()
            assert ext.drives_powered is True

            assert "FRC_Initialize" not in seen


# ---------------------------------------------------------------------------
# Emitted request JSON byte-matches the committed RMI wire goldens
# ---------------------------------------------------------------------------


def test_emitted_request_bytes_match_wire_goldens() -> None:
    sink: list[bytes] = []
    with FakeCRXController() as c:
        rmi = _RecordingRmiClient(c.sm.host, c.rmi_port, sink=sink)
        rmi.start()  # emits Connect_STMO
        try:
            rmi.get_status()  # emits FRC_GetStatus
            rmi.read_register(1)  # emits FRC_ReadRegister R[1]
            rmi.write_register(2, 1)  # emits FRC_WriteRegister int (lowercase "integer")
            rmi.reset()  # emits FRC_Reset
        finally:
            rmi.stop()  # emits FRC_Disconnect

    for name in (
        "connect_stmo",
        "get_status",
        "read_register_r1",
        "write_register_integer",
        "reset",
        "disconnect",
    ):
        assert _golden_wire(name) in sink, f"client did not emit golden {name!r} verbatim"


# ---------------------------------------------------------------------------
# RmiError carries the correct ErrorID + decoded text
# ---------------------------------------------------------------------------


def test_non_zero_error_id_raises_rmi_error_with_id_and_text() -> None:
    with FakeCRXController() as c:
        c.arm_rmi_error("FRC_GetStatus", ERR_INVALID_CONTROLLER_STATE, times=1)  # 2556943
        with _client(c) as rmi:
            with pytest.raises(RmiError) as ei:
                rmi.get_status()
    assert ei.value.error_id == 2556943
    assert ei.value.text == "Invalid Controller State."
    assert "2556943" in str(ei.value)


# ---------------------------------------------------------------------------
# Init recovery ladder + SequenceID reseed (+ stale-drop then fix)
# ---------------------------------------------------------------------------


def test_initialize_ladder_pass1_and_seq_reseed_fixes_stale_call() -> None:
    # seq_seed=42 models a NextSequenceID persisted from a previous session, so a
    # client that fails to reseed sends a stale FRC_Call that the fake drops.
    with FakeCRXController(FakeCRXConfig(seq_seed=42)) as c:
        seen = _spy_dispatch(c)
        with _client(c) as rmi:
            # Before reseed the local seq id is 1 (< 42) -> the fake silently
            # drops the FRC_Call; STREAM_MOTN never launches.
            rmi.program_call("STREAM_MOTN")
            rmi.get_status()  # synchronous round-trip flushes the fire-and-forget
            assert c.state.stream_motn_launched is False

            # Pass-1 ladder: GetStatus -> Reset -> GetStatus -> Initialize, then
            # reseed from the controller's NextSequenceID (42).
            rmi.initialize()

            idx_init = seen.index("FRC_Initialize")
            assert seen.index("FRC_Reset") < idx_init
            assert seen[:idx_init].count("FRC_GetStatus") >= 2
            assert "FRC_Abort" not in seen[: idx_init + 1]  # pass 1 succeeded
            assert "FRC_GetStatus" in seen[idx_init + 1 :]  # reseed GetStatus after Init

            # The reseed re-anchored the local counter to the controller's 42.
            assert rmi._instruction_seq_id == 42

            # A correctly-reseeded FRC_Call now launches STREAM_MOTN.
            rmi.program_call("STREAM_MOTN")
            rmi.get_status()  # flush
            assert _wait_until(lambda: c.state.stream_motn_launched is True)
            assert c.rmi.next_sequence_id == 43


def test_initialize_ladder_escalates_to_abort_on_2556943() -> None:
    with FakeCRXController() as c:
        c.arm_rmi_error("FRC_Initialize", ERR_INVALID_CONTROLLER_STATE, times=1)
        seen = _spy_dispatch(c)
        with _client(c) as rmi:
            rmi.initialize()  # pass-1 Init hits 2556943 -> pass-2 Abort/Reset/GetStatus/Init

            first_init = seen.index("FRC_Initialize")
            abort_idx = seen.index("FRC_Abort")
            assert abort_idx > first_init  # Abort escalation only after the failed Init
            second_init = seen.index("FRC_Initialize", first_init + 1)
            assert second_init > abort_idx  # the recovered Initialize follows the Abort
            assert "FRC_GetStatus" in seen[second_init + 1 :]  # reseed after recovery


# ---------------------------------------------------------------------------
# Async FRC_SystemFault push
# ---------------------------------------------------------------------------


def test_async_system_fault_push_surfaces_via_poll() -> None:
    with FakeCRXController() as c:
        with _client(c) as rmi:
            rmi.get_status()  # bring the redirect session live
            assert c.push_system_fault(event_id=99) is True
            drained = _drain_async_until(rmi, lambda p: p.get("Communication") == "FRC_SystemFault")
    fault = [p for p in drained if p.get("Communication") == "FRC_SystemFault"]
    assert fault, "FRC_SystemFault push never surfaced in the async ring"
    assert fault[0]["EventID"] == 99


def test_rx_loop_survives_malformed_line_and_still_surfaces_push() -> None:
    """A malformed JSON line (json.JSONDecodeError) or a non-ASCII byte
    (UnicodeDecodeError) — both ``ValueError`` subclasses — must not escape
    ``_read_one_json``. Letting one out kills the sole daemon RX reader thread, and
    the failure is silent: async fault-push surfacing simply stops, and nothing
    reports it until some unrelated later request times out. The reader must skip
    the bad line, stay alive, and still surface a following ``FRC_SystemFault``
    push. Driven over a socketpair (no controller needed)."""
    reader_end, controller_end = socket.socketpair()
    rmi = RmiClient("127.0.0.1", 16001)  # never .start()ed — we drive the RX loop directly
    rmi._sock = reader_end  # noqa: SLF001 - install the reader socket in isolation
    rmi._ensure_rx_thread()  # noqa: SLF001
    try:
        rx = rmi._rx_thread  # noqa: SLF001
        assert rx is not None and rx.is_alive()

        # Malformed JSON, then a non-ASCII byte — either one alone kills an
        # unguarded reader thread.
        controller_end.sendall(b"{this is not valid json}\r\n")
        controller_end.sendall(b"\xff\xfe not ascii text\r\n")

        time.sleep(0.1)
        assert rx.is_alive(), "RX reader thread died on a malformed line"

        # A subsequent valid FRC_SystemFault push IS still surfaced after the skips.
        controller_end.sendall(b'{"Communication": "FRC_SystemFault", "ErrorID": 999}\n')
        drained = _drain_async_until(rmi, lambda p: p.get("Communication") == "FRC_SystemFault")
        faults = [p for p in drained if p.get("Communication") == "FRC_SystemFault"]
        assert faults, "FRC_SystemFault push never surfaced after the malformed line"
        assert faults[0]["ErrorID"] == 999
        assert rx.is_alive()
    finally:
        rmi._stop_evt.set()  # noqa: SLF001
        reader_end.close()  # unblocks the RX recv() so the loop can observe the stop
        controller_end.close()
        rmi._join_rx_thread()  # noqa: SLF001


# ---------------------------------------------------------------------------
# Transport auto-reopen WITHOUT Initialize; persistent failure -> RmiSessionDown
# ---------------------------------------------------------------------------


def test_transport_auto_reopen_without_initialize() -> None:
    # single_session off so the reopen's Connect_STMO isn't racing the old
    # session teardown — this test is purely about NOT sending Initialize.
    with FakeCRXController(FakeCRXConfig(single_session=False)) as c:
        seen = _spy_dispatch(c)
        with _client(c) as rmi:
            rmi.get_status()  # live commands-only session

            # Simulate the controller dropping the connection.
            conn = c.rmi._redir_conn
            assert conn is not None
            conn.close()
            assert _wait_until(lambda: rmi._sock is None, timeout=2.0)

            # The next request transparently re-establishes the socket.
            reopened = rmi.get_status()
            assert reopened.servo_ready is True

            # Auto-reopen must NEVER issue Initialize or Abort: a socket blip in one
            # caller must not restart the session or abort another caller's motion.
            assert "FRC_Initialize" not in seen
            assert "FRC_Abort" not in seen
            # Only GetStatus / Connect_STMO crossed the wire during recovery —
            # the redirect session came back with no restart command.
            assert set(seen) <= {"FRC_GetStatus"}


def test_persistent_transport_failure_raises_rmi_session_down() -> None:
    c = FakeCRXController(FakeCRXConfig(single_session=False)).start()
    rmi = _client(c, request_timeout=0.3)
    rmi.start()
    try:
        rmi.get_status()
        c.close()  # tear the controller down entirely
        assert _wait_until(lambda: rmi._sock is None, timeout=2.0)
        with pytest.raises(RmiSessionDown):
            rmi.get_status()
    finally:
        rmi.stop()
        c.close()


def test_start_against_unreachable_controller_raises_connection_error() -> None:
    # Bind+immediately free a port so the connect is refused.
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    dead_port = s.getsockname()[1]
    s.close()
    rmi = RmiClient("127.0.0.1", dead_port, connect_timeout=0.5)
    with pytest.raises(FanucConnectionError):
        rmi.start()


# ---------------------------------------------------------------------------
# Single-session: a second connect gets 2556954, with no reopen storm
# ---------------------------------------------------------------------------


def test_single_session_second_connect_raises_2556954_no_storm() -> None:
    with FakeCRXController() as c:  # single_session=True by default
        with _client(c) as rmi_a:
            rmi_a.get_status()  # A holds the live redirect session

            count = _spy_bootstrap(c)  # count Connect_STMO from here on
            rmi_b = _client(c)
            try:
                with pytest.raises(RmiError) as ei:
                    rmi_b.start()
            finally:
                rmi_b.stop()

            assert ei.value.error_id == ERR_ALREADY_CONNECTED  # 2556954
            assert ei.value.text == "Robot is Already Connected."
            # No self-reopen storm: exactly one Connect_STMO attempt from B.
            assert count["n"] == 1
            # A's session is untouched.
            assert rmi_a.get_status().servo_ready is True


# ---------------------------------------------------------------------------
# abort() is supervisor-only (never on the auto-reopen path)
# ---------------------------------------------------------------------------


def test_abort_is_supervisor_only_not_on_reopen_path() -> None:
    with FakeCRXController(FakeCRXConfig(single_session=False)) as c:
        seen = _spy_dispatch(c)
        with _client(c) as rmi:
            rmi.get_status()

            # Drop + transparently reopen; the reopen must not issue Abort.
            conn = c.rmi._redir_conn
            assert conn is not None
            conn.close()
            assert _wait_until(lambda: rmi._sock is None, timeout=2.0)
            rmi.get_status()  # reopens
            assert "FRC_Abort" not in seen

            # An explicit supervisor abort() DOES reach the controller.
            rmi.abort()
            assert "FRC_Abort" in seen


# ---------------------------------------------------------------------------
# GRIPDISP register happy path
# ---------------------------------------------------------------------------


def test_gripdisp_register_happy_path() -> None:
    with FakeCRXController() as c:  # gripdisp_clear_after_reads=1, seq_seed=1
        with _client(c) as rmi:
            # Local seq id (1) matches the controller's NextSequenceID (1), so the
            # fire-and-forget FRC_Call(GRPRUN) is accepted and forks the dispatcher.
            rmi.program_call("GRPRUN")
            rmi.get_status()  # flush the fire-and-forget
            assert _wait_until(lambda: c.gripdisp_running is True)

            # Gripper poke: R[3]=modifier, R[2]=action, R[1]=1 trigger.
            rmi.write_register(3, 1)
            rmi.write_register(2, 1)  # open
            rmi.write_register(1, 1)  # trigger

            # Poll R[1] until the dispatcher clears it back to 0.
            cleared = False
            for _ in range(20):
                if int(rmi.read_register(1)) == 0:
                    cleared = True
                    break
            assert cleared
            assert c.last_gripper_command == {"action": 1, "modifier": 1}


# ---------------------------------------------------------------------------
# FRC_ReadCartesianPosition — the pose read that names its own frame
# ---------------------------------------------------------------------------


def test_read_cartesian_position_decodes_pose_and_active_frame() -> None:
    with FakeCRXController() as c:
        c.set_cartesian([1234.5, -678.25, 901.75, -179.5, 45.25, 90.125], utool=2, uframe=3)
        with _client(c) as rmi:
            pos = rmi.read_cartesian_position()

    # X/Y/Z in mm, W/P/R in degrees, extended axes appended (zero on a 6-DOF arm).
    assert pos.xyzwpr[:6] == pytest.approx([1234.5, -678.25, 901.75, -179.5, 45.25, 90.125])
    # The reason this read exists: the pose comes tagged with the frame it is in.
    assert (pos.uframe_number, pos.utool_number) == (3, 2)
    assert pos.turn == (0, 0, 0)


def test_read_cartesian_position_rejects_a_reply_missing_the_pose() -> None:
    # Getters may return None for "no data", but an RMI read either decodes or
    # raises — a half-populated Position block must never become a partial pose.
    with FakeCRXController() as c:
        orig = c.rmi._handle_read_cartesian_position

        def truncated(req: dict[str, Any]) -> dict[str, Any]:
            reply = orig(req)
            reply["Position"] = {"X": 1.0, "Y": 2.0, "Z": 3.0}  # no W/P/R
            return reply

        c.rmi._handle_read_cartesian_position = truncated  # type: ignore[method-assign]
        with _client(c) as rmi:
            with pytest.raises(RmiError, match="3 of the 6 required"):
                rmi.read_cartesian_position()
