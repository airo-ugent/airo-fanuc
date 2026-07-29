# SPDX-License-Identifier: Apache-2.0
"""L2 self-tests for the FakeCRXController RMI side.

Prove the RMI emulator is a faithful spec: bootstrap→redirect port-hop,
Initialize/Reset ladders with NextSequenceID reseed, stale-SequenceID drop,
single-session 2556954, scriptable error injection (2556943 / 2556938 /
SYST-348), the GRIPDISP register contract (R[1] auto-clear with correct R[3]
reading; direct-GRIPDISP wedge), and async FRC_SystemFault push.

The ``RmiProbe`` helper mirrors ``dries`` ``rmi_client``'s drain-by-identifier
behavior (push/orphan packets stashed, matched reply returned) so these tests
double as a check that the fake plays correctly against that client shape.
"""

from __future__ import annotations

import json
import socket

from airo_fanuc.testing import FakeCRXConfig, FakeCRXController
from airo_fanuc.testing.fake_crx_rmi import (
    ERR_ALREADY_CONNECTED,
    ERR_CANNOT_EXECUTE_TP_B,
    ERR_INVALID_CONTROLLER_STATE,
    ERR_TP_NOT_PAUSED,
)

_ECHO_KEYS = ("Command", "Communication", "Instruction")


def _ident(obj: dict) -> str | None:
    for k in _ECHO_KEYS:
        if k in obj:
            return obj[k]
    return None


def _send_line(sock: socket.socket, obj: dict) -> None:
    sock.sendall((json.dumps(obj) + "\r\n").encode("ascii"))


def _bootstrap_connect(host: str, port: int) -> dict:
    """One-shot FRC_Connect_STMO against the bootstrap port; return the reply."""
    s = socket.socket()
    s.settimeout(2.0)
    s.connect((host, port))
    try:
        _send_line(s, {"Communication": "FRC_Connect_STMO"})
        buf = b""
        while b"\n" not in buf:
            buf += s.recv(4096)
        return json.loads(buf.split(b"\n", 1)[0].rstrip(b"\r").decode())
    finally:
        s.close()


class RmiProbe:
    """Minimal RMI client: bootstrap→redirect hop + drain-by-identifier."""

    def __init__(self, host: str, boot_port: int) -> None:
        self.connect_reply = _bootstrap_connect(host, boot_port)
        self.sock = socket.socket()
        self.sock.settimeout(2.0)
        self.sock.connect((host, int(self.connect_reply["PortNumber"])))
        self._rxbuf = b""
        self.async_packets: list[dict] = []

    def request(self, obj: dict) -> dict:
        _send_line(self.sock, obj)
        want = _ident(obj)
        while True:
            pkt = self._recv_line()
            if _ident(pkt) == want:
                return pkt
            self.async_packets.append(pkt)

    def call(self, seq: int, program: str) -> None:
        """Fire-and-forget FRC_Call (no synchronous read, per dries)."""
        _send_line(self.sock, {"Instruction": "FRC_Call", "SequenceID": seq, "ProgramName": program})

    def read_raw(self, timeout: float = 1.0) -> dict:
        self.sock.settimeout(timeout)
        return self._recv_line()

    def get_status(self) -> dict:
        return self.request({"Command": "FRC_GetStatus"})

    def read_register(self, n: int) -> float:
        return self.request({"Command": "FRC_ReadRegister", "RegisterNumber": n})["RegisterValue"]

    def write_register(self, n: int, value: int) -> dict:
        return self.request(
            {
                "Command": "FRC_WriteRegister",
                "RegisterNumber": n,
                "RegisterValue": value,
                "DataType": "integer",
            }
        )

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def _recv_line(self) -> dict:
        while b"\n" not in self._rxbuf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise OSError("RMI probe: connection closed")
            self._rxbuf += chunk
        line, self._rxbuf = self._rxbuf.split(b"\n", 1)
        return json.loads(line.rstrip(b"\r").decode())


# ---------------------------------------------------------------------------
# Bootstrap / port-hop / single-session
# ---------------------------------------------------------------------------


def test_bootstrap_redirects_to_port_and_reports_version() -> None:
    with FakeCRXController() as c:
        reply = _bootstrap_connect(c.sm.host, c.rmi_port)
        assert reply["ErrorID"] == 0
        assert reply["PortNumber"] == c.rmi_redirect_port
        assert reply["MajorVersion"] == 9 and reply["MinorVersion"] == 40


def test_redirect_session_handles_get_status() -> None:
    with FakeCRXController() as c:
        p = RmiProbe(c.sm.host, c.rmi_port)
        st = p.get_status()
        assert st["ErrorID"] == 0
        assert st["ServoReady"] == 1
        assert st["NextSequenceID"] == 1
        p.close()


def test_single_session_second_connect_returns_2556954() -> None:
    with FakeCRXController() as c:
        p = RmiProbe(c.sm.host, c.rmi_port)
        p.get_status()  # ensure the redirect session is live
        reply = _bootstrap_connect(c.sm.host, c.rmi_port)
        assert reply["ErrorID"] == ERR_ALREADY_CONNECTED
        assert "PortNumber" not in reply
        p.close()


# ---------------------------------------------------------------------------
# NextSequenceID reseed + stale-drop
# ---------------------------------------------------------------------------


def test_ladder_reseeds_next_sequence_id_and_launches_stream_motn() -> None:
    # seq_seed models a NextSequenceID persisted from a previous session.
    with FakeCRXController(FakeCRXConfig(seq_seed=42)) as c:
        p = RmiProbe(c.sm.host, c.rmi_port)
        # Session-open ladder: GetStatus → Reset → GetStatus → Initialize.
        p.get_status()
        assert p.request({"Command": "FRC_Reset"})["ErrorID"] == 0
        assert p.request({"Command": "FRC_Initialize"})["ErrorID"] == 0
        # Reseed: the client re-anchors from the controller's NextSequenceID.
        next_seq = p.get_status()["NextSequenceID"]
        assert next_seq == 42
        # A correctly-reseeded FRC_Call launches STREAM_MOTN.
        p.call(next_seq, "STREAM_MOTN")
        p.get_status()  # drains the fire-and-forget ack orphan
        assert c.state.stream_motn_launched is True
        assert c.rmi.next_sequence_id == next_seq + 1
        p.close()


def test_stale_sequence_id_call_is_silently_dropped() -> None:
    with FakeCRXController(FakeCRXConfig(seq_seed=50)) as c:
        p = RmiProbe(c.sm.host, c.rmi_port)
        # A client that failed to reseed sends a stale (too-low) SequenceID.
        p.call(10, "STREAM_MOTN")
        p.get_status()  # no ack orphan for a dropped call
        assert c.state.stream_motn_launched is False
        assert not p.async_packets  # dropped call produced no ack
        p.close()


# ---------------------------------------------------------------------------
# Error injection ladders
# ---------------------------------------------------------------------------


def test_initialize_2556943_then_abort_reset_recovery() -> None:
    with FakeCRXController() as c:
        c.arm_rmi_error("FRC_Initialize", ERR_INVALID_CONTROLLER_STATE, times=1)
        p = RmiProbe(c.sm.host, c.rmi_port)
        # Pass 1 Initialize hits the stale-state error.
        assert p.request({"Command": "FRC_Initialize"})["ErrorID"] == ERR_INVALID_CONTROLLER_STATE
        # Pass 2 escalation: Abort → Reset → GetStatus → Initialize succeeds.
        assert p.request({"Command": "FRC_Abort"})["ErrorID"] == 0
        assert p.request({"Command": "FRC_Reset"})["ErrorID"] == 0
        p.get_status()
        assert p.request({"Command": "FRC_Initialize"})["ErrorID"] == 0
        p.close()


def test_continue_2556938_when_program_not_paused() -> None:
    with FakeCRXController() as c:
        c.arm_rmi_error("FRC_Continue", ERR_TP_NOT_PAUSED, times=1)
        p = RmiProbe(c.sm.host, c.rmi_port)
        assert p.request({"Command": "FRC_Continue"})["ErrorID"] == ERR_TP_NOT_PAUSED
        # Interim recovery tolerates it: the next Continue is clean.
        assert p.request({"Command": "FRC_Continue"})["ErrorID"] == 0
        p.close()


def test_syst_348_injection_blocks_initialize_and_surfaces_alarm() -> None:
    with FakeCRXController() as c:
        c.arm_syst_348(on="FRC_Initialize")
        p = RmiProbe(c.sm.host, c.rmi_port)
        assert p.request({"Command": "FRC_Initialize"})["ErrorID"] == ERR_CANNOT_EXECUTE_TP_B
        err = p.request({"Command": "FRC_ReadError", "Count": 5})
        assert err["ErrorID"] == 0
        assert any("SYST-348" in v for v in err.values() if isinstance(v, str))
        p.close()


# ---------------------------------------------------------------------------
# GRIPDISP register contract
# ---------------------------------------------------------------------------


def _run_gripdisp(c: FakeCRXController, p: RmiProbe, *, action: int, modifier: int) -> int:
    """Poke R[3]→R[2]→R[1]=1, then poll R[1] until it clears; return poll count."""
    p.write_register(3, modifier)
    p.write_register(2, action)
    p.write_register(1, 1)
    polls = 0
    while polls < 20:
        polls += 1
        if int(p.read_register(1)) == 0:
            return polls
    return polls


def test_gripdisp_open_clears_r1_with_correct_r3_reading() -> None:
    with FakeCRXController() as c:
        p = RmiProbe(c.sm.host, c.rmi_port)
        p.get_status()
        next_seq = p.get_status()["NextSequenceID"]
        p.call(next_seq, "GRPRUN")  # fork the dispatcher (never call GRIPDISP directly)
        p.get_status()  # drain ack
        assert c.gripdisp_running is True
        # Open, mid width (R[2]=1 open, R[3]=1).
        polls = _run_gripdisp(c, p, action=1, modifier=1)
        assert int(p.read_register(1)) == 0  # trigger cleared
        assert c.last_gripper_command == {"action": 1, "modifier": 1}
        assert polls >= 1
        # Close, hard force (R[2]=2 close, R[3]=2).
        _run_gripdisp(c, p, action=2, modifier=2)
        assert c.last_gripper_command == {"action": 2, "modifier": 2}
        p.close()


def test_direct_gripdisp_call_wedges_and_r1_never_clears() -> None:
    with FakeCRXController() as c:
        p = RmiProbe(c.sm.host, c.rmi_port)
        next_seq = p.get_status()["NextSequenceID"]
        p.call(next_seq, "GRIPDISP")  # the forbidden direct call (B10 wedge)
        p.get_status()  # drain ack
        assert c.rmi.gripdisp_direct_called is True
        assert c.gripdisp_running is False
        # The dispatcher never forked → R[1] stays 1 forever (gripper wedge).
        p.write_register(3, 0)
        p.write_register(2, 1)
        p.write_register(1, 1)
        for _ in range(10):
            assert int(p.read_register(1)) == 1
        assert c.last_gripper_command is None
        p.close()


# ---------------------------------------------------------------------------
# Async push
# ---------------------------------------------------------------------------


def test_async_system_fault_push_is_observable() -> None:
    with FakeCRXController() as c:
        p = RmiProbe(c.sm.host, c.rmi_port)
        p.get_status()  # bring the redirect session live
        assert c.push_system_fault(event_id=99) is True
        pkt = p.read_raw(timeout=1.0)
        assert pkt["Communication"] == "FRC_SystemFault"
        assert pkt["EventID"] == 99
        p.close()


def test_read_error_reports_scripted_alarms() -> None:
    with FakeCRXController() as c:
        c.inject_alarm("SRVO-002", "Teach pendant E-stop")
        p = RmiProbe(c.sm.host, c.rmi_port)
        err = p.request({"Command": "FRC_ReadError", "Count": 5})
        assert err["Count"] >= 1
        assert any("SRVO-002" in v for v in err.values() if isinstance(v, str))
        p.close()


def test_disconnect_resets_next_sequence_id() -> None:
    with FakeCRXController(FakeCRXConfig(seq_seed=77)) as c:
        p = RmiProbe(c.sm.host, c.rmi_port)
        assert p.get_status()["NextSequenceID"] == 77
        p.request({"Communication": "FRC_Disconnect"})
        assert c.rmi.next_sequence_id == 1
        p.close()
