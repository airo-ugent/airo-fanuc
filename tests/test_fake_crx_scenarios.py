# SPDX-License-Identifier: Apache-2.0
"""L2 scenario-matrix self-tests for the FakeCRXController (design doc 09).

End-to-end rows that exercise BOTH protocol planes together and the fault
matrix. Coverage vs the S-* matrix:

* **S-VERS** — full v3 and v4 bring-up: RMI FRC_Call(STREAM_MOTN) then the SM
  handshake, ``motion_possible`` asserting only when both are satisfied.
* **S-LIFE** — e-stop cycle (drop → latched in_error → FRC_Reset recovery),
  contact stop {2,4}, SAFETY_CLAMP, motion_possible-drop-alone, TEACH toggle,
  FRC_SystemFault push.
* **S-DROP** — status drop / duplicate / sequence-gap taxonomy on the wire.

Deferred (documented in the report): S-SOAK (long mixed-workload), S-FLAP
(dries 9-flap/7-oscillation replays), S-API / S-B15 brake-on-wire and the
servo-sine rows — those belong to the C++-core (L1) and driver (P4) layers that
consume this fake, not to the fake's own self-tests.
"""

from __future__ import annotations

import json
import socket

import pytest

from airo_fanuc.testing import FakeCRXConfig, FakeCRXController, wire

_ECHO_KEYS = ("Command", "Communication", "Instruction")


# --- lean RMI + UDP helpers (self-contained; see test_fake_crx_rmi for the
#     fuller RmiProbe) --------------------------------------------------------


def _ident(obj: dict):
    for k in _ECHO_KEYS:
        if k in obj:
            return obj[k]
    return None


class _Rmi:
    def __init__(self, host: str, boot_port: int) -> None:
        b = socket.socket()
        b.settimeout(2.0)
        b.connect((host, boot_port))
        b.sendall(b'{"Communication": "FRC_Connect_STMO"}\r\n')
        reply = json.loads(b.recv(4096).split(b"\n", 1)[0].rstrip(b"\r").decode())
        b.close()
        self.sock = socket.socket()
        self.sock.settimeout(2.0)
        self.sock.connect((host, int(reply["PortNumber"])))
        self._buf = b""

    def request(self, obj: dict) -> dict:
        self.sock.sendall((json.dumps(obj) + "\r\n").encode())
        want = _ident(obj)
        while True:
            pkt = self._recv()
            if _ident(pkt) == want:
                return pkt

    def call(self, seq: int, program: str) -> None:
        req = {"Instruction": "FRC_Call", "SequenceID": seq, "ProgramName": program}
        self.sock.sendall((json.dumps(req) + "\r\n").encode())

    def read_raw(self, timeout: float = 1.0) -> dict:
        self.sock.settimeout(timeout)
        return self._recv()

    def close(self) -> None:
        self.sock.close()

    def _recv(self) -> dict:
        while b"\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise OSError("closed")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line.rstrip(b"\r").decode())


def _udp() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1.0)
    return s


def _bringup(c: FakeCRXController) -> tuple[_Rmi, socket.socket, int]:
    """Full two-plane bring-up: RMI ladder + FRC_Call(STREAM_MOTN), then the SM
    defensive-stop → GetCapability → Start handshake. Returns (rmi, udp, ver)."""
    rmi = _Rmi(c.sm.host, c.rmi_port)
    rmi.request({"Command": "FRC_GetStatus"})
    rmi.request({"Command": "FRC_Reset"})
    rmi.request({"Command": "FRC_Initialize"})
    next_seq = rmi.request({"Command": "FRC_GetStatus"})["NextSequenceID"]
    rmi.call(next_seq, "STREAM_MOTN")
    rmi.request({"Command": "FRC_GetStatus"})  # drain the FRC_Call ack orphan

    cli = _udp()
    addr = (c.sm.host, c.sm_port)
    cli.sendto(wire.encode_stop_packet(), addr)
    cli.sendto(wire.encode_get_capability_packet(), addr)
    c.tick()
    cap = wire.decode_capability_result_packet(cli.recvfrom(4096)[0])
    assert cap is not None
    ver = cap.available_version
    cli.sendto(wire.encode_start_packet(version_no=ver), addr)
    c.tick()
    cli.recvfrom(4096)
    return rmi, cli, ver


def _command_hold(c: FakeCRXController, cli: socket.socket, ver: int, seq: int) -> wire.RobotStatusPacket:
    """Command the current measured pose (no watchdog trip), tick, read status."""
    q = list(c.q_meas_deg)
    cli.sendto(wire.encode_command_packet(seq, q, version_no=ver), (c.sm.host, c.sm_port))
    c.tick()
    st = wire.decode_status_packet(cli.recvfrom(4096)[0])
    assert st is not None
    return st


# ---------------------------------------------------------------------------
# S-VERS — full bring-up, both planes
# ---------------------------------------------------------------------------


def test_full_bringup_v3_motion_possible_after_stream_motn() -> None:
    with FakeCRXController(FakeCRXConfig(available_version=3)) as c:
        rmi, cli, ver = _bringup(c)
        assert ver == 3
        assert c.state.stream_motn_launched is True
        st = _command_hold(c, cli, ver, 1)
        assert st.motion_possible is True
        assert st.in_error is False and st.e_stopped is False
        rmi.close()
        cli.close()


def test_full_bringup_v4() -> None:
    with FakeCRXController(FakeCRXConfig(available_version=4)) as c:
        rmi, cli, ver = _bringup(c)
        assert ver == 4
        # v4 accepts ForceSensorConfig.
        cli.sendto(wire.encode_force_sensor_config_packet(version_no=4), (c.sm.host, c.sm_port))
        st = _command_hold(c, cli, ver, 1)
        assert st.motion_possible is True
        assert st.fs_type == wire.FORCE_SENSOR_TYPE_EMBEDDED
        rmi.close()
        cli.close()


# ---------------------------------------------------------------------------
# S-LIFE — fault matrix rows
# ---------------------------------------------------------------------------


def test_estop_cycle_drops_then_reset_recovers() -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        assert _command_hold(c, cli, ver, 1).motion_possible is True

        c.press_estop()
        st = _command_hold(c, cli, ver, 2)
        assert st.e_stopped is True and st.in_error is True
        assert st.motion_possible is False

        # Releasing the button leaves in_error latched (needs FRC_Reset).
        c.release_estop()
        st = _command_hold(c, cli, ver, 3)
        assert st.e_stopped is False and st.in_error is True
        assert st.motion_possible is False

        # The recovery ladder's FRC_Reset clears the latched fault.
        assert rmi.request({"Command": "FRC_Reset"})["ErrorID"] == 0
        st = _command_hold(c, cli, ver, 4)
        assert st.in_error is False and st.motion_possible is True
        rmi.close()
        cli.close()


@pytest.mark.parametrize("mode", [2, 4])
def test_contact_stop_modes_reflected_on_wire(mode: int) -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        c.set_contact_stop(mode)
        st = _command_hold(c, cli, ver, 1)
        assert int(st.contact_stop_status) == mode
        rmi.close()
        cli.close()


def test_safety_scale_clamp_reflected() -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        c.set_safety_scale(0.02)
        st = _command_hold(c, cli, ver, 1)
        assert st.safety_scale == pytest.approx(0.02, abs=1e-6)
        rmi.close()
        cli.close()


def test_motion_possible_drop_alone() -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        assert _command_hold(c, cli, ver, 1).motion_possible is True
        c.drop_motion_possible()  # steady-state drop with no e-stop / in_error
        st = _command_hold(c, cli, ver, 2)
        assert st.motion_possible is False
        assert st.e_stopped is False and st.in_error is False
        rmi.close()
        cli.close()


def test_teach_toggle_streams_state_and_flips_tp_enabled() -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        c.set_teach(True)
        st = _command_hold(c, cli, ver, 1)
        assert st.tp_enabled is True
        # Status keeps flowing in TEACH (T1) — no freeze (S-B16 invariant).
        st2 = _command_hold(c, cli, ver, 2)
        assert st2.tp_enabled is True
        # RMI GetStatus reports T1 (TPMode 1) while tp_enabled.
        assert rmi.request({"Command": "FRC_GetStatus"})["TPMode"] == 1
        c.set_teach(False)
        assert _command_hold(c, cli, ver, 3).tp_enabled is False
        rmi.close()
        cli.close()


def test_system_fault_push_during_streaming() -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        _command_hold(c, cli, ver, 1)
        assert c.push_system_fault(event_id=7) is True
        pkt = rmi.read_raw(timeout=1.0)
        assert pkt["Communication"] == "FRC_SystemFault" and pkt["EventID"] == 7
        rmi.close()
        cli.close()


# ---------------------------------------------------------------------------
# S-DROP — status stream perturbation taxonomy
# ---------------------------------------------------------------------------


def test_status_drop_produces_rx_silence_window() -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        cli.settimeout(0.15)
        c.drop_status(3)
        for _ in range(3):
            c.tick()
            with pytest.raises((socket.timeout, TimeoutError)):
                cli.recvfrom(4096)
        # Stream resumes after the drop window.
        c.tick()
        assert wire.decode_status_packet(cli.recvfrom(4096)[0]) is not None
        rmi.close()
        cli.close()


def test_status_sequence_gap_injection() -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        c.tick()
        s0 = wire.decode_status_packet(cli.recvfrom(4096)[0]).sequence_no
        c.inject_seq_gap(5)
        c.tick()
        s1 = wire.decode_status_packet(cli.recvfrom(4096)[0]).sequence_no
        assert s1 - s0 == 6  # +1 normal advance, +5 injected gap
        rmi.close()
        cli.close()


def test_status_duplicate_injection() -> None:
    with FakeCRXController() as c:
        rmi, cli, ver = _bringup(c)
        c.duplicate_next_status()
        c.tick()
        a = wire.decode_status_packet(cli.recvfrom(4096)[0])
        b = wire.decode_status_packet(cli.recvfrom(4096)[0])
        assert a.sequence_no == b.sequence_no  # byte-identical duplicate
        rmi.close()
        cli.close()
