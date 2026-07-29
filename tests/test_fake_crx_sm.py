# SPDX-License-Identifier: Apache-2.0
"""L2 self-tests for the FakeCRXController Stream Motion side + plant.

Prove the fake is a faithful executable spec: the handshake completes, status
streams at the ITP cadence under the injectable clock, the plant lags toward
commands with tau and settles, the deviation watchdog fires on an un-ramped
step but not a smooth ramp, TX-silence produces the interim hold, and strict
mode fires on a 0-dataStyle command and a double-send-in-one-window.

Deterministic + fast: all driving is manual-tick (no wall-time sleeps); UDP is
loopback so send→tick→recv is ordered.
"""

from __future__ import annotations

import socket
import struct

import numpy as np
import pytest

from airo_fanuc.testing import FakeCRXConfig, FakeCRXController, StrictConformanceError, wire
from airo_fanuc.testing.plant import JointPlant

_CMD_FMT = ">IIIBBH9d256B"


def _udp_client() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1.0)
    return s


def _q9(*vals: float) -> list[float]:
    out = [0.0] * 9
    for i, v in enumerate(vals):
        out[i] = float(v)
    return out


def _handshake(c: FakeCRXController, cli: socket.socket, *, launch_stream_motn: bool = True) -> int:
    """Drive defensive-stop → GetCapability → Start; return negotiated version.

    ``launch_stream_motn`` flips the RMI-coupled bit directly (the full RMI
    FRC_Call(STREAM_MOTN) coupling is exercised end-to-end in the scenario
    suite); here we isolate the Stream Motion mechanics.
    """
    addr = (c.sm.host, c.sm_port)
    cli.sendto(wire.encode_stop_packet(), addr)
    cli.sendto(wire.encode_get_capability_packet(), addr)
    c.tick()  # process handshake, emit capability reply
    cap = wire.decode_capability_result_packet(cli.recvfrom(4096)[0])
    assert cap is not None
    ver = cap.available_version
    cli.sendto(wire.encode_start_packet(version_no=ver), addr)
    if launch_stream_motn:
        c.state.stream_motn_launched = True
    c.tick()  # begin streaming, emit first status
    cli.recvfrom(4096)  # drain first status
    return ver


def _bad_command(
    seq: int,
    pos: list[float],
    *,
    version: int = 3,
    data_style: int = wire.COMMAND_DATA_STYLE,
    do_motn_ctrl: int = 1,
    io0: int = 0,
    is_last: int = 0,
) -> bytes:
    io = [0] * 256
    io[0] = io0
    return struct.pack(
        _CMD_FMT,
        wire.PACKET_TYPE_COMMAND,
        version,
        seq,
        is_last,
        do_motn_ctrl,
        data_style,
        *[float(v) for v in _fill9(pos)],
        *io,
    )


def _fill9(pos: list[float]) -> list[float]:
    out = [0.0] * 9
    out[: len(pos)] = [float(v) for v in pos[:9]]
    return out


# ---------------------------------------------------------------------------
# Handshake + cadence
# ---------------------------------------------------------------------------


def test_handshake_negotiates_capability_and_starts_streaming() -> None:
    with FakeCRXController(FakeCRXConfig(available_version=3)) as c:
        cli = _udp_client()
        addr = (c.sm.host, c.sm_port)
        cli.sendto(wire.encode_stop_packet(), addr)
        cli.sendto(wire.encode_get_capability_packet(), addr)
        c.tick()
        cap = wire.decode_capability_result_packet(cli.recvfrom(4096)[0])
        assert cap is not None
        assert cap.available_version == 3
        assert cap.sampling_rate_ms == 8
        assert not c.sm.streaming
        cli.sendto(wire.encode_start_packet(version_no=3), addr)
        c.tick()
        assert c.sm.streaming
        st = wire.decode_status_packet(cli.recvfrom(4096)[0])
        # At negotiated v3 the controller streams legacy type-202 (no force block) —
        # what the real P-1 controller sends (E6 pcap). type-204 requires v4.
        assert st is not None and st.packet_type == wire.PACKET_TYPE_STATUS_V3
        assert st.fs_type is None  # no force-sensor block on the v3 wire
        cli.close()


def test_status_streams_one_per_tick_with_incrementing_seq() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        _handshake(c, cli)
        seqs, stamps = [], []
        for _ in range(20):
            c.tick()
            st = wire.decode_status_packet(cli.recvfrom(4096)[0])
            assert st is not None
            seqs.append(st.sequence_no)
            stamps.append(st.time_stamp)
        # Exactly one packet per tick, strictly monotonic seq (+1 each).
        assert all(b - a == 1 for a, b in zip(seqs, seqs[1:], strict=False))
        # Timestamps advance with the injectable clock (never go backwards).
        assert all(b >= a for a, b in zip(stamps, stamps[1:], strict=False))
        assert stamps[-1] > stamps[0]
        cli.close()


def test_no_status_before_start() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        addr = (c.sm.host, c.sm_port)
        cli.sendto(wire.encode_get_capability_packet(), addr)
        c.tick()
        cli.recvfrom(4096)  # capability reply
        cli.settimeout(0.2)
        c.tick()  # not streaming yet
        with pytest.raises((socket.timeout, TimeoutError)):
            cli.recvfrom(4096)
        cli.close()


# ---------------------------------------------------------------------------
# motion_possible coupling
# ---------------------------------------------------------------------------


def test_motion_possible_requires_start_and_stream_motn() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        ver = _handshake(c, cli, launch_stream_motn=False)
        # Start done, but STREAM_MOTN not yet launched → motion_possible false.
        c.tick()
        st = wire.decode_status_packet(cli.recvfrom(4096)[0])
        assert st is not None and st.motion_possible is False
        # Launch STREAM_MOTN (coupled bit) + command the measured pose.
        c.state.stream_motn_launched = True
        cli.sendto(wire.encode_command_packet(1, _q9(0.0), version_no=ver), (c.sm.host, c.sm_port))
        c.tick()
        st = wire.decode_status_packet(cli.recvfrom(4096)[0])
        assert st is not None and st.motion_possible is True
        cli.close()


# ---------------------------------------------------------------------------
# Version negotiation v4 + ForceSensorConfig gate
# ---------------------------------------------------------------------------


def test_v4_fsconfig_enables_force_streaming() -> None:
    with FakeCRXController(FakeCRXConfig(available_version=4)) as c:
        c.set_force(3.0, -4.0, 5.0)
        cli = _udp_client()
        ver = _handshake(c, cli)
        assert ver == 4
        # Before FSConfig: fs_type Unavailable.
        c.tick()
        st = wire.decode_status_packet(cli.recvfrom(4096)[0])
        assert st is not None and st.fs_type == wire.FORCE_SENSOR_TYPE_UNAVAILABLE
        # Send FSConfig (v>=4 accepted) → force streams with EMBEDDED fs_type.
        cli.sendto(wire.encode_force_sensor_config_packet(version_no=4), (c.sm.host, c.sm_port))
        c.tick()
        st = wire.decode_status_packet(cli.recvfrom(4096)[0])
        assert st is not None
        assert st.fs_type == wire.FORCE_SENSOR_TYPE_EMBEDDED
        assert (st.force_x, st.force_y, st.force_z) == (3.0, -4.0, 5.0)
        cli.close()


def test_fsconfig_on_v3_raises_host380() -> None:
    with FakeCRXController(FakeCRXConfig(available_version=3)) as c:
        cli = _udp_client()
        _handshake(c, cli)
        cli.sendto(wire.encode_force_sensor_config_packet(version_no=3), (c.sm.host, c.sm_port))
        with pytest.raises(StrictConformanceError, match="HOST-380"):
            c.tick()
        assert any("HOST-380" in a for a in c.state.alarm_texts)
        assert c.state.in_error is True
        cli.close()


# ---------------------------------------------------------------------------
# Plant: first-order lag + deviation watchdog + TX-silence (object-level)
# ---------------------------------------------------------------------------


def test_plant_first_order_lag_settles_with_tau() -> None:
    tau, dt = 0.025, 0.008  # MEASURED servo lag (P-1 E9); was 0.107 interim
    p = JointPlant(tau_s=tau, itp_s=dt, deviation_watchdog_enabled=False)
    target = _q9(30.0)
    xs = []
    # ~12 tau: enough for a first-order response to a 30° step to settle AND its
    # velocity to decay below 1°/s (at the smaller measured tau, 6 tau still leaves
    # a few °/s of residual velocity — the settle time scales with tau).
    for _ in range(int(12 * tau / dt)):
        p.command_step(target, dt)
        xs.append(float(p.q_meas[0]))
    # Monotonic approach, settles near target, velocity decays to ~0.
    assert all(b >= a for a, b in zip(xs, xs[1:], strict=False))
    assert abs(p.q_meas[0] - 30.0) < 0.2
    assert abs(p.qd_meas[0]) < 1.0
    # One-tau response reaches ~63% of the step.
    p2 = JointPlant(tau_s=tau, itp_s=dt, deviation_watchdog_enabled=False)
    for _ in range(int(round(tau / dt))):
        p2.command_step(target, dt)
    assert 0.55 < p2.q_meas[0] / 30.0 < 0.72


def test_deviation_watchdog_fires_on_step_not_on_ramp() -> None:
    dt = 0.008
    ramp = JointPlant(tau_s=0.025, deviation_watchdog_deg=5.0, itp_s=dt, initial_q_deg=_q9())
    for i in range(1, 40):
        ramp.command_step(_q9(1.5 * i), dt)  # 1.5 deg/tick < 5 deg threshold
    assert ramp.watchdog_tripped is False

    step = JointPlant(tau_s=0.025, deviation_watchdog_deg=5.0, itp_s=dt, initial_q_deg=_q9())
    step.command_step(_q9(30.0), dt)  # 30 deg un-ramped step
    assert step.watchdog_tripped is True
    assert step.watchdog_joint == 0
    assert step.watchdog_step_deg == pytest.approx(30.0)


def test_deviation_watchdog_surfaces_controller_fault_on_wire() -> None:
    with FakeCRXController(FakeCRXConfig(strict=False)) as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        # Un-ramped 30 deg step → deviation watchdog trips → in_error, motion drops.
        cli.sendto(wire.encode_command_packet(1, _q9(30.0), version_no=ver), (c.sm.host, c.sm_port))
        c.tick()
        st = wire.decode_status_packet(cli.recvfrom(4096)[0])
        assert st is not None
        assert st.in_error is True
        assert st.motion_possible is False
        assert any("SRVO-023" in a for a in c.state.alarm_texts)
        cli.close()


def test_tx_silence_produces_interim_hold() -> None:
    with FakeCRXController(FakeCRXConfig(tx_silence_backstop_ok=True)) as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        # Drive a short ramp so the plant has non-zero velocity.
        for i in range(1, 6):
            cli.sendto(wire.encode_command_packet(i, _q9(1.0 * i), version_no=ver), (c.sm.host, c.sm_port))
            c.tick()
            cli.recvfrom(4096)
        assert c.tx_silent is False
        # Stop feeding commands: controller must decelerate to a hold.
        c.advance(6)
        assert c.tx_silent is True
        assert abs(float(c.qd_meas_deg_s[0])) < 1e-6
        q_hold = c.q_meas_deg.copy()
        c.advance(10)
        assert np.allclose(c.q_meas_deg, q_hold)  # frozen
        cli.close()


# ---------------------------------------------------------------------------
# Strict-conformance mode
# ---------------------------------------------------------------------------


def test_strict_rejects_zero_datastyle() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        cli.sendto(_bad_command(1, _q9(0.1), version=ver, data_style=0x0000), (c.sm.host, c.sm_port))
        with pytest.raises(StrictConformanceError, match="dataStyle"):
            c.tick()
        cli.close()


def test_strict_rejects_double_send_in_one_window() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        addr = (c.sm.host, c.sm_port)
        cli.sendto(wire.encode_command_packet(1, _q9(0.1), version_no=ver), addr)
        cli.sendto(wire.encode_command_packet(2, _q9(0.2), version_no=ver), addr)
        with pytest.raises(StrictConformanceError, match="one-TX-per-window|window"):
            c.tick()
        cli.close()


def test_strict_rejects_non_monotonic_sequence() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        addr = (c.sm.host, c.sm_port)
        cli.sendto(wire.encode_command_packet(5, _q9(0.1), version_no=ver), addr)
        c.tick()
        cli.recvfrom(4096)
        cli.sendto(wire.encode_command_packet(5, _q9(0.2), version_no=ver), addr)  # duplicate seq
        with pytest.raises(StrictConformanceError, match="strictly increasing"):
            c.tick()
        cli.close()


def test_strict_rejects_do_motn_ctrl_zero() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        cli.sendto(
            wire.encode_command_packet(1, _q9(0.1), do_motn_ctrl=False, version_no=ver),
            (c.sm.host, c.sm_port),
        )
        with pytest.raises(StrictConformanceError, match="do_motn_ctrl"):
            c.tick()
        cli.close()


def test_strict_rejects_nonzero_io() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        cli.sendto(_bad_command(1, _q9(0.1), version=ver, io0=1), (c.sm.host, c.sm_port))
        with pytest.raises(StrictConformanceError, match="io_command"):
            c.tick()
        cli.close()


def test_strict_rejects_version_drift() -> None:
    with FakeCRXController() as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        cli.sendto(wire.encode_command_packet(1, _q9(0.1), version_no=ver + 7), (c.sm.host, c.sm_port))
        with pytest.raises(StrictConformanceError, match="version_no"):
            c.tick()
        cli.close()


def test_strict_can_be_disabled() -> None:
    with FakeCRXController(FakeCRXConfig(strict=False)) as c:
        cli = _udp_client()
        ver = _handshake(c, cli)
        # A 0-dataStyle command must NOT raise when strict mode is off.
        cli.sendto(_bad_command(1, _q9(0.1), version=ver, data_style=0), (c.sm.host, c.sm_port))
        c.tick()  # no raise
        cli.recvfrom(4096)
        cli.close()
