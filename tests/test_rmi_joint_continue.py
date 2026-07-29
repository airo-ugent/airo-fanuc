# SPDX-License-Identifier: Apache-2.0
"""Tests for the RMI joint-read and program-resume commands: FRC_ReadJointAngles
and FRC_Continue.

Covers, against the real-socket :class:`~airo_fanuc.testing.FakeCRXController`
RMI emulator (mirroring the ``test_rmi_client`` pattern):

* :meth:`RmiClient.read_joint_angles` returns the controller joints, with the
  vendor ``J3 += J2`` conversion visible in the reply (see
  ``controller_facts.rmi_j3_plus_j2_conversion``);
* the emitted request bytes byte-match the committed RMI wire goldens;
* :meth:`RmiClient.program_continue` succeeds on a paused program, tolerates
  2556938 ("TP Program is Not Paused.") as a no-op on an unpaused one, and still
  raises :class:`RmiError` on any other ErrorID;
* :class:`RmiClientJointReader` tags samples :data:`SOURCE_RMI_UNCONVERTED` and
  the :class:`FanucReceiveInterface` calibration path HARD-REJECTS them — an
  unconverted J3 would silently bias a calibration by the J2 angle.
"""

from __future__ import annotations

import json
import socket
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

from airo_fanuc.exceptions import CalibrationSourceError, RmiError
from airo_fanuc.receive_interface import (
    SOURCE_RMI_UNCONVERTED,
    FanucReceiveInterface,
    RmiClientJointReader,
)
from airo_fanuc.rmi_client import RmiClient
from airo_fanuc.testing import FakeCRXController
from airo_fanuc.testing.fake_crx_rmi import ERR_INVALID_CONTROLLER_STATE, ERR_TP_NOT_PAUSED

_GOLDENS_DIR = Path(__file__).resolve().parent / "goldens" / "rmi"

# True plant joints (deg) chosen so J3 + J2 is unambiguous: J2=5, J3=30 →
# reported J3=35, distinct from every other axis, so the +J2 offset is visible.
_Q_TRUE = [10.0, 5.0, 30.0, 40.0, 50.0, 60.0]


# ---------------------------------------------------------------------------
# Helpers (mirror test_rmi_client.py)
# ---------------------------------------------------------------------------


def _client(c: FakeCRXController, **kwargs: Any) -> RmiClient:
    """An RmiClient pointed at the fake controller's RMI bootstrap port."""
    return RmiClient(c.sm.host, c.rmi_port, **kwargs)


def _wait_until(pred: Callable[[], bool], timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


def _golden_wire(name: str) -> bytes:
    """The exact on-wire bytes pinned by the golden ``name``."""
    data = json.loads((_GOLDENS_DIR / f"{name}.json").read_text(encoding="utf-8"))
    return data["wire"].encode("ascii")


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
# read_joint_angles: values + the vendor J3 += J2 conversion
# ---------------------------------------------------------------------------


def test_read_joint_angles_returns_joints_with_j3_plus_j2() -> None:
    with FakeCRXController(initial_q_deg=_Q_TRUE) as c:
        with _client(c) as rmi:
            joints = rmi.read_joint_angles()

    # Nine axes reported (6-DOF CRX + three zero-padded).
    assert len(joints) == 9
    assert joints[0] == pytest.approx(10.0)
    assert joints[1] == pytest.approx(5.0)  # J2 unchanged
    # J3 carries the vendor +J2 offset: 30 + 5 = 35 (NOT the true 30).
    assert joints[2] == pytest.approx(35.0)
    assert joints[2] != pytest.approx(30.0)
    assert joints[3:6] == pytest.approx([40.0, 50.0, 60.0])
    assert joints[6:] == pytest.approx([0.0, 0.0, 0.0])


def test_read_joint_angles_tracks_the_plant() -> None:
    # A different plant pose flows straight through (reply tracks shared state).
    with FakeCRXController(initial_q_deg=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]) as c:
        with _client(c) as rmi:
            joints = rmi.read_joint_angles()
    assert joints[1] == pytest.approx(2.0)
    assert joints[2] == pytest.approx(5.0)  # 3 + 2


# ---------------------------------------------------------------------------
# Emitted request bytes byte-match the goldens
# ---------------------------------------------------------------------------


def test_read_joint_angles_and_continue_emit_golden_bytes() -> None:
    sink: list[bytes] = []
    with FakeCRXController(initial_q_deg=_Q_TRUE) as c:
        rmi = _RecordingRmiClient(c.sm.host, c.rmi_port, sink=sink)
        rmi.start()
        try:
            rmi.read_joint_angles()  # emits FRC_ReadJointAngles
            rmi.program_continue()  # emits FRC_Continue (paused by default → OK)
        finally:
            rmi.stop()

    assert _golden_wire("read_joint_angles") in sink
    assert _golden_wire("continue") in sink


# ---------------------------------------------------------------------------
# program_continue: paused OK, unpaused tolerated, other errors raise
# ---------------------------------------------------------------------------


def test_program_continue_succeeds_on_paused_program() -> None:
    with FakeCRXController() as c:  # default: program paused → ErrorID 0
        with _client(c) as rmi:
            rmi.program_continue()  # must not raise


def test_program_continue_tolerates_2556938_when_unpaused() -> None:
    with FakeCRXController() as c:
        c.rmi.set_program_paused(False)  # unpaused → controller returns 2556938
        with _client(c) as rmi:
            rmi.program_continue()  # 2556938 tolerated as a no-op — must not raise


def test_program_continue_tolerates_injected_2556938() -> None:
    with FakeCRXController() as c:
        c.arm_rmi_error("FRC_Continue", ERR_TP_NOT_PAUSED, times=1)
        with _client(c) as rmi:
            rmi.program_continue()  # injected 2556938 also tolerated


def test_program_continue_raises_on_other_error() -> None:
    with FakeCRXController() as c:
        c.arm_rmi_error("FRC_Continue", ERR_INVALID_CONTROLLER_STATE, times=1)  # 2556943
        with _client(c) as rmi:
            with pytest.raises(RmiError) as ei:
                rmi.program_continue()
    assert ei.value.error_id == ERR_INVALID_CONTROLLER_STATE


# ---------------------------------------------------------------------------
# RmiClientJointReader tags SOURCE_RMI_UNCONVERTED; calibration hard-rejects it
# ---------------------------------------------------------------------------


def test_rmi_client_joint_reader_tags_unconverted() -> None:
    with FakeCRXController(initial_q_deg=_Q_TRUE) as c:
        with _client(c) as rmi:
            reader = RmiClientJointReader(rmi)
            sample = reader.read()

    assert sample is not None
    assert sample.source == SOURCE_RMI_UNCONVERTED
    assert sample.t_wall_ns > 0
    # J3 carries the +J2 offset the reader deliberately does NOT convert.
    assert sample.q_deg[2] == pytest.approx(35.0)


def test_receive_interface_hard_rejects_rmi_reader_sample() -> None:
    with FakeCRXController(initial_q_deg=_Q_TRUE) as c:
        with _client(c) as rmi:
            reader = RmiClientJointReader(rmi)
            sample = reader.read()

    assert sample is not None
    # Feed the RMI-sourced sample into the calibration path: it must hard-reject,
    # not silently calibrate off unconverted joints.
    ri = FanucReceiveInterface()
    ri.ingest_sample(sample)
    with pytest.raises(CalibrationSourceError) as ei:
        ri.capture_calibration_sample()
    assert ei.value.source == SOURCE_RMI_UNCONVERTED
    assert ei.value.fact == "rmi_joints_identical_to_stream"


def test_rmi_client_joint_reader_returns_none_on_dead_session() -> None:
    # A torn-down controller makes read_joint_angles raise; the reader swallows
    # it and returns None per the JointReader contract (poll loop keeps running).
    c = FakeCRXController().start()
    rmi = _client(c, request_timeout=0.3)
    rmi.start()
    try:
        reader = RmiClientJointReader(rmi)
        assert reader.read() is not None  # live session reads fine
        c.close()  # tear the controller down entirely
        assert _wait_until(lambda: rmi._sock is None, timeout=2.0)
        assert reader.read() is None  # dead session → None, not an exception
    finally:
        rmi.stop()
        c.close()
