# SPDX-License-Identifier: Apache-2.0
"""Tests for :class:`airo_fanuc.receive_interface.FanucReceiveInterface`.

The velocity gate is the structural fix for the 2026-05-17 T1-freeze calibration
corruption: a still-vs-moving predicate over an LSQ window, a None-velocity hard
reject, a frozen-feed changed-position guard, and the J2/J3 ``rmi_unconverted``
source hard reject. A frozen feed must never read as "settled" — fabricating
0 deg/s is exactly what corrupts a hand-eye dataset.
"""

from __future__ import annotations

import time
from dataclasses import replace

import numpy as np
import pytest

from airo_fanuc.controller_facts import INTERIM_FACTS
from airo_fanuc.exceptions import (
    CalibrationError,
    CalibrationSourceError,
    CalibrationVelocityUnavailable,
)
from airo_fanuc.receive_interface import (
    SOURCE_RMI_UNCONVERTED,
    SOURCE_STREAM,
    FanucReceiveInterface,
    JointSample,
)
from airo_fanuc.rmi_client import RmiClient
from airo_fanuc.testing import FakeCRXController

_Q_A = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
_Q_B = [15.0, 25.0, 30.0, 40.0, 50.0, 60.0]


class _Clock:
    def __init__(self) -> None:
        self.ns = 0

    def __call__(self) -> int:
        return self.ns


def _feed(
    ri: FanucReceiveInterface,
    clock: _Clock,
    q0: list[float],
    v_deg_s: list[float],
    *,
    n: int = 10,
    dt_s: float = 0.06,
    t0_s: float = 0.0,
    source: str = SOURCE_STREAM,
) -> None:
    """Ingest ``n`` samples of ``q0 + v*t`` and park the clock 1 ms past the last."""
    q0a = np.asarray(q0, dtype=np.float64)
    va = np.asarray(v_deg_s, dtype=np.float64)
    last_t = t0_s
    for i in range(n):
        t = t0_s + i * dt_s
        last_t = t
        ri.ingest_sample(JointSample(q_deg=q0a + va * t, t_wall_ns=int(t * 1e9), source=source))
    clock.ns = int(last_t * 1e9) + 1_000_000


# ---------------------------------------------------------------------------
# Velocity estimate + settled predicate
# ---------------------------------------------------------------------------


def test_settled_true_when_still() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    _feed(ri, clock, _Q_A, [0.0] * 6)
    res = ri.settled()
    assert res.settled is True
    assert res.reason == "settled"
    assert res.max_velocity_deg_s is not None
    assert res.max_velocity_deg_s < 0.1
    assert res.sample_count >= 3


def test_settled_false_when_moving() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    _feed(ri, clock, _Q_A, [5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    res = ri.settled()
    assert res.settled is False
    assert res.reason == "moving"
    assert res.max_velocity_deg_s is not None
    assert res.max_velocity_deg_s == pytest.approx(5.0, abs=0.1)


def test_estimate_velocity_recovers_known_slope() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    _feed(ri, clock, _Q_A, [2.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    vel = ri.estimate_velocity_deg_s()
    assert vel is not None
    assert vel[0] == pytest.approx(2.0, abs=1e-6)
    assert vel[1] == pytest.approx(-1.0, abs=1e-6)


def test_velocity_unavailable_short_span_returns_none() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    # 3 samples spanning only 0.04 s << 0.5 s window ⇒ insufficient time base.
    _feed(ri, clock, _Q_A, [0.0] * 6, n=3, dt_s=0.02)
    assert ri.estimate_velocity_deg_s() is None
    res = ri.settled()
    assert res.settled is False
    assert res.reason == "velocity_unavailable"


def test_velocity_unavailable_frozen_stamps_returns_none() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    # All identical timestamps (a hard-frozen feed) ⇒ degenerate time base.
    for _ in range(6):
        ri.ingest_sample(JointSample(q_deg=np.asarray(_Q_A), t_wall_ns=1_000))
    clock.ns = 1_000
    assert ri.estimate_velocity_deg_s() is None


# ---------------------------------------------------------------------------
# Calibration capture: an unavailable velocity is rejected, never read as zero
# ---------------------------------------------------------------------------


def test_capture_rejects_none_velocity() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    _feed(ri, clock, _Q_A, [0.0] * 6, n=3, dt_s=0.02)  # short span ⇒ velocity None
    with pytest.raises(CalibrationVelocityUnavailable):
        ri.capture_calibration_sample()


def test_capture_rejects_while_moving() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    _feed(ri, clock, _Q_A, [5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    with pytest.raises(CalibrationError, match="still moving"):
        ri.capture_calibration_sample()


def test_capture_rejects_no_samples() -> None:
    ri = FanucReceiveInterface(now_ns=_Clock())
    with pytest.raises(CalibrationError, match="no joint samples"):
        ri.capture_calibration_sample()


# ---------------------------------------------------------------------------
# Calibration capture: J2/J3 source hard reject
# ---------------------------------------------------------------------------


def test_rmi_unconverted_hard_rejected_for_calibration() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    # A perfectly still RMI feed still hard-rejects — the reject is about the
    # J2/J3 representation, not motion.
    _feed(ri, clock, _Q_A, [0.0] * 6, source=SOURCE_RMI_UNCONVERTED)
    with pytest.raises(CalibrationSourceError) as ei:
        ri.capture_calibration_sample()
    assert ei.value.source == SOURCE_RMI_UNCONVERTED
    assert ei.value.fact == "rmi_to_stream_j3_plus_j2_verified"


def test_rmi_accepted_once_conversion_verified() -> None:
    # Flip the gate, as a second J2 confirming the RMI→stream J3 conversion would
    # (docs/controller-notes.md §1.5): RMI joints become acceptable and are retagged
    # rmi_converted via the single per-model policy point.
    clock = _Clock()
    facts = replace(INTERIM_FACTS, rmi_to_stream_j3_plus_j2_verified=True)
    ri = FanucReceiveInterface(now_ns=clock, facts=facts)
    _feed(ri, clock, _Q_A, [0.0] * 6, source=SOURCE_RMI_UNCONVERTED)
    sample = ri.capture_calibration_sample()
    assert sample.source == "rmi_converted"
    # That policy point retags only: q[2] += q[1] is not written there, so the values
    # pass through. Writing it must update this assertion in the same change.
    np.testing.assert_allclose(sample.q_deg, _Q_A)


def test_no_mixing_sources_in_one_dataset() -> None:
    clock = _Clock()
    facts = replace(INTERIM_FACTS, rmi_to_stream_j3_plus_j2_verified=True)
    ri = FanucReceiveInterface(now_ns=clock, facts=facts)
    _feed(ri, clock, _Q_A, [0.0] * 6, source=SOURCE_STREAM)
    ri.capture_calibration_sample()  # accept a stream sample first
    _feed(ri, clock, _Q_B, [0.0] * 6, t0_s=1.0, source=SOURCE_RMI_UNCONVERTED)
    with pytest.raises(CalibrationSourceError, match="mix"):
        ri.capture_calibration_sample()


# ---------------------------------------------------------------------------
# Calibration capture: frozen-feed changed-position guard
# ---------------------------------------------------------------------------


def test_capture_accepts_still_changed_pose() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    _feed(ri, clock, _Q_A, [0.0] * 6)
    first = ri.capture_calibration_sample()
    assert np.allclose(first.q_deg, _Q_A)
    _feed(ri, clock, _Q_B, [0.0] * 6, t0_s=1.0)
    second = ri.capture_calibration_sample()
    assert np.allclose(second.q_deg, _Q_B)


def test_capture_rejects_unchanged_pose_frozen_feed() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    _feed(ri, clock, _Q_A, [0.0] * 6)
    ri.capture_calibration_sample()  # accept pose A
    # Feed the SAME pose again at later timestamps (a frozen feed at rest):
    # still (velocity 0) but unchanged vs the previous accepted sample.
    _feed(ri, clock, _Q_A, [0.0] * 6, t0_s=1.0)
    with pytest.raises(CalibrationError, match="unchanged"):
        ri.capture_calibration_sample()


def test_settled_flags_position_unchanged_with_previous_q() -> None:
    clock = _Clock()
    ri = FanucReceiveInterface(now_ns=clock)
    _feed(ri, clock, _Q_A, [0.0] * 6)
    res = ri.settled(previous_q=np.asarray(_Q_A))
    assert res.settled is False
    assert res.reason == "position_unchanged"


# ---------------------------------------------------------------------------
# RMI status polling (commands-only session; never initializes)
# ---------------------------------------------------------------------------


def test_status_polling_uses_commands_only_session() -> None:
    with FakeCRXController() as crx:
        seen: list[str | None] = []
        orig = crx.rmi._dispatch

        def spy(req: dict) -> dict | None:
            seen.append(req.get("Command") or req.get("Instruction") or req.get("Communication"))
            return orig(req)

        crx.rmi._dispatch = spy  # type: ignore[method-assign]

        rmi = RmiClient(crx.sm.host, crx.rmi_port)
        ri = FanucReceiveInterface(joint_reader=None, rmi=rmi, status_poll_hz=25.0)
        ri.start()
        try:
            deadline = time.monotonic() + 2.0
            while ri.get_controller_status() is None and time.monotonic() < deadline:
                time.sleep(0.02)
        finally:
            ri.stop()

    status = ri.get_controller_status()
    assert status is not None
    assert ri.get_extended_status() is not None
    # NEVER initializes: FRC_Initialize claims the controller's motion interface and
    # belongs to the lifecycle supervisor alone. Status polling stays on the
    # commands-only session, so a read-only tool works in T1 and with no motion
    # session running.
    assert "FRC_Initialize" not in seen
    assert "FRC_GetStatus" in seen


# ---------------------------------------------------------------------------
# Lifecycle ordering: close RMI before releasing the flock, so the next owner can
# never open its single RMI session while ours is still up
# ---------------------------------------------------------------------------


def test_stop_closes_rmi_before_releasing_flock() -> None:
    log: list[str] = []

    class OrderRmi:
        def start(self) -> None:
            log.append("rmi.start")

        def stop(self) -> None:
            log.append("rmi.stop")

        def get_status(self) -> None:
            return None

        def get_extended_status(self, *, quiet: bool = False) -> None:
            return None

    class OrderLock:
        def acquire(self) -> OrderLock:
            log.append("lock.acquire")
            return self

        def release(self) -> None:
            log.append("lock.release")

    ri = FanucReceiveInterface(joint_reader=None, rmi=OrderRmi(), ownership=OrderLock())  # type: ignore[arg-type]
    ri.start()
    time.sleep(0.05)
    ri.stop()

    assert log.index("lock.acquire") < log.index("rmi.start")
    assert log.index("rmi.stop") < log.index("lock.release")


def test_start_releases_flock_if_rmi_open_fails() -> None:
    log: list[str] = []

    class FailRmi:
        def start(self) -> None:
            raise RuntimeError("connect refused")

        def stop(self) -> None:  # pragma: no cover - not reached
            log.append("rmi.stop")

        def get_status(self) -> None:  # pragma: no cover
            return None

        def get_extended_status(self, *, quiet: bool = False) -> None:  # pragma: no cover
            return None

    class OrderLock:
        def acquire(self) -> OrderLock:
            log.append("lock.acquire")
            return self

        def release(self) -> None:
            log.append("lock.release")

    ri = FanucReceiveInterface(joint_reader=None, rmi=FailRmi(), ownership=OrderLock())  # type: ignore[arg-type]
    with pytest.raises(RuntimeError, match="connect refused"):
        ri.start()
    # Ownership must be released when bring-up fails after acquiring it.
    assert log == ["lock.acquire", "lock.release"]
