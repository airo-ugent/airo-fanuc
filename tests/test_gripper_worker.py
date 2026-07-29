# SPDX-License-Identifier: Apache-2.0
"""Tests for :class:`airo_fanuc.gripper_worker.GripperWorker` (PLAN §5.4, R2 F32).

Covers the GRIPDISP register sequence happy path against the real
:class:`~airo_fanuc.testing.FakeCRXController` + :class:`RmiClient`, plus the
timeout / recovery-fast-reject / validation / serialization behaviors against a
deterministic in-memory RMI double.
"""

from __future__ import annotations

import threading
import time

import pytest

from airo_fanuc.gripper import (
    ACTION_CLOSE,
    ACTION_OPEN,
    FORCE_HARD,
    OPEN_MID,
    REG_ACTION,
    REG_CMD,
    REG_R3,
)
from airo_fanuc.gripper_worker import GripperWorker
from airo_fanuc.rmi_client import RmiClient
from airo_fanuc.testing import FakeCRXController


class FakeGripperRmi:
    """Deterministic in-memory GRIPDISP model (no sockets, no threads).

    Mirrors the FakeCRX register semantics: writing ``R[1]=1`` latches a pending
    action; each subsequent read of ``R[1]`` advances the dispatcher and, after
    ``clears_after`` reads, clears ``R[1]`` to 0 (unless ``never_clears``).
    """

    def __init__(self, *, clears_after: int = 1, never_clears: bool = False) -> None:
        self.registers: dict[int, float] = {}
        self.write_log: list[tuple[int, float]] = []
        self.read_count = 0
        self._clears_after = clears_after
        self._never_clears = never_clears
        self._reads_since_trigger: int | None = None

    def write_register(self, register_number: int, value: float | int) -> None:
        self.write_log.append((register_number, float(value)))
        self.registers[register_number] = float(value)
        if register_number == REG_CMD and int(value) == 1:
            self._reads_since_trigger = 0

    def read_register(self, register_number: int) -> float:
        self.read_count += 1
        if register_number == REG_CMD and self._reads_since_trigger is not None and not self._never_clears:
            self._reads_since_trigger += 1
            if self._reads_since_trigger >= self._clears_after:
                self.registers[REG_CMD] = 0.0
                self._reads_since_trigger = None
        return float(self.registers.get(register_number, 0.0))


class RaisingRmi:
    """RMI double whose register access raises — proves fast-reject never touches it."""

    def __init__(self) -> None:
        self.write_calls = 0
        self.read_calls = 0

    def write_register(self, register_number: int, value: float | int) -> None:
        self.write_calls += 1
        raise AssertionError("write_register must not be called")

    def read_register(self, register_number: int) -> float:
        self.read_calls += 1
        raise AssertionError("read_register must not be called")


# ---------------------------------------------------------------------------
# Happy path against the real FakeCRXController + RmiClient
# ---------------------------------------------------------------------------


def test_open_close_happy_path_against_fakecrx() -> None:
    with FakeCRXController() as crx:
        rmi = RmiClient(crx.sm.host, crx.rmi_port)
        rmi.start()
        try:
            # Start the GRIPDISP dispatcher (fire-and-forget FRC_Call(GRPRUN)).
            rmi.program_call("GRPRUN")
            deadline = time.monotonic() + 2.0
            while not crx.gripdisp_running and time.monotonic() < deadline:
                time.sleep(0.01)
            assert crx.gripdisp_running

            worker = GripperWorker(rmi, dispatch_timeout_s=2.0)
            try:
                result = worker.open_gripper_and_wait(open_state=OPEN_MID, timeout=3.0)
                assert result is not None and result["success"] is True
                # R[3] carried the open-state selector; R[2] the open action.
                assert crx.last_gripper_command == {"action": ACTION_OPEN, "modifier": OPEN_MID}

                result = worker.close_gripper_and_wait(close_force=FORCE_HARD, timeout=3.0)
                assert result is not None and result["success"] is True
                assert crx.last_gripper_command == {"action": ACTION_CLOSE, "modifier": FORCE_HARD}
            finally:
                worker.close()
        finally:
            rmi.stop()


# ---------------------------------------------------------------------------
# Register sequence + poll behavior (deterministic double)
# ---------------------------------------------------------------------------


def test_register_poke_sequence_order() -> None:
    rmi = FakeGripperRmi(clears_after=1)
    with GripperWorker(rmi, dispatch_timeout_s=1.0) as worker:
        result = worker.open_gripper_and_wait(open_state=OPEN_MID, timeout=2.0)
    assert result is not None and result["success"] is True
    # Poke order: R[3]=modifier, R[2]=action, R[1]=1 (dries sequence).
    assert rmi.write_log == [(REG_R3, float(OPEN_MID)), (REG_ACTION, float(ACTION_OPEN)), (REG_CMD, 1.0)]


def test_poll_loop_waits_for_clear() -> None:
    rmi = FakeGripperRmi(clears_after=3)
    with GripperWorker(rmi, dispatch_timeout_s=2.0) as worker:
        result = worker.close_gripper_and_wait(timeout=2.0)
    assert result is not None and result["success"] is True
    assert rmi.read_count >= 3


def test_timeout_returns_dict_fail() -> None:
    rmi = FakeGripperRmi(never_clears=True)
    with GripperWorker(rmi, dispatch_timeout_s=0.3) as worker:
        result = worker.open_gripper_and_wait(open_state=0, timeout=None)
    assert result is not None
    assert result["success"] is False
    assert "timed out" in str(result["message"])


def test_write_failure_returns_dict_fail() -> None:
    class WriteFailRmi(FakeGripperRmi):
        def write_register(self, register_number: int, value: float | int) -> None:
            raise RuntimeError("socket blip")

    with GripperWorker(WriteFailRmi(), dispatch_timeout_s=1.0) as worker:
        result = worker.open_gripper_and_wait(timeout=2.0)
    assert result is not None and result["success"] is False
    assert "trigger registers" in str(result["message"])


# ---------------------------------------------------------------------------
# Recovery fast-reject (R2 F32)
# ---------------------------------------------------------------------------


def test_recovery_gate_rejects_fast_without_touching_rmi() -> None:
    rmi = RaisingRmi()
    with GripperWorker(rmi, dispatch_timeout_s=5.0) as worker:
        worker.set_recovery(True)
        t0 = time.monotonic()
        result = worker.open_gripper_and_wait(open_state=0, timeout=2.0)
        elapsed = time.monotonic() - t0
    assert result is not None and result["success"] is False
    assert "recovery" in str(result["message"]).lower()
    # Fast: no 5 s poll loop, no RMI access at all.
    assert elapsed < 1.0
    assert rmi.write_calls == 0 and rmi.read_calls == 0


def test_recovery_gate_cleared_allows_command() -> None:
    rmi = FakeGripperRmi(clears_after=1)
    with GripperWorker(rmi, dispatch_timeout_s=1.0) as worker:
        worker.set_recovery(True)
        assert worker.open_gripper_and_wait(timeout=2.0)["success"] is False  # type: ignore[index]
        worker.set_recovery(False)
        result = worker.open_gripper_and_wait(timeout=2.0)
    assert result is not None and result["success"] is True


# ---------------------------------------------------------------------------
# Validation (ValueError; bool rejected)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("bad", [5, -1, 3])
def test_invalid_open_state_raises(bad: int) -> None:
    worker = GripperWorker(FakeGripperRmi())
    with pytest.raises(ValueError, match="open_state"):
        worker.open_gripper(open_state=bad)
    worker.close()


@pytest.mark.parametrize("bad", [5, -1, 3])
def test_invalid_close_force_raises(bad: int) -> None:
    worker = GripperWorker(FakeGripperRmi())
    with pytest.raises(ValueError, match="close_force"):
        worker.close_gripper(close_force=bad)
    worker.close()


def test_bool_selector_rejected() -> None:
    worker = GripperWorker(FakeGripperRmi())
    with pytest.raises(ValueError, match="open_state"):
        worker.open_gripper(open_state=True)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="close_force"):
        worker.close_gripper(close_force=False)  # type: ignore[arg-type]
    worker.close()


# ---------------------------------------------------------------------------
# Serialization (one physical gripper)
# ---------------------------------------------------------------------------


def test_commands_are_serialized() -> None:
    """Two commands submitted back to back never interleave on the wire."""
    order_lock = threading.Lock()
    events: list[str] = []

    class SlowRmi(FakeGripperRmi):
        def write_register(self, register_number: int, value: float | int) -> None:
            if register_number == REG_R3:
                with order_lock:
                    events.append("begin")
                time.sleep(0.02)
            super().write_register(register_number, value)
            if register_number == REG_CMD:
                with order_lock:
                    events.append("end")

    rmi = SlowRmi(clears_after=1)
    with GripperWorker(rmi, dispatch_timeout_s=1.0) as worker:
        worker.open_gripper(open_state=0)
        worker.close_gripper(close_force=0)
        # The single worker runs the two commands strictly one after another;
        # poll until both have completed (a single _done event can't reliably
        # signal "the second specific command finished").
        deadline = time.monotonic() + 3.0
        while len(events) < 4 and time.monotonic() < deadline:
            time.sleep(0.01)
    # Each command's begin must be followed by its own end before the next begin
    # — never begin/begin/end/end.
    assert events == ["begin", "end", "begin", "end"]
