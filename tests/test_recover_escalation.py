# SPDX-License-Identifier: Apache-2.0
"""``FanucDriver.recover()`` Tier-2 cold-reconnect escalation + gripper handling.

The light supervisor ladder cannot re-arm ``motion_possible`` after a collision
``FRC_SystemFault`` (it de-initialises the controller's RMI motion interface —
HW-observed). ``FanucDriver.recover()`` therefore escalates to a cold
``reconnect()`` (= full re-bring-up: re-``initialize`` + GRIPDISP re-fork) — but ONLY
for faults a re-bring-up can clear (not a held e-stop / teach / operator-required).
``reconnect()`` holds the gripper fail-fast gate across the rebuild so no gripper
command actuates GRIPDISP mid-reconnect. What ``recover()`` returns is whether the
driver reached STREAMING, not whether it is commandable — the ARM gate stays latched
across a successful e-stop recovery by design.

Unit-level by design: the escalation *decision* + the gripper-gate ordering are pure
policy, so these drive a ``FanucDriver`` with ``__init__`` bypassed and the
collaborators (supervisor / core / rmi / gripper) faked — no FakeCRX, milliseconds.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from airo_fanuc.driver import FanucDriver
from airo_fanuc.exceptions import FanucConnectionError
from airo_fanuc.lifecycle import LifecycleState


def _bare_driver() -> FanucDriver:
    d = FanucDriver.__new__(FanucDriver)
    d._closed = False
    d.core = None
    d.rmi = None
    d.gripper = None
    d._supervisor = None
    return d


def _driver_with_failing_ladder(**human_required: bool) -> FanucDriver:
    """A driver whose light ladder fails, so ``recover()`` reaches the escalation.

    ``state()`` reports STREAMING because that is what ``recover()`` reads after a
    reconnect — deliberately NOT ``is_commandable()``, which would also fold in the
    ARM gate and report a fine robot waiting for ``arm()`` as a failed reconnect.
    """
    d = _bare_driver()
    d._supervisor = SimpleNamespace(
        recover=lambda timeout_s=None: False,
        state=lambda: LifecycleState.STREAMING,
    )
    st = {"e_stopped": False, "tp_enabled": False, "operator_required": False, **human_required}
    d.get_state = lambda: st  # type: ignore[method-assign]
    return d


class TestRecoverEscalation:
    def test_no_escalation_when_ladder_succeeds(self) -> None:
        d = _bare_driver()
        d._supervisor = SimpleNamespace(recover=lambda timeout_s=None: True)
        called: list[str] = []
        d.reconnect = lambda: called.append("reconnect")  # type: ignore[method-assign]
        assert d.recover() is True
        assert called == []  # never cold-reconnected

    def test_escalates_to_reconnect_on_recoverable_fault(self) -> None:
        d = _driver_with_failing_ladder()
        called: list[str] = []
        d.reconnect = lambda: called.append("reconnect")  # type: ignore[method-assign]
        assert d.recover() is True  # STREAMING after the reconnect
        assert called == ["reconnect"]

    @pytest.mark.parametrize("flag", ["e_stopped", "tp_enabled", "operator_required"])
    def test_no_reconnect_for_human_required_fault(self, flag: str) -> None:
        d = _driver_with_failing_ladder(**{flag: True})
        called: list[str] = []
        d.reconnect = lambda: called.append("reconnect")  # type: ignore[method-assign]
        assert d.recover() is False  # human required — leave FAULTED, do not reconnect
        assert called == []

    def test_escalate_reconnect_false_never_reconnects(self) -> None:
        # Calibration free-drive heal opts out: light ladder only, no cold reconnect.
        d = _driver_with_failing_ladder()
        called: list[str] = []
        d.reconnect = lambda: called.append("reconnect")  # type: ignore[method-assign]
        assert d.recover(escalate_reconnect=False) is False
        assert called == []

    def test_reconnect_failure_returns_false(self) -> None:
        d = _driver_with_failing_ladder()

        def _boom() -> None:
            raise FanucConnectionError("reconnect could not reach the controller")

        d.reconnect = _boom  # type: ignore[method-assign]
        assert d.recover() is False


class TestReconnectGripperGate:
    """`reconnect()` holds the gripper fail-fast gate across the rebuild."""

    def _rig(self, events: list[str], *, bringup_raises: bool = False) -> FanucDriver:
        d = _bare_driver()
        d.core = SimpleNamespace(
            running=False,
            stop=lambda: events.append("core.stop"),
            stop_j=lambda: events.append("stop_j"),
        )
        d.rmi = SimpleNamespace(stop=lambda: events.append("rmi.stop"))

        def _bringup() -> str:
            events.append("bringup")
            if bringup_raises:
                raise FanucConnectionError("bringup failed")
            return "report"

        d._supervisor = SimpleNamespace(
            shutdown=lambda: events.append("shutdown"),
            bringup=_bringup,
            start_watch=lambda: events.append("start_watch"),
        )
        d.gripper = SimpleNamespace(set_recovery=lambda a: events.append(f"gate:{a}"))
        return d

    def test_gate_held_after_shutdown_and_across_bringup(self) -> None:
        events: list[str] = []
        self._rig(events).reconnect()
        # Gate is set True AFTER shutdown() (so a just-quiesced ladder's finally can't
        # clobber it) and BEFORE bringup(); released (False) after start_watch().
        assert events.index("shutdown") < events.index("gate:True") < events.index("bringup")
        assert events.index("gate:False") > events.index("start_watch")

    def test_gate_released_even_if_bringup_raises(self) -> None:
        events: list[str] = []
        d = self._rig(events, bringup_raises=True)
        with pytest.raises(FanucConnectionError):
            d.reconnect()
        assert "gate:True" in events
        assert events[-1] == "gate:False"  # released last, in the finally
