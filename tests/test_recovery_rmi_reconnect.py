"""Recovery-ladder full-RMI-reconnect tier (a SystemFault that wedges RMI).

A ``FRC_SystemFault`` can leave RMI unresponsive, so ``FRC_Reset`` / ``FRC_Call``
time out (``RmiSessionDown``). The recovery ladder used to hard-``return False``
there and freeze the arm. These tests cover the new escalation: on
``RmiSessionDown`` the ladder falls back to a bounded full RMI reconnect (the
bring-up "flush stale SystemFault" sequence — ``rmi.stop`` → reconnect → reset →
reseed → ``FRC_Call``) and retries, gated by
``DriverPolicy.recovery_reconnect_attempts``.

Unit-level by design: the socket-based ``FakeCRXController`` can't easily model
a wedged transport, so these drive the new ``Supervisor`` logic directly against
a fake RMI client (``__init__`` bypassed, only the reconnect tier's collaborators
wired up).
"""

from __future__ import annotations

import threading
from types import SimpleNamespace

from airo_fanuc.exceptions import RmiSessionDown
from airo_fanuc.supervisor import FaultReason, Supervisor, _STREAM_MOTN


class _FakeRmi:
    """Minimal RMI client: records the call sequence and can be told to raise
    ``RmiSessionDown`` on the first N ``program_call()``s (a wedged session)."""

    def __init__(self, *, program_call_fails: int = 0):
        self._program_call_fails = program_call_fails
        self.calls: list[str] = []
        self.stop_count = 0
        self.start_count = 0
        self.program_call_count = 0

    def stop(self) -> None:
        self.stop_count += 1
        self.calls.append("stop")

    def start(self) -> None:  # used by _reconnect_rmi_tolerating_already_connected
        self.start_count += 1
        self.calls.append("start")

    def reset(self) -> None:
        self.calls.append("reset")

    def reseed_sequence_id_from_controller(self) -> None:
        self.calls.append("reseed")

    def program_call(self, prog: str) -> None:
        self.program_call_count += 1
        self.calls.append(f"call:{prog}")
        if self.program_call_count <= self._program_call_fails:
            raise RmiSessionDown("simulated session down")

    # no program_continue → _maybe_continue is a no-op


def _bare_supervisor(rmi, *, reconnect_attempts: int) -> Supervisor:
    """Supervisor with ``__init__`` bypassed, wired with only what the reconnect
    tier touches. ``_abort_recovery`` is shadowed to never abort."""
    sup = Supervisor.__new__(Supervisor)
    sup._policy = SimpleNamespace(recovery_reconnect_attempts=reconnect_attempts)
    sup._rmi = rmi
    sup._abort_recovery = lambda: False
    return sup


class TestRelaunchViaReconnect:
    def test_reconnects_and_relaunches(self):
        rmi = _FakeRmi(program_call_fails=0)
        sup = _bare_supervisor(rmi, reconnect_attempts=1)
        assert sup._relaunch_stream_motn_via_reconnect() is True
        # Flush sequence on a fresh session: stop → (re)start → … → FRC_Call.
        assert rmi.stop_count == 1
        assert rmi.start_count >= 1
        assert f"call:{_STREAM_MOTN}" in rmi.calls
        assert rmi.calls.index("stop") < rmi.calls.index(f"call:{_STREAM_MOTN}")

    def test_zero_budget_is_the_old_bail(self):
        rmi = _FakeRmi()
        sup = _bare_supervisor(rmi, reconnect_attempts=0)
        assert sup._relaunch_stream_motn_via_reconnect() is False
        assert rmi.stop_count == 0 and rmi.start_count == 0  # no RMI churn

    def test_exhausts_budget_and_returns_false(self):
        rmi = _FakeRmi(program_call_fails=1)  # the single attempt still wedges
        sup = _bare_supervisor(rmi, reconnect_attempts=1)
        assert sup._relaunch_stream_motn_via_reconnect() is False
        assert rmi.stop_count == 1  # tried exactly once

    def test_retries_within_budget(self):
        rmi = _FakeRmi(program_call_fails=1)  # 1st reconnect fails, 2nd succeeds
        sup = _bare_supervisor(rmi, reconnect_attempts=2)
        assert sup._relaunch_stream_motn_via_reconnect() is True
        assert rmi.stop_count == 2


def _recovery_supervisor(rmi, *, reconnect_attempts: int) -> Supervisor:
    """A bare Supervisor able to run the whole ``_recovery_body`` — the tail
    (wait motion_possible → core.recover → wait HOLD → success) is shadowed so
    the test isolates the RMI-relaunch escalation."""
    sup = Supervisor.__new__(Supervisor)
    sup._policy = SimpleNamespace(
        recovery_reconnect_attempts=reconnect_attempts,
        ready_wait_s=1.0,
        recovery_motion_probe_s=1.0,
        arm_gate=True,
    )
    sup._rmi = rmi
    sup._core = SimpleNamespace(recover=lambda: None)
    sup._lock = threading.RLock()
    sup._operator_required = False
    sup._fault_reason = FaultReason.CONTACT_STOP  # not arm-class → no MOTION_INHIBITED
    sup._recovery_count = 0
    sup._motion_inhibited = False
    sup._fault_since_mono = 1.0
    sup._last_recovery_mono = 0.0
    # Shadow the tail + guards so only the RMI ladder + escalation are exercised.
    sup._snap = lambda: {"e_stopped": False, "tp_enabled": False, "motion_possible": True}
    sup._abort_recovery = lambda: False
    sup._wait = lambda pred, t: True
    sup._wait_mode = lambda mode, t: True
    sup._set_state_locked = lambda *a, **k: None
    return sup


class TestRecoveryBodyEscalation:
    def test_no_reconnect_when_ladder_succeeds(self):
        rmi = _FakeRmi(program_call_fails=0)
        sup = _recovery_supervisor(rmi, reconnect_attempts=1)
        assert sup._recovery_body(explicit=True, timeout_s=1.0) is True
        assert rmi.stop_count == 0  # clean ladder → never reconnects

    def test_escalates_to_reconnect_on_session_down(self):
        # The normal ladder's FRC_Call wedges (RmiSessionDown); the reconnect
        # flush clears it and relaunches STREAM_MOTN → recovery succeeds.
        rmi = _FakeRmi(program_call_fails=1)
        sup = _recovery_supervisor(rmi, reconnect_attempts=1)
        assert sup._recovery_body(explicit=True, timeout_s=1.0) is True
        assert rmi.stop_count == 1
        # normal FRC_Call (failed) + reconnect FRC_Call (ok).
        assert rmi.calls.count(f"call:{_STREAM_MOTN}") == 2

    def test_faults_out_when_reconnect_exhausted(self):
        rmi = _FakeRmi(program_call_fails=99)  # every FRC_Call wedges
        sup = _recovery_supervisor(rmi, reconnect_attempts=1)
        assert sup._recovery_body(explicit=True, timeout_s=1.0) is False

    def test_returns_false_fast_when_motion_never_rearms(self):
        # The light ladder's FRC_Call succeeds (no RmiSessionDown), but motion_possible
        # never re-asserts (the FRC_SystemFault de-init that only a cold reconnect
        # clears). The short motion probe must FAIL THE TIER and return False so the
        # driver can escalate — NOT stall on the HOLD wait.
        rmi = _FakeRmi(program_call_fails=0)
        sup = _recovery_supervisor(rmi, reconnect_attempts=1)
        sup._wait = lambda pred, t: False  # motion_possible probe never satisfied
        assert sup._recovery_body(explicit=True, timeout_s=1.0) is False
        assert rmi.stop_count == 0  # did not churn into the RmiSessionDown reconnect tier
