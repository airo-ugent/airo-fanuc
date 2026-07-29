# SPDX-License-Identifier: Apache-2.0
"""Lifecycle-supervisor policy tests against the realtime FakeCRX.

The C++ core does the ≤8 ms mechanical gating/kill/SAFE_FOLLOW autonomously; these
tests exercise the POLICY the :class:`~airo_fanuc.supervisor.Supervisor` layers on
top: fault→lifecycle-state mapping, auto-recovery + TEACH→AUTO self-heal, the ARM
gate, the SYST-348 OPERATOR_REQUIRED flow, preflight hard-blocks, ownership, and
the SUPERVISOR_LOST invariant (the core's safety reaction never depends on the
Python supervisor being alive).

Reuses :class:`test_driver.DriverRig` (pytest prepends the tests dir to sys.path).
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

import pytest
from test_driver import _READY_TIMEOUT_S, DriverRig, _traj_from

from airo_fanuc import (
    DriverConfig,
    DriverPolicy,
    FanucDriver,
    FanucPreflightError,
    OwnershipError,
    RobotFaultedError,
)
from airo_fanuc._core import Mode
from airo_fanuc.lifecycle import OPERATOR_REQUIRED_HINT
from airo_fanuc.testing import FakeCRXConfig, FakeCRXController


def _wait(pred: Callable[[], bool], timeout_s: float = 3.0, interval_s: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(interval_s)
    return pred()


def _state(rig: DriverRig) -> str:
    return str(rig.driver.get_state()["lifecycle_state"])


def _fault(rig: DriverRig) -> str:
    return str(rig.driver.get_state()["fault_reason"])


# --------------------------------------------------------------------------- #
# Fault → lifecycle-state mapping (auto-recovery OFF for determinism).
# --------------------------------------------------------------------------- #


def _manual_rig(tmp_path: Any) -> DriverRig:
    return DriverRig(tmp_path, policy_overrides={"auto_recover": False})


def test_estop_drives_faulted(tmp_path: Any) -> None:
    rig = _manual_rig(tmp_path)
    try:
        rig.controller.press_estop()
        assert _wait(lambda: _state(rig) == "faulted", 2.0)
        assert _fault(rig) in ("e_stop", "in_error")
        assert rig.driver.get_state()["operator_hint"] is not None
    finally:
        rig.controller.release_estop()
        rig.close()


def test_teach_drives_degraded_and_states_flow(tmp_path: Any) -> None:
    rig = _manual_rig(tmp_path)
    try:
        rig.controller.set_teach(True)
        assert _wait(lambda: _state(rig) == "degraded", 2.0)
        assert _fault(rig) == "teach_mode"
        # States keep flowing in TEACH (T1-freeze fix): RX stays fresh.
        assert rig.driver.get_state()["rx_fresh"] is True
        assert rig.driver.get_state()["tp_enabled"] is True
    finally:
        rig.controller.set_teach(False)
        rig.close()


def test_contact_stop_drives_degraded(tmp_path: Any) -> None:
    rig = _manual_rig(tmp_path)
    try:
        rig.controller.set_contact_stop(2)
        assert _wait(lambda: _state(rig) == "degraded", 2.0)
        assert _fault(rig) == "contact_stop"
    finally:
        rig.controller.set_contact_stop(0)
        rig.close()


def test_motion_possible_drop_drives_faulted(tmp_path: Any) -> None:
    rig = _manual_rig(tmp_path)
    try:
        rig.controller.drop_motion_possible()
        assert _wait(lambda: _state(rig) == "faulted", 2.0)
        assert _fault(rig) == "motion_not_possible"
    finally:
        rig.controller.restore_motion_possible()
        rig.close()


def test_system_fault_and_in_error_drives_faulted(tmp_path: Any) -> None:
    rig = _manual_rig(tmp_path)
    try:
        # FRC_SystemFault is an RMI-plane async push; the accompanying SM-plane
        # in_error drives the core to SAFE_FOLLOW → the supervisor maps FAULTED.
        assert rig.controller.push_system_fault(event_id=7) is True
        rig.controller.inject_alarm("SRVO-050", "Collision detect alarm", in_error=True)
        assert _wait(lambda: _state(rig) == "faulted", 2.0)
        assert _fault(rig) == "in_error"
    finally:
        rig.close()


# --------------------------------------------------------------------------- #
# RECOVERING safety: recover() bails on a held e-stop — an e-stop is a human decision,
# so the ladder must not try to clear it out from under the operator.
# --------------------------------------------------------------------------- #


def test_recover_bails_on_held_estop(tmp_path: Any) -> None:
    rig = _manual_rig(tmp_path)
    try:
        rig.controller.press_estop()
        assert _wait(lambda: _state(rig) == "faulted", 2.0)
        # A held e-stop cannot be recovered — recover() refuses without clearing.
        assert rig.driver.recover(timeout_s=1.0) is False
        assert _state(rig) == "faulted"
    finally:
        rig.controller.release_estop()
        rig.close()


# --------------------------------------------------------------------------- #
# Auto-recovery + TEACH→AUTO self-heal.
# --------------------------------------------------------------------------- #


def test_teach_to_auto_self_heal(tmp_path: Any) -> None:
    rig = DriverRig(tmp_path)  # auto_recover ON (default)
    try:
        rig.controller.set_teach(True)
        assert _wait(lambda: _state(rig) == "degraded", 2.0)
        # Return to AUTO — the TEACH→AUTO edge clears the cooldown; auto-recovery
        # fires immediately and the core leaves SAFE_FOLLOW after its dwell.
        rig.controller.set_teach(False)
        assert _wait(lambda: _state(rig) == "streaming", 4.0), _state(rig)
        # TEACH is not an arm-class fault → no MOTION_INHIBITED; motion resumes.
        assert rig.driver.get_state()["motion_inhibited"] is False
    finally:
        rig.close()


def test_auto_recovery_of_transient_motion_possible_drop(tmp_path: Any) -> None:
    rig = DriverRig(tmp_path)
    try:
        rig.controller.drop_motion_possible()
        assert _wait(lambda: _state(rig) == "faulted", 2.0)
        rig.controller.restore_motion_possible()
        # Auto-recovery ladder (reset → reseed → FRC_Call → core.recover) brings it
        # back to STREAMING; motion_not_possible is not arm-class.
        assert _wait(lambda: _state(rig) == "streaming", 5.0), _state(rig)
        assert rig.driver.get_state()["motion_inhibited"] is False
    finally:
        rig.close()


# --------------------------------------------------------------------------- #
# SYST-348 OPERATOR_REQUIRED flow: FRC_Reset cannot clear a payload-monitor alarm —
# only a payload confirm at the teach pendant can — so the ladder must stop retrying
# and hand the fault to a human instead of looping.
# --------------------------------------------------------------------------- #


def test_syst348_operator_required_then_arm(tmp_path: Any) -> None:
    rig = DriverRig(tmp_path)  # auto_recover ON
    try:
        # Arm the recovery FRC_Reset to fail with SYST-348, then trip a fault so
        # the auto-recovery ladder hits it.
        rig.controller.arm_syst_348(on="FRC_Reset")
        rig.controller.drop_motion_possible()

        # Auto-recovery reset → 2556936 ∧ SYST-348 → OPERATOR_REQUIRED (no retries).
        assert _wait(lambda: rig.driver.get_state()["operator_required"] is True, 3.0)
        assert rig.driver.get_state()["operator_hint"] == OPERATOR_REQUIRED_HINT
        assert rig.driver.get_state()["motion_inhibited"] is True

        # Operator confirms payload (clears SYST-348) + motion_possible restored.
        with rig.controller.state.lock:
            rig.controller.state.alarm_texts.clear()
        rig.controller.restore_motion_possible()

        # The 1 Hz ReadError watch detects the clear → recovery resumes → STREAMING,
        # but MOTION_INHIBITED persists (payload-confirm still requires arm()).
        assert _wait(lambda: rig.driver.get_state()["operator_required"] is False, 4.0)
        assert _wait(lambda: _state(rig) == "streaming", 5.0), _state(rig)
        assert rig.driver.get_state()["motion_inhibited"] is True

        # Move before arm() rejects; after arm() it executes.
        times, q, qd = _traj_from(rig.driver, 0.1)
        with pytest.raises(RobotFaultedError):
            rig.driver.move_trajectory(times, q, qd)
        rig.driver.arm()
        times, q, qd = _traj_from(rig.driver, 0.1)
        assert rig.driver.move_trajectory(times, q, qd).wait(timeout=4.0).value == "done"
    finally:
        rig.close()


# --------------------------------------------------------------------------- #
# Gripper fail-fast gate during recovery.
# --------------------------------------------------------------------------- #


def test_gripper_fail_fast_gated_during_recovery_ladder(tmp_path: Any) -> None:
    """While a recovery ladder runs, a gripper command must fast-reject (never actuate
    GRIPDISP mid-recovery — e.g. just after an e-stop release with the operator's hands
    at the pendant) and must work again once recovery completes.

    The gate is only as good as its callers: if ``GripperWorker.set_recovery`` is not
    invoked on the real ladder path the gate is inert and a ``close_gripper()`` during
    recovery EXECUTES, which is why this drives the ladder end-to-end rather than
    poking the flag."""
    from test_gripper_worker import FakeGripperRmi

    from airo_fanuc.gripper_worker import GripperWorker

    rig = _manual_rig(tmp_path)  # auto_recover OFF — we drive recovery explicitly
    probe_rmi = FakeGripperRmi(clears_after=1)
    probe = GripperWorker(probe_rmi, dispatch_timeout_s=1.0)
    rig.driver._supervisor.set_gripper(probe)  # noqa: SLF001 - register the probe gripper
    try:
        # Trip a (non-arm-class) fault; do NOT restore motion_possible yet.
        rig.controller.drop_motion_possible()
        assert _wait(lambda: _state(rig) == "faulted", 2.0)

        # Run the ladder in the background. With motion_possible still dropped, the
        # body blocks in _wait(motion_possible, ready_wait=30 s) — a wide, deterministic
        # window during which the gripper gate is engaged.
        rec_result: dict[str, bool] = {}

        def _run_recover() -> None:
            rec_result["ok"] = rig.driver.recover(timeout_s=30.0)

        t = threading.Thread(target=_run_recover, daemon=True)
        t.start()
        try:
            assert _wait(lambda: probe._is_recovering(), 3.0)  # noqa: SLF001 - gate engaged

            # A gripper command DURING recovery fast-rejects and never touches RMI.
            during = probe.close_gripper_and_wait(timeout=2.0)
            assert during is not None and during["success"] is False
            assert "recovery" in str(during["message"]).lower()
            assert probe_rmi.write_log == [], "gripper wrote registers during recovery"

            rig.controller.restore_motion_possible()  # let the ladder proceed
        finally:
            t.join(timeout=10.0)
        assert not t.is_alive()
        assert rec_result.get("ok") is True
        assert _wait(lambda: _state(rig) == "streaming", 5.0), _state(rig)

        # Gate cleared in the ladder's finally → the gripper works again.
        assert _wait(lambda: not probe._is_recovering(), 2.0)  # noqa: SLF001
        after = probe.close_gripper_and_wait(timeout=2.0)
        assert after is not None and after["success"] is True
        assert probe_rmi.write_log, "gripper did not write registers after recovery"
    finally:
        probe.close()
        rig.close()


# --------------------------------------------------------------------------- #
# Preflight hard block (SYST-348 present at bring-up).
# --------------------------------------------------------------------------- #


def test_preflight_hard_block_raises(tmp_path: Any) -> None:
    controller = FakeCRXController(FakeCRXConfig(strict=True))
    controller.start()
    controller.start_realtime(speed=1.0)
    controller.arm_syst_348(on="FRC_Initialize")  # also raises the SYST-348 alarm text
    cfg = DriverConfig(
        sm_port=controller.sm_port, rmi_port=controller.rmi_port, preroll_timeout_s=_READY_TIMEOUT_S
    )
    policy = DriverPolicy(config=cfg, connect_retries=2, lock_path=str(tmp_path / "owner.lock"))
    try:
        with pytest.raises(FanucPreflightError) as ei:
            FanucDriver("127.0.0.1", policy)
        assert "SYST-348" in str(ei.value)
    finally:
        controller.stop_realtime()
        controller.close()


# --------------------------------------------------------------------------- #
# Ownership — a second control instance fails loudly rather than sharing the arm.
# --------------------------------------------------------------------------- #


def test_ownership_conflict_raises(tmp_path: Any) -> None:
    lock_path = str(tmp_path / "owner.lock")
    rig1 = DriverRig(tmp_path)  # default lock is tmp_path/owner.lock == lock_path
    controller2 = FakeCRXController(FakeCRXConfig(strict=True))
    controller2.start()
    controller2.start_realtime(speed=1.0)
    cfg = DriverConfig(
        sm_port=controller2.sm_port, rmi_port=controller2.rmi_port, preroll_timeout_s=_READY_TIMEOUT_S
    )
    policy = DriverPolicy(config=cfg, connect_retries=1, lock_path=lock_path)
    try:
        with pytest.raises(OwnershipError):
            FanucDriver("127.0.0.1", policy)
    finally:
        controller2.stop_realtime()
        controller2.close()
        rig1.close()


# --------------------------------------------------------------------------- #
# GRPRUN anti-stacking: the GRIPDISP RUN-fork fires at most once per bringup()
# across all retries (measured on hardware 2026-07-07 — a RUN-forked task cannot be
# killed by FRC_Reset/FRC_Abort, and _teardown_partial does not abort it, so a
# per-attempt re-fork stacks un-killable GRIPDISP tasks that wedge STREAM_MOTN at
# program_status=2 until a power-cycle).
# --------------------------------------------------------------------------- #


def test_grprun_fork_at_most_once_across_failed_retries(tmp_path: Any) -> None:
    """A gripper-enabled bring-up that fails every attempt must fork GRPRUN AT MOST
    ONCE (not once per attempt). A _bringup_once that re-forks on every retry stacks one
    un-killable GRIPDISP task per attempt, so a 3-attempt flaky bring-up leaves 3.

    We drive the supervisor directly (no FanucDriver) so we can construct a bring-up
    that reaches the GRPRUN fork (TP_LAUNCH) on every attempt but then fails at the
    SM preroll: the FakeCRX RMI plane is up (so STREAM_MOTN launches + GRPRUN forks),
    but its 125 Hz Stream-Motion loop is deliberately NOT started, so motion_possible
    never asserts and wait_ready times out — a retryable FanucConnectionError.
    """
    from airo_fanuc import DriverConfig, DriverPolicy, FanucConnectionError
    from airo_fanuc._core import StreamCore
    from airo_fanuc.rmi_client import RmiClient
    from airo_fanuc.supervisor import Supervisor

    controller = FakeCRXController(FakeCRXConfig(strict=True))
    controller.start()  # RMI + SM sockets bound; NOTE: start_realtime() intentionally NOT called
    cfg = DriverConfig(
        sm_port=controller.sm_port, rmi_port=controller.rmi_port,
        preroll_timeout_s=0.3, gripdisp_probe_timeout_s=0.3,
    )
    policy = DriverPolicy(config=cfg, connect_retries=3, enable_gripper=True, hold_wait_s=0.3)
    rmi = RmiClient(
        "127.0.0.1",
        controller.rmi_port,
        connect_timeout=cfg.rmi_connect_timeout,
        request_timeout=cfg.rmi_request_timeout,
    )
    core = StreamCore("127.0.0.1", controller.sm_port, cfg.to_rt_core_config())
    sup = Supervisor(core, rmi, policy)
    try:
        with pytest.raises(FanucConnectionError):
            sup.bringup()
        # 3 failed attempts, at-most-one fork (a per-attempt fork would give 3).
        assert controller.rmi.grprun_call_count == 1
    finally:
        sup.shutdown()
        try:
            core.stop()
        finally:
            try:
                rmi.stop()
            finally:
                controller.close()


def test_successful_gripper_bringup_forks_grprun_exactly_once(tmp_path: Any) -> None:
    """The healthy path (one successful gripper-enabled bring-up) still forks once."""
    rig = DriverRig(tmp_path)  # enable_gripper=True by default, connect_retries=1
    try:
        assert _state(rig) == "streaming"
        assert rig.controller.rmi.grprun_call_count == 1
    finally:
        rig.close()


def test_no_gripper_bringup_never_forks_grprun(tmp_path: Any) -> None:
    """A no-gripper session (calibration bare-move / --no-move) must never fork GRPRUN."""
    rig = DriverRig(tmp_path, policy_overrides={"enable_gripper": False})
    try:
        assert _state(rig) == "streaming"
        assert rig.controller.rmi.grprun_call_count == 0
    finally:
        rig.close()


def test_gripper_bringup_skips_fork_when_gripdisp_already_running(tmp_path: Any) -> None:
    """CROSS-PROCESS anti-stacking: if a GRIPDISP dispatcher is ALREADY running (a
    prior process's surviving RUN-fork), the bring-up's liveliness probe detects it
    (REG_CMD auto-clears) and does NOT fork a second GRPRUN.

    Guards the stacking wedge that repeated driver restarts otherwise create: the
    per-process _grprun_forked latch cannot see a fork inherited from another process,
    so without the probe each restart would stack an un-killable GRIPDISP task until
    STREAM_MOTN wedges (program_status=2).
    """
    from airo_fanuc import DriverConfig, DriverPolicy
    from airo_fanuc._core import StreamCore
    from airo_fanuc.rmi_client import RmiClient
    from airo_fanuc.supervisor import Supervisor

    controller = FakeCRXController(FakeCRXConfig(strict=True))
    controller.start()
    controller.start_realtime(speed=1.0)  # preroll must succeed on the healthy path
    # Simulate a prior process's surviving RUN-fork: GRIPDISP is already running
    # BEFORE this driver connects (Connect/Initialize/Reset don't clear it).
    controller.rmi._gripdisp_running = True  # noqa: SLF001 - test-only state injection
    cfg = DriverConfig(
        sm_port=controller.sm_port, rmi_port=controller.rmi_port,
        preroll_timeout_s=0.3, gripdisp_probe_timeout_s=0.3,
    )
    policy = DriverPolicy(config=cfg, connect_retries=1, enable_gripper=True, hold_wait_s=0.3)
    rmi = RmiClient(
        "127.0.0.1", controller.rmi_port,
        connect_timeout=cfg.rmi_connect_timeout, request_timeout=cfg.rmi_request_timeout,
    )
    core = StreamCore("127.0.0.1", controller.sm_port, cfg.to_rt_core_config())
    sup = Supervisor(core, rmi, policy)
    try:
        sup.bringup()  # healthy bring-up; must not fork over the live dispatcher
        assert controller.rmi.grprun_call_count == 0
    finally:
        sup.shutdown()
        try:
            core.stop()
        finally:
            try:
                rmi.stop()
            finally:
                controller.stop_realtime()
                controller.close()


# --------------------------------------------------------------------------- #
# SUPERVISOR_LOST invariant: the RT core is autonomous of the Python thread.
# --------------------------------------------------------------------------- #


def test_supervisor_lost_core_stays_autonomous(tmp_path: Any) -> None:
    rig = DriverRig(tmp_path)
    try:
        assert rig.driver.get_state()["mode"] == int(Mode.HOLD)
        # Simulate the supervisor thread dying: stop the watch loop entirely.
        rig.driver._supervisor.shutdown()  # noqa: SLF001

        # The C++ core keeps its 125 Hz loop with NO Python supervision: it holds,
        # and its autonomous gate still faults to SAFE_FOLLOW on an e-stop —
        # proving the safety reaction never depended on the Python thread.
        assert rig.driver.core.get_snapshot()["mode"] == int(Mode.HOLD)
        rig.controller.press_estop()
        assert _wait(lambda: rig.driver.core.get_snapshot()["mode"] == int(Mode.SAFE_FOLLOW), 2.0)
        # Getters still never raise even with the supervisor gone.
        assert rig.driver.get_state()["e_stopped"] is True
    finally:
        rig.controller.release_estop()
        rig.close()


# --------------------------------------------------------------------------- #
# shutdown() quiesces an in-flight recovery ladder (race-free teardown).
# --------------------------------------------------------------------------- #


def _quiesce_supervisor(recovering: bool) -> Any:
    """A bare Supervisor exercising only shutdown()'s recovery-quiesce wait."""
    from airo_fanuc.lifecycle import LifecycleState
    from airo_fanuc.supervisor import FaultReason, Supervisor

    sup = Supervisor.__new__(Supervisor)
    sup._lock = threading.RLock()
    sup._stop_evt = threading.Event()
    sup._thread = None
    sup._hb_thread = None
    sup._recovering = recovering
    sup._state = LifecycleState.STREAMING
    sup._fault_reason = FaultReason.NONE
    sup._set_state_locked = lambda *a, **k: None  # avoid the hint/log machinery
    return sup


def test_shutdown_waits_out_in_flight_recovery() -> None:
    """shutdown() must not return while a recovery ladder is still using rmi/core."""
    sup = _quiesce_supervisor(recovering=True)

    def _clear_soon() -> None:
        time.sleep(0.15)
        sup._recovering = False

    threading.Thread(target=_clear_soon, daemon=True).start()
    t0 = time.monotonic()
    sup.shutdown(timeout_s=2.0)
    waited = time.monotonic() - t0
    assert sup._recovering is False
    assert waited >= 0.12  # blocked until the ladder cleared, not returned immediately


def test_shutdown_is_bounded_when_recovery_never_clears() -> None:
    """A stuck ladder must not hang shutdown() forever — the wait is bounded."""
    sup = _quiesce_supervisor(recovering=True)  # never cleared
    t0 = time.monotonic()
    sup.shutdown(timeout_s=0.3)
    waited = time.monotonic() - t0
    assert 0.3 <= waited < 1.5  # returned near the bound, did not hang
