# SPDX-License-Identifier: Apache-2.0
"""Lifecycle supervisor — the POLICY layer above the autonomous C++ core.

The C++ ``StreamCore`` does the ≤8 ms mechanical gating, kill, and SAFE_FOLLOW
re-anchor *autonomously*: it never waits for Python, so a slow or dead supervisor
can delay recovery but can never delay a stop. The :class:`Supervisor` adds only
the policy on top of that core:

* the **bring-up ladder** (RMI connect+initialize → FRC_Call(GRPRUN) → full RMI
  reconnect → FRC_Call(STREAM_MOTN) → ``StreamCore.start`` + ``wait_ready`` →
  poll HOLD), with 3 retries and a 4-branch startup triage;
* the **recovery ladders** (reset → best-effort FRC_Continue → seq reseed → FRC_Call
  → wait motion_possible → ``core.recover()``), single-flight, cooldown-gated;
* the **ARM gate**: after an e-stop or an operator-required fault, recovery ends in
  MOTION_INHIBITED and the next motion needs an explicit ``arm()``. Auto-arming
  would move the robot the instant the fault cleared — with the operator still
  standing at the pendant that just cleared it — so the decision to resume motion
  is handed back to the caller rather than taken here;
* the **SYST-348 OPERATOR_REQUIRED** flow (detect 2556936 ∧ SYST-348 → stop
  retrying → 1 Hz ReadError watch on the commands-only session → on clear, still
  require ``arm()``);
* **TEACH→AUTO self-heal** (clear the cooldown on the tp_enabled True→False edge);
* **skip recovery while tp_enabled** without arming the cooldown, so the
  TEACH→AUTO edge re-fires recovery immediately instead of waiting one out.

The anti-flap dwell (500 ms all-clear) and the mode re-anchor are owned by the
C++ ``recover()`` (the FakeCRX integration test
``test_contact_flap_storm_holds_dwell_no_premature_recovery`` covers it); the
supervisor just calls ``core.recover()`` once the RMI-side fault is cleared.

**SUPERVISOR_LOST invariant:** the RT core is independent of this thread. If the
watch loop dies, the core keeps its 125 Hz loop and stays in HOLD; the autonomous
gates still fault it to SAFE_FOLLOW on an e-stop. ``tests/test_supervisor.py``
asserts this.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, cast

from ._core import FaultReason, Mode, StreamCore
from .config import DriverPolicy
from .exceptions import FanucConnectionError, FanucPreflightError, RmiError, RmiSessionDown
from .gripper import ACTION_OPEN, OPEN_FULL, REG_ACTION, REG_CMD, REG_R3
from .lifecycle import (
    LifecycleState,
    classify,
    fault_reason_string,
    operator_hint_for,
    requires_arm,
)
from .preflight import PreflightReport, run_preflight
from .rmi_client import RmiClient

if TYPE_CHECKING:
    from .driver import MotionHandle
    from .gripper_worker import GripperWorker

logger = logging.getLogger("airo_fanuc.supervisor")

# SYST-348 payload-monitor family ("Cannot Execute TP program.").
_SYST348_ERROR_IDS: frozenset[int] = frozenset({2556934, 2556936})
_ERR_TP_NOT_PAUSED = 2556938  # FRC_Continue of an unpaused program — benign, tolerated.
_ERR_ALREADY_CONNECTED = 2556954  # "Robot is Already Connected." — prior session not yet freed.

_GRPRUN = "GRPRUN"
_STREAM_MOTN = "STREAM_MOTN"

# Times bring-up re-applies the STREAM_MOTN relaunch when motion_possible will not stay
# asserted (see _settle_stream_motn). Two: one drop is the expected re-FRC_Call transient,
# a second means something other than the handover is dropping it, and re-Calling harder
# is not the answer — RMI churn is itself an SM-daemon wedge vector (controller-notes §2.5).
_SETTLE_ATTEMPTS = 2

# TPMode values that mean AUTO. The standard FANUC AUTO code is 2, but this
# SOP-less CRX reports 0 in its (permanent) AUTO — measured on the controller
# (docs/controller-notes.md §1.6). Both are AUTO here; only 1/3 are genuine T1/T2.
_TP_MODES_AUTO = (0, 2)
_SYST348_WATCH_PERIOD_S = 1.0

# Supervisor-liveness heartbeat (SUPERVISOR_LOST). A dedicated lightweight thread
# beats the C++ RT core at this cadence; the core faults+holds if beats lapse for
# controller_facts.SUPERVISOR_LOST_S. Kept well below that threshold so GIL-storm /
# GC jitter never starves it into a false trip.
_HEARTBEAT_INTERVAL_S = 0.1


class Supervisor:
    """Owns the lifecycle state machine, RMI ladders, ARM gate and recovery policy.

    The driver constructs the :class:`StreamCore` and :class:`RmiClient`, then
    hands them here; the supervisor drives them but does NOT own their teardown
    (that is the driver's ordered ``close()``).
    """

    def __init__(self, core: StreamCore, rmi: RmiClient, policy: DriverPolicy) -> None:
        self._core = core
        self._rmi = rmi
        self._policy = policy
        self._cfg = policy.config

        # -- lifecycle state (guarded by _lock) --------------------------
        self._lock = threading.RLock()
        self._state = LifecycleState.DISCONNECTED
        self._fault_reason: FaultReason = FaultReason.NONE
        self._fault_reason_str = "none"
        self._operator_hint: str | None = None
        self._operator_required = False
        self._motion_inhibited = False
        self._recovery_count = 0
        self._epoch = 0
        self._preflight_report: PreflightReport | None = None

        # Anti-stacking latch for the GRPRUN RUN-fork (see _bringup_once). Once set,
        # GRIPDISP is known-present for the rest of this bringup() — either because we
        # forked it, OR because the cross-process liveliness probe found one already
        # running (a prior process's surviving RUN-fork). Reset at the top of each
        # bringup() so the probe+fork fires at most once across ALL of that call's
        # retries; a fresh bringup() (the reconnect() path) re-arms it.
        self._grprun_forked = False

        # -- recovery bookkeeping ----------------------------------------
        self._recovery_lock = threading.Lock()
        self._recovering = False
        self._last_recovery_mono = -math.inf
        self._fault_since_mono: float | None = None
        self._prev_tp_enabled: bool | None = None
        self._last_syst348_watch_mono = -math.inf

        # -- watch thread + supervisor-liveness heartbeat thread ---------
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None
        self._hb_thread: threading.Thread | None = None

        # -- active motion handle (for trajectory_start_mono_ns latch) ---
        self._active_handle: MotionHandle | None = None

        # -- gripper fail-fast gate ---------------------------------------
        # Registered post-construction by the driver (the gripper is built after
        # the supervisor). While a recovery ladder runs, the supervisor toggles the
        # gripper's fail-fast gate so a concurrent gripper command is rejected
        # instead of actuating GRIPDISP mid-recovery (e.g. right after an e-stop
        # release with the operator's hands at the pendant). None-safe: no gripper
        # registered (or gripper disabled) → no-op.
        self._gripper: GripperWorker | None = None

    # ==================================================================
    # Bring-up (synchronous; called from the FanucDriver constructor)
    # ==================================================================

    def bringup(self) -> PreflightReport:
        """Run the full bring-up ladder with retries. Returns the preflight report
        or raises :class:`FanucConnectionError` (with triage) / :class:`FanucPreflightError`."""
        # Clear the stop signal BEFORE any bring-up work. shutdown() sets _stop_evt
        # (and only start_watch() clears it — which runs AFTER bringup on the
        # reconnect() path); a still-set event makes the SM_HANDSHAKE _wait_mode(HOLD)
        # short-circuit, so a cold re-bring-up would raise "core did not publish HOLD
        # after preroll" on every retry. Harmless on the construct-time bringup (the
        # event is already clear).
        self._stop_evt.clear()
        # Arm the one-shot GRPRUN fork budget for this whole bringup() (all retries).
        self._grprun_forked = False
        last_exc: BaseException | None = None
        triage = ""
        for attempt in range(1, self._policy.connect_retries + 1):
            try:
                report = self._bringup_once()
                with self._lock:
                    self._preflight_report = report
                logger.info("airo_fanuc: bring-up complete (attempt %d) — %s", attempt, report.summary())
                return report
            except FanucPreflightError:
                # A hard preflight block (SYST-348 / SYST-322) will not clear on a
                # retry — surface it immediately with its report.
                self._teardown_partial()
                raise
            except (FanucConnectionError, RmiError, RmiSessionDown, OSError, TimeoutError) as exc:
                last_exc = exc
                triage = self._triage()
                logger.warning(
                    "airo_fanuc: bring-up attempt %d/%d failed: %s — triage: %s",
                    attempt,
                    self._policy.connect_retries,
                    exc,
                    triage,
                )
                self._teardown_partial()
        with self._lock:
            self._set_state_locked(LifecycleState.DISCONNECTED, FaultReason.NONE)
        raise FanucConnectionError(
            f"airo_fanuc: bring-up failed after {self._policy.connect_retries} attempt(s): "
            f"{last_exc} — triage: {triage}"
        )

    def _bringup_once(self) -> PreflightReport:
        # RMI_CONNECTING: Connect_STMO + redirect, preflight, Init ladder + reseed.
        self._set_state(LifecycleState.RMI_CONNECTING, FaultReason.NONE)
        self._rmi.start()  # raises FanucConnectionError if unreachable
        report = run_preflight(
            self._rmi,
            full=self._policy.preflight_full,
            profile=self._policy.config.profile,
            expect_gripper=self._policy.enable_gripper,
        )
        self._rmi.initialize()
        self._maybe_continue()

        # TP_LAUNCH: reset (never unconditional Abort) → [gripper only: GRPRUN fork →
        # 0.2 s → reset → FULL RMI reconnect to flush INTP-267 / stale SystemFault] →
        # seq reseed (FINAL before FRC_Call) → FRC_Call(STREAM_MOTN).
        #
        # STREAM_MOTN cannot be un-launched via RMI (FRC_Abort AND FRC_Reset both leave
        # program_status=2 — observed on hardware 2026-07-07), and a pure SM handshake to a
        # running instance does NOT re-arm motion_possible — so the FRC_Call is REQUIRED on
        # every bring-up, even over a running instance. Re-Calling causes a brief
        # motion_possible transient the supervisor auto-recovers from; it WEDGES
        # (unrecoverably) only on a controller already degraded by stacked GRIPDISP
        # RUN-forks — which the enable_gripper gate below prevents (a RUN-forked task
        # FRC_Reset can't kill, so never launch it on a no-gripper session: bare move /
        # calibration / --no-move).
        #
        # WHY A RUN-FORK AT ALL: FRC_Call(GRIPDISP) never returns. GRIPDISP is an infinite
        # TP loop polling R[1], so a blocking Call would own the RMI session forever and no
        # further command (including FRC_Call(STREAM_MOTN)) could be issued. GRPRUN is the
        # launcher: a TP wrapper that RUN-forks GRIPDISP as an independent task and returns
        # immediately. The price of the fork is that the task then outlives the RMI session
        # that started it — hence all the anti-stacking machinery below.
        #
        # ANTI-STACKING WITHIN ONE PROCESS: the enable_gripper gate stops no-gripper
        # sessions from EVER forking, but bringup() retries _bringup_once up to
        # connect_retries times and _teardown_partial does NOT abort/reset the GRPRUN task
        # (and a RUN-fork survives both FRC_Reset and FRC_Disconnect), so a naive re-fork
        # on each retry would stack up to connect_retries un-killable GRIPDISP forks and
        # re-trigger the same program_status=2 wedge on a merely flaky bring-up. The
        # _grprun_forked latch (armed once per bringup(), NOT per attempt) makes the fork
        # at-most-once across all retries; it is set the instant the fork dispatches — before
        # the settle/flush below — so even a flush failure that forces a retry cannot re-fork.
        # Skipping the whole block on later attempts is correct: the dispatcher is already
        # running, and the per-attempt Init ladder (initialize() above) + top-level reset
        # do the fault-clearing the fork's flush-reconnect otherwise piggybacked on, without
        # the extra RMI churn (itself a documented SM-daemon wedge vector — controller-notes §2.5).
        self._set_state(LifecycleState.TP_LAUNCH, FaultReason.NONE)
        self._rmi.reset()
        if self._policy.enable_gripper and not self._grprun_forked:
            # CROSS-PROCESS anti-stacking: the _grprun_forked latch alone only makes
            # the fork at-most-once WITHIN one process, which is not enough. A
            # RUN-forked GRIPDISP survives the process that forked it
            # (FRC_Reset/FRC_Disconnect can't kill it — only FRC_Abort / TP ABORT
            # ALL can), so every fresh driver bring-up blindly re-forking stacks
            # un-killable GRIPDISP tasks that wedge STREAM_MOTN at program_status=2 —
            # and the only way out of that wedge is an operator at the teach pendant
            # pressing FCTN → ABORT ALL. So gate the fork on a liveness probe: if a
            # dispatcher is already running, skip the fork entirely.
            if self._gripdisp_alive():
                logger.info(
                    "airo_fanuc: GRIPDISP already running (probe: R[%d] auto-cleared) — "
                    "skipping GRPRUN fork (cross-process anti-stacking)", REG_CMD)
                self._grprun_forked = True  # known-present; no re-probe/fork on retries
            else:
                logger.info(
                    "airo_fanuc: no GRIPDISP detected (probe: R[%d] not cleared in %.1fs) — "
                    "forking GRPRUN", REG_CMD, self._cfg.gripdisp_probe_timeout_s)
                self._rmi.program_call(_GRPRUN)
                self._grprun_forked = True  # latch BEFORE the flush: the fork is now irrevocable
                time.sleep(0.2)
                self._rmi.reset()
                self._rmi.stop()
                self._reconnect_rmi_tolerating_already_connected()
        self._rmi.reseed_sequence_id_from_controller()
        self._rmi.program_call(_STREAM_MOTN)

        # SM_HANDSHAKE: open the SM socket + preroll to motion_possible/HOLD.
        self._set_state(LifecycleState.SM_HANDSHAKE, FaultReason.NONE)
        if not self._core.start():
            raise FanucConnectionError("airo_fanuc: Stream Motion socket could not open")
        if not self._core.wait_ready(self._cfg.preroll_timeout_s):
            raise FanucConnectionError(
                "airo_fanuc: Stream Motion preroll timed out — motion_possible never asserted"
            )
        if not self._wait_mode(Mode.HOLD, self._policy.hold_wait_s):
            raise FanucConnectionError("airo_fanuc: core did not publish HOLD after preroll")
        # motion_possible asserting once is not the same as the robot being commandable:
        # the required re-FRC_Call above drops it ~1 s later. Settle before reporting
        # success, so the constructor cannot hand back a robot that is about to fault.
        if not self._settle_stream_motn():
            raise FanucConnectionError(
                "airo_fanuc: motion_possible would not stay asserted after the STREAM_MOTN "
                f"relaunch ({_SETTLE_ATTEMPTS} settle attempts) — see controller-notes.md §4.2"
            )

        self._set_state(LifecycleState.STREAMING, FaultReason.NONE)
        return report

    def _gripdisp_alive(self) -> bool:
        """Probe whether a GRIPDISP dispatcher is already running, WITHOUT forking one.

        Reuses the gripper command handshake as a liveliness token: GRIPDISP
        auto-clears REG_CMD (R[1]) to 0 only after it has handled a command, so a
        benign OPEN whose REG_CMD clears within ``gripdisp_probe_timeout_s`` proves a
        dispatcher (this or a prior process's surviving RUN-fork) is alive → the
        caller skips the fork. If REG_CMD never clears (no dispatcher, or a wedged
        one), the caller forks a fresh GRPRUN; the pending REG_CMD=1 this probe left
        is picked up when GRIPDISP starts, so the open still completes (a benign,
        usually-idempotent bring-up open — the desired clean start state).

        The window is deliberately generous (≥ the gripper dispatch timeout): a
        false "absent" verdict re-forks and re-creates the un-killable stacking
        wedge this probe exists to prevent, so err toward waiting. A live dispatcher
        returns as soon as REG_CMD clears (well under the ceiling). Any RMI error is
        treated as "absent" — a probe error on the first (unlatched) attempt costs at
        most one extra fork, bounded by the per-process ``_grprun_forked`` latch.
        """
        try:
            self._rmi.write_register(REG_R3, OPEN_FULL)
            self._rmi.write_register(REG_ACTION, ACTION_OPEN)
            self._rmi.write_register(REG_CMD, 1)
        except (RmiError, RmiSessionDown, OSError) as exc:
            logger.warning(
                "airo_fanuc: GRIPDISP liveliness probe write failed (%s) — treating as absent", exc)
            return False
        deadline = time.monotonic() + float(self._cfg.gripdisp_probe_timeout_s)
        while time.monotonic() < deadline:
            time.sleep(0.05)
            try:
                if self._rmi.read_register(REG_CMD) == 0:
                    return True
            except (RmiError, RmiSessionDown, OSError) as exc:
                logger.warning(
                    "airo_fanuc: GRIPDISP liveliness probe read failed (%s) — treating as absent", exc)
                return False
        return False

    def _reconnect_rmi_tolerating_already_connected(self, attempts: int = 12, delay_s: float = 0.05) -> None:
        """Re-open the RMI session (the TP_LAUNCH flush reconnect), tolerating a
        transient 2556954 while the controller frees the just-disconnected session
        (a real-controller teardown delay; on the FakeCRX the redirect thread
        clears ``_session_active`` a beat after the FRC_Disconnect reply)."""
        for i in range(attempts):
            try:
                self._rmi.start()
                return
            except RmiError as exc:
                if exc.error_id == _ERR_ALREADY_CONNECTED and i < attempts - 1:
                    time.sleep(delay_s)
                    continue
                raise

    def _triage(self) -> str:
        """4-branch hypothesis for why Stream Motion bring-up failed, so the raised
        :class:`FanucConnectionError` names the actual blocker instead of a timeout."""
        try:
            st = self._rmi.get_status()
            err = self._rmi.read_error(3)
        except (RmiError, RmiSessionDown, OSError) as exc:
            return f"RMI unreachable — network / controller down or foreign SM peer holds the session ({exc})"
        # This SOP-less CRX reports TPMode=0 in its (permanent) AUTO — NOT the
        # standard 2 (measured on the controller; docs/controller-notes.md §1.6).
        # Only a genuine T1/T2 (TPMode 1/3) means "return the keyswitch"; 0 and 2
        # are both AUTO here.
        if st.tp_mode not in _TP_MODES_AUTO:
            return f"teach pendant in T1/T2 (TPMode={st.tp_mode}) — return the keyswitch to AUTO"
        if not st.servo_ready:
            return f"drives off / e-stop / alarm — ReadError={list(err.messages)}"
        return (
            f"RMI healthy (AUTO, servo_ready) but motion_possible never asserted — likely a wedged "
            f"SM daemon / stale STREAM_MOTN (program_status={st.program_status}); POWER-CYCLE the "
            f"controller (docs/controller-notes.md §2.5). alarms={list(err.messages)}"
        )

    def _teardown_partial(self) -> None:
        """Best-effort teardown between bring-up attempts (core socket + RMI session)."""
        try:
            if self._core.running:
                self._core.stop()
        except Exception as exc:  # noqa: BLE001 - teardown is best-effort
            logger.debug("airo_fanuc: core stop during teardown failed: %s", exc)
        try:
            self._rmi.stop()
        except Exception as exc:  # noqa: BLE001
            logger.debug("airo_fanuc: rmi stop during teardown failed: %s", exc)

    # ==================================================================
    # Watch thread
    # ==================================================================

    def start_watch(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._watch_loop, name="airo-fanuc-supervisor", daemon=True)
        self._thread.start()
        # Dedicated liveness heartbeat (SUPERVISOR_LOST). Separate from the watch
        # loop so it never blocks on RMI — it reflects "the supervisor is alive",
        # not "RMI is fast". First beat arms the C++ watchdog.
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, name="airo-fanuc-heartbeat", daemon=True
        )
        self._hb_thread.start()

    def _heartbeat_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                self._core.heartbeat()
            except Exception as exc:  # noqa: BLE001 - liveness beat must never die on a transient
                logger.warning("airo_fanuc: supervisor heartbeat raised (continuing): %s", exc)
            self._stop_evt.wait(_HEARTBEAT_INTERVAL_S)

    def _watch_loop(self) -> None:
        interval = self._policy.watch_interval_s
        while not self._stop_evt.is_set():
            try:
                self._watch_tick()
            except Exception as exc:  # noqa: BLE001 - never let the loop die on a transient
                logger.warning("airo_fanuc: supervisor watch tick raised (continuing): %s", exc)
            self._stop_evt.wait(interval)

    def _watch_tick(self) -> None:
        snap = self._snap()
        self._core.poll_events()  # drain the RT event ring (keep it from filling)

        mode = Mode(int(snap["mode"]))
        fault = FaultReason(int(snap["fault"]))
        tp = bool(snap["tp_enabled"])
        estop = bool(snap["e_stopped"])
        in_error = bool(snap["in_error"])
        motion_possible = bool(snap["motion_possible"])
        self._epoch = int(snap["epoch"])

        # TEACH→AUTO self-heal edge: clear the cooldown so recovery fires at once.
        if self._prev_tp_enabled is True and tp is False:
            logger.info("airo_fanuc: TEACH→AUTO — clearing recovery cooldown (self-heal)")
            self._last_recovery_mono = -math.inf
        self._prev_tp_enabled = tp

        observed = classify(mode, fault)
        with self._lock:
            if self._state != LifecycleState.SHUTTING_DOWN and not self._recovering:
                if observed in (LifecycleState.FAULTED, LifecycleState.DEGRADED):
                    if self._fault_since_mono is None:
                        self._fault_since_mono = time.monotonic()
                    self._set_state_locked(observed, fault)
                elif observed == LifecycleState.STREAMING:
                    self._fault_since_mono = None
                    self._set_state_locked(LifecycleState.STREAMING, FaultReason.NONE)

        # Latch trajectory_start for the active handle at the CAPTURE→TRAJECTORY edge.
        if self._active_handle is not None:
            self._active_handle._observe(snap)  # noqa: SLF001 - cooperative latch

        # SYST-348 OPERATOR_REQUIRED watch (1 Hz on the commands-only session).
        if self._operator_required:
            self._syst348_watch()
            return  # no auto-recovery while operator action is pending

        # Auto-recovery policy.
        if self._policy.auto_recover and observed in (LifecycleState.FAULTED, LifecycleState.DEGRADED):
            self._maybe_auto_recover(
                fault, estop=estop, in_error=in_error, tp=tp, motion_possible=motion_possible
            )

    # ==================================================================
    # Recovery
    # ==================================================================

    def _maybe_auto_recover(
        self, fault: FaultReason, *, estop: bool, in_error: bool, tp: bool, motion_possible: bool
    ) -> None:
        if self._recovering:
            return
        # Skip while tp_enabled, and deliberately WITHOUT arming the cooldown: the
        # keyswitch is a human decision, so the TEACH→AUTO edge should re-fire
        # recovery at once rather than wait out a cooldown nobody asked for.
        if tp:
            return
        # Cannot auto-recover a held e-stop or a latched alarm — the controller
        # refuses motion until a human clears them, so only an operator action or an
        # explicit recover() can move this forward.
        if estop or in_error:
            return
        now = time.monotonic()
        if now - self._last_recovery_mono < self._policy.recovery_cooldown_s:
            return
        if self._fault_since_mono is None or (now - self._fault_since_mono) < self._policy.recovery_delay_s:
            return
        threading.Thread(target=self._run_recovery_ladder, kwargs={"explicit": False}, daemon=True).start()

    def recover(self, *, timeout_s: float | None = None) -> bool:
        """Explicit recovery ladder (immediate — bypasses the cooldown / settle delay).

        Blocks until the core is back at HOLD (or the timeout). Returns True on
        success. Sets MOTION_INHIBITED when the latched fault was an e-stop /
        operator-required class (ARM gate). Refuses while a held e-stop or TEACH
        keyswitch physically blocks RMI motion ops.
        """
        return self._run_recovery_ladder(explicit=True, timeout_s=timeout_s)

    def _run_recovery_ladder(self, *, explicit: bool, timeout_s: float | None = None) -> bool:
        with self._recovery_lock:
            if self._recovering:
                return False
            self._recovering = True
        ok = False
        # Fail-fast-gate the gripper for the whole ladder: a gripper command
        # racing the ladder must be rejected, not actuate GRIPDISP mid-recovery. Set
        # OUTSIDE the supervisor's _recovery_lock — set_recovery only takes the
        # gripper's own lock (cheap flag; no RMI), so there is no lock-ordering hazard.
        self._gripper_set_recovery(True)
        try:
            self._set_state(LifecycleState.RECOVERING, self._fault_reason)
            ok = self._recovery_body(explicit=explicit, timeout_s=timeout_s)
            return ok
        finally:
            self._gripper_set_recovery(False)
            with self._recovery_lock:
                self._recovering = False
            if not ok:
                # Don't leave the state stuck in RECOVERING — restore the honestly
                # observed state (FAULTED/DEGRADED/STREAMING) from the live snapshot.
                self._reclassify_now()

    def _reclassify_now(self) -> None:
        snap = self._snap()
        observed = classify(Mode(int(snap["mode"])), FaultReason(int(snap["fault"])))
        fault = FaultReason(int(snap["fault"])) if observed != LifecycleState.STREAMING else FaultReason.NONE
        with self._lock:
            if self._state != LifecycleState.SHUTTING_DOWN:
                self._set_state_locked(observed, fault, operator_required=self._operator_required)

    def _recovery_body(self, *, explicit: bool, timeout_s: float | None) -> bool:
        snap = self._snap()
        if bool(snap["e_stopped"]):
            logger.warning("airo_fanuc: cannot recover — e-stop is held; release it first")
            return False
        if bool(snap["tp_enabled"]):
            # Skip WITHOUT arming the cooldown (TEACH→AUTO edge re-fires recovery).
            logger.info("airo_fanuc: skipping recovery — teach pendant in T1/T2")
            return False
        if self._operator_required and not explicit:
            return False

        self._last_recovery_mono = time.monotonic()
        latched = self._fault_reason
        ready_wait = timeout_s if timeout_s is not None else self._policy.ready_wait_s

        # RMI ladder — reset → (best-effort Continue) → reseed (FINAL) → FRC_Call.
        # A SystemFault can leave RMI unresponsive: FRC_Reset / FRC_Call then
        # time out → RmiSessionDown. Without an escalation the ladder dead-ends
        # right here and the arm stays frozen, so on RmiSessionDown escalate to a
        # full RMI reconnect (the bring-up "flush stale SystemFault" sequence),
        # bounded by policy.recovery_reconnect_attempts.
        try:
            if not self._rmi_relaunch_stream_motn(abort=self._abort_recovery):
                logger.warning("airo_fanuc: recovery aborted — a new fault arrived mid-ladder")
                return False
        except RmiError as exc:
            # Controller-reported ErrorID — not a dead session; do NOT churn RMI.
            if self._is_syst348(exc):
                self._enter_operator_required()
                return False
            logger.error("airo_fanuc: recovery RMI step failed: %s", exc)
            return False
        except RmiSessionDown as exc:
            logger.warning(
                "airo_fanuc: RMI session down during recovery (%s) — full RMI reconnect", exc,
            )
            if not self._relaunch_stream_motn_via_reconnect():
                return False

        # Motion must re-arm before we ask the C++ dwell to leave SAFE_FOLLOW. Probe
        # motion_possible on a SHORT timeout (recovery_motion_probe_s, ~5 s) — NOT the
        # 60 s ready_wait — so a fault that a plain reset+FRC_Call cannot clear (a
        # FRC_SystemFault de-initialises the controller's RMI motion interface, which
        # only a re-`initialize()` fixes — HW-observed 2026-07-17) FAILS THIS TIER
        # FAST and returns False. FanucDriver.recover() then escalates to the cold
        # reconnect (full re-bring-up: initialize + GRIPDISP re-fork), rather than
        # stalling the whole ready_wait here and looking like a HOLD timeout.
        motion_probe_s = min(self._policy.recovery_motion_probe_s, ready_wait)
        if not self._wait(lambda: bool(self._snap()["motion_possible"]), motion_probe_s):
            logger.warning(
                "airo_fanuc: recovery ladder did not re-arm motion_possible within "
                "%.1fs — tier failed (caller may escalate to cold reconnect)",
                motion_probe_s,
            )
            return False
        self._core.recover()
        if not self._wait_mode(Mode.HOLD, ready_wait):
            logger.warning("airo_fanuc: recovery did not reach HOLD within %.1fs", ready_wait)
            return False

        # ARM gate: an e-stop / operator-required recovery ends MOTION_INHIBITED, and
        # every motion method raises until an explicit arm(). Whoever cleared the
        # fault is standing at the pendant, inside the robot's envelope; resuming
        # motion on our own initiative would move the robot at them.
        with self._lock:
            self._recovery_count += 1
            if self._policy.arm_gate and (requires_arm(latched) or self._operator_required):
                self._motion_inhibited = True
                logger.info("airo_fanuc: MOTION_INHIBITED after %s recovery — arm() required", latched.name)
            self._operator_required = False
            self._fault_since_mono = None
            self._set_state_locked(LifecycleState.STREAMING, FaultReason.NONE)
        logger.info("airo_fanuc: recovery complete → STREAMING (from %s)", latched.name)
        return True

    def _rmi_relaunch_stream_motn(self, *, abort: Callable[[], bool] | None = None) -> bool:
        """Re-launch STREAM_MOTN over the live RMI session, the step that re-arms motion.

        ``reset`` → best-effort ``FRC_Continue`` → seq reseed → ``FRC_Call(STREAM_MOTN)``.
        The reseed must be the LAST thing before the Call (the controller's
        ``NextSequenceID`` moves under us) and its own failure is non-fatal.

        Shared by the recovery ladder and by :meth:`_settle_stream_motn` so the two cannot
        drift apart — they are the same operation, applied for different reasons. Error
        handling stays with the callers because their policies differ: recovery escalates a
        dead session to a full reconnect, bring-up retries the whole ladder with triage.
        ``abort`` is polled once after the reset, before anything is launched, so a fault
        arriving mid-step stops the Call; ``False`` means it fired.

        NB this is deliberately NOT used for the FIRST launch in :meth:`_bringup_once`. That
        ordering (reset → [GRPRUN fork → settle → reset → full reconnect] → reseed → Call)
        is confirmed on hardware step by step (docs/controller-notes.md §4) and the
        ``FRC_Continue`` sits elsewhere in it; re-shaping it to share code here would change
        a sequence whose every step is load-bearing.
        """
        self._rmi.reset()
        self._maybe_continue()
        if abort is not None and abort():
            return False
        try:
            self._rmi.reseed_sequence_id_from_controller()
        except (RmiError, RmiSessionDown) as exc:
            logger.warning("airo_fanuc: relaunch reseed failed (proceeding): %s", exc)
        self._rmi.program_call(_STREAM_MOTN)
        return True

    def _motion_possible_holds(self, hold_s: float) -> bool:
        """True iff ``motion_possible`` stays asserted for ``hold_s`` continuously."""
        deadline = time.monotonic() + hold_s
        while time.monotonic() < deadline:
            if not bool(self._snap()["motion_possible"]):
                return False
            time.sleep(0.01)
        return True

    def _settle_stream_motn(self) -> bool:
        """Require ``motion_possible`` to HOLD before bring-up reports success.

        STREAM_MOTN cannot be un-launched over RMI (``FRC_Abort`` and ``FRC_Reset`` both
        leave ``program_status=2``), and a bare SM handshake to an already-running instance
        does not re-arm motion — so every bring-up must re-``FRC_Call`` it, and that re-Call
        drops ``motion_possible`` for about a second. Measured: the drop lands roughly 1 s
        after the preroll reports ready, so a single assert-once check passes and the robot
        faults immediately afterwards (docs/controller-notes.md §4.2).

        The first Call therefore lands while the previous session's instance is still live.
        The same step applied once more, after it has gone, sticks — which is exactly what
        the recovery ladder was doing, one fault later. Doing it here instead keeps the
        constructor's contract (it returns a commandable robot, not one about to fault),
        leaves ``recovery_count`` at 0 so a genuine early fault is distinguishable, and
        makes bring-up work under ``auto_recover=False``.
        """
        settle_s = self._policy.bringup_settle_s
        if settle_s <= 0.0:
            return True
        for attempt in range(1, _SETTLE_ATTEMPTS + 1):
            if self._motion_possible_holds(settle_s):
                return True
            logger.info(
                "airo_fanuc: motion_possible dropped within %.1fs of preroll (settle attempt "
                "%d/%d) — re-applying the STREAM_MOTN relaunch",
                settle_s,
                attempt,
                _SETTLE_ATTEMPTS,
            )
            self._rmi_relaunch_stream_motn()
            if not self._wait(
                lambda: bool(self._snap()["motion_possible"]), self._policy.recovery_motion_probe_s
            ):
                continue
            # The core entered SAFE_FOLLOW on the drop and holds there until told the
            # anti-flap dwell may end — the same call the recovery ladder makes.
            self._core.recover()
            self._wait_mode(Mode.HOLD, self._policy.hold_wait_s)
        return self._motion_possible_holds(settle_s)

    def _relaunch_stream_motn_via_reconnect(self) -> bool:
        """Clear an RMI-wedging SystemFault via the bring-up flush reconnect.

        The normal recovery ladder dead-ends when a SystemFault leaves RMI
        unresponsive (FRC_Reset / FRC_Call time out → RmiSessionDown). This
        mirrors ``_bringup_once``'s TP_LAUNCH flush — ``rmi.stop()`` → reconnect
        (tolerating already-connected) → reset → (best-effort Continue) → reseed
        → FRC_Call(STREAM_MOTN) — to re-arm motion on a fresh RMI session
        without a full cold ``FanucDriver.reconnect()`` (which also tears down
        and rebuilds the C++ core). Bounded by
        ``policy.recovery_reconnect_attempts`` because RMI churn is itself a
        documented SM-daemon wedge vector. Returns True once STREAM_MOTN is
        (re)launched, False if the budget is exhausted or recovery aborts.
        """
        for attempt in range(1, self._policy.recovery_reconnect_attempts + 1):
            if self._abort_recovery():
                logger.warning("airo_fanuc: recovery aborted before RMI reconnect")
                return False
            try:
                self._rmi.stop()
            except Exception as exc:  # noqa: BLE001 — best-effort teardown, never fatal
                logger.debug("airo_fanuc: recovery RMI stop raised (continuing): %s", exc)
            try:
                self._reconnect_rmi_tolerating_already_connected()
                self._rmi.reset()
                self._maybe_continue()
                self._rmi.reseed_sequence_id_from_controller()
                self._rmi.program_call(_STREAM_MOTN)
            except RmiError as exc:
                if self._is_syst348(exc):
                    self._enter_operator_required()
                    return False
                logger.warning(
                    "airo_fanuc: RMI reconnect attempt %d/%d failed (ErrorID): %s",
                    attempt, self._policy.recovery_reconnect_attempts, exc,
                )
                continue
            except (FanucConnectionError, RmiSessionDown) as exc:
                logger.warning(
                    "airo_fanuc: RMI reconnect attempt %d/%d failed (session): %s",
                    attempt, self._policy.recovery_reconnect_attempts, exc,
                )
                continue
            logger.info(
                "airo_fanuc: RMI reconnect (attempt %d) relaunched STREAM_MOTN", attempt,
            )
            return True
        logger.error(
            "airo_fanuc: recovery exhausted %d RMI reconnect(s) — session still wedged",
            self._policy.recovery_reconnect_attempts,
        )
        return False

    def _abort_recovery(self) -> bool:
        """A new e-stop / TEACH / RX-silence event aborts an in-flight recovery: a
        ladder that pushed on through a fresh fault would re-arm motion the operator
        just interrupted."""
        snap = self._snap()
        return (
            bool(snap["e_stopped"]) or bool(snap["tp_enabled"]) or Mode(int(snap["mode"])) == Mode.RX_SILENT
        )

    def _maybe_continue(self) -> None:
        """Best-effort FRC_Continue (tolerate 2556938 "TP program not paused").

        Wired via :meth:`RmiClient.program_continue`; issued after FRC_Reset in the
        recovery ladder (FRC_Continue → ErrorID 0 OK, observed on hardware).
        Duck-typed via ``getattr`` so a hypothetical commands-only ``RmiClient``
        variant without the method degrades to a no-op rather than raising."""
        fn = getattr(self._rmi, "program_continue", None)
        if fn is None:
            return
        try:
            fn()
        except RmiError as exc:
            if exc.error_id != _ERR_TP_NOT_PAUSED:
                logger.warning("airo_fanuc: best-effort FRC_Continue failed: %s", exc)
        except RmiSessionDown as exc:
            logger.warning("airo_fanuc: best-effort FRC_Continue — RMI down: %s", exc)

    # ==================================================================
    # SYST-348 OPERATOR_REQUIRED flow
    # ==================================================================

    def _is_syst348(self, exc: RmiError) -> bool:
        if exc.error_id not in _SYST348_ERROR_IDS:
            return False
        try:
            err = self._rmi.read_error(5)
        except (RmiError, RmiSessionDown, OSError):
            return True  # the 2556936 alone is enough to stop retrying
        return any("SYST-348" in m for m in err.messages)

    def _enter_operator_required(self) -> None:
        with self._lock:
            self._operator_required = True
            self._operator_hint = operator_hint_for(self._fault_reason, operator_required=True)
            if self._policy.arm_gate:
                self._motion_inhibited = True  # latches through the clear (arm() still needed)
            self._set_state_locked(LifecycleState.FAULTED, self._fault_reason, operator_required=True)
        logger.error("airo_fanuc: SYST-348 payload monitor — OPERATOR_REQUIRED: %s", self._operator_hint)

    def _syst348_watch(self) -> None:
        now = time.monotonic()
        if now - self._last_syst348_watch_mono < _SYST348_WATCH_PERIOD_S:
            return
        self._last_syst348_watch_mono = now
        try:
            err = self._rmi.read_error(5)
        except (RmiError, RmiSessionDown, OSError):
            return
        if not any("SYST-348" in m for m in err.messages):
            with self._lock:
                self._operator_required = False
                self._last_recovery_mono = -math.inf  # let recovery fire immediately
            logger.info("airo_fanuc: SYST-348 cleared — resuming recovery (arm() still required)")

    # ==================================================================
    # ARM gate + state accessors
    # ==================================================================

    def arm(self) -> None:
        """Lift the ARM gate: permit motion again after an e-stop / operator-required
        recovery left the driver MOTION_INHIBITED. Only the caller may do this — the
        gate exists so the robot never moves on its own initiative while an operator
        is still at the pendant."""
        with self._lock:
            was = self._motion_inhibited
            self._motion_inhibited = False
        if was:
            logger.info("airo_fanuc: armed — motion permitted")

    def is_motion_inhibited(self) -> bool:
        with self._lock:
            return self._motion_inhibited

    def is_commandable(self) -> bool:
        with self._lock:
            return self._state == LifecycleState.STREAMING and not self._motion_inhibited

    def state(self) -> LifecycleState:
        with self._lock:
            return self._state

    def set_active_handle(self, handle: MotionHandle | None) -> None:
        self._active_handle = handle

    def set_gripper(self, gripper: GripperWorker | None) -> None:
        """Register (or clear) the gripper the recovery ladder fail-fast-gates.

        Called by the driver after it builds the gripper. Just stores the reference;
        the ladder toggles ``gripper.set_recovery(...)`` — a cheap flag toggle guarded
        only by the gripper's OWN lock (no RMI, no callback into the supervisor), so
        it never blocks on the shared RMI ``_request_lock`` and cannot deadlock the
        recovery path."""
        self._gripper = gripper

    def _gripper_set_recovery(self, active: bool) -> None:
        """Toggle the registered gripper's fail-fast gate (None-safe, never raises).

        Wrapped so a gripper-side failure can never wedge the recovery ladder —
        critically, the ``set_recovery(False)`` in the ladder's ``finally`` MUST NOT
        leave the gate stuck ``True`` (which would reject every later gripper command)."""
        gripper = self._gripper
        if gripper is None:
            return
        try:
            gripper.set_recovery(active)
        except Exception as exc:  # noqa: BLE001 - gate toggle must never break recovery
            logger.warning("airo_fanuc: gripper set_recovery(%s) failed: %s", active, exc)

    def lifecycle_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "lifecycle_state": self._state.value,
                "fault_reason": self._fault_reason_str,
                "operator_hint": self._operator_hint,
                "operator_required": self._operator_required,
                "motion_inhibited": self._motion_inhibited,
                "recovery_count": self._recovery_count,
                "epoch": self._epoch,
            }

    def command_reject_reason(self) -> tuple[str, str | None] | None:
        """``(reason, hint)`` if a motion should be refused now, else ``None``.

        Motion is refused when not STREAMING or while MOTION_INHIBITED (ARM gate)."""
        with self._lock:
            if self._motion_inhibited:
                return ("motion_inhibited", "call arm() to permit motion after a fault/e-stop recovery")
            if self._state != LifecycleState.STREAMING:
                return (self._fault_reason_str, self._operator_hint)
            return None

    # ==================================================================
    # Shutdown
    # ==================================================================

    def shutdown(self, timeout_s: float = 2.0) -> None:
        """Stop the watch thread (deterministic join). Does NOT stop core/rmi —
        the driver's ordered close() owns that."""
        with self._lock:
            self._set_state_locked(LifecycleState.SHUTTING_DOWN, self._fault_reason)
        self._stop_evt.set()
        t = self._thread
        if t is not None:
            t.join(timeout=timeout_s)
            self._thread = None
        hb = self._hb_thread
        if hb is not None:
            hb.join(timeout=timeout_s)
            self._hb_thread = None
        # Wait out any in-flight recovery-ladder daemon (spawned by
        # _maybe_auto_recover; NOT the watch thread joined above) before returning, so
        # a caller that tears down rmi/core right after shutdown() —
        # FanucDriver.reconnect() (the cold-recovery escalation) and close() — never
        # races a ladder still using them. _stop_evt (set above) makes the ladder's
        # _wait/_wait_mode bail promptly and its finally clears _recovering; the watch
        # loop is joined so no NEW ladder can spawn. Best-effort + bounded.
        deadline = time.monotonic() + timeout_s
        while self._recovering and time.monotonic() < deadline:
            time.sleep(0.01)

    # ==================================================================
    # Internals
    # ==================================================================

    def _set_state(
        self, state: LifecycleState, fault: FaultReason, *, operator_required: bool = False
    ) -> None:
        with self._lock:
            self._set_state_locked(state, fault, operator_required=operator_required)

    def _set_state_locked(
        self, state: LifecycleState, fault: FaultReason, *, operator_required: bool = False
    ) -> None:
        # While OPERATOR_REQUIRED is latched (SYST-348 / payload-confirm), the hint
        # MUST stay the actionable operator-required instruction — a coexisting
        # lower-priority observed fault (e.g. motion_not_possible on the same tick)
        # must NOT clobber it: the operator-required reason outranks everything else
        # in the ordered fault list, because it is the only one that tells the
        # operator the action that will actually clear the state. The observed
        # ``fault`` still drives ``fault_reason`` (ordered-list semantics
        # preserved); only the hint honors the latched operator-required precedence.
        op_req = operator_required or self._operator_required
        changed = state != self._state or fault != self._fault_reason
        self._state = state
        self._fault_reason = fault
        self._fault_reason_str = fault_reason_string(fault)
        if fault == FaultReason.NONE and not op_req:
            self._operator_hint = None
        else:
            self._operator_hint = operator_hint_for(fault, operator_required=op_req)
        if changed:
            logger.info(
                "airo_fanuc: lifecycle → %s (fault=%s%s)",
                state.value,
                self._fault_reason_str,
                ", OPERATOR_REQUIRED" if op_req else "",
            )

    def _wait(self, pred: Any, timeout_s: float, interval_s: float = 0.004) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._stop_evt.is_set():
                return False
            if pred():
                return True
            time.sleep(interval_s)
        return bool(pred())

    def _wait_mode(self, mode: Mode, timeout_s: float) -> bool:
        return self._wait(lambda: Mode(int(self._snap()["mode"])) == mode, timeout_s)

    def _snap(self) -> dict[str, Any]:
        """``StreamCore.get_snapshot`` typed as ``dict[str, Any]`` (the stub types
        the values ``object``; the runtime values are int/float/bool/list)."""
        return cast("dict[str, Any]", self._core.get_snapshot())


__all__ = ["Supervisor"]
