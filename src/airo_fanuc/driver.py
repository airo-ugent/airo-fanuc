# SPDX-License-Identifier: Apache-2.0
"""``FanucDriver`` — the ur_rtde-shaped, construct-and-go public API (PLAN.md §5.1).

One object combines the ur_rtde Control + Receive concerns over the FANUC
CRX-10iA/L: the constructor brings the robot up to *commandable* (or raises with a
real reason), :meth:`move_trajectory` / :meth:`servo_j` are the only motion
surfaces, :meth:`stop_j` is the universal preempt, and the getters never raise and
never lie (value + age). The C++ ``StreamCore`` owns the 125 Hz timeline; the
:class:`~airo_fanuc.supervisor.Supervisor` owns lifecycle/recovery policy; this
class is the thin, honest facade over both.

Composition (all public attrs — the ur_rtde "reach through" doctrine):
``driver.core`` (RT core), ``driver.rmi`` (RMI session), ``driver.gripper``.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Sequence
from types import TracebackType
from typing import Any, cast

import numpy as np

from . import _core
from ._core import Mode, MotionStatus, StreamCore
from .config import DriverPolicy, MotionResult
from .controller_facts import SettlePolicy
from .exceptions import (
    FanucError,
    RejectedStartMismatch,
    RobotFaultedError,
    TrajectoryValidationError,
)
from .gripper_worker import GripperWorker
from .lifecycle import motion_result_of
from .ownership import OwnershipLock
from .republisher import Republisher
from .rmi_client import RmiClient
from .supervisor import Supervisor

logger = logging.getLogger("airo_fanuc.driver")

__all__ = ["FanucDriver", "MotionHandle"]


def _snap(core: StreamCore) -> dict[str, Any]:
    """``StreamCore.get_snapshot`` typed as ``dict[str, Any]`` (the stub types the
    values ``object``; the runtime values are int/float/bool/list)."""
    return cast("dict[str, Any]", core.get_snapshot())


_NDOF = 6
_INT64_MAX = 2**63 - 1
_STEADY_QD_EPS_RAD_S = math.radians(2.0)  # SettlePolicy vel_eps default


class MotionHandle:
    """Non-raising handle over one submitted motion (PLAN.md §5.1).

    Maps the terminal C++ :class:`MotionStatus` (via
    :func:`airo_fanuc.lifecycle.motion_result_of`) to a :class:`MotionResult`.
    ``stop_j()`` during a blocking :meth:`wait` resolves it as ``STOPPED`` (the
    core flips the active motion's status; :meth:`wait` observes it) — the
    brake+replan contract (R3 A1).
    """

    def __init__(self, core: StreamCore, motion_id: int, submit_mono_ns: int) -> None:
        self._core = core
        self._motion_id = motion_id
        self._submit_mono_ns = submit_mono_ns
        self._traj_start_mono_ns: int | None = None
        self._lock = threading.Lock()

    @property
    def motion_id(self) -> int:
        return self._motion_id

    @property
    def submit_mono_ns(self) -> int:
        return self._submit_mono_ns

    @property
    def trajectory_start_mono_ns(self) -> int | None:
        """Monotonic ns the executed TRAJECTORY playback began (post-capture anchor;
        TrajectoryMonitor uses this). Host-observed at the CAPTURE→TRAJECTORY edge
        (±supervisor watch interval), because the C++ core exposes no per-motion
        anchor; ``None`` until the edge is seen."""
        self._observe(_snap(self._core))
        with self._lock:
            return self._traj_start_mono_ns

    def _observe(self, snap: dict[str, Any]) -> None:
        """Cooperative latch of the trajectory-start anchor. Called by the handle's
        own accessors AND by the supervisor watch loop (so it latches even if the
        caller never polls)."""
        with self._lock:
            if self._traj_start_mono_ns is not None:
                return
            if (
                int(snap.get("active_motion_id", 0)) == self._motion_id
                and Mode(int(snap["mode"])) == Mode.TRAJECTORY
            ):
                self._traj_start_mono_ns = time.monotonic_ns()

    def _status(self) -> MotionStatus:
        return MotionStatus(self._core.motion_status(self._motion_id))

    def done(self) -> bool:
        """True once the motion has reached a terminal result."""
        return motion_result_of(self._status()) is not None

    def result(self) -> MotionResult | None:
        """The terminal :class:`MotionResult`, or ``None`` while still running."""
        return motion_result_of(self._status())

    def wait(self, timeout: float | None = None) -> MotionResult:
        """Block until the motion reaches a terminal :class:`MotionResult` and return it.

        Motion OUTCOMES never raise (a faulted motion returns
        ``MotionResult.FAULTED``, a preempt returns ``STOPPED``, etc. — the airo
        convention). A positive ``timeout`` that elapses while the motion is still
        RUNNING raises :class:`TimeoutError` (a caller-supervision concern, not a
        motion outcome; the motion keeps running — poll :meth:`result`). ``None``
        (default) waits indefinitely.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            self._observe(_snap(self._core))
            result = motion_result_of(self._status())
            if result is not None:
                return result
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError(
                    f"motion {self._motion_id} did not finish within {timeout:.3f}s "
                    f"(status={self._status().name}); it is still running"
                )
            time.sleep(0.002)


class FanucDriver:
    """Construct-and-go FANUC CRX-10iA/L driver (ur_rtde-shaped)."""

    def __init__(self, ip: str, policy: DriverPolicy | None = None) -> None:
        self._ip = ip
        self._policy = policy if policy is not None else DriverPolicy()
        self._cfg = self._policy.config
        self._closed = False
        self._close_lock = threading.Lock()

        self._ownership: OwnershipLock | None = None
        self.rmi: RmiClient | None = None
        self.core: StreamCore | None = None
        self.gripper: GripperWorker | None = None
        self._supervisor: Supervisor | None = None
        self._republisher: Republisher | None = None

        try:
            if self._policy.enable_ownership:
                self._ownership = OwnershipLock(self._policy.ownership_mode, path=self._policy.lock_path)
                self._ownership.acquire()  # raises OwnershipError naming the holder

            self.rmi = RmiClient(
                ip,
                self._cfg.rmi_port,
                connect_timeout=self._cfg.rmi_connect_timeout,
                request_timeout=self._cfg.rmi_request_timeout,
            )
            self.core = StreamCore(ip, self._cfg.sm_port, self._cfg.to_rt_core_config())
            self._supervisor = Supervisor(self.core, self.rmi, self._policy)
            self._preflight_report = self._supervisor.bringup()  # raises FanucConnectionError/Preflight
            self._supervisor.start_watch()

            if self._policy.enable_gripper:
                self.gripper = GripperWorker(self.rmi)
                # Let the supervisor fail-fast-gate the gripper during a recovery
                # ladder (R2 F32) — a gripper command must never actuate GRIPDISP
                # mid-recovery (e.g. just after an e-stop release).
                self._supervisor.set_gripper(self.gripper)
            if self._policy.publisher is not None:
                self._republisher = Republisher(
                    self._policy.publisher,
                    self.get_state,
                    joints_at_wall=self.joints_at_wall,
                )
                self._republisher.start()
        except BaseException:
            self._emergency_cleanup()
            raise

    # ==================================================================
    # Motion
    # ==================================================================

    def move_trajectory(
        self,
        times: Sequence[int] | np.ndarray,
        q: Sequence[Sequence[float]] | np.ndarray,
        qd: Sequence[Sequence[float]] | np.ndarray,
        *,
        speed_scale: float = 1.0,
        settle: SettlePolicy | None = None,
        deadman_s: float | None = None,
        force_stop_n: float | None = None,
        asynchronous: bool = False,
    ) -> MotionHandle:
        """Submit ONE whole trajectory (rad, ns-relative int64 times) — CAPTURE-or-REJECT
        splice, Hermite playback, settle → :class:`MotionHandle`.

        Validation (per-violation typed errors, PLAN §5.1 / R4): strictly-increasing
        int64 ns times, ≥2 knots, finite q/qd, ``|s·qd| ≤ v_lim``, ``s ≤ 1.0``.
        The CAPTURE collision-check hook runs when ``policy.capture_check`` is set.
        """
        self._require_commandable()
        assert self.core is not None
        if force_stop_n is not None and force_stop_n > 0.0 and not self._force_telemetry_available():
            raise TrajectoryValidationError(
                "force_stop_n was requested but this controller provides no force telemetry "
                "(Stream Motion v3 / type-202: fs_type unavailable, wrench invalid). The C++ "
                "force-guard cannot trip without a wrench, so arming it would be a silent "
                "safety no-op. Rely on the controller's collaborative contact-stop for contact "
                "detection (it halts the descent; the motion resolves FAULTED with reason "
                "CONTACT_STOP), or upgrade to a v4 / type-204 controller for a numeric force "
                "threshold. See controller-notes.md §1.8."
            )
        times_ns, q_arr, qd_arr = self._validate_trajectory(times, q, qd, speed_scale)
        settle = settle if settle is not None else self._policy.settle

        self._capture_gate(q_arr, qd_arr, asynchronous)

        mid = self.core.submit_trajectory(
            times_ns,
            q_arr.tolist(),
            qd_arr.tolist(),
            float(speed_scale),
            settle_tol_rad=math.radians(settle.tol_deg),
            settle_vel_eps_rad_s=math.radians(settle.vel_eps_deg_s),
            settle_timeout_s=float(settle.timeout_s),
            force_stop_n=float(force_stop_n) if force_stop_n is not None else 0.0,
            deadman_s=float(deadman_s) if deadman_s is not None else 0.0,
        )
        handle = MotionHandle(self.core, mid, time.monotonic_ns())
        assert self._supervisor is not None
        self._supervisor.set_active_handle(handle)
        return handle

    def servo_j(
        self,
        q: Sequence[float] | np.ndarray,
        duration: float,
        *,
        qd: Sequence[float] | np.ndarray | None = None,
        qdd: Sequence[float] | np.ndarray | None = None,
    ) -> MotionHandle:
        """Replace-not-queue servo target (rad). The core rejects a target > 5° from
        the commanded pose (:class:`MotionResult.REJECTED`). Starvation-safe.

        Feed-forward: pass ``qd`` (and optionally ``qdd``, rad/s and rad/s²) to use them
        as Ruckig's target velocity/acceleration instead of the secant reconstruction the
        position-only path derives from consecutive targets. Supplying an externally
        planned knot's own derivatives (e.g. an MPC action-sequence point) renders that
        smooth trajectory faithfully — no secant lag, no accel=0 forcing at each knot
        (the servoing twitch). ``qd``/``qdd`` are clamped to the servo limits in the core.
        With no ``qd`` the behaviour is unchanged (secant + zero target acceleration).
        """
        self._require_commandable()
        assert self.core is not None
        q_arr = np.asarray(q, dtype=np.float64).reshape(-1)
        if q_arr.shape[0] != _NDOF:
            raise TrajectoryValidationError(f"servo_j expects {_NDOF} joint values, got {q_arr.shape[0]}")
        if not np.all(np.isfinite(q_arr)):
            raise TrajectoryValidationError("servo_j target is not finite")
        if not (duration > 0.0 and math.isfinite(duration)):
            raise TrajectoryValidationError(f"servo_j duration must be finite and > 0, got {duration}")
        if qd is None:
            mid = self.core.submit_servo(q_arr.tolist(), float(duration))
        else:
            qd_arr = np.asarray(qd, dtype=np.float64).reshape(-1)
            qdd_arr = (
                np.zeros(_NDOF, dtype=np.float64)
                if qdd is None
                else np.asarray(qdd, dtype=np.float64).reshape(-1)
            )
            if qd_arr.shape[0] != _NDOF or qdd_arr.shape[0] != _NDOF:
                raise TrajectoryValidationError(
                    f"servo_j qd/qdd expect {_NDOF} values, got {qd_arr.shape[0]}/{qdd_arr.shape[0]}"
                )
            if not (np.all(np.isfinite(qd_arr)) and np.all(np.isfinite(qdd_arr))):
                raise TrajectoryValidationError("servo_j qd/qdd are not finite")
            mid = self.core.submit_servo_ff(
                q_arr.tolist(), qd_arr.tolist(), qdd_arr.tolist(), float(duration)
            )
        return MotionHandle(self.core, mid, time.monotonic_ns())

    def stop_j(self) -> None:
        """Universal preempt (any thread, ≤1 tick). NEVER raises (per-state no-op)."""
        core = self.core
        if core is None:
            return
        try:
            core.stop_j()
        except Exception as exc:  # noqa: BLE001 - stop_j must never raise
            logger.warning("airo_fanuc: stop_j swallowed an error (never raises): %s", exc)

    def hold(self) -> None:
        """Request HOLD at the current commanded pose."""
        if self.core is not None:
            self.core.hold()

    def kick(self) -> None:
        """Feed the caller-fed deadman (collision-supervision-alive interlock)."""
        if self.core is not None:
            self.core.kick()

    # ==================================================================
    # Steadiness / recovery / arm
    # ==================================================================

    def is_steady(self) -> bool:
        """True when the robot is at rest in HOLD (settled, near-zero velocity)."""
        if self.core is None:
            return False
        snap = _snap(self.core)
        if Mode(int(snap["mode"])) != Mode.HOLD:
            return False
        return max(abs(v) for v in snap["qd_est"]) < _STEADY_QD_EPS_RAD_S

    def wait_until_steady(self, timeout: float = 5.0) -> bool:
        """Block until :meth:`is_steady` (R3 F1). Returns False on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.is_steady():
                return True
            time.sleep(0.01)
        return self.is_steady()

    def arm(self) -> None:
        """Clear MOTION_INHIBITED after an e-stop / operator-required recovery (ARM gate)."""
        if self._supervisor is not None:
            self._supervisor.arm()

    def recover(self, timeout_s: float | None = None, *, escalate_reconnect: bool = True) -> bool:
        """Run the recovery ladder now (immediate — bypasses cooldown/settle delay).

        Two tiers (2026-07-17): the light supervisor ladder (reset → FRC_Continue →
        reseed → FRC_Call → short ``motion_possible`` probe) first; if it cannot
        re-arm motion and ``escalate_reconnect`` is set, escalate to a **cold
        reconnect** = the full bring-up ladder. A ``FRC_SystemFault`` (a collision
        contact-stop) de-initialises the controller's RMI motion interface, so a bare
        re-``FRC_Call`` cannot re-arm motion — only a re-``initialize()`` can, and
        bring-up pairs it with the GRIPDISP re-fork, so the cold reconnect restores
        motion AND the gripper in one proven pass (HW-observed: reset+FRC_Call alone
        is not enough).

        ``escalate_reconnect=False`` runs ONLY the light ladder — used by the
        calibration free-drive lifecycle-heal, a best-effort cosmetic latch clear
        that must never tear down + rebuild the RX stream the tool is reading.

        Ends in MOTION_INHIBITED for an e-stop / operator-required class fault
        (call :meth:`arm` before the next motion). Returns True iff the driver is
        commandable again."""
        if self._supervisor is None:
            return False
        if self._supervisor.recover(timeout_s=timeout_s):
            return True
        if not escalate_reconnect or self._closed:
            return False
        # Only escalate to a cold reconnect for faults a re-bring-up can actually
        # clear. A HELD e-stop, a TEACH keyswitch, or an operator-required latch
        # (SYST-348 payload confirm) need a human first — a reconnect would just fail
        # preflight / re-fault, so leave the supervisor's FAULTED verdict intact.
        st = self.get_state()
        if st.get("e_stopped") or st.get("tp_enabled") or st.get("operator_required"):
            logger.info(
                "airo_fanuc: recovery ladder failed but a human-required condition is "
                "latched (e_stop/teach/operator_required) — not auto-reconnecting"
            )
            return False
        logger.warning(
            "airo_fanuc: recovery ladder failed to re-arm motion; escalating to cold "
            "reconnect (full re-bring-up + GRIPDISP re-fork)"
        )
        try:
            self.reconnect()
        except FanucError as exc:  # FanucConnectionError / FanucPreflightError ⊂ FanucError
            logger.error("airo_fanuc: cold reconnect escalation failed: %s", exc)
            return False
        return self._supervisor.is_commandable()

    def reconnect(self) -> None:
        """Cold re-bring-up: quiesce, tear the SM/RMI session down, run the full
        bring-up ladder again. Raises :class:`FanucConnectionError` on failure.

        Also re-establishes the **GRIPDISP gripper dispatcher**: bring-up's
        ``_gripdisp_alive`` probe re-forks GRPRUN when the prior dispatcher was
        aborted/wedged (e.g. a collision that fired mid-close left ``R[1]`` set), and
        its benign OPEN neutralises the stale trigger + clears ``R[1]`` so the gripper
        comes back usable and open. The gripper fail-fast gate is held across the
        rebuild so no gripper command actuates GRIPDISP mid-reconnect (R2 F32); the
        :class:`GripperWorker` object itself survives (same RMI ref, re-``start``ed
        inside ``bringup``), so there is no worker rebuild and no double fork."""
        self._require_open()
        assert self._supervisor is not None and self.core is not None and self.rmi is not None
        self.stop_j()
        # shutdown() now waits out any in-flight recovery ladder, so the teardown below
        # cannot race a ladder still using rmi/core, and the gripper gate we set next
        # cannot be clobbered by that ladder's finally set_recovery(False).
        self._supervisor.shutdown()
        self._set_gripper_recovery(True)  # None-safe; held across the whole rebuild
        try:
            try:
                if self.core.running:
                    self.core.stop()
            except Exception as exc:  # noqa: BLE001
                logger.debug("airo_fanuc: core stop during reconnect: %s", exc)
            try:
                self.rmi.stop()
            except Exception as exc:  # noqa: BLE001
                logger.debug("airo_fanuc: rmi stop during reconnect: %s", exc)
            self._preflight_report = self._supervisor.bringup()
            self._supervisor.start_watch()
        finally:
            self._set_gripper_recovery(False)

    def _set_gripper_recovery(self, active: bool) -> None:
        """Toggle the gripper's fail-fast recovery gate (None-safe, never raises).

        Mirrors ``Supervisor._gripper_set_recovery`` so a gripper-side failure can
        never wedge the reconnect; critically the ``False`` in ``reconnect``'s
        ``finally`` must not be left stuck ``True`` (which would reject every later
        gripper command)."""
        gripper = self.gripper
        if gripper is None:
            return
        try:
            gripper.set_recovery(active)
        except Exception as exc:  # noqa: BLE001 - gate toggle must never break reconnect
            logger.warning("airo_fanuc: gripper set_recovery(%s) during reconnect failed: %s", active, exc)

    # ==================================================================
    # Getters (never raise, never lie)
    # ==================================================================

    def get_state(self) -> dict[str, Any]:
        """Merged seqlock snapshot + lifecycle state + owner (never raises)."""
        state: dict[str, Any] = {}
        core = self.core
        if core is not None:
            try:
                state.update(core.get_snapshot())
            except Exception as exc:  # noqa: BLE001 - getters never raise
                logger.debug("airo_fanuc: get_snapshot failed: %s", exc)
        sup = self._supervisor
        if sup is not None:
            state.update(sup.lifecycle_snapshot())
        state["owner"] = self._owner_record()
        return state

    def _force_telemetry_available(self) -> bool:
        """True iff the controller is currently delivering a valid wrench (fs_type
        ∈ {1, 2}). False on a v3 / type-202 controller (no force block on the wire)
        — see controller-notes.md §1.8. Used to reject an inert ``force_stop_n``."""
        core = self.core
        if core is None:
            return False
        try:
            return bool(core.get_snapshot().get("wrench_valid", False))
        except Exception:  # noqa: BLE001 - a read failure means "not available"
            return False

    def get_wrench(self) -> np.ndarray | None:
        """Tool-frame wrench ``[fx, fy, fz, mx, my, mz]`` (N, Nm) or ``None`` when
        force telemetry is unavailable (fs_type gate / wrench invalid)."""
        core = self.core
        if core is None:
            return None
        snap = core.get_snapshot()
        if not bool(snap.get("wrench_valid", False)):
            return None
        return np.array(
            [snap["fx"], snap["fy"], snap["fz"], snap["mx"], snap["my"], snap["mz"]],
            dtype=np.float64,
        )

    def joints_at_wall(self, t_wall_ns: int) -> np.ndarray | None:
        """Joint state (rad) nearest a wall-clock stamp (camera FK-at-shutter)."""
        core = self.core
        if core is None:
            return None
        joints = core.joints_at_wall(int(t_wall_ns))
        return None if joints is None else np.asarray(joints, dtype=np.float64)

    def timing_stats(self) -> dict[str, Any]:
        """RT tick histograms + invariant counters (never raises)."""
        core = self.core
        if core is None:
            return {}
        try:
            return dict(core.timing_stats())
        except Exception as exc:  # noqa: BLE001
            logger.debug("airo_fanuc: timing_stats failed: %s", exc)
            return {}

    @property
    def preflight_report(self) -> Any:
        return getattr(self, "_preflight_report", None)

    # ==================================================================
    # Lifecycle
    # ==================================================================

    def close(self) -> None:
        """Ordered, poison-not-exit shutdown (R2 F2).

        Quiesce → stop republisher → cancel gripper → join the supervisor thread →
        terminal Stop + join the RT thread → RMI disconnect → release ownership.
        Every join is TIMED: a wedged thread is *abandoned* (left daemon) and a
        typed :class:`FanucError` is raised — the library NEVER calls ``os._exit``.
        The process survives; the app entry point owns any hard-exit watchdog.
        """
        with self._close_lock:
            if self._closed:
                return
            self._closed = True

        problems: list[str] = []

        self.stop_j()  # quiesce (never raises)

        if self._republisher is not None and not _run_with_timeout(self._republisher.stop, 2.0):
            problems.append("republisher.stop() wedged")
        if self.gripper is not None and not _run_with_timeout(self.gripper.close, 2.0):
            problems.append("gripper.close() wedged")
        if self._supervisor is not None and not _run_with_timeout(self._supervisor.shutdown, 3.0):
            problems.append("supervisor thread did not join")
        if self.core is not None and not _run_with_timeout(self._safe_core_stop, 6.0):
            problems.append("core.stop() (RT thread) did not join — abandoned")
        if self.rmi is not None and not _run_with_timeout(self.rmi.stop, 3.0):
            problems.append("rmi.stop() wedged")
        if self._ownership is not None:
            try:
                self._ownership.release()
            except Exception as exc:  # noqa: BLE001
                problems.append(f"ownership release: {exc}")

        if problems:
            # POISON, not exit: surface the wedge as a typed error; the process lives.
            raise FanucError(
                "airo_fanuc: close() did not fully quiesce (process NOT exited): " + "; ".join(problems)
            )
        logger.info("airo_fanuc: closed cleanly")

    def __enter__(self) -> FanucDriver:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    # ==================================================================
    # Internals
    # ==================================================================

    def _safe_core_stop(self) -> None:
        core = self.core
        if core is not None and core.running:
            core.stop()

    def _require_open(self) -> None:
        if self._closed:
            raise FanucError("airo_fanuc: driver is closed")

    def _require_commandable(self) -> None:
        self._require_open()
        sup = self._supervisor
        if sup is None:
            raise FanucError("airo_fanuc: driver not brought up")
        rej = sup.command_reject_reason()
        if rej is not None:
            reason, hint = rej
            raise RobotFaultedError(reason, hint)

    def _owner_record(self) -> dict[str, Any] | None:
        lock = self._ownership
        if lock is None or lock.fd is None:
            return None
        return {"pid": None, "mode": lock.mode, "since": lock.since}

    def _capture_gate(self, q_arr: np.ndarray, qd_arr: np.ndarray, asynchronous: bool) -> None:
        """CAPTURE-or-REJECT + collision-check hook (decision 6 / R3 A2).

        Synthesizes the EXACT capture path the C++ core will execute via
        :func:`airo_fanuc._core.generate_capture_path` (one code path — "the
        checked path IS the executed path"). Beyond the 5° window → synchronous
        :class:`RejectedStartMismatch` (async submissions let the core resolve
        REJECTED). When ``policy.capture_check`` is set, the synthesized knots are
        handed to the caller's collision checker; ``False`` → typed reject.
        """
        assert self.core is not None
        snap = _snap(self.core)
        q_cmd = [float(v) for v in snap["q_cmd"]]
        qd_cmd = [float(v) for v in snap["qd_cmd"]]
        q0 = q_arr[0].tolist()
        qd0 = qd_arr[0].tolist()
        path = cast("dict[str, Any]", _core.generate_capture_path(q_cmd, qd_cmd, q0, qd0))

        if bool(path["would_reject"]):
            if asynchronous:
                return  # the core resolves this submission as REJECTED
            raise RejectedStartMismatch(
                f"trajectory first knot {q0} is beyond the 5° capture window from the commanded pose {q_cmd}"
            )

        check = self._policy.capture_check
        if check is not None:
            count = int(path["count"])
            knots_q = np.asarray(path["q"][:count], dtype=np.float64)
            knots_qd = np.asarray(path["qd"][:count], dtype=np.float64)
            if not check(knots_q, knots_qd):
                raise TrajectoryValidationError(
                    "capture splice failed the collision check — the bridge from the commanded "
                    "pose to the trajectory start would collide"
                )

    def _validate_trajectory(
        self,
        times: Sequence[int] | np.ndarray,
        q: Sequence[Sequence[float]] | np.ndarray,
        qd: Sequence[Sequence[float]] | np.ndarray,
        speed_scale: float,
    ) -> tuple[list[int], np.ndarray, np.ndarray]:
        if not math.isfinite(speed_scale) or speed_scale <= 0.0:
            raise TrajectoryValidationError(f"speed_scale must be finite and > 0, got {speed_scale}")
        if speed_scale > 1.0:
            raise TrajectoryValidationError(f"speed_scale must be ≤ 1.0 (s>1 rejected), got {speed_scale}")

        q_arr = np.asarray(q, dtype=np.float64)
        qd_arr = np.asarray(qd, dtype=np.float64)
        if q_arr.ndim != 2 or q_arr.shape[1] != _NDOF:
            raise TrajectoryValidationError(f"q must be (N, {_NDOF}); got shape {q_arr.shape}")
        if qd_arr.shape != q_arr.shape:
            raise TrajectoryValidationError(f"qd shape {qd_arr.shape} must match q shape {q_arr.shape}")
        n = q_arr.shape[0]
        if n < 2:
            raise TrajectoryValidationError(f"need ≥2 knots, got {n}")
        if not (np.all(np.isfinite(q_arr)) and np.all(np.isfinite(qd_arr))):
            raise TrajectoryValidationError("q/qd contain non-finite values")

        times_arr = np.asarray(times)
        if times_arr.shape[0] != n:
            raise TrajectoryValidationError(f"times length {times_arr.shape[0]} must match {n} knots")
        if not np.all(np.isfinite(times_arr.astype(np.float64))):
            raise TrajectoryValidationError("times contain non-finite values")
        times_ns = [int(t) for t in times_arr.tolist()]
        if any(not (-_INT64_MAX <= t <= _INT64_MAX) for t in times_ns):
            raise TrajectoryValidationError("times exceed int64 range")
        if any(times_ns[i] <= times_ns[i - 1] for i in range(1, n)):
            raise TrajectoryValidationError("times must be strictly increasing (ns, relative)")

        vlim = self._cfg.velocity_limits
        peak = np.max(np.abs(speed_scale * qd_arr), axis=0)
        if np.any(peak > vlim + 1e-9):
            over = np.where(peak > vlim + 1e-9)[0].tolist()
            raise TrajectoryValidationError(
                f"|s·qd| exceeds velocity limit on joint(s) {over}: peak={peak.tolist()} vs {vlim.tolist()}"
            )
        return times_ns, q_arr, qd_arr

    def _emergency_cleanup(self) -> None:
        """Best-effort teardown when the constructor fails partway (never raises)."""
        for label, fn in (
            ("supervisor", lambda: self._supervisor.shutdown() if self._supervisor else None),
            ("core", lambda: self.core.stop() if self.core and self.core.running else None),
            ("rmi", lambda: self.rmi.stop() if self.rmi else None),
            ("ownership", lambda: self._ownership.release() if self._ownership else None),
        ):
            try:
                fn()
            except Exception as exc:  # noqa: BLE001
                logger.debug("airo_fanuc: emergency cleanup of %s failed: %s", label, exc)


def _run_with_timeout(fn: Any, timeout_s: float) -> bool:
    """Run ``fn()`` in a daemon thread; return True if it finished within
    ``timeout_s``, else False (the thread is abandoned — the caller decides how to
    surface the wedge; the process is never killed)."""
    done = threading.Event()

    def _run() -> None:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            logger.debug("airo_fanuc: teardown step raised: %s", exc)
        finally:
            done.set()

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return done.wait(timeout_s)
