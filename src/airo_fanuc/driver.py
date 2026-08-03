# SPDX-License-Identifier: Apache-2.0
"""``FanucDriver`` — the construct-and-go public API.

The API takes the shape industrial-arm drivers usually take: a *receive* half (state
getters that never block) and a *control* half (motion submission, usable blocking or
non-blocking), both reachable through one object rather than two session handles. Over
a FANUC arm that means: the constructor brings the robot up to *commandable*
(or raises with a real reason), :meth:`move_trajectory` / :meth:`move_j` / :meth:`servo_j`
are the only motion surfaces, :meth:`stop_j` is the universal preempt, and the getters never raise
and never lie (value + age). The C++ ``StreamCore`` owns the 125 Hz timeline; the
:class:`~airo_fanuc.supervisor.Supervisor` owns lifecycle/recovery policy; this
class is the thin, honest facade over both.

Composition — the collaborators are public attributes deliberately, so a caller can
reach through to the layer it needs instead of waiting for a passthrough wrapper:
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
from . import controller_facts as cf
from ._core import Mode, MotionStatus, StreamCore
from .config import DriverPolicy, MotionResult
from .controller_facts import SettlePolicy
from .exceptions import (
    FanucConnectionError,
    FanucError,
    RejectedStartMismatch,
    RobotFaultedError,
    TrajectoryValidationError,
)
from .gripper_worker import GripperWorker
from .lifecycle import LifecycleState, motion_result_of
from .ownership import OwnershipLock
from .receive_interface import SOURCE_STREAM
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


def _capture_reject_message(
    path: dict[str, Any],
    q_cmd: Sequence[float],
    qd_cmd: Sequence[float],
    q0: Sequence[float],
    qd0: Sequence[float],
) -> str:
    """Format a CAPTURE reject into a message the caller can act on without reading us.

    Every quantity here comes either from the C++ gate's own output — ``tol_exceeded``,
    ``reject_joints``, ``shed_travel``, exported by
    :func:`airo_fanuc._core.generate_capture_path` — or from the caller's own arguments.
    The feasibility condition is NOT recomputed on this side: there is one derivation, in
    ``src/cpp/tick_engine/capture.cpp``, and this function only converts its radians to
    degrees and names the joints. That is deliberate. A Python reimplementation of the
    same closed form would be a second source of truth able to disagree with the gate the
    core actually applies, and the disagreement would surface as a submission the
    pre-flight accepted and the core rejected.
    """
    deg = math.degrees
    if bool(path["tol_exceeded"]):
        worst = max(range(_NDOF), key=lambda j: abs(q_cmd[j] - q0[j]))
        return (
            f"trajectory first knot is beyond the {cf.CAPTURE_TOL_DEG:g}° capture window from "
            f"the commanded pose: joint {worst} is "
            f"{deg(abs(q_cmd[worst] - q0[worst])):.3f}° away, limit {cf.CAPTURE_TOL_DEG:g}°. "
            f"Replan from the commanded pose "
            f"({[round(deg(v), 3) for v in q_cmd]}°)."
        )
    shed = [float(v) for v in cast("list[float]", path["shed_travel"])]
    clauses = [
        f"joint {j} must shed {deg(qd_cmd[j]):+.3f}°/s → {deg(qd0[j]):+.3f}°/s, which costs "
        f"{deg(shed[j]):.3f}° of travel"
        for j in (int(x) for x in cast("list[int]", path["reject_joints"]))
    ]
    return (
        "trajectory first knot is unreachable from the commanded pose: "
        + "; ".join(clauses)
        + f" — more than the {cf.CAPTURE_TOL_DEG:g}° capture window is wide. The splice runs "
        f"at the brake-class acceleration and jerk clamps, so no profile can absorb that "
        f"velocity change inside the window: it would have to leave and come back, sweeping "
        f"travel the window never bounded. Submit the trajectory from rest, or match its "
        f"first knot's velocity to the commanded one."
    )


#: Axis labels published alongside the joint stream: the pendant's own J1..J6, in
#: Stream Motion wire order. This package ships no URDF, so they are labels for a
#: consumer to map, not URDF joint names.
_JOINT_NAMES: tuple[str, ...] = tuple(f"J{i}" for i in range(1, _NDOF + 1))


def _republish_block(snap: dict[str, Any]) -> dict[str, Any]:
    """Seqlock snapshot → the republisher's snapshot keys (degrees, wall-clock ns).

    ``rx_mono_ns`` is CLOCK_MONOTONIC at packet ingest, while the published ages are
    wall-clock, so ``t_meas_ns`` is that same instant converted through one offset
    read — both clocks sampled now, the way ``RealtimeCore::joints_at_wall`` does it.
    Publishing the monotonic stamp unconverted would report an age of the host's
    uptime. Wall clock is also the clock a camera stamps a grab with, which is what
    makes a published pose pairable with an image.

    The joint block is gated on ``rx_mono_ns > 0``: before the first status packet it
    is still zero-initialised, and all-zero joints are a pose rather than a "no data"
    marker — the same gate :meth:`FanucDriver.get_flange_pose` applies.
    """
    rx_mono_ns = int(snap["rx_mono_ns"])
    have_rx = rx_mono_ns > 0
    wrench: dict[str, Any] | None = None
    if bool(snap["wrench_valid"]):
        fx, fy, fz = float(snap["fx"]), float(snap["fy"]), float(snap["fz"])
        wrench = {
            "force": [fx, fy, fz],
            "moment": [float(snap["mx"]), float(snap["my"]), float(snap["mz"])],
            "magnitude_n": math.hypot(fx, fy, fz),
        }
    return {
        "q_meas_deg": np.degrees(snap["q_meas"]).tolist() if have_rx else None,
        "qd_meas_deg": np.degrees(snap["qd_est"]).tolist() if have_rx else None,
        "joint_names": list(_JOINT_NAMES),
        "t_meas_ns": (time.time_ns() - (time.monotonic_ns() - rx_mono_ns)) if have_rx else None,
        "source": SOURCE_STREAM,
        "e_stopped": bool(snap["e_stopped"]),
        "in_error": bool(snap["in_error"]),
        "tp_enabled": bool(snap["tp_enabled"]),
        "motion_possible": bool(snap["motion_possible"]),
        "motion_in_progress": bool(snap["motion_in_progress"]),
        "contact_stop_status": int(snap["contact_stop_status"]),
        "safety_scale": float(snap["safety_scale"]),
        # RX_SILENT is the core's parked mode, entered together with the RX_SILENT
        # fault (tick_core.enter_rx_silent).
        "rx_silent": Mode(int(snap["mode"])) == Mode.RX_SILENT,
        "command_epoch": int(snap["epoch"]),
        "wrench": wrench,
    }


class MotionHandle:
    """Non-raising handle over one submitted motion.

    Maps the terminal C++ :class:`MotionStatus` (via
    :func:`airo_fanuc.lifecycle.motion_result_of`) to a :class:`MotionResult`.
    ``stop_j()`` during a blocking :meth:`wait` resolves it as ``STOPPED`` (the
    core flips the active motion's status; :meth:`wait` observes it): a preempt
    brakes and returns control to the caller to replan — it is never an exception.
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
    """Construct-and-go FANUC driver: receive + control in one object.

    ``policy`` is required, because the :class:`~airo_fanuc.config.DriverConfig`
    inside it carries the arm's :class:`~airo_fanuc.robot_profile.RobotProfile` — the
    limits the RT core clamps against. There is no default arm: this driver has no way
    to ask the controller which robot is attached, so the caller states it.
    ``examples/crx10ial.py`` builds a profile for the CRX-10iA/L the examples run on.
    """

    def __init__(self, ip: str, policy: DriverPolicy) -> None:
        self._ip = ip
        self._policy = policy
        self._cfg = self._policy.config
        self._closed = False
        self._close_lock = threading.Lock()

        # Built once and retained, not rebuilt per use: the CAPTURE collision check
        # must synthesize its splice with the exact knobs the core runs with, or it
        # would be checking a path the core does not execute.
        self._rt_cfg = self._cfg.to_rt_core_config()

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
            self.core = StreamCore(ip, self._cfg.sm_port, self._rt_cfg)
            self._supervisor = Supervisor(self.core, self.rmi, self._policy)
            self._preflight_report = self._supervisor.bringup()  # raises FanucConnectionError/Preflight
            self._verify_controller_itp()
            self._supervisor.start_watch()

            if self._policy.enable_gripper:
                self.gripper = GripperWorker(self.rmi)
                # Let the supervisor fail-fast-gate the gripper during a recovery
                # ladder — a gripper command must never actuate GRIPDISP
                # mid-recovery (e.g. just after an e-stop release).
                self._supervisor.set_gripper(self.gripper)
            if self._policy.publisher is not None:
                self._republisher = Republisher(
                    self._policy.publisher,
                    self._republish_snapshot,
                    joints_at_wall=self.joints_at_wall,
                    ext_status=self._read_ext_status,
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
        settle: SettlePolicy | None = None,
        deadman_s: float | None = None,
        force_stop_n: float | None = None,
        plan_tick: int | None = None,
        asynchronous: bool = False,
    ) -> MotionHandle:
        """Submit ONE whole trajectory (rad, ns-relative int64 times) — CAPTURE-or-REJECT
        splice, Hermite playback, settle → :class:`MotionHandle`.

        ``plan_tick`` declares WHICH commanded state the first knot was built from: the
        ``cmd_tick`` of the snapshot it was read from. Given one, the core joins the plan
        at the phase the elapsed ticks imply instead of splicing back to a knot the arm
        has already passed — which is what makes a replan while moving land smoothly
        rather than as a bridge back to a stale pose. Omit it when the first knot is meant
        literally (a plan in absolute joint space, or a replay); the core then joins at
        knot 0. It is not inferred here, because only the caller knows which state its
        planner started from.

        Validation — every violation raises its own typed error naming the offending
        joint/knot, never a generic reject: strictly-increasing int64 ns times, ≥2
        knots, finite q/qd, ``|qd| ≤ v_lim``, and a first knot inside the capture
        envelope. The CAPTURE collision-check hook runs when ``policy.capture_check``
        is set.

        ``force_stop_n`` (N) and ``deadman_s`` (s) are optional guards, and both must be
        finite and > 0 when given. A threshold that cannot trip is rejected rather than
        armed. ``force_stop_n`` additionally needs live force telemetry, which a v3 /
        type-202 controller does not provide.

        There is no speed-scale knob: the trajectory's own ``times`` and ``qd`` ARE the
        speed. To replay a plan slower, stretch it caller-side — ``times / s`` with
        ``qd * s`` for ``s < 1`` — which leaves the position path identical. That has to
        happen caller-side, because scaling here could only scale playback: the capture
        splice is built from the first knot, so a scaled playback bridged from an
        unscaled first knot steps the commanded velocity at the handover.
        """
        self._require_commandable()
        assert self.core is not None
        # Both guards are armed in C++ by a bare `> 0.0` test and tripped by `>` against
        # the measured quantity (tick_core.cpp), so 0.0 is the disarmed encoding — which
        # is what None maps to below. A negative or NaN threshold fails that same arming
        # comparison, and +inf arms a guard no finite reading reaches. Each of those asks
        # for a guard and would silently get one that cannot fire, so each is refused at
        # the call rather than accepted: an inert safety guard is worse than none, because
        # the caller believes it has one.
        for _name, _value in (("force_stop_n", force_stop_n), ("deadman_s", deadman_s)):
            if _value is not None and not (math.isfinite(_value) and _value > 0.0):
                raise TrajectoryValidationError(
                    f"{_name} must be finite and > 0, got {_value}. This value would arm "
                    f"nothing, or arm a guard nothing can reach. Pass None (the default) "
                    f"to run without it."
                )
        if force_stop_n is not None and not self._force_telemetry_available():
            raise TrajectoryValidationError(
                "force_stop_n was requested but this controller provides no force telemetry "
                "(Stream Motion v3 / type-202: fs_type unavailable, wrench invalid). The C++ "
                "force-guard cannot trip without a wrench, so arming it would be a silent "
                "safety no-op. Rely on the controller's collaborative contact-stop for contact "
                "detection (it halts the descent; the motion resolves FAULTED with reason "
                "CONTACT_STOP), or upgrade to a v4 / type-204 controller for a numeric force "
                "threshold. See controller-notes.md §1.8."
            )
        times_ns, q_arr, qd_arr = self._validate_trajectory(times, q, qd)
        settle = settle if settle is not None else self._policy.settle

        self._capture_gate(q_arr, qd_arr, asynchronous)

        mid = self.core.submit_trajectory(
            times_ns,
            q_arr.tolist(),
            qd_arr.tolist(),
            1.0,  # speed_scale: pinned. The trajectory's own times/qd are the speed.
            settle_tol_rad=math.radians(settle.tol_deg),
            settle_vel_eps_rad_s=math.radians(settle.vel_eps_deg_s),
            settle_timeout_s=float(settle.timeout_s),
            force_stop_n=float(force_stop_n) if force_stop_n is not None else 0.0,
            deadman_s=float(deadman_s) if deadman_s is not None else 0.0,
            plan_tick=int(plan_tick) if plan_tick is not None else 0,
        )
        handle = MotionHandle(self.core, mid, time.monotonic_ns())
        assert self._supervisor is not None
        self._supervisor.set_active_handle(handle)
        return handle

    def move_j(
        self,
        q: Sequence[float] | np.ndarray,
        *,
        joint_speed: float | None = None,
        settle: SettlePolicy | None = None,
        deadman_s: float | None = None,
        force_stop_n: float | None = None,
        asynchronous: bool = False,
    ) -> MotionHandle:
        """Point-to-point move to a joint configuration (rad) → :class:`MotionHandle`.

        The convenience :meth:`move_trajectory` lacks: give it a target pose and a
        speed, and it plans the profile to get there. It shapes a jerk-limited
        point-to-point profile from the current COMMANDED state to ``q`` with
        :func:`airo_fanuc._core.plan_joint_move` (offline Ruckig, under this driver's
        own limits) and submits the resulting knots through :meth:`move_trajectory`, so
        it inherits the capture gate, the collision hook, the settle policy and the
        handle unchanged.

        ``joint_speed`` (rad/s) is the LEADING-AXIS speed, the airo-robots convention:
        it caps every joint and the profile is time-synchronized, so the joint with the
        furthest to travel runs at ``joint_speed`` and the others scale down to land
        with it. ``None`` uses ``MOVEJ_DEFAULT_SPEED_FRACTION`` of the profile's
        slowest joint. Acceleration and jerk are not arguments: they come from
        ``config.movej_scale_a`` / ``movej_scale_j`` as fractions of the arm's limits,
        because a plan shaped at the limits themselves is one the tick engine clips.

        The plan is anchored at ``q_cmd``, not ``q_meas``. Anchoring at the measured
        pose would fold the servo tracking lag into the first knot as a step, and the
        capture splice bridges from the commanded pose regardless.

        Raises :class:`TrajectoryValidationError` for a target outside the profile's
        position limits (the core would silently CLAMP it, and a clamped MoveJ reports
        DONE somewhere other than where it was asked to go), for a speed above the
        arm's limits, and for a start already moving faster than the capture envelope
        — the splice cannot reach such a first knot, whatever the arm is doing. Brake
        first (:meth:`stop_j` then :meth:`wait_until_steady`) in that last case; this
        method does not brake on the caller's behalf, so it never preempts a motion
        the caller did not know was running.
        """
        self._require_commandable()
        assert self.core is not None

        q_arr = np.asarray(q, dtype=np.float64).reshape(-1)
        if q_arr.shape[0] != _NDOF:
            raise TrajectoryValidationError(f"move_j expects {_NDOF} joint values, got {q_arr.shape[0]}")
        if not np.all(np.isfinite(q_arr)):
            raise TrajectoryValidationError("move_j target is not finite")

        profile = self._cfg.profile
        below = np.where(q_arr < profile.position_limits_lower)[0].tolist()
        above = np.where(q_arr > profile.position_limits_upper)[0].tolist()
        if below or above:
            raise TrajectoryValidationError(
                f"move_j target is outside the arm's position limits on joint(s) "
                f"{sorted(below + above)}: {np.degrees(q_arr).round(3).tolist()}° against "
                f"[{profile.position_limits_lower_deg.round(3).tolist()}, "
                f"{profile.position_limits_upper_deg.round(3).tolist()}]°"
            )

        vlim = profile.velocity_limits
        if joint_speed is None:
            speed = float(np.min(vlim)) * cf.MOVEJ_DEFAULT_SPEED_FRACTION
        else:
            speed = float(joint_speed)
            if not (speed > 0.0 and math.isfinite(speed)):
                raise TrajectoryValidationError(f"move_j joint_speed must be finite and > 0, got {speed}")
            if speed > float(np.min(vlim)) + 1e-9:
                raise TrajectoryValidationError(
                    f"move_j joint_speed {speed:.4f} rad/s ({math.degrees(speed):.2f}°/s) exceeds the "
                    f"slowest joint's velocity limit {float(np.min(vlim)):.4f} rad/s "
                    f"({math.degrees(float(np.min(vlim))):.2f}°/s). It is a LEADING-AXIS speed, so it "
                    f"must be reachable by every joint."
                )

        snap = _snap(self.core)
        q_cmd = [float(v) for v in snap["q_cmd"]]
        qd_cmd = [float(v) for v in snap["qd_cmd"]]
        capture_rate = float(np.deg2rad(cf.CAPTURE_RATE_DEG_S))
        moving = np.abs(np.asarray(qd_cmd, dtype=np.float64))
        if np.any(moving > capture_rate * (1.0 + 1e-9)):
            over = np.where(moving > capture_rate * (1.0 + 1e-9))[0].tolist()
            raise TrajectoryValidationError(
                f"move_j cannot start while joint(s) {over} are moving faster than the "
                f"{cf.CAPTURE_RATE_DEG_S:g}°/s capture envelope "
                f"({np.rad2deg(moving).round(3).tolist()}°/s). The capture splice that bridges the "
                f"commanded pose to the plan's first knot cannot reach that velocity. Call stop_j() "
                f"and wait_until_steady() first, then plan from rest."
            )

        try:
            plan = cast(
                "dict[str, Any]",
                _core.plan_joint_move(
                    q_cmd,
                    qd_cmd,
                    q_arr.tolist(),
                    self._rt_cfg,
                    max_velocity_rad_s=speed,
                    accel_scale=float(self._cfg.movej_scale_a),
                    jerk_scale=float(self._cfg.movej_scale_j),
                    # The commanded acceleration this plan starts under. Seeding it is what
                    # makes the plan continue the arm's current motion instead of starting
                    # from a curvature the arm does not have.
                    qdd0=[float(v) for v in snap["qdd_cmd"]],
                ),
            )
        except (ValueError, RuntimeError) as exc:
            raise TrajectoryValidationError(
                f"move_j could not plan a profile to {q_arr.tolist()}: {exc}"
            ) from exc

        # The plan is anchored at the commanded state `snap` carries, so its tick is the
        # one the core needs to know which phase of this plan belongs to the tick that
        # consumes it.
        return self.move_trajectory(
            plan["times_ns"],
            plan["q"],
            plan["qd"],
            settle=settle,
            deadman_s=deadman_s,
            force_stop_n=force_stop_n,
            plan_tick=int(snap["cmd_tick"]),
            asynchronous=asynchronous,
        )

    def servo_j(
        self,
        q: Sequence[float] | np.ndarray,
        duration: float,
        *,
        qd: Sequence[float] | np.ndarray | None = None,
        qdd: Sequence[float] | np.ndarray | None = None,
    ) -> MotionHandle:
        """Replace-not-queue servo target (rad): head for ``q``, with ``duration``
        seconds to get there. Best-effort and starvation-safe.

        "Best-effort" is the contract, in the sense UR's ``servoj`` means it: the core
        tracks toward the target under the servo velocity/acceleration/jerk limits and
        does not promise to arrive on time, at rest, or at any particular velocity. It
        never refuses a target for being far away either — a distant or stale setpoint
        produces bounded, jerk-shaped motion toward it, not a rejection, so a stream
        that falls behind keeps going instead of stopping. ``duration`` is the spacing
        between successive calls (``1/f`` in the ``servo_j(q, 1/f)`` pattern), not the
        8 ms tick; it stretches the profile so the command glides between targets
        rather than arriving early and dwelling.

        Note what is NOT checked: no collision check runs anywhere on the servo path
        (unlike :meth:`move_trajectory`, whose capture splice goes through
        ``policy.capture_check``), and the only bound on a wrong setpoint is the servo
        limits. Streaming setpoints into an occupied workspace is the caller's risk.

        ``qd`` / ``qdd`` (rad/s, rad/s²) are accepted and CURRENTLY IGNORED. They were
        previously used as Ruckig's target velocity/acceleration; demanding an arrival
        velocity is what made the command reverse against a forward-moving stream when
        the caller's clock and the tick clock drifted (see BEST EFFORT in
        ``src/cpp/tick_engine/servo.hpp``). The arguments are kept because a future
        tracking law can use them as a lookahead — the way UR's ``servoj`` uses velocity
        — rather than as an arrival state.
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
        """Block until :meth:`is_steady`. Returns False on timeout."""
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

        Two tiers: the light supervisor ladder (reset → FRC_Continue →
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

        Returns True iff the driver reached STREAMING. That is NOT the same as
        commandable: an e-stop or a controller alarm latches the ARM gate the moment
        it is observed, so for those faults a fully successful recovery still returns
        True with motion refused until :meth:`arm` is called. Read
        ``get_state()["motion_inhibited"]`` — or just call :meth:`arm` — rather than
        reading a False here as a failed recovery."""
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
        # STREAMING, not is_commandable(): the latter also folds in the ARM gate, which
        # is exactly what stays set after an e-stop recovery — reporting that as a
        # failed reconnect would tell the caller to give up on a robot that is fine and
        # merely waiting for arm().
        return self._supervisor.state() is LifecycleState.STREAMING

    def reconnect(self) -> None:
        """Cold re-bring-up: quiesce, tear the SM/RMI session down, run the full
        bring-up ladder again. Raises :class:`FanucConnectionError` on failure.

        Also re-establishes the **GRIPDISP gripper dispatcher**: bring-up's
        ``_gripdisp_alive`` probe re-forks GRPRUN when the prior dispatcher was
        aborted/wedged (e.g. a collision that fired mid-close left ``R[1]`` set), and
        its benign OPEN neutralises the stale trigger + clears ``R[1]`` so the gripper
        comes back usable and open. The gripper fail-fast gate is held across the
        rebuild so no gripper command actuates GRIPDISP mid-reconnect; the
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

    def _republish_snapshot(self) -> dict[str, Any]:
        """The :class:`~airo_fanuc.republisher.Republisher` snapshot contract (never raises).

        Separate from :meth:`get_state`, which reports the core's own names and units —
        radians, monotonic stamps, the raw force fields. The published topics are
        degrees, wall-clock nanoseconds and a wrench block, so the conversion lives
        here instead of widening the getter every other caller reads.
        """
        state: dict[str, Any] = {}
        core = self.core
        if core is not None:
            try:
                state.update(_republish_block(_snap(core)))
                state["tick_p99_ms"] = self.timing_stats().get("tx_interval_p99_ms")
            except Exception as exc:  # noqa: BLE001 - telemetry never raises
                logger.debug("airo_fanuc: republish snapshot assembly failed: %s", exc)
        sup = self._supervisor
        if sup is not None:
            life = sup.lifecycle_snapshot()
            for key in ("lifecycle_state", "fault_reason", "faulted", "operator_hint", "recovery_count"):
                state[key] = life.get(key)
        state["owner"] = self._owner_record()
        return state

    def _read_ext_status(self) -> dict[str, Any] | None:
        """RMI extended status for the 1 Hz ``fanuc/ext_status`` topic, or ``None``.

        Only while STREAMING. This is a telemetry read on the one RMI session the
        bring-up and recovery ladders also use, and ``RmiClient`` holds its request
        lock across a reopen ladder, so a poll issued while the session is being torn
        down or rebuilt would both compete with FRC_Reset / FRC_Call for that lock and
        be able to reopen the session under them. Reads at this rate are measured safe
        during live motion (``controller_facts.MEASURED_FACTS.rmi_reads_ok_in_t1``:
        450/450 at 15 Hz).
        """
        rmi = self.rmi
        sup = self._supervisor
        if rmi is None or sup is None or sup.state() is not LifecycleState.STREAMING:
            return None
        try:
            ext = rmi.get_extended_status(quiet=True)
        except Exception as exc:  # noqa: BLE001 - telemetry never raises
            logger.debug("airo_fanuc: ext-status poll failed: %s", exc)
            return None
        return {
            "gen_override_pct": ext.gen_override_pct,
            "speed_clamp_limit_pct": ext.speed_clamp_limit_pct,
            "control_mode": ext.control_mode,
            "in_motion": ext.in_motion,
            "drives_powered": ext.drives_powered,
            "t_read_ns": time.time_ns(),
        }

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

    def get_tcp_pose(self) -> np.ndarray | None:
        """The controller's own **tool-tip** pose ``[X, Y, Z, W, P, R]`` (mm, deg), or
        ``None`` if the controller could not be asked.

        This is the TCP of whatever tool the control box has configured — the number
        the controller computes itself, with its active UTOOL applied, not a pose this
        driver derived. Nothing about the tool is hardcoded here or in
        :class:`~airo_fanuc.config.DriverConfig`: change the UTOOL entry on the
        pendant and this follows, because the controller is the one doing the
        arithmetic.

        **It costs an RMI round trip (tens of ms) and is not on the 125 Hz timeline.**
        That makes it the one getter on this class that blocks, so it does not belong
        in a control loop — see :meth:`get_flange_pose` for the streamed pose, which
        is a lock-free snapshot read sampled on the same packet as ``q_meas``. The
        difference between them is the tool offset: 175.000 mm on this cell's Robotiq
        gripper (``docs/controller-notes.md`` §1.10).

        Sourced from ``FRC_ReadCartesianPosition`` (RMI §2.3.14), which also names the
        UFRAME/UTOOL the pose is expressed in; call
        :meth:`airo_fanuc.rmi_client.RmiClient.read_cartesian_position` directly when
        you need those numbers or the arm's configuration branch alongside the pose.

        Honest failure: ``None`` when there is no RMI session or the read fails, never
        a stale or substituted value. In particular it does **not** fall back to the
        streamed faceplate pose — silently returning a point 175 mm from the tool tip
        under the same method name is exactly the lie the getters must not tell.

        Extended axes are dropped (6-DOF arm; the controller zero-pads them).
        """
        rmi = self.rmi
        if rmi is None:
            return None
        try:
            pose = rmi.read_cartesian_position()
        except Exception as exc:  # noqa: BLE001 - getters never raise
            logger.debug("airo_fanuc: read_cartesian_position failed: %s", exc)
            return None
        return np.asarray(pose.xyzwpr[:_NDOF], dtype=np.float64)

    def get_flange_pose(self) -> np.ndarray | None:
        """Streamed **faceplate** pose ``[X, Y, Z, W, P, R]`` (mm, deg), or ``None``
        before the first status packet has landed.

        Read straight out of the Stream Motion status packet's ``position`` block —
        the controller's own FK, computed on the same packet as ``q_meas``, so the
        pose and the joints are the same instant with no interpolation skew between
        them. Lock-free, never blocks; age comes from ``get_state()["rx_age_ms"]``
        like every other snapshot value. Present at both v3/type-202 and v4/type-204:
        the block sits in the shared header ahead of the force fields, so unlike
        :meth:`get_wrench` this is available on *this* controller.

        **This is the faceplate, NOT the tool tip** — measured, ``controller-notes.md``
        §1.10: against ``FRC_ReadCartesianPosition`` at one standstill pose the
        orientation was bit-identical but the position sat exactly 175.000 mm short
        along tool +Z, which is this cell's Robotiq gripper. Use :meth:`get_tcp_pose`
        for the tool tip. If you need a TCP at tick rate rather than per RMI round
        trip, apply your own tool transform to this pose — the driver ships no pose
        algebra, because the controller already does the conversion for the one caller
        that just wants the answer.

        **Wire units, not SI**: millimetres and degrees, exactly what the pendant's
        POSITION screen shows, which is what makes the two comparable by eye. W/P/R
        compose as fixed-axis XYZ, ``R = Rz(R)·Ry(P)·Rx(W)`` — resolved on hardware by
        that 175 mm lever arm to better than 0.1° (§1.10), not assumed. That is the
        convention to use if you do interpret these angles yourself.

        ``None`` before the first RX is deliberate. The snapshot's pose field starts
        zero-initialised, and all-zero XYZWPR is not a pose this arm can hold — it
        would put the flange at the world origin, inside the robot base — so returning
        it would be a getter lying. The gate is on ``rx_mono_ns``, not on the value,
        so a real pose is never mistaken for no-data.

        Extended axes (``cart[6:9]``) are dropped: this arm is 6-DOF and the
        controller zero-pads them. Reach into ``get_state()["cart"]`` if you need the
        raw length-9 block.

        **Open (§1.10a):** whether this field tracks the *active* UTOOL. The packet
        carries no frame tag, and ``FRC_GetStatus`` and ``FRC_ReadCartesianPosition``
        report *different* active UFRAME/UTOOL numbers on this controller. If the
        streamed pose follows the active tool, the flange/TCP gap changes silently
        when someone switches UTOOL at the pendant.
        """
        core = self.core
        if core is None:
            return None
        snap = _snap(core)
        if int(snap.get("rx_mono_ns", 0)) <= 0:
            return None  # no status packet yet — the pose field is still zeros
        cart = snap.get("cart")
        if not isinstance(cart, list) or len(cart) < _NDOF:
            return None
        return np.array(cart[:_NDOF], dtype=np.float64)

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
        """Ordered, poison-not-exit shutdown.

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

    def _verify_controller_itp(self) -> None:
        """Check the configured interpolation period against the controller's own.

        The controller states its interpolation period in the GetCapability reply, in
        whole milliseconds. Every per-tick quantity in the core is scaled by
        ``config.itp_s`` — the slew clip, the brake and settle windows, each mode's
        Ruckig period — so a driver configured for one period against a controller
        running another produces limits that are silently wrong by that ratio, in the
        permissive direction if the real period is shorter. Bring-up refuses instead.

        A reported 0 means no capability reply was seen; that is not treated as a
        mismatch here, because a controller that never completed the handshake fails
        earlier and more informatively on the preroll timeout.
        """
        assert self.core is not None
        reported_ms = int(self.core.sm_sampling_rate_ms)
        if reported_ms == 0:
            return
        configured_ms = self._cfg.itp_s * 1000.0
        # The wire field is integer milliseconds, so allow the rounding it implies and
        # nothing more.
        if abs(configured_ms - reported_ms) > 0.5:
            raise FanucConnectionError(
                f"controller reports a {reported_ms} ms interpolation period, but the driver is "
                f"configured for {configured_ms:.3f} ms (DriverConfig.itp_s={self._cfg.itp_s}). "
                f"Every per-tick limit is scaled by itp_s, so continuing would apply limits "
                f"computed for the wrong period. Set DriverConfig.itp_s to {reported_ms / 1000.0}."
            )

    def _capture_gate(self, q_arr: np.ndarray, qd_arr: np.ndarray, asynchronous: bool) -> None:
        """CAPTURE-or-REJECT + collision-check hook.

        Synthesizes the EXACT capture path the C++ core will execute via
        :func:`airo_fanuc._core.generate_capture_path` (one code path — "the
        checked path IS the executed path"). Rejected by the gate → synchronous
        :class:`RejectedStartMismatch` (async submissions let the core resolve
        REJECTED). The gate is the 5° endpoint window AND whether that window can absorb
        the first knot's velocity change at the brake-class clamps — one closed form,
        evaluated in C++ and reported per joint (see THE CAPTURE ACCEPTANCE GATE in
        ``tick_engine/capture.hpp``); this side formats its numbers and derives none.
        When ``policy.capture_check`` is set, the synthesized knots are
        handed to the caller's collision checker; ``False`` → typed reject.
        """
        assert self.core is not None
        snap = _snap(self.core)
        q_cmd = [float(v) for v in snap["q_cmd"]]
        qd_cmd = [float(v) for v in snap["qd_cmd"]]
        q0 = q_arr[0].tolist()
        qd0 = qd_arr[0].tolist()
        path = cast(
            "dict[str, Any]",
            _core.generate_capture_path(q_cmd, qd_cmd, q0, qd0, self._rt_cfg),
        )

        if bool(path["would_reject"]):
            if asynchronous:
                return  # the core resolves this submission as REJECTED
            raise RejectedStartMismatch(_capture_reject_message(path, q_cmd, qd_cmd, q0, qd0))

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
    ) -> tuple[list[int], np.ndarray, np.ndarray]:
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

        vlim = self._cfg.profile.velocity_limits
        peak = np.max(np.abs(qd_arr), axis=0)
        if np.any(peak > vlim + 1e-9):
            over = np.where(peak > vlim + 1e-9)[0].tolist()
            raise TrajectoryValidationError(
                f"|qd| exceeds velocity limit on joint(s) {over}: peak={peak.tolist()} vs {vlim.tolist()}"
            )

        # FIRST-KNOT velocity vs the capture envelope. The core bridges the commanded
        # pose to knot 0 with a capture splice bounded by CAPTURE_RATE_DEG_S, and that
        # bound applies to the splice's ENDPOINT velocities, not to the gap it closes:
        # measured on hardware, a first knot at 15.1°/s is unreachable even when the arm
        # is already travelling at exactly that speed. Left to the core this surfaces as
        # MotionStatus.REJECTED carrying FaultReason.INTERNAL, which tells the caller
        # nothing and reads like a driver bug rather than a trajectory the driver cannot
        # start. Checked here so it is a typed error naming the offending joints and the
        # ceiling. NB this caps the START velocity only — the interior of the trajectory
        # is bounded by the joint velocity limits checked above, so a profile that begins
        # at rest may run as fast as those allow.
        # FIRST-KNOT velocity vs the capture envelope, checked against the same raw qd the
        # core splices to (consume() calls generate_capture_path with the first knot as
        # given) — the "checked path IS the executed path" rule.
        capture_rate = float(np.deg2rad(cf.CAPTURE_RATE_DEG_S))
        qd_first = np.abs(qd_arr[0])
        if np.any(qd_first > capture_rate * (1.0 + 1e-9)):
            over = np.where(qd_first > capture_rate * (1.0 + 1e-9))[0].tolist()
            raise TrajectoryValidationError(
                f"first-knot |qd| exceeds the {cf.CAPTURE_RATE_DEG_S:g}°/s capture envelope on "
                f"joint(s) {over}: {np.rad2deg(qd_first).round(3).tolist()}°/s. The capture splice "
                f"that bridges the commanded pose to knot 0 cannot reach that velocity, whatever "
                f"the arm is currently doing. Start the trajectory at rest, or within the envelope "
                f"— a profile beginning at rest may then run up to the joint velocity limits."
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
