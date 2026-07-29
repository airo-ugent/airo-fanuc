# SPDX-License-Identifier: Apache-2.0
"""Driver configuration + policy for the ur_rtde-shaped :class:`~airo_fanuc.driver.FanucDriver`.

Two dataclasses (PLAN.md §5.1):

* :class:`DriverConfig` — the *facts*: controller endpoints, kinematic limits and
  RT-hygiene knobs. Its limit defaults are single-sourced from
  :mod:`airo_fanuc.controller_facts` (a unit test asserts they stay equal), and it
  produces the C++ :class:`airo_fanuc._core.RtCoreConfig` via
  :meth:`DriverConfig.to_rt_core_config`.
* :class:`DriverPolicy` — the *behaviour*: bring-up retries, auto-recovery, the
  ARM gate, the CAPTURE collision-check hook, ownership, the optional republisher.

:class:`MotionResult` is the non-raising airo-convention result a
:class:`~airo_fanuc.driver.MotionHandle` resolves to.
"""

from __future__ import annotations

import os
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from . import controller_facts as cf
from ._core import RtCoreConfig
from .controller_facts import SettlePolicy
from .ownership import DEFAULT_LOCK_PATH
from .republisher import Publisher

__all__ = [
    "DEFAULT_RMI_PORT",
    "DEFAULT_SM_PORT",
    "CaptureCheck",
    "DriverConfig",
    "DriverPolicy",
    "MotionResult",
    "SettlePolicy",
]

#: Stream Motion UDP port (J519) and RMI bootstrap TCP port (R912) — fixed on the
#: R-30iB controller; overridable only so tests can point at a FakeCRX on :0.
DEFAULT_SM_PORT: int = 60015
DEFAULT_RMI_PORT: int = 16001

#: The CAPTURE collision-check hook (decision 6 / R3 A2). Grocery wires this to
#: curobo's ``check_trajectory_collision``; it receives the *synthesized capture
#: path* (the exact knots the C++ core will execute — see
#: :func:`airo_fanuc._core.generate_capture_path`) and returns ``True`` iff the
#: splice is collision-free / safe to execute. The wheel never imports curobo:
#: collision-checking is the caller's responsibility. ``None`` = no check.
CaptureCheck = Callable[[np.ndarray, np.ndarray], bool]


class MotionResult(Enum):
    """Terminal outcome of a :class:`~airo_fanuc.driver.MotionHandle` (non-raising,
    airo-convention; maps 1:1 from the terminal :class:`airo_fanuc._core.MotionStatus`
    ordinals plus the ``stop_j``/preempt paths, PLAN.md §5.1)."""

    DONE = "done"
    SETTLE_TIMEOUT = "settle_timeout"
    STOPPED = "stopped"
    PREEMPTED = "preempted"
    FAULTED = "faulted"
    REJECTED = "rejected"


@dataclass
class DriverConfig:
    """Controller endpoints, kinematic limits and RT-hygiene knobs.

    Limit defaults are single-sourced from :mod:`airo_fanuc.controller_facts`
    (``tests/test_lifecycle.py`` asserts equality — PLAN.md §5.1 "limits are
    single-sourced"). The C++ tick engine carries a mirror copy of the same
    numbers in ``TickEngineConfig``; :meth:`to_rt_core_config` only sets the
    :class:`airo_fanuc._core.RtCoreConfig` fields that are actually exposed to
    Python (the tick-engine limits/capture knobs stay at their C++ defaults,
    which equal these).
    """

    # -- endpoints -------------------------------------------------------
    sm_port: int = DEFAULT_SM_PORT
    rmi_port: int = DEFAULT_RMI_PORT
    rmi_connect_timeout: float = 5.0
    rmi_request_timeout: float = 2.0

    # -- kinematic limits (rad, rad/s, rad/s², rad/s³) — controller_facts ------
    velocity_limits: np.ndarray = field(default_factory=lambda: cf.CRX10IAL_VELOCITY_LIMITS.copy())
    acceleration_limits: np.ndarray = field(default_factory=lambda: cf.CRX10IAL_ACCELERATION_LIMITS.copy())
    jerk_limits: np.ndarray = field(default_factory=lambda: cf.CRX10IAL_JERK_LIMITS.copy())

    # -- brake / slew envelope (provenance; mirror of controller_facts) --------
    stop_scale_va: float = cf.STOP_LIMIT_SCALE_VA
    stop_scale_j: float = cf.STOP_LIMIT_SCALE_J
    slew_factor: float = cf.SLEW_FACTOR
    tracking_lag_s: float = cf.INTERIM_FACTS.tracking_lag_s

    # -- in-process safety watchdogs (P-1 finalization; controller_facts) --------
    #: SUPERVISOR_LOST hold if the supervisor heartbeat lapses this long (s).
    supervisor_lost_s: float = cf.SUPERVISOR_LOST_S
    #: DRIFT fault: sustained commanded↔measured divergence > this many degrees...
    drift_fault_deg: float = cf.DRIFT_FAULT_DEG
    #: ...for this many consecutive fresh-RX ticks (the 22°-runaway guard).
    drift_fault_ticks: int = cf.DRIFT_FAULT_TICKS

    # -- Stream Motion protocol / RT hygiene -----------------------------
    #: Negotiated Stream Motion version (3 = no force config, 4 = FSConfig/force).
    sm_version: int = 3
    preroll_timeout_s: float = 5.0
    #: GRIPDISP liveliness-probe ceiling (s) for the cross-process anti-stacking
    #: gate in the bring-up ladder (supervisor ``_gripdisp_alive``). Before forking
    #: GRPRUN, the supervisor issues a benign OPEN and waits up to this long for the
    #: dispatcher to auto-clear REG_CMD (proving it is already running → skip the
    #: fork). A live dispatcher returns as soon as REG_CMD clears (well under this);
    #: only a genuinely-absent one waits it out. Deliberately ≥ the gripper dispatch
    #: timeout (``gripper_worker.GRIPPER_DISPATCH_TIMEOUT_S`` = 5 s): a false "absent"
    #: verdict re-forks and re-creates the un-killable stacking wedge, so err toward
    #: waiting. Python-only (not mirrored into the RT core).
    gripdisp_probe_timeout_s: float = 6.0
    #: RT-thread isolation. Default OFF (test/dev safe; pthread affinity EINVALs
    #: outside the process cgroup cpuset). Production/P5 enables via the grocery
    #: entry point once the ``grocery-demo.slice`` reservation is in place.
    rt_core: int = 31
    rt_priority: int = 80
    pin_core: bool = False
    sched_fifo: bool = False
    mlock: bool = False

    def to_rt_core_config(self) -> RtCoreConfig:
        """Build the C++ :class:`airo_fanuc._core.RtCoreConfig`.

        Starts from the shipped C++ defaults (which mirror
        :mod:`airo_fanuc.controller_facts`) and overrides only the protocol +
        RT-hygiene knobs Python owns. The capture/brake/settle/limit knobs live
        inside the (Python-opaque) embedded ``TickEngineConfig`` and are left at
        their C++ defaults so the executed capture path equals the one
        :func:`airo_fanuc._core.generate_capture_path` synthesizes.
        """
        rc = RtCoreConfig()
        rc.sm_version = int(self.sm_version)
        # In-process safety watchdogs (P-1 finalization). drift_lag_ticks is derived
        # from the measured servo lag (E9: 25 ms) so the drift guard is lag-aligned.
        rc.supervisor_lost_s = float(self.supervisor_lost_s)
        rc.drift_lag_ticks = int(round(self.tracking_lag_s / cf.ITP_S))
        rc.drift_fault_rad = float(np.deg2rad(self.drift_fault_deg))
        rc.drift_fault_ticks = int(self.drift_fault_ticks)
        rc.preroll_timeout_s = float(self.preroll_timeout_s)
        rc.rt_core = int(self.rt_core)
        rc.rt_priority = int(self.rt_priority)
        rc.pin_core = bool(self.pin_core)
        rc.sched_fifo = bool(self.sched_fifo)
        rc.mlock = bool(self.mlock)
        return rc


@dataclass
class DriverPolicy:
    """Behavioural policy for the driver + lifecycle supervisor (PLAN.md §5.3).

    Defaults reproduce the ``dries`` demo doctrine: auto-recovery ON, the ARM
    gate armed on operator/e-stop faults, single-owner flock acquired.
    """

    config: DriverConfig = field(default_factory=DriverConfig)

    # -- bring-up --------------------------------------------------------
    connect_retries: int = 3
    #: Grace, post-bring-up, to observe the seqlock publish HOLD after wait_ready
    #: (P3b note: wait_ready flips STREAMING a beat before HOLD is published).
    hold_wait_s: float = 3.0
    preflight_full: bool = False

    # -- recovery / faults ----------------------------------------------
    auto_recover: bool = True
    #: Cooldown between auto-recovery ladder attempts (dries 15 s anti-churn).
    recovery_cooldown_s: float = 15.0
    #: Settle delay before an *auto* recovery attempt (dries MOTION_RECOVERY_DELAY_S
    #: = 3 s; explicit ``recover()`` bypasses it). Tests set this small.
    recovery_delay_s: float = 3.0
    #: Grace for motion_possible / HOLD to reassert after a ladder relaunch.
    ready_wait_s: float = 60.0
    #: SHORT probe (s) the recovery ladder waits for ``motion_possible`` to reassert
    #: after its reset/relaunch before giving up on THAT tier. Kept well under
    #: ``ready_wait_s`` so a fault a plain ``reset + FRC_Call`` cannot clear (e.g. a
    #: ``FRC_SystemFault`` that de-initialised the RMI motion interface) fails the
    #: tier FAST — the driver then escalates to the cold reconnect (full re-bring-up:
    #: re-``initialize`` + GRIPDISP re-fork) instead of stalling ~60 s. HW-observed
    #: 2026-07-17: reset+reseed+FRC_Call alone does NOT re-arm motion after a collision
    #: SystemFault, so a fast escalation is the difference between hands-off recovery
    #: and a wedged idle loop.
    recovery_motion_probe_s: float = 5.0
    #: Full-RMI-reconnect attempts inside the recovery ladder when a SystemFault
    #: leaves RMI unresponsive (FRC_Reset / FRC_Call → RmiSessionDown). The
    #: normal ladder dead-ends on RmiSessionDown; this many times it falls back
    #: to the bring-up "flush stale SystemFault" reconnect (rmi.stop → reconnect
    #: → reset → reseed → FRC_Call). Kept small (1): RMI churn is itself a
    #: documented SM-daemon wedge vector, so this trades one clean reconnect for
    #: the wedge rather than hammering the session. 0 restores the old bail.
    recovery_reconnect_attempts: int = 1
    estop_continuation_path: str = cf.INTERIM_FACTS.estop_continuation_path

    # -- ARM gate (R2 F1) ------------------------------------------------
    #: When True, recovery from an e-stop / operator-required fault ends in
    #: MOTION_INHIBITED — the next motion needs an explicit ``arm()``.
    arm_gate: bool = True

    # -- CAPTURE collision-check hook (decision 6 / R3 A2) --------------
    capture_check: CaptureCheck | None = None

    # -- peripherals -----------------------------------------------------
    enable_gripper: bool = True
    #: Injected zenoh (or any) publisher for the republisher threads. ``None`` =
    #: no republish (grocery provides one at P5).
    publisher: Publisher | None = None

    # -- ownership -------------------------------------------------------
    enable_ownership: bool = True
    ownership_mode: str = "control"
    lock_path: str | os.PathLike[str] = DEFAULT_LOCK_PATH

    # -- end-of-trajectory settle default -------------------------------
    settle: SettlePolicy = field(default_factory=SettlePolicy)

    # -- supervisor loop -------------------------------------------------
    watch_interval_s: float = 0.02
