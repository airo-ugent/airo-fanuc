# SPDX-License-Identifier: Apache-2.0
"""Driver configuration + policy for :class:`~airo_fanuc.driver.FanucDriver`.

Two dataclasses:

* :class:`DriverConfig` — the *facts*: controller endpoints, the arm's
  :class:`~airo_fanuc.robot_profile.RobotProfile` and RT-hygiene knobs. It produces
  the C++ :class:`airo_fanuc._core.RtCoreConfig` via
  :meth:`DriverConfig.to_rt_core_config`, which is what carries the profile's limits
  to the tick engine that enforces them.
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
from .gripper import ROBOTIQ_2F85, RegisterGripperProtocol
from .ownership import DEFAULT_LOCK_PATH
from .republisher import Publisher
from .robot_profile import RobotProfile

__all__ = [
    "DEFAULT_RMI_PORT",
    "DEFAULT_SM_PORT",
    "CaptureCheck",
    "DriverConfig",
    "DriverPolicy",
    "MotionResult",
    "RegisterGripperProtocol",
    "RobotProfile",
    "SettlePolicy",
]

#: Stream Motion UDP port (J519) and RMI bootstrap TCP port (R912) — fixed on the
#: R-30iB controller; overridable only so tests can point at a FakeCRX on :0.
DEFAULT_SM_PORT: int = 60015
DEFAULT_RMI_PORT: int = 16001

#: The CAPTURE collision-check hook. A consumer wires this to its own collision
#: query (e.g. curobo's ``check_trajectory_collision``); it receives the
#: *synthesized capture path* (the exact knots the C++ core will execute — see
#: :func:`airo_fanuc._core.generate_capture_path`) and returns ``True`` iff the
#: splice is collision-free / safe to execute. This package deliberately depends on
#: no collision checker: world modelling is the caller's responsibility, and the
#: driver cannot know the scene. ``None`` = no check.
CaptureCheck = Callable[[np.ndarray, np.ndarray], bool]


class MotionResult(Enum):
    """Terminal outcome of a :class:`~airo_fanuc.driver.MotionHandle` (non-raising,
    airo-convention; maps 1:1 from the terminal :class:`airo_fanuc._core.MotionStatus`
    ordinals plus the ``stop_j``/preempt paths)."""

    DONE = "done"
    SETTLE_TIMEOUT = "settle_timeout"
    STOPPED = "stopped"
    PREEMPTED = "preempted"
    FAULTED = "faulted"
    REJECTED = "rejected"


@dataclass
class DriverConfig:
    """Controller endpoints, the arm's motion envelope and RT-hygiene knobs.

    :attr:`profile` is required and has no default: the limits it carries are the
    ceiling the RT core clamps against, this driver cannot verify them, and a default
    would silently apply limits measured on somebody else's arm.
    ``examples/crx10ial.py`` builds one for a CRX-10iA/L.

    :meth:`to_rt_core_config` carries the profile's limits into the C++ tick engine,
    which is what enforces them; the same struct is handed to
    :func:`airo_fanuc._core.generate_capture_path`, so the pre-flight capture check
    and the executed splice are computed from one set of numbers.
    """

    # -- the arm ---------------------------------------------------------
    #: The arm's motion envelope. Keyword-only and required — see the class docstring.
    profile: RobotProfile = field(kw_only=True)

    # -- endpoints -------------------------------------------------------
    sm_port: int = DEFAULT_SM_PORT
    rmi_port: int = DEFAULT_RMI_PORT
    rmi_connect_timeout: float = 5.0
    rmi_request_timeout: float = 2.0

    # -- brake / slew envelope -------------------------------------------------
    #: Fractions of the profile's limits, not limits themselves, so they carry across
    #: arms unchanged and stay in :mod:`airo_fanuc.controller_facts` with the rest of
    #: this driver's tuning. The brake runs at ``stop_scale_va × (v, a)`` and
    #: ``stop_scale_j × j``; the per-tick slew clip is ``slew_factor × v × itp_s``.
    stop_scale_va: float = cf.STOP_LIMIT_SCALE_VA
    stop_scale_j: float = cf.STOP_LIMIT_SCALE_J
    slew_factor: float = cf.SLEW_FACTOR

    # -- MoveJ planning envelope ----------------------------------------------
    #: Fractions of the profile's acceleration / jerk limits that
    #: :meth:`airo_fanuc.driver.FanucDriver.move_j` shapes its point-to-point profile
    #: with. Python-only — not mirrored into the RT core, which clamps against the
    #: full limits; these keep the PLAN clear of that ceiling. A cell that finds the
    #: default too slow raises them here rather than editing the package. See
    #: :mod:`airo_fanuc.controller_facts` for why jerk is scaled harder than
    #: acceleration.
    movej_scale_a: float = cf.MOVEJ_LIMIT_SCALE_A
    movej_scale_j: float = cf.MOVEJ_LIMIT_SCALE_J

    # -- in-process safety watchdogs (mirror of controller_facts) --------------
    #: SUPERVISOR_LOST hold if the supervisor heartbeat lapses this long (s).
    supervisor_lost_s: float = cf.SUPERVISOR_LOST_S

    # -- Stream Motion protocol / RT hygiene -----------------------------
    #: Stream Motion version REQUESTED at handshake. The session adopts the version the
    #: controller reports it supports, so this is the ask and the fallback, not the outcome —
    #: read ``sm_negotiated_version`` for that. No ForceSensorConfig packet is sent at any
    #: version, so no force block is streamed and ``get_wrench()`` stays ``None``.
    sm_version: int = 3
    #: Controller interpolation period in seconds. The R-30iB class runs an 8 ms ITP
    #: (125 Hz), which is the default; a controller with a different period is
    #: configured here rather than by editing the package. Every per-tick quantity is
    #: expressed against it — the slew clip is ``slew_factor · v_lim · itp_s``, and the
    #: brake / settle / RX-silence windows are tick counts — so this value and the
    #: controller's real period must agree. The controller reports its own period in
    #: the GetCapability reply, and bring-up rejects a mismatch (see
    #: :meth:`airo_fanuc.driver.FanucDriver._verify_controller_itp`) rather than running
    #: with per-tick limits scaled to a period the hardware is not using.
    itp_s: float = cf.ITP_S
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
    #: RT-thread hygiene, applied best-effort on the RT thread itself. Both flags
    #: default OFF so an unprivileged process constructs and runs unchanged, and
    #: each tolerates EPERM rather than failing bring-up.
    #:
    #: ``sched_fifo`` raises the 125 Hz thread to ``SCHED_FIFO`` at ``rt_priority``
    #: so it preempts normal-priority threads instead of queueing behind them; the
    #: loop must put a packet on the wire every ITP, and a scheduling delay is a
    #: missed tick. ``mlock`` calls ``mlockall`` so no page of the process can be
    #: swapped out — a major fault on the tick path costs milliseconds.
    #:
    #: Enabling them requires privilege: ``CAP_SYS_NICE`` (or an ``RTPRIO`` rlimit)
    #: for ``sched_fifo``, a ``MEMLOCK`` rlimit for ``mlock``. This driver does not
    #: set CPU affinity and does not require any host CPU reservation; pinning or
    #: isolating cores, if wanted at all, is the consumer's concern.
    rt_priority: int = 80
    sched_fifo: bool = False
    mlock: bool = False

    def to_rt_core_config(self) -> RtCoreConfig:
        """Build the C++ :class:`airo_fanuc._core.RtCoreConfig`.

        Starts from the shipped C++ defaults — a synthetic envelope that exists only
        so the tick-engine math is testable stand-alone — and overwrites the arm's
        limits from :attr:`profile` along with the protocol, brake-scale and
        RT-hygiene knobs Python owns. The limits are the reason this must run: the
        tick engine clamps against ``cfg.tick.limits``, so a profile that never
        reached this struct would be a profile the RT core is not enforcing.

        The capture window, the SAFE_FOLLOW envelope and the mirrored watchdog windows are
        all set from :mod:`airo_fanuc.controller_facts` below. The per-motion settle
        overrides are not: ``submit_trajectory`` carries those, so they stay at the C++
        defaults that mirror :class:`~airo_fanuc.controller_facts.SettlePolicy`.
        """
        rc = RtCoreConfig()
        rc.velocity_limits = [float(v) for v in self.profile.velocity_limits]
        rc.acceleration_limits = [float(v) for v in self.profile.acceleration_limits]
        rc.jerk_limits = [float(v) for v in self.profile.jerk_limits]
        # Position limits too: the core clamps every mode's command against these, and
        # its own defaults are ±inf, so a profile that stopped here would leave the
        # soft-limit clamp switched off.
        rc.position_limits_lower = [float(v) for v in self.profile.position_limits_lower]
        rc.position_limits_upper = [float(v) for v in self.profile.position_limits_upper]
        rc.stop_scale_va = float(self.stop_scale_va)
        rc.stop_scale_j = float(self.stop_scale_j)
        rc.slew_factor = float(self.slew_factor)
        # The capture window and splice envelope: set here because
        # ``move_trajectory`` refuses a submission against the ``controller_facts``
        # values, so the gate that executes has to be the same number the refusal was
        # decided by — otherwise the two drift silently and the refusal stops
        # predicting the gate.
        # np.deg2rad, not math.radians: driver.py builds the refusal threshold the same
        # way, and the two must be bit-identical or a submission could be refused by a
        # check the gate would have passed.
        rc.capture_rate_rad_s = float(np.deg2rad(cf.CAPTURE_RATE_DEG_S))
        rc.capture_tol_rad = float(np.deg2rad(cf.CAPTURE_TOL_DEG))
        # The SAFE_FOLLOW re-anchor walks a FAULTED arm's commanded pose back onto the
        # measured one, and ``controller_facts`` says outright that these two constants own
        # that envelope as well as the capture window. Both are the same pair of numbers, so
        # tightening the capture rate has to tighten the rate a faulted arm is walked at
        # too — otherwise the one place the commanded pose moves without a motion behind it
        # keeps running at a rate nothing in the single source still names.
        rc.safe_follow_rate_rad_s = float(np.deg2rad(cf.CAPTURE_RATE_DEG_S))
        rc.safe_follow_deadband_rad = float(np.deg2rad(cf.CAPTURE_TOL_DEG))
        rc.servo_limit_scale = float(cf.SERVO_LIMIT_SCALE)
        rc.qd_end_blend_min_s = float(cf.QD_END_BLEND_MIN_MS) / 1000.0
        # Mirrored watchdog/dwell windows. The C++ defaults match, but leaving them
        # unset made "single source of truth" untrue for exactly these fields.
        rc.rx_silence_blind_hold_ms = float(cf.RX_SILENCE_BLIND_HOLD_MS)
        rc.rx_silence_qd_ramp_ms = float(cf.RX_SILENCE_QD_RAMP_MS)
        rc.rx_silent_park_ms = float(cf.RX_SILENT_PARK_MS)
        rc.antiflap_dwell_ms = float(cf.ANTIFLAP_DWELL_MS)
        rc.sm_version = int(self.sm_version)
        rc.supervisor_lost_s = float(self.supervisor_lost_s)
        rc.itp_s = float(self.itp_s)
        rc.preroll_timeout_s = float(self.preroll_timeout_s)
        rc.rt_priority = int(self.rt_priority)
        rc.sched_fifo = bool(self.sched_fifo)
        rc.mlock = bool(self.mlock)
        return rc


@dataclass
class DriverPolicy:
    """Behavioural policy for the driver + lifecycle supervisor.

    The defaults are tuned for an unattended cell: auto-recovery ON, the ARM gate
    armed on operator/e-stop faults, single-owner flock acquired. :attr:`config` is
    required, because the :class:`DriverConfig` it holds needs an arm profile that only
    the caller can supply.
    """

    config: DriverConfig

    # -- bring-up --------------------------------------------------------
    #: Bring-up attempts. Must be > 1 when :attr:`enable_gripper` is set: observed
    #: twice, the attempt that RUN-forks GRIPDISP times out waiting for
    #: ``motion_possible`` (the fork and the ``FRC_Call(STREAM_MOTN)`` do not both
    #: land in one attempt), and the NEXT attempt succeeds — it skips the fork via
    #: the ``_grprun_forked`` latch, finds the dispatcher already alive, and only
    #: has the ``FRC_Call`` left to do. A no-gripper session forks nothing and
    #: reaches STREAMING on attempt 1, which is why the examples can pass 1.
    connect_retries: int = 3
    #: Grace, post-bring-up, to observe the seqlock publish HOLD after wait_ready.
    #: ``wait_ready`` flips STREAMING a beat before HOLD is published, so without a
    #: little slack bring-up would read a not-yet-published mode as a failure.
    hold_wait_s: float = 3.0
    #: How long ``motion_possible`` must stay asserted before bring-up reports success.
    #: STREAM_MOTN cannot be un-launched over RMI, so a bring-up over a controller that
    #: already has an instance running must re-``FRC_Call`` it, and that re-Call drops
    #: ``motion_possible`` roughly 1 s later — after a single assert-once check has already
    #: passed (measured; docs/controller-notes.md §4.2). Without this window the
    #: constructor returns a robot that faults immediately afterwards, and clearing it is
    #: left to the recovery ladder, which makes a normal startup depend on
    #: ``auto_recover``. Costs this much on every bring-up; 0 disables the settle.
    bringup_settle_s: float = 2.0
    preflight_full: bool = False

    # -- recovery / faults ----------------------------------------------
    auto_recover: bool = True
    #: Cooldown between auto-recovery ladder attempts — anti-churn, so a fault the
    #: ladder cannot clear becomes one attempt every 15 s instead of a retry storm
    #: hammering the RMI session (RMI churn is itself an SM-daemon wedge vector).
    recovery_cooldown_s: float = 15.0
    #: Settle delay before an *auto* recovery attempt: the controller's own latches
    #: need a moment after a fault before a reset will take. Explicit ``recover()``
    #: bypasses it (a human is asking now). Tests set this small.
    recovery_delay_s: float = 3.0
    #: Grace for motion_possible / HOLD to reassert after a ladder relaunch.
    ready_wait_s: float = 60.0
    #: SHORT probe (s) the recovery ladder waits for ``motion_possible`` to reassert
    #: after its reset/relaunch before giving up on THAT tier. Kept well under
    #: ``ready_wait_s`` so a fault a plain ``reset + FRC_Call`` cannot clear (e.g. a
    #: ``FRC_SystemFault`` that de-initialised the RMI motion interface) fails the
    #: tier FAST — the driver then escalates to the cold reconnect (full re-bring-up:
    #: re-``initialize`` + GRIPDISP re-fork) instead of stalling ~60 s. HW-observed:
    #: reset+reseed+FRC_Call alone does NOT re-arm motion after a collision
    #: SystemFault, so a fast escalation is the difference between hands-off recovery
    #: and a wedged idle loop.
    recovery_motion_probe_s: float = 5.0
    #: Full-RMI-reconnect attempts inside the recovery ladder when a SystemFault
    #: leaves RMI unresponsive (FRC_Reset / FRC_Call → RmiSessionDown). The
    #: normal ladder dead-ends on RmiSessionDown; this many times it falls back
    #: to the bring-up "flush stale SystemFault" reconnect (rmi.stop → reconnect
    #: → reset → reseed → FRC_Call). Kept small (1): RMI churn is itself a
    #: documented SM-daemon wedge vector, so this trades one clean reconnect for
    #: the wedge rather than hammering the session. 0 disables the fallback (the
    #: ladder then simply dead-ends on RmiSessionDown).
    recovery_reconnect_attempts: int = 1

    # -- ARM gate --------------------------------------------------------
    #: When True, recovery from an e-stop / operator-required fault ends in
    #: MOTION_INHIBITED — the next motion needs an explicit ``arm()``.
    arm_gate: bool = True

    # -- CAPTURE collision-check hook ------------------------------------
    capture_check: CaptureCheck | None = None

    # -- peripherals -----------------------------------------------------
    enable_gripper: bool = True
    #: Which registers the gripper dispatcher on this controller watches, and which
    #: values it understands. Defaults to the shipped
    #: :data:`~airo_fanuc.gripper.ROBOTIQ_2F85` preset, which is the gripper this
    #: package has been exercised against; a cell with any other gripper supplies its
    #: own :class:`~airo_fanuc.gripper.RegisterGripperProtocol` here and needs no other
    #: change. Unlike :attr:`DriverConfig.profile` this has a default, because a wrong
    #: gripper protocol cannot drive the arm into anything — the dispatcher either
    #: recognises the values or does not answer at all, and bring-up says so.
    gripper_protocol: RegisterGripperProtocol = ROBOTIQ_2F85
    #: Injected publisher (zenoh, ROS, anything matching the ``Publisher`` protocol)
    #: for the republisher threads. ``None`` = no republish.
    publisher: Publisher | None = None

    # -- ownership -------------------------------------------------------
    enable_ownership: bool = True
    ownership_mode: str = "control"
    lock_path: str | os.PathLike[str] = DEFAULT_LOCK_PATH

    # -- end-of-trajectory settle default -------------------------------
    settle: SettlePolicy = field(default_factory=SettlePolicy)

    # -- supervisor loop -------------------------------------------------
    watch_interval_s: float = 0.02
