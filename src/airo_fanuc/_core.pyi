# SPDX-License-Identifier: Apache-2.0
"""Type stub for the ``airo_fanuc._core`` C++17 extension.

Two layers: the Stream Motion packet codec (``encode_command_packet`` /
``decode_status_204`` / ``decode_status_v3`` / ``generate_capture_path``) and the
real-time ``StreamCore`` class + ``RtCoreConfig`` and the ``Mode`` /
``FaultReason`` / ``MotionStatus`` enums. ``StreamCore`` is the production RT-core
surface the shipped, hardware-validated :class:`airo_fanuc.FanucDriver` drives (the
codec surface is also what the pure-Python oracle in ``airo_fanuc.testing.wire``
byte-compares against).
"""

from __future__ import annotations

from typing import ClassVar

__version__: str

# Wire sizes of the #pragma pack(1) Stream Motion structs.
COMMAND_PACKET_SIZE: int
STATUS_204_PACKET_SIZE: int
STATUS_V3_PACKET_SIZE: int
FORCE_SENSOR_CONFIG_PACKET_SIZE: int

def encode_command_packet(
    seq: int,
    is_last: bool,
    do_motn_ctrl: int,
    pos_deg: list[float],
) -> bytes:
    """Encode a Stream Motion CommandPacket (type 201, 344 B, big-endian).

    ``pos_deg`` must have length 9 (kMaxAxisNumber); joint angles in degrees,
    trailing entries zero-padded for the 6-DOF CRX. The dataStyle selector is
    pinned to 0xFFFF (joint angles). Returns 344 raw bytes.
    """

def generate_capture_path(
    q_cmd: list[float],
    qd_cmd: list[float],
    q0: list[float],
    qd0: list[float],
    config: RtCoreConfig | None = None,
) -> dict[str, object]:
    """Synthesize the deterministic CAPTURE splice ``(q_cmd, qd_cmd) → (q0, qd0)``.

    All arguments are length-6 radian vectors. Returns the EXACT knots the RT core
    will execute (same code path, default ``TickEngineConfig`` → byte-identical),
    so the Python collision check IS the executed path. Keys:
    ``would_reject`` (bool; the capture gate's verdict — the 5° endpoint window, and
    whether that window can absorb the first knot's velocity change at the brake-class
    clamps), ``tol_exceeded`` (bool; the endpoint-window term specifically),
    ``reject_joints`` (list[int]; joints failing the arrival-rate or velocity-shed term),
    ``shed_travel`` (length-6 radian list; the travel that shedding
    ``|qd0 − qd_cmd|`` costs per joint — 0 where the endpoint velocities match. These are
    the gate's OWN numbers, so a caller formatting an error message never has to
    re-derive the condition), ``count``
    (int), ``finished`` (bool), ``overflow`` (bool), ``q`` / ``qd`` (lists of
    length-6 radian knot vectors, ITP-spaced; ``q[0] == q_cmd``), ``residue_ns``
    (int; how far past ``(q0, qd0)`` the last knot lies in trajectory time, in
    ``(0, itp]`` — the core resumes trajectory playback at ``itp + residue_ns``,
    so this is part of the executed path, not a diagnostic).
    """

def plan_joint_move(
    q0: list[float],
    qd0: list[float],
    q_target: list[float],
    config: RtCoreConfig | None = None,
    max_velocity_rad_s: float = 0.0,
    accel_scale: float = 1.0,
    jerk_scale: float = 1.0,
    qdd0: list[float] = ...,
) -> dict[str, object]:
    """Plan a point-to-point joint move offline with Ruckig; return ITP-spaced knots.

    All vectors are length-6 radians. Returns ``times_ns`` (strictly-increasing int64
    ns, relative), ``q`` / ``qd`` (lists of length-6 radian knots), ``count`` and
    ``duration_s`` — the argument shape :meth:`StreamCore.submit_trajectory` and
    :meth:`airo_fanuc.FanucDriver.move_trajectory` take.

    ``max_velocity_rad_s`` is the LEADING-AXIS speed: it caps every joint, and Ruckig's
    time-synchronization lands them together, so the joint travelling furthest runs at
    this speed and the rest scale down. ``<= 0`` means the config's own velocity limits.
    ``accel_scale`` / ``jerk_scale`` are FRACTIONS of the config's acceleration / jerk
    limits (see ``controller_facts.MOVEJ_LIMIT_SCALE_A`` / ``_J``) and must lie in
    ``(0, 1]``: 1.0 is the arm's own envelope, and above it the plan exceeds the limits
    the tick engine enforces with nothing downstream to re-time it.

    Pass the same :class:`RtCoreConfig` the core was constructed with. The core plays
    knots back with cubic Hermite and never re-times them, so the profile has to be
    shaped by the limits the tick engine enforces or it gets clipped on the tick.

    Always returns ≥2 knots: a plan shorter than one interpolation period (already at
    the target) is stretched over one ITP rather than returning a degenerate timeline.
    The final knot is pinned exactly to ``q_target`` at zero velocity.

    Raises ``ValueError`` on a malformed vector or a scale outside ``(0, 1]``, ``RuntimeError``
    if Ruckig cannot plan the move.
    """

def decode_status_204(data: bytes) -> dict[str, object]:
    """Decode a Stream Motion type-204 RobotStatusPacket (416 B, big-endian).

    Raises ``ValueError`` if ``data`` is shorter than 416 bytes or is not a
    type-204 packet. Keys: ``packet_type``, ``version_no``, ``sequence_no``,
    ``status``, ``robot_status``, ``contact_stop_status``, ``time_stamp``,
    ``position``, ``joint_angle``, ``current`` (length-9 float lists),
    ``safety_scale``, ``force_x/y/z``, ``moment_x/y/z``, ``fs_type``,
    ``io_status`` (256 raw bytes).
    """

def decode_status_v3(data: bytes) -> dict[str, object]:
    """Decode a V3 type-202 RobotStatusPacket (388 B, big-endian; no force block).

    Raises ``ValueError`` if ``data`` is shorter than 388 bytes or is not a
    type-202 packet. Same keys as :func:`decode_status_204`, but ``force_x/y/z`` /
    ``moment_x/y/z`` are ``0.0`` and ``fs_type`` is ``0xFFFFFFFF`` (Unavailable) —
    the wire carries no force block. This is what the CRX controller actually
    streams: it negotiates ``available_version=3``, so type-204 never appears.
    """

# ---------------------------------------------------------------------------
# RT core. Internal units are RADIANS; the deg/9-pad/0xFFFF wire conversion
# happens inside the C++ core.
# ---------------------------------------------------------------------------

class _CoreEnum:
    """Base of the four core enums. NOT an :class:`enum.IntEnum`, and not an ``int``.

    These are pybind11 enumerations, whose members do not derive from ``int``. That
    matters because :meth:`StreamCore.get_snapshot` publishes ``mode``, ``fault`` and
    ``active_motion_status`` as plain integers, and the obvious way to read one is wrong::

        snap["mode"] == Mode.HOLD             # ALWAYS False — different types
        Mode(int(snap["mode"])) == Mode.HOLD  # this is the comparison you want

    Declaring these as ``IntEnum`` here would make a type checker bless the first line.
    ``int(member)``, ``member.value``, ``member.name`` and ``Mode(2)`` all work; ``<`` and
    iterating the class do not. Bitwise operators work on :class:`Condition` only, which is
    bound with pybind11's ``arithmetic`` tag because it is a mask; they do not work on the
    other three, which are ordinals. Compare with ``==``, not ``is``: converting
    from an int builds a new object rather than returning the class attribute.
    """

    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...
    def __int__(self) -> int: ...
    def __init__(self, value: int) -> None: ...

class Mode(_CoreEnum):
    STREAM_DOWN: ClassVar[Mode]
    PREROLL: ClassVar[Mode]
    HOLD: ClassVar[Mode]
    CAPTURE: ClassVar[Mode]
    TRAJECTORY: ClassVar[Mode]
    SERVO: ClassVar[Mode]
    BRAKE: ClassVar[Mode]
    SAFE_FOLLOW: ClassVar[Mode]
    RX_SILENT: ClassVar[Mode]

class FaultReason(_CoreEnum):
    NONE: ClassVar[FaultReason]
    E_STOP: ClassVar[FaultReason]
    IN_ERROR: ClassVar[FaultReason]
    MOTION_NOT_POSSIBLE: ClassVar[FaultReason]
    TEACH_MODE: ClassVar[FaultReason]
    CONTACT_STOP: ClassVar[FaultReason]
    SAFETY_CLAMP: ClassVar[FaultReason]
    RX_SILENT: ClassVar[FaultReason]
    RX_DEGRADED: ClassVar[FaultReason]
    WATCHDOG_EXPIRED: ClassVar[FaultReason]
    FORCE_GUARD: ClassVar[FaultReason]
    REJECTED_START_MISMATCH: ClassVar[FaultReason]
    SUPERVISOR_LOST: ClassVar[FaultReason]
    INTERNAL: ClassVar[FaultReason]

class Condition(_CoreEnum):
    """Bit flags, not ordinals: the snapshot's ``conditions`` is a SET, because several
    can be live at once (an e-stop while in TEACH). Bound with pybind11's ``arithmetic``
    tag, so ``mask & Condition.E_STOP`` works. ``SUSTAINED_SLEW`` is diagnostic and never
    faults, so a non-zero mask does not by itself mean the arm is in trouble."""

    NONE: ClassVar[Condition]
    E_STOP: ClassVar[Condition]
    IN_ERROR: ClassVar[Condition]
    MOTION_NOT_POSSIBLE: ClassVar[Condition]
    TEACH: ClassVar[Condition]
    CONTACT_STOP: ClassVar[Condition]
    SAFETY_CLAMP: ClassVar[Condition]
    RX_DEGRADED: ClassVar[Condition]
    RX_SILENT: ClassVar[Condition]
    SUSTAINED_SLEW: ClassVar[Condition]

class MotionStatus(_CoreEnum):
    PENDING: ClassVar[MotionStatus]
    RUNNING: ClassVar[MotionStatus]
    DONE: ClassVar[MotionStatus]
    SETTLE_TIMEOUT: ClassVar[MotionStatus]
    STOPPED: ClassVar[MotionStatus]
    PREEMPTED: ClassVar[MotionStatus]
    FAULTED: ClassVar[MotionStatus]
    REJECTED: ClassVar[MotionStatus]

class RtCoreConfig:
    """RT core knobs (mirror of ``airo_fanuc.controller_facts``;
    ``DriverConfig.to_rt_core_config`` fills these in)."""

    #: The arm's motion envelope, one value per joint, in rad/s, rad/s² and rad/s³.
    #: Every stage — trajectory, servo, brake, capture, slew — clamps against these,
    #: so they are what a ``RobotProfile`` has to reach. The C++ defaults are a
    #: synthetic envelope for the stand-alone tick-engine tests, not any real arm;
    #: assigning a list of the wrong length raises rather than partially overwriting.
    velocity_limits: list[float]
    acceleration_limits: list[float]
    jerk_limits: list[float]
    #: Fractions of the limits above: the brake envelope (v/a and jerk scaled
    #: separately) and the per-tick slew clip, ``slew_factor × v × itp_s``.
    stop_scale_va: float
    stop_scale_j: float
    slew_factor: float
    rx_silence_blind_hold_ms: float
    rx_silence_qd_ramp_ms: float
    rx_silent_park_ms: float
    antiflap_dwell_ms: float
    servo_limit_scale: float
    qd_end_blend_min_s: float
    #: The capture window, and the envelope the splice into knot 0 runs at.
    #: ``move_trajectory`` refuses a submission against the ``controller_facts`` values,
    #: so ``DriverConfig.to_rt_core_config`` sets these from the same constants — the
    #: refusal and the gate that executes must be decided by one number.
    capture_rate_rad_s: float
    capture_tol_rad: float
    safe_follow_rate_rad_s: float
    safe_follow_deadband_rad: float
    safety_scale_min: float
    supervisor_lost_s: float
    preroll_timeout_s: float
    #: Joint position limits (rad). Default ±inf — the core clamps every commanded
    #: pose against these, so leaving them unset disables the soft-limit clamp.
    position_limits_lower: list[float]
    position_limits_upper: list[float]
    #: Controller interpolation period in seconds (8 ms on the R-30iB class). Every
    #: per-tick limit is scaled by it, so it must equal the controller's real period.
    itp_s: float
    rt_priority: int
    sched_fifo: bool
    mlock: bool
    pll_rx_lead_us: float
    pll_kp: float
    sm_version: int
    def __init__(self) -> None: ...

class StreamCore:
    """Thin driver of the C++ RT core over Stream Motion UDP. The RT thread never
    calls back into Python; getters read a seqlock snapshot and never raise."""

    def __init__(self, host: str, port: int, config: RtCoreConfig = ...) -> None: ...
    def start(self) -> bool:
        """Open the socket, do the SM handshake (defensive Stop → GetCapability
        type-7 → adopt available_version + sampling_rate → Start), spawn the RT
        thread. Non-blocking w.r.t. PREROLL; returns False if the socket cannot open."""

    def stop(self) -> None:
        """Terminal is_last command → Stop packet → join the RT thread (wedge-free)."""

    def wait_ready(self, timeout_s: float = 5.0) -> bool:
        """Block until STREAMING (motion_possible / HOLD) or timeout."""

    @property
    def running(self) -> bool: ...
    @property
    def sm_negotiated_version(self) -> int:
        """Stream Motion version the controller reported it will serve, from the
        GetCapability reply. 0 until a reply has been seen."""

    @property
    def sm_sampling_rate_ms(self) -> int:
        """The controller's own interpolation period in whole milliseconds, from the
        GetCapability reply. 0 until a reply has been seen. Compare against the
        configured ``itp_s`` to detect a controller running a different period."""

    def submit_trajectory(
        self,
        times_ns: list[int],
        q: list[list[float]],
        qd: list[list[float]],
        speed_scale: float = 1.0,
        settle_tol_rad: float = ...,
        settle_vel_eps_rad_s: float = ...,
        settle_timeout_s: float = 2.0,
        force_stop_n: float = 0.0,
        deadman_s: float = 0.0,
    ) -> int:
        """Submit ONE whole trajectory (radians). CAPTURE-or-REJECT splice from
        the commanded pose. Returns a motion_id. ``force_stop_n`` arms the C++
        force-guard; ``deadman_s`` arms the caller-fed watchdog (kick()).

        ``speed_scale`` scales Hermite playback (qd by s, qdd by s²) but NOT the
        capture splice, which is built from the unscaled first knot — so s ≠ 1 steps
        the commanded velocity at the capture→trajectory handover.
        :meth:`airo_fanuc.FanucDriver.move_trajectory` therefore pins it to 1.0 and
        exposes no scale knob: a trajectory's own ``times``/``qd`` are its speed, and
        stretching them caller-side scales the first knot too."""

    def submit_servo(self, q: list[float], duration: float) -> int:
        """Best-effort servo target: head for ``q`` (length-6 radians), with
        ``duration`` seconds to get there. Never refused for distance — motion toward
        a far target is bounded by the servo limits, not rejected."""
    def submit_servo_ff(
        self,
        q: list[float],
        qd: list[float],
        qdd: list[float],
        duration: float,
    ) -> int:
        """As :meth:`submit_servo`; ``qd``/``qdd`` are accepted and CURRENTLY IGNORED.
        All three vectors are length-6 radians (rad, rad/s, rad/s²). They previously
        became Ruckig's target velocity/acceleration, but demanding an arrival velocity
        made the command reverse against a forward-moving stream under clock drift —
        see BEST EFFORT in ``src/cpp/tick_engine/servo.hpp``."""

    def submit_brake(self) -> int: ...
    def submit_hold(self) -> int: ...
    def stop_j(self) -> None:
        """Universal preempt (any thread; ≤1 tick). Converges to BRAKE."""

    def hold(self) -> None: ...
    def recover(self) -> None:
        """Leave SAFE_FOLLOW / RX_SILENT after the anti-flap dwell."""

    def kick(self) -> None:
        """Feed the caller-fed deadman (collision-supervision-alive interlock)."""

    def heartbeat(self) -> None:
        """Supervisor-liveness beat. The first beat arms the SUPERVISOR_LOST
        watchdog; a lapse > ``supervisor_lost_s`` while streaming latches
        FAULTED(SUPERVISOR_LOST) and holds. Beat from a dedicated thread."""

    def motion_status(self, motion_id: int) -> int:
        """MotionStatus ordinal for a submitted motion (PENDING if unknown)."""

    def joints_at_wall(self, wall_ns: int) -> list[float] | None:
        """Joint state (radians) nearest a wall-clock stamp (camera FK-at-shutter)."""

    def get_snapshot(self) -> dict[str, object]:
        """Seqlock read; never raises. Keys include mode/fault/conditions/epoch,
        q_meas/qd_est/q_cmd/qd_cmd/qdd_cmd (length-6), cart, status bits, wrench,
        active_motion_id/status, rx_age_ms, total_slew_clips, and ``cmd_tick`` — the
        tick the commanded state was commanded on, which advances once per tick whether
        or not that tick transmitted, so a stalled value means the RT loop is not
        ticking."""

    def poll_events(self) -> list[dict[str, object]]:
        """Drain the RT event ring (mode/epoch/fault/motion events)."""

    def timing_stats(self) -> dict[str, object]:
        """Tick histograms (p50/p99/p999/max) + invariant counters
        (tx_count/tau_advance_count/double_send_guard/...)."""
