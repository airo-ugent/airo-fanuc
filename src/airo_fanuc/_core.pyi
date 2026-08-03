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

import enum

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
    ``would_reject`` (bool; ``|q_cmd − q0|∞`` beyond the 5° window), ``count``
    (int), ``finished`` (bool), ``overflow`` (bool), ``q`` / ``qd`` (lists of
    length-6 radian knot vectors, ITP-spaced; ``q[0] == q_cmd``).
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

class Mode(enum.IntEnum):
    STREAM_DOWN = ...
    PREROLL = ...
    HOLD = ...
    CAPTURE = ...
    TRAJECTORY = ...
    SERVO = ...
    BRAKE = ...
    SAFE_FOLLOW = ...
    RX_SILENT = ...

class FaultReason(enum.IntEnum):
    NONE = ...
    E_STOP = ...
    IN_ERROR = ...
    MOTION_NOT_POSSIBLE = ...
    TEACH_MODE = ...
    CONTACT_STOP = ...
    SAFETY_CLAMP = ...
    RX_SILENT = ...
    RX_DEGRADED = ...
    WATCHDOG_EXPIRED = ...
    FORCE_GUARD = ...
    REJECTED_START_MISMATCH = ...
    SUPERVISOR_LOST = ...
    INTERNAL = ...

class MotionStatus(enum.IntEnum):
    PENDING = ...
    RUNNING = ...
    DONE = ...
    SETTLE_TIMEOUT = ...
    STOPPED = ...
    PREEMPTED = ...
    FAULTED = ...
    REJECTED = ...

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
    safe_follow_rate_rad_s: float
    safe_follow_deadband_rad: float
    safety_scale_min: float
    supervisor_lost_s: float
    preroll_timeout_s: float
    #: Controller interpolation period in seconds (8 ms on the R-30iB class). Every
    #: per-tick limit is scaled by it, so it must equal the controller's real period.
    #: Joint position limits (rad). Default ±inf — the core clamps every commanded
    #: pose against these, so leaving them unset disables the soft-limit clamp.
    position_limits_lower: list[float]
    position_limits_upper: list[float]
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
        q_meas/qd_est/q_cmd/qd_cmd (length-6), cart, status bits, wrench,
        active_motion_id/status, rx_age_ms, total_slew_clips."""

    def poll_events(self) -> list[dict[str, object]]:
        """Drain the RT event ring (mode/epoch/fault/motion events)."""

    def timing_stats(self) -> dict[str, object]:
        """Tick histograms (p50/p99/p999/max) + invariant counters
        (tx_count/tau_advance_count/double_send_guard/...)."""
