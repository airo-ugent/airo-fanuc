# SPDX-License-Identifier: Apache-2.0
"""Stream Motion UDP server for the FakeCRXController.

A strict-conformance emulator of the controller's Stream Motion side:

* Handshake state machine the driver must follow: defensive
  StopPacket (type 2) → GetCapability (type 7, negotiate ``available_version``,
  ``sampling_rate=8``) → StartPacket (type 200) → optional ForceSensorConfig
  (type 205, accepted only when negotiated v >= 4) → stream status.
* Streams status at the controller ITP driven by an injectable clock (one packet
  per :meth:`tick`), populating the status/robot_status bits, contact_stop,
  safety_scale and measured joints from the
  :class:`~airo_fanuc.testing.plant.JointPlant`. **Version-correct**: at the
  default negotiated v3 it emits **legacy type-202** (388 B, NO force block — what
  the physical controller streams, confirmed by packet capture); only at v >= 4
  does it emit **type-204** (416 B, +force/moment/fs_type). Set
  ``FakeCRXConfig.available_version=4`` to exercise the force path.
* Consumes **type-201 CommandPackets**, feeds the commanded joints to the plant,
  and — in strict mode — asserts every per-packet invariant that the real
  controller (or our own driver) must satisfy, most importantly
  ``dataStyle == 0xFFFF`` (the 0-dataStyle Cartesian-slew bug, incident
  2026-05-06) and the **min inter-TX spacing** guard. The spacing check
  reads the **kernel RX timestamp** of every command (``SO_TIMESTAMPNS`` +
  ``recvmsg``/CMSG), so it flags a genuine double-send (two TXs < 1 ms apart)
  while NOT false-flagging two legitimate ~8 ms-apart TXs that the fake's own
  ~125 Hz tick clock happened to bin into one drain — the fake and the driver run
  on independent clocks that jitter across a drain-window boundary, so a
  per-window count of commands cannot distinguish the two cases and kernel
  timestamps can. The threshold is ``0.5 · itp`` (4 ms): a double-send is < 1 ms,
  the nominal cadence is ~8 ms, so 4 ms cleanly separates them. The **first
  inter-command interval of each (re)started stream** is the exception: the host
  RT core's PLL re-locks phase on the first RX after a StartPacket and
  legitimately compresses that one interval (measured ≥ 0.57 ms), so it is
  checked against a tighter same-instant floor (``itp / 80``) that still flags a
  genuine ~simultaneous double-send while tolerating the PLL transient — see
  :attr:`_first_interval_floor_ns`.

``motion_possible`` (status bit 0) asserts only after a StartPacket AND the RMI
side has accepted ``FRC_Call("STREAM_MOTN")`` — the two subsystems are coupled
through the shared :class:`ControllerState` on the facade.

Byte encode/decode is delegated to the ``airo_fanuc.testing.wire`` oracle, whose
layout the goldens pin; this module never hand-packs bytes (it only *decodes* the inbound
CommandPacket, whose layout it re-derives from the wire codec's documented
struct and cross-checks against ``wire.COMMAND_PACKET_SIZE`` at import).
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import TYPE_CHECKING

import numpy as np

from airo_fanuc.testing import wire
from airo_fanuc.testing.plant import N_AXES, Clock, JointPlant, ManualClock

if TYPE_CHECKING:  # pragma: no cover - typing only
    from airo_fanuc.testing.fake_crx import ControllerState, FakeCRXConfig


class StrictConformanceError(AssertionError):
    """Raised when a Stream Motion command violates a strict-mode invariant.

    Subclasses :class:`AssertionError` so a scenario/self-test fails loudly the
    instant the driver-under-test puts a non-conformant packet on the wire.
    """


# CommandPacket (type 201) decode format — re-derived from the wire codec's
# documented layout (``wire._FMT_COMMAND``); cross-checked below so a wire
# layout change surfaces here immediately rather than as a silent mis-decode.
_FMT_COMMAND = ">IIIBBH9d256B"
assert struct.calcsize(_FMT_COMMAND) == wire.COMMAND_PACKET_SIZE

# Force-sensor-config (type 205) decode format (see wire._FMT_FORCE_SENSOR_CONFIG).
_FMT_FSCONFIG = ">IIII"
assert struct.calcsize(_FMT_FSCONFIG) == wire.FORCE_SENSOR_CONFIG_PACKET_SIZE

_RX_BUF_BYTES = 2048
# |qd| above which we assert status bit 3 (motion_in_progress), deg/s.
_MOTION_IN_PROGRESS_EPS_DEG_S = 1.0

# Kernel RX timestamping (for the min-inter-TX-spacing check). Python's socket
# module does NOT expose these Linux SOL_SOCKET constants on every build, so we
# fall back to the fixed x86-64 Linux values (SO_TIMESTAMPNS == SCM_TIMESTAMPNS
# == 35). The RT core is Linux-only (timerfd/epoll), so this is not a portability
# regression; the setsockopt is guarded so construction never raises elsewhere.
_SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
_SCM_TIMESTAMPNS = getattr(socket, "SCM_TIMESTAMPNS", 35)
# One control message carrying a `struct timespec` (two native 8-byte longs).
_ANC_BUF_BYTES = socket.CMSG_SPACE(16)
_TIMESPEC_FMT = "qq"  # tv_sec, tv_nsec (64-bit Linux)


class FakeStreamMotionServer:
    """UDP Stream Motion emulator. One :meth:`tick` = one controller ITP.

    The server is driven by :meth:`tick` (advance the plant, emit one status
    packet) rather than wall time, so scenario tests are deterministic. A
    wall-paced :meth:`start_realtime` driver is provided for downstream
    integration/soak use.
    """

    def __init__(
        self,
        state: ControllerState,
        plant: JointPlant,
        clock: Clock,
        cfg: FakeCRXConfig,
        *,
        host: str = "127.0.0.1",
        port: int = 0,
    ) -> None:
        self._state = state
        self._plant = plant
        self._clock = clock
        self._cfg = cfg
        self.itp_s: float = float(cfg.itp_s)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.setblocking(False)
        # Kernel RX timestamps for the min-inter-TX-spacing check. Guarded
        # so a non-Linux build degrades to "no spacing check" rather than crashing.
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, _SO_TIMESTAMPNS, 1)
            self._rx_timestamping = True
        except OSError:  # pragma: no cover - non-Linux fallback
            self._rx_timestamping = False
        self.host, self.port = self._sock.getsockname()[:2]

        # Min inter-arrival spacing: two consecutive commands drained in
        # the SAME window that arrive (kernel RX time) closer than this are a
        # double-send. Scoped within one drain batch — that is exactly where the
        # fake-vs-core clock jitter can bin two commands together, and it keeps
        # the deterministic manual-tick tests (which send one command per tick
        # with no wall delay) from false-flagging consecutive-tick commands.
        self._min_inter_tx_ns: float = 0.5 * self.itp_s * 1e9
        self._last_cmd_kernel_ns: int | None = None
        # (Re)start settling floor. The very FIRST inter-command interval of a
        # (re)started stream is legitimately compressed: the host RT core's PLL
        # re-locks phase on the first RX after a StartPacket and shortens that one
        # interval (measured >= 0.57 ms, below the 0.5·itp steady guard). That is NOT
        # a double-send — but the reconnect / e-stop path-B re-handshake (StopPacket →
        # drain → StartPacket → resume) trips the steady guard on it. So the first
        # interval of each stream is checked against this much tighter floor instead,
        # which still catches a GENUINE double-send: a real double-send emits both
        # CommandPackets in one host iteration and they arrive ~simultaneously
        # (measured <= 12 us on loopback). 1/80·itp (0.1 ms @ 8 ms ITP) sits in the
        # ~48x-wide gap: ~8x above a real double-send, ~5x below the shortest
        # legitimate PLL-compressed interval. Only the FIRST interval is relaxed
        # (via _commands_seen == 2); every steady-stream interval keeps the full guard.
        self._first_interval_floor_ns: float = self.itp_s * 1e9 / 80.0

        self._peer_addr: tuple[str, int] | None = None
        self._streaming = False
        self._available_version = int(cfg.available_version)
        self._negotiated_version = int(cfg.available_version)
        self._fsconfig_received = False

        self._status_seq = 0  # controller-side status counter (echoed on wire)
        self._rx_seq_prev: int | None = None
        self._commands_seen = 0
        self._last_cmd_is_last = False

        # Per-window command bookkeeping (reset every tick).
        self._pending_cmd: np.ndarray | None = None
        self._cmds_this_window = 0
        self._ticks_since_cmd = 0

        # Outbound status-stream perturbation (drop / seq gap / duplicate).
        self._drop_status_countdown = 0
        self._pending_seq_gap = 0
        self._dup_next = False

        self._violations: list[str] = []
        self._realtime_thread: threading.Thread | None = None
        self._realtime_stop = threading.Event()
        self.realtime_error: BaseException | None = None

    # -- properties -------------------------------------------------------

    @property
    def streaming(self) -> bool:
        return self._streaming

    @property
    def negotiated_version(self) -> int:
        return self._negotiated_version

    @property
    def violations(self) -> list[str]:
        return list(self._violations)

    # -- RX / handshake ---------------------------------------------------

    def _drain_rx(self) -> None:
        """Process every UDP packet currently pending (non-blocking).

        Uses ``recvmsg`` + ``SO_TIMESTAMPNS`` so each CommandPacket carries its
        KERNEL arrival timestamp (not the drain-return time). Batched draining
        must NOT lose true inter-arrival spacing — that is why kernel timestamps
        are required for the min-inter-TX-spacing check. The spacing
        reference is reset per drain: only commands drained together (where the
        fake/core clock-jitter binning happens) are compared.
        """
        self._last_cmd_kernel_ns = None
        while True:
            try:
                buf, ancdata, _flags, addr = self._sock.recvmsg(_RX_BUF_BYTES, _ANC_BUF_BYTES)
            except BlockingIOError:
                return
            except OSError:
                return
            self._peer_addr = addr
            if len(buf) < 4:
                continue
            kernel_ns = self._kernel_timestamp_ns(ancdata)
            (ptype,) = struct.unpack_from(">I", buf, 0)
            if ptype == wire.PACKET_TYPE_STOP:
                self._on_stop(buf)
            elif ptype == wire.PACKET_TYPE_GET_CAPABILITY:
                self._on_get_capability(buf, addr)
            elif ptype == wire.PACKET_TYPE_START:
                self._on_start(buf)
            elif ptype == wire.PACKET_TYPE_FORCE_SENSOR_CONFIG:
                self._on_force_sensor_config(buf)
            elif ptype == wire.PACKET_TYPE_COMMAND:
                self._on_command(buf, kernel_ns)
            # Unknown types are dropped (a real controller ignores garbage).

    @staticmethod
    def _kernel_timestamp_ns(ancdata: list[tuple[int, int, bytes]]) -> int | None:
        """Extract the ``SCM_TIMESTAMPNS`` kernel RX time (ns) from recvmsg cmsgs."""
        for level, ctype, cdata in ancdata:
            if level == socket.SOL_SOCKET and ctype == _SCM_TIMESTAMPNS and len(cdata) >= 16:
                sec, nsec = struct.unpack(_TIMESPEC_FMT, cdata[:16])
                return sec * 1_000_000_000 + nsec
        return None

    def _on_stop(self, buf: bytes) -> None:
        if self._cfg.strict and self._streaming and self._commands_seen > 0 and not self._last_cmd_is_last:
            self._record_violation(
                "StopPacket received while streaming without a preceding "
                "is_last_command=1 CommandPacket (unclean stop)"
            )
        self._streaming = False
        with self._state.lock:
            self._state.stream_started = False

    def _on_get_capability(self, buf: bytes, addr: tuple[str, int]) -> None:
        # Reply with the controller's advertised capability. The requester sends
        # the base version (3); we answer with our available_version, which the
        # client MUST adopt for Start / Command / FSConfig.
        reply = wire.encode_capability_result_packet(
            packet_type=wire.PACKET_TYPE_GET_CAPABILITY,
            version_no=self._available_version,
            id=1,
            sampling_rate_ms=int(round(self.itp_s * 1000.0)),
            start_move=0,
            available_version=self._available_version,
            rob_status_use_tcp=0,
        )
        try:
            self._sock.sendto(reply, addr)
        except OSError:
            pass

    def _on_start(self, buf: bytes) -> None:
        _ptype, version = struct.unpack(">II", buf[: wire.START_PACKET_SIZE])
        self._negotiated_version = int(version)
        if self._cfg.strict and int(version) != self._available_version:
            self._record_violation(
                f"StartPacket version_no={version} != negotiated available_version={self._available_version}"
            )
        self._streaming = True
        self._status_seq = 0
        self._rx_seq_prev = None
        self._commands_seen = 0
        self._last_cmd_is_last = False
        self._ticks_since_cmd = 0
        # SM (re)start boundary. Forget any pre-restart CommandPacket timestamp so the
        # first command of the new stream is never compared against a stale reference
        # (a StartPacket only ever arrives at (re)start, never mid-steady-stream). The
        # PLL's one-time first-interval compression at (re)start is handled by the
        # _commands_seen == 2 relaxation in _on_command (see _first_interval_floor_ns);
        # resetting _commands_seen = 0 above re-arms that per-stream first-interval
        # detection.
        self._last_cmd_kernel_ns = None
        with self._state.lock:
            self._state.stream_started = True

    def _on_force_sensor_config(self, buf: bytes) -> None:
        _ptype, version, _do_reset, fs_type = struct.unpack(_FMT_FSCONFIG, buf)
        if self._negotiated_version < 4:
            # On v < 4 the real controller raises HOST-380.
            with self._state.lock:
                self._state.raise_alarm("HOST-380", "System error 0x19,0x0")
                self._state.in_error = True
            if self._cfg.strict:
                self._record_violation(
                    "ForceSensorConfigPacket (type 205) sent on negotiated "
                    f"v{self._negotiated_version} < 4 (triggers HOST-380)"
                )
            return
        self._fsconfig_received = True
        with self._state.lock:
            self._state.fs_type = int(fs_type)

    def _on_command(self, buf: bytes, kernel_ns: int | None = None) -> None:
        fields = struct.unpack(_FMT_COMMAND, buf)
        (_ptype, version, seq, is_last, do_motn_ctrl, data_style) = fields[:6]
        pos = np.array(fields[6 : 6 + N_AXES], dtype=np.float64)
        io = fields[6 + N_AXES :]

        self._cmds_this_window += 1
        self._commands_seen += 1
        self._last_cmd_is_last = bool(is_last)

        if self._cfg.strict:
            # Min inter-TX spacing: a genuine double-send emits two TXs
            # < 1 ms apart; kernel RX timestamps separate that from two legit
            # ~8 ms TXs jittered into one drain. Compared within the drain only.
            if kernel_ns is not None and self._last_cmd_kernel_ns is not None:
                spacing_ns = kernel_ns - self._last_cmd_kernel_ns
                # The FIRST inter-command interval of a (re)started stream (this is the
                # 2nd command → _commands_seen == 2) is the PLL's phase re-lock and is
                # legitimately compressed; check it against the tight same-instant floor
                # so only a genuine ~simultaneous double-send trips. Every steady-stream
                # interval uses the full 0.5·itp guard. See _first_interval_floor_ns.
                is_first_interval = self._commands_seen == 2
                threshold = self._first_interval_floor_ns if is_first_interval else self._min_inter_tx_ns
                if spacing_ns < threshold:
                    self._record_violation(
                        f"consecutive CommandPackets {spacing_ns / 1e6:.3f} ms apart "
                        f"< min inter-TX spacing {threshold / 1e6:.1f} ms — "
                        "the one-TX-per-window / double-send guard, measured "
                        "from kernel RX timestamps"
                    )
            if data_style != wire.COMMAND_DATA_STYLE:
                self._record_violation(
                    f"CommandPacket dataStyle=0x{data_style:04X} != 0xFFFF — a "
                    "0 dataStyle makes the controller slew the joints as a "
                    "Cartesian XYZWPR pose (E-stop incident 2026-05-06)"
                )
            if do_motn_ctrl != 1:
                self._record_violation(
                    f"CommandPacket do_motn_ctrl={do_motn_ctrl} != 1 "
                    "(nominal joint command must assert motion control)"
                )
            if any(b != 0 for b in io):
                self._record_violation(
                    "CommandPacket io_command is not all-zero (no GPIO surface is driven over Stream Motion)"
                )
            if int(version) != self._negotiated_version:
                self._record_violation(
                    f"CommandPacket version_no={version} != negotiated {self._negotiated_version}"
                )
            if not np.all(np.isfinite(pos)):
                self._record_violation("CommandPacket contains non-finite joint angle")
            if self._rx_seq_prev is not None and seq <= self._rx_seq_prev:
                self._record_violation(
                    f"CommandPacket sequence_no={seq} not strictly increasing "
                    f"(prev={self._rx_seq_prev}) — duplicate / out-of-order TX"
                )

        self._rx_seq_prev = int(seq)
        if kernel_ns is not None:
            self._last_cmd_kernel_ns = kernel_ns
        self._pending_cmd = pos  # latest-wins within the window

    # -- tick -------------------------------------------------------------

    def tick(self, dt: float | None = None, *, raise_on_violation: bool = True) -> None:
        """Advance one controller ITP: drain RX, step the plant, emit status.

        In strict mode, records any invariant violation seen this tick. With
        ``raise_on_violation`` (the manual-tick default) it re-raises the first
        as :class:`StrictConformanceError` so a deterministic self-test fails
        loudly. The wall-paced :meth:`start_realtime` loop passes ``False`` so it
        RECORDS the violation and KEEPS STREAMING (an integration test can assert
        on ``violations`` without the core going RX-silent).

        The min-inter-TX-spacing / double-send guard is enforced per-packet in
        :meth:`_on_command` from kernel RX timestamps, NOT by a fixed per-tick
        window count: the fake and the driver run on independent ~125 Hz clocks
        that drift across a window boundary, so a count check bins two legitimate
        ~8 ms-apart TXs into one drain and false-positives.
        """
        dt = self.itp_s if dt is None else float(dt)
        n_viol_start = len(self._violations)

        self._drain_rx()

        n_cmds = self._cmds_this_window

        if self._streaming:
            self._advance_plant(dt, n_cmds)
            self._maybe_trip_deviation_fault()

        self._cmds_this_window = 0
        self._pending_cmd = None

        if self._streaming and self._peer_addr is not None:
            self._send_status(dt)

        if self._cfg.strict and raise_on_violation and len(self._violations) > n_viol_start:
            raise StrictConformanceError(self._violations[n_viol_start])

    def _advance_plant(self, dt: float, n_cmds: int) -> None:
        if n_cmds >= 1 and self._pending_cmd is not None:
            self._plant.command_step(self._pending_cmd, dt)
            self._ticks_since_cmd = 0
            with self._state.lock:
                self._state.tx_silent = False
            return

        # No fresh command this window.
        self._ticks_since_cmd += 1
        if self._ticks_since_cmd > self._cfg.tx_silence_threshold_ticks:
            with self._state.lock:
                self._state.tx_silent = True
            # Measured controller behavior (tx_silence_backstop_ok reads
            # INTERIM_FACTS.tx_silence_backstop_ok = False): the controller coasts
            # at the last commanded velocity, it does not fast-decel.
            self._plant.silence_step(dt, backstop_ok=self._cfg.tx_silence_backstop_ok)
        else:
            # Brief within-tolerance gap: hold with the safe backstop.
            self._plant.silence_step(dt, backstop_ok=True)

    def _maybe_trip_deviation_fault(self) -> None:
        if self._plant.watchdog_tripped:
            with self._state.lock:
                if not self._state.deviation_fault_latched:
                    self._state.trip_deviation_fault(
                        self._plant.watchdog_joint, self._plant.watchdog_step_deg
                    )

    # -- status TX --------------------------------------------------------

    def drop_status(self, n: int = 1) -> None:
        """Skip sending the next ``n`` status packets (RX-silence / drop)."""
        self._drop_status_countdown += int(n)

    def inject_seq_gap(self, delta: int = 1) -> None:
        """Add ``delta`` to the status sequence counter on the next packet."""
        self._pending_seq_gap += int(delta)

    def duplicate_next_status(self) -> None:
        """Emit the next status packet twice (byte-identical duplicate)."""
        self._dup_next = True

    def _send_status(self, dt: float) -> None:
        if self._drop_status_countdown > 0:
            self._drop_status_countdown -= 1
            return
        self._status_seq = (self._status_seq + 1 + self._pending_seq_gap) & 0xFFFF_FFFF
        self._pending_seq_gap = 0
        with self._state.lock:
            robot_status = self._state.robot_status_byte()
            motion_possible = self._state.motion_possible()
            contact_stop = int(self._state.contact_stop)
            safety_scale = float(self._state.safety_scale)
            fx, fy, fz = self._state.force
            mx, my, mz = self._state.moment
            fs_type = self._state.effective_fs_type(self._fsconfig_received)

        q_meas, qd_meas = self._plant.snapshot_deg()
        moving = bool(np.max(np.abs(qd_meas)) > _MOTION_IN_PROGRESS_EPS_DEG_S)
        status_byte = 0
        if motion_possible:
            status_byte |= 0x1
        if moving:
            status_byte |= 0x8

        ts_ms = int(self._clock.now() * 1000.0) & 0xFFFF_FFFF
        if self._negotiated_version >= 4:
            # v4+: type-204 status (force/torque + fs_type). Needs the FSConfig
            # handshake to have selected a force sensor.
            pkt = wire.build_status_204_packet(
                version_no=self._negotiated_version,
                sequence_no=self._status_seq,
                status=status_byte,
                robot_status=robot_status,
                contact_stop_status=contact_stop,
                time_stamp=ts_ms,
                position_deg=[0.0] * N_AXES,  # fake does no FK; joints are authoritative
                joint_angle_deg=list(q_meas),
                current=[0.0] * N_AXES,
                safety_scale=safety_scale,
                force_x=fx,
                force_y=fy,
                force_z=fz,
                moment_x=mx,
                moment_y=my,
                moment_z=mz,
                fs_type=fs_type,
            )
        else:
            # v3: legacy type-202 status — NO force block. This is what a real
            # V9.40/P82 CRX streams (388 B; confirmed by packet capture).
            # Emitting it here is what makes the default fake exercise the real wire.
            pkt = wire.build_status_v3_packet(
                version_no=self._negotiated_version,
                sequence_no=self._status_seq,
                status=status_byte,
                robot_status=robot_status,
                contact_stop_status=contact_stop,
                time_stamp=ts_ms,
                position_deg=[0.0] * N_AXES,  # fake does no FK; joints are authoritative
                joint_angle_deg=list(q_meas),
                current=[0.0] * N_AXES,
                safety_scale=safety_scale,
            )
        n_sends = 2 if self._dup_next else 1
        self._dup_next = False
        for _ in range(n_sends):
            try:
                self._sock.sendto(pkt, self._peer_addr)  # type: ignore[arg-type]
            except OSError:
                pass

    # -- real-time driver (downstream integration/soak use) ---------------

    def start_realtime(self, *, speed: float = 1.0) -> None:
        """Spawn a wall-paced daemon thread calling :meth:`tick` at 125 Hz.

        Advances the installed clock by one ITP per tick (so status timestamps
        stay perfectly periodic, matching a PLL-clocked controller) and paces
        the loop to real time / ``speed``. Not used by the deterministic
        self-tests, which drive :meth:`tick` directly.
        """
        if self._realtime_thread is not None:
            raise RuntimeError("real-time driver already running")
        self._realtime_stop.clear()
        self.realtime_error = None
        self._realtime_thread = threading.Thread(
            target=self._realtime_loop, args=(speed,), name="fake-crx-sm", daemon=True
        )
        self._realtime_thread.start()

    def _realtime_loop(self, speed: float) -> None:
        period = self.itp_s / max(1e-6, speed)
        try:
            while not self._realtime_stop.is_set():
                t0 = time.monotonic()
                if isinstance(self._clock, ManualClock):
                    self._clock.advance(self.itp_s)
                n_before = len(self._violations)
                # RECORD-but-keep-streaming: a strict violation (false or real)
                # must not halt the realtime stream, or the core cascades to
                # RX_SILENT and every integration test inherits a phantom fault.
                self.tick(self.itp_s, raise_on_violation=False)
                if len(self._violations) > n_before:
                    self.realtime_error = StrictConformanceError(self._violations[n_before])
                sleep_s = period - (time.monotonic() - t0)
                if sleep_s > 0:
                    self._realtime_stop.wait(sleep_s)
        except BaseException as exc:  # pragma: no cover - defensive
            self.realtime_error = exc

    def stop_realtime(self) -> None:
        self._realtime_stop.set()
        if self._realtime_thread is not None:
            self._realtime_thread.join(timeout=2.0)
            self._realtime_thread = None

    # -- teardown ---------------------------------------------------------

    def close(self) -> None:
        self.stop_realtime()
        try:
            self._sock.close()
        except OSError:
            pass

    def _record_violation(self, msg: str) -> None:
        self._violations.append(msg)
