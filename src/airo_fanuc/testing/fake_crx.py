# SPDX-License-Identifier: Apache-2.0
"""FakeCRXController — strict-conformance software emulator of the FANUC
CRX-10iA/L controller.

This facade owns the shared controller state and wires together the three
subsystems that together *are* the executable spec the C++ core + Python
lifecycle are tested against:

* :class:`~airo_fanuc.testing.fake_crx_sm.FakeStreamMotionServer` — UDP Stream
  Motion (handshake + 125 Hz status [type-202 at the v3 default, type-204 at v4]
  + type-201 command consumption).
* :class:`~airo_fanuc.testing.fake_crx_rmi.FakeRmiServer` — RMI JSON/TCP
  (bootstrap→redirect, FRC_* vocabulary, ladders, GRIPDISP, async push).
* :class:`~airo_fanuc.testing.plant.JointPlant` — first-order-lag servo model +
  deviation watchdog + TX-silence backstop.

The two protocol planes are coupled through :class:`ControllerState`: Stream
Motion ``motion_possible`` (status bit 0) asserts only after a StartPacket AND
the RMI side has accepted ``FRC_Call("STREAM_MOTN")``.

Determinism: with the default :class:`~airo_fanuc.testing.plant.ManualClock`,
call :meth:`tick` / :meth:`advance` to step the controller in exact ITP
increments — no wall-time dependence — so scenario tests are fast and
reproducible. :meth:`start_realtime` provides a wall-paced driver for
downstream integration/soak use.

Strict-conformance mode (default ON) makes any non-conformant command the
driver-under-test emits raise :class:`StrictConformanceError` (0-dataStyle,
do_motn_ctrl≠1, non-zero io, version drift, non-monotonic seq, and the
one-TX-per-window / double-send guard) — if the fake lies about controller
behavior, every downstream test inherits the lie, so it fails loudly instead.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from airo_fanuc.controller_facts import INTERIM_FACTS, ITP_S
from airo_fanuc.testing import wire
from airo_fanuc.testing.fake_crx_rmi import FakeRmiServer
from airo_fanuc.testing.fake_crx_sm import FakeStreamMotionServer, StrictConformanceError
from airo_fanuc.testing.plant import N_AXES, Clock, JointPlant, ManualClock, WallClock

# Wire force-sensor type IDs (re-exported convenience).
FS_TYPE_UNAVAILABLE = wire.FORCE_SENSOR_TYPE_UNAVAILABLE
FS_TYPE_EMBEDDED = wire.FORCE_SENSOR_TYPE_EMBEDDED


@dataclass
class FakeCRXConfig:
    """Knobs for the fake. Values measured on the physical controller are read
    from :mod:`airo_fanuc.controller_facts` — never hardcoded here, so a
    re-measure updates in one place. Mutable so a scenario can, e.g., toggle
    strict mode mid-run."""

    strict: bool = True
    itp_s: float = ITP_S

    # Stream Motion capability negotiation.
    available_version: int = 3  # set 4 to exercise the FSConfig / force path

    # Plant. tau_s reads INTERIM_FACTS.tracking_lag_s (measured servo lag: 25 ms).
    tau_s: float = INTERIM_FACTS.tracking_lag_s
    # deviation_watchdog_deg reads INTERIM_FACTS.deviation_watchdog_deg (measured: 5.0°).
    deviation_watchdog_deg: float = INTERIM_FACTS.deviation_watchdog_deg
    deviation_watchdog_enabled: bool = True
    silence_decel_ticks: int = 3

    # TX-silence. tx_silence_backstop_ok reads INTERIM_FACTS.tx_silence_backstop_ok
    # (measured False: the controller coasts, it does not fast-decel).
    tx_silence_backstop_ok: bool = INTERIM_FACTS.tx_silence_backstop_ok
    tx_silence_threshold_ticks: int = 2

    # RMI. single_session reads INTERIM_FACTS.rmi_single_session (measured: the
    # controller serves one RMI session at a time).
    single_session: bool = INTERIM_FACTS.rmi_single_session
    seq_seed: int = 1  # controller NextSequenceID seed (persists across sessions)
    major_version: int = 9
    minor_version: int = 40
    send_call_ack: bool = True
    gripdisp_clear_after_reads: int = 1


class ControllerState:
    """Shared, lock-guarded controller state coupling the SM + RMI planes.

    Holds the physical/latched signals both protocol servers read or write; the
    facade constructs exactly one and hands it to each server.
    """

    def __init__(self, plant: JointPlant) -> None:
        import threading

        self.lock = threading.RLock()
        self._plant = plant

        # Physical / safety signals.
        self.e_stopped: bool = False
        self.in_error: bool = False
        self.tp_enabled: bool = False  # TEACH (T1/T2)
        self.contact_stop: int = int(wire.ContactStopStatus.NONE)
        self.safety_scale: float = 1.0
        self.motion_possible_forced_off: bool = False

        # Force/torque (tool frame, N / Nm) + fs_type.
        self.force: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.moment: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.fs_type: int = FS_TYPE_EMBEDDED

        # FACEPLATE Cartesian pose: X, Y, Z (mm), W, P, R (deg), then 3 extended axes.
        # This is what the SM status `position` block streams — measured on the real
        # controller, that block is the faceplate and NOT the tool tip
        # (controller-notes.md §1.10).
        #
        # The fake does NO forward kinematics: this is whatever a test injects, so it
        # does not track `plant` joints. That bounds what a fake-based test can prove —
        # that the pose survives the wire, the decode and the seqlock publish.
        self.cartesian: tuple[float, ...] = (0.0,) * N_AXES

        # TOOL-TIP Cartesian pose, reported by FRC_ReadCartesianPosition. Held
        # SEPARATELY from `cartesian` rather than derived from it, because the fake
        # computes nothing — and because the split is the point: on the physical
        # controller the RMI read applies the active tool and the SM stream does not,
        # 175 mm apart on this cell (§1.10). `None` means "same as the faceplate",
        # which is what a controller with an empty UTOOL reports.
        self.cartesian_tcp: tuple[float, ...] | None = None

        # Active frame/tool numbers, reported by FRC_GetStatus and named in the
        # FRC_ReadCartesianPosition Configuration block.
        self.number_utool: int = 1
        self.number_uframe: int = 1

        # Coupling flags.
        self.stream_started: bool = False  # SM StartPacket accepted
        self.stream_motn_launched: bool = False  # RMI FRC_Call(STREAM_MOTN)
        self.tx_silent: bool = False
        self.deviation_fault_latched: bool = False

        # RMI ext-status signals.
        self.in_motion: bool = False
        self.gen_override_pct: int = 100
        self.speed_clamp_limit_pct: float = 100.0
        self.error_code: str | None = None
        self.control_mode: str | None = "RemoteControl"

        # Alarm strings surfaced by FRC_ReadError (most-recent first).
        self.alarm_texts: list[str] = []

    # -- derived signals --------------------------------------------------

    def robot_status_byte(self) -> int:
        """Stream Motion robot_status: 0x1 in_error | 0x2 tp_enabled | 0x4 e_stop."""
        b = 0
        if self.in_error:
            b |= 0x1
        if self.tp_enabled:
            b |= 0x2
        if self.e_stopped:
            b |= 0x4
        return b

    def motion_possible(self) -> bool:
        return (
            self.stream_started
            and self.stream_motn_launched
            and not self.e_stopped
            and not self.in_error
            and not self.motion_possible_forced_off
        )

    def servo_ready(self) -> bool:
        return not self.e_stopped and not self.in_error

    def drives_powered(self) -> bool:
        return not self.e_stopped

    def effective_fs_type(self, fsconfig_received: bool) -> int:
        # Force telemetry only streams once the host has selected a sensor
        # (v>=4 FSConfig). Before that the controller reports Unavailable.
        return int(self.fs_type) if fsconfig_received else FS_TYPE_UNAVAILABLE

    # -- fault helpers (called under self.lock) ---------------------------

    def raise_alarm(self, error_id: str, text: str) -> None:
        entry = f"{error_id} {text}".strip()
        if entry not in self.alarm_texts:
            self.alarm_texts.insert(0, entry)
        self.error_code = error_id

    def trip_deviation_fault(self, joint: int | None, step_deg: float) -> None:
        self.in_error = True
        self.deviation_fault_latched = True
        self.raise_alarm(
            "SRVO-023",
            f"Stop error excess (J{(joint or 0) + 1}, step {step_deg:.2f} deg)",
        )

    def clear_recoverable_faults(self) -> None:
        """FRC_Reset / FRC_Initialize semantics: clear recoverable faults but
        NOT a physically-held e-stop button."""
        self.in_error = False
        self.deviation_fault_latched = False
        self.motion_possible_forced_off = False
        if self.contact_stop in (
            int(wire.ContactStopStatus.STOP),
            int(wire.ContactStopStatus.ESCP),
        ):
            self.contact_stop = int(wire.ContactStopStatus.NONE)
        self.alarm_texts.clear()
        self.error_code = None
        self._plant.reset_watchdog()


class FakeCRXController:
    """Facade over the SM + RMI servers and the plant. Start it, read
    ``.sm_port`` / ``.rmi_port``, then :meth:`tick`/:meth:`advance` (manual
    clock) or :meth:`start_realtime` (wall-paced)."""

    def __init__(
        self,
        config: FakeCRXConfig | None = None,
        *,
        host: str = "127.0.0.1",
        sm_port: int = 0,
        rmi_bootstrap_port: int = 0,
        rmi_redirect_port: int = 0,
        clock: Clock | None = None,
        initial_q_deg: np.ndarray | list[float] | None = None,
    ) -> None:
        self.config = config or FakeCRXConfig()
        self.clock: Clock = clock if clock is not None else ManualClock()

        self.plant = JointPlant(
            tau_s=self.config.tau_s,
            deviation_watchdog_deg=self.config.deviation_watchdog_deg,
            deviation_watchdog_enabled=self.config.deviation_watchdog_enabled,
            silence_decel_ticks=self.config.silence_decel_ticks,
            itp_s=self.config.itp_s,
            initial_q_deg=initial_q_deg if initial_q_deg is not None else [0.0] * N_AXES,
        )
        self.state = ControllerState(self.plant)
        self.sm = FakeStreamMotionServer(
            self.state, self.plant, self.clock, self.config, host=host, port=sm_port
        )
        self.rmi = FakeRmiServer(
            self.state,
            self.config,
            host=host,
            bootstrap_port=rmi_bootstrap_port,
            redirect_port=rmi_redirect_port,
        )
        if self.config.seq_seed != 1:
            self.rmi.set_next_sequence_id(self.config.seq_seed)
        self._started = False

    # -- lifecycle --------------------------------------------------------

    def start(self) -> FakeCRXController:
        if not self._started:
            self.rmi.start()
            self._started = True
        return self

    def close(self) -> None:
        self.sm.close()
        self.rmi.close()
        self._started = False

    def __enter__(self) -> FakeCRXController:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- ports ------------------------------------------------------------

    @property
    def sm_port(self) -> int:
        return self.sm.port

    @property
    def rmi_port(self) -> int:
        """RMI bootstrap port (the client connects here first)."""
        return self.rmi.bootstrap_port

    @property
    def rmi_redirect_port(self) -> int:
        return self.rmi.redirect_port

    # -- clock / stepping -------------------------------------------------

    def tick(self, n: int = 1) -> None:
        """Advance ``n`` controller ITPs (default 1). With a ManualClock the
        clock advances one ITP per tick before the plant/status step."""
        for _ in range(int(n)):
            if isinstance(self.clock, ManualClock):
                self.clock.advance(self.config.itp_s)
            self.sm.tick(self.config.itp_s)

    def advance(self, n: int = 1) -> None:
        """Alias for :meth:`tick` reading better in loops (advance many ITPs)."""
        self.tick(n)

    def start_realtime(self, *, speed: float = 1.0) -> None:
        self.sm.start_realtime(speed=speed)

    def stop_realtime(self) -> None:
        self.sm.stop_realtime()

    # -- observability ----------------------------------------------------

    @property
    def q_meas_deg(self) -> np.ndarray:
        return self.plant.q_meas.copy()

    @property
    def qd_meas_deg_s(self) -> np.ndarray:
        return self.plant.qd_meas.copy()

    @property
    def motion_possible(self) -> bool:
        with self.state.lock:
            return self.state.motion_possible()

    @property
    def tx_silent(self) -> bool:
        return self.state.tx_silent

    @property
    def violations(self) -> list[str]:
        return self.sm.violations

    @property
    def last_gripper_command(self) -> dict[str, int] | None:
        return self.rmi.last_gripper_command

    @property
    def gripdisp_running(self) -> bool:
        return self.rmi.gripdisp_running

    # -- Stream-Motion fault injection ------------------------------------

    def press_estop(self) -> None:
        with self.state.lock:
            self.state.e_stopped = True
            # in_error latches: it survives e-stop release until an FRC_Reset, so
            # recovery needs the ladder, not just releasing the button.
            self.state.in_error = True
            self.state.raise_alarm("SRVO-001", "Operator panel E-stop")

    def release_estop(self) -> None:
        with self.state.lock:
            self.state.e_stopped = False  # in_error may persist until FRC_Reset

    def set_teach(self, on: bool) -> None:
        with self.state.lock:
            self.state.tp_enabled = bool(on)

    def set_contact_stop(self, mode: int) -> None:
        """mode in {0 NONE, 1 SAFE, 2 STOP, 3 DSBL, 4 ESCP}."""
        with self.state.lock:
            self.state.contact_stop = int(mode)

    def set_safety_scale(self, scale: float) -> None:
        with self.state.lock:
            self.state.safety_scale = float(scale)

    def set_force(self, fx: float, fy: float, fz: float) -> None:
        with self.state.lock:
            self.state.force = (float(fx), float(fy), float(fz))

    def set_moment(self, mx: float, my: float, mz: float) -> None:
        with self.state.lock:
            self.state.moment = (float(mx), float(my), float(mz))

    def set_cartesian(
        self,
        xyzwpr: Sequence[float],
        *,
        utool: int | None = None,
        uframe: int | None = None,
    ) -> None:
        """Set the FACEPLATE pose the SM plane streams: X, Y, Z (mm), W, P, R (deg).

        Accepts 6 values (extended axes zero-filled) or all :data:`N_AXES`. The fake
        does no FK, so this does not follow the plant — see
        :attr:`ControllerState.cartesian`. ``FRC_ReadCartesianPosition`` reports
        :meth:`set_cartesian_tcp` instead, or this pose if none was set.
        """
        values = [float(v) for v in xyzwpr]
        if len(values) not in (6, N_AXES):
            raise ValueError(f"set_cartesian: expected 6 or {N_AXES} values (got {len(values)})")
        values += [0.0] * (N_AXES - len(values))
        with self.state.lock:
            self.state.cartesian = tuple(values)
            if utool is not None:
                self.state.number_utool = int(utool)
            if uframe is not None:
                self.state.number_uframe = int(uframe)

    def set_cartesian_tcp(self, xyzwpr: Sequence[float] | None) -> None:
        """Set the TOOL-TIP pose ``FRC_ReadCartesianPosition`` reports (mm / deg).

        Independent of :meth:`set_cartesian` on purpose — the fake computes no tool
        composition, it just serves the two poses it was given, which is how the real
        controller behaves (the RMI read applies the active tool, the SM stream does
        not; controller-notes.md §1.10). ``None`` makes the RMI read report the
        faceplate, as a controller with an empty UTOOL does.
        """
        if xyzwpr is None:
            with self.state.lock:
                self.state.cartesian_tcp = None
            return
        values = [float(v) for v in xyzwpr]
        if len(values) not in (6, N_AXES):
            raise ValueError(f"set_cartesian_tcp: expected 6 or {N_AXES} values (got {len(values)})")
        values += [0.0] * (N_AXES - len(values))
        with self.state.lock:
            self.state.cartesian_tcp = tuple(values)

    def drop_motion_possible(self) -> None:
        with self.state.lock:
            self.state.motion_possible_forced_off = True

    def restore_motion_possible(self) -> None:
        with self.state.lock:
            self.state.motion_possible_forced_off = False

    def inject_alarm(self, error_id: str, text: str, *, in_error: bool = True) -> None:
        """Script an alarm (SRVO-xxx / MOTN-607 / SYST-348 / HOST-380)."""
        with self.state.lock:
            self.state.raise_alarm(error_id, text)
            if in_error:
                self.state.in_error = True

    def set_ext_status(
        self,
        *,
        gen_override_pct: int | None = None,
        speed_clamp_limit_pct: float | None = None,
        in_motion: bool | None = None,
        error_code: str | None = None,
        control_mode: str | None = None,
    ) -> None:
        with self.state.lock:
            if gen_override_pct is not None:
                self.state.gen_override_pct = int(gen_override_pct)
            if speed_clamp_limit_pct is not None:
                self.state.speed_clamp_limit_pct = float(speed_clamp_limit_pct)
            if in_motion is not None:
                self.state.in_motion = bool(in_motion)
            if error_code is not None:
                self.state.error_code = error_code
            if control_mode is not None:
                self.state.control_mode = control_mode

    # -- Stream-Motion TX perturbation (drop / seq gap / duplicate) -------

    def drop_status(self, n: int = 1) -> None:
        """Skip sending the next ``n`` status packets (RX-silence / drop)."""
        self.sm.drop_status(n)

    def inject_seq_gap(self, delta: int = 1) -> None:
        """Add ``delta`` to the status sequence counter on the next packet."""
        self.sm.inject_seq_gap(delta)

    def duplicate_next_status(self) -> None:
        """Send the next status packet twice (dup)."""
        self.sm.duplicate_next_status()

    # -- RMI fault injection ----------------------------------------------

    def arm_rmi_error(self, command: str, error_id: int, *, times: int = 1) -> None:
        self.rmi.arm_error(command, error_id, times=times)

    def arm_syst_348(self, *, on: str = "FRC_Initialize") -> None:
        self.rmi.arm_syst_348(on=on)

    def push_system_fault(self, *, event_id: int | None = None) -> bool:
        return self.rmi.push_system_fault(event_id=event_id)


__all__ = [
    "FakeCRXController",
    "FakeCRXConfig",
    "ControllerState",
    "StrictConformanceError",
    "FakeStreamMotionServer",
    "FakeRmiServer",
    "JointPlant",
    "ManualClock",
    "WallClock",
    "Clock",
    "FS_TYPE_EMBEDDED",
    "FS_TYPE_UNAVAILABLE",
]
