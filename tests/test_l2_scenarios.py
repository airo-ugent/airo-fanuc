# SPDX-License-Identifier: Apache-2.0
"""L2 scenario matrix — the C++ ``StreamCore`` driven against the REAL
``FakeCRXController`` in realtime mode (PLAN.md §8, report 09 §4 S-* matrix).

This is the regression wall that keeps the RT core correct as P4b/P5 build on
it. Unlike ``test_fake_crx_scenarios.py`` (which manual-ticks the fake to prove
the *fake* is faithful), every test here spins up the fake's wall-paced 125 Hz
thread AND the C++ core's real RT thread and asserts real state transitions
observed through ``get_snapshot`` / ``motion_status`` / ``poll_events`` — never
sleeps-and-hope. Fault scenarios use the FakeCRX injectors + explicit polling.

**Strict conformance is always-on**: the fake runs ``strict=True`` and every
test asserts, at teardown, that the core emitted ZERO wire-conformance
violations (dataStyle 0xFFFF, one-TX min-inter-spacing, monotone seq, version
echo, ...) and that the fake's realtime loop recorded no error. A single
violation fails the owning test — a conformant core is the invariant.

Coverage vs the report-09 / §8 matrix (row → status):

    Bring-up (preroll→HOLD, wait_ready, rx freshness)  → covered
    Trajectory CAPTURE→TRAJECTORY→settle→DONE + tracking → covered
    speed_scale 0.5 halves velocity                     → covered
    S-B4 strict-mode always-on (zero violations)        → covered (every test)
    S-B12 stop_j during running trajectory → STOPPED    → covered
    S-B15 preempt-at-speed brake reaches rest           → covered
    S-LIFE e-stop → SAFE_FOLLOW(E_STOP) → recover→HOLD   → covered
    S-LIFE contact_stop ∈ {2,4} kill → SAFE_FOLLOW       → covered
    S-LIFE safety_scale clamp → SAFE_FOLLOW(SAFETY_CLAMP) → covered
    S-LIFE TEACH toggle → SAFE_FOLLOW(TEACH_MODE)        → covered
    S-LIFE motion_possible drop → SAFE_FOLLOW            → covered
    S-LIFE FRC_SystemFault push (RMI async ring)         → covered (RMI-plane)
    S-DROP 100 ms → in-flight FAULTED(RX_DEGRADED)       → covered
    S-DROP 500 ms → RX_SILENT (TX parked)                → covered
    S-DROP seq-gap + duplicate (counter/no-crash)        → covered
    S-VERS v3 + v4 bring-up (version echo, FSConfig gate) → covered
    Capture settle→3° executes; capture-or-reject >5°    → covered
    Capture ≥200 ms splice executes                      → covered
    Servo sine + direction reversal (no dwell)           → covered
    Servo 5° distance-guard reject                       → covered
    S-FLAP dries 9-flap / 7-oscillation dwell (F7)       → covered (constructed)

Deferred to HIL (L4) / P4b, noted where relevant:
    * ``MotionHandle.trajectory_start_mono_ns`` post-capture anchor value — the
      P4b FanucDriver surface; StreamCore exposes no per-motion anchor, so the
      ≥200 ms capture row asserts the *executed* capture duration instead.
    * FRC_SystemFault → supervisor lifecycle reaction (P4b): StreamCore is
      SM-only, so the push is asserted on the RMI async ring; the SM-plane
      in_error that accompanies a real system fault is covered separately.
    * FSConfig actually emitted on v4 (force streaming) — the P3b StreamCore
      does not send type-205, so fs_type stays Unavailable; the v4 row asserts a
      clean version-4 handshake (the gate that matters for HOST-380 on v3).
    * pcap trajectory-diff, physical lag ±20 ms, brake distance vs Ruckig — HIL.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Iterator

import pytest

from airo_fanuc._core import FaultReason, Mode, MotionStatus, RtCoreConfig, StreamCore
from airo_fanuc.rmi_client import RmiClient
from airo_fanuc.testing import FakeCRXConfig, FakeCRXController

# ---------------------------------------------------------------------------
# Realtime rig + polling helpers (no fixed sleeps for state assertions).
# ---------------------------------------------------------------------------

_READY_TIMEOUT_S = 5.0

# EventType::kModeChange ordinal (bindings expose raw ints; kModeChange is the
# first EventType in rt_core_config.hpp). For a mode-change event, `value` is the
# new Mode ordinal.
_EVENT_MODE_CHANGE = 0


def _poll(fn: Callable[[], bool], timeout_s: float = 3.0, interval_s: float = 0.004) -> bool:
    """Return True as soon as ``fn()`` is truthy, else False after ``timeout_s``."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if fn():
            return True
        time.sleep(interval_s)
    return False


class Rig:
    """Bring up FakeCRX (realtime) + RMI STREAM_MOTN + StreamCore; tear all down.

    Asserts (on close) zero strict violations and no fake realtime error — the
    always-on wire-conformance guarantee for every L2 scenario.
    """

    def __init__(
        self,
        *,
        available_version: int = 3,
        core_sm_version: int | None = None,
        initial_q_deg: list[float] | None = None,
    ) -> None:
        self.controller = FakeCRXController(
            FakeCRXConfig(strict=True, available_version=available_version),
            initial_q_deg=initial_q_deg if initial_q_deg is not None else [0.0] * 9,
        )
        self.controller.start()
        self.rmi = RmiClient("127.0.0.1", self.controller.rmi_port)
        self.rmi.start()
        self.rmi.initialize()
        self.rmi.program_call("STREAM_MOTN")
        self.rmi.get_status()
        self.controller.start_realtime(speed=1.0)
        cfg = RtCoreConfig()
        cfg.sm_version = core_sm_version if core_sm_version is not None else available_version
        self.core = StreamCore("127.0.0.1", self.controller.sm_port, cfg)
        self.core.start()
        self.ready = self.core.wait_ready(_READY_TIMEOUT_S)

    # -- snapshot conveniences -------------------------------------------
    def mode(self) -> Mode:
        return Mode(self.core.get_snapshot()["mode"])

    def fault(self) -> FaultReason:
        return FaultReason(self.core.get_snapshot()["fault"])

    def status(self, motion_id: int) -> MotionStatus:
        return MotionStatus(self.core.motion_status(motion_id))

    def wait_mode(self, mode: Mode, timeout_s: float = 3.0) -> bool:
        return _poll(lambda: self.mode() == mode, timeout_s)

    def wait_status(self, motion_id: int, status: MotionStatus, timeout_s: float = 3.0) -> bool:
        return _poll(lambda: self.status(motion_id) == status, timeout_s)

    def violations(self) -> list[str]:
        return list(self.controller.violations)

    # -- teardown --------------------------------------------------------
    def close(self) -> None:
        # Snapshot the conformance state BEFORE teardown TX (Stop packet etc.).
        viol = list(self.controller.violations)
        rt_err = self.controller.sm.realtime_error
        try:
            self.core.stop()
        finally:
            self.controller.stop_realtime()
            try:
                self.rmi.stop()
            finally:
                self.controller.close()
        assert viol == [], f"strict wire-conformance violations: {viol}"
        assert rt_err is None, f"fake realtime loop errored: {rt_err!r}"


@pytest.fixture
def rig() -> Iterator[Rig]:
    r = Rig()
    assert r.ready, "StreamCore did not reach STREAMING within the preroll timeout"
    # wait_ready flips the internal STREAMING flag a beat before the seqlock
    # snapshot publishes HOLD (the first read can still return the default
    # STREAM_DOWN snapshot); confirm the published state before the test runs.
    assert r.wait_mode(Mode.HOLD, 3.0), "core did not publish HOLD after preroll"
    try:
        yield r
    finally:
        r.close()


_TRAJ_QD_REST = [[0.0] * 6, [0.0] * 6]


def _traj(target0: float, duration_ns: int = 1_000_000_000, start0: float = 0.0) -> tuple:
    """One-segment J0 trajectory (start0 → target0 rad) over ``duration_ns``."""
    times = [0, duration_ns]
    q = [[start0, 0, 0, 0, 0, 0], [target0, 0, 0, 0, 0, 0]]
    return times, q, _TRAJ_QD_REST


# ---------------------------------------------------------------------------
# Bring-up
# ---------------------------------------------------------------------------


def test_bringup_prerolls_to_hold_with_fresh_rx(rig: Rig) -> None:
    # Fixture already confirmed HOLD is published.
    assert rig.mode() == Mode.HOLD
    snap = rig.core.get_snapshot()
    assert snap["motion_possible"] is True
    # RX is fresh in steady HOLD (status arriving every ~8 ms).
    assert _poll(lambda: rig.core.get_snapshot()["rx_fresh"] is True, 1.0)
    assert 0.0 <= rig.core.get_snapshot()["rx_age_ms"] < 60.0, snap["rx_age_ms"]
    # A steady stream of TXes accumulates (TX is parked during PREROLL and starts
    # at HOLD), with the one-TX-per-window invariant intact throughout.
    assert _poll(lambda: rig.core.timing_stats()["tx_count"] > 20, 1.5), "TX cadence flowing"
    assert rig.core.timing_stats()["double_send_guard"] == 0


# ---------------------------------------------------------------------------
# Trajectory: CAPTURE → TRAJECTORY → settle → DONE + tracking; speed_scale
# ---------------------------------------------------------------------------


def test_trajectory_executes_to_done_and_plant_tracks(rig: Rig) -> None:
    times, q, qd = _traj(0.4)
    mid = rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.CAPTURE, 1.0) or rig.wait_mode(Mode.TRAJECTORY, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5), "reaches TRAJECTORY playback"
    assert rig.wait_status(mid, MotionStatus.DONE, 3.0), "settles to DONE"
    assert rig.mode() == Mode.HOLD
    # Plant tracked the commanded target to within the first-order lag residual.
    assert abs(rig.core.get_snapshot()["q_meas"][0] - 0.4) < 0.02


def test_speed_scale_half_halves_peak_velocity(rig: Rig) -> None:
    # Same target/time at s=1.0 vs s=0.5: half-speed peak |qd_cmd| ≈ 0.5×.
    def peak_qd(rig_: Rig, scale: float) -> float:
        # Start each move from the CURRENT commanded pose (the previous move left
        # the robot at its target — starting from 0 would capture-reject).
        start = rig_.core.get_snapshot()["q_cmd"][0]
        times = [0, 1_500_000_000]
        q = [[start, 0, 0, 0, 0, 0], [start + 0.5, 0, 0, 0, 0, 0]]
        rig_.core.submit_trajectory(times, q, _TRAJ_QD_REST, scale)
        assert rig_.wait_mode(Mode.TRAJECTORY, 2.0)
        peak = 0.0
        deadline = time.monotonic() + 4.0  # s=0.5 stretches a 1.5 s traj to 3.0 s wire
        while time.monotonic() < deadline and rig_.mode() == Mode.TRAJECTORY:
            peak = max(peak, abs(rig_.core.get_snapshot()["qd_cmd"][0]))
            time.sleep(0.004)
        return peak

    peak_full = peak_qd(rig, 1.0)
    assert rig.wait_mode(Mode.HOLD, 2.0)
    peak_half = peak_qd(rig, 0.5)
    assert peak_full > 0.05, peak_full
    # 0.5× speed scale ⇒ ~0.5× peak velocity (generous band for sampling jitter).
    assert 0.35 < peak_half / peak_full < 0.65, (peak_half, peak_full)


# ---------------------------------------------------------------------------
# stop_j during a running trajectory (S-B12 / S-B15)
# ---------------------------------------------------------------------------


def test_stop_j_during_trajectory_brakes_and_resolves_stopped(rig: Rig) -> None:
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    mid = rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    time.sleep(0.25)  # let it build real velocity (preempt-at-speed, S-B15)
    moving_qd = abs(rig.core.get_snapshot()["qd_cmd"][0])
    assert moving_qd > 0.05, "trajectory is actually moving before stop_j"
    rig.core.stop_j()
    assert rig.wait_mode(Mode.BRAKE, 1.0), "stop_j → BRAKE"
    assert rig.wait_status(mid, MotionStatus.STOPPED, 3.0), "motion resolves STOPPED"
    # Brake settles back to a steady HOLD at rest.
    assert rig.wait_mode(Mode.HOLD, 3.0)
    assert abs(rig.core.get_snapshot()["qd_cmd"][0]) < 0.05
    assert rig.core.get_snapshot()["epoch"] == 0, "clean preempt does not bump epoch"


# ---------------------------------------------------------------------------
# S-LIFE — fault matrix
# ---------------------------------------------------------------------------


def test_estop_kills_to_safe_follow_then_recovers(rig: Rig) -> None:
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    mid = rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    rig.controller.press_estop()
    assert rig.wait_mode(Mode.SAFE_FOLLOW, 2.0), "e-stop kills the motion → SAFE_FOLLOW"
    assert rig.fault() == FaultReason.E_STOP
    assert rig.wait_status(mid, MotionStatus.FAULTED, 2.0)
    # Release + FRC_Reset (recovery ladder) clears the latched controller fault;
    # recover() leaves SAFE_FOLLOW once the anti-flap dwell is satisfied.
    rig.controller.release_estop()
    rig.rmi.reset()
    rig.core.recover()
    assert rig.wait_mode(Mode.HOLD, 3.0), "recover() → HOLD after the dwell"
    assert rig.fault() == FaultReason.NONE


@pytest.mark.parametrize("contact_mode", [2, 4])
def test_contact_stop_kills_to_safe_follow(rig: Rig, contact_mode: int) -> None:
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    rig.controller.set_contact_stop(contact_mode)  # STOP(2) / ESCP(4)
    assert rig.wait_mode(Mode.SAFE_FOLLOW, 2.0)
    assert rig.fault() == FaultReason.CONTACT_STOP
    # Clearing contact + recover returns to HOLD.
    rig.controller.set_contact_stop(0)
    rig.core.recover()
    assert rig.wait_mode(Mode.HOLD, 3.0)


def test_safety_scale_clamp_kills_to_safe_follow(rig: Rig) -> None:
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    rig.controller.set_safety_scale(0.02)  # < safety_scale_min (0.05)
    assert rig.wait_mode(Mode.SAFE_FOLLOW, 2.0)
    assert rig.fault() == FaultReason.SAFETY_CLAMP
    rig.controller.set_safety_scale(1.0)
    rig.core.recover()
    assert rig.wait_mode(Mode.HOLD, 3.0)


def test_teach_toggle_kills_to_safe_follow_and_states_flow(rig: Rig) -> None:
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    rig.controller.set_teach(True)  # TEACH (T1)
    assert rig.wait_mode(Mode.SAFE_FOLLOW, 2.0)
    assert rig.fault() == FaultReason.TEACH_MODE
    # States keep flowing in TEACH — RX stays fresh (S-B16 / B19 T1-freeze fix).
    assert rig.core.get_snapshot()["rx_fresh"] is True
    assert rig.core.get_snapshot()["tp_enabled"] is True
    rig.controller.set_teach(False)
    rig.core.recover()
    assert rig.wait_mode(Mode.HOLD, 3.0)


def test_motion_possible_drop_kills_to_safe_follow(rig: Rig) -> None:
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    rig.controller.drop_motion_possible()
    assert rig.wait_mode(Mode.SAFE_FOLLOW, 2.0)
    assert rig.fault() == FaultReason.MOTION_NOT_POSSIBLE
    rig.controller.restore_motion_possible()
    rig.core.recover()
    assert rig.wait_mode(Mode.HOLD, 3.0)


def test_system_fault_push_on_rmi_ring_and_in_error_kills_sm(rig: Rig) -> None:
    # FRC_SystemFault is an RMI-plane async push (P4b supervisor consumes it);
    # StreamCore is SM-only, so assert the push landed on the RMI ring...
    assert rig.controller.push_system_fault(event_id=9) is True
    # ...and that the SM-plane in_error that a real system fault raises drives the
    # C++ core to SAFE_FOLLOW autonomously (the reaction StreamCore is responsible
    # for). Reuse inject_alarm to set in_error on the streaming plane.
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    rig.controller.inject_alarm("SRVO-050", "Collision detect alarm", in_error=True)
    assert rig.wait_mode(Mode.SAFE_FOLLOW, 2.0)
    assert rig.fault() == FaultReason.IN_ERROR


# ---------------------------------------------------------------------------
# S-DROP — graduated RX-silence + seq-gap / duplicate taxonomy
# ---------------------------------------------------------------------------


def test_graduated_rx_silence_degrades_then_parks(rig: Rig) -> None:
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    mid = rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    # ~100 ms of status silence (≈13 ITPs): in-flight motion FAULTED(RX_DEGRADED),
    # core kill-holds into SAFE_FOLLOW while TX keeps flowing.
    rig.controller.drop_status(18)
    assert rig.wait_mode(Mode.SAFE_FOLLOW, 1.5), "100 ms silence → SAFE_FOLLOW"
    assert rig.fault() == FaultReason.RX_DEGRADED
    assert rig.wait_status(mid, MotionStatus.FAULTED, 1.5)
    # Sustained silence past 500 ms → RX_SILENT (TX parked).
    rig.controller.drop_status(100)
    assert rig.wait_mode(Mode.RX_SILENT, 2.0), "500 ms silence → RX_SILENT"


def test_status_seq_gap_counted_not_faulted(rig: Rig) -> None:
    gaps0 = rig.core.timing_stats()["rx_seq_gaps"]
    rig.controller.inject_seq_gap(5)  # one status jumps seq by +5
    assert _poll(lambda: rig.core.timing_stats()["rx_seq_gaps"] > gaps0, 1.0), "seq gap counted"
    # A single seq gap is a counter event, not a fault — HOLD is preserved.
    assert rig.mode() == Mode.HOLD


def test_status_duplicate_tolerated(rig: Rig) -> None:
    rig.controller.duplicate_next_status()
    time.sleep(0.1)
    # A byte-identical duplicate must not fault or wedge the core.
    assert rig.mode() == Mode.HOLD
    assert rig.core.get_snapshot()["rx_fresh"] is True


# ---------------------------------------------------------------------------
# S-VERS — v3 + v4 bring-up (version echo + FSConfig gate)
# ---------------------------------------------------------------------------


def test_version_v3_bringup_clean() -> None:
    r = Rig(available_version=3)
    try:
        assert r.ready
        assert r.wait_mode(Mode.HOLD, 3.0)
        # v3: no ForceSensorConfig is sent (no HOST-380 on the controller).
        assert not any("HOST-380" in a for a in r.controller.state.alarm_texts)
    finally:
        r.close()  # asserts zero version-drift / conformance violations


def test_version_v4_bringup_clean() -> None:
    r = Rig(available_version=4, core_sm_version=4)
    try:
        assert r.ready
        assert r.wait_mode(Mode.HOLD, 3.0)
    finally:
        r.close()


# ---------------------------------------------------------------------------
# Capture — settle-at-3° executes; capture-or-reject at 5°; ≥200 ms splice
# ---------------------------------------------------------------------------


def test_capture_within_tolerance_executes_next_move(rig: Rig) -> None:
    # New trajectory starting ~3° (0.052 rad) from the commanded pose → within the
    # 5° capture window → the splice is bridged and the move runs to DONE.
    times, q, qd = _traj(0.15, duration_ns=600_000_000, start0=0.052)
    mid = rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.CAPTURE, 1.0) or rig.wait_mode(Mode.TRAJECTORY, 1.0)
    assert rig.wait_status(mid, MotionStatus.DONE, 3.0)


def test_capture_beyond_tolerance_rejected(rig: Rig) -> None:
    # Trajectory starting > 5° (0.6 rad ≈ 34°) from commanded → REJECTED, no motion.
    times, q, qd = _traj(0.7, duration_ns=500_000_000, start0=0.6)
    mid = rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_status(mid, MotionStatus.REJECTED, 1.5)
    assert rig.mode() == Mode.HOLD, "rejected splice starts no motion"


def test_capture_long_splice_takes_at_least_200ms(rig: Rig) -> None:
    # A ~4.5° (0.078 rad) splice at the 15°/s capture rate takes ≥ ~300 ms; assert
    # the CAPTURE phase actually spans ≥ 200 ms before TRAJECTORY (the anchor-math
    # window; MotionHandle.trajectory_start_mono_ns value itself is a P4b surface).
    times, q, qd = _traj(0.2, duration_ns=600_000_000, start0=0.078)
    mid = rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.CAPTURE, 1.0), "enters CAPTURE for a > tol-rate splice"
    t_capture_start = time.monotonic()
    assert rig.wait_mode(Mode.TRAJECTORY, 2.0), "capture completes into TRAJECTORY"
    capture_dur = time.monotonic() - t_capture_start
    assert capture_dur >= 0.2, f"capture splice spanned only {capture_dur * 1e3:.0f} ms"
    assert rig.wait_status(mid, MotionStatus.DONE, 3.0)


# ---------------------------------------------------------------------------
# Servo — sine tracking + direction reversal (C2); 5° distance guard (C3)
# ---------------------------------------------------------------------------


def test_servo_sine_tracks_and_reverses_without_dwell(rig: Rig) -> None:
    # Drive a sine on J0 via replace-not-queue servo targets at ~50 Hz. Assert the
    # core enters SERVO, the commanded velocity REVERSES sign (a direction flip —
    # the C2 overshoot-reverse case), and never dwells at zero velocity through the
    # sweep (the C2 stutter the minimum_duration fix removes).
    amp = 0.08  # rad (~4.6°, each 20 ms step ≪ 5° servo window)
    period = 0.6
    hz = 50.0
    qd_hist: list[float] = []
    saw_servo = False
    t0 = time.monotonic()
    while (t := time.monotonic() - t0) < period:
        target = amp * math.sin(2.0 * math.pi * t / period)
        rig.core.submit_servo([target, 0, 0, 0, 0, 0], 1.0 / hz)
        if rig.mode() == Mode.SERVO:
            saw_servo = True
            qd_hist.append(rig.core.get_snapshot()["qd_cmd"][0])
        time.sleep(1.0 / hz)
    assert saw_servo, "servo targets drive the core into SERVO mode"
    assert len(qd_hist) > 10
    # Direction reversal: commanded velocity took both signs across the sweep.
    assert max(qd_hist) > 0.01 and min(qd_hist) < -0.01, (max(qd_hist), min(qd_hist))
    # No sustained zero-velocity dwell mid-sweep: not a long run of ~0 velocity.
    zero_run = max_zero_run = 0
    for v in qd_hist:
        zero_run = zero_run + 1 if abs(v) < 5e-4 else 0
        max_zero_run = max(max_zero_run, zero_run)
    assert max_zero_run < len(qd_hist) // 2, f"velocity dwelled at zero for {max_zero_run} samples"
    # Velocity stays within the joint limit.
    assert max(abs(v) for v in qd_hist) < 2.094


def test_servo_distance_guard_rejects_far_target(rig: Rig) -> None:
    # |q_target − q_cmd| > 5° servo window → typed REJECT (imports no ur_rtde jump).
    mid = rig.core.submit_servo([0.9, 0, 0, 0, 0, 0], 0.5)  # ~51° from ~0
    assert rig.wait_status(mid, MotionStatus.REJECTED, 1.5)
    assert rig.mode() in (Mode.HOLD, Mode.SERVO)


# ---------------------------------------------------------------------------
# S-FLAP — dries 9-flap / 7-oscillation anti-flap dwell (R2 F7)
# ---------------------------------------------------------------------------


def test_contact_flap_storm_holds_dwell_no_premature_recovery(rig: Rig) -> None:
    """Replay a rapid contact-stop flap (the dries 9-flap / 7-oscillation class):
    the condition toggles faster than the 500 ms anti-flap dwell, so the core must
    NOT flap back to STREAMING — DEGRADED→STREAMING requires all-clear sustained
    ≥ 500 ms (F7). Assert bounded mode transitions (no per-flap re-recovery)."""
    times, q, qd = _traj(1.0, duration_ns=3_000_000_000)
    rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_mode(Mode.TRAJECTORY, 1.5)
    rig.core.poll_events()  # drain pre-flap events

    # 9 flaps of contact-stop, each shorter than the dwell, with recover() poked
    # each time an outsider might (mirrors the demo keep_trying retry loop).
    for _ in range(9):
        rig.controller.set_contact_stop(2)
        assert rig.wait_mode(Mode.SAFE_FOLLOW, 1.0)
        rig.controller.set_contact_stop(0)
        rig.core.recover()
        time.sleep(0.05)  # 50 ms clear << 500 ms dwell → must NOT recover yet
        assert rig.mode() == Mode.SAFE_FOLLOW, "no recovery before the 500 ms dwell"

    # Count HOLD entries emitted across the storm: the dwell prevented premature
    # recoveries, so we never bounced into HOLD mid-flap.
    hold_entries = sum(
        1
        for e in rig.core.poll_events()
        if e["type"] == int(_EVENT_MODE_CHANGE) and e["value"] == int(Mode.HOLD)
    )
    assert hold_entries == 0, f"flapped into HOLD {hold_entries}× before the dwell"

    # Finally hold clear long enough → single clean recovery.
    rig.controller.set_contact_stop(0)
    rig.core.recover()
    assert rig.wait_mode(Mode.HOLD, 3.0), "recovers once all-clear is sustained past the dwell"


# ---------------------------------------------------------------------------
# S-SOAK — CI-short conformance soak (the R1 A1 fix regression wall)
# ---------------------------------------------------------------------------


def test_soak_short_conformance_and_cadence(rig: Rig) -> None:
    """A few seconds of SUSTAINED 125 Hz streaming (one move, then steady HOLD)
    must hold the wire-conformance + one-TX-per-window invariants: ZERO strict
    violations and ``double_send_guard == 0`` over ~500 TX windows. This is the
    fast CI canary for the FakeCRX min-inter-TX-spacing fix — the fix's exact
    target is the steady stream where two independent ~125 Hz clocks must NOT
    manufacture a phantom double-send, which HOLD exercises continuously. The FULL
    3-min ``-m soak_short`` and the 30-min adversarial soak run on olifant (L3,
    report 09 §2.4 / §8) — not on jittery CI runners.

    NOTE: this soak does a SINGLE trajectory submit for cadence focus. Rapid
    re-submission ORIGINALLY hit a submit_mu_ re-lock deadlock in reap_retired_
    that this soak first surfaced; it is now FIXED by splitting out
    reap_retired_locked_() for the submit paths (realtime_core.cpp). The active
    regression guard is ``test_submit_trajectory_deadlocks_after_buffer_retirement``
    below (it runs and passes — not skipped).
    """
    # One real move to exercise CAPTURE→TRAJECTORY→settle, then hold-stream.
    times, q, qd = _traj(0.3, duration_ns=800_000_000)
    mid = rig.core.submit_trajectory(times, q, qd, 1.0)
    assert rig.wait_status(mid, MotionStatus.DONE, 3.0)

    # Sustained HOLD streaming for the remainder (~3 s ≈ 375 TX windows).
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        assert rig.mode() in (Mode.HOLD, Mode.TRAJECTORY, Mode.CAPTURE), rig.mode()
        assert rig.core.timing_stats()["double_send_guard"] == 0
        time.sleep(0.1)

    ts = rig.core.timing_stats()
    assert ts["double_send_guard"] == 0, "one-TX-per-window held over the soak"
    assert ts["tx_count"] > 300, ts["tx_count"]
    assert rig.violations() == [], "zero strict wire-conformance violations over the soak"
    assert rig.controller.sm.realtime_error is None


@pytest.mark.timeout(30)  # regression guard: if the self-deadlock returns, fail fast (don't hang)
def test_submit_trajectory_deadlocks_after_buffer_retirement() -> None:
    """REGRESSION guard for the submit_mu_/reap_retired_ self-deadlock.

    Repro (deterministic): submit a trajectory, wait DONE, sleep so the RT thread
    supersedes+retires the completed buffer, repeat. Submits #1 and #2 return;
    submit #3 blocks forever inside the C++ ``submit_trajectory`` because
    ``reap_retired_()`` (line ~647) re-locks ``submit_mu_`` already held at line
    ~646. Suggested fix: a ``reap_retired_locked_()`` helper for the submit paths
    (which already hold the lock), or make ``submit_mu_`` recursive. Once fixed,
    all five submits below return and this test passes.
    """
    r = Rig()
    assert r.wait_mode(Mode.HOLD, 3.0)
    cur = 0.0
    for k, tgt in enumerate([0.2, -0.2, 0.3, -0.3, 0.4]):
        mid = r.core.submit_trajectory(
            [0, 500_000_000], [[cur, 0, 0, 0, 0, 0], [tgt, 0, 0, 0, 0, 0]], _TRAJ_QD_REST, 1.0
        )
        assert r.wait_status(mid, MotionStatus.DONE, 3.0), f"submit #{k + 1} did not reach DONE"
        time.sleep(0.4)  # let the RT thread supersede + retire the completed buffer
        cur = r.core.get_snapshot()["q_cmd"][0]
    r.close()


# ---------------------------------------------------------------------------
# Finding 6 (R4(b)) — superseded (mailbox-coalesced) handles resolve PREEMPTED
# ---------------------------------------------------------------------------


def test_rapid_supersede_resolves_superseded_handles_preempted(rig: Rig) -> None:
    """A burst of submits fired faster than the 8 ms RT tick coalesces in the
    mailbox: the drain loop keeps only the LAST target, so only it reaches
    ``TickCore::consume``. Every superseded (coalesced-away) handle MUST resolve
    ``PREEMPTED`` — never hang ``PENDING`` (R4(b)).

    Pre-fix, the drain loop recorded no terminal resolution for the coalesced
    ids, so ``motion_status`` returned ``PENDING`` forever → ``MotionHandle.wait``
    (which polls ``motion_status``) blocked indefinitely / raised ``TimeoutError``.
    Reproduced with 7 back-to-back submits (ids 1-6 stuck PENDING); this fires 12,
    kept under the 16-slot mailbox so none are dropped-at-submit.
    """
    n = 12
    ids: list[int] = []
    for _ in range(n):  # tight burst from the current (~0) HOLD pose; tiny in-window move
        times, q, qd = _traj(0.05)
        ids.append(rig.core.submit_trajectory(times, q, qd, 1.0))
    assert all(mid > 0 for mid in ids), f"burst under the mailbox capacity should all enqueue: {ids}"

    survivor = ids[-1]
    # The one survivor resolves terminally (it is the only target consumed).
    assert _poll(lambda: rig.status(survivor) not in (MotionStatus.PENDING, MotionStatus.RUNNING), 4.0), (
        f"survivor {survivor} never resolved (status={rig.status(survivor).name})"
    )

    # Every handle is now terminal — NONE stuck PENDING (the finding-6 hang).
    statuses = {mid: rig.status(mid) for mid in ids}
    pending = [mid for mid, s in statuses.items() if s == MotionStatus.PENDING]
    assert not pending, (
        f"superseded handles hung PENDING: {pending} (all={[s.name for s in statuses.values()]})"
    )
    # The coalesced-away handles resolved specifically PREEMPTED (proves the
    # drain-loop resolution path fired, not just the consume-time preempt).
    preempted = [mid for mid, s in statuses.items() if s == MotionStatus.PREEMPTED]
    assert len(preempted) >= n // 2, (
        f"expected the burst to coalesce → superseded PREEMPTED; got {[s.name for s in statuses.values()]}"
    )

    # The synthetic kMotionPreempted events were emitted on the event ring too
    # (EventType::kMotionPreempted ordinal = 14), so an event-driven observer
    # sees the resolution, not only the motion_status table.
    _EVENT_MOTION_PREEMPTED = 14
    preempt_events = {
        e["motion_id"]
        for e in rig.core.poll_events()
        if e["type"] == _EVENT_MOTION_PREEMPTED and e["motion_id"] != 0
    }
    assert set(preempted) & preempt_events, (
        f"no kMotionPreempted events for the superseded ids (events for {preempt_events})"
    )
