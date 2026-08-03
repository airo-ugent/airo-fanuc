# SPDX-License-Identifier: Apache-2.0
"""``FanucDriver`` public-API tests against the REAL realtime ``FakeCRXController``.

Reuses the realtime-rig pattern of the FakeCRX integration tests
(``test_integration_scenarios.Rig``): the fake's wall-paced 125 Hz thread + RMI server + the
C++ RT core, driven through :class:`~airo_fanuc.driver.FanucDriver` — whose API keeps
the conventional manipulator-driver shape: a receive side (state getters) split from
a control side (motion commands), each command available blocking or non-blocking.
Construct-and-go bring-up, ``move_trajectory`` → ``MotionHandle.wait``, the
validation table, ``stop_j`` during a blocking wait, the force-guard, the ARM gate,
the CAPTURE hook, honest getters and poison-not-exit ``close()``.

:class:`DriverRig` is shared with ``test_supervisor.py`` (imported from here).
"""

from __future__ import annotations

import math
import socket
import threading
import time
from collections.abc import Iterator
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from conftest import TEST_PROFILE

from airo_fanuc import (
    DriverConfig,
    DriverPolicy,
    FanucConnectionError,
    FanucDriver,
    FanucError,
    MotionResult,
    RejectedStartMismatch,
    RobotFaultedError,
    TrajectoryValidationError,
)
from airo_fanuc import controller_facts as cf
from airo_fanuc.exceptions import RmiSessionDown
from airo_fanuc.ownership import OwnershipLock
from airo_fanuc.testing import FakeCRXConfig, FakeCRXController

_READY_TIMEOUT_S = 6.0
# Short GRIPDISP liveliness-probe ceiling for tests: the FakeCRX starts with no
# dispatcher running, so a gripper-enabled bring-up's probe always times out before
# forking GRPRUN — keep that wait tiny so the suite stays fast (prod default is 6 s).
_GRIPDISP_PROBE_TIMEOUT_S = 0.3


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class DriverRig:
    """Bring up a :class:`FanucDriver` against a realtime FakeCRX; tear both down."""

    def __init__(
        self,
        tmp_path: Any,
        *,
        available_version: int = 3,
        strict: bool = True,
        policy_overrides: dict[str, Any] | None = None,
    ) -> None:
        self.controller = FakeCRXController(FakeCRXConfig(strict=strict, available_version=available_version))
        self.controller.start()
        # The physical controller streams once STREAM_MOTN launches; the fake needs
        # its realtime loop spun up. motion_possible stays False until the driver's
        # bring-up does FRC_Call(STREAM_MOTN) + the core's StartPacket.
        self.controller.start_realtime(speed=1.0)

        cfg = DriverConfig(
            profile=TEST_PROFILE,
            sm_port=self.controller.sm_port,
            rmi_port=self.controller.rmi_port,
            sm_version=available_version,
            preroll_timeout_s=_READY_TIMEOUT_S,
            gripdisp_probe_timeout_s=_GRIPDISP_PROBE_TIMEOUT_S,
        )
        overrides = dict(policy_overrides or {})
        policy = DriverPolicy(
            config=cfg,
            connect_retries=overrides.pop("connect_retries", 1),
            recovery_delay_s=overrides.pop("recovery_delay_s", 0.05),
            # The fake has no STREAM_MOTN handover, so it never shows the re-FRC_Call
            # transient the production 2 s settle exists for — paying it in every test
            # would only slow the suite. Tests that care set it explicitly.
            bringup_settle_s=overrides.pop("bringup_settle_s", 0.05),
            recovery_cooldown_s=overrides.pop("recovery_cooldown_s", 0.2),
            watch_interval_s=overrides.pop("watch_interval_s", 0.01),
            lock_path=overrides.pop("lock_path", str(tmp_path / "owner.lock")),
            **overrides,
        )
        self.driver = FanucDriver("127.0.0.1", policy)

    def enable_force_path(self) -> None:
        """Poke the fake so its type-204 carries a VALID wrench (fs_type EMBEDDED).

        StreamCore never sends an FSConfig, so the fake reports fs_type
        Unavailable and the force-guard is inert. This test-only poke flips the
        fake's ``_fsconfig_received`` so wrench_valid asserts in the core (the
        real controller does this after the v4 FSConfig handshake)."""
        self.controller.sm._fsconfig_received = True  # noqa: SLF001 - test-only

    def close(self) -> None:
        viol = list(self.controller.violations)
        try:
            self.driver.close()
        finally:
            self.controller.stop_realtime()
            self.controller.close()
        assert viol == [], f"strict wire-conformance violations: {viol}"


@pytest.fixture
def rig(tmp_path: Any) -> Iterator[DriverRig]:
    r = DriverRig(tmp_path)
    try:
        yield r
    finally:
        r.close()


def _cur_q0(driver: FanucDriver) -> float:
    return float(driver.get_state()["q_cmd"][0])


def _traj_from(driver: FanucDriver, delta: float, duration_ns: int = 1_000_000_000) -> tuple:
    start = _cur_q0(driver)
    times = [0, duration_ns]
    q = [[start, 0, 0, 0, 0, 0], [start + delta, 0, 0, 0, 0, 0]]
    qd = [[0.0] * 6, [0.0] * 6]
    return times, q, qd


# --------------------------------------------------------------------------- #
# Construct-and-go
# --------------------------------------------------------------------------- #


def test_construct_and_go_reaches_commandable(rig: DriverRig) -> None:
    st = rig.driver.get_state()
    assert st["lifecycle_state"] == "streaming"
    assert st["motion_possible"] is True
    assert rig.driver.wait_until_steady(2.0)
    assert rig.driver.is_steady()
    # Reach-through attrs: the driver owns its collaborators but exposes them rather
    # than wrapping every call, so a caller can drive RMI / the core / the gripper
    # directly without a second connection to the single-session controller.
    assert rig.driver.rmi is not None and rig.driver.core is not None and rig.driver.gripper is not None


def test_unreachable_controller_raises_connection_error(tmp_path: Any) -> None:
    dead = _free_port()  # nothing listening here
    cfg = DriverConfig(profile=TEST_PROFILE, sm_port=_free_port(), rmi_port=dead, rmi_connect_timeout=1.0)
    policy = DriverPolicy(config=cfg, connect_retries=1, lock_path=str(tmp_path / "owner.lock"))
    with pytest.raises(FanucConnectionError):
        FanucDriver("127.0.0.1", policy)


# --------------------------------------------------------------------------- #
# move_trajectory → MotionHandle
# --------------------------------------------------------------------------- #


def test_move_trajectory_runs_to_done_and_plant_tracks(rig: DriverRig) -> None:
    times, q, qd = _traj_from(rig.driver, 0.4)
    handle = rig.driver.move_trajectory(times, q, qd)
    assert handle.wait(timeout=4.0) == MotionResult.DONE
    assert abs(rig.driver.get_state()["q_meas"][0] - 0.4) < 0.02
    # Post-capture anchor was observed (host-side) at the CAPTURE→TRAJECTORY edge.
    assert handle.trajectory_start_mono_ns is not None
    assert handle.trajectory_start_mono_ns >= handle.submit_mono_ns


def test_move_trajectory_asynchronous_returns_immediately(rig: DriverRig) -> None:
    times, q, qd = _traj_from(rig.driver, 0.3, duration_ns=1_500_000_000)
    t0 = time.monotonic()
    handle = rig.driver.move_trajectory(times, q, qd, asynchronous=True)
    assert (time.monotonic() - t0) < 0.2, "asynchronous submit must return promptly"
    assert handle.wait(timeout=4.0) == MotionResult.DONE


# --------------------------------------------------------------------------- #
# Validation table (each violation → its typed error)
# --------------------------------------------------------------------------- #


def test_validation_table(rig: DriverRig) -> None:
    d = rig.driver
    good_t = [0, 1_000_000_000]
    good_q = [[0, 0, 0, 0, 0, 0], [0.2, 0, 0, 0, 0, 0]]
    good_qd = [[0.0] * 6, [0.0] * 6]

    # <2 knots
    with pytest.raises(TrajectoryValidationError):
        d.move_trajectory([0], [[0, 0, 0, 0, 0, 0]], [[0.0] * 6])
    # times not strictly increasing
    with pytest.raises(TrajectoryValidationError):
        d.move_trajectory([0, 0], good_q, good_qd)
    with pytest.raises(TrajectoryValidationError):
        d.move_trajectory([1_000_000_000, 0], good_q, good_qd)
    # non-finite q
    bad_q = [[0, 0, 0, 0, 0, 0], [float("nan"), 0, 0, 0, 0, 0]]
    with pytest.raises(TrajectoryValidationError):
        d.move_trajectory(good_t, bad_q, good_qd)
    # wrong DOF
    with pytest.raises(TrajectoryValidationError):
        d.move_trajectory(good_t, [[0, 0, 0], [0.2, 0, 0]], [[0.0] * 3, [0.0] * 3])
    # |qd| > v_lim (TEST_PROFILE J0 is 120°/s = 2.094 rad/s; qd=3.0 exceeds it)
    fast_qd = [[3.0, 0, 0, 0, 0, 0], [3.0, 0, 0, 0, 0, 0]]
    with pytest.raises(TrajectoryValidationError):
        d.move_trajectory(good_t, good_q, fast_qd)
    # There is no speed_scale parameter to abuse — the trajectory's times/qd are the
    # speed. Passing one is a TypeError, not a value to validate.
    with pytest.raises(TypeError):
        d.move_trajectory(good_t, good_q, good_qd, speed_scale=0.5)  # type: ignore[call-arg]


def test_first_knot_velocity_beyond_the_capture_envelope_is_a_typed_error(rig: DriverRig) -> None:
    """A trajectory the capture splice cannot start must say so, not return REJECTED.

    The splice bridging the commanded pose to knot 0 is bounded by
    CAPTURE_RATE_DEG_S, and that bound is on its endpoint velocities — a first knot
    above it is unreachable no matter what the arm is currently doing (measured
    2026-07-30: accepted at 15.0°/s, refused at 15.1). Left to the core it surfaces
    as MotionStatus.REJECTED carrying FaultReason.INTERNAL, which reads like a driver
    bug rather than a trajectory that cannot be started.
    """
    d = rig.driver
    over = math.radians(cf.CAPTURE_RATE_DEG_S + 0.5)
    inside = math.radians(cf.CAPTURE_RATE_DEG_S - 0.5)

    def submit(qd0: float) -> MotionResult:
        # Rebuilt from the CURRENT commanded pose every time: a stale first knot would
        # trip the 5° position guard instead, which is a different rejection.
        times, q, _ = _traj_from(d, 0.1)
        return d.move_trajectory(times, q, [[qd0] + [0.0] * 5, [0.0] * 6]).wait(timeout=4.0)

    times, q, _ = _traj_from(d, 0.1)
    with pytest.raises(TrajectoryValidationError, match="capture envelope"):
        d.move_trajectory(times, q, [[over] + [0.0] * 5, [0.0] * 6])

    # Just inside the envelope is accepted, and actually runs — so the ceiling is the
    # capture rate itself and not some smaller fudge.
    assert submit(inside) == MotionResult.DONE


# --------------------------------------------------------------------------- #
# stop_j during a blocking wait() → STOPPED
# --------------------------------------------------------------------------- #


def test_stop_j_during_wait_resolves_stopped(rig: DriverRig) -> None:
    import threading

    times, q, qd = _traj_from(rig.driver, 1.0, duration_ns=3_000_000_000)
    handle = rig.driver.move_trajectory(times, q, qd)
    # Let it build real velocity, then stop_j from another thread while wait() blocks.
    time.sleep(0.3)
    threading.Timer(0.05, rig.driver.stop_j).start()
    assert handle.wait(timeout=4.0) == MotionResult.STOPPED
    assert rig.driver.wait_until_steady(3.0)


def test_stop_j_never_raises_when_idle(rig: DriverRig) -> None:
    rig.driver.stop_j()  # HOLD → no-op, must not raise
    rig.driver.stop_j()


def test_force_stop_trips_to_stopped(tmp_path: Any) -> None:
    # Force telemetry requires a v4 / type-204 controller (the default v3 fake
    # streams type-202 with NO force block, matching the measured CRX behaviour — see
    # docs/controller-notes.md §1.8). enable_force_path() pokes the fake's FSConfig so
    # its type-204 carries a valid wrench. strict=False: the C++ command encoder pins
    # version_no=3 (wiring it to sm_version is deferred until v4 is actually reachable
    # on the controller, codec.hpp), so a strict v4 fake would flag a benign
    # command-version mismatch; that is orthogonal to the version-agnostic
    # force-guard under test here.
    rig = DriverRig(tmp_path, available_version=4, strict=False)
    try:
        rig.enable_force_path()
        # Wait for the FSConfig poke to reach the wire (wrench_valid live) so the guard
        # ARMS rather than being (correctly) rejected as "no force telemetry" — a real
        # v4 controller has force live at steady state; the fake takes a few ticks.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and rig.driver.get_wrench() is None:
            time.sleep(0.01)
        assert rig.driver.get_wrench() is not None, "force telemetry did not come live"
        times, q, qd = _traj_from(rig.driver, 1.0, duration_ns=3_000_000_000)
        handle = rig.driver.move_trajectory(times, q, qd, force_stop_n=20.0)
        # Wait until it is actually executing, then spike the tool-frame force past 20 N.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and abs(rig.driver.get_state()["qd_cmd"][0]) < 0.05:
            time.sleep(0.004)
        rig.controller.set_force(0.0, 0.0, 60.0)
        assert handle.wait(timeout=4.0) == MotionResult.STOPPED
    finally:
        rig.close()


def test_force_stop_rejected_without_force_telemetry(rig: DriverRig) -> None:
    # On the real controller (v3 / type-202) there is no force telemetry, so arming
    # a force guard is a silent no-op — the driver must REJECT it loudly instead.
    times, q, qd = _traj_from(rig.driver, 0.3)
    with pytest.raises(TrajectoryValidationError):
        rig.driver.move_trajectory(times, q, qd, force_stop_n=20.0)
    # The same move WITHOUT force_stop_n is accepted (contact-stop is the real path).
    assert rig.driver.move_trajectory(times, q, qd).wait(timeout=4.0) == MotionResult.DONE


# --------------------------------------------------------------------------- #
# CAPTURE collision-check hook: the driver synthesizes the splice from the currently
# commanded pose to the caller's first knot, so those knots are geometry the caller
# never planned. The hook is its veto over that synthesized segment.
# --------------------------------------------------------------------------- #


def test_capture_beyond_window_raises_rejected_start_mismatch(rig: DriverRig) -> None:
    # First knot ~34° from the commanded pose (> 5° window) → synchronous reject.
    times = [0, 500_000_000]
    q = [[0.6, 0, 0, 0, 0, 0], [0.7, 0, 0, 0, 0, 0]]
    qd = [[0.0] * 6, [0.0] * 6]
    with pytest.raises(RejectedStartMismatch):
        rig.driver.move_trajectory(times, q, qd)


def test_capture_check_hook_rejects_colliding_splice(tmp_path: Any) -> None:
    seen: dict[str, Any] = {}

    def checker(knots_q: np.ndarray, knots_qd: np.ndarray) -> bool:
        seen["q"] = knots_q
        return False  # pretend the splice collides

    rig = DriverRig(tmp_path, policy_overrides={"capture_check": checker})
    try:
        times, q, qd = _traj_from(rig.driver, 0.05)  # within the capture window
        with pytest.raises(TrajectoryValidationError):
            rig.driver.move_trajectory(times, q, qd)
        assert "q" in seen and seen["q"].shape[1] == 6, "the hook received the synthesized knots"
    finally:
        rig.close()


def test_capture_check_hook_allows_safe_splice(tmp_path: Any) -> None:
    rig = DriverRig(tmp_path, policy_overrides={"capture_check": lambda q, qd: True})
    try:
        times, q, qd = _traj_from(rig.driver, 0.05)
        handle = rig.driver.move_trajectory(times, q, qd)
        assert handle.wait(timeout=4.0) == MotionResult.DONE
    finally:
        rig.close()


# --------------------------------------------------------------------------- #
# servo_j
# --------------------------------------------------------------------------- #


def test_servo_j_executes(rig: DriverRig) -> None:
    start = _cur_q0(rig.driver)
    handle = rig.driver.servo_j([start + 0.04, 0, 0, 0, 0, 0], 0.1)
    # Servo has no natural terminal; just assert it did not reject and drives SERVO.
    time.sleep(0.1)
    assert handle.result() != MotionResult.REJECTED


def test_servo_j_far_target_is_tracked_not_rejected(rig: DriverRig) -> None:
    # No distance guard: a far target is chased under the servo limits rather than
    # refused, so a stream that falls behind keeps going instead of stopping.
    start = _cur_q0(rig.driver)
    handle = rig.driver.servo_j([start + 0.9, 0, 0, 0, 0, 0], 0.5)  # ~51° away
    time.sleep(0.3)
    assert handle.result() != MotionResult.REJECTED
    assert _cur_q0(rig.driver) > start + 0.01, "moved toward the far target"


def test_servo_j_bad_shape_raises(rig: DriverRig) -> None:
    with pytest.raises(TrajectoryValidationError):
        rig.driver.servo_j([0.0, 0.0], 0.1)
    with pytest.raises(TrajectoryValidationError):
        rig.driver.servo_j([0.0] * 6, 0.0)


def test_servo_j_feedforward_executes(rig: DriverRig) -> None:
    # qd/qdd are accepted and ignored; the call must still behave exactly as the
    # position-only one rather than erroring on arguments it no longer uses.
    start = _cur_q0(rig.driver)
    handle = rig.driver.servo_j(
        [start + 0.04, 0, 0, 0, 0, 0],
        0.1,
        qd=[0.2, 0, 0, 0, 0, 0],
        qdd=[0.0] * 6,
    )
    time.sleep(0.1)
    assert handle.result() != MotionResult.REJECTED


def test_servo_j_feedforward_far_target_is_tracked_not_rejected(rig: DriverRig) -> None:
    # Same best-effort behaviour on the qd-carrying overload as on the plain one.
    start = _cur_q0(rig.driver)
    handle = rig.driver.servo_j(
        [start + 0.9, 0, 0, 0, 0, 0],
        0.5,
        qd=[0.2, 0, 0, 0, 0, 0],
    )
    time.sleep(0.3)
    assert handle.result() != MotionResult.REJECTED
    assert _cur_q0(rig.driver) > start + 0.01, "moved toward the far target"


def test_servo_j_feedforward_bad_shape_raises(rig: DriverRig) -> None:
    with pytest.raises(TrajectoryValidationError):
        rig.driver.servo_j([0.0] * 6, 0.1, qd=[0.0, 0.0])  # qd wrong length
    with pytest.raises(TrajectoryValidationError):
        rig.driver.servo_j([0.0] * 6, 0.1, qd=[0.0] * 6, qdd=[0.0, 0.0])  # qdd wrong length


# --------------------------------------------------------------------------- #
# Getters — never raise, never lie
# --------------------------------------------------------------------------- #


def test_get_state_merges_snapshot_and_lifecycle(rig: DriverRig) -> None:
    st = rig.driver.get_state()
    for key in ("mode", "q_meas", "qd_est", "q_cmd", "lifecycle_state", "fault_reason", "epoch", "owner"):
        assert key in st, key
    assert st["fault_reason"] == "none"
    assert st["owner"]["mode"] == "control"


def test_get_wrench_none_on_v3_controller(rig: DriverRig) -> None:
    # The target CRX controller (v3 / type-202) has NO force block on the wire, so
    # get_wrench() is always None (fs_type Unavailable). See
    # docs/controller-notes.md §1.8.
    assert rig.driver.get_wrench() is None


def test_get_wrench_gated_on_fs_type(tmp_path: Any) -> None:
    # v4 / type-204 controller: before the FSConfig handshake fs_type is Unavailable
    # (wrench None); after enable_force_path() it becomes EMBEDDED and the wrench flows.
    # strict=False — see test_force_stop_trips_to_stopped (pinned command version_no=3
    # vs a v4 fake is the deferred v4 wiring, orthogonal to get_wrench gating).
    rig = DriverRig(tmp_path, available_version=4, strict=False)
    try:
        assert rig.driver.get_wrench() is None  # pre-FSConfig: fs_type Unavailable
        rig.enable_force_path()
        rig.controller.set_force(1.0, 2.0, 3.0)
        # Poll (not a fixed sleep): under full-suite CPU load the realtime FakeCRX can
        # take a few 8 ms ticks to stream the new force into the seqlock snapshot.
        w = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            w = rig.driver.get_wrench()
            if w is not None and abs(w[2] - 3.0) < 0.5:
                break
            time.sleep(0.01)
        assert w is not None and w.shape == (6,)
        assert w[2] == pytest.approx(3.0, abs=0.5)
    finally:
        rig.close()


def test_get_flange_pose_carries_the_status_packets_pose_block(rig: DriverRig) -> None:
    # End-to-end passthrough for the Cartesian pose: the fake's injected value →
    # big-endian float32 on the SM status wire → C++ decode → seqlock snapshot →
    # getter. Unlike the wrench this rides the shared header, so it IS present on
    # this v3 / type-202 controller (docs/controller-notes.md §1.8).
    #
    # The fake does no FK, so the value is injected rather than derived from the
    # plant's joints — this proves the pose survives the pipe, NOT that the
    # controller's pose agrees with any frame (that is examples/verify_tcp_frame.py).
    pose_mm_deg = [1234.5, -678.25, 901.75, -179.5, 45.25, 90.125]
    rig.controller.set_cartesian(pose_mm_deg)

    # Poll: under full-suite load the realtime FakeCRX needs a few 8 ms ticks to
    # stream the new pose into the seqlock snapshot.
    got = None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        got = rig.driver.get_flange_pose()
        if got is not None and abs(got[0] - pose_mm_deg[0]) < 0.01:
            break
        time.sleep(0.01)

    assert got is not None and got.shape == (6,), got
    # float32 on the wire (the joints are float32 too); these values are chosen to
    # be exactly representable, so they must survive bit-for-bit.
    assert list(got) == pytest.approx(pose_mm_deg, abs=0.0)
    # The raw length-9 block stays reachable, ext axes zero-padded by the fake.
    assert rig.driver.get_state()["cart"][6:] == [0.0, 0.0, 0.0]


def test_get_flange_pose_is_none_before_the_first_status_packet() -> None:
    # A getter must not present the zero-initialised snapshot field as a pose:
    # all-zero XYZWPR is not a pose this arm can hold (flange at the world origin,
    # inside the base). No core at all is the same story — None, never zeros.
    driver = SimpleNamespace(core=None)
    assert FanucDriver.get_flange_pose(driver) is None  # type: ignore[arg-type]

    never_rx = SimpleNamespace(
        core=SimpleNamespace(get_snapshot=lambda: {"rx_mono_ns": 0, "cart": [0.0] * 9})
    )
    assert FanucDriver.get_flange_pose(never_rx) is None  # type: ignore[arg-type]

    # One RX is all it takes to make the same zeros meaningful-or-not the
    # controller's problem rather than the getter's.
    after_rx = SimpleNamespace(
        core=SimpleNamespace(get_snapshot=lambda: {"rx_mono_ns": 1, "cart": [1.0] * 9})
    )
    pose = FanucDriver.get_flange_pose(after_rx)  # type: ignore[arg-type]
    assert pose is not None and pose.shape == (6,)


def test_get_tcp_pose_returns_the_controllers_tool_tip_not_the_faceplate(rig: DriverRig) -> None:
    # The two getters read two different planes, and must not be confused for each
    # other: get_tcp_pose() asks the controller (FRC_ReadCartesianPosition, which
    # applies the active UTOOL), get_flange_pose() reads the streamed faceplate. On the
    # real controller those are 175 mm apart — this cell's Robotiq gripper,
    # docs/controller-notes.md §1.10 — so the fake serves two distinct poses and a
    # regression that quietly answered both from one plane fails here.
    #
    # The geometric relationship between them is a HARDWARE fact and is verified where
    # it can be: examples/verify_tcp_frame.py, against the real controller. Asserting it
    # here would only be re-checking arithmetic the fake was told to produce.
    flange = [20.166, 703.511, 504.091, -115.210, -26.319, 35.132]  # measured faceplate
    tool_tip = [-43.921, 852.017, 437.279, -115.210, -26.319, 35.132]  # measured TCP
    rig.controller.set_cartesian(flange)
    rig.controller.set_cartesian_tcp(tool_tip)

    streamed = None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        streamed = rig.driver.get_flange_pose()
        if streamed is not None and abs(streamed[0] - flange[0]) < 0.01:
            break
        time.sleep(0.01)

    assert streamed is not None
    assert list(streamed) == pytest.approx(flange, abs=1e-3)  # the SM plane

    tcp = rig.driver.get_tcp_pose()
    assert tcp is not None and tcp.shape == (6,)
    assert list(tcp) == pytest.approx(tool_tip, abs=1e-3)  # the RMI plane

    # And they are genuinely different points, not the same value twice.
    assert float(np.linalg.norm(tcp[:3] - streamed[:3])) > 1.0


def test_get_tcp_pose_returns_none_rather_than_substituting_the_faceplate() -> None:
    # No RMI session, and an RMI read that raises, both yield None. The one thing this
    # must never do is fall back to the streamed faceplate: that would report a point
    # 175 mm from the tool tip under a method named get_tcp_pose.
    assert FanucDriver.get_tcp_pose(SimpleNamespace(rmi=None)) is None  # type: ignore[arg-type]

    def boom() -> Any:
        raise RmiSessionDown("session is down")

    broken = SimpleNamespace(
        rmi=SimpleNamespace(read_cartesian_position=boom),
        # A perfectly good faceplate pose is available and must NOT be used.
        core=SimpleNamespace(get_snapshot=lambda: {"rx_mono_ns": 1, "cart": [1.0] * 9}),
    )
    assert FanucDriver.get_tcp_pose(broken) is None  # type: ignore[arg-type]


def test_timing_stats_and_joints_at_wall(rig: DriverRig) -> None:
    ts = rig.driver.timing_stats()
    assert "tx_count" in ts and ts["double_send_guard"] == 0
    now_ns = time.time_ns()
    # A wall stamp far in the future/past returns None or the nearest ring entry;
    # a current stamp should resolve once the ring has samples.
    time.sleep(0.1)
    res = rig.driver.joints_at_wall(now_ns)
    assert res is None or res.shape == (6,)


# --------------------------------------------------------------------------- #
# close() — poison-not-exit
# --------------------------------------------------------------------------- #


def test_close_is_clean_and_idempotent(tmp_path: Any) -> None:
    rig = DriverRig(tmp_path)
    rig.driver.close()
    rig.driver.close()  # idempotent
    rig.controller.stop_realtime()
    rig.controller.close()
    # Motion after close raises (driver closed) — never a segfault / exit.
    with pytest.raises(FanucError):
        rig.driver.move_trajectory([0, 1_000_000_000], [[0] * 6, [0.1] + [0] * 5], [[0.0] * 6, [0.0] * 6])


def test_bringup_settles_a_motion_possible_drop_without_the_recovery_ladder(tmp_path: Any) -> None:
    """A ``motion_possible`` drop just after the preroll must be absorbed BY bring-up.

    On the real controller the required re-``FRC_Call(STREAM_MOTN)`` drops
    ``motion_possible`` about a second after the preroll reports ready, so an
    assert-once bring-up hands back a robot that faults immediately afterwards
    (docs/controller-notes.md §4.2, where the hardware evidence for the fix lives — the
    fake has no STREAM_MOTN handover and cannot reproduce the transient on its own).

    What this pins is our *logic*, against an input we control rather than a modelled
    behaviour: the status bit is forced off during the settle window, so bring-up must
    re-apply the relaunch and still arrive at STREAMING with ``recovery_count == 0`` —
    the ladder having run instead would mean startup depends on ``auto_recover``.
    """
    controller = FakeCRXController(FakeCRXConfig(strict=True))
    controller.start()
    controller.start_realtime(speed=1.0)
    # Force the bit off the moment the core is streaming, and restore it shortly after —
    # the shape of the real transient. A thread, because bring-up is synchronous.
    dropped = threading.Event()

    def _blip() -> None:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not controller.motion_possible:
            time.sleep(0.005)
        controller.drop_motion_possible()
        dropped.set()
        time.sleep(0.3)
        controller.restore_motion_possible()

    threading.Thread(target=_blip, daemon=True).start()
    cfg = DriverConfig(
        profile=TEST_PROFILE,
        sm_port=controller.sm_port,
        rmi_port=controller.rmi_port,
        preroll_timeout_s=_READY_TIMEOUT_S,
        gripdisp_probe_timeout_s=_GRIPDISP_PROBE_TIMEOUT_S,
    )
    policy = DriverPolicy(
        config=cfg,
        connect_retries=1,
        enable_gripper=False,
        auto_recover=False,  # nothing may paper over the drop but the settle itself
        bringup_settle_s=0.5,
        lock_path=str(tmp_path / "owner.lock"),
    )
    driver = FanucDriver("127.0.0.1", policy)
    try:
        # Guard against the test passing vacuously: if the blip never fired (a typo in the
        # fake's surface is enough), bring-up had nothing to settle and proves nothing.
        assert dropped.is_set(), "motion_possible was never dropped — the test exercised nothing"
        st = driver.get_state()
        assert st["lifecycle_state"] == "streaming", st
        assert st["fault_reason"] == "none", st
        assert int(st["recovery_count"]) == 0, "the recovery ladder ran; bring-up should have settled"
    finally:
        driver.close()
        controller.stop_realtime()
        controller.close()


def test_close_is_clean_while_an_estop_is_latched(tmp_path: Any) -> None:
    """An E-stop mid-trajectory must not turn shutdown into a wedge.

    The E-stop is the one fault guaranteed to arrive while a trajectory is running,
    and it arrives with auto-recovery armed — so ``close()`` runs against a latched
    controller and races a recovery ladder that is itself doing RMI I/O. It must
    still tear down in order and without raising, put nothing invalid on the wire
    (``DriverRig.close`` asserts the fake saw no strict-conformance violation), and
    release the flock: an abandoned lock would keep the next process off the
    single-session controller with nothing left alive to release it.
    """
    lock_path = tmp_path / "owner.lock"
    rig = DriverRig(tmp_path, policy_overrides={"auto_recover": True})
    driver = rig.driver
    handle = driver.move_trajectory(*_traj_from(driver, 0.2, 2_000_000_000))
    # Let the trajectory actually start moving before the E-stop lands.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and driver.get_state()["active_motion_id"] != handle.motion_id:
        time.sleep(0.01)

    rig.controller.press_estop()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and driver.get_state()["lifecycle_state"] != "faulted":
        time.sleep(0.01)
    assert driver.get_state()["fault_reason"] == "e_stop"

    t0 = time.monotonic()
    rig.close()  # driver.close() must not raise; then the fake is torn down
    assert time.monotonic() - t0 < 12.0, "close() outran its own timed joins"
    # The motion is a FAULTED outcome, not an exception — the airo convention holds
    # even for the fault that arrives from outside the process.
    assert handle.result() == MotionResult.FAULTED
    # PROOF the flock is free: a fresh acquirer takes it without contention.
    OwnershipLock("control", path=lock_path).acquire().release()


def test_close_poison_does_not_exit_on_wedged_thread(tmp_path: Any) -> None:
    """A wedged teardown step is abandoned + surfaced as a typed error — the
    library NEVER calls os._exit, so the process survives (this test keeps running
    after ``close()`` raises)."""
    rig = DriverRig(tmp_path)

    def _hang() -> None:
        time.sleep(30.0)  # simulate a wedged RT-thread join

    # StreamCore.stop is a read-only pybind method; wedge the driver's Python
    # wrapper around it instead (same _run_with_timeout abandon path).
    rig.driver._safe_core_stop = _hang  # type: ignore[method-assign]  # noqa: SLF001
    with pytest.raises(FanucError) as ei:
        rig.driver.close()
    assert "did not join" in str(ei.value) or "abandoned" in str(ei.value)
    # PROOF the process survived the poison close(): we are still executing.
    survived = 2 + 2
    assert survived == 4
    rig.controller.stop_realtime()
    rig.controller.close()


def test_context_manager(tmp_path: Any) -> None:
    controller = FakeCRXController(FakeCRXConfig(strict=True))
    controller.start()
    controller.start_realtime(speed=1.0)
    cfg = DriverConfig(
        profile=TEST_PROFILE,
        sm_port=controller.sm_port,
        rmi_port=controller.rmi_port,
        preroll_timeout_s=_READY_TIMEOUT_S,
        gripdisp_probe_timeout_s=_GRIPDISP_PROBE_TIMEOUT_S,
    )
    policy = DriverPolicy(config=cfg, connect_retries=1, lock_path=str(tmp_path / "owner.lock"))
    try:
        with FanucDriver("127.0.0.1", policy) as d:
            assert d.get_state()["lifecycle_state"] == "streaming"
    finally:
        controller.stop_realtime()
        controller.close()


# --------------------------------------------------------------------------- #
# ARM gate end-to-end — the full e-stop → recover → arm cycle.
# --------------------------------------------------------------------------- #


def test_arm_gate_end_to_end(rig: DriverRig) -> None:
    d = rig.driver
    # A normal move works before any fault.
    times, q, qd = _traj_from(d, 0.2)
    assert d.move_trajectory(times, q, qd).wait(timeout=4.0) == MotionResult.DONE

    # E-stop → FAULTED; release + explicit recover() ends in MOTION_INHIBITED.
    rig.controller.press_estop()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and d.get_state()["fault_reason"] == "none":
        time.sleep(0.01)
    assert d.get_state()["fault_reason"] in ("e_stop", "in_error")

    rig.controller.release_estop()
    # Wait for the cleared e-stop bit to propagate to the core snapshot before
    # recover() (the operator releases, then triggers recovery — there is inherent
    # delay; recover() bails on a still-held e-stop).
    rel_deadline = time.monotonic() + 2.0
    while time.monotonic() < rel_deadline and d.get_state()["e_stopped"]:
        time.sleep(0.01)
    assert d.recover(timeout_s=5.0) is True
    assert d.get_state()["motion_inhibited"] is True

    # Motion before arm() → typed reject.
    times, q, qd = _traj_from(d, 0.1)
    with pytest.raises(RobotFaultedError):
        d.move_trajectory(times, q, qd)

    # After arm() → executes.
    d.arm()
    assert d.get_state()["motion_inhibited"] is False
    times, q, qd = _traj_from(d, 0.1)
    assert d.move_trajectory(times, q, qd).wait(timeout=4.0) == MotionResult.DONE


# --------------------------------------------------------------------------- #
# reconnect() — cold re-bring-up returns to commandable.
# --------------------------------------------------------------------------- #


def test_reconnect_returns_to_commandable(tmp_path: Any) -> None:
    """reconnect() must clear the supervisor stop-signal before the cold re-bring-up.

    The trap: ``shutdown()`` sets ``_stop_evt`` and only ``start_watch()`` clears it —
    but reconnect runs ``bringup()`` (which short-circuits its ``_wait_mode(HOLD)`` on
    a set event) BEFORE ``start_watch()``, so without an explicit clear every retry
    raises ``FanucConnectionError('core did not publish HOLD after preroll')``.
    reconnect() must succeed and return the driver to a commandable/steady state.

    Runs under a STRICT FakeCRX via :class:`DriverRig` (its ``close()`` asserts zero
    wire-conformance violations), matching the other driver tests. The reconnect
    re-handshake's PLL phase re-lock compresses the first CommandPacket interval of
    the restarted stream; the FakeCRX SM server checks that single first interval
    against a tight same-instant floor, so the transient is not mistaken for a
    double-send while a genuine double-send is still caught. connect_retries=3: the
    cold re-bring-up's Connect_STMO can briefly race the single-session teardown of
    the just-dropped session (2556954), which is a transport race rather than the
    stop-signal bug under test."""
    # connect_retries=3 (see above); DriverRig defaults to a strict FakeCRX.
    rig = DriverRig(tmp_path, policy_overrides={"connect_retries": 3})
    d = rig.driver
    try:
        assert d.get_state()["lifecycle_state"] == "streaming"

        d.reconnect()  # must NOT raise FanucConnectionError

        assert d.get_state()["lifecycle_state"] == "streaming"
        assert d.wait_until_steady(3.0)
        # Commandable again: a normal move runs to completion after the cold re-bring-up.
        times, q, qd = _traj_from(d, 0.1)
        assert d.move_trajectory(times, q, qd).wait(timeout=4.0) == MotionResult.DONE
    finally:
        rig.close()  # asserts strict wire-conformance: violations == []


def test_gripper_registered_with_supervisor(rig: DriverRig) -> None:
    """The driver registers its gripper with the supervisor so the recovery ladder
    can fail-fast-gate it."""
    assert rig.driver.gripper is not None
    assert rig.driver._supervisor._gripper is rig.driver.gripper  # noqa: SLF001


# --------------------------------------------------------------------------- #
# Interpolation period: the controller states its own in the GetCapability reply,
# and a driver configured for a different one applies per-tick limits scaled by the
# wrong ratio, so bring-up must refuse.
# --------------------------------------------------------------------------- #


def test_core_reports_the_controllers_interpolation_period(rig: DriverRig) -> None:
    """The period reported by the controller reaches Python through a real handshake."""
    assert rig.driver.core is not None
    assert rig.driver.core.sm_sampling_rate_ms == 8
    # The negotiated version comes from the same reply; the fake serves v3.
    assert rig.driver.core.sm_negotiated_version == 3


def _itp_check(reported_ms: int, configured_itp_s: float) -> None:
    """Drive FanucDriver._verify_controller_itp against a stub core.

    The method reads only the reported rate and the configured period, so a stub is
    enough and keeps the case controller-free.
    """
    stub = SimpleNamespace(
        core=SimpleNamespace(sm_sampling_rate_ms=reported_ms),
        _cfg=DriverConfig(profile=TEST_PROFILE, itp_s=configured_itp_s),
    )
    FanucDriver._verify_controller_itp(stub)  # type: ignore[arg-type]  # noqa: SLF001


def test_matching_interpolation_period_is_accepted() -> None:
    _itp_check(8, 0.008)


def test_unreported_interpolation_period_is_tolerated() -> None:
    """0 means no capability reply was seen. That failure surfaces on the preroll
    timeout, which reports every cause, so it must not be reported here as a
    period mismatch."""
    _itp_check(0, 0.008)


def test_mismatched_interpolation_period_refuses_bringup() -> None:
    with pytest.raises(FanucConnectionError) as exc:
        _itp_check(8, 0.004)
    msg = str(exc.value)
    assert "8 ms" in msg and "itp_s" in msg
    # The message must name the value that would fix it.
    assert "0.008" in msg
