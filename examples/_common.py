# SPDX-License-Identifier: Apache-2.0
"""Shared plumbing for the validation examples in this directory.

The examples target one specific robot — a FANUC CRX-10iA/L on an R-30iB-class
controller, Stream Motion v3 at an 8 ms interpolation period — and assume rather than
detect it. ``examples/README.md`` lists what that assumption covers and what to change
for a different arm.

The examples answer one question: *does this driver, built on this host, actually
drive my controller?* They all do it the same way — connect, print what the
controller said during bring-up, move a little under watch, then print whether the
125 Hz loop held its deadline — so that scaffolding lives here once instead of
being copied into each script. Each example keeps inline the part worth reading as
a usage pattern: how its trajectory is built and how the motion is commanded.

This is example support code, not part of the package's public API.
"""

from __future__ import annotations

import argparse
import logging
import tempfile
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any, NamedTuple

import numpy as np

from airo_fanuc import DriverConfig, DriverPolicy, FanucError, MotionResult
from airo_fanuc import controller_facts as cf

#: Joint count. Six is baked into the C++ core (the online trajectory generator's
#: DOF is a compile-time template parameter), so this is a constant, not a knob.
NDOF = 6

#: CRX-10iA/L active joint position limits (deg), measured on OUR controller and
#: recorded in docs/controller-notes.md §1.1 — not read from the robot, so they are
#: only true for that arm. §1.1 also records that the vendored URDF's J6 (±190) is
#: narrower than the controller's own (±225); the controller's values are the
#: authoritative ones and are what is used here.
LIMIT_LOWER_DEG = np.array([-180.0, -180.0, -270.0, -190.0, -180.0, -225.0])
LIMIT_UPPER_DEG = np.array([180.0, 180.0, 270.0, 190.0, 180.0, 225.0])

#: Kept back from the soft limit by the pre-motion guard. A validation script should
#: refuse a move *before* the controller does: a Python abort with no motion is a
#: readable result, while a commanded overrun is a servo alarm that needs a reset on
#: the pendant before anything else can run.
LIMIT_MARGIN_DEG = 1.0


def degrees(q_rad: Any) -> list[float]:
    """Length-6 radian vector → degrees, rounded for printing."""
    return np.round(np.degrees(np.asarray(q_rad, dtype=float)[:NDOF]), 3).tolist()


def rule(title: str = "") -> str:
    """A section rule for the report output."""
    return f"--- {title} ".ljust(76, "-") if title else "-" * 76


# --------------------------------------------------------------------------- #
# Connection: CLI knobs → a target (real controller, or an in-process FakeCRX)
# --------------------------------------------------------------------------- #


def add_connection_args(ap: argparse.ArgumentParser) -> None:
    """Add the connection + RT-hygiene arguments every example shares."""
    ap.add_argument("--ip", default="192.168.1.100", help="controller IP (real robot)")
    ap.add_argument("--fake", action="store_true", help="offline in-process FakeCRX (no hardware)")
    ap.add_argument(
        "--itp-ms",
        type=float,
        default=cf.ITP_S * 1000.0,
        help="controller interpolation period, ms (default %(default)s). Every per-tick limit is "
        "scaled by it; bring-up compares it against the period the controller reports in its "
        "GetCapability reply and refuses a mismatch rather than running with wrongly-scaled limits.",
    )
    ap.add_argument(
        "--sched-fifo",
        action="store_true",
        help="request SCHED_FIFO for the 125 Hz thread (needs CAP_SYS_NICE or an RTPRIO rlimit; "
        "a denied request is tolerated, not fatal)",
    )
    ap.add_argument(
        "--mlock",
        action="store_true",
        help="request mlockall so no page of the process can be swapped out (needs a MEMLOCK "
        "rlimit; a denied request is tolerated, not fatal)",
    )
    ap.add_argument("--rt-priority", type=int, default=80, help="SCHED_FIFO priority for --sched-fifo")
    ap.add_argument("--lock-path", default=None, help="ownership flock path (default: the package's)")
    ap.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
        help="show the driver's own log: -v for INFO (lifecycle transitions, the bring-up "
        "ladder, recovery attempts), -vv for DEBUG (every RMI exchange). The library logs "
        "through the stdlib and installs no handler of its own, so without this the reason "
        "behind a state change is computed and then discarded.",
    )


class Target(NamedTuple):
    """Where to connect and with what configuration.

    ``controller`` is the in-process :class:`~airo_fanuc.testing.FakeCRXController`
    under ``--fake`` (and must be closed by the caller), ``None`` against real
    hardware.
    """

    ip: str
    config: DriverConfig
    controller: Any | None
    lock_path: str | None

    @property
    def is_fake(self) -> bool:
        return self.controller is not None

    def close(self) -> None:
        """Tear down the FakeCRX, if this target owns one."""
        if self.controller is not None:
            self.controller.stop_realtime()
            self.controller.close()


def open_target(args: argparse.Namespace) -> Target:
    """Build the :class:`DriverConfig` from the parsed args, starting a FakeCRX for ``--fake``.

    Under ``--fake`` the fake's own interpolation period follows ``--itp-ms``, so the
    offline run stays self-consistent whatever period is asked for.
    """
    if args.verbose:
        # Bare stderr handler on the package's logger only, so the driver's narrative
        # interleaves with the report instead of being discarded.
        logging.basicConfig(
            level=logging.DEBUG if args.verbose > 1 else logging.INFO,
            format="  %(levelname)-7s %(name)s: %(message)s",
        )
        logging.getLogger("airo_fanuc").setLevel(logging.DEBUG if args.verbose > 1 else logging.INFO)

    itp_s = float(args.itp_ms) / 1000.0
    hygiene = {
        "itp_s": itp_s,
        "sched_fifo": bool(args.sched_fifo),
        "mlock": bool(args.mlock),
        "rt_priority": int(args.rt_priority),
    }

    if args.fake:
        from airo_fanuc.testing import FakeCRXConfig, FakeCRXController

        # available_version=3 → type-202 status, no force block: what the real CRX
        # negotiates (docs/controller-notes.md §1.1).
        controller = FakeCRXController(FakeCRXConfig(available_version=3, itp_s=itp_s))
        controller.start()
        controller.start_realtime(speed=1.0)
        config = DriverConfig(
            sm_port=controller.sm_port,
            rmi_port=controller.rmi_port,
            sm_version=3,
            **hygiene,
        )
        lock_path = str(Path(tempfile.gettempdir()) / "airo-fanuc-example.lock")
        ip = "127.0.0.1"
    else:
        controller = None
        config = DriverConfig(**hygiene)  # default Stream Motion (60015) + RMI (16001)
        lock_path = None
        ip = args.ip

    if args.lock_path is not None:
        lock_path = str(args.lock_path)
    return Target(ip=ip, config=config, controller=controller, lock_path=lock_path)


def build_policy(target: Target, **overrides: Any) -> DriverPolicy:
    """Policy for a validation run: one bring-up attempt, no gripper.

    ``connect_retries=1`` keeps the signal clean — a validation run should report the
    first failure verbatim rather than retrying past it. The gripper is off because
    it is the one part of this package that is specific to a particular end effector.
    """
    kwargs: dict[str, Any] = {"config": target.config, "enable_gripper": False, "connect_retries": 1}
    if target.lock_path is not None:
        kwargs["lock_path"] = target.lock_path
    kwargs.update(overrides)
    return DriverPolicy(**kwargs)


def confirm(lines: Sequence[str], *, delay_s: float = 3.0) -> bool:
    """Print a pre-motion banner and give the operator ``delay_s`` to Ctrl-C out."""
    print("=" * 76)
    for line in lines:
        print(f"  {line}")
    print(f"  Ctrl-C now to abort; starting in {delay_s:.0f} s ...")
    print("=" * 76)
    try:
        time.sleep(delay_s)
    except KeyboardInterrupt:
        print("aborted before connect")
        return False
    return True


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #


def report_bringup(driver: Any, config: DriverConfig) -> None:
    """Print everything bring-up learned: preflight gate, negotiated protocol, state.

    This is the whole point of a first hardware run — every line here is something
    the controller told us, not something this package assumed.
    """
    print(rule("bring-up"))
    report = driver.preflight_report
    print(f"  preflight    : {report.summary() if report is not None else '(not reported)'}")

    core = driver.core
    version = int(core.sm_negotiated_version)
    reported_ms = int(core.sm_sampling_rate_ms)
    wire = "type-202, no force block" if version < 4 else "type-204, force block"
    print(
        f"  stream motion: negotiated v{version} ({wire}), controller ITP {reported_ms} ms "
        f"vs configured {config.itp_s * 1000.0:.3f} ms"
    )
    print(
        f"  rt hygiene   : sched_fifo={config.sched_fifo} (priority {config.rt_priority}) "
        f"mlock={config.mlock} — requested best-effort on the RT thread; no CPU affinity is set"
    )

    st = driver.get_state()
    print(
        f"  lifecycle    : {st.get('lifecycle_state')} fault={st.get('fault_reason')} "
        f"motion_inhibited={st.get('motion_inhibited')}"
    )
    print(
        f"  controller   : motion_possible={st.get('motion_possible')} e_stopped={st.get('e_stopped')} "
        f"in_error={st.get('in_error')} tp_enabled={st.get('tp_enabled')} "
        f"safety_scale={st.get('safety_scale')}"
    )
    print(f"  ownership    : {st.get('owner')}")
    print(f"  joints (deg) : {degrees(st.get('q_meas', [0.0] * NDOF))}")
    wrench = driver.get_wrench()
    print(
        "  wrench       : "
        + (
            "unavailable — a v3 controller streams no force block, so force_stop_n is inert"
            if wrench is None
            else str(np.round(wrench, 2).tolist())
        )
    )
    print(f"  rx age (ms)  : {st.get('rx_age_ms', 0.0):.1f}")


class Watch(NamedTuple):
    """What a watched motion did. ``result`` is ``None`` only on a watch timeout."""

    result: MotionResult | None
    #: Worst ``|q_cmd − q_meas|`` over the run, in degrees.
    max_lag_deg: float
    #: Worst measured joint speed over the run, in deg/s.
    max_speed_deg_s: float
    #: Slew clips the core applied during the run. Non-zero means a commanded step
    #: exceeded ``slew_factor · v_limit · itp_s`` and was clipped.
    slew_clips: int
    samples: int
    elapsed_s: float
    #: Set when ``stop_after_s`` fired: seconds from ``stop_j()`` to standstill and
    #: the distance the arm travelled in that time (worst joint, degrees).
    brake_s: float | None = None
    brake_deg: float | None = None
    #: The first fault seen during the run, with the operator instruction that came
    #: with it. Both ``None`` on a clean run.
    fault: str | None = None
    operator_hint: str | None = None


def watch(
    driver: Any,
    handle: Any,
    *,
    timeout_s: float,
    period_s: float = 0.25,
    stop_after_s: float | None = None,
) -> Watch:
    """Poll a running motion, printing state and measuring how well the arm follows.

    ``q_cmd`` is what the driver put on the wire; ``q_meas`` is what the controller
    reported. Their difference is dominated by the servo lag, which is a property of
    the controller and not an error: at commanded speed ``v`` it settles around
    ``v · tracking_lag_s`` (25 ms measured — docs/controller-notes.md §1.9). A lag far
    above that is the arm not following the commanded path.

    ``stop_after_s`` calls :meth:`FanucDriver.stop_j` that many seconds in — the
    universal preempt — and measures how long the arm takes to reach standstill and
    how far it travels doing so. On a watch timeout the motion is stopped rather than
    left running, and ``result`` comes back ``None``.
    """
    start = time.monotonic()
    deadline = start + timeout_s
    slew_clips_0 = int(driver.get_state().get("total_slew_clips", 0))
    max_lag = 0.0
    max_speed = 0.0
    samples = 0
    stopped_at: float | None = None
    q_at_stop: np.ndarray | None = None
    brake_s: float | None = None
    brake_deg: float | None = None
    fault: str | None = None
    operator_hint: str | None = None
    timed_out = False

    while True:
        result = handle.result()
        st = driver.get_state()
        q_cmd = np.asarray(st.get("q_cmd", [0.0] * NDOF), dtype=float)[:NDOF]
        q_meas = np.asarray(st.get("q_meas", [0.0] * NDOF), dtype=float)[:NDOF]
        qd_est = np.asarray(st.get("qd_est", [0.0] * NDOF), dtype=float)[:NDOF]
        lag_deg = float(np.max(np.abs(np.degrees(q_cmd - q_meas))))
        speed_deg_s = float(np.max(np.abs(np.degrees(qd_est))))
        elapsed = time.monotonic() - start
        max_lag = max(max_lag, lag_deg)
        max_speed = max(max_speed, speed_deg_s)
        samples += 1

        # Formatted with an explicit fallback: a missing key must not crash the monitor
        # loop mid-motion.
        mode_str = str(st.get("lifecycle_state") or "?")
        fault_str = str(st.get("fault_reason") or "none")

        # A fault carries an operator instruction (release the E-stop, RESET on the TP,
        # confirm the payload...). Print it the moment it appears rather than only in
        # the summary: during a drill it is what tells the operator what to do next.
        if fault is None and fault_str.lower() != "none":
            fault = fault_str
            operator_hint = st.get("operator_hint")
            print(f"  !! fault: {fault}" + (f" — {operator_hint}" if operator_hint else ""))

        # While braking, sample fast and quietly: the brake lasts a few ITPs, so the
        # normal sample period would round the measurement to itself.
        braking = stopped_at is not None and brake_s is None
        if not braking:
            print(
                f"  t={elapsed:5.2f}s mode={mode_str:<9} fault={fault_str:<6} "
                f"|q_cmd-q_meas|={lag_deg:5.3f}deg speed={speed_deg_s:6.2f}deg/s "
                f"rx_age={st.get('rx_age_ms', 0.0):4.1f}ms"
            )

        # Standstill after a preempt: measure the brake, then stop reporting it.
        if braking and speed_deg_s < 0.1:
            brake_s = time.monotonic() - stopped_at
            assert q_at_stop is not None
            brake_deg = float(np.max(np.abs(np.degrees(q_meas - q_at_stop))))
            print(f"  brake: standstill {brake_s:.3f}s after stop_j, travelled {brake_deg:.3f}deg")

        # The core flips the motion to STOPPED the tick stop_j lands, which is before
        # the arm has physically stopped — keep sampling until it has (bounded), or the
        # brake measurement would always be missed.
        if result is not None and not (braking and time.monotonic() - float(stopped_at or 0.0) < 2.0):
            break

        if stop_after_s is not None and stopped_at is None and elapsed >= stop_after_s:
            q_at_stop = q_meas.copy()
            stopped_at = time.monotonic()
            print(f"  --> stop_j() at t={elapsed:.2f}s (universal preempt; expect STOPPED)")
            driver.stop_j()

        if time.monotonic() >= deadline:
            timed_out = True
            print(f"  !! motion did not finish within {timeout_s:.1f}s — issuing stop_j()")
            driver.stop_j()
            handle.wait(timeout=5.0)
            break

        # Re-derive rather than reusing `braking`: stop_j may have been issued just
        # above, and the first brake sample is the one worth having at full rate.
        time.sleep(0.002 if (stopped_at is not None and brake_s is None) else period_s)

    return Watch(
        result=None if timed_out else handle.result(),
        max_lag_deg=max_lag,
        max_speed_deg_s=max_speed,
        slew_clips=int(driver.get_state().get("total_slew_clips", 0)) - slew_clips_0,
        samples=samples,
        elapsed_s=time.monotonic() - start,
        brake_s=brake_s,
        brake_deg=brake_deg,
        fault=fault,
        operator_hint=operator_hint,
    )


def report_motion(w: Watch, *, expect_result: MotionResult) -> bool:
    """Print a watched motion's outcome and whether it is the expected one."""
    print(rule("motion"))
    print(f"  result       : {w.result} (expected {expect_result})")
    print(f"  duration     : {w.elapsed_s:.2f}s over {w.samples} samples")
    print(f"  peak speed   : {w.max_speed_deg_s:.2f} deg/s")
    lag_s = cf.INTERIM_FACTS.tracking_lag_s
    print(
        f"  peak lag     : {w.max_lag_deg:.3f} deg — the {lag_s * 1000:.0f} ms recorded servo lag "
        f"accounts for {w.max_speed_deg_s * lag_s:.3f} deg at this peak speed"
    )
    if w.max_speed_deg_s > 0.5:
        # Lag divided by speed is the offset expressed as a time. Reported because it is
        # comparable across runs at different speeds, unlike the raw degrees: if it stays
        # put as the speed changes, the offset is a delay rather than measurement noise.
        # NB this is command-to-reported-measurement, so it includes the status-packet
        # pipeline as well as the servo — it is not the cross-correlation lag of §1.9.
        implied_ms = 1000.0 * w.max_lag_deg / w.max_speed_deg_s
        print(f"                  ⇒ implied offset {implied_ms:.0f} ms at {w.max_speed_deg_s:.1f} deg/s")
        if implied_ms > 2000.0 * lag_s:
            print(
                f"                  NOTE: {implied_ms / (1000.0 * lag_s):.1f}x the recorded lag. "
                f"Open question — docs/controller-notes.md §1.9a"
            )
    print(
        f"  slew clips   : {w.slew_clips}"
        + ("" if w.slew_clips == 0 else "  <-- a commanded step was clipped")
    )
    if w.brake_s is not None:
        print(f"  brake        : standstill in {w.brake_s:.3f}s, travelled {w.brake_deg:.3f} deg")
    if w.fault is not None:
        print(f"  fault        : {w.fault}")
        print(f"  operator     : {w.operator_hint or '(no instruction)'}")
    return w.result == expect_result


def report_rt_health(driver: Any, config: DriverConfig) -> bool:
    """Print the RT loop's timing and decide whether it held its deadline.

    The loop must put exactly one command packet on the wire every interpolation
    period. If it is late, the controller does not fault immediately: it coasts at
    the last commanded velocity and drops ``motion_possible`` after roughly 9–15
    missed periods (measured, docs/controller-notes.md §1.2/§3). So a late tick is not
    instantly dangerous, but it is the number that decides whether this host can run
    the driver — which matters because this package sets no CPU affinity and requires
    no reserved core, and this is the measurement that shows whether that holds here.
    """
    itp_ms = config.itp_s * 1000.0
    ts = driver.timing_stats()
    print(rule("rt health"))
    if not ts:
        print("  (no timing stats available)")
        return False

    print(
        f"  tx interval  : p50 {ts['tx_interval_p50_ms']:.3f}  p99 {ts['tx_interval_p99_ms']:.3f}  "
        f"p999 {ts['tx_interval_p999_ms']:.3f}  max {ts['tx_interval_max_ms']:.3f} ms   (target {itp_ms:.3f})"
    )
    print(
        f"  rx→tx        : p50 {ts['rx2tx_p50_us']:.0f}  p99 {ts['rx2tx_p99_us']:.0f}  "
        f"p999 {ts['rx2tx_p999_us']:.0f}  max {ts['rx2tx_max_us']:.0f} us"
    )
    ticks = int(ts["tick_count"]) or 1
    print(
        f"  counters     : ticks={ts['tick_count']} tx={ts['tx_count']} parked={ts['parked_ticks']} "
        f"tau_advance={ts['tau_advance_count']}"
    )
    missed_pct = 100.0 * int(ts["missed_rx_ticks"]) / ticks
    print(
        f"  rx quality   : missed_rx_ticks={ts['missed_rx_ticks']} ({missed_pct:.2f}% of ticks) "
        f"rx_seq_gaps={ts['rx_seq_gaps']}"
    )
    print(
        f"  cpu          : migrations={ts['cpu_migrations']} — this driver sets no CPU affinity, so any "
        f"count is fine; what matters is the tx interval above"
    )

    # One τ-advance per tick is the core's central invariant: the trajectory clock
    # must advance exactly once per tick, or playback runs fast or slow.
    checks = [
        (
            f"tx interval p50 within 1% of the {itp_ms:.1f} ms ITP",
            abs(ts["tx_interval_p50_ms"] - itp_ms) <= 0.01 * itp_ms,
        ),
        (f"tx interval max under 2 ITP ({2 * itp_ms:.1f} ms)", ts["tx_interval_max_ms"] < 2.0 * itp_ms),
        ("no two sends inside one ITP", int(ts["double_send_guard"]) == 0),
        ("exactly one tau-advance per tick", int(ts["tau_advance_count"]) == int(ts["tick_count"])),
    ]
    ok = True
    for label, passed in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}")
        ok = ok and passed
    return ok


def guard_joint_limits(lo_deg: Any, hi_deg: Any, joints: Sequence[int]) -> bool:
    """Pre-motion guard: does the commanded envelope stay inside the soft limits?

    ``lo_deg`` / ``hi_deg`` are per-joint degree bounds of everything the motion will
    command — the two endpoints of a rest-to-rest move (a cubic Hermite with zero end
    velocities does not leave them), or pose ± amplitude for a sine. Prints one line
    per offending joint and returns False, so the caller can abort before issuing
    anything. Guards the *commanded* path; the controller enforces the real limit.
    """
    lo = np.asarray(lo_deg, dtype=float)[:NDOF]
    hi = np.asarray(hi_deg, dtype=float)[:NDOF]
    bad = []
    for j in joints:
        floor = LIMIT_LOWER_DEG[j] + LIMIT_MARGIN_DEG
        ceil = LIMIT_UPPER_DEG[j] - LIMIT_MARGIN_DEG
        if lo[j] < floor or hi[j] > ceil:
            bad.append(
                f"J{j + 1}: would reach [{lo[j]:.1f}, {hi[j]:.1f}] deg, "
                f"limit [{LIMIT_LOWER_DEG[j]:.0f}, {LIMIT_UPPER_DEG[j]:.0f}] "
                f"less a {LIMIT_MARGIN_DEG:.0f} deg margin"
            )
    if bad:
        print("  ABORT (no motion issued) — the move would leave the soft limits:")
        for line in bad:
            print(f"    {line}")
    return not bad


def close_driver(driver: Any) -> bool:
    """Shut the driver down and say whether it went cleanly.

    ``close()`` is poison-not-exit: it abandons a wedged teardown step and raises
    :class:`FanucError` rather than killing the process. A validation run must catch
    that and report it — an unhandled raise here would replace the verdict with a
    traceback, which is the worst moment to lose the output. Idempotent, because it
    is called both on the normal path and from a ``finally``.
    """
    if getattr(driver, "_closed", False):
        return True
    print(rule("shutdown"))
    try:
        driver.close()
    except FanucError as exc:
        # A wedged join means a thread was abandoned. The flock is released by the
        # kernel when this process dies either way, so the next run is not locked out,
        # but the controller did not get its terminal Stop packet.
        print(f"  !! did not fully quiesce: {exc}")
        return False
    print("  closed cleanly — Stop packet sent, RT thread joined, RMI disconnected, lock released")
    return True


def verdict(name: str, checks: Sequence[tuple[str, bool]], driver: Any = None) -> int:
    """Close the driver, then print the final verdict block; return the exit code.

    Shutting down is part of a run being valid, not an afterthought: an E-stop can
    land at any moment, including while the driver is tearing down, and a close that
    wedges leaves the controller without its terminal Stop packet. So the shutdown
    happens here — before the verdict is printed, and as the last check in it —
    rather than in a ``finally`` whose output would land after the verdict.
    """
    checks = list(checks)
    if driver is not None:
        checks.append(("driver shut down cleanly", close_driver(driver)))
    print(rule("verdict"))
    failed = [label for label, passed in checks if not passed]
    for label, passed in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}")
    if failed:
        print(f"\n{name}: FAILED ({len(failed)} of {len(checks)} checks)")
        return 2
    print(f"\n{name}: PASSED ({len(checks)} checks)")
    return 0
