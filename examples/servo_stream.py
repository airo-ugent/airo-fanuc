#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validation step 7: the same sine, streamed setpoint-by-setpoint from Python.

WRITTEN FOR ONE SPECIFIC ROBOT: a FANUC CRX-10iA/L on an R-30iB-class controller
negotiating Stream Motion v3 at an 8 ms interpolation period. It shares
``sine_wave.py``'s joint-limit guard, which carries that arm's measured soft limits and
is the only thing standing between an ``--amplitude-deg`` argument and a soft-limit
hit — so on a different FANUC it must be replaced, not merely reviewed. See
``examples/README.md``.

``sine_wave.py`` hands the WHOLE path over in one ``move_trajectory`` and the C++ RT
core owns the timeline from there. This script commands the identical raised cosine the
other way round: Python evaluates the setpoint every ``1/rate`` seconds and pushes it
with ``servo_j`` — the replace-not-queue path a teleop loop or an MPC uses, where the
plan does not exist ahead of time. Same motion on purpose, so the two runs are directly
comparable and what differs is the delivery mechanism.

What that moves onto the host: under ``move_trajectory`` a late Python thread costs
nothing, because the 125 Hz C++ loop already holds every knot it needs. A servo stream
puts the host INSIDE the control loop. The core stays starvation-safe — with no fresh
target it re-issues the last one and rests — so a late setpoint is a dwell, not a
fault: the arm stutters rather than faulting, and nothing in the driver's own health
numbers reports it. Measuring that dwell is the point of this script, and it is what
decides whether your host can servo at the rate you want.

  # Offline (no hardware):
  python examples/servo_stream.py --fake

  # Real controller (operator AT THE ROBOT, E-STOP in hand, area clear):
  python examples/servo_stream.py --ip 192.168.1.100 --amplitude-deg 5 --period 10 --rate 50

  # Stop mid-stream: expect a standstill, plus the measured brake distance:
  python examples/servo_stream.py --ip 192.168.1.100 --stop-after 4

Two things about servoing that a trajectory caller never has to think about, both of
which this script is built around:

* **``stop_j()`` does not stop a servo LOOP.** It preempts every target submitted
  before it, but a target submitted AFTER it carries the new stop generation, is not
  superseded, and is accepted — so a stream that keeps feeding restarts the arm a tick
  or two after the brake. Stopping means stopping the loop too, which is what
  ``--stop-after`` does, in that order.
* **A servo stream never ends by itself.** Stop feeding and the core holds the last
  target, in SERVO mode, indefinitely; it never reaches HOLD, so ``is_steady()`` stays
  False however long you wait. ``driver.hold()`` is how a stream is ended — it brakes
  to rest and lands in HOLD.

Defaults are deliberately gentle: a 10° swing at a 10 s period ⇒ peak joint speed
≈ 3.1°/s, streamed at 50 Hz.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any, NamedTuple

import numpy as np

from _common import (
    NDOF,
    add_connection_args,
    build_policy,
    close_driver,
    confirm,
    degrees,
    guard_joint_limits,
    open_target,
    report_bringup,
    report_rt_health,
    rule,
    verdict,
    wait_streaming,
)
from airo_fanuc import FanucDriver, MotionResult, RobotFaultedError
from airo_fanuc import controller_facts as cf
from airo_fanuc._core import Mode


def _sine_setpoint(q_start: np.ndarray, joint_idx: list[int], amp_rad: float, omega: float, t: float):
    """Raised-cosine setpoint ``(q, qd, qdd)`` at time ``t``, in radians.

        q_j(t) = q_start_j + A·(1 − cos ωt),  qd_j = A·ω·sin ωt,  qdd_j = A·ω²·cos ωt

    The path ``sine_wave.py`` builds as a knot table, evaluated one point at a time —
    a servo stream has no table. All selected joints in phase; the others hold.

    A raised cosine rather than a plain sine, for a different reason than in
    ``sine_wave.py``: a servo target is not spliced from the commanded pose, so the
    15 °/s capture envelope that rejects a plain sine there does not apply here. What
    applies instead is that the FIRST target is planned from wherever the arm actually
    is — at rest — so a plain sine's ``qd(0) = A·ω`` asks the online generator to reach
    peak velocity within one update period. At any real amplitude that is not feasible
    inside the servo limits, and the profile spends the start of the run catching up.
    """
    q = np.array(q_start, dtype=float)
    qd = np.zeros(NDOF)
    qdd = np.zeros(NDOF)
    for j in joint_idx:
        q[j] = q_start[j] + amp_rad * (1.0 - math.cos(omega * t))
        qd[j] = amp_rad * omega * math.sin(omega * t)
        qdd[j] = amp_rad * omega * omega * math.cos(omega * t)
    return q, qd, qdd


def _wait_converged(driver: Any, *, timeout_s: float, eps_deg_s: float = 0.5) -> bool:
    """Wait for a starved servo stream to settle onto its last target, still in SERVO.

    ``FanucDriver.is_steady`` deliberately reports rest only in HOLD, so it cannot see
    this: the core stays in SERVO after the targets stop and converges there. Watches
    the measured speed instead, which is what "the arm has stopped" actually means.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        qd = np.asarray(driver.get_state().get("qd_est", [0.0] * NDOF), dtype=float)[:NDOF]
        if float(np.max(np.abs(np.degrees(qd)))) < eps_deg_s:
            return True
        time.sleep(0.01)
    return False


class Stream(NamedTuple):
    """What one servo stream did."""

    #: Targets submitted, and how many the core refused. Nothing rejects a servo
    #: target for distance any more, so a non-zero count means the driver was not
    #: commandable (faulted / recovering), not that the stream got ahead of itself.
    steps: int
    rejected: int
    #: Whether the stream actually drove the core into SERVO mode.
    saw_servo: bool
    #: Worst ``|q_cmd − q_meas|`` and worst measured joint speed over the run.
    max_lag_deg: float
    max_speed_deg_s: float
    #: Worst COMMANDED joint speed, and the worst gap between the commanded pose and
    #: where the plan says it should have been. Both are core-side: they say how
    #: faithfully the driver rendered the stream, before the robot is involved at all.
    max_cmd_speed_deg_s: float
    max_plan_err_deg: float
    #: Slew clips the core applied during the run.
    slew_clips: int
    #: Host-side spacing between successive submissions, ms.
    send_p50_ms: float
    send_max_ms: float
    #: Gaps longer than ``horizon_ms`` — each one a tick where the core had no fresh
    #: target, re-issued the last one and rested (a dwell the arm renders as a stutter).
    starved: int
    horizon_ms: float
    elapsed_s: float
    #: Set when ``stop_after_s`` fired: seconds from ``stop_j()`` to standstill, and
    #: how far the arm travelled getting there (worst joint, degrees).
    brake_s: float | None = None
    brake_deg: float | None = None
    #: The first fault seen, with the operator instruction that came with it.
    fault: str | None = None
    operator_hint: str | None = None


def _stream_sine(
    driver: Any,
    q_start: np.ndarray,
    joint_idx: list[int],
    *,
    amp_rad: float,
    period_s: float,
    cycles: float,
    rate_hz: float,
    itp_s: float,
    feedforward: bool,
    stop_after_s: float | None,
) -> Stream:
    """Feed the raised cosine to ``servo_j`` at ``rate_hz`` and measure what happens.

    THE USAGE PATTERN, and the whole of it: evaluate the plan where it should be one
    update period from now, push it, sleep to the next slot, repeat. ``duration`` is the
    spacing between targets (``1/rate_hz``), not the 8 ms tick — the core stretches its
    profile to exactly fill that gap so the arm arrives moving instead of reaching each
    target early and dwelling until the next one lands.

    The setpoint is evaluated at ``t + dt`` because that is when the core is being asked
    to arrive there. Evaluating it at ``t`` would command the arm to a pose it should
    already have reached, i.e. one update period of built-in lag.

    ``qd``/``qdd`` are passed too (unless ``feedforward`` is off): this loop knows its
    own derivatives analytically, so handing them over lets the generator plan to arrive
    *moving at the right speed*, instead of reconstructing a secant velocity from
    consecutive positions and forcing zero acceleration at every knot.
    """
    dt = 1.0 / rate_hz
    omega = 2.0 * math.pi / period_s
    total_s = cycles * period_s

    # Staleness horizon, derived rather than chosen — it mirrors the arithmetic in
    # tick_core.cpp: the core counts ticks since the last target, goes stale past
    # max(1, ceil(dt/itp)) + 2 of them, and then re-issues the last target, which holds
    # (re-issuing the same position zeroes the reconstructed feedforward). The first
    # held tick is therefore one tick past that, so a submission gap at or beyond this
    # is a gap the arm spent standing still.
    horizon_s = (max(1, math.ceil(dt / itp_s - 1e-9)) + 3) * itp_s

    slew_clips_0 = int(driver.get_state().get("total_slew_clips", 0))
    gaps: list[float] = []
    steps = rejected = 0
    saw_servo = False
    max_lag = max_speed = max_cmd_speed = max_plan_err = 0.0
    fault: str | None = None
    operator_hint: str | None = None
    pending = None
    last_send: float | None = None
    next_print = 0.0
    stopped_at: float | None = None
    q_at_stop: np.ndarray | None = None
    brake_s: float | None = None
    brake_deg: float | None = None

    t0 = time.monotonic()
    next_slot = t0
    while True:
        now = time.monotonic()
        t = now - t0
        if t >= total_s:
            break

        # Last submission's outcome. A servo target that lands normally is simply
        # replaced by the next one and never resolves at all (it stays PENDING —
        # replace-not-queue has no per-target completion), so the only thing worth
        # reading back is a REJECTED. Distance is no longer a reason for one: the core
        # tracks a far target rather than refusing it, so this now means the driver
        # stopped being commandable mid-stream.
        if pending is not None and pending.result() == MotionResult.REJECTED:
            rejected += 1
            if rejected == 1:
                print(f"  !! servo target rejected at t={t:.2f}s — the driver refused the command")

        st = driver.get_state()
        q_cmd = np.asarray(st.get("q_cmd", [0.0] * NDOF), dtype=float)[:NDOF]
        q_meas = np.asarray(st.get("q_meas", [0.0] * NDOF), dtype=float)[:NDOF]
        qd_est = np.asarray(st.get("qd_est", [0.0] * NDOF), dtype=float)[:NDOF]
        qd_cmd = np.asarray(st.get("qd_cmd", [0.0] * NDOF), dtype=float)[:NDOF]
        lag_deg = float(np.max(np.abs(np.degrees(q_cmd - q_meas))))
        speed_deg_s = float(np.max(np.abs(np.degrees(qd_est))))
        max_lag = max(max_lag, lag_deg)
        max_speed = max(max_speed, speed_deg_s)
        max_cmd_speed = max(max_cmd_speed, float(np.max(np.abs(np.degrees(qd_cmd)))))
        # Commanded vs THE PLAN, evaluated at the same instant. This is the one
        # comparison a trajectory run cannot make: with move_trajectory the core holds
        # the plan and renders it itself, whereas here the caller keeps the plan and can
        # ask whether what went on the wire is the path it asked for. Wholly core-side —
        # the robot has not entered into it yet — with a floor of one snapshot age
        # (≤ 1 ITP of travel) from reading q_cmd and the clock at slightly different
        # instants.
        q_now, _, _ = _sine_setpoint(q_start, joint_idx, amp_rad, omega, t)
        max_plan_err = max(max_plan_err, float(np.max(np.abs(np.degrees(q_cmd - q_now)))))
        saw_servo = saw_servo or Mode(int(st.get("mode", 0))) == Mode.SERVO

        fault_str = str(st.get("fault_reason") or "none")
        if fault is None and fault_str.lower() != "none":
            fault = fault_str
            operator_hint = st.get("operator_hint")
            print(f"  !! fault: {fault}" + (f" — {operator_hint}" if operator_hint else ""))

        if t >= next_print:
            next_print = t + 0.25
            print(
                f"  t={t:5.2f}s mode={str(st.get('lifecycle_state') or '?'):<9} fault={fault_str:<6} "
                f"|q_cmd-q_meas|={lag_deg:5.3f}deg speed={speed_deg_s:6.2f}deg/s "
                f"sent={steps:4d} rejected={rejected}"
            )

        # The preempt drill. stop_j() brakes within a tick — but a target submitted
        # after it is NOT superseded by it, so the loop must break here as well or the
        # very next iteration would servo the arm back out of the brake.
        if stop_after_s is not None and t >= stop_after_s:
            q_at_stop = q_meas.copy()
            stopped_at = time.monotonic()
            print(f"  --> stop_j() at t={t:.2f}s, and the stream stops feeding (both are needed)")
            driver.stop_j()
            break

        q, qd, qdd = _sine_setpoint(q_start, joint_idx, amp_rad, omega, t + dt)
        try:
            if feedforward:
                pending = driver.servo_j(q, dt, qd=qd, qdd=qdd)
            else:
                pending = driver.servo_j(q, dt)
        except RobotFaultedError as exc:
            # Unlike a watched trajectory, a servo loop calls into the driver every
            # step, so a fault mid-stream surfaces as a raise from servo_j rather than
            # as a motion result. Stop feeding and report it; the fault itself is
            # already latched in get_state().
            fault = fault or exc.reason
            operator_hint = operator_hint or exc.operator_hint
            print(f"  !! servo_j refused at t={t:.2f}s: {exc}")
            break

        steps += 1
        if last_send is not None:
            gaps.append(now - last_send)
        last_send = now

        # Absolute slots rather than sleep(dt), so the spacing does not drift by one
        # scheduling delay per step. On an overrun, re-base instead of catching up: a
        # burst of back-to-back targets is coalesced by the core's latest-wins mailbox
        # anyway, and would report a healthy cadence that the arm never saw.
        next_slot += dt
        slack = next_slot - time.monotonic()
        if slack > 0.0:
            time.sleep(slack)
        else:
            next_slot = time.monotonic()

    elapsed = time.monotonic() - t0

    # Brake measurement, at full rate: the brake lasts a few ITPs, so the sample period
    # above would round it to itself.
    if stopped_at is not None:
        deadline = stopped_at + 2.0
        while time.monotonic() < deadline:
            st = driver.get_state()
            q_meas = np.asarray(st.get("q_meas", [0.0] * NDOF), dtype=float)[:NDOF]
            speed_deg_s = float(np.max(np.abs(np.degrees(st.get("qd_est", [0.0] * NDOF)))))
            if speed_deg_s < 0.1:
                brake_s = time.monotonic() - stopped_at
                assert q_at_stop is not None
                brake_deg = float(np.max(np.abs(np.degrees(q_meas - q_at_stop))))
                print(f"  brake: standstill {brake_s:.3f}s after stop_j, travelled {brake_deg:.3f}deg")
                break
            time.sleep(0.002)

    gaps_ms = np.array(gaps) * 1000.0 if gaps else np.zeros(1)
    return Stream(
        steps=steps,
        rejected=rejected,
        saw_servo=saw_servo,
        max_lag_deg=max_lag,
        max_speed_deg_s=max_speed,
        max_cmd_speed_deg_s=max_cmd_speed,
        max_plan_err_deg=max_plan_err,
        slew_clips=int(driver.get_state().get("total_slew_clips", 0)) - slew_clips_0,
        send_p50_ms=float(np.percentile(gaps_ms, 50)),
        send_max_ms=float(np.max(gaps_ms)),
        starved=int(np.count_nonzero(gaps_ms > horizon_s * 1000.0)),
        horizon_ms=horizon_s * 1000.0,
        elapsed_s=elapsed,
        brake_s=brake_s,
        brake_deg=brake_deg,
        fault=fault,
        operator_hint=operator_hint,
    )


def report_stream(s: Stream, *, rate_hz: float, plan_peak_deg_s: float) -> None:
    """Print what the stream did, in three groups: what was sent, how faithfully the
    core rendered it, and how well the robot then followed."""
    print(rule("servo stream"))
    print(f"  targets      : {s.steps} sent over {s.elapsed_s:.2f}s at a requested {rate_hz:.0f} Hz")
    print(f"  rejected     : {s.rejected}" + ("" if s.rejected == 0 else "  <-- the core refused a target"))

    # Plan fidelity — core-side, and the question a servo stream exists to ask: did the
    # driver put THIS path on the wire? Reported, not asserted: the position figure has
    # a floor of one snapshot age and neither has a principled ceiling, so a threshold
    # here would be invented rather than derived. The comparison that gives them meaning
    # is sine_wave.py, which hands the same path over in one move_trajectory.
    ratio = s.max_cmd_speed_deg_s / plan_peak_deg_s if plan_peak_deg_s > 0 else 0.0
    print(
        f"  commanded    : peak {s.max_cmd_speed_deg_s:.2f} deg/s against a plan peak of "
        f"{plan_peak_deg_s:.2f} ({ratio:.2f}x)"
    )
    print(f"  plan error   : worst |q_cmd - plan(t)| {s.max_plan_err_deg:.3f} deg")
    if ratio > 1.25:
        print(
            f"                  NOTE: the core commanded {ratio:.1f}x the plan's peak speed. The "
            f"streamed path is not being rendered tightly — compare the same motion under "
            f"sine_wave.py, where the core holds the whole trajectory."
        )

    print(f"  peak speed   : {s.max_speed_deg_s:.2f} deg/s (measured)")
    lag_s = cf.INTERIM_FACTS.tracking_lag_s
    print(
        f"  peak lag     : {s.max_lag_deg:.3f} deg — the {lag_s * 1000:.0f} ms recorded servo lag "
        f"accounts for {s.max_speed_deg_s * lag_s:.3f} deg at this peak speed"
    )
    if s.max_speed_deg_s > 0.5:
        # Lag divided by speed is the offset expressed as a time — comparable across
        # runs at different speeds, and comparable to the same number in sine_wave.py,
        # which is the interesting comparison: it says what streaming the path from
        # Python costs over handing the same path to the core in one call. It stays
        # comparable only because the setpoint is read one update period ahead; take
        # the lookahead out and 1000/rate ms of pure scheme lag lands on top of it.
        implied_ms = 1000.0 * s.max_lag_deg / s.max_speed_deg_s
        print(f"                  ⇒ implied offset {implied_ms:.0f} ms at {s.max_speed_deg_s:.1f} deg/s")
        if implied_ms > 2000.0 * lag_s:
            print(
                f"                  NOTE: {implied_ms / (1000.0 * lag_s):.1f}x the recorded lag. "
                f"Open question — docs/controller-notes.md §1.9a"
            )
    print(
        f"  send spacing : p50 {s.send_p50_ms:.3f}  max {s.send_max_ms:.3f} ms   "
        f"(requested {1000.0 / rate_hz:.3f}, staleness horizon {s.horizon_ms:.3f})"
    )
    print(
        f"  starved      : {s.starved} gap(s) past the horizon"
        + (
            ""
            if s.starved == 0
            else "  <-- the core re-issued its last target and rested; the arm dwelled there"
        )
    )
    print(f"  slew clips   : {s.slew_clips}" + ("" if s.slew_clips == 0 else "  <-- a step was clipped"))
    if s.brake_s is not None:
        print(f"  brake        : standstill in {s.brake_s:.3f}s, travelled {s.brake_deg:.3f} deg")
    if s.fault is not None:
        print(f"  fault        : {s.fault}")
        print(f"  operator     : {s.operator_hint or '(no instruction)'}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Stream a sine to the robot one servo_j setpoint at a time, and measure the cadence."
    )
    add_connection_args(ap)
    ap.add_argument("--joints", default="1,2,3,4,5,6", help="comma list of joints 1-6 (default all)")
    ap.add_argument("--amplitude-deg", type=float, default=5.0, help="sine amplitude, deg (default 5)")
    ap.add_argument("--period", type=float, default=10.0, help="seconds per cycle (default 10)")
    ap.add_argument("--cycles", type=float, default=2.0, help="number of full cycles (default 2)")
    ap.add_argument("--rate", type=float, default=50.0, help="servo target rate, Hz (default 50)")
    ap.add_argument(
        "--no-feedforward",
        action="store_true",
        help="send positions only, letting the core reconstruct a secant velocity, instead of passing "
        "the analytic qd/qdd this loop already knows. Comparing the two peak lags is the point.",
    )
    ap.add_argument(
        "--starved-pct",
        type=float,
        default=1.0,
        help="fail if more than this %% of send gaps exceed the staleness horizon (default 1)",
    )
    ap.add_argument(
        "--stop-after",
        type=float,
        default=None,
        help="call stop_j() this many seconds in AND stop feeding targets, then measure the brake",
    )
    args = ap.parse_args()

    try:
        joint_idx = sorted({int(x) - 1 for x in args.joints.split(",") if x.strip()})
    except ValueError:
        ap.error("--joints must be a comma list of integers, e.g. 1,2,3,4,5,6")
    if not joint_idx or any(j < 0 or j >= NDOF for j in joint_idx):
        ap.error("--joints entries must be in 1..6")
    if args.amplitude_deg <= 0 or args.period <= 0 or args.cycles <= 0 or args.rate <= 0:
        ap.error("--amplitude-deg / --period / --cycles / --rate must be > 0")
    amp_rad = math.radians(args.amplitude_deg)
    peak_speed = args.amplitude_deg * 2.0 * math.pi / args.period  # deg/s
    total_s = args.cycles * args.period

    target = open_target(args)
    policy = build_policy(target)
    itp_s = target.config.itp_s
    if args.rate > 1.0 / itp_s:
        print(
            f"note: --rate {args.rate:g} Hz is above the {1.0 / itp_s:.0f} Hz tick rate. The core's "
            f"mailbox is latest-wins, so the extra targets are coalesced away, not queued."
        )

    joints_1based = [j + 1 for j in joint_idx]
    if not target.is_fake:
        banner = [
            "REAL ROBOT MOVE — operator must be present, E-STOP in hand, area clear.",
            f"Servo stream on J{joints_1based}: from the current pose to "
            f"+{2 * args.amplitude_deg:.1f}° and back (starts and ends at rest),",
            f"{args.cycles:g} cycle(s) @ {args.period:.1f}s, peak ~{peak_speed:.1f} deg/s, "
            f"fed at {args.rate:.0f} Hz from this process.",
        ]
        if args.stop_after is not None:
            banner.append(f"stop_j() will preempt it {args.stop_after:.1f}s in, and the stream will stop.")
        if not confirm(banner):
            target.close()
            return 1

    print(f"connecting to {target.ip} (construct-and-go: blocks until commandable or raises) ...")
    checks: list[tuple[str, bool]] = []
    try:
        driver = FanucDriver(target.ip, policy)
    except KeyboardInterrupt:
        # The constructor's own cleanup already released the lock and stopped the core.
        print("\naborted during bring-up")
        target.close()
        return 1
    except Exception as exc:
        print(f"\nbring-up FAILED: {type(exc).__name__}: {exc}")
        target.close()
        return 2

    try:
        report_bringup(driver, target.config)
        checks.append(("bring-up reached a commandable driver", True))
        q_start = np.asarray(driver.get_state()["q_cmd"], dtype=float)[:NDOF]

        # Pre-motion guard: the raised cosine reaches exactly start .. start + 2A.
        start_deg = np.degrees(q_start)
        safe = guard_joint_limits(start_deg, start_deg + 2.0 * args.amplitude_deg, joint_idx)
        checks.append(("sine stays inside the soft limits", safe))
        if not safe:
            return verdict("servo_stream", checks, driver)

        # There is deliberately no per-step distance guard here. The core used to
        # refuse a servo target more than 5° from the commanded pose, which made the
        # rate/speed pairing a pre-motion check; it no longer does — a far target is
        # tracked under the servo limits instead. What the rate still decides is
        # tracking quality, which the run measures rather than predicts.
        step_deg = peak_speed / args.rate
        print(f"  peak step    : {step_deg:.2f} deg per target at {args.rate:g} Hz")

        if not wait_streaming(driver):
            print("  driver did not reach stable streaming — aborting the stream (no motion issued).")
            checks.append(("driver reached stable streaming", False))
            return verdict("servo_stream", checks, driver)
        checks.append(("driver reached stable streaming", True))

        mode = "position + analytic qd/qdd" if not args.no_feedforward else "position only"
        print(
            rule(f"servo stream: {args.rate:.0f} Hz, ~{total_s:.0f}s, peak ~{peak_speed:.1f} deg/s, {mode}")
        )
        s = _stream_sine(
            driver,
            q_start,
            joint_idx,
            amp_rad=amp_rad,
            period_s=args.period,
            cycles=args.cycles,
            rate_hz=args.rate,
            itp_s=itp_s,
            feedforward=not args.no_feedforward,
            stop_after_s=args.stop_after,
        )
        report_stream(s, rate_hz=args.rate, plan_peak_deg_s=peak_speed)

        checks.append(("the stream drove the core into SERVO mode", s.saw_servo))
        checks.append((f"no servo target was rejected (got {s.rejected})", s.rejected == 0))
        checks.append((f"no fault during the stream (got {s.fault or 'none'})", s.fault is None))
        # The arm must have actually moved: a driver that streams a perfect hold while
        # believing it is servoing looks identical from the outside otherwise
        # (docs/controller-notes.md §4.1).
        checks.append(
            (
                f"the arm moved (peak measured speed {s.max_speed_deg_s:.2f} deg/s > 0.5)",
                s.max_speed_deg_s > 0.5,
            )
        )
        checks.append((f"no slew clips (got {s.slew_clips})", s.slew_clips == 0))
        # The headline check, and the one that is genuinely about THIS host: could it
        # hold the cadence it asked for? Tracking is measured and reported rather than
        # asserted, for the reason sine_wave.py records — no honest threshold exists at
        # this altitude — but starvation is different: the horizon is derived from the
        # core's own staleness arithmetic, so a gap past it is a dwell that happened,
        # not a model of one. A small tail is tolerated because a plain Python loop is
        # not a real-time thread; a large one means this host cannot servo this fast.
        gaps = max(s.steps - 1, 1)
        starved_pct = 100.0 * s.starved / gaps
        checks.append(
            (
                f"host held the {args.rate:.0f} Hz cadence: {s.starved} of {gaps} gaps past the "
                f"{s.horizon_ms:.1f} ms staleness horizon ({starved_pct:.2f}% <= {args.starved_pct:g}%)",
                starved_pct <= args.starved_pct,
            )
        )

        if args.stop_after is None:
            # Ending a stream takes two steps, in this order.
            #
            # First LET IT DRAIN. The tracker is best-effort, so when the last target
            # goes out the command is still short of it by roughly the response lag —
            # about 115 ms of travel, which is nothing at 3 °/s and over 5° at 60. The
            # core is starvation-safe: with no fresh target it re-issues the last one
            # and converges to it. Skipping this and braking immediately abandons that
            # tail, and the arm stops wherever the tracker had got to (measured: 5.2°
            # short of the start pose at a 2 s period, against 0.14° when drained).
            # Done inline rather than with wait_until_steady(), which reports rest only
            # in HOLD — and the whole point here is to reach rest while still in SERVO.
            drained = _wait_converged(driver, timeout_s=3.0)

            # THEN hold(). Draining leaves the core resting but still in SERVO, and it
            # stays there indefinitely — is_steady() requires HOLD, so it never comes
            # true on its own. hold() brakes to rest and lands in HOLD; after a drain
            # the brake is a formality.
            driver.hold()
            steady = driver.wait_until_steady(timeout=3.0)
            checks.append(("stream drained, then hold() ended it at rest in HOLD", drained and steady))

            # Whole cycles ⇒ the sine ends where it started.
            end_deg = np.degrees(np.asarray(driver.get_state()["q_meas"], dtype=float)[:NDOF])
            drift = float(np.max(np.abs(end_deg - start_deg)))
            print(f"  start (deg): {degrees(q_start)}")
            print(f"  end   (deg): {np.round(end_deg, 3).tolist()}")
            checks.append(
                (
                    f"returned to the start pose within the {policy.settle.tol_deg:.2f} deg "
                    f"settle tolerance (worst joint {drift:.3f} deg)",
                    drift <= policy.settle.tol_deg,
                )
            )
        else:
            print(f"  joints now (deg): {degrees(driver.get_state()['q_meas'])}")
            checks.append(("stop_j() brought the arm to standstill", s.brake_s is not None))
            # And it must STAY stopped. This is the check that the loop really did stop
            # feeding: a stream that kept going would have servoed the arm back out of
            # the brake within a tick or two, so a second of standstill is the evidence
            # that stop_j() plus stopping the loop is what actually stops a servo stream.
            braked_deg = np.degrees(np.asarray(driver.get_state()["q_meas"], dtype=float)[:NDOF])
            time.sleep(1.0)
            after_deg = np.degrees(np.asarray(driver.get_state()["q_meas"], dtype=float)[:NDOF])
            crept = float(np.max(np.abs(after_deg - braked_deg)))
            checks.append(
                (
                    f"the arm stayed stopped for a further 1.0s — nothing resumed it "
                    f"(crept {crept:.3f} deg)",
                    crept <= policy.settle.tol_deg,
                )
            )

        checks.append(("rt loop held its deadline", report_rt_health(driver, target.config)))
        return verdict("servo_stream", checks, driver)
    except KeyboardInterrupt:
        # Ctrl-C mid-stream: brake first (stop_j is callable from any thread, takes
        # effect within one tick and never raises), then shut down and still report.
        # The loop is already gone with the exception, so nothing re-feeds the core.
        print("\ninterrupted — braking, then shutting down")
        driver.stop_j()
        checks.append(("interrupted by the operator", False))
        return verdict("servo_stream", checks, driver)
    finally:
        # Belt and braces: close_driver() is idempotent, so this only does anything on
        # a path that never reached a verdict (an unexpected exception).
        close_driver(driver)
        target.close()


if __name__ == "__main__":
    sys.exit(main())
