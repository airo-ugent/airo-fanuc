#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validation step 2: continuous multi-joint tracking, and the protective stop.

Where ``move_joints.py`` proves the stack connects and executes a move, this proves
it *tracks*: every selected joint oscillates ``±amplitude`` degrees about its
CURRENT pose as a slow sine, for a whole number of cycles, so the trajectory both
starts and ends at the start pose. One ``move_trajectory`` of dense cubic-Hermite
knots with analytic velocities — the C++ RT core interpolates and executes it while
this script watches how far measured lags commanded.

  # Offline (no hardware):
  python examples/sine_wave.py --fake

  # Real controller (operator AT THE ROBOT, E-STOP in hand, area clear):
  python examples/sine_wave.py --ip 192.168.1.100 --amplitude-deg 5 --period 10 --cycles 2

  # Protective stop mid-motion: expect STOPPED, plus the measured brake distance:
  python examples/sine_wave.py --ip 192.168.1.100 --stop-after 4

Defaults are deliberately gentle: ±5° at a 10 s period ⇒ peak joint speed ≈ 3.1°/s.
A joint-limit guard aborts (no motion) if any joint's start±amplitude would leave
the CRX-10iA/L soft limits.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from _common import (
    NDOF,
    add_connection_args,
    build_policy,
    confirm,
    degrees,
    open_target,
    report_bringup,
    report_motion,
    report_rt_health,
    rule,
    verdict,
    watch,
)
from airo_fanuc import FanucDriver, MotionResult
from airo_fanuc import controller_facts as cf

# CRX-10iA/L active joint limits (deg), measured on the controller and recorded in
# docs/controller-notes.md §1.1. Used only as a safety guard for this all-joints exercise.
_LIMIT_LOWER_DEG = np.array([-180.0, -180.0, -270.0, -190.0, -180.0, -225.0])
_LIMIT_UPPER_DEG = np.array([180.0, 180.0, 270.0, 190.0, 180.0, 225.0])


def _wait_streaming(driver, hold_s: float = 2.0, timeout_s: float = 10.0) -> bool:
    """Wait until the driver is STABLY streaming (mode=streaming, fault=none held for
    ``hold_s``) — rides out any brief post-bring-up motion_possible transient."""
    deadline = time.monotonic() + timeout_s
    stable_since = None
    while time.monotonic() < deadline:
        st = driver.get_state()
        streaming = st.get("lifecycle_state") == "streaming"
        no_fault = str(st.get("fault_reason") or "none").lower() == "none"
        if streaming and no_fault:
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= hold_s:
                return True
        else:
            stable_since = None
        time.sleep(0.1)
    return False


def _build_sine(q_start_rad, joint_idx, amp_rad: float, period_s: float, cycles: float, knot_dt: float):
    """Dense sine trajectory (times_ns, q, qd). Each selected joint j:
        q_j(t)  = q_start_j + A·sin(ω t),  qd_j(t) = A·ω·cos(ω t),   ω = 2π / period.
    Starts at the current pose (sin 0 = 0) and, over whole cycles, ends there too.
    All joints in phase (a gentle coordinated 'breathe'). Other joints hold."""
    omega = 2.0 * math.pi / period_s
    total = cycles * period_s
    ts = np.arange(0.0, total + knot_dt * 0.5, knot_dt)  # include the endpoint
    q_start = np.asarray(q_start_rad, dtype=float)[:NDOF]
    q = np.tile(q_start, (len(ts), 1))
    qd = np.zeros((len(ts), NDOF))
    for j in joint_idx:
        q[:, j] = q_start[j] + amp_rad * np.sin(omega * ts)
        qd[:, j] = amp_rad * omega * np.cos(omega * ts)
    times_ns = [int(round(t * 1e9)) for t in ts]
    return times_ns, q.tolist(), qd.tolist()


def main() -> int:
    ap = argparse.ArgumentParser(description="±deg sine-wave joint exercise via airo_fanuc.FanucDriver.")
    add_connection_args(ap)
    ap.add_argument("--joints", default="1,2,3,4,5,6", help="comma list of joints 1-6 (default all)")
    ap.add_argument("--amplitude-deg", type=float, default=5.0, help="sine amplitude, deg (default 5)")
    ap.add_argument("--period", type=float, default=10.0, help="seconds per cycle (default 10)")
    ap.add_argument("--cycles", type=float, default=2.0, help="number of full cycles (default 2)")
    ap.add_argument("--knot-dt", type=float, default=0.05, help="trajectory knot spacing, s (default 0.05)")
    ap.add_argument(
        "--stop-after",
        type=float,
        default=None,
        help="call stop_j() this many seconds into the motion and measure the brake "
        "(expects STOPPED instead of DONE)",
    )
    args = ap.parse_args()

    try:
        joint_idx = sorted({int(x) - 1 for x in args.joints.split(",") if x.strip()})
    except ValueError:
        ap.error("--joints must be a comma list of integers, e.g. 1,2,3,4,5,6")
    if not joint_idx or any(j < 0 or j >= NDOF for j in joint_idx):
        ap.error("--joints entries must be in 1..6")
    if args.amplitude_deg <= 0 or args.period <= 0 or args.cycles <= 0 or args.knot_dt <= 0:
        ap.error("--amplitude-deg / --period / --cycles / --knot-dt must be > 0")
    amp_rad = math.radians(args.amplitude_deg)
    peak_speed = args.amplitude_deg * 2.0 * math.pi / args.period  # deg/s
    total_s = args.cycles * args.period
    expect = MotionResult.DONE if args.stop_after is None else MotionResult.STOPPED

    target = open_target(args)
    policy = build_policy(target)

    joints_1based = [j + 1 for j in joint_idx]
    if not target.is_fake:
        banner = [
            "REAL ROBOT MOVE — operator must be present, E-STOP in hand, area clear.",
            f"Sine on J{joints_1based}: ±{args.amplitude_deg:.1f}° about the current pose,",
            f"{args.cycles:g} cycle(s) @ {args.period:.1f}s, peak ~{peak_speed:.1f} deg/s.",
        ]
        if args.stop_after is not None:
            banner.append(f"stop_j() will preempt it {args.stop_after:.1f}s in.")
        if not confirm(banner):
            target.close()
            return 1

    print(f"connecting to {target.ip} (construct-and-go: blocks until commandable or raises) ...")
    checks: list[tuple[str, bool]] = []
    try:
        driver = FanucDriver(target.ip, policy)
    except Exception as exc:
        print(f"\nbring-up FAILED: {type(exc).__name__}: {exc}")
        target.close()
        return 2

    try:
        report_bringup(driver, target.config)
        checks.append(("bring-up reached a commandable driver", True))
        q_start = np.asarray(driver.get_state()["q_cmd"], dtype=float)[:NDOF]

        # Joint-limit guard: abort (no motion) if any joint's start±amplitude leaves limits.
        start_deg = np.degrees(q_start)
        bad = []
        for j in joint_idx:
            lo, hi = start_deg[j] - args.amplitude_deg, start_deg[j] + args.amplitude_deg
            if lo < _LIMIT_LOWER_DEG[j] or hi > _LIMIT_UPPER_DEG[j]:
                lim = f"[{_LIMIT_LOWER_DEG[j]:.0f},{_LIMIT_UPPER_DEG[j]:.0f}]"
                bad.append(f"J{j + 1}: [{lo:.1f},{hi:.1f}] deg outside {lim} deg")
        if bad:
            print("  ABORT (no motion) — sine would exceed joint limits:")
            for b in bad:
                print(f"    {b}")
            checks.append(("sine stays inside the joint limits", False))
            return verdict("sine_wave", checks)

        if not _wait_streaming(driver):
            print("  driver did not reach stable streaming — aborting the move (no motion issued).")
            checks.append(("driver reached stable streaming", False))
            return verdict("sine_wave", checks)
        checks.append(("driver reached stable streaming", True))

        times_ns, q, qd = _build_sine(q_start, joint_idx, amp_rad, args.period, args.cycles, args.knot_dt)
        print(rule(f"sine: {len(times_ns)} knots, ~{total_s:.0f}s, peak ~{peak_speed:.1f} deg/s"))
        handle = driver.move_trajectory(times_ns, q, qd)
        w = watch(driver, handle, timeout_s=total_s + 10.0, stop_after_s=args.stop_after)
        checks.append((f"sine returned {expect.value}", report_motion(w, expect_result=expect)))

        # The arm must have actually moved: a driver that streams a perfect hold while
        # believing it is executing looks identical from the outside otherwise
        # (docs/controller-notes.md §4.1).
        checks.append(
            (
                f"the arm moved (peak measured speed {w.max_speed_deg_s:.2f} deg/s > 0.5)",
                w.max_speed_deg_s > 0.5,
            )
        )
        # Tracking: the lag is the controller's servo lag at this speed, not an error
        # (25 ms measured — docs/controller-notes.md §1.9). Allow 3x that expectation,
        # with a floor so a very slow sine is not judged against a near-zero budget.
        lag_budget = max(3.0 * peak_speed * cf.INTERIM_FACTS.tracking_lag_s, 0.5)
        checks.append(
            (
                f"tracking lag {w.max_lag_deg:.3f} deg stayed under the {lag_budget:.3f} deg budget",
                w.max_lag_deg <= lag_budget,
            )
        )
        checks.append((f"no slew clips (got {w.slew_clips})", w.slew_clips == 0))

        if args.stop_after is None:
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
            checks.append(("stop_j() brought the arm to standstill", w.brake_s is not None))

        checks.append(("rt loop held its deadline", report_rt_health(driver, target.config)))
        return verdict("sine_wave", checks)
    finally:
        driver.close()
        target.close()


if __name__ == "__main__":
    sys.exit(main())
