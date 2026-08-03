#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validation step 2: continuous multi-joint tracking, and the protective stop.

WRITTEN FOR ONE SPECIFIC ROBOT: a FANUC CRX-10iA/L on an R-30iB-class controller
negotiating Stream Motion v3 at an 8 ms interpolation period. The joint-limit guard
below carries that arm's measured soft limits, and it is the only thing standing
between an ``--amplitude-deg`` argument and a soft-limit hit — so on a different
FANUC it must be replaced, not merely reviewed. See ``examples/README.md``.

Where ``move_joints.py`` proves the stack connects and executes a move, this proves
it *tracks*: every selected joint swings from its CURRENT pose out to
``+2 × amplitude`` degrees and back as a raised cosine, for a whole number of cycles,
so the trajectory starts and ends at the start pose AT REST (see ``_build_sine`` for
why at rest matters — a plain sine is refused above 15 °/s). One ``move_trajectory``
of dense cubic-Hermite knots with analytic velocities — the C++ RT core interpolates
and executes it while this script watches how far measured lags commanded.

  # Offline (no hardware):
  python examples/sine_wave.py --fake

  # Real controller (operator AT THE ROBOT, E-STOP in hand, area clear):
  python examples/sine_wave.py --ip 192.168.1.100 --amplitude-deg 5 --period 10 --cycles 2

  # Protective stop mid-motion: expect STOPPED, plus the measured brake distance:
  python examples/sine_wave.py --ip 192.168.1.100 --stop-after 4

Defaults are deliberately gentle: a 10° swing at a 10 s period ⇒ peak joint speed
≈ 3.1°/s. A joint-limit guard aborts (no motion) if the swing would leave the
CRX-10iA/L soft limits.
"""

from __future__ import annotations

import argparse
import math
import sys

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
    report_motion,
    report_rt_health,
    rule,
    verdict,
    wait_streaming,
    watch,
)
from airo_fanuc import FanucDriver, MotionResult


def _build_sine(q_start_rad, joint_idx, amp_rad: float, period_s: float, cycles: float, knot_dt: float):
    """Dense raised-cosine trajectory (times_ns, q, qd). Each selected joint j:
        q_j(t)  = q_start_j + A·(1 − cos(ω t)),   qd_j(t) = A·ω·sin(ω t),   ω = 2π / period.
    All joints in phase (a coordinated 'breathe'). Other joints hold.

    A raised cosine rather than a plain sine, because ``q_start + A·sin(ω t)`` has
    ``qd(0) = A·ω`` — it demands its PEAK velocity instantaneously at t=0. The core
    bridges the commanded pose to a trajectory's first knot with a bounded capture
    splice whose velocity envelope is ``controller_facts.CAPTURE_RATE_DEG_S`` (15 °/s),
    so a plain sine is rejected outright the moment its peak exceeds 15 °/s — which is
    12% of this arm's velocity limit and would make this script useless for anything but
    slow motion. Measured 2026-07-30: rejected at 15.1 °/s, accepted at 15.0.

    The raised cosine has the SAME peak velocity (A·ω), acceleration (A·ω²) and jerk
    (A·ω³) for the same amplitude and period, but starts and ends at rest, so nothing
    needs bridging. The excursion is one-sided: it swings from the start pose to
    ``+2A`` and back, the same peak-to-peak as ``±A``.
    """
    omega = 2.0 * math.pi / period_s
    total = cycles * period_s
    ts = np.arange(0.0, total + knot_dt * 0.5, knot_dt)  # include the endpoint
    q_start = np.asarray(q_start_rad, dtype=float)[:NDOF]
    q = np.tile(q_start, (len(ts), 1))
    qd = np.zeros((len(ts), NDOF))
    for j in joint_idx:
        q[:, j] = q_start[j] + amp_rad * (1.0 - np.cos(omega * ts))
        qd[:, j] = amp_rad * omega * np.sin(omega * ts)
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
            f"Sine on J{joints_1based}: from the current pose to "
            f"+{2 * args.amplitude_deg:.1f}° and back (starts and ends at rest),",
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
            return verdict("sine_wave", checks, driver)

        if not wait_streaming(driver):
            print("  driver did not reach stable streaming — aborting the move (no motion issued).")
            checks.append(("driver reached stable streaming", False))
            return verdict("sine_wave", checks, driver)
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
        # Tracking is MEASURED AND REPORTED, not asserted (report_motion prints the peak
        # lag and the offset it implies in ms). There is no threshold here because no
        # honest one exists at this altitude: a budget modelled on tracking_lag_s asserts
        # the model rather than the robot, and on this arm that model is out by ~3.4x
        # (docs/controller-notes.md §1.9a). Divergence big enough to matter trips the
        # controller's own deviation monitor (§1.2), which lands here as a non-DONE
        # result.
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
        return verdict("sine_wave", checks, driver)
    except KeyboardInterrupt:
        # Ctrl-C mid-sine: brake first (stop_j is callable from any thread, takes effect
        # within one tick and never raises), then shut down and still report.
        print("\ninterrupted — braking, then shutting down")
        driver.stop_j()
        checks.append(("interrupted by the operator", False))
        return verdict("sine_wave", checks, driver)
    finally:
        # Belt and braces: close_driver() is idempotent, so this only does anything on
        # a path that never reached a verdict (an unexpected exception).
        close_driver(driver)
        target.close()


if __name__ == "__main__":
    sys.exit(main())
