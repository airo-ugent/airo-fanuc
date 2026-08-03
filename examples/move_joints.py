#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validation step 1: connect to the controller and move one joint.

WRITTEN FOR ONE SPECIFIC ROBOT: a FANUC CRX-10iA/L on an R-30iB-class controller
negotiating Stream Motion v3 at an 8 ms interpolation period. Six joints, the
CRX-10iA/L limit defaults out of ``controller_facts``, and J6 as a wrist roll are all
assumed here, not detected. On a different FANUC, read the table in
``examples/README.md`` first.

The smallest run that proves the whole stack works end to end — ownership lock,
RMI bring-up ladder, preflight gate, Stream Motion handshake, the 125 Hz C++ tick
loop, and one commanded trajectory. Every line it prints is something the
controller reported, and it ends in an explicit PASS/FAIL verdict.

  # Offline — no hardware. Spins up an in-process FakeCRX and drives it:
  python examples/move_joints.py --fake

  # Real controller, bring-up only. NO commanded motion — run this one first:
  python examples/move_joints.py --ip 192.168.1.100 --no-move

  # Real controller, one joint. Operator AT THE ROBOT, E-STOP in hand, area clear:
  python examples/move_joints.py \
      --ip 192.168.1.100 --joint 6 --delta-deg 10 --duration 4 --return

The API is split into a receive side (state getters, never raise) and a control side
(motion commands, blocking or non-blocking); this file is the joint-space slice of it:

    read the current configuration  -> driver.get_state()["q_meas"]   (rad)
    execute a planned trajectory    -> driver.move_trajectory(times, q, qd)   (this file)
    move to a configuration         -> driver.move_j(q, joint_speed=...)      (examples/move_j.py)
    servo towards a configuration   -> driver.servo_j(q, dt)
    (protective) stop               -> driver.stop_j()                (universal preempt)

This file takes the trajectory route on purpose: it is the layer everything else is
built on, so it is the one to prove first. ``move_j`` plans a jerk-limited profile and
submits it through exactly this call — for a point-to-point move with a speed rather
than a duration, reach for that instead of hand-building knots.

TCP-pose moves are deliberately absent: the wheel is numpy-only and ships no
kinematics, so IK/FK — and therefore any Cartesian move — belongs to the caller.
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


def _build_trajectory(q_start_rad, joint_idx: int, delta_rad: float, duration_s: float):
    """A rest-to-rest 2-knot trajectory that moves one joint by ``delta_rad``.

    ``times`` are int64 ns-relative; ``q`` starts at the current commanded pose (so
    the CAPTURE splice is a no-op) and ends with the single joint offset; ``qd`` is
    zero at both ends (the C++ core cubic-Hermite-interpolates the rest-to-rest move).
    """
    q0 = [float(x) for x in np.asarray(q_start_rad, dtype=float)[:NDOF]]
    q1 = list(q0)
    q1[joint_idx] += delta_rad
    times = [0, int(duration_s * 1e9)]
    qd = [[0.0] * NDOF, [0.0] * NDOF]
    return times, [q0, q1], qd


def main() -> int:
    ap = argparse.ArgumentParser(description="Move one FANUC joint a bit via airo_fanuc.FanucDriver.")
    add_connection_args(ap)
    ap.add_argument("--joint", type=int, default=6, help="joint to move, 1-6 (default 6 = wrist roll)")
    ap.add_argument("--delta-deg", type=float, default=10.0, help="relative move, degrees (default 10)")
    ap.add_argument("--duration", type=float, default=4.0, help="move duration, seconds (default 4)")
    ap.add_argument("--return", dest="return_", action="store_true", help="move back to start afterward")
    ap.add_argument("--no-move", action="store_true", help="bring up + stream only; no motion")
    ap.add_argument("--observe", type=float, default=5.0, help="--no-move: seconds to watch")
    args = ap.parse_args()

    if not (1 <= args.joint <= NDOF):
        ap.error("--joint must be in 1..6")
    jidx = args.joint - 1
    delta_rad = math.radians(args.delta_deg)

    target = open_target(args)
    policy = build_policy(target)

    if not target.is_fake:
        banner = (
            [
                "BRING-UP ONLY — no commanded motion. The driver will hold the current pose.",
                f"It streams status + holds for {args.observe:.0f} s, then closes.",
            ]
            if args.no_move
            else [
                "REAL ROBOT MOVE — operator must be present, E-STOP in hand, area clear.",
                f"About to move J{args.joint} by {args.delta_deg:+.1f} deg over {args.duration:.1f} s.",
            ]
        )
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

        # --- no-motion bring-up validation: stream state, confirm no fault, hold ----
        if args.no_move:
            # Settle first, exactly as the motion paths do. A bring-up over a controller
            # that has recently run STREAM_MOTN drops motion_possible for about a second
            # and the recovery ladder restores it (measured, docs/controller-notes.md
            # §4.2), so judging faults from the first sample would report the driver's
            # designed behaviour as a failure. What must hold is that it settles and
            # then STAYS settled.
            settled = wait_streaming(driver)
            checks.append(("settled into stable streaming", settled))
            if not settled:
                print("  never reached stable streaming — see the fault above.")

            print(rule(f"holding for {args.observe:.0f}s (NO commanded motion)"))
            deadline = time.monotonic() + args.observe
            faulted = False
            recoveries_before = int(driver.get_state().get("recovery_count", 0))
            while time.monotonic() < deadline:
                st = driver.get_state()
                print(
                    f"  mode={st.get('lifecycle_state')} fault={st.get('fault_reason')} "
                    f"rx_age_ms={st.get('rx_age_ms', 0.0):.1f} joints_deg={degrees(st['q_meas'])}"
                )
                if str(st.get("fault_reason") or "none").lower() != "none":
                    faulted = True
                time.sleep(1.0)
            # Recoveries are reported, not failed on: one during bring-up is expected on a
            # re-connect. Any during the quiescent hold is not, and shows up as a fault too.
            recoveries = int(driver.get_state().get("recovery_count", 0))
            print(f"  recovery ladder runs: {recoveries_before} during bring-up, {recoveries} total")
            checks.append((f"no fault while holding for {args.observe:.0f}s", not faulted))
            checks.append(("rt loop held its deadline", report_rt_health(driver, target.config)))
            return verdict("bring-up validation", checks, driver)

        # Ride out any post-bring-up motion_possible transient before commanding.
        if not wait_streaming(driver):
            print("  driver did not reach stable streaming — aborting the move (no motion issued).")
            checks.append(("driver reached stable streaming", False))
            return verdict("move_joints", checks, driver)
        checks.append(("driver reached stable streaming", True))

        # Pre-motion guard. The move is rest-to-rest, so its endpoints bound everything
        # it commands; --return comes back to the start, which is where we already are.
        start_deg = np.degrees(q_start)
        end_deg = start_deg.copy()
        end_deg[jidx] += args.delta_deg
        safe = guard_joint_limits(np.minimum(start_deg, end_deg), np.maximum(start_deg, end_deg), [jidx])
        checks.append((f"J{args.joint} move stays inside the soft limits", safe))
        if not safe:
            return verdict("move_joints", checks, driver)

        times, q, qd = _build_trajectory(q_start, jidx, delta_rad, args.duration)
        print(rule(f"moving J{args.joint} by {args.delta_deg:+.1f} deg over {args.duration:.1f}s"))
        handle = driver.move_trajectory(times, q, qd)
        w = watch(driver, handle, timeout_s=args.duration + 5.0)
        checks.append(
            (f"J{args.joint} move returned DONE", report_motion(w, expect_result=MotionResult.DONE))
        )

        # Did the joint actually arrive? DONE already means the core saw the settle
        # criteria met; this re-checks it from the outside, in the units the operator
        # asked in, against the same tolerance the settle policy uses.
        tol_deg = policy.settle.tol_deg
        reached = np.asarray(driver.get_state()["q_meas"], dtype=float)[:NDOF]
        arrived_deg = math.degrees(reached[jidx] - q_start[jidx])
        print(f"  commanded {args.delta_deg:+.3f} deg, measured {arrived_deg:+.3f} deg")
        print(f"  joints now (deg): {degrees(reached)}")
        checks.append(
            (
                f"J{args.joint} arrived within the {tol_deg:.2f} deg settle tolerance "
                f"of the commanded {args.delta_deg:+.1f} deg",
                abs(arrived_deg - args.delta_deg) <= tol_deg,
            )
        )

        if args.return_ and w.result == MotionResult.DONE:
            times, q, qd = _build_trajectory(driver.get_state()["q_cmd"], jidx, -delta_rad, args.duration)
            print(rule("returning to start"))
            back = watch(driver, driver.move_trajectory(times, q, qd), timeout_s=args.duration + 5.0)
            checks.append(("return move returned DONE", report_motion(back, expect_result=MotionResult.DONE)))
            print(f"  joints now (deg): {degrees(driver.get_state()['q_meas'])}")

        checks.append(("rt loop held its deadline", report_rt_health(driver, target.config)))
        return verdict("move_joints", checks, driver)
    except KeyboardInterrupt:
        # Ctrl-C mid-motion: brake first (stop_j is callable from any thread, takes
        # effect within one tick and never raises), then shut down and still report.
        print("\ninterrupted — braking, then shutting down")
        driver.stop_j()
        checks.append(("interrupted by the operator", False))
        return verdict("move_joints", checks, driver)
    finally:
        # Belt and braces: close_driver() is idempotent, so this only does anything on
        # a path that never reached a verdict (an unexpected exception).
        close_driver(driver)
        target.close()


if __name__ == "__main__":
    sys.exit(main())
