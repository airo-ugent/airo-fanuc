#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validation step 2b: point-to-point moves with ``move_j``, at a speed you name.

WRITTEN FOR ONE SPECIFIC ROBOT: a FANUC CRX-10iA/L on an R-30iB-class controller
negotiating Stream Motion v3 at an 8 ms interpolation period. Six joints and the
CRX-10iA/L limits out of ``examples/crx10ial.py`` are assumed here, not detected.

``examples/move_joints.py`` hand-builds a 2-knot trajectory and hands it to
``move_trajectory``; this file asks ``move_j`` for the same journey and lets the driver
shape the profile. The difference is what is worth measuring on hardware:

* **A named speed.** ``joint_speed`` is the LEADING-AXIS speed — it caps every joint,
  and the profile is time-synchronised so the joint travelling furthest runs at it and
  the rest scale down to land with it. ``--multi`` moves three joints different
  distances at once and times each one's arrival, which is the only way to see the
  synchronisation from outside.
* **A jerk-limited profile.** A 2-knot cubic steps the acceleration at both endpoints;
  ``move_j`` ramps it. Two numbers say whether that lands: **slew clips** must stay at
  zero (a clip means the planned profile exceeded the per-tick clamp and the core
  trimmed it, so the executed path is no longer the planned one), and the run must not
  fault with CONTACT_STOP (the CRX collaborative-stop monitor infers contact force from
  motor disturbance torque, so too sharp a ramp reads as a phantom collision).

That second point is the one to watch at speed. The acceleration and jerk clamps in
``crx10ial.py`` are derived at 2x and 8x velocity, not measured, and FANUC publishes much
lower accelerations for this arm; ``move_j`` plans at a fraction of them
(``controller_facts.MOVEJ_LIMIT_SCALE_A`` / ``_J``). Walking ``--speed-scale`` up while
watching for clips and contact stops is how that fraction gets validated.

``--speed-scale`` is a FRACTION of the arm's slowest joint velocity limit, so ``0.25``
means the same thing on any arm and ``1.0`` is the fastest a leading-axis speed can be
while still being reachable by every joint. That is the shape a consumer wants — it is
how ``airo-robots`` picks its own default, ``min(max_joint_speeds) / 4`` — and it keeps
the argument valid by construction rather than needing to be range-checked against the
profile. The script always echoes the absolute deg/s it resolves to, because that is the
number that matters standing next to the robot.

**``--speed-scale`` is applied BEFORE planning**, never to an already-built trajectory:
the profile, its first knot and the capture splice that reaches it are all built at the
resulting speed. Rescaling playback afterwards would leave the splice at the old speed
and step the commanded velocity at the handover; there is no handover to step here.

  # Offline — no hardware. Spins up an in-process FakeCRX and drives it:
  python examples/move_j.py --fake
  python examples/move_j.py --fake --multi

  # Real controller, one joint. Operator AT THE ROBOT, E-STOP in hand, area clear:
  python examples/move_j.py \
      --ip 192.168.1.100 --joint 6 --delta-deg 20 --speed-scale 0.1 --return

  # Three joints at once, to see the synchronisation:
  python examples/move_j.py --ip 192.168.1.100 --multi --speed-scale 0.1 --return
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
from airo_fanuc import FanucDriver, FanucError, MotionResult
from airo_fanuc import controller_facts as cf

#: Joints the --multi move drives, and how far each travels (deg). Deliberately unequal:
#: equal distances would be synchronised by accident. J2/J3/J5 are the ones with room to
#: move on this arm without swinging the tool through the cell.
_MULTI_DELTAS_DEG = {1: 12.0, 2: -8.0, 4: 20.0}

#: Default --speed-scale. Below the library's own MOVEJ_DEFAULT_SPEED_FRACTION (0.25) so
#: a first hardware run is slow: 12 deg/s on the CRX-10iA/L, whose slowest joint is
#: 120 deg/s.
_DEFAULT_SPEED_SCALE = 0.1

#: Slack on the COMMANDED peak. The planner caps every joint at the requested speed
#: exactly, and cubic-Hermite playback between ITP-spaced knots of a smooth profile does
#: not meaningfully overshoot its knot velocities, so this is float dust — not a budget.
_CMD_SPEED_SLACK = 1.001
#: Slack on the MEASURED peak, which is a different quantity: qd_est is differentiated
#: from a quantised position feed, so it overshoots the command as a matter of course
#: (~17.6 deg/s estimated against a 15.000 deg/s command, measured on the FakeCRX plant).
#: This is a gross-error tripwire — the arm running away — not a check on the profile.
_MEAS_SPEED_TRIPWIRE = 1.5
#: Headroom on the computed ramp distance before the "did it reach cruise" check
#: applies. ``_ramp_distance_deg`` is exact for a profile that reaches constant
#: acceleration and approximate below that, so a move only just past the boundary is
#: reported rather than judged.
_CRUISE_MARGIN = 1.05


def _ramp_distance_deg(v_deg_s: float, a_deg_s2: float, j_deg_s3: float) -> float:
    """Travel consumed by accelerating to ``v`` and back to rest, for a double-S profile.

    Jerk-limited acceleration to ``v`` takes ``v/a + a/j`` seconds and covers
    ``v/2`` times that; decelerating costs the same again, hence ``v^2/a + v*a/j``. A
    move shorter than this never reaches ``v`` at all — it is triangular, and peaks
    wherever the distance runs out. That boundary moves with the speed, which is why
    it is computed here rather than fixed: at 15 deg/s on this arm it is under 4 deg,
    at 120 deg/s it is 100 deg.
    """
    return v_deg_s**2 / a_deg_s2 + v_deg_s * a_deg_s2 / j_deg_s3


def _peak_speed_checks(
    w: object,
    requested_deg_s: float,
    travel_deg: float,
    ramp_deg: float,
    checks: list[tuple[str, bool]],
) -> None:
    """Judge the achieved peak speed against the speed that was asked for.

    Two quantities, deliberately judged differently. Whether ``move_j`` honoured the
    speed it was given is a fact about the COMMAND, so that is asserted tightly. What
    the arm was measured doing is the follow, and the estimate carries the feed's noise,
    so it only has to be free of gross error.
    """
    commanded = float(getattr(w, "max_cmd_speed_deg_s", 0.0))
    measured = float(getattr(w, "max_speed_deg_s", 0.0))
    print(
        f"  requested {requested_deg_s:.2f} deg/s -> commanded peak {commanded:.3f} deg/s, "
        f"measured peak {measured:.2f} deg/s"
    )
    checks.append(
        (
            f"commanded peak {commanded:.3f} deg/s never exceeded the requested {requested_deg_s:.2f} deg/s",
            commanded <= requested_deg_s * _CMD_SPEED_SLACK,
        )
    )
    checks.append(
        (
            f"measured peak {measured:.2f} deg/s tracked the command without running away "
            f"(< {requested_deg_s * _MEAS_SPEED_TRIPWIRE:.2f} deg/s)",
            measured <= requested_deg_s * _MEAS_SPEED_TRIPWIRE,
        )
    )
    if travel_deg >= ramp_deg * _CRUISE_MARGIN:
        checks.append(
            (
                f"the move actually reached its commanded speed (>=99% of {requested_deg_s:.2f} deg/s)",
                commanded >= 0.99 * requested_deg_s,
            )
        )
    else:
        print(
            f"  {travel_deg:.1f} deg is too short to reach {requested_deg_s:.2f} deg/s — the "
            f"accel and decel ramps alone need ~{ramp_deg:.1f} deg at this speed, so the move is "
            f"triangular and peaks at {commanded:.2f} deg/s. Not judging cruise; give it more "
            f"travel (--delta-deg) or less speed (--speed-scale) to exercise the cap."
        )


def _profile_health_checks(w: object, checks: list[tuple[str, bool]]) -> None:
    """The two numbers that say the planned profile is the one that executed."""
    clips = int(getattr(w, "slew_clips", 0))
    checks.append(
        (
            "no slew clips — the planned profile fit under the per-tick clamp",
            clips == 0,
        )
    )
    if clips:
        print(
            f"  {clips} slew clips: the profile exceeded slew_factor x v_limit x itp and was "
            f"trimmed, so the executed path is not the planned one. Lower --speed-scale, or "
            f"lower DriverConfig.movej_scale_a / movej_scale_j."
        )
    fault = getattr(w, "fault", None)
    checks.append((f"no fault during the move (saw {fault or 'none'})", fault is None))
    if fault is not None and "contact" in str(fault).lower():
        print(
            "  CONTACT_STOP with a clear cell is the phantom-contact signature: the jerk ramp "
            "read as a collision. Lower DriverConfig.movej_scale_j and re-run."
        )


def _time_arrivals(
    driver: FanucDriver, handle: object, target_rad: np.ndarray, tol_rad: float, timeout_s: float
) -> dict[int, float]:
    """Poll during a motion; return {joint index: seconds until it first settled}.

    ``watch`` reports the run as a whole; synchronisation is a per-joint question, so
    this samples fast and records when each joint first came inside the settle
    tolerance of its own target. Joints that never arrive are simply absent.
    """
    start = time.monotonic()
    arrived: dict[int, float] = {}
    while time.monotonic() - start < timeout_s:
        q = np.asarray(driver.get_state()["q_meas"], dtype=float)[:NDOF]
        now = time.monotonic() - start
        for j in range(NDOF):
            if j not in arrived and abs(q[j] - target_rad[j]) <= tol_rad:
                arrived[j] = now
        if handle.result() is not None:  # type: ignore[attr-defined]
            break
        time.sleep(0.02)
    return arrived


def _report_synchronisation(
    arrivals: dict[int, float], moving: list[int], checks: list[tuple[str, bool]]
) -> None:
    """Did the joints land together? That is what a leading-axis speed buys."""
    print(rule("synchronisation"))
    missing = [j for j in moving if j not in arrivals]
    for j in moving:
        when = arrivals.get(j)
        print(f"  J{j + 1}: " + (f"settled at t={when:.2f}s" if when is not None else "never settled"))
    if missing:
        checks.append((f"every moving joint settled (J{[j + 1 for j in missing]} did not)", False))
        return
    times = [arrivals[j] for j in moving]
    spread = max(times) - min(times)
    # The sampler runs at 50 Hz and the joints are compared against one settle
    # tolerance, so a few tens of ms of spread is measurement, not desynchronisation.
    # Sequential moves would spread by the duration of a whole move instead.
    print(f"  spread: {spread * 1000:.0f} ms between first and last arrival")
    checks.append(
        (f"all {len(moving)} joints landed together (spread {spread * 1000:.0f} ms < 300 ms)", spread < 0.3)
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Point-to-point FANUC moves via FanucDriver.move_j.")
    add_connection_args(ap)
    ap.add_argument("--joint", type=int, default=6, help="joint to move, 1-6 (default 6 = wrist roll)")
    ap.add_argument("--delta-deg", type=float, default=20.0, help="relative move, degrees (default 20)")
    ap.add_argument(
        "--speed-scale",
        type=float,
        default=_DEFAULT_SPEED_SCALE,
        help="leading-axis speed as a FRACTION of the arm's slowest joint velocity limit, "
        "0 < scale <= 1 (default %(default)s). 1.0 is the fastest a leading-axis speed can be "
        "and still be reachable by every joint. The library's own default when a caller names "
        f"no speed is {cf.MOVEJ_DEFAULT_SPEED_FRACTION} (airo-robots' min(max_joint_speeds)/4); "
        "this default is lower so a first hardware run is slow. Walk it up while watching for "
        "slew clips and contact stops.",
    )
    ap.add_argument(
        "--multi",
        action="store_true",
        help=f"move {len(_MULTI_DELTAS_DEG)} joints unequal distances at once and time each "
        f"one's arrival (the synchronisation check)",
    )
    ap.add_argument("--return", dest="return_", action="store_true", help="move back to start afterward")
    args = ap.parse_args()

    if not (1 <= args.joint <= NDOF):
        ap.error("--joint must be in 1..6")
    if not (0.0 < args.speed_scale <= 1.0):
        # Only suggest the /100 reading for values that actually look like a percentage.
        # 1.5 is a fat-fingered fraction, not "1.5%", and 0.015 would be a silly guess.
        hint = f" — did you mean {args.speed_scale / 100:g}?" if 2.0 <= args.speed_scale <= 100.0 else ""
        ap.error(f"--speed-scale must be a fraction in (0, 1], got {args.speed_scale:g}{hint}")

    deltas_deg = dict(_MULTI_DELTAS_DEG) if args.multi else {args.joint - 1: args.delta_deg}
    moving = sorted(deltas_deg)

    target = open_target(args)
    policy = build_policy(target)
    # A fraction OF THE SLOWEST JOINT, because joint_speed is a single leading-axis
    # speed that every joint has to be able to reach — move_j rejects one above
    # min(velocity_limits). That is also what makes the scale valid by construction:
    # any fraction in (0, 1] maps to a speed the arm can take, on any arm.
    vlim_deg_s = np.degrees(target.config.profile.velocity_limits)
    speed_deg_s = args.speed_scale * float(np.min(vlim_deg_s))
    speed_rad_s = args.speed_scale * float(np.min(target.config.profile.velocity_limits))
    speed_note = f"{args.speed_scale:g} x {float(np.min(vlim_deg_s)):.0f} deg/s = {speed_deg_s:.2f} deg/s"

    if not target.is_fake:
        described = ", ".join(f"J{j + 1} by {deltas_deg[j]:+.1f} deg" for j in moving)
        if not confirm(
            [
                "REAL ROBOT MOVE — operator must be present, E-STOP in hand, area clear.",
                f"About to move {described}.",
                f"Leading-axis speed: {speed_note}.",
            ]
        ):
            target.close()
            return 1

    print(f"connecting to {target.ip} (construct-and-go: blocks until commandable or raises) ...")
    checks: list[tuple[str, bool]] = []
    try:
        driver = FanucDriver(target.ip, policy)
    except KeyboardInterrupt:
        print("\naborted during bring-up")
        target.close()
        return 1
    except FanucError as exc:
        print(f"\nbring-up FAILED: {type(exc).__name__}: {exc}")
        target.close()
        return 2

    try:
        report_bringup(driver, target.config)
        checks.append(("bring-up reached a commandable driver", True))

        if not wait_streaming(driver):
            print("  driver did not reach stable streaming — aborting the move (no motion issued).")
            checks.append(("driver reached stable streaming", False))
            return verdict("move_j", checks, driver)
        checks.append(("driver reached stable streaming", True))

        q_start = np.asarray(driver.get_state()["q_cmd"], dtype=float)[:NDOF]
        q_target = q_start.copy()
        for j, delta in deltas_deg.items():
            q_target[j] += math.radians(delta)

        # A rest-to-rest profile is monotone per joint, so its endpoints bound everything
        # it commands — the same guard move_joints.py applies to its cubic Hermite.
        start_deg, end_deg = np.degrees(q_start), np.degrees(q_target)
        safe = guard_joint_limits(np.minimum(start_deg, end_deg), np.maximum(start_deg, end_deg), moving)
        checks.append(("the move stays inside the soft limits", safe))
        if not safe:
            return verdict("move_j", checks, driver)

        travel_deg = float(np.max(np.abs(end_deg - start_deg)))
        # Generous: the settle runs after playback, and a slow speed makes a long move.
        timeout_s = travel_deg / speed_deg_s + 15.0

        # Whether the requested speed is reachable over this distance depends on the
        # LEADING joint's ramps: with time-synchronisation it is the one that takes
        # longest, so it runs at its own acceleration and jerk while the rest stretch to
        # match. move_j plans at a fraction of the profile's clamps, so those fractions
        # are what bound the ramp — not the clamps themselves.
        lead = int(np.argmax(np.abs(end_deg - start_deg)))
        prof = target.config.profile
        ramp_deg = _ramp_distance_deg(
            speed_deg_s,
            target.config.movej_scale_a * float(np.degrees(prof.acceleration_limits[lead])),
            target.config.movej_scale_j * float(np.degrees(prof.jerk_limits[lead])),
        )

        described = ", ".join(f"J{j + 1} {deltas_deg[j]:+.1f} deg" for j in moving)
        print(rule(f"move_j: {described} at {speed_note}"))
        tol_rad = math.radians(policy.settle.tol_deg)

        if args.multi:
            handle = driver.move_j(q_target, joint_speed=speed_rad_s, asynchronous=True)
            arrivals = _time_arrivals(driver, handle, q_target, tol_rad, timeout_s)
            result = handle.wait(timeout=5.0)
            print(f"  result: {result}")
            checks.append(("move_j returned DONE", result == MotionResult.DONE))
            _report_synchronisation(arrivals, moving, checks)
        else:
            handle = driver.move_j(q_target, joint_speed=speed_rad_s)
            w = watch(driver, handle, timeout_s=timeout_s, period_s=0.15)
            checks.append(("move_j returned DONE", report_motion(w, expect_result=MotionResult.DONE)))
            _peak_speed_checks(w, speed_deg_s, travel_deg, ramp_deg, checks)
            _profile_health_checks(w, checks)

        # Did it arrive? DONE means the core saw its settle criteria met; this re-checks
        # from outside, in the operator's units, against that same tolerance.
        reached = np.asarray(driver.get_state()["q_meas"], dtype=float)[:NDOF]
        worst = float(np.max(np.abs(np.degrees(reached - q_target))))
        print(f"  joints now (deg): {degrees(reached)}")
        print(f"  worst joint is {worst:.3f} deg from its target")
        checks.append(
            (
                f"every joint arrived within the {policy.settle.tol_deg:.2f} deg settle tolerance",
                worst <= policy.settle.tol_deg,
            )
        )

        if args.return_ and checks[-1][1]:
            print(rule("returning to start"))
            back = driver.move_j(q_start, joint_speed=speed_rad_s)
            bw = watch(driver, back, timeout_s=timeout_s, period_s=0.15)
            checks.append(("return move returned DONE", report_motion(bw, expect_result=MotionResult.DONE)))
            print(f"  joints now (deg): {degrees(driver.get_state()['q_meas'])}")

        checks.append(("rt loop held its deadline", report_rt_health(driver, target.config)))
        return verdict("move_j", checks, driver)
    except KeyboardInterrupt:
        # Ctrl-C mid-motion: brake first (any thread, one tick, never raises), then
        # shut down and still report.
        print("\ninterrupted — braking, then shutting down")
        driver.stop_j()
        checks.append(("interrupted by the operator", False))
        return verdict("move_j", checks, driver)
    finally:
        close_driver(driver)
        target.close()


if __name__ == "__main__":
    sys.exit(main())
