#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Read J3 on BOTH planes and work out whether the offset tracks J2. NO MOTION by default.

WRITTEN FOR ONE SPECIFIC ROBOT: a FANUC CRX-10iA/L on an R-30iB-class controller
negotiating Stream Motion v3 at an 8 ms interpolation period. See
``examples/README.md`` before running it on anything else.

A FANUC controller can report J3 absolutely, or relative to J2 — the "interlinked" or
J2/J3-coupled representation. Which one a given plane serves is a controller
CONFIGURATION, not a property of the arm, and this driver reads joints on two planes
that need not agree:

    get_state()["q_meas"]        Stream Motion status packet, 125 Hz, non-blocking
    rmi.read_joint_angles()      FRC_ReadJointAngles, one RMI round trip

``docs/controller-notes.md`` §1.5 records ``RMI J3 = SM J3 - J2`` on the controller this
driver was developed against, measured at two poses far enough apart in J2 to establish
that the offset tracks J2 rather than being a fixed calibration offset. That says nothing
about YOUR controller: the coupling is a controller configuration, so the driver ships with
the conversion OFF, tags RMI joints ``rmi_unconverted``, and has the calibration path
hard-reject them. Being wrong in either direction is a silent J2-sized error in every pose
derived from it, so the default is the one that fails loudly.

This script settles it for your cell. It reads both planes at one standstill pose, then —
with ``--move`` — takes J2 somewhere materially different and reads both again:

    offset tracks J2       -> the coupled representation; the conversion is J3 += J2
    offset stays constant  -> a fixed calibration offset, NOT a J2 coupling
    offset does neither    -> stop trusting either plane's J3 and read the pendant

IT DOES NOT PATCH THE PACKAGE. What it prints is the gate to set for YOUR controller.
A number measured in one cell is not true of the next installation, so the finding
belongs in your configuration, not compiled into a published driver.

Both planes must be read at ONE STANDSTILL POSE inside ONE session, which is why this
needs the driver rather than a bare RMI client: the two reads are of the same instant
only if nothing is moving, and a stationary check is the only one where a 25 ms RMI
round trip cannot be confused with real motion.

  # Offline — no hardware, exercises the comparison against an in-process FakeCRX:
  python examples/verify_j2j3_coupling.py --fake

  # Real controller, read-only. NOTHING MOVES. Reports the offset at the pose it finds:
  python examples/verify_j2j3_coupling.py --ip 192.168.1.100

  # The full answer: read, move J2, read again, compare.
  # Operator AT THE ROBOT, E-STOP in hand, area clear:
  python examples/verify_j2j3_coupling.py --ip 192.168.1.100 --move

  # Pick the second J2 explicitly (default is +25 deg from wherever J2 starts):
  python examples/verify_j2j3_coupling.py --ip 192.168.1.100 --move --j2-delta-deg -30

Also reported, because a bring-up is needed anyway and both are open questions that a
stationary read settles: what ``safety_scale`` reads when no rescaling is in effect, and
whether ``in_error`` is latched.
"""

from __future__ import annotations

import argparse
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
    rule,
    verdict,
    wait_streaming,
)
from airo_fanuc import FanucDriver

#: Below this, "tracks J2" and "fixed offset" are not distinguishable and the run says so
#: rather than guessing. The recorded measurement was taken at 2.595 deg, which is why one
#: pose could not settle it; 10 deg of separation makes the two hypotheses differ by more
#: than any plausible read noise (the RMI wire quantizes to 0.0001 deg).
MIN_J2_SEPARATION_DEG = 10.0

#: Agreement tolerance for a per-joint comparison across the two planes. The RMI reply
#: quantizes to 0.0001 deg and the Stream Motion status carries float32, so this is wire
#: resolution rather than a physical tolerance.
WIRE_TOL_DEG = 0.001

#: How long to hold still before reading. The RMI round trip and the 125 Hz sample are
#: separate events; only a stationary arm makes them the same pose.
SETTLE_S = 0.5


def _read_both_planes(driver: FanucDriver) -> tuple[list[float], list[float]] | None:
    """(stream_deg, rmi_deg) at one standstill pose, or None if the arm would not settle."""
    if not driver.wait_until_steady(3.0):
        print("  the arm is not steady — refusing to read (a moving read compares two instants)")
        return None
    time.sleep(SETTLE_S)
    stream = degrees(driver.get_state()["q_meas"])[:NDOF]
    assert driver.rmi is not None
    rmi = [float(v) for v in driver.rmi.read_joint_angles()[:NDOF]]
    return stream, rmi


def _report_pose(label: str, stream: list[float], rmi: list[float]) -> float:
    """Print the per-joint comparison; return the J3 offset (RMI minus stream), deg."""
    print(rule(f"{label}"))
    print(f"    {'joint':<8}{'stream':>12}{'RMI':>12}{'RMI - stream':>15}")
    for j in range(NDOF):
        d = rmi[j] - stream[j]
        flag = "" if abs(d) <= WIRE_TOL_DEG else "   <-- differs"
        print(f"    J{j + 1:<7}{stream[j]:>12.4f}{rmi[j]:>12.4f}{d:>15.4f}{flag}")
    offset = rmi[2] - stream[2]
    print(f"\n    J2 = {stream[1]:+.4f} deg,  J3 offset (RMI - stream) = {offset:+.4f} deg")
    return offset


def _other_joints_agree(stream: list[float], rmi: list[float]) -> bool:
    """Every joint except J3 must agree, or the difference is not a J2/J3 question."""
    bad = [j for j in range(NDOF) if j != 2 and abs(rmi[j] - stream[j]) > WIRE_TOL_DEG]
    if bad:
        print(f"    joints {[f'J{j + 1}' for j in bad]} ALSO differ across the planes.")
        print("    That is not a J2/J3 representation difference — do not apply any J3")
        print("    conversion, and check UFRAME / the pendant before trusting either plane.")
    return not bad


def _classify(j2_a: float, off_a: float, j2_b: float, off_b: float) -> str:
    """Which hypothesis the two readings support: 'tracks', 'fixed', or 'neither'."""
    d_j2 = j2_b - j2_a
    d_off = off_b - off_a
    # "Tracks J2" predicts d_off == -d_j2 (RMI J3 = SM J3 - J2). "Fixed" predicts d_off == 0.
    tracks = abs(d_off + d_j2)
    fixed = abs(d_off)
    print(f"    J2 moved {d_j2:+.4f} deg;  the J3 offset moved {d_off:+.4f} deg")
    print(f"    'offset tracks J2' predicts {-d_j2:+.4f} — residual {tracks:.4f} deg")
    print(f"    'offset is fixed'   predicts {0.0:+.4f} — residual {fixed:.4f} deg")
    if tracks <= WIRE_TOL_DEG * 10 and fixed > abs(d_j2) / 2:
        return "tracks"
    if fixed <= WIRE_TOL_DEG * 10 and tracks > abs(d_j2) / 2:
        return "fixed"
    return "neither"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Read J3 on both planes and determine whether the offset tracks J2.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_connection_args(ap)
    ap.add_argument(
        "--move",
        action="store_true",
        help="move J2 to a second value and re-read, which is what actually settles the "
        "question. Without this the run reports the offset at one pose only.",
    )
    ap.add_argument(
        "--j2-delta-deg",
        type=float,
        default=25.0,
        help="how far to move J2 for the second reading (default %(default)s). Sign is yours "
        f"to choose; magnitude must exceed {MIN_J2_SEPARATION_DEG:.0f} deg, below which the "
        "two hypotheses are not distinguishable from the readings.",
    )
    ap.add_argument(
        "--joint-speed-deg-s",
        type=float,
        default=8.0,
        help="leading-axis speed for the J2 move (default %(default)s, deliberately slow)",
    )
    args = ap.parse_args(argv)

    if args.move and abs(args.j2_delta_deg) < MIN_J2_SEPARATION_DEG:
        print(
            f"--j2-delta-deg {args.j2_delta_deg} is under {MIN_J2_SEPARATION_DEG} deg. Below that, "
            "'tracks J2' and 'fixed offset' are not distinguishable from the readings, which is "
            "the exact reason one pose could not settle this."
        )
        return 2

    target = open_target(args)
    checks: list[tuple[str, bool]] = []
    driver = None
    try:
        if args.move and not target.is_fake:
            if not confirm(
                [
                    "THE ROBOT WILL MOVE.",
                    f"J2 by {args.j2_delta_deg:+.1f} deg at {args.joint_speed_deg_s:.0f} deg/s, then back.",
                    "Stand clear of the envelope. Keep the E-stop in hand.",
                ]
            ):
                return 1

        print(f"connecting to {target.ip} (construct-and-go: blocks until commandable or raises) ...")
        driver = FanucDriver(target.ip, build_policy(target))
        report_bringup(driver, target.config)
        checks.append(("bring-up reached streaming", wait_streaming(driver)))
        if not checks[-1][1]:
            return verdict("verify_j2j3_coupling", checks, driver)

        # Two open questions a stationary read settles for free.
        st = driver.get_state()
        scale = float(st["safety_scale"])
        print(rule("at rest"))
        print(f"    safety_scale = {scale:.4f}   (with no rescaling in effect)")
        print(f"    in_error     = {bool(st['in_error'])}")
        if scale == 0.0:
            print("    ^ 0.0 at rest would latch SAFETY_CLAMP on the first streaming tick.")
        checks.append(("safety_scale at rest is not 0.0", scale != 0.0))

        first = _read_both_planes(driver)
        if first is None:
            return verdict("verify_j2j3_coupling", checks, driver)
        stream_a, rmi_a = first
        off_a = _report_pose("pose A", stream_a, rmi_a)
        checks.append(("only J3 differs across the planes", _other_joints_agree(stream_a, rmi_a)))

        if not args.move:
            print(rule("result"))
            print("    One pose only. This reports the offset; it cannot tell whether that")
            print("    offset tracks J2 or is a fixed calibration offset. Re-run with --move.")
            return verdict("verify_j2j3_coupling", checks, driver)

        # -- second reading, at a materially different J2 ----------------------
        q_now = np.asarray(driver.get_state()["q_cmd"], dtype=float)[:NDOF]
        q_target = q_now.copy()
        q_target[1] += np.radians(args.j2_delta_deg)
        lo = np.minimum(degrees(q_now), degrees(q_target))
        hi = np.maximum(degrees(q_now), degrees(q_target))
        if not guard_joint_limits(lo, hi, [1]):
            checks.append(("J2 move stays inside the soft limits", False))
            return verdict("verify_j2j3_coupling", checks, driver)

        print(rule(f"moving J2 by {args.j2_delta_deg:+.1f} deg"))
        res = driver.move_j(q_target, joint_speed=np.radians(args.joint_speed_deg_s)).wait(timeout=60.0)
        print(f"    move_j -> {res.value}")
        checks.append((f"J2 move completed ({res.value})", res.value == "done"))
        if res.value != "done":
            return verdict("verify_j2j3_coupling", checks, driver)

        second = _read_both_planes(driver)
        if second is None:
            return verdict("verify_j2j3_coupling", checks, driver)
        stream_b, rmi_b = second
        off_b = _report_pose("pose B", stream_b, rmi_b)
        checks.append(("only J3 differs across the planes (pose B)", _other_joints_agree(stream_b, rmi_b)))

        separation = abs(stream_b[1] - stream_a[1])
        checks.append(
            (
                f"J2 separation {separation:.2f} deg exceeds {MIN_J2_SEPARATION_DEG:.0f} deg",
                separation >= MIN_J2_SEPARATION_DEG,
            )
        )

        print(rule("result"))
        which = _classify(stream_a[1], off_a, stream_b[1], off_b)
        print()
        if which == "tracks":
            print("    THE OFFSET TRACKS J2 — the coupled representation is in effect on the")
            print("    RMI plane. The conversion to the Stream Motion frame is q[2] += q[1].")
            print("    The conversion is already written, in the driver's single per-model")
            print("    point. To adopt it for THIS controller, enable it in your own")
            print("    configuration:")
            print("        from dataclasses import replace")
            print("        from airo_fanuc.controller_facts import INTERIM_FACTS")
            print("        facts = replace(INTERIM_FACTS, rmi_to_stream_j3_plus_j2_verified=True)")
        elif which == "fixed":
            print("    THE OFFSET IS FIXED, not a J2 coupling. Do NOT apply q[2] += q[1] —")
            print(f"    it is a constant {off_a:+.4f} deg, which is a calibration offset on this")
            print("    controller, not the interlinked representation. Leave the gate False;")
            print("    a J2-proportional conversion would be wrong everywhere except the pose")
            print("    it was measured at.")
        else:
            print("    NEITHER hypothesis fits. The offset moved, but not by -J2 and not by")
            print("    zero. Leave the gate False and read J3 on the pendant at both poses")
            print("    before trusting either plane.")
        print()
        print("    Nothing in the package was changed by this run, deliberately: what the")
        print("    controller in one cell does is not true of the next installation, so this")
        print("    is a setting for your configuration rather than a constant to compile in.")

        # Return J2 where it started, so the run leaves the cell as it found it.
        print(rule("returning J2"))
        back = driver.move_j(q_now, joint_speed=np.radians(args.joint_speed_deg_s)).wait(timeout=60.0)
        print(f"    move_j -> {back.value}")
        checks.append((f"J2 returned to start ({back.value})", back.value == "done"))
        checks.append(("the two readings support one hypothesis", which != "neither"))

        return verdict("verify_j2j3_coupling", checks, driver)
    except KeyboardInterrupt:
        print("\ninterrupted — stopping")
        if driver is not None:
            driver.stop_j()
            close_driver(driver)
        return 1
    finally:
        target.close()


if __name__ == "__main__":
    sys.exit(main())
