#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify the recorded joint position limits against the actual robot. NO MOTION.

WRITTEN FOR ONE SPECIFIC ROBOT: a FANUC CRX-10iA/L. The table it checks against is
``_common.LIMIT_LOWER_DEG`` / ``LIMIT_UPPER_DEG``, measured on our controller and
recorded in ``docs/controller-notes.md`` §1.1.

WHY THIS IS OPERATOR-DRIVEN. Those limits are not read from the robot — they were
measured once and written down, and the pre-motion guard in ``sine_wave.py`` /
``move_joints.py`` is the only thing between an amplitude argument and a soft-limit
hit. A table that is too WIDE means the guard passes a command the controller will
refuse; too narrow only costs range. So they are worth confirming. There is no
automated way to do it here:

* this package's RMI client exposes registers, status, error and joint angles — no
  system-variable read, so ``$MOR_GRP[1].$LOWERLIMS`` and friends are out of reach;
* driving toward a limit to find it deliberately provokes the servo alarm the guard
  exists to prevent.

So YOU move the arm and this script watches. It commands nothing: it opens a
**connect-only** RMI session (never ``FRC_Initialize`` — per controller-notes §1.6
that locks the motion group and disables hand-guidance, which is exactly the motion
you need) and polls ``FRC_ReadJointAngles`` at 15 Hz, recording each joint's extremes.
§1.6 measured that path: 450/450 reads at 15.0 Hz during hand-guided motion, all
distinct, TimeTag monotonic — the reads follow the arm rather than latching.

Usage — hand-guide the arm, taking each joint you care about to its stop, then Ctrl-C:

  python examples/check_joint_limits.py --ip 192.168.1.100

It also records J2 at each J3 extreme, because RMI reports J3 one J2 below the Stream
Motion value (§1.5, measured at a single J2). Reading only the RMI plane, this script
cannot confirm that offset — it records the pair. Comparing the reported J3 against the
pendant's displayed J3 at the same pose does settle a different open question: which
plane the pendant shows.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from _common import LIMIT_LOWER_DEG, LIMIT_MARGIN_DEG, LIMIT_UPPER_DEG, NDOF, rule
from airo_fanuc.exceptions import FanucError
from airo_fanuc.ownership import OwnershipLock
from airo_fanuc.rmi_client import RmiClient


def main() -> int:
    ap = argparse.ArgumentParser(description="Confirm the recorded joint limits by hand-guiding. No motion.")
    ap.add_argument("--ip", default="192.168.1.100", help="controller IP")
    ap.add_argument(
        "--tolerance-deg",
        type=float,
        default=2.0,
        help="how close to the recorded limit counts as confirming it (default %(default)s)",
    )
    ap.add_argument(
        "--hz", type=float, default=15.0, help="poll rate (default %(default)s, the measured max)"
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop and report after this many seconds (default 0 = run until Ctrl-C)",
    )
    ap.add_argument("--lock-path", default=None, help="ownership flock path (default: the package's)")
    args = ap.parse_args()

    print("=" * 76)
    print("  READ-ONLY. This script commands NO motion — you move the arm.")
    print("  Hand-guide each joint you want to check to its stop (both directions).")
    print("  Every joint's extremes are recorded live. Ctrl-C when you are done.")
    print("=" * 76)

    lock_kwargs = {"path": args.lock_path} if args.lock_path is not None else {}
    lock = OwnershipLock("receive", **lock_kwargs)  # type: ignore[arg-type]
    rmi = RmiClient(args.ip, 16001, connect_timeout=5.0, request_timeout=2.0)
    try:
        lock.acquire()
    except FanucError as exc:
        print(f"\nownership: {exc}")
        return 2
    try:
        # Connect-only. NEVER initialize() here — see the module docstring.
        rmi.start()
    except Exception as exc:
        print(f"\nRMI connect FAILED: {type(exc).__name__}: {exc}")
        lock.release()
        return 2

    lo = np.full(NDOF, np.inf)
    hi = np.full(NDOF, -np.inf)
    pose_at_lo = [np.zeros(NDOF) for _ in range(NDOF)]
    pose_at_hi = [np.zeros(NDOF) for _ in range(NDOF)]
    reads = 0
    failures = 0
    period = 1.0 / max(args.hz, 1.0)
    last_print = 0.0

    stop_at = time.monotonic() + args.duration if args.duration > 0 else None
    how_it_ends = f"for {args.duration:g}s" if stop_at else "until Ctrl-C"
    print(f"\npolling FRC_ReadJointAngles at {args.hz:g} Hz — move the arm now, {how_it_ends}\n")
    try:
        while stop_at is None or time.monotonic() < stop_at:
            t0 = time.monotonic()
            try:
                joints = np.asarray(rmi.read_joint_angles(), dtype=float)[:NDOF]
                reads += 1
            except Exception as exc:  # noqa: BLE001 — a read failure must not end the session
                failures += 1
                if failures <= 3:
                    print(f"  read failed ({failures}): {type(exc).__name__}: {exc}")
                time.sleep(period)
                continue

            for j in range(NDOF):
                if joints[j] < lo[j]:
                    lo[j] = joints[j]
                    pose_at_lo[j] = joints.copy()
                if joints[j] > hi[j]:
                    hi[j] = joints[j]
                    pose_at_hi[j] = joints.copy()

            if t0 - last_print >= 0.5:
                last_print = t0
                span = " ".join(f"J{j + 1}[{lo[j]:+7.1f},{hi[j]:+7.1f}]" for j in range(NDOF))
                print(f"  {span}  ({reads} reads)")

            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\n  (stopped)")
    finally:
        rmi.stop()
        lock.release()

    # ----------------------------------------------------------------- report
    print(rule("observed vs recorded"))
    if reads == 0:
        print("  no successful reads — nothing to compare")
        return 2
    print(f"  {reads} reads, {failures} failures")
    print(f"  tolerance {args.tolerance_deg:g}°: an extreme within that of the recorded limit confirms it\n")
    header = (
        f"  {'':4} {'observed lo':>12} {'recorded':>9} {'verdict':<14}"
        f"{'observed hi':>12} {'recorded':>9} verdict"
    )
    print(header)

    wider: list[str] = []
    for j in range(NDOF):
        obs_lo, obs_hi = lo[j], hi[j]
        rec_lo, rec_hi = LIMIT_LOWER_DEG[j], LIMIT_UPPER_DEG[j]

        def classify(observed: float, recorded: float, *, is_lower: bool, joint: int = j) -> str:
            beyond = observed < recorded if is_lower else observed > recorded
            if beyond:
                wider.append(
                    f"J{joint + 1} {'lower' if is_lower else 'upper'}: reached {observed:+.2f}°, "
                    f"recorded {recorded:+.0f}°"
                )
                return "BEYOND"
            gap = abs(observed - recorded)
            return "CONFIRMED" if gap <= args.tolerance_deg else f"{gap:.0f}° short"

        print(
            f"  J{j + 1:<3} {obs_lo:>12.2f} {rec_lo:>9.0f} {classify(obs_lo, rec_lo, is_lower=True):<14}"
            f"{obs_hi:>12.2f} {rec_hi:>9.0f} {classify(obs_hi, rec_hi, is_lower=False)}"
        )

    print(rule("what to do with this"))
    print("  A 'short' verdict is AMBIGUOUS and only you can resolve it: either you did not")
    print("  take that joint to its stop, or the recorded limit is too wide — and too wide is")
    print("  the direction that matters, because the guard would then pass a command the")
    print("  controller refuses. If the joint WAS at its stop, narrow the table in")
    print(f"  examples/_common.py (the guard already keeps {LIMIT_MARGIN_DEG:g}° back from it).")
    if wider:
        print("\n  These went BEYOND the recorded limit — the table is conservative there, which is")
        print("  safe but costs range:")
        for line in wider:
            print(f"    {line}")

    # J2/J3: the RMI→stream J3 offset is measured at one J2 only (§1.5).
    print(rule("J2/J3 representation (docs/controller-notes.md §1.5, one J2 measured)"))
    print("  MEASURED (§1.5): RMI reports J3 one J2 BELOW the Stream Motion value,")
    print("  so the RMI→stream conversion is J3 += J2. This plane is the only one read here,")
    print("  so this run cannot confirm it; it records the pair for a run that reads BOTH")
    print("  planes at one pose, at a materially different J2:")
    for label, pose, j3 in (("J3 min", pose_at_lo[2], lo[2]), ("J3 max", pose_at_hi[2], hi[2])):
        print(
            f"    {label}: RMI J3 = {j3:+8.3f}°, J2 = {pose[1]:+8.3f}° "
            f"→ stream-frame J3 = {j3 + pose[1]:+8.3f}°"
        )
    print("  Still open: WHICH plane matches the pendant's displayed J3. One glance settles it.")

    return 2 if wider else 0


if __name__ == "__main__":
    sys.exit(main())
