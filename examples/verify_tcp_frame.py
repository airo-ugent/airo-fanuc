#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Check the tool offset the controller is applying. NO MOTION by default.

WRITTEN FOR ONE SPECIFIC ROBOT: a FANUC CRX-10iA/L on an R-30iB-class controller
negotiating Stream Motion v3 at an 8 ms interpolation period. See
``examples/README.md`` before running it on anything else.

The driver exposes the arm's Cartesian pose twice, because the controller reports it
twice and the two are **not the same point**:

    get_flange_pose()  Stream Motion status packet, 125 Hz, non-blocking  -> FACEPLATE
    get_tcp_pose()     FRC_ReadCartesianPosition, one RMI round trip      -> TOOL TIP

That split was measured, not assumed (``docs/controller-notes.md`` §1.10): at one
standstill pose the orientations were bit-identical and the positions differed by
exactly 175.000 mm along tool +Z — this cell's Robotiq gripper. The streamed pose does
not carry the tool; the RMI read does, with the controller doing the arithmetic.

This run reads both at one stationary pose and reports what separates them:

    a pure translation, no rotation → a TOOL offset; the number is what the
                                      controller has in its UTOOL entry
    identical                       → the controller is applying no tool at all
    orientation differs too         → NOT just a tool: suspect a different UFRAME,
                                      and stop trusting either pose until you know

Run it after changing an end effector, after anyone touches UTOOL/UFRAME on the
pendant, or whenever a grasp lands off by a suspiciously round number. No motion is
needed, and by default this script commands none:

  # Offline — no hardware, exercises the comparison against an in-process FakeCRX:
  python examples/verify_tcp_frame.py --fake

  # Real controller, read-only. NOTHING MOVES:
  python examples/verify_tcp_frame.py --ip 192.168.1.100

  # Assert a specific tool length instead of just reporting it:
  python examples/verify_tcp_frame.py --ip 192.168.1.100 --expect-tool-mm 175

  # Add the stronger check: move one joint a little and re-read both planes, which
  # also proves the poses track the arm instead of being constants.
  # Operator AT THE ROBOT, E-STOP in hand, area clear:
  python examples/verify_tcp_frame.py --ip 192.168.1.100 --move --joint 6 --delta-deg 10

What it cannot tell you: whether the active tool is the tool you think it is. It
reports which tool NUMBER the controller named and what offset it is applying, not
what is entered in the other nine. Read the pendant's SETUP → Frames page for that.
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
    rule,
    verdict,
    wait_streaming,
    watch,
)
from airo_fanuc import FanucDriver, FanucError, MotionResult

#: Names of the six Cartesian components, in wire order.
_AXES = ("X", "Y", "Z", "W", "P", "R")

#: A pose of exactly zero is not something this arm can hold — it would put the TCP
#: at the world origin, inside the base. Both planes reporting it means neither is
#: reporting a pose, and two non-answers agreeing is not evidence of anything.
_ZERO_POSE_EPS = 1e-6


class PairedRead:
    """One Stream Motion pose + one RMI pose, read at the same stationary pose.

    The two planes are sampled at completely different rates — Stream Motion at
    125 Hz, RMI as a request/reply round trip of tens of milliseconds — so they are
    only comparable while the arm is standing still. This brackets the RMI read
    with an SM read on each side and keeps both: if the arm moved during the round
    trip, the bracket shows it and :attr:`stationary` goes False, which invalidates
    the comparison rather than quietly skewing it.
    """

    def __init__(self, driver: FanucDriver, *, bracket_tol_mm: float, bracket_tol_deg: float) -> None:
        sm_before = driver.get_flange_pose()
        self.rmi = driver.rmi.read_cartesian_position()
        sm_after = driver.get_flange_pose()

        self.sm = sm_after
        self.sm_before = sm_before
        self.q_meas_deg = degrees(driver.get_state()["q_meas"])

        if sm_before is None or sm_after is None:
            self.stationary = False
            self.bracket_drift = None
        else:
            drift = np.abs(sm_after - sm_before)
            self.bracket_drift = drift
            self.stationary = bool(
                np.all(drift[:3] <= bracket_tol_mm) and np.all(drift[3:] <= bracket_tol_deg)
            )

    @property
    def rmi_pose(self) -> np.ndarray:
        return np.asarray(self.rmi.xyzwpr[:NDOF], dtype=float)

    def delta(self) -> np.ndarray | None:
        """``sm − rmi`` per component, or ``None`` if Stream Motion gave no pose."""
        if self.sm is None:
            return None
        return self.sm - self.rmi_pose

    def is_all_zero(self) -> bool:
        if self.sm is None:
            return True
        return bool(
            np.all(np.abs(self.sm) < _ZERO_POSE_EPS) and np.all(np.abs(self.rmi_pose) < _ZERO_POSE_EPS)
        )

    def report(self, label: str, *, tol_mm: float, tol_deg: float) -> bool:
        """Print both poses and what separates them; return True iff that separation is
        a pure tool translation (the expected state — see the module docstring)."""
        print(rule(label))
        print(f"  joints (deg)  : {self.q_meas_deg}")
        if self.sm is None:
            print("  flange (SM)   : NO POSE — get_flange_pose() returned None (no status packet yet)")
            return False

        print(f"  {'':14}{'  '.join(f'{a:>10}' for a in _AXES)}")
        print(f"  flange (SM)   : {'  '.join(f'{v:10.3f}' for v in self.sm)}")
        print(f"  tcp (RMI)     : {'  '.join(f'{v:10.3f}' for v in self.rmi_pose)}")
        delta = self.delta()
        assert delta is not None
        print(f"  tcp - flange  : {'  '.join(f'{-v:10.3f}' for v in delta)}   (mm | deg)")
        print(
            f"  rmi says      : UFRAME {self.rmi.uframe_number}, UTOOL {self.rmi.utool_number}, "
            f"config F={self.rmi.front} U={self.rmi.up} L={self.rmi.left} "
            f"FLIP={self.rmi.flip} turn={self.rmi.turn}"
        )
        if self.bracket_drift is not None:
            worst_mm = float(np.max(self.bracket_drift[:3]))
            worst_deg = float(np.max(self.bracket_drift[3:]))
            print(
                f"  bracket drift : {worst_mm:.4f} mm / {worst_deg:.4f} deg across the RMI round trip"
                + ("" if self.stationary else "  <-- ARM MOVED; this pair is not comparable")
            )
        return _report_separation(-delta, tol_mm=tol_mm, tol_deg=tol_deg)


def _report_separation(tcp_minus_flange: np.ndarray, *, tol_mm: float, tol_deg: float) -> bool:
    """Identify what separates the two planes, quantitatively. True iff it is a tool.

    A rigid tool moves the reported point without rotating it, so the discriminator is
    simply: does the orientation match while the position does not? An orientation
    difference means something more than a tool is in play — a rotated tool, or two
    different user frames — and either way the poses stop meaning what their names say
    until someone reads the pendant.
    """
    xyz = tcp_minus_flange[:3]
    wpr_off = float(np.max(np.abs(tcp_minus_flange[3:])))
    dist = float(np.linalg.norm(xyz))

    if dist <= tol_mm and wpr_off <= tol_deg:
        print(
            f"  -> IDENTICAL within {tol_mm} mm / {tol_deg} deg: the controller is applying NO tool "
            "offset (UTOOL is empty, or the RMI session's tool is the faceplate). get_tcp_pose() "
            "and get_flange_pose() will return the same thing."
        )
        return True

    if wpr_off > tol_deg:
        print(
            f"  -> NOT A TOOL OFFSET: orientation differs by up to {wpr_off:.3f} deg as well as "
            f"position ({dist:.3f} mm). A tool with a rotation would do this, but so would the two "
            "planes using different user frames (UFRAME) — read UFRAME on the pendant before "
            "trusting either pose."
        )
        return False

    print(
        f"  -> TOOL OFFSET: orientation identical ({wpr_off:.3f} deg), position differs by "
        f"{dist:.4f} mm — a pure translation, so the planes differ by a TOOL and not by a frame."
    )
    print(
        "     Cross-check that length against the UTOOL entry on the pendant "
        "(SETUP -> Frames): the tool number RMI named above should hold it."
    )
    return True


def _build_trajectory(q_start_rad, joint_idx: int, delta_rad: float, duration_s: float):
    """Rest-to-rest 2-knot trajectory moving one joint by ``delta_rad`` (see move_joints.py)."""
    q0 = [float(x) for x in np.asarray(q_start_rad, dtype=float)[:NDOF]]
    q1 = list(q0)
    q1[joint_idx] += delta_rad
    times = [0, int(duration_s * 1e9)]
    qd = [[0.0] * NDOF, [0.0] * NDOF]
    return times, [q0, q1], qd


def main() -> int:
    ap = argparse.ArgumentParser(description="Check the tool offset the controller is applying.")
    add_connection_args(ap)
    ap.add_argument(
        "--move",
        action="store_true",
        help="also move one joint and re-read both planes (proves the poses track the arm). "
        "Without this, NOTHING MOVES.",
    )
    ap.add_argument("--joint", type=int, default=6, help="--move: joint to move, 1-6 (default 6)")
    ap.add_argument("--delta-deg", type=float, default=10.0, help="--move: degrees (default 10)")
    ap.add_argument("--duration", type=float, default=4.0, help="--move: seconds (default 4)")
    ap.add_argument(
        "--expect-tool-mm",
        type=float,
        default=None,
        help="assert the tool offset is this long, in mm (e.g. 175 for this cell's Robotiq "
        "gripper). Omitted, the run reports the offset it measured without judging it.",
    )
    ap.add_argument(
        "--tol-mm",
        type=float,
        default=0.5,
        help="tolerance on X/Y/Z, mm (default %(default)s) — the threshold for calling two poses "
        "the same point, and for --expect-tool-mm.",
    )
    ap.add_argument(
        "--tol-deg", type=float, default=0.1, help="tolerance on W/P/R, deg (default %(default)s)"
    )
    args = ap.parse_args()

    if not (1 <= args.joint <= NDOF):
        ap.error("--joint must be in 1..6")
    jidx = args.joint - 1

    target = open_target(args)
    policy = build_policy(target)

    if target.is_fake:
        print(
            "NOTE --fake: the fake applies an INJECTED tool offset to its RMI reply, mirroring\n"
            "     the split measured on the real controller. It computes no kinematics, so this\n"
            "     run proves the comparison works; only the real controller reports what tool it\n"
            "     is actually applying.\n"
        )
    else:
        banner = (
            [
                "READ-ONLY — no commanded motion. The driver brings up, holds, and reads.",
                "Both planes are read at the pose the arm is already in.",
            ]
            if not args.move
            else [
                "REAL ROBOT MOVE — operator must be present, E-STOP in hand, area clear.",
                f"Reads both planes, moves J{args.joint} by {args.delta_deg:+.1f} deg "
                f"over {args.duration:.1f} s, reads again.",
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

        settled = wait_streaming(driver)
        checks.append(("settled into stable streaming", settled))
        if not settled:
            print("  never reached stable streaming — see the fault above.")
            return verdict("verify_tcp_frame", checks, driver)

        # The comparison is only valid at a standstill (see PairedRead), and the pose
        # is only meaningful once a status packet has landed.
        driver.wait_until_steady(timeout=5.0)
        if target.is_fake:
            # The fake reports whatever is injected; give it a pose plausible for a
            # CRX-10iA/L plus this cell's gripper, so the report reads like a real one.
            target.controller.set_cartesian([612.5, -128.25, 431.75, -178.5, 2.25, 91.125])
            target.controller.set_cartesian_tcp([608.055, -135.207, 256.945, -178.5, 2.25, 91.125])
            time.sleep(0.1)  # let a few 8 ms status packets carry it

        # --- pose A: the read that does the work, no motion required -------------
        pose_a = PairedRead(driver, bracket_tol_mm=args.tol_mm, bracket_tol_deg=args.tol_deg)
        is_tool = pose_a.report("pose A (as found)", tol_mm=args.tol_mm, tol_deg=args.tol_deg)
        checks.append(("pose A: the two planes were read at a standstill", pose_a.stationary))
        checks.append(("pose A: what separates the two planes is a tool offset, not a frame", is_tool))
        # Two planes both reporting zeros would "agree" while telling us nothing.
        checks.append(("pose A: the reported pose is a real pose, not all zeros", not pose_a.is_all_zero()))

        # get_tcp_pose() must BE the controller's tool tip, not a local derivation of it.
        tcp = driver.get_tcp_pose()
        offset_mm = (
            float(np.linalg.norm(pose_a.rmi_pose[:3] - pose_a.sm[:3])) if pose_a.sm is not None else 0.0
        )
        print(rule("what the driver's two getters return"))
        print(f"  get_flange_pose() : {None if pose_a.sm is None else np.round(pose_a.sm, 3).tolist()}")
        print(f"  get_tcp_pose()    : {None if tcp is None else np.round(tcp, 3).tolist()}")
        print(f"  they differ by    : {offset_mm:.4f} mm  (the tool the controller is applying)")
        checks.append(
            (
                "get_tcp_pose() returns the controller's tool tip",
                tcp is not None and bool(np.allclose(tcp, pose_a.rmi_pose, atol=args.tol_mm)),
            )
        )

        if args.expect_tool_mm is not None:
            matches = abs(offset_mm - args.expect_tool_mm) <= args.tol_mm
            print(
                f"  expected tool     : {args.expect_tool_mm:.3f} mm — measured {offset_mm:.4f} mm "
                f"({'match' if matches else 'MISMATCH'})"
            )
            checks.append((f"the tool offset is {args.expect_tool_mm:.3f} mm as expected", matches))

        # FRC_GetStatus reports the active frame numbers too; the two RMI replies
        # disagreeing would mean the Configuration block cannot be trusted as the tag.
        status = driver.rmi.get_status()
        print(rule("active frame numbers, cross-checked across two RMI replies"))
        print(
            f"  FRC_ReadCartesianPosition : UFRAME {pose_a.rmi.uframe_number}, "
            f"UTOOL {pose_a.rmi.utool_number}"
        )
        print(f"  FRC_GetStatus             : UFRAME {status.number_uframe}, UTOOL {status.number_utool}")
        frames_consistent = (
            status.number_uframe == pose_a.rmi.uframe_number
            and status.number_utool == pose_a.rmi.utool_number
        )
        if not frames_consistent:
            print(
                "  the two replies name DIFFERENT active frames — so 'the active tool' is not one\n"
                "  number on this controller, and the pose above is tagged by the Cartesian read.\n"
                "  Measured on this controller (docs/controller-notes.md §1.10a): open question."
            )
        checks.append(("both RMI replies report the same active UFRAME/UTOOL", frames_consistent))

        # --- pose B: optional, and the part that needs motion -------------------
        if args.move:
            q_start = np.asarray(driver.get_state()["q_cmd"], dtype=float)[:NDOF]
            start_deg = np.degrees(q_start)
            end_deg = start_deg.copy()
            end_deg[jidx] += args.delta_deg
            safe = guard_joint_limits(np.minimum(start_deg, end_deg), np.maximum(start_deg, end_deg), [jidx])
            checks.append((f"J{args.joint} move stays inside the soft limits", safe))
            if not safe:
                return verdict("verify_tcp_frame", checks, driver)

            times, q, qd = _build_trajectory(q_start, jidx, math.radians(args.delta_deg), args.duration)
            print(rule(f"moving J{args.joint} by {args.delta_deg:+.1f} deg over {args.duration:.1f}s"))
            w = watch(driver, driver.move_trajectory(times, q, qd), timeout_s=args.duration + 5.0)
            moved = report_motion(w, expect_result=MotionResult.DONE)
            checks.append((f"J{args.joint} move returned DONE", moved))
            if not moved:
                return verdict("verify_tcp_frame", checks, driver)

            driver.wait_until_steady(timeout=5.0)
            if target.is_fake:
                # The fake does no FK, so its pose does not follow the plant. Inject a
                # different one, or the tracking check below would fail for a reason
                # that says nothing about the driver.
                target.controller.set_cartesian([601.25, -140.5, 428.5, -177.0, 3.75, 101.25])
                target.controller.set_cartesian_tcp([596.805, -147.457, 253.695, -177.0, 3.75, 101.25])
                time.sleep(0.1)

            pose_b = PairedRead(driver, bracket_tol_mm=args.tol_mm, bracket_tol_deg=args.tol_deg)
            b_is_tool = pose_b.report("pose B (after the move)", tol_mm=args.tol_mm, tol_deg=args.tol_deg)
            checks.append(("pose B: the two planes were read at a standstill", pose_b.stationary))
            checks.append(("pose B: the separation is still a tool offset", b_is_tool))

            # The offset must be the SAME tool at both poses. A number that changes with
            # the arm's configuration is not a tool offset, whatever pose A suggested.
            if pose_a.sm is not None and pose_b.sm is not None:
                offset_b = float(np.linalg.norm(pose_b.rmi_pose[:3] - pose_b.sm[:3]))
                print(rule("is it the same tool at both poses?"))
                print(f"  pose A offset : {offset_mm:.4f} mm")
                print(f"  pose B offset : {offset_b:.4f} mm")
                same_tool = abs(offset_b - offset_mm) <= args.tol_mm
                if not same_tool:
                    print(
                        "  the offset CHANGED between poses: a fixed tool cannot do that. Suspect "
                        "the two planes disagreeing about the frame rather than about the tool."
                    )
                checks.append(("the tool offset is the same at both poses", same_tool))

                # Not-frozen: a constant field would have looked consistent at pose A too.
                travel = np.abs(pose_b.sm - pose_a.sm)
                print(rule("did the poses follow the arm?"))
                print(f"  A -> B change : {'  '.join(f'{v:10.3f}' for v in travel)}   (mm | deg)")
                print(
                    f"  worst position change {float(np.max(travel[:3])):.3f} mm, "
                    f"worst orientation change {float(np.max(travel[3:])):.3f} deg"
                )
                tracked = bool(np.any(travel[:3] > args.tol_mm) or np.any(travel[3:] > args.tol_deg))
                if not tracked:
                    print(
                        "  the pose did not change: a frozen or unpopulated field would look "
                        "exactly like this, and would have looked consistent at pose A as well."
                    )
                elif float(np.max(travel[:3])) <= args.tol_mm:
                    print(
                        f"  orientation moved, position did not: expected for a J{args.joint} "
                        "rotation whose axis passes through the reported point. Re-run with a "
                        "joint that translates the tool (e.g. --joint 1) to exercise X/Y/Z."
                    )
                checks.append(("the streamed pose changed when the arm moved", tracked))

        # --- what this run establishes -------------------------------------------
        print(rule("conclusion"))
        if is_tool and not pose_a.is_all_zero():
            print(
                f"  The controller is applying a {offset_mm:.3f} mm tool offset, and it reports the\n"
                f"  tool tip over RMI (UFRAME {pose_a.rmi.uframe_number}, UTOOL "
                f"{pose_a.rmi.utool_number}) while streaming the faceplate over Stream Motion.\n"
                f"\n"
                f"  So: get_tcp_pose() is the tool tip, get_flange_pose() is {offset_mm:.3f} mm short\n"
                f"  of it, and that is the documented split (docs/controller-notes.md §1.10).\n"
                f"  Confirm the number against the UTOOL entry on the pendant if this is the first\n"
                f"  run after changing an end effector."
            )
            if not frames_consistent:
                print(
                    f"\n  CAVEAT: the two RMI replies named different active frames "
                    f"(Cartesian read UFRAME {pose_a.rmi.uframe_number}/UTOOL "
                    f"{pose_a.rmi.utool_number}, GetStatus UFRAME {status.number_uframe}/UTOOL "
                    f"{status.number_utool}).\n"
                    "  The pose above is the one the Cartesian read named. §1.10a is open on\n"
                    "  whether the streamed faceplate follows the pendant's active tool."
                )
        else:
            print(
                "  NOT a clean tool offset. The numbers above are the evidence: read them next to\n"
                "  the pendant's SETUP -> Frames page (UFRAME and UTOOL entries) before trusting\n"
                "  either get_tcp_pose() or get_flange_pose() to mean what its name says."
            )

        return verdict("verify_tcp_frame", checks, driver)
    except KeyboardInterrupt:
        print("\ninterrupted — braking, then shutting down")
        driver.stop_j()
        checks.append(("interrupted by the operator", False))
        return verdict("verify_tcp_frame", checks, driver)
    finally:
        close_driver(driver)
        target.close()


if __name__ == "__main__":
    sys.exit(main())
