#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Minimal example: drive the FANUC CRX-10iA/L with ``airo_fanuc.FanucDriver``.

Moves ONE joint by a small delta (and optionally back), then reports the result.
Runnable two ways:

  # Offline — no hardware. Spins up an in-process FakeCRX and drives it:
  packages/airo_fanuc/.venv-dev/bin/python packages/airo_fanuc/examples/move_joints.py --fake

  # Real controller — operator AT THE ROBOT, E-STOP in hand, workspace clear:
  packages/airo_fanuc/.venv-dev/bin/python packages/airo_fanuc/examples/move_joints.py \
      --ip 192.168.1.100 --joint 6 --delta-deg 10 --duration 4 --return

The driver is ur_rtde-shaped; this is the joint-space slice of it. Mapping onto the
airo-mono ``PositionManipulator`` ABC (the full ABC — TCP-pose moves + IK/FK — is the
P9 milestone and needs an injected kinematics provider, since the wheel is numpy-only):

    get_joint_configuration()        -> driver.get_state()["q_meas"]   (rad)
    move_to_joint_configuration(q)   -> driver.move_trajectory(times, q, qd)   (this file)
    servo_to_joint_configuration(q)  -> driver.servo_j(q, dt)
    (protective) stop                -> driver.stop_j()                (universal preempt)
"""

from __future__ import annotations

import argparse
import math
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from airo_fanuc import DriverConfig, DriverPolicy, FanucDriver, MotionResult

_NDOF = 6


def _degrees(q_rad) -> list[float]:
    return np.round(np.degrees(np.asarray(q_rad, dtype=float)[:_NDOF]), 3).tolist()


def _wait_streaming(driver, hold_s: float = 2.0, timeout_s: float = 10.0) -> bool:
    """Wait until the driver is STABLY streaming (mode=streaming, fault=none held for
    ``hold_s``). A re-bring-up over a running STREAM_MOTN can show a brief
    motion_possible transient that auto-recovers; ride it out before commanding."""
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


def _build_trajectory(q_start_rad, joint_idx: int, delta_rad: float, duration_s: float):
    """A rest-to-rest 2-knot trajectory that moves one joint by ``delta_rad``.

    ``times`` are int64 ns-relative; ``q`` starts at the current commanded pose (so
    the CAPTURE splice is a no-op) and ends with the single joint offset; ``qd`` is
    zero at both ends (the C++ core cubic-Hermite-interpolates the rest-to-rest move).
    """
    q0 = [float(x) for x in np.asarray(q_start_rad, dtype=float)[:_NDOF]]
    q1 = list(q0)
    q1[joint_idx] += delta_rad
    times = [0, int(duration_s * 1e9)]
    qd = [[0.0] * _NDOF, [0.0] * _NDOF]
    return times, [q0, q1], qd


def main() -> int:
    ap = argparse.ArgumentParser(description="Move one FANUC joint a bit via airo_fanuc.FanucDriver.")
    ap.add_argument("--ip", default="192.168.1.100", help="controller IP (real robot)")
    ap.add_argument("--fake", action="store_true", help="offline in-process FakeCRX (no hardware)")
    ap.add_argument("--joint", type=int, default=6, help="joint to move, 1-6 (default 6 = wrist roll)")
    ap.add_argument("--delta-deg", type=float, default=10.0, help="relative move, degrees (default 10)")
    ap.add_argument("--duration", type=float, default=4.0, help="move duration, seconds (default 4)")
    ap.add_argument("--return", dest="return_", action="store_true", help="move back to start afterward")
    ap.add_argument("--no-move", action="store_true", help="bring up + stream only; no motion")
    ap.add_argument("--observe", type=float, default=5.0, help="--no-move: seconds to watch")
    ap.add_argument("--lock-path", default=None, help="ownership flock path")
    args = ap.parse_args()

    if not (1 <= args.joint <= _NDOF):
        ap.error("--joint must be in 1..6")
    jidx = args.joint - 1
    delta_rad = math.radians(args.delta_deg)

    # --- build the driver config; spin up an offline FakeCRX for --fake ----------
    controller = None
    if args.fake:
        from airo_fanuc.testing import FakeCRXConfig, FakeCRXController

        controller = FakeCRXController(FakeCRXConfig(available_version=3))  # v3/type-202, like the real one
        controller.start()
        controller.start_realtime(speed=1.0)  # stream status at the ITP
        cfg = DriverConfig(sm_port=controller.sm_port, rmi_port=controller.rmi_port, sm_version=3)
        ip = "127.0.0.1"
        lock_path = str(Path(tempfile.gettempdir()) / "airo-fanuc-example.lock")
    else:
        cfg = DriverConfig()  # default Stream Motion (60015) + RMI (16001) ports
        ip = args.ip
        lock_path = None

    if args.lock_path is not None:
        lock_path = args.lock_path

    # joint move only — no gripper; single bring-up attempt (clean signal, no churn).
    policy_kwargs = {"config": cfg, "enable_gripper": False, "connect_retries": 1}
    if lock_path is not None:
        policy_kwargs["lock_path"] = lock_path
    policy = DriverPolicy(**policy_kwargs)

    # --- safety gate on real hardware --------------------------------------------
    if not args.fake:
        print("=" * 72)
        if args.no_move:
            print("  BRING-UP ONLY — no commanded motion. The driver will hold the current pose.")
            print(f"  It streams status + holds for {args.observe:.0f} s, then closes.")
        else:
            print("  REAL ROBOT MOVE — operator must be present, E-STOP in hand, area clear.")
            print(f"  About to move J{args.joint} by {args.delta_deg:+.1f} deg over {args.duration:.1f} s.")
        print("  Ctrl-C now to abort; starting in 3 s ...")
        print("=" * 72)
        try:
            time.sleep(3.0)
        except KeyboardInterrupt:
            print("aborted before connect")
            return 1

    print(f"connecting to {ip} (construct-and-go: blocks until commandable or raises) ...")
    driver = FanucDriver(ip, policy)
    try:
        q_start = np.asarray(driver.get_state()["q_cmd"], dtype=float)[:_NDOF]
        print("commandable. current joints (deg):", _degrees(q_start))

        # --- no-motion bring-up validation: stream state, confirm no fault, hold ----
        if args.no_move:
            print(f"bring-up OK. streaming state for {args.observe:.0f} s (NO commanded motion) ...")
            deadline = time.monotonic() + args.observe
            faulted = False
            while time.monotonic() < deadline:
                st = driver.get_state()
                print(
                    f"  mode={st.get('lifecycle_state')} fault={st.get('fault_reason')} "
                    f"rx_age_ms={st.get('rx_age_ms', 0.0):.1f} wrench={driver.get_wrench()} "
                    f"joints_deg={_degrees(st['q_meas'])}"
                )
                if str(st.get("fault_reason") or "none").lower() != "none":
                    faulted = True
                time.sleep(1.0)
            print("done — no commanded motion was issued." + ("  (a fault was observed!)" if faulted else ""))
            return 2 if faulted else 0

        # Ride out any post-bring-up motion_possible transient before commanding.
        if not _wait_streaming(driver):
            print("  driver did not reach stable streaming — aborting the move (no motion issued).")
            return 3
        times, q, qd = _build_trajectory(q_start, jidx, delta_rad, args.duration)
        print(f"moving J{args.joint} by {args.delta_deg:+.1f} deg ...")
        result = driver.move_trajectory(times, q, qd).wait(timeout=args.duration + 5.0)
        print(f"  result: {result}")
        print("  joints now (deg):", _degrees(driver.get_state()["q_meas"]))

        if args.return_ and result == MotionResult.DONE:
            times, q, qd = _build_trajectory(driver.get_state()["q_cmd"], jidx, -delta_rad, args.duration)
            print("returning to start ...")
            back = driver.move_trajectory(times, q, qd).wait(timeout=args.duration + 5.0)
            print(f"  result: {back}")
            print("  joints now (deg):", _degrees(driver.get_state()["q_meas"]))

        return 0 if result == MotionResult.DONE else 2
    finally:
        driver.close()  # poison-not-exit: timed joins + StopPacket
        if controller is not None:
            controller.stop_realtime()
            controller.close()


if __name__ == "__main__":
    sys.exit(main())
