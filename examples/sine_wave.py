#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Sine-wave joint exercise for the FANUC CRX-10iA/L via ``airo_fanuc.FanucDriver``.

Every selected joint oscillates ``±amplitude`` degrees about its CURRENT pose as a
slow sine, for a whole number of cycles, so the trajectory both starts and ends at
the start pose. One `move_trajectory` (dense cubic-Hermite knots with analytic
velocities) — the C++ RT core interpolates + executes it.

  # Offline (no hardware):
  packages/airo_fanuc/.venv-dev/bin/python packages/airo_fanuc/examples/sine_wave.py --fake

  # Real controller (operator AT THE ROBOT, E-STOP in hand, area clear):
  packages/airo_fanuc/.venv-dev/bin/python packages/airo_fanuc/examples/sine_wave.py \
      --ip 192.168.1.100 --amplitude-deg 5 --period 10 --cycles 2

Defaults are deliberately gentle: ±5° at a 10 s period ⇒ peak joint speed ≈ 3.1°/s.
A joint-limit guard aborts (no motion) if any joint's start±amplitude would leave
the CRX-10iA/L soft limits.
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

# CRX-10iA/L active joint limits (deg), from docs/controller-notes.md §1.1 (P-1
# measured). Used only as a safety guard for this all-joints exercise.
_LIMIT_LOWER_DEG = np.array([-180.0, -180.0, -270.0, -190.0, -180.0, -225.0])
_LIMIT_UPPER_DEG = np.array([180.0, 180.0, 270.0, 190.0, 180.0, 225.0])


def _degrees(q_rad) -> list[float]:
    return np.round(np.degrees(np.asarray(q_rad, dtype=float)[:_NDOF]), 3).tolist()


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
    q_start = np.asarray(q_start_rad, dtype=float)[:_NDOF]
    q = np.tile(q_start, (len(ts), 1))
    qd = np.zeros((len(ts), _NDOF))
    for j in joint_idx:
        q[:, j] = q_start[j] + amp_rad * np.sin(omega * ts)
        qd[:, j] = amp_rad * omega * np.cos(omega * ts)
    times_ns = [int(round(t * 1e9)) for t in ts]
    return times_ns, q.tolist(), qd.tolist()


def main() -> int:
    ap = argparse.ArgumentParser(description="±deg sine-wave joint exercise via airo_fanuc.FanucDriver.")
    ap.add_argument("--ip", default="192.168.1.100", help="controller IP (real robot)")
    ap.add_argument("--fake", action="store_true", help="offline in-process FakeCRX (no hardware)")
    ap.add_argument("--joints", default="1,2,3,4,5,6", help="comma list of joints 1-6 (default all)")
    ap.add_argument("--amplitude-deg", type=float, default=5.0, help="sine amplitude, deg (default 5)")
    ap.add_argument("--period", type=float, default=10.0, help="seconds per cycle (default 10)")
    ap.add_argument("--cycles", type=float, default=2.0, help="number of full cycles (default 2)")
    ap.add_argument("--knot-dt", type=float, default=0.05, help="trajectory knot spacing, s (default 0.05)")
    ap.add_argument("--lock-path", default=None, help="ownership flock path")
    args = ap.parse_args()

    try:
        joint_idx = sorted({int(x) - 1 for x in args.joints.split(",") if x.strip()})
    except ValueError:
        ap.error("--joints must be a comma list of integers, e.g. 1,2,3,4,5,6")
    if not joint_idx or any(j < 0 or j >= _NDOF for j in joint_idx):
        ap.error("--joints entries must be in 1..6")
    if args.amplitude_deg <= 0 or args.period <= 0 or args.cycles <= 0 or args.knot_dt <= 0:
        ap.error("--amplitude-deg / --period / --cycles / --knot-dt must be > 0")
    amp_rad = math.radians(args.amplitude_deg)
    peak_speed = args.amplitude_deg * 2.0 * math.pi / args.period  # deg/s
    total_s = args.cycles * args.period

    # --- build config; offline FakeCRX for --fake ------------------------------
    controller = None
    if args.fake:
        from airo_fanuc.testing import FakeCRXConfig, FakeCRXController

        controller = FakeCRXController(FakeCRXConfig(available_version=3))
        controller.start()
        controller.start_realtime(speed=1.0)
        cfg = DriverConfig(sm_port=controller.sm_port, rmi_port=controller.rmi_port, sm_version=3)
        ip = "127.0.0.1"
        lock_path = str(Path(tempfile.gettempdir()) / "airo-fanuc-sine.lock")
    else:
        cfg = DriverConfig()
        ip = args.ip
        lock_path = None
    if args.lock_path is not None:
        lock_path = args.lock_path

    policy_kwargs = {"config": cfg, "enable_gripper": False, "connect_retries": 1}
    if lock_path is not None:
        policy_kwargs["lock_path"] = lock_path
    policy = DriverPolicy(**policy_kwargs)

    joints_1based = [j + 1 for j in joint_idx]
    if not args.fake:
        print("=" * 72)
        print("  REAL ROBOT MOVE — operator must be present, E-STOP in hand, area clear.")
        print(f"  Sine on J{joints_1based}: ±{args.amplitude_deg:.1f}° about the current pose,")
        print(f"  {args.cycles:g} cycle(s) @ {args.period:.1f}s, peak ~{peak_speed:.1f} deg/s.")
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
            return 4

        if not _wait_streaming(driver):
            print("  driver did not reach stable streaming — aborting the move (no motion issued).")
            return 3

        times_ns, q, qd = _build_sine(q_start, joint_idx, amp_rad, args.period, args.cycles, args.knot_dt)
        print(f"running sine: {len(times_ns)} knots, ~{total_s:.0f}s, peak ≈ {peak_speed:.1f}°/s ...")
        result = driver.move_trajectory(times_ns, q, qd).wait(timeout=total_s + 10.0)
        print(f"  result: {result}")
        print("  joints now (deg):", _degrees(driver.get_state()["q_meas"]))
        return 0 if result == MotionResult.DONE else 2
    finally:
        driver.close()
        if controller is not None:
            controller.stop_realtime()
            controller.close()


if __name__ == "__main__":
    sys.exit(main())
