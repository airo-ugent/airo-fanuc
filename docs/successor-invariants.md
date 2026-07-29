<!-- SPDX-License-Identifier: Apache-2.0 -->
# Successor CLAUDE.md invariants — `airo_fanuc` driver

Draft of the invariants that become the successor `CLAUDE.md` section when the
grocery demo cuts over to `airo_fanuc.FanucDriver` (PLAN.md §7, ships **with** P5).
These encode time already paid; treat them as binding once the cutover lands.

## Motion

- **Every motion goes through `FanucDriver.move_trajectory` or `FanucDriver.servo_j`.**
  Construct the driver once (`FanucDriver(ip, policy)` — construct-and-go, blocks
  until commandable or raises `FanucConnectionError`/`FanucPreflightError` with the
  triage/preflight report), then call `move_trajectory(...)` (blocking via
  `MotionHandle.wait(...)`, or `asynchronous=True`) / `servo_j(q, dt)`.
- **`stop_j()` is the universal preempt.** Any thread, ≤1 tick, never raises. Every
  watchdog / manual-STOP / fault path converges here. A `stop_j()` during a blocking
  `MotionHandle.wait()` resolves it as `MotionResult.STOPPED` — the brake+replan
  contract. `stop_j()` is a clean preempt, **not** a fault: it does not bump the
  epoch and does not arm the ARM gate.
- **Collision response is brake+replan.** `TrajectoryMonitor` calls `stop_j()`,
  waits for the handle to resolve `STOPPED`, then replans from rest and issues a new
  `move_trajectory`. There is no graded slowdown and no mid-flight trajectory→
  trajectory swap in v1.
- **No direct `driver.core` submissions in demo code.** `driver.core`,
  `driver.rmi`, `driver.gripper` are reach-through attributes for diagnostics/extra
  functionality only. Calling `driver.core.submit_trajectory(...)` /
  `submit_servo(...)` / `stop_j()` directly bypasses validation, the ARM gate and
  the CAPTURE collision-check hook — a regression.
- **The ARM gate is real.** After an e-stop or an OPERATOR_REQUIRED (SYST-348)
  recovery, the driver ends in `MOTION_INHIBITED`: motion methods raise
  `RobotFaultedError` until an explicit `driver.arm()`. Do not auto-`arm()` in a
  retry loop — that would move the robot with the operator at the pendant.

## Collision-check hook (CAPTURE)

- **The checked path IS the executed path.** `move_trajectory` synthesizes the exact
  CAPTURE splice the RT core will execute via `airo_fanuc._core.generate_capture_path`
  (one C++ code path) and hands it to `policy.capture_check` when set. Grocery wires
  `capture_check` to curobo's `check_trajectory_collision` (returning `True` iff the
  splice is collision-free). **The `airo_fanuc` wheel never imports curobo** —
  collision-checking is the caller's responsibility; the wheel stays numpy-only.

## Faults, getters, timestamps

- **Getters never raise and never lie.** `get_state()` / `get_wrench()` /
  `joints_at_wall()` / `timing_stats()` always return (value + age); `get_wrench()`
  returns `None` when force telemetry is unavailable rather than fabricating zeros.
- **Timestamps are absolute int64 ns; doubles are for differences only.**

## Force / grasping (P-1 measured: this controller has NO force telemetry)

- **This controller streams Stream Motion v3 / type-202 — no force block** (proven
  on the wire, `controller-notes.md` §1.8). `get_wrench()` is always `None` and the
  C++ force-guard has no signal.
- **Grasps terminate on contact-stop, NOT `force_stop_n`.** The collaborative
  contact-stop (`contact_stop_status` ∈ {2,4} / `safety_scale` drop) IS carried in
  type-202; on contact the RT core kills the motion — it resolves **`FAULTED` with
  `fault_reason == CONTACT_STOP`** and drops to SAFE_FOLLOW (a *transient* DEGRADED
  fault: auto-recovers after the dwell, **no `arm()`**). The grasp logic reads
  `fault_reason == CONTACT_STOP` (via `get_state()`) as "contacted the object",
  exactly as the dries executor's `CONTACT_STOP` fault did — then recover +
  close-gripper + lift. (Only the armed `force_stop_n` guard resolves the cleaner
  `STOPPED`, and that path needs a v4 / type-204 controller.)
- **`move_trajectory(force_stop_n=...)` is REJECTED** (typed `TrajectoryValidationError`)
  when no force telemetry is available — arming an inert force-guard would be a silent
  safety no-op. A numeric force threshold returns only with a v4 / type-204 controller
  (P84 firmware). Do not re-introduce a Python force watchdog.

## Host-death & liveness watchdogs (P-1 E6 NO-GO)

- **The controller's TX-silence backstop is a bounded ~120 ms coast**, not a fast
  decel (E6). It is the accepted host-death backstop; there is deliberately **no
  external RMI-abort watchdog** (a same-host process dies with the host and is
  slower than the coast). Any hard-exit / stop-on-exit belongs to the app entry
  point, not the library.
- **`SUPERVISOR_LOST` and `DRIFT` are live, in-process, C++ RT-core faults.** The
  supervisor beats a dedicated heartbeat thread (`Supervisor._heartbeat_loop`); a
  lapse faults→HOLD. The drift guard faults on sustained commanded↔measured
  divergence (the 22°-runaway guard, lag-aligned to the measured 25 ms). Both were
  defined-but-unset before P-1; do not remove the heartbeat or the guard.

## Backend / process hygiene

- **Chunk topics (`fanuc/joint_chunk`, `fanuc/joint_setpoint_published`) and the
  gripper Zenoh topics are dead.** `fanuc/joint_states` / `robot_status` /
  `external_force` / `ext_status` survive via the driver-host `Republisher`.
  Importing `MotionExecutor` / `MotionExecutorClient` or publishing a chunk topic
  is a regression.
- **Never run the legacy `grocery-fanuc-bridge` systemd unit alongside the driver
  backend.** The controller accepts exactly one Stream Motion peer and one RMI
  session; the driver acquires the single-owner flock and fails loudly
  (`OwnershipError`) on contention. After cutover the unit is `systemctl disable`d
  (kept on disk = the rollback step).
- **The library never exits the process.** `close()` is poison-not-exit: timed
  thread joins, then abandon + typed `FanucError` on a wedge. Any hard-exit watchdog
  and all signal handling belong to the application entry point, not the library.
