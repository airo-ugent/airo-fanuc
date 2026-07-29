<!-- SPDX-License-Identifier: Apache-2.0 -->
# Binding invariants — `airo_fanuc` driver

The safety and motion invariants this driver holds. Each one is here because
something concrete breaks without it, and the breakage is stated inline. Treat
them as binding: code that regresses one of these is a bug, not a refactor.

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
- **Collision response is brake+replan.** A trajectory monitor on the caller's side
  calls `stop_j()`, waits for the handle to resolve `STOPPED`, then replans from rest
  and issues a new `move_trajectory`. There is no graded slowdown and no mid-flight
  trajectory→trajectory swap in v1.
- **No direct `driver.core` submissions in application code.** `driver.core`,
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
  (one C++ code path) and hands it to `policy.capture_check` when set. The hook is
  supplied by the caller and returns `True` iff the splice is collision-free.
  **The `airo_fanuc` wheel never imports a collision-checking or kinematics
  library** — collision-checking is the caller's responsibility, and the runtime
  dependency set stays numpy-only so the wheel installs standalone.

## Faults, getters, timestamps

- **Getters never raise and never lie.** `get_state()` / `get_wrench()` /
  `joints_at_wall()` / `timing_stats()` always return (value + age); `get_wrench()`
  returns `None` when force telemetry is unavailable rather than fabricating zeros.
- **Timestamps are absolute int64 ns; doubles are for differences only.**

## Force / grasping — this controller has no force telemetry

- **This controller streams Stream Motion v3 / type-202 — no force block** (proven
  on the wire, `controller-notes.md` §1.8). `get_wrench()` is always `None` and the
  C++ force-guard has no signal.
- **Grasps terminate on contact-stop, NOT `force_stop_n`.** The collaborative
  contact-stop (`contact_stop_status` ∈ {2,4} / `safety_scale` drop) IS carried in
  type-202; on contact the RT core kills the motion — it resolves **`FAULTED` with
  `fault_reason == CONTACT_STOP`** and drops to SAFE_FOLLOW (a *transient* DEGRADED
  fault: auto-recovers after the dwell, **no `arm()`**). Grasp logic therefore reads
  `fault_reason == CONTACT_STOP` (via `get_state()`) as "contacted the object", then
  recovers + closes the gripper + lifts. (Only the armed `force_stop_n` guard
  resolves the cleaner `STOPPED`, and that path needs a v4 / type-204 controller.)
- **`move_trajectory(force_stop_n=...)` is REJECTED** (typed `TrajectoryValidationError`)
  when no force telemetry is available — arming an inert force-guard would be a silent
  safety no-op. A numeric force threshold is only meaningful on a v4 / type-204
  controller (P84 firmware). Do not add a Python-side force watchdog either: with no
  force signal to read it is the same silent no-op, one layer up.

## Host-death & liveness watchdogs

- **The controller's TX-silence backstop is a bounded, deviation-triggered coast of
  ~120 ms**, not a fast decel: on command silence it holds the last commanded
  velocity until position error accrues, then hard-stops (measured overrun 2.10° at
  15.3 °/s, 4.63° at 49.9 °/s — `controller-notes.md` §1.2). It is the accepted
  host-death backstop, and there is deliberately **no external RMI-abort watchdog**:
  a watchdog process on the same host dies *with* the host, and is slower than the
  coast even when it survives. Any hard-exit / stop-on-exit belongs to the
  application entry point, not the library.
- **`SUPERVISOR_LOST` and `DRIFT` are live, in-process, C++ RT-core faults.** The
  supervisor beats a dedicated heartbeat thread (`Supervisor._heartbeat_loop`); a
  lapse faults→HOLD. The drift guard faults on sustained commanded↔measured
  divergence (the 22°-runaway guard, lag-aligned to the measured 25 ms servo lag).
  Do not remove the heartbeat or the guard: they are the only fast reaction to a
  wedged supervisor or a diverging robot, since the controller's own backstop is the
  slow coast above.

## Backend / process hygiene

- **The republish path is output-only and its sink is injected.** The `Republisher`
  publishes state (`fanuc/joint_states`, `fanuc/robot_status`,
  `fanuc/external_force`, `fanuc/ext_status`, plus the typed log events) through a
  duck-typed `Publisher` handed in by the hosting process; the wheel itself imports
  no messaging library. The driver never *accepts* commands over a topic — motion
  enters only through the `FanucDriver` API, so a published message cannot bypass
  validation, the ARM gate or the CAPTURE hook.
- **Exactly one process may own the controller.** The controller serves exactly one
  Stream Motion peer and one RMI session: a second Stream Motion peer receives no
  status at all and a foreign StopPacket silences the live session
  (`controller-notes.md` §1.8), while a second RMI connect leaves the redirect port
  timing out (§1.4). The driver therefore takes a single-owner flock and fails
  loudly (`OwnershipError`) on contention rather than racing another owner.
- **The library never exits the process.** `close()` is poison-not-exit: timed
  thread joins, then abandon + typed `FanucError` on a wedge. Any hard-exit watchdog
  and all signal handling belong to the application entry point, not the library.
