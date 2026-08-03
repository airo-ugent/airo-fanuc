<!-- SPDX-License-Identifier: Apache-2.0 -->
# Binding invariants — `airo_fanuc` driver

The safety and motion invariants this driver holds. Each one is here because
something concrete breaks without it, and the breakage is stated inline. Treat
them as binding: code that regresses one of these is a bug, not a refactor.

## The wire

- **The CommandPacket's reserved `u16` at offset 14 is `0xFFFF`, and nothing else.**
  The vendored struct names the field `unused`; it is not unused. `0xFFFF` tells the
  controller the payload is joint angles. Any other value — 0 included, which is what
  a zero-initialised struct gives you — makes it read the same twelve floats as a
  Cartesian XYZWPR pose and slew the arm at full speed toward whatever pose that
  implies. The constant lives in exactly two places, `airo_fanuc.controller_facts`
  and `src/cpp/codec/codec.cpp`, and the wire goldens pin the encoded bytes.
- **Internal units are radians; degrees exist only at the wire boundary.** The
  conversion, the 9-slot zero padding for a 6-DOF arm and the `0xFFFF` selector are
  all in the codec, so no motion code above it ever handles a degree.
- **Timestamps are absolute int64 ns; doubles are for differences only.**

## Motion

- **Every motion goes through `FanucDriver.move_trajectory`, `move_j` or `servo_j`.**
  Construct the driver once (`FanucDriver(ip, policy)` — construct-and-go, blocks
  until commandable or raises `FanucConnectionError`/`FanucPreflightError` with the
  triage/preflight report), then call `move_trajectory(...)` (blocking via
  `MotionHandle.wait(...)`, or `asynchronous=True`) / `move_j(q, joint_speed=...)` /
  `servo_j(q, dt)`. `move_j` is not a fourth path into the core: it plans a
  jerk-limited profile offline and submits the knots through `move_trajectory`, so it
  inherits the same validation, capture gate, collision hook and settle policy.
- **`stop_j()` is the universal preempt.** Any thread, ≤1 tick, never raises,
  reachable in every mode. Every watchdog / manual-STOP / fault path converges here.
  A `stop_j()` during a blocking `MotionHandle.wait()` resolves it as
  `MotionResult.STOPPED` — the brake+replan contract. `stop_j()` is a clean preempt,
  **not** a fault: it does not bump the epoch and does not arm the ARM gate. A mode
  in which the request is accepted and then dropped is a regression, because the
  caller's handle would report success for a motion that never stopped.
- **Collision response is brake+replan.** A trajectory monitor on the caller's side
  calls `stop_j()`, waits for the handle to resolve `STOPPED`, then replans from rest
  and issues a new `move_trajectory`. There is deliberately no graded slowdown and no
  mid-flight trajectory→trajectory swap: both would need a second planner inside the
  driver, and planning is the caller's.
- **The brake is planned from the COMMANDED state, never the measured one.** The seed
  is `q_cmd`/`qd_cmd` plus the interpolator's analytic second derivative at the brake
  tick. Measured velocity lags the command, so seeding from it starts the decel at a
  velocity the controller is not being asked for — the brake then injects the very
  discontinuity it exists to remove. Measured velocity is a termination test only.
  Seeding acceleration to zero is the same mistake: a brake can be entered
  mid-segment where `|qdd|` peaks, and the CRX contact-stop monitor infers contact
  force from motor disturbance torque, so an acceleration step reads as a phantom
  contact.
- **The brake's clamp split is real: `stop_scale_va × (v, a)` and `stop_scale_j × j`.**
  Acceleration and jerk are scaled by different factors on purpose, and both are
  fractions of the injected profile rather than absolute numbers, so they carry
  across arms unchanged. A stop path that picks its duration without consulting them
  is not honouring the envelope, whatever its duration happens to be.
- **No direct `driver.core` submissions in application code.** `driver.core`,
  `driver.rmi`, `driver.gripper` are reach-through attributes for diagnostics/extra
  functionality only. Calling `driver.core.submit_trajectory(...)` /
  `submit_servo(...)` / `stop_j()` directly bypasses validation, the ARM gate and
  the CAPTURE collision-check hook — a regression.
- **The ARM gate latches when the fault is observed, not when recovery succeeds.**
  An e-stop or a latched controller alarm sets `MOTION_INHIBITED` the moment the
  supervisor sees it, and motion methods raise `RobotFaultedError` until an explicit
  `driver.arm()`. Latching on the tail of a successful recovery instead leaves the
  gate clear on every path where the recovery ladder returns early, and the
  cold-reconnect escalation then produces a commandable robot with no `arm()` ever
  asked for. Nothing clears the flag but `arm()`, and nothing may call `arm()` inside
  a retry loop — that moves the robot with the operator at the pendant.
  `FanucDriver.recover()` returning True therefore does **not** imply commandable.

## Collision-check hook (CAPTURE)

- **The checked path IS the executed path.** `move_trajectory` synthesizes the exact
  CAPTURE splice the RT core will execute via `airo_fanuc._core.generate_capture_path`
  (one C++ code path) and hands it to `policy.capture_check` when set. The hook is
  supplied by the caller and returns `True` iff the splice is collision-free.
  **The `airo_fanuc` wheel never imports a collision-checking or kinematics
  library** — collision-checking is the caller's responsibility, and the runtime
  dependency set stays numpy-only so the wheel installs standalone.

## Bring-up

- **The configured interpolation period must match the controller's own.** Every
  per-tick quantity in the core is scaled by `config.itp_s` — the slew clip, the brake
  and settle windows, each mode's Ruckig period — so a driver configured for one
  period against a controller running another produces limits wrong by that ratio, and
  permissively wrong if the real period is shorter. Bring-up compares against the
  period the controller states in its GetCapability reply and refuses rather than
  adapting. A reported 0 means no reply was seen and is not treated as a mismatch: a
  controller that never completed the handshake fails earlier and more informatively.
- **`GRPRUN` is forked at most once per `bringup()`, and only after a liveliness
  probe.** `GRIPDISP` is a dispatcher loop living on controller flash, and `GRPRUN` is
  a one-line launcher that RUN-forks it as an independent task. The fork therefore
  outlives the RMI session, so a second one stacks a second dispatcher on the same
  trigger register and both act on every command. The probe is what makes a
  re-bring-up safe: it detects a prior process's surviving fork and adopts it instead
  of forking again. The one-shot budget covers all of a `bringup()`'s retries, not one
  attempt.
- **A bring-up is not complete until `motion_possible` HOLDS.** Re-calling
  `STREAM_MOTN` drops `motion_possible` for about a second, and the drop lands after
  the preroll reports ready — so an assert-once check passes and the robot faults
  immediately afterwards. The settle requires the flag to stay asserted for a window,
  which is what makes the difference between a bring-up that reports success and one
  that has actually succeeded.

## Faults, getters, timestamps

- **Getters never raise and never lie.** `get_state()` / `get_wrench()` /
  `joints_at_wall()` / `get_flange_pose()` / `timing_stats()` always return (value +
  age); `get_wrench()` returns `None` when force telemetry is unavailable rather than
  fabricating zeros.
- **`get_tcp_pose()` is the one getter that BLOCKS**, and deliberately so. The Stream
  Motion stream carries the *faceplate*, not the tool tip (measured — 175 mm apart on
  this cell, `controller-notes.md` §1.10), so the tool tip can only come from the
  controller's own `FRC_ReadCartesianPosition`: one RMI round trip, tens of ms, off the
  125 Hz timeline. That keeps the tool definition where it belongs — in the control box,
  not duplicated in driver config that could drift out of sync with the pendant. It
  returns `None` on any RMI failure and **never substitutes the faceplate**: silently
  answering with a point one tool-length away would be exactly the lie these getters
  forbid. Do not call it per tick — a caller needing a TCP at tick rate applies its own
  tool transform to `get_flange_pose()`. **The wheel ships no pose algebra**, for the
  same reason it ships no kinematics: the controller already does this conversion, and a
  second copy of the tool definition in driver code is a copy that can drift from the
  pendant.
- **`airo_fanuc.lifecycle` is the only classifier.** Mode plus fault reason maps to a
  lifecycle state in one place. A second copy of that mapping anywhere else — a
  republish path, a report formatter — is two answers to one question, and they diverge
  silently because only one of them is tested.
- **A published field is either assembled or absent.** A key that is declared and then
  always `None` is not a placeholder, it is a false promise in a message a consumer may
  be gating on.

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
- **`SUPERVISOR_LOST` is a live, in-process, C++ RT-core fault.** The supervisor
  beats a dedicated heartbeat thread (`Supervisor._heartbeat_loop`); a lapse
  faults→HOLD. Do not remove the heartbeat: it is the only fast reaction to a wedged
  supervisor, since the controller's own backstop is the slow coast above.
- **Commanded↔measured divergence is the controller's to detect, not the driver's.**
  There is deliberately no host-side divergence guard. An arm that stops following the
  stream accrues position error until the controller's deviation monitor hard-stops it
  and drops `motion_possible` / raises `in_error` — which the C++ tick reacts to within
  one ITP, on the same gate path as an e-stop. That monitor is measured (71–121 ms,
  overrun 2.10° @ 15.3 °/s / 4.63° @ 49.9 °/s — `controller-notes.md` §1.2), whereas a
  host-side guard has to model the command→report offset to tell divergence from
  ordinary lag, and that offset is itself unsettled (§1.9a). Anything reintroduced here
  must be lag-aligned against a *measured* offset, not against `tracking_lag_s`.

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
- **The 125 Hz loop stays in C++, and nothing on the tick path allocates, locks or
  logs.** State reaches Python through a seqlock snapshot and SPSC rings; the event
  ring drops if full rather than blocking the producer. A Python-side tick would put
  the interpreter lock and the garbage collector on the 8 ms deadline.
- **The library installs no signal handler and never exits the process.** `close()` is
  poison-not-exit: timed thread joins, then abandon plus a typed `FanucError` on a
  wedge. Signal handling and any hard-exit watchdog belong to the application entry
  point — a handler here would run arbitrary library code on an arbitrary thread, and
  logging from one is not async-signal-safe.
