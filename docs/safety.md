# Safety, preconditions and limitations

This driver commands a 10 kg-payload industrial arm at 125 Hz. Read this before the first
bring-up, and read [invariants.md](invariants.md) before changing anything in the motion
path.

---

## Before you connect

- **Controller option S636** (J519 Stream Motion + R912 RMI). `preflight_full=True` reads it
  off the controller rather than taking your word for it.
- **Controller in AUTO**, drives powered, E-stop released, alarms reset, general override at
  100%. T1/T2 is a soft DEGRADED warning, not a hard block — but nothing will move.
- **Nothing else talking to the controller.** One Stream Motion peer per controller is a
  hardware-level constraint, not a policy choice: a second peer receives no status at all, a
  foreign Stop packet silences the live session mid-motion, and a second RMI connect leaves
  the redirect port timing out. The driver takes an advisory `flock` and fails loudly with
  `OwnershipError` naming the holder rather than racing it.
- **A gripper dispatcher on controller flash** if `enable_gripper` — otherwise
  `DriverPolicy(enable_gripper=False)`. See [gripper.md](gripper.md).
- **An operator at the robot with the E-stop in hand**, for every step of a first bring-up.

The per-connect preflight gate runs on every bring-up regardless of `preflight_full`: AUTO
mode, servo ready, drives powered, general override, the DCS speed clamp, and alarm
classification. Two alarm classes are hard blocks with operator instructions rather than
warnings, because neither clears by retrying.

---

## The fault model

Every fault is latched in C++, drives the core to a bounded-follow mode, and stays latched
until a recovery ladder clears it *and* the underlying condition has been clear for the
anti-flap dwell.

Faults split two ways, and the difference is who decelerates the arm:

**DEGRADED** — the controller is handling the stop itself; the driver waits it out.

| Fault | What it is | To resume |
|---|---|---|
| `CONTACT_STOP` | the collaborative contact-stop monitor tripped | clear it on the pendant, move the arm off the obstacle |
| `TEACH_MODE` | keyswitch in T1/T2 | return it to AUTO |
| `SAFETY_CLAMP` | a residual DCS speed clamp | press RESET on the pendant |

**FAULTED** — needs the RMI recovery ladder.

| Fault | To resume |
|---|---|
| `E_STOP` | release the E-stop, RESET on the pendant if it persists — **then `arm()`** |
| `IN_ERROR` | RESET on the pendant — **then `arm()`** |
| `MOTION_NOT_POSSIBLE` | check drives and active alarms |
| `RX_SILENT` / `RX_DEGRADED` | check the network link and host load; a silent stream can need a controller power-cycle |
| `WATCHDOG_EXPIRED` | your supervisor stopped calling `kick()` while a `deadman_s` was armed |
| `FORCE_GUARD` | clear the contact and replan |
| `REJECTED_START_MISMATCH` | the submitted first knot was out of the capture window — replan from where the arm is |
| `SUPERVISOR_LOST` | the Python supervisor thread died. **The core is holding on its own** — restart the process |
| `INTERNAL` | a driver bug. Capture `get_state()` and `timing_stats()` and report it |

`get_state()["operator_hint"]` carries the actionable instruction for whatever is currently
latched, and it is what a UI should surface.

### The ARM gate

**After an E-stop, a latched controller alarm, or an operator-required condition, motion
stays refused until you explicitly call `driver.arm()`.**

The gate is set the moment the driver *observes* the fault, not when recovery finishes. So:

- `driver.recover()` returning `True` means **"back to STREAMING"**, not "commandable".
- Motion methods keep raising `RobotFaultedError` after a successful recovery until `arm()`.
- **Nothing may call `arm()` inside a retry loop.** The whole point is that an E-stop was
  pressed for a reason and somebody may be standing in the envelope. Re-arming is a human
  decision, expressed once, deliberately.

`DriverPolicy(arm_gate=False)` disables it. Do not, unless the cell has some other
interlock that makes an unattended resume safe.

### Recovery after an E-stop takes the long path, always

Two ways exist in principle to resume a Stream Motion session after an E-stop: continue the
existing session, or tear it down and re-handshake. **This driver always does the latter.**

Whether the fast path works on this controller class is **unverified** — the attempt to
measure it was inconclusive, twice, because the status stream was down at the time — so the
safe path is the only one implemented. There is no knob to select the other one.

---

## What stops the arm, and what each one is

| | What it is | What it is not |
|---|---|---|
| `stop_j()` | the driver's brake: a limit-respecting deceleration on the 8 ms path, taking effect within one tick | not an E-stop, not the controller's backstop |
| `hold()` | brake to rest and hold, resolving the motion `PREEMPTED` | — |
| `force_stop_n` | **inert on a controller that streams no force block, and refused with a typed error rather than silently armed** | not a substitute for the contact-stop monitor |
| `deadman_s` + `kick()` | your supervisor's liveness, enforced in C++ | not a network watchdog |
| contact stop | the controller's own collaborative monitor. The motion resolves `FAULTED` with `CONTACT_STOP` | not a numeric force threshold you set |
| the E-stop | the only actual emergency stop | — |

The important one: **on a v3 controller there is no force telemetry at all**, so contact
detection is the controller's collaborative stop and nothing else. A `force_stop_n` that
cannot be evaluated is a typed rejection at submission, never a guard that quietly never
trips.

---

## Known limitations

**Out-of-limit joint targets are handled differently by the three entry points.** This is
worth knowing exactly:

| | Behaviour |
|---|---|
| `move_j(q)` | **rejected** synchronously, `TrajectoryValidationError` naming the joints and both limits |
| `move_trajectory(times, q, qd)` | position limits are **not validated in Python**. The core clamps every commanded position into the profile's limits each tick, before the slew clip, so nothing out of range reaches the wire — but the clamp is silent |
| `servo_j(q, ...)` | the target is **clamped** into the limits by the servo planner |

Nothing lets an out-of-limit command reach the controller. But a `move_trajectory` whose
knots leave the envelope executes a *clamped* path, which is not the path you submitted and
not the path your collision check saw, with no counter reporting it. If your planner can
produce out-of-envelope knots, validate them yourself against
`config.profile.position_limits_*` before submitting.

**Velocity, by contrast, is refused.** `|qd|` above the profile's velocity limits is a typed
error at submission.

**A servo stream never ends by itself.** Stop feeding it and the core holds the last target
in SERVO indefinitely. `is_steady()` stays `False`, and a `move_j` will then refuse because
the arm is not at rest. `hold()` is the way out.

**`stop_j()` does not stop a servo loop.** It preempts every target submitted before it; a
target submitted after is accepted normally, so a loop that keeps feeding restarts the arm a
tick or two after the brake. Stop the loop too.

**There is no collision check on the servo path.** `capture_check` is called for
`move_trajectory` and `move_j`; `servo_j` has no hook anywhere. Setpoint sanity is the
caller's.

**Recovery from a collision-induced controller SystemFault is a different path** from the
E-stop drill and can leave RMI unresponsive, forcing the cold-reconnect escalation.

**The gripper path is not covered by the validation ladder** — every example runs with the
gripper disabled.

**`SETTLE_TIMEOUT` is not a fault.** It means the commanded timeline finished and the
measured joints never confirmed arrival within the settle window. The arm may be stalled
against something, or the tolerance may simply be tight for the speed. Nothing faulted.

---

## Invariants you should not regress

The full set with the one-line symptom of breaking each is in
[invariants.md](invariants.md). The ones a *caller* can trip over:

- **The collision-checked path is the executed path.** The capture splice is synthesized
  once, in C++, and Python's pre-check calls the same function with the same inputs.
  Anything that reimplements it in Python breaks the guarantee silently.
- **`stop_j()` always wins**, including against a command submitted in the same 8 ms
  window.
- **The universal preempt is a clean stop, not a fault** — so a caller can replan from rest
  rather than running a recovery ladder.
- **Motion is inhibited after an E-stop until explicit re-arm.** Nothing auto-re-arms.
- **Getters never raise and never lie.** Every value that can be stale is published with
  its age.
- **One Stream Motion peer per controller**, enforced with an advisory lock.
- **The 125 Hz loop stays in C++**, and the RT thread never calls into Python.

---

See also: [invariants.md](invariants.md) for the full list,
[troubleshooting](troubleshooting.md) for what a given symptom usually means,
`docs/controller-notes.md` for measured controller behaviour and recovery procedures, and
[`examples/README.md`](../examples/README.md) for the ordered bring-up ladder.
