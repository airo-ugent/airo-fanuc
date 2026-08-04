# Driving your own gripper

A FANUC controller has no gripper API. What it has is numeric registers and the ability
to run a Teach-Pendant program, so this driver actuates a gripper the only way RMI
allows: it writes registers, and a TP program you install watches them and does the work.

That indirection is what makes the mechanism general. The driver never learns what is
bolted to the flange — only which registers a dispatcher is watching and which values it
understands. The three parts are:

| Part | Lives | Who writes it |
|---|---|---|
| `GripperWorker` | this package | us |
| `RegisterGripperProtocol` | this package, as data | you, for your gripper |
| the dispatcher TP program | controller flash | **you** |

The last row is the important one. **The TP program is not in this repository and cannot
be** — it lives on the controller's flash, it is edited on the pendant or loaded over
FTP, and it is a site-installation prerequisite rather than a Python dependency. `pip
install` cannot supply it. Preflight refuses a gripper-enabled session when it is
missing, which is the earliest anything can notice.

---

## 1. What the driver gives you

`FanucDriver` exposes the worker as `driver.gripper`, built at bring-up when
`DriverPolicy(enable_gripper=True)` (the default). With `enable_gripper=False`,
`driver.gripper` is `None` and none of the below exists.

```python
driver.gripper.open_gripper()                    # submit, returns immediately
driver.gripper.close_gripper(close_force=0)      # submit, returns immediately
result = driver.gripper.wait_gripper_done()      # block for the outcome

driver.gripper.close_gripper_and_wait()          # the two above, in one call
```

| Method | Blocks | Returns |
|---|---|---|
| `open_gripper(open_state=None)` | no | `None` |
| `close_gripper(close_force=None)` | no | `None` |
| `wait_gripper_done(timeout=None)` | yes | `{"success": bool, "message": str}`, or `None` |
| `is_gripper_done()` | no | `bool` |
| `last_result` | no | the latest completed command's dict, or `None` if none has completed since the last submit |
| `protocol` | no | the `RegisterGripperProtocol` this worker executes |
| `open_gripper_and_wait(open_state=None, timeout=None)` | yes | as `wait_gripper_done` |
| `close_gripper_and_wait(close_force=None, timeout=None)` | yes | as `wait_gripper_done` |
| `close()` | no | `None` |

`open_state` / `close_force` are **modifier values your dispatcher defines** — a bucket
index, not millimetres and not newtons (see §2 for why the mechanism cannot carry a
continuous value). `None` means the protocol's default. A value the protocol does not
list raises `ValueError` synchronously, before anything reaches the wire: the dispatcher
has no way to say "I did not understand that", so an unrecognised modifier would be
actuated as whatever its `else` branch does.

### The completion contract

A submitted command has exactly three outcomes, and they are deliberately three:

- `{"success": True, ...}` — the dispatcher cleared the trigger.
- `{"success": False, "message": ...}` — the worker reached a verdict: a register access
  failed, the dispatcher never cleared the trigger within its window, recovery was in
  progress, or the worker is shutting down. The message says which.
- `None` — **your wait** timed out; the worker is still running. Distinct from a
  `{"success": False}` on purpose, because "I stopped waiting" and "the gripper failed"
  call for different responses.

`success: True` means *the dispatcher said it finished*. It does not mean a width was
reached or an object is held — nothing in this mechanism can tell you that.

`last_result` is the same dict read without waiting for it, and its `None` is the third
thing again but from the other side: **no verdict yet**, because a submit clears it and the
command has not finished. It never hands back the previous command's verdict.

### The never-hang guarantee, and its numbers

Every path through the worker ends in exactly one verdict, and each is bounded:

| Bound | Default | Why it exists |
|---|---|---|
| `dispatch_timeout_s` | 5.0 s | The poll loop runs against a monotonic deadline. A dispatcher that never clears the trigger produces a timeout verdict, never a hang. **Set it on your protocol if your tool strokes for longer than this**, or every command reports a spurious timeout — bring-up builds the worker, so there is no later moment to change it. |
| `trigger_settle_s` | 0.1 s | Delay before the *first* poll. Without it the first read can observe the pre-trigger 0 and report done instantly. |
| `poll_hz` | 20 Hz | Poll cadence (0.05 s period). |
| `open_gripper_and_wait` / `close_gripper_and_wait`, `timeout=None` | 6.1 s | `dispatch_timeout_s + trigger_settle_s + 1 s`, so the wait outlives the worker's own bound and captures its verdict instead of returning a premature `None`. |
| `FanucDriver.close()`'s gripper step | 2.0 s | A wedged worker cannot hang driver shutdown; the step is abandoned and reported. |

`wait_gripper_done(timeout=None)` is the exception: it waits **indefinitely**. The worker's own
bound still applies, so a dead dispatcher wakes it with a verdict rather than hanging it — but
if you want the wait itself bounded, pass a timeout.

Two behaviours worth knowing before you build on this:

- **Serialized.** One physical gripper, so one worker thread. Concurrent `open`/`close`
  calls are ordered, never interleaved on the wire — but only the *latest* command's
  result is retrievable.
- **Refused during recovery.** The supervisor gates the worker while a recovery ladder is
  running, and a command then fails fast rather than queueing behind a 5 s poll loop that
  would hold the RMI path the ladder needs for its `FRC_Reset`. This is also why a
  gripper command right after an e-stop release can come back `{"success": False}`: retry
  once the driver is streaming again.

---

## 2. The contract your TP program must satisfy

This is the specification. Anything meeting it works with this driver.

### The registers

Three numeric registers, whose numbers you choose:

| Role | Written by | Read by | Meaning |
|---|---|---|---|
| **trigger** | driver sets it to `1` | dispatcher, then **cleared to 0** by the dispatcher | `1` = a command is pending or in progress; `0` = idle/finished |
| **action** | driver | dispatcher | which verb — one value for open, one for close |
| **modifier** | driver | dispatcher | the verb's single argument |

The split of verb from argument is not decoration: it is what lets your dispatcher read a
complete command in one scan.

### The handshake

The driver, per command:

1. writes the **modifier**,
2. writes the **action**,
3. writes `1` to the **trigger** — last, and only once the other two are settled,
4. waits `trigger_settle_s`,
5. polls the **trigger** until it reads `0`, or the dispatch timeout elapses.

Your dispatcher must:

1. **Loop forever**, polling the trigger register. Not run once and exit — the driver
   forks it at bring-up and expects it to still be there for every later command.
2. On seeing the trigger set, **read the action and modifier registers on that same
   scan**. They are guaranteed settled: the trigger is written last, specifically so a
   dispatcher acting the instant it sees the trigger cannot read half a command.
3. **Dispatch on the action value** and use the modifier as that verb's argument.
4. **Clear the trigger to 0 — and only after the physical motion has finished.** This is
   the whole completion signal. Clearing it early reports success before the gripper has
   moved, and every caller believes it.
5. **Never write the action or modifier registers, and never set the trigger.** The
   driver is the only writer of those, and the only setter of the trigger; your program is
   the only thing that clears it. That one-way split is what makes the handshake
   unambiguous with no lock.

If your dispatcher never clears the trigger, the driver reports a timeout naming your
dispatcher and the register that stayed set. It does not retry and does not hang.

### Why polling is the only completion signal

RMI is request/response and the controller initiates nothing the driver can subscribe to.
There is no gripper-done event, no interrupt, and no readback of gripper state. A register
the dispatcher clears is the only thing the driver can observe, so the completion signal
and the busy flag are necessarily the same bit. Two consequences you cannot design around:

- **No feedback of any kind.** No width, no force, no grip-detected. If your gripper can
  report those over its own I/O, your TP program can latch them into further registers,
  but the driver will not read them — and `read_register` is available if you want to.
- **Nothing continuous.** A modifier is one number that your dispatcher interprets. You
  *can* make it a width in tenths of a millimetre; the driver does not care. What the
  driver will not do is validate it as a physical quantity — it checks membership in the
  list your protocol declares, so a continuous range means declaring every value you
  accept, which is usually a sign that buckets are the better design.

### Launching it: fork, never call

`FRC_Call(<your dispatcher>)` **never returns** — the dispatcher is an infinite loop, so
the RMI session that called it is owned forever and bring-up cannot proceed to Stream
Motion. So the driver calls a one-line **launcher** program that `RUN`-forks the
dispatcher and returns immediately. `deploy/GRPRUN.LS` is ours, and its entire body is:

```
   1:  RUN GRIPDISP    ;
```

Two things about the fork you should plan for:

- **It outlives the process that started it,** and no RMI verb terminates it — not
  `FRC_Abort`, not `FRC_Reset`, not disconnecting. Measured on our controller: after all
  three, the dispatcher still answered the register handshake and the gripper still
  actuated.
- **Which is why bring-up probes before forking.** Blindly forking on every start would
  stack un-killable dispatchers. Instead the driver writes a benign command (your
  protocol's open verb at its default modifier) and watches whether the trigger clears
  within `gripdisp_probe_timeout_s` (6 s). Cleared → a dispatcher is alive, skip the fork.
  Not cleared → fork the launcher; the pending trigger is picked up when your dispatcher
  starts, so the open still completes.

That probe is the reason your dispatcher's **open verb at its default modifier must be
safe to execute at bring-up, with whatever is or is not in the gripper.** For a
two-finger gripper, opening is the natural clean start state. If for your tool it is not,
supply a protocol whose `default_open_modifier` names something that is.

**And it is not only at bring-up.** The **cold-reconnect** recovery escalation rebuilds the
dispatcher through that same probe, so the benign open can fire *mid-session*, after a fault, on
a gripper that may be holding a workpiece — `FanucDriver.reconnect()` documents it as leaving the
gripper "usable and open". Reaching it needs a fault the lighter RMI ladder cannot clear: an
**E-stop recovery does not re-probe** (measured — it recovers via the RMI tier and the dispatcher
is never rebuilt). But when the escalation does run, the part is released. If that is
unacceptable for your tool, hold the workpiece mechanically or gate recovery in your own layer.

---

## 3. Writing the protocol

```python
from airo_fanuc import DriverConfig, DriverPolicy, FanucDriver, RegisterGripperProtocol

MY_GRIPPER = RegisterGripperProtocol(
    name="Schunk EGP-40 via EGPDISP",   # only ever appears in messages and logs
    trigger_reg=1,
    action_reg=2,
    modifier_reg=3,
    open_action=1,
    close_action=2,
    open_modifiers=(0, 1),             # every value your dispatcher implements
    close_modifiers=(0, 1),
    default_open_modifier=0,           # must be safe to run at bring-up — see §2
    default_close_modifier=1,
    launcher_program="EGPRUN",         # the RUN-fork wrapper
    dispatcher_program="EGPDISP",      # the infinite loop it forks
)

driver = FanucDriver("192.168.1.100", DriverPolicy(
    config=DriverConfig(profile=my_profile),
    gripper_protocol=MY_GRIPPER,
))
```

Nothing else changes. The worker, the bring-up probe and the preflight TP-program check
all read this one instance, so they cannot disagree about which registers your gripper
uses.

What is configurable and what is not:

| Configurable | Fixed |
|---|---|
| all three register numbers | the three-register shape |
| both action values | that the trigger is written last and cleared by the dispatcher |
| the accepted modifier values, per verb, and their defaults | that there are exactly two verbs (open, close) |
| both TP program names | that the launcher forks and returns |
| `trigger_settle_s`, `poll_hz`, `dispatch_timeout_s`, on the protocol | polling as the completion signal |

The two-verb limit is the real constraint. A tool whose useful commands are not a pair
(a three-finger hand with named grasps, a tool changer, a multi-stage crimper) does not
fit this surface — but it fits the *mechanism*, so `driver.rmi.write_register()` and
`read_register()` are public and you can run your own handshake over them.

---

## 4. Worked example: our Robotiq 2F-85

Shipped as `airo_fanuc.gripper.ROBOTIQ_2F85`, and the default. The gripper is wired to
the controller's tool I/O; the dispatcher is `GRIPDISP` and the launcher is `GRPRUN`.

**Registers.** R[1] trigger, R[2] action, R[3] modifier. Action: `1` = open, `2` = close.

**R[3] on open — a width bucket.** The millimetre figures are what the gripper reaches
for the POSITION value GRIPDISP sends; nothing reads them.

| Value | Name | GRIPDISP sends | Opening |
|---|---|---|---|
| 0 | `OPEN_FULL` (default) | POSITION 0 | ~85 mm, fully open |
| 1 | `OPEN_MID` | POSITION 75 | ~60 mm |
| 2 | `OPEN_NARROW` | POSITION 150 | ~35 mm |

**R[3] on close — a force class.** The choice is about the object, not the gripper.

| Value | Name | GRIPDISP sends | For |
|---|---|---|---|
| 0 | `FORCE_LIGHT` | POSITION 220, FORCE 100 | rigid or easily-crushed parts |
| 1 | `FORCE_MEDIUM` (default) | POSITION 220, FORCE 150 | most things |
| 2 | `FORCE_HARD` | POSITION 255, FORCE 255 | compressible parts that only hold once squeezed |

```python
from airo_fanuc.gripper import FORCE_LIGHT, OPEN_NARROW

driver.gripper.open_gripper_and_wait(OPEN_NARROW)
driver.gripper.close_gripper_and_wait(FORCE_LIGHT)
```

**GRIPDISP is not in this repository.** Only its launcher (`deploy/GRPRUN.LS`) is,
because the launcher is one line and the dispatcher is a Robotiq-block program specific
to how our tool is wired. To reproduce this cell you write GRIPDISP against §2 and
install both on controller flash.

---

## 5. Known limits

- **The gripper path is not covered by the validation ladder.** Every script in
  `examples/` runs with `enable_gripper=False`. Passing all eight steps says nothing
  about your gripper; it needs its own run once the tool is mounted. For reference, ours was
  exercised separately: bring-up with the gripper enabled took 14.9 s against ~11 s without
  (the GRPRUN fork plus the probe), and every bucket completed in **0.32–1.52 s** against a 5 s
  `dispatch_timeout_s` — roughly 3× headroom. A gripper command issued while recovery is in
  flight is refused immediately with `{'success': False, 'message': 'RMI recovery in progress
  — refusing gripper command (retry once recovery has completed)'}` rather than queued or hung.
- **`dispatch_timeout_s` itself is untested on hardware.** A dispatcher killed while the driver
  stays healthy should fail the command at that bound. The one attempt aborted the
  `STREAM_MOTN` program along with the dispatcher, so the fail-fast gate answered first and the
  timeout never ran. Abort the `GRIPDISP` task specifically if you want to exercise it.
- **`success: True` is the dispatcher's word.** No independent confirmation exists.
- **One command's result at a time.** A second submit replaces the first's result.
- **Bring-up on a cold controller may need `connect_retries > 1`** when the gripper is
  enabled: the launcher fork perturbs the RMI session, and the ladder's flush-and-reconnect
  can consume an attempt.

---

See also: [configuration](configuration.md) for where `gripper_protocol` sits among the
other policy fields, [safety](safety.md) for the preconditions bring-up enforces, and
[troubleshooting](troubleshooting.md) for what a failed gripper command usually means.
