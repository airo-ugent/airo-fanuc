# Validating this driver on a real CRX

> **These scripts are written for one specific robot: a FANUC CRX-10iA/L on an
> R-30iB-class controller negotiating Stream Motion v3 at an 8 ms interpolation
> period.** That is the arm this package was developed against and the only one any
> of it has been measured on. Nothing here auto-detects the arm, so on a different
> FANUC every assumption below is yours to check first — see
> [What is specific to this arm](#what-is-specific-to-this-arm).

Six runnable scripts. The first four are the ordered validation run: each connects,
prints what the controller told it, moves a little, and ends in an explicit
`PASS`/`FAIL` verdict — so a run is something you can read rather than something you
have to interpret. Exit code `0` means every check passed. The last two command nothing
by default: one checks the guard table against the arm, the other resolves what frame
the streamed Cartesian pose is in.

| Script | What it proves |
|---|---|
| `move_joints.py` | The stack connects and executes one commanded move |
| `move_j.py` | `move_j` plans that move for you, at a speed you name, and lands every joint together |
| `sine_wave.py` | It *tracks* a continuous multi-joint path, and `stop_j()` stops it |
| `servo_stream.py` | The same path *streamed* setpoint-by-setpoint through `servo_j` |
| `check_joint_limits.py` | The recorded soft limits match the arm. Read-only, you move it |
| `verify_tcp_frame.py` | Which frame `get_tcp_pose()` is in — read-only unless `--move` |

The motion scripts take `--fake`, which stands up an in-process fake controller and
needs no hardware (`check_joint_limits.py` has nothing to fake — it needs a real arm to
move; the fake serves `verify_tcp_frame.py` one injected pose on both planes, so a
`--fake` PASS there proves the comparison runs, never the frame). `--help` lists every
knob. Shared connection/reporting plumbing lives in `_common.py`; each script keeps its
own trajectory construction inline, which is the part worth copying into your own code.

The arm itself is `crx10ial.py`: the `RobotProfile` — velocity/acceleration/jerk clamps
and joint position limits, in degrees, each carrying the provenance of its number — that
every script passes to `DriverConfig(profile=...)`. The package requires one and ships
none, so this file is the answer to "which robot is on the other end of the wire".

Most of it was generated rather than typed. The controller reports its own active
velocity and position limits, and

```bash
python -m airo_fanuc.controller_probe --ip 192.168.1.100 --emit-profile
```

prints that file's profile call ready to paste, along with the model, software P-level,
ordered options and which teach-pendant programs are installed. It is read-only — FTP
fetches from the controller's `md:` device, no motion, no RMI — so it is safe to run
against a live cell, and it is the first thing to run on an arm that is not this one.

## Before you touch the robot

```bash
uv sync --extra dev                       # builds the C++ extension into the venv
uv run python examples/move_joints.py --fake
uv run python examples/move_j.py --fake
uv run python examples/move_j.py --fake --multi
uv run python examples/sine_wave.py --fake --period 4 --cycles 1
uv run python examples/sine_wave.py --fake --period 8 --cycles 1 --stop-after 2
uv run python examples/servo_stream.py --fake --period 4 --cycles 1
uv run python examples/verify_tcp_frame.py --fake
```

All seven must end in `PASSED`. This is also the check to re-run after editing any
C++: `uv run pytest` does **not** rebuild the extension, so run `uv sync --extra dev
--reinstall-package airo-fanuc` first or you will be validating the old `_core`.

CI lints these scripts but does not run them — they assert the real-time loop held
an 8 ms deadline, which a shared runner cannot be relied on to do. The `--fake` runs
above are that check, on your own machine.

## On the robot

Preconditions (the full list is in the top-level README): controller in AUTO, drives
powered, no active alarm, override 100%, and **nothing else talking to the
controller** — it serves exactly one Stream Motion peer and one RMI session, and
this driver takes an `flock` to enforce that. An operator stays at the robot with the
E-stop in hand for every step below.

### Step 1 — bring-up, no motion

```bash
uv run python examples/move_joints.py --ip <CONTROLLER_IP> --no-move --observe 10
```

Nothing moves. This exercises the whole bring-up ladder — ownership lock, RMI
connect, `GRPRUN`, the double reconnect, `FRC_Call(STREAM_MOTN)`, the Stream Motion
handshake, preroll — and then holds position while streaming.

Read the `bring-up` block:

- `preflight` — the gate's own verdict. Warnings are fine; `HARD_BLOCKS` are not (a
  hard block raises instead of reaching here).
- `stream motion` — the negotiated version and, importantly, **the interpolation
  period the controller reported next to the one this driver was configured for**.
  Every per-tick limit is scaled by that period, so a mismatch is refused at
  bring-up rather than run with wrongly-scaled limits. On an R-30iB-class controller
  expect `negotiated v3 ... controller ITP 8 ms`. If your controller reports a
  different period, pass `--itp-ms <that value>`.
- `wrench` — `unavailable` on a v3 controller is correct, not a failure: v3 streams
  no force block, so the C++ force guard has nothing to read and `force_stop_n`
  would be inert.
- `rt health` — see [Reading the RT numbers](#reading-the-rt-numbers).

### Step 2 — one joint

```bash
uv run python examples/move_joints.py --ip <CONTROLLER_IP> --joint 6 --delta-deg 10 --duration 4 --return
```

J6 is the default because a wrist-roll error is the cheapest kind. The run checks
that the motion returned `DONE` and that the joint actually arrived within the
settle tolerance — measured from outside the driver, in the degrees you asked for.

### Step 2b — the same move, planned for you

```bash
uv run python examples/move_j.py --ip <CONTROLLER_IP> --joint 6 --delta-deg 20 --speed-scale 0.1 --return
uv run python examples/move_j.py --ip <CONTROLLER_IP> --multi --speed-scale 0.1 --return
```

Step 2 hand-builds a 2-knot trajectory; this asks `move_j` for the journey and gives it
a speed instead of a duration. `--speed-scale` is a fraction of the arm's slowest joint
velocity limit — `0.1` is 12 deg/s on the CRX-10iA/L — so it means the same thing on
any arm, and `1.0` is the fastest a leading-axis speed can be while every joint can
still reach it. The run echoes the absolute deg/s it resolves to. (This is not the
`speed_scale` `move_trajectory` used to take: that rescaled an already-built
trajectory's playback but not its capture splice, stepping velocity at the handover.
This one is applied before planning, so profile and splice agree.)

`--multi` moves three joints unequal distances at once and times each one's arrival,
which is how the leading-axis synchronisation shows up from outside — a 20 ms spread
offline, against the ~1.5 s a sequential version would take.

If a move is too short to reach the speed you asked for, the run says so and prints how
much travel the accel and decel ramps need at that speed, rather than failing: at
`--speed-scale 1.0` on this arm those ramps alone are ~100 deg, so a 20 deg move peaks
near 47 deg/s no matter what you ask for.

Two numbers decide whether the jerk-limited profile suits this arm: **slew clips** must
stay at zero (a clip means the core trimmed the planned profile, so the executed path is
not the planned one), and the run must not fault with `CONTACT_STOP` in a clear cell
(the collaborative-stop monitor reading a jerk ramp as a collision). The acceleration
and jerk numbers in `crx10ial.py` are derived, not measured, and `move_j` plans at
`MOVEJ_LIMIT_SCALE_A` / `_J` of them — walking `--speed-scale` up while watching those
two is how that fraction gets validated on real hardware.

### Step 3 — continuous tracking

```bash
uv run python examples/sine_wave.py --ip <CONTROLLER_IP> --amplitude-deg 5 --period 10 --cycles 2
```

All six joints swing from the current pose to +10° and back as a raised cosine, ~3°/s
peak. Whole cycles, so it ends where it started — itself a check (`returned to the
start pose`). A raised cosine rather than a plain sine because a sine demands its peak
velocity at t=0, and the capture splice that bridges the commanded pose to knot 0 tops
out at 15°/s: a plain sine is refused outright above that (measured — accepted at
15.0°/s, refused at 15.1). Starting at rest sidesteps it, so speed is limited only by
the joint velocity limits.

The number to watch is `peak lag`: `q_cmd` is what went on the wire, `q_meas` is what
the controller reported. Their difference is dominated by the controller's servo lag,
not by error, and the run prints it as an implied offset in ms so it is comparable
across speeds. Expect ~85–100 ms on this arm — which is 3–4× the recorded
`tracking_lag_s`, an open question (§1.9a), not a fault.

`the arm moved` looks trivial and is not. An interpolator that silently re-anchors to
a frozen pose is indistinguishable from a healthy hold from the outside — status
keeps flowing, no fault is raised — so the run asserts a non-zero measured speed.

Then work the speed up. Peak speed is `amplitude × 2π / period`, and peak accel and
jerk go as `ω²` and `ω³` — so raise speed by *amplitude* at a long period rather than
by shortening the period, which escalates both far faster. Validated to 63°/s on
2026-07-30 (`--joints 6 --amplitude-deg 30 --period 3 --cycles 3 --knot-dt 0.02`). The
joint-limit guard aborts before any motion if a swing would leave the soft limits.

### Step 4 — the protective stop

```bash
uv run python examples/sine_wave.py --ip <CONTROLLER_IP> --amplitude-deg 10 --period 6 --stop-after 3
```

Calls `stop_j()` mid-motion — the universal preempt — and expects `STOPPED`, not
`DONE`. It reports how long the arm took to reach standstill and how far it
travelled getting there. Note this is the *driver's* brake, a limit-respecting decel
on the 8 ms path; it is not the controller's own backstop and not an E-stop.

### Step 5 — the joint-limit guard, against the arm

```bash
uv run python examples/check_joint_limits.py --ip <CONTROLLER_IP>
```

Commands nothing: it opens a **connect-only** RMI session and polls joint angles at
15 Hz while *you* hand-guide each joint to its stop, then reports the extremes it saw
against the recorded table. Never `FRC_Initialize` — that locks the motion group and
disables the hand-guidance you need (§1.6).

Worth doing because that table is copied from `docs/controller-notes.md` §1.1, not read
from the robot, and a limit recorded too WIDE means the guard passes a command the
controller refuses. A `short` verdict is ambiguous — either you did not reach the stop,
or the table is too wide — and only the operator can tell which.

### Step 6 — a fault, on purpose

No script for this one: press the E-stop during step 3 or 4, while it is moving.

What should happen: the arm stops, the sample line reports `fault=e_stop` with the
operator instruction that goes with it, the motion resolves `FAULTED` (so the verdict
fails — correct, the motion did not complete), and the `shutdown` block reports
`closed cleanly`. Then release the E-stop, press `RESET` on the pendant, and re-run
step 1: it should come back up. If it does not, that is the recovery ladder having
something real to say, and the printed operator hint is where to start.

**Shutting down is itself a check** (`driver shut down cleanly`), because an E-stop is
exactly when teardown gets interesting: it lands mid-trajectory, the controller is
latched, and auto-recovery is armed, so the shutdown runs against a recovery ladder
that is still talking to the controller. The teardown is ordered — quiesce, stop the
supervisor, terminal Stop packet, join the RT thread, RMI disconnect, release the lock
— and every join is timed. A join that wedges is *abandoned* and reported rather than
killing the process, which is why it is a check and not an assumption. `Ctrl-C`
mid-motion takes the same path, with a `stop_j()` brake first.

`tests/test_driver.py::test_close_is_clean_while_an_estop_is_latched` exercises this
against the fake controller, including that the ownership lock comes back free.

Two things worth knowing before you try it. The driver's policy defaults leave an
armed E-stop recovery in `MOTION_INHIBITED` — motion stays refused until something
calls `arm()`, deliberately, so a fault cannot silently resume into motion. And
`TP RESET` has no authority in AUTO on this controller, so a reset that appears to
do nothing on the pendant is expected; the driver clears faults over RMI.

### Step 7 — servoing

Not part of the minimal ladder: steps 1–6 validate the motion path, and this validates
the *other* way of driving it. Run it before servoing in anger.

```bash
uv run python examples/servo_stream.py --ip <CONTROLLER_IP> --amplitude-deg 5 --period 10 --rate 50
```

The same raised cosine as step 3, but streamed — Python evaluates the setpoint every
20 ms and pushes it with `servo_j` instead of handing the whole path over in one
`move_trajectory`. Running the identical motion both ways is the point: the difference
in the numbers is the delivery mechanism.

Three blocks are worth reading, in this order.

- **`commanded` / `plan error`** — core-side, and the question a servo stream exists to
  ask: did the driver put *this* path on the wire? `commanded` is the peak velocity the
  core actually sent, against the peak the plan asked for; `plan error` is the worst gap
  between the commanded pose and where the plan says it should have been. Neither is
  asserted — the position figure has a floor of one snapshot age, and neither has a
  principled ceiling — but a ratio well above 1.0 means the stream is not being rendered
  tightly, and step 3's numbers for the same motion are the comparison. This is the
  measurement the fake cannot settle either way, so take it on the arm.
- **`send spacing` / `starved`** — host-side, and specific to servoing. With
  `move_trajectory` a late Python thread costs nothing, because the C++ loop already
  holds every knot it needs. Streaming puts the host inside the control loop: past the
  printed *staleness horizon* the core re-issues its last target and rests, so the arm
  dwells. That is a stutter, not a fault, and nothing else in the driver reports it —
  which is why `starved` is a check. The horizon is derived from the core's own
  arithmetic, not chosen.
- **`peak lag`** — the same servo-lag number as step 3, and comparable to it.

Two servoing hazards the script is built around, both of which bite in real code:

- **`stop_j()` does not stop a servo loop.** It preempts every target submitted before
  it, but a target submitted *after* it is accepted normally — so a loop that keeps
  feeding restarts the arm a tick or two after the brake. `--stop-after 4` stops the
  loop as well, and then checks the arm was still stopped a second later.
- **A servo stream never ends by itself.** Stop feeding and the core holds the last
  target, in SERVO mode, indefinitely; it never reaches HOLD, so `is_steady()` stays
  False however long you wait. `driver.hold()` is how a stream is ended.

`--rate` decides how well the arm tracks, not whether the targets are accepted. A servo
target is never refused for being far away: the core plans a fresh profile to it under
the joint velocity, acceleration and jerk clamps and follows it best-effort, so a rate
too low for the speed asked for shows up as a coarser commanded path and a larger `plan
error`, never as a stalled arm. The script prints the peak per-step distance before it
moves, so the granularity you chose is on the record. A non-zero `rejected` count means
something else entirely — the driver was not commandable, i.e. faulted or recovering.
Above 125 Hz there is nothing to gain — the mailbox is latest-wins, so extra targets are
coalesced away rather than queued.

## Reading the RT numbers

Every run ends with an `rt health` block. The loop must put exactly one command
packet on the wire every interpolation period; these numbers say whether your host
does that.

- **`tx interval`** — the real measurement. `p50` should sit within 1% of the ITP.
  `max` is the worst single late tick: the check allows up to 2 ITP because the
  controller tolerates roughly 9–15 missed periods before it coasts and drops
  `motion_possible`, so one late tick is not dangerous — but a `max` that keeps
  growing means the host is not keeping up.
- **`cpu migrations`** — expected to be non-zero. This driver sets no CPU affinity
  and needs no reserved core; migrations are normal, and `tx interval` is what
  decides whether that is fine on your machine. It is here to be correlated with a
  bad `max`, not to be zero.
- **`missed_rx_ticks`** — ticks with no fresh status packet. A few percent is normal
  (the two clocks are independent). `rx_seq_gaps` counts actually-dropped packets and
  should be zero on a wired link.
- **`no two sends inside one ITP`** and **`one tau-advance per tick`** are core
  invariants, not tuning: the trajectory clock must advance exactly once per tick or
  playback runs fast or slow. A `FAIL` there is a driver bug, not a host problem.

If `tx interval max` fails on an otherwise idle machine, try `--sched-fifo --mlock`.
Both are off by default so an unprivileged process runs unchanged, and both are
best-effort — a denied request is logged and tolerated, not fatal, so if you pass
them without the privileges (`CAP_SYS_NICE`, a `MEMLOCK` rlimit) nothing will fail
loudly and nothing will improve either.

## What is specific to this arm

The driver itself is far more portable than these scripts are — the top-level README's
"Which robots this actually works on" is the accurate account of that. The scripts, by
contrast, name our arm on purpose, so a validation run has concrete numbers to check
against instead of asking the operator for six of them. Almost all of it is in one
place: **`crx10ial.py`** holds the `RobotProfile` — velocity, acceleration and jerk
clamps plus joint position limits, in degrees, each with the provenance of its number.
That file is what the driver clamps against and what these scripts guard against, so
for a different FANUC it is the file to copy. What is left:

| What | Where | Why it is arm- or controller-specific |
|---|---|---|
| Velocity / acceleration / jerk clamps, joint position limits | `crx10ial.py` | The whole arm envelope, in one profile. Passed to `DriverConfig(profile=...)`, which has no default — the package ships no arm's numbers. `_common.py`'s `LIMIT_LOWER_DEG` / `LIMIT_UPPER_DEG` read from it, so the pre-motion guard and the RT core's clamps cannot disagree. Regenerate the velocity and position halves for a new arm with `controller_probe --emit-profile`; only the acceleration and jerk derivation is a judgement call. |
| 8 ms interpolation period | `--itp-ms` default | An R-30iB-class fact. Bring-up refuses a mismatch, so a wrong value fails loudly rather than silently mis-scaling every per-tick limit. |
| Stream Motion v3 / type-202, no force block | the `bring-up` report, and `sm_version=3` under `--fake` | What our controller negotiates. A v4 controller streams a force block and `get_wrench()` starts returning values. |
| Six joints, `--joint 1..6`, "J6 = wrist roll" | `NDOF = 6` in `_common.py` | `kNumJoints = 6` is a C++ compile-time constant, not a config knob. 7-axis arms and positioner axes are out of scope. |

## What these scripts deliberately do not cover

Passing all seven steps validates the motion path end to end. It does not validate:

- **The gripper.** Every script runs `enable_gripper=False`. The `GRPRUN`/`GRIPDISP`
  path is the one part of this package specific to a particular end effector, and it
  needs its own run once the tool is mounted.
- **How faithfully a streamed servo plan is rendered.** Step 7 measures it and prints
  it; nothing asserts it, because there is no principled threshold and the fake is not
  authority on it. `plan error` is never zero and is not by itself a fault: a
  best-effort tracker trails a moving target by roughly its response time, so the figure
  grows with commanded speed. The two things worth reading are that `commanded` stays
  near 1.0× — a core rendering the stream tightly has no reason to exceed the plan's own
  peak speed — and that the error is steady rather than cyclic, since a cycle means the
  profile is repeatedly giving ground and then making it up. Both are core-side (SERVO
  dispatch reads no measurement), so the plant is not in the loop and the fake cannot
  settle them; step 3 runs the identical path with the core holding the whole
  trajectory, and that comparison is what gives either number meaning.
- **The acceleration and jerk clamps.** The values in `crx10ial.py` are derived from
  the velocity limits; FANUC's own `joint_limits.yaml` in the vendored driver
  publishes accelerations 6–16× lower. Nothing in these runs distinguishes
  the two: both are permissive enough that 63°/s ran clean on 2026-07-30. Decide it by
  working the speed up further in step 3 and watching for vibration or a servo alarm.
- **Recovery from a collision-induced `SystemFault`**, which is a different path from
  the E-stop drill (it can leave RMI unresponsive and forces the cold-reconnect
  escalation). Provoking it deliberately is not something to do casually.
- **The J2/J3 angle representation on RMI reads**, which only affects the
  RMI-sourced receive/calibration path, not Stream Motion. A wrong assumption there
  is a silent J2-sized FK error, so it wants its own check against the pendant's
  displayed joint angles.

## When something goes wrong

The scripts print the failure and exit non-zero rather than raising a traceback at
you. Exit codes: `0` all checks passed; `1` aborted before the driver was up (the
pre-motion banner or bring-up itself was interrupted); `2` bring-up failed, or a check
failed — including a `Ctrl-C` mid-motion, which is recorded as a failed check because
the run did not finish what it set out to do.

- `OwnershipError: ... already owned by pid=N` — another process holds the
  controller. Kill it; the lock is released by the kernel, so there is no stale file
  to clean up.
- `FanucConnectionError: controller reports a N ms interpolation period` — the ITP
  guard. Pass `--itp-ms N`.
- `FanucPreflightError` — the gate refused before any motion, and the message names
  which check and what to do about it (`SYST-348` wants the payload confirmed on the
  pendant; `SYST-322` wants a power-cycle).
- Bring-up hangs or the driver never reaches streaming — `docs/controller-notes.md`
  §2 has the measured recovery procedures, including the Stream Motion daemon wedge
  that only a controller power-cycle clears.
