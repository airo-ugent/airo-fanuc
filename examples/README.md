# Validating this driver on a real CRX

Two runnable scripts, meant to be run in order. Each one connects, prints what the
controller told it, moves a little, and ends in an explicit `PASS`/`FAIL` verdict —
so a run is something you can read rather than something you have to interpret.
Exit code `0` means every check passed.

| Script | What it proves |
|---|---|
| `move_joints.py` | The stack connects and executes one commanded move |
| `sine_wave.py` | It *tracks* a continuous multi-joint path, and `stop_j()` stops it |

Both take `--fake`, which stands up an in-process fake controller and needs no
hardware. `--help` lists every knob. Shared connection/reporting plumbing lives in
`_common.py`; each script keeps its own trajectory construction inline, which is the
part worth copying into your own code.

## Before you touch the robot

```bash
uv sync --extra dev                       # builds the C++ extension into the venv
uv run python examples/move_joints.py --fake
uv run python examples/sine_wave.py --fake --period 4 --cycles 1
uv run python examples/sine_wave.py --fake --period 8 --cycles 1 --stop-after 2
```

All three must end in `PASSED`. This is also the check to re-run after editing any
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

### Step 3 — continuous tracking

```bash
uv run python examples/sine_wave.py --ip <CONTROLLER_IP> --amplitude-deg 5 --period 10 --cycles 2
```

±5° on all six joints at ~3°/s peak. Whole cycles, so it ends where it started —
which is itself a check (`returned to the start pose`). The one to watch is
`peak lag`: `q_cmd` is what went on the wire, `q_meas` is what the controller
reported, and their difference is the controller's ~25 ms servo lag, not an error.
At 3°/s that is under a tenth of a degree. A lag far above `speed × 0.025` means the
arm is not following the path it was given.

`the arm moved` looks trivial and is not. An interpolator that silently re-anchors to
a frozen pose is indistinguishable from a healthy hold from the outside — status
keeps flowing, no fault is raised — so the run asserts a non-zero measured speed.

Then work up: `--amplitude-deg 15 --period 6` is ~15°/s, still gentle. Increase
speed before amplitude; the lag budget scales with speed, and the joint-limit guard
aborts (before any motion) if an amplitude would leave the soft limits.

### Step 4 — the protective stop

```bash
uv run python examples/sine_wave.py --ip <CONTROLLER_IP> --amplitude-deg 10 --period 6 --stop-after 3
```

Calls `stop_j()` mid-motion — the universal preempt — and expects `STOPPED`, not
`DONE`. It reports how long the arm took to reach standstill and how far it
travelled getting there. Note this is the *driver's* brake, a limit-respecting decel
on the 8 ms path; it is not the controller's own backstop and not an E-stop.

### Step 5 — a fault, on purpose

No script for this one: press the E-stop during step 3 or 4, while it is moving.

What should happen: the arm stops, the sample line reports `fault=e_stop` with the
operator instruction that goes with it, the motion resolves `FAULTED` (so the verdict
fails — correct, the motion did not complete), and the script closes down cleanly
rather than hanging or leaving the controller wedged. Then release the E-stop, press
`RESET` on the pendant, and re-run step 1: it should come back up. If it does not,
that is the recovery ladder having something real to say, and the printed operator
hint is where to start.

Two things worth knowing before you try it. The driver's policy defaults leave an
armed E-stop recovery in `MOTION_INHIBITED` — motion stays refused until something
calls `arm()`, deliberately, so a fault cannot silently resume into motion. And
`TP RESET` has no authority in AUTO on this controller, so a reset that appears to
do nothing on the pendant is expected; the driver clears faults over RMI.

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

## What these scripts deliberately do not cover

Passing all five steps validates the motion path end to end. It does not validate:

- **The gripper.** Both scripts run `enable_gripper=False`. The `GRPRUN`/`GRIPDISP`
  path is the one part of this package specific to a particular end effector, and it
  needs its own run once the tool is mounted.
- **The acceleration and jerk clamps.** The values in `controller_facts.py` are
  derived from the velocity limits; FANUC's own `joint_limits.yaml` in the vendored
  driver publishes accelerations 6–16× lower. Nothing in these runs distinguishes
  the two, because both are permissive enough for the gentle speeds above. Decide it
  by working the speed up in step 3 and watching for vibration or a servo alarm.
- **Recovery from a collision-induced `SystemFault`**, which is a different path from
  the E-stop drill (it can leave RMI unresponsive and forces the cold-reconnect
  escalation). Provoking it deliberately is not something to do casually.
- **The J2/J3 angle representation on RMI reads**, which only affects the
  RMI-sourced receive/calibration path, not Stream Motion. A wrong assumption there
  is a silent J2-sized FK error, so it wants its own check against the pendant's
  displayed joint angles.

## When something goes wrong

The scripts print the failure and exit non-zero rather than raising a traceback at
you. Exit codes: `0` all checks passed, `1` aborted before connecting, `2` bring-up
failed or a check failed.

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
