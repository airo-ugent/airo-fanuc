# Troubleshooting

---

## Bring-up fails

**`OwnershipError: ... already owned by pid=N`** — another process holds the controller.
Kill it. The lock is released by the kernel, so there is no stale file to clean up. If the
pid does not exist, the lock is already free and something else is wrong.

**`FanucConnectionError: controller reports a N ms interpolation period`** — the ITP guard.
The controller told the driver its period differs from the configured one, and bring-up
refuses rather than running with every per-tick limit scaled wrongly. Pass `itp_s=N/1000`
(or `--itp-ms N` in the examples).

**`FanucPreflightError`** — the gate refused before any motion, and the message names which
check and what to do. Two alarm classes are hard blocks rather than warnings because neither
clears by retrying: one wants the payload confirmed on the pendant, the other wants a
controller power-cycle.

**`FanucConnectionError` after several attempts, gripper enabled** — expected on a cold
controller, and why `connect_retries` defaults to 3. The attempt that forks the gripper
dispatcher perturbs the RMI session and can time out; the next attempt adopts the
now-running fork. A no-gripper session reaches streaming on the first attempt.

**Bring-up hangs, or never reaches streaming** — [`controller-notes.md`](controller-notes.md) §2 has the
measured recovery procedures, including a Stream Motion daemon wedge that only a controller
power-cycle clears.

**TP `RESET` appears to do nothing** — expected in AUTO on this controller class. The driver
clears faults over RMI instead.

---

## A command is refused

**`RobotFaultedError`** — the driver is not commandable. Two distinct cases, and
`get_state()` tells them apart:

- `lifecycle_state` is not `"streaming"` → a fault is latched. `fault_reason` and
  `operator_hint` say which and what to do; see [safety](safety.md).
- `lifecycle_state` **is** `"streaming"` but `motion_inhibited` is `True` → the ARM gate.
  Recovery succeeded and the driver is waiting for a deliberate `driver.arm()`. This is the
  common surprise after an E-stop: `recover()` returned `True` and motion still refuses.

**`RejectedStartMismatch`** — the trajectory's first knot is farther than 5° from the last
*commanded* pose, or shedding the velocity change would need more travel than the splice has. A
refusal rather than a snap-to-target, because snapping starts a motion with an unplanned jump.
Anchor your first knot on `get_state()["q_cmd"]`.

**`TrajectoryValidationError: first-knot |qd| exceeds the 15°/s capture envelope`** — a
different failure with a different exception: the splice that bridges the commanded pose to your
first knot cannot reach that velocity, so it is refused during validation, before the capture
gate runs. Start the trajectory at rest, or within the envelope. (A plain sine demands its peak
velocity at t=0 and is refused for exactly this reason; a raised cosine is not.)

**`TrajectoryValidationError: move_j requires a stationary arm`** — `move_j` plans from the
commanded state read at the call, and a moving anchor advances while the plan is in flight,
so the executed motion would depend on how long submission took. Call `hold()` (or
`stop_j()`), then `wait_until_steady()`.

**`TrajectoryValidationError` on `force_stop_n`** — the controller streams no force block, so
the guard could never trip. Refused rather than armed as a silent no-op. There is no force
telemetry to enable; contact detection is the controller's collaborative stop.

**A gripper command returns `{"success": False}`** — read the message. "not cleared after
5.0s" means your dispatcher is not running or not clearing the trigger; "RMI recovery in
progress" means retry once the driver is streaming again.

---

## The arm moves, but not as expected

**Non-zero `total_slew_clips`** — the core trimmed the commanded path against its per-tick
position bound, so **the executed path is not the planned one.** Either the trajectory is
asking for more per-tick travel than the velocity limits allow, or there is a discontinuity
at a knot. Not a fault, and counted rather than raised, but a clip on a path you planned
means the plan and the profile disagree.

**`q_meas` lags `q_cmd`** — expected, and mostly the controller's servo lag rather than
error. Compare it as an implied time offset (divide the position gap by the commanded speed)
so it is comparable across speeds. On our arm that offset is **84–180 ms and moves with recent
duty** — it climbs about one 8 ms tick per successive motion and falls back after a few minutes
idle, so two runs of the same motion legitimately differ (`controller-notes.md` §1.9a). Not a
fault, and nothing in the driver is gated on it. Two cautions if you measure it: the figure the
examples print samples every 250 ms and so under-reads the true peak, and below ~10 °/s its
noise floor dominates — four runs of one identical 3.7 °/s command spanned 71–102 ms. For a
number you can rely on, cross-correlate per-tick `q_cmd` against `q_meas` instead.

**A `move_j` peaks well below the speed you asked for** — the move is too short to reach it.
Accel and decel ramps need travel, and at a high `joint_speed` those ramps alone can exceed
the whole move. The examples print how much travel the ramps need.

**`is_steady()` never becomes `True`** — you are in SERVO. A servo stream has no terminal
condition; `hold()` ends it.

**`CONTACT_STOP` in a clear cell** — the collaborative-stop monitor infers contact force
from motor disturbance torque, and a sharp jerk ramp reads as a phantom contact. Shape
trajectories with a softer jerk than the clamp — see
[portability](portability.md#the-accelerationjerk-question).

---

## The real-time loop

Every example run ends with an `rt health` block; `driver.timing_stats()` is the same data.
The loop must put exactly one command packet on the wire every interpolation period.

- **`tx_interval_p50_ms`** — should sit within 1% of the ITP. This is *the* measurement.
- **`tx_interval_max_ms`** — the worst single late tick. Up to 2 ITP is tolerable: the
  controller stands roughly 9-15 missed periods before it coasts and drops
  `motion_possible`, so one late tick is not dangerous. A `max` that keeps growing means the
  host is not keeping up.
- **`cpu_migrations`** — any count is fine. The driver sets no CPU affinity and needs no
  reserved core; this is here to correlate with a bad `max`, not to be minimised.
- **`missed_rx_ticks`** — ticks with no fresh status packet. A few percent is normal; the two
  clocks are independent. **`rx_seq_gaps`** counts actually-dropped packets and should be
  zero on a wired link.
- **`double_send_guard`** and **`tau_advance_count`** are core invariants, not tuning. The
  trajectory clock must advance exactly once per tick or playback runs fast or slow. A
  problem here is a driver bug, not a host problem.

If `tx_interval_max_ms` fails on an otherwise idle machine, try `sched_fifo=True` and
`mlock=True` (`--sched-fifo --mlock` in the examples). Both are best-effort: without the
privileges (`CAP_SYS_NICE`, a `MEMLOCK` rlimit) nothing fails loudly and nothing improves
either, so check the numbers rather than assuming it took effect.

---

## After a run ends

**`MOTN-603 ST: Receiving interval over` on the pendant** — expected, and not a fault in the
run that just finished. Closing the session sends the Stop packet and then stops
transmitting, and the controller posts this because command packets ceased while the
`STREAM_MOTN` program was still up — no RMI verb terminates it. It clears on the next
bring-up and needs no operator action.

Worth distinguishing from the same alarm appearing **during** a run, which would mean the
8 ms deadline is genuinely being missed. Check `tx_interval` and `skipped_tick_windows`
first.

---

## Working on the driver

**A passing test suite proving nothing.** `uv run pytest` does **not** rebuild the C++
extension. After touching anything under `src/cpp/`:

```bash
uv sync --extra dev --reinstall-package airo-fanuc
```

Otherwise the suite tested the previously compiled `_core` and a green run means only that
the *old* binary still passes. This is the single most common way to waste an hour here.

**`ctest` passing against a stale binary** has the same shape: a failed build leaves the
previous test executables in place, and `ctest` will happily run them. Check the build
succeeded before believing the result.

---

See also: [safety](safety.md), [api](api.md) for what each getter reports, and
[`controller-notes.md`](controller-notes.md) for measured controller behaviour, exact alarm texts and recovery
procedures.
