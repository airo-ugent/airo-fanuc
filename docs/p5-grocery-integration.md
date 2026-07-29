<!-- SPDX-License-Identifier: Apache-2.0 -->
# P5 grocery integration runbook (`airo_fanuc` cutover)

**Where this runs.** These steps execute on a **dries-HEAD base**. The
`feat/airo-fanuc` branch (the whole `packages/airo_fanuc/` tree + this
`src/grocery_bot/robot/fanuc_backend/` layer) is **disjoint from dries** — it adds
new files only and touches no existing grocery consumer, so it is a **0-conflict
merge**. **Merge `feat/airo-fanuc` into dries first**, then work through the
per-consumer edits below on the merged tree.

P5a (this branch) shipped the base-independent, `FakeCRX`-testable half: the
compat **shim** + **facade** + **backend switch** + **host-setup files** + this
runbook. P5b is the per-consumer migration + the on-hardware wiring (curobo
`capture_check`, curobo FK hook, real zenoh republisher) — all below.

The two halves of the compat surface (PLAN §7, R3 A6+C1):

| Layer | Module | Replaces | Drop-in for |
|---|---|---|---|
| Executor shim | `grocery_bot.robot.fanuc_backend.shim.FanucDriverExecutor` | `MotionExecutor` / `MotionExecutorClient` | `execute` / `set_target` / `start` / `stop(brake_first)` / `recover` / `wait_until_idle` / `current_target` / `current_target_anchor_ns` / `state` / `fault()` + `compute_trajectory_timeout` + `TrajectoryTarget` / `BrakeTarget(+DEFAULT_MAX_DURATION_SEC)` / `HoldTarget` / `ExecutorState` / `Fault` / `FaultType` / `ExecutorResult` / `TargetSource.wait_done` |
| State facade | `grocery_bot.robot.fanuc_backend.facade.FanucFacade` | `FanucCRX10iAL` (read half) | `get_joint_states` (deg dict) / `get_external_force_n` / `get_tcp_pose_in_base` / `joints_at_timestamp` (→ `joints_at_wall`) / `get_robot_status` / gripper dict-or-None + `wait_gripper_done` / `on_joint_states` |
| Backend switch | `grocery_bot.robot.fanuc_backend.backend` | — | `make_motion_backend(...)` / `make_facade(...)` / `resolve_backend()` |
| Zenoh republish | `grocery_bot.robot.fanuc_backend.zenoh_publisher` | old bridge republish | `ZenohSessionPublisher` / `driver_snapshot` / `build_republisher` |

**Import purity.** `grocery_bot.robot.fanuc_backend` imports only `airo_fanuc` +
numpy + stdlib. The legacy `MotionExecutorClient` (curobo/zenoh/loguru) is
imported dynamically inside `make_motion_backend`'s `"legacy"` branch, and
`import zenoh` in `zenoh_publisher` is guarded — so the whole package imports in a
curobo/zenoh-free venv (asserted by `tests/test_grocery_shim.py`).

---

## 0. One driver, two wrappers

On the `"driver"` backend there is exactly **one** `airo_fanuc.FanucDriver`
(construct-and-go: blocks until commandable or raises with a triage/preflight
report). The executor shim and the state facade both wrap it:

```python
from grocery_bot.robot.fanuc_backend import make_motion_backend, make_facade
from grocery_bot.planning.kinematics import Kinematics   # grocery-side; NOT imported by the package

kin = Kinematics()
# Constructs FanucDriver(ip, policy) unless you inject driver=; owns it (close() tears it down).
executor = make_motion_backend(backend="driver", joint_limits=kin.joint_limits, ip="192.168.1.100")
robot = make_facade(executor.driver, fk_in_base=kin.forward_kinematics_in_base)   # shares the SAME driver
```

The old code held `self._robot = FanucCRX10iAL(...)` and
`self._executor = MotionExecutorClient(...)` as two independent Zenoh peers; now
`robot` and `executor` share one driver. `executor.stop()` never closes the
driver (parity with the old executor); `executor.close()` (driver-owned) or
`executor.driver.close()` is the teardown.

### `DriverPolicy(capture_check=...)` wiring (decision 6 / R1 B1)

`move_trajectory` synthesizes the exact CAPTURE splice the RT core will execute
(`airo_fanuc._core.generate_capture_path`, one C++ code path) and hands the
resulting knots to `policy.capture_check` before submitting. Grocery wires it to
curobo — **the wheel never imports curobo; grocery injects it**:

```python
from airo_fanuc import DriverConfig, DriverPolicy

def _capture_check(knots_q, knots_qd) -> bool:
    # True iff the commanded→trajectory-start bridge is collision-free.
    return adapter.check_trajectory_collision(knots_q)          # curobo CuroboAdapter

policy = DriverPolicy(config=DriverConfig(), capture_check=_capture_check)
executor = make_motion_backend(backend="driver", joint_limits=kin.joint_limits, policy=policy)
```

The hook fires exactly when obstacles are closest (a post-collision-brake replan);
a `False` return → typed `TrajectoryValidationError`, surfaced by the shim as a
`FAULTED` `ExecutorResult`.

### Zenoh republisher wiring (PLAN §5.4/§7, R3 C3)

Independent consumers (cockpit toasts, perception capture, rrd recorder,
scene_perception) still read `fanuc/joint_states` / `fanuc/robot_status` /
`fanuc/external_force` / `fanuc/ext_status` / `grocery/log/event`. Wire the
driver's republisher onto the demo's zenoh session:

```python
from grocery_bot.robot.fanuc_backend.zenoh_publisher import build_republisher

repub = build_republisher(executor.driver, zenoh_session)   # runs; repub.stop() at teardown
```

**Do NOT** rely on `DriverPolicy(publisher=...)`: the driver's internal
republisher is fed `driver.get_state` verbatim (radian keys), which the
republisher's degrees-keyed `publish_joint_states` would skip. `build_republisher`
passes `driver_snapshot(...)` — the adapter that maps `get_state()`'s radian
internals + status bits to the republisher's `q_meas_deg` / wrench-dict /
lifecycle contract — so `fanuc/joint_states` actually publishes. The exact toast
triples (`("SYSTEM","BRIDGE:MOTION_POSSIBLE","FAIL")` /
`("SYSTEM","BRIDGE:FAULT",None)`) are preserved by `airo_fanuc.Republisher`.

### Fault recovery + the ARM gate — **behaviour change vs dries (safety)**

The old `MotionExecutor.recover()` always returned to a *movable* RUNNING state
(the bridge auto-recovered). The shim's `recover()` **does NOT auto-arm** — it
calls `driver.recover()` only, leaving the driver's ARM gate (PLAN R2 F1) intact.
Rationale: auto-arming would re-introduce the hands-at-the-pendant hazard — the
`keep_trying_plan` retry loop would move the arm ~1-2 s after the operator's
confirm keystroke. So:

* **Transient** faults (contact-stop, safety-clamp, TEACH→AUTO) auto-resume after
  the F7 dwell with **no arm() needed** — `recover()` leaves them commandable.
* **Operator-required** faults (**e-stop, SYST-348**; `ARM_FAULTS = {E_STOP,
  IN_ERROR}` + the SYST-348 operator flag) leave the driver `MOTION_INHIBITED`:
  after `recover()` the executor is RUNNING but the next `move_trajectory` raises
  `RobotFaultedError` (surfaced as a **FAULTED `ExecutorResult`, no motion**) until
  an explicit `executor.arm()`.

**P5b MUST wire the `arm()` call** in the runner's recovery path, or an
operator-required recovery silently refuses every subsequent motion (the order
flow hangs). The correct sequence for the demo runner:

```
# operator releases E-stop / clears the SYST-348 payload confirm on the TP, then:
executor.recover()          # clears the executor's FAULTED; driver still MOTION_INHIBITED for e-stop/SYST-348
executor.arm()              # ONLY after the operator has confirmed — drops the gate
# ... now execute()/keep_trying_plan retries move again.
```

Do **not** put `arm()` inside an unconditional retry loop — gate it on the
operator-confirm event. (Verified by `test_grocery_shim.py`:
`test_recover_preserves_arm_gate_for_operator_fault` — e-stop→recover stays
inhibited until arm(); `test_recover_no_arm_needed_for_transient_fault` — a TEACH
recovery is movable with no arm.) Matches the successor invariants: "the ARM gate
is real … do not auto-`arm()` in a retry loop".

---

## 1. Per-consumer migration (P5b)

Every consumer below is migrated **directly** (not left on the shim). The long
tail of tooling not listed rides the shim until P8.

### `robot/real_robot.py`
- `self._robot = FanucCRX10iAL(...)` → `make_facade(driver, fk_in_base=kin.forward_kinematics_in_base)`.
- `self._executor = MotionExecutorClient(...)` → `make_motion_backend(backend="driver", driver=driver, joint_limits=kin.joint_limits)` (pass the shared `driver`).
- `executor.execute(TrajectoryTarget(...))`, `set_target(BrakeTarget())`,
  `wait_until_idle()` are unchanged — same surface. STOP still converges on
  `set_target(BrakeTarget())`.
- **Recovery path (behaviour change — see "Fault recovery + the ARM gate" above):**
  `recover()` no longer auto-arms. For an operator-required fault (e-stop /
  SYST-348) the runner MUST call `executor.arm()` **after** the operator confirms,
  then let `keep_trying_plan` retry; without the explicit `arm()` every retry is
  refused and the order flow hangs. Transient faults need no `arm()`.

### `robot/grasp_execution.py` — contact-stop termination (NOT `force_stop_n`) — P-1 UPDATE
- **P-1 measured (controller-notes.md §1.8): this controller streams Stream Motion
  v3 / type-202 with NO force telemetry.** `get_wrench()` is `None`, and the C++
  force-guard has no signal. So the numeric-force-threshold plan below is **not
  available on this controller** — `move_trajectory(force_stop_n=...)` is **rejected**
  with a typed `TrajectoryValidationError` (arming an inert guard would be a silent
  safety no-op).
- **Grasp descent terminates on the controller's collaborative contact-stop.**
  type-202 still carries `contact_stop_status` (∈ {2,4}) + `safety_scale`; on contact
  the RT core kills the descent — the motion resolves **`FAULTED` with
  `fault_reason == CONTACT_STOP`** and the driver drops to SAFE_FOLLOW. CONTACT_STOP
  is a *transient* DEGRADED fault: the supervisor auto-recovers after the F7 dwell
  with **no `arm()`**, leaving the driver commandable for the gripper close + lift.
  The grasp runner reads `fault_reason == CONTACT_STOP` (or `contact_stop_status` in
  `get_state()` / the `robot_status` topic) as "contacted the object" — exactly the
  signal the dries executor's `CONTACT_STOP{2,4}` fault carried (NOT `get_wrench()`,
  which is None here). Do **not** re-add a Python force watchdog. (The cleaner
  `MotionResult.STOPPED` "expected grasp stop" is the *armed force-guard* path, which
  returns only on a v4 / type-204 controller.)
- Honest scope: contact is detected by the controller's own collaborative-stop
  monitor (motor disturbance torque), not a tunable Newton threshold. A numeric
  `force_stop_n` guard returns only with a v4 / type-204 controller (P84 firmware);
  the RT force-guard code path stays in place for that day, gated on `wrench_valid`.

### `planning/trajectory_monitor.py` — stop_j + anchor + kick (R1 B2, R4)
- Collision → `executor.set_target(BrakeTarget())` (converges on `stop_j`); then
  wait for the target `wait_done`/`STOPPED`, replan **from rest**, issue a new
  `execute(TrajectoryTarget(...))`. Brake-then-replan, no graded slowdown.
- Timeout / elapsed math: prefer `executor.current_target_anchor_ns()` when set,
  else the caller-supplied `t0_wall × speed_scale` (F1: the anchor is a smoothness
  refinement, not required for correctness).
- Deadman interlock (R4): the monitor tick calls `executor.kick()` →
  `driver.kick()` — "collision supervision is alive". Pass `deadman_s=` on the
  monitored `TrajectoryTarget`; expiry auto-`stop_j`s in the RT core. Unsupervised
  short moves (goto / idle) pass no `deadman_s` (blocking wait + controller backstop).

### `idle/behavior.py` — deadline-lapse REWRITTEN (R4, R3 A2)
- The old idle deadline-lapse did a mid-flight trajectory-over-trajectory swap.
  v1 has no such splice (CAPTURE-or-REJECT can't reproduce it), so rewrite it as
  **`stop_j` → wait `STOPPED` → `move_trajectory`**: on deadline lapse,
  `executor.set_target(BrakeTarget())`, `wait_until_idle()`, then
  `execute(TrajectoryTarget(next_idle_move))`. Every idle motion still replans
  through curobo so the live ESDF blocks restockers walking into the lane.

### `robot/goto_waypoint.py`
- `execute(TrajectoryTarget(plan_to_joints(q_goal)))` — named-waypoint moves target
  **joints, not TCP** (the plan is built curobo-side; the shim just runs it). No
  `deadman_s` (unsupervised short move).

### `grasping/pipeline/*` + `handover/*` — move_trajectory ONLY; servo_j NOT wired (R3 E1)
- Pipeline runtime/context and handover swap `MotionExecutorClient` → the shim and
  `FanucCRX10iAL` reads → the facade. **`servo_j` stays unwired in handover**:
  v1 handover uses planned, collision-checked, y/n-gated short `move_trajectory`
  segments (raw servo would bypass curobo collision checking with a person in the
  workspace). `servo_j` exists on `FanucDriver` for a later true-servo redesign.

### `visualization/test_planner.py`, `robot/grasp_execution` live demos
- `test_planner`, `test_grasp_pipeline`, `test_wheel_pipeline` are the actual live
  demo entry points: swap the executor/robot constructors as in `real_robot`. These
  are the L5 parity gate's exercisers.

### `calibration/auto_external.py`
- A full control-mode move+settle+gripper workflow (also a good pre-demo HIL
  script): executor for the moves, facade for the gripper + settled-state reads.
  Calibration joint reads go through `FanucReceiveInterface` (RMI-poll, single
  session) — NOT while a demo holds the SM session (see §3).

### `robot/fanuc/probe_streaming_sine` (+ `example_client`, `test_tcp_jog`)
- Diagnostic sine/jog probes move to `servo_j` (bounded, RT-gated) or short
  `move_trajectory` segments on the shim. `probe_streaming_sine` validates the
  servo path envelope (R3 C2/C3 servo window) before anything else wires `servo_j`.

---

## 2. Cutover (backend flip + disable the legacy unit)

Gate: **L5** — ≥ 3 clean cycles (zero driver-attributed `FAULTED` entries, on
separate days, at speed_scale 1.0-class). Then:

```bash
# 1. Flip the default to the driver backend (systemd env or shell profile).
export GROCERY_FANUC_BACKEND=driver
# 2. Disable the legacy bridge unit (FILE kept installed for rollback).
sudo systemctl disable --now grocery-fanuc-bridge
# 3. Install the host-setup files (see deploy/): SMT-pair slices + IRQ/ethtool + limits.
```

**Never run the legacy `grocery-fanuc-bridge` unit AND the driver backend at
once** — both drive the controller's single Stream Motion session. Preflight
fingerprints "legacy bridge owns SM session" with a targeted hint.

### Rollback (documented, one env var + one systemctl)
```bash
unset GROCERY_FANUC_BACKEND          # or: export GROCERY_FANUC_BACKEND=legacy
sudo systemctl enable --now grocery-fanuc-bridge
```
The legacy unit file, legacy entry points and the chunk topics stay installed
through P8 precisely so re-enabling them is the rollback step. `resolve_backend()`
defaults to `"legacy"` until the flip.

### Ops note (R3 G1)
Restarting the demo now = a full SM re-handshake (~6–10 s), not a bridge bounce.
The router-death / SM-session-loss failure mode is rewritten around construct-and-go
(RobotNotReady retry maps onto the driver's typed
`FanucConnectionError`/`RobotFaultedError`).

---

## 3. Per-CLI source policy (R3 C5/C6 — dual-source, no-demo)

The RMI session is **single-session**: `FanucReceiveInterface` CANNOT connect
while a demo holds it. So the policy is **topic-if-fresh, else RMI-poll**:

| Tool | While a demo runs | Standalone (no demo) |
|---|---|---|
| `grocery-status` | read republished `fanuc/robot_status` topic | RMI-poll (ReceiveInterface) |
| `record_waypoints` | — (needs the robot) | RMI-poll; works in **T1** (no AUTO-flip workaround) |
| `test_planner` record panel | republished topics | RMI-poll |
| `grocery-capture` (perception) | `fanuc/joint_states` + `joints_at_wall` (FK-at-shutter) | RMI-poll |
| `diagnose_start_collision` | republished status | RMI-poll |
| `test_eef_pose` | facade FK (topic joints) | RMI-poll + curobo FK |
| calibration suite (`hand_eye`, `test_calibration*`) | — (conflicts with SM) | RMI-poll (LSQ velocity, `settled()`, None-reject) |
| scene_perception monitor | republished topics | RMI-poll |
| `snoop` | passive wire sniff (setpoint/status cross-check) | passive |
| `measure_servo_lag` | — | debug-echo `{"t_ns","q"}` + motion dispatch |

Bonus (R3 C5): RMI-poll works in T1, so `record_waypoints` / `test_planner` lose
the AUTO-flip workaround; the 2026-05-17 T1-freeze corruption class is
structurally fixed by RMI-poll + the driver's staleness/age gates.

---

## 4. Successor invariants

The binding successor `CLAUDE.md` invariants ship WITH P5 — see
[`successor-invariants.md`](successor-invariants.md). In one line each: every
motion through `FanucDriver.move_trajectory`/`servo_j`; `stop_j()` is the universal
preempt (`STOPPED` is a clean preempt, not a fault); collision response is
brake+replan (no graded slowdown, no mid-flight traj→traj swap); no direct
`driver.core` submissions in demo code; the ARM gate is real (no auto-`arm()` in a
retry loop); reintroducing chunk topics / `MotionExecutor` imports = regression;
never run the legacy unit alongside the driver backend.

---

## 5. Host setup (`deploy/`) — PLAN §5.5

Install these on olifant at cutover (details in each file's header). Based on
PLAN §5.5 + `spike/README.md §2` (the temporary spike reservation, made permanent):

| File | Purpose |
|---|---|
| `deploy/grocery-demo.slice` | `AllowedCPUs=0-31` — the only slice on the reserved SMT pair |
| `deploy/user.slice.d/99-reserve-smt-pair.conf` | `AllowedCPUs=0-14,16-30` — user procs off the pair |
| `deploy/system.slice.d/99-reserve-smt-pair.conf` | `AllowedCPUs=0-14,16-30` — system services off the pair |
| `deploy/eno2-rt-setup.sh` | `ethtool -C eno2 rx-usecs 0 adaptive-rx off` + eno2 RX-IRQ→core 31 + irqbalance ban |
| `deploy/limits.d/99-fanuc-rt.conf` | rtprio 99 + memlock unlimited (only load-bearing if FIFO/mlock are enabled) |

- **SMT pair (R1 E1):** core 31's hyperthread sibling is core 15
  (`thread_siblings_list=15,31`), so the pair is reserved — this supersedes the old
  single-core-31 units (`fanuc.slice`, `99-reserve-core-31.conf`). The demo entry
  point additionally masks Python to `0-14,16-30` before importing torch/zenoh so
  the RT thread is the sole occupant of 31 and sibling 15 stays idle.
- **FIFO/mlock drop-criterion (PLAN §5.5, R4):** `pin_core`/`sched_fifo`/`mlock`
  default **False** in `DriverConfig`. If affinity-only holds p99 ≤ 8.5 ms &
  max < 16 ms over the 1 h adversarial soak (spike §6 ablation), FIFO+mlock+limits.d
  stay off; otherwise the entry point enables them and `limits.d/99-fanuc-rt.conf`
  becomes load-bearing.
- **fork hygiene (R2 F27):** the grocery-robot entry point sets
  `multiprocessing.set_start_method("spawn", force=True)`; O_CLOEXEC everywhere.
