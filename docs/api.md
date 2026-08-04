# API reference

The surface is split the way robot client libraries usually are: a **receive** half of
state getters, and a **control** half of motion commands. `FanucDriver` presents both on
one object.

Two conventions run through all of it:

- **Getters never lie about the robot's state.** Every one returns a value or `None` rather
  than raising because of what the robot is doing, and every value that can be stale is
  published with its age beside it. (A wrong *argument* type is still a `TypeError` — that is
  programmer error, not robot state.) Two block, and both are marked below.
- **Motion outcomes never raise.** A faulted, stopped or rejected motion *resolves* to a
  `MotionResult`. Exceptions are for programmer error and for commands that cannot be
  accepted at all.

---

## Constructing a driver

```python
from airo_fanuc import DriverConfig, DriverPolicy, FanucDriver

driver = FanucDriver("192.168.1.100", DriverPolicy(config=DriverConfig(profile=my_profile)))
```

`FanucDriver(ip, policy)` **blocks** and runs the whole bring-up ladder — ownership lock,
RMI connect, preflight gate, TP launch, Stream Motion handshake, preroll — returning only
once the robot is streaming and commandable. There is no separate `connect()`. On any
failure it cleans up everything it built and re-raises.

| Raises | When |
|---|---|
| `OwnershipError` | another process holds the controller lock |
| `FanucPreflightError` | a preflight gate failed (carries the structured report) |
| `FanucConnectionError` | unreachable, handshake failed, preroll timed out, or the controller reports a different interpolation period than configured |

`close()` is the counterpart, and `FanucDriver` is a context manager. Both are safe to call
twice. `close()` raises `FanucError` if a teardown step exceeded its timeout, naming which —
the wedged thread is abandoned rather than killing the process, so that report is the only
signal you get.

Three composed parts are public, for the things the facade does not wrap:
`driver.core` (the C++ real-time core), `driver.rmi` (the RMI client — `write_register`
and friends), `driver.gripper` (`None` unless `enable_gripper`).

`driver.policy` reads back the `DriverPolicy` it was constructed with — `policy.settle`
(what a caller's own wait around a motion has to be sized against), `policy.gripper_protocol`
(which protocol `driver.gripper` is executing) and `policy.config` (`itp_s`, the period the
real-time loop targets). It is the live object, not a copy, and mutating a field afterwards
reconfigures nothing: the C++ config was mirrored once at construction.

---

## Control

| Method | Blocks | Returns | Requires STREAMING + armed |
|---|---|---|---|
| `move_trajectory(times, q, qd=None, *, settle=None, deadman_s=None, force_stop_n=None, asynchronous=False)` | no | `MotionHandle` | yes |
| `move_j(q, *, joint_speed=None, settle=None, deadman_s=None, force_stop_n=None, asynchronous=False)` | no | `MotionHandle` | yes |
| `servo_j(q, duration, *, qd=None, qdd=None)` | no | `MotionHandle` | yes |
| `stop_j()` | no | `None` | **no — any state, any thread** |
| `hold()` | no | `None` | no |
| `kick()` | no | `None` | no |
| `arm()` | no | `None` | no |
| `recover(timeout_s=None, *, escalate_reconnect=True)` | yes | `bool` | no |
| `reconnect()` | yes | `None` | no |

All angles are **radians**, velocities rad/s, accelerations rad/s². `times` is int64
nanoseconds relative to the start of the motion.

**`move_trajectory`** submits one whole timeline and returns; the C++ core owns playback
from there, so a late Python thread costs nothing. `times` must be strictly increasing
with at least two knots, `q`/`qd` finite and `|qd|` within the profile's velocity limits,
and the first knot must be within the capture window of where the arm currently is.

**`qd` is optional.** Omit it and the driver derives the knot velocities from `times` and
`q`: monotone cubic tangents (the PCHIP rule), with both ends at rest. Playback is cubic
Hermite between knots, so `qd` is not a description of the path but the tangent the core
interpolates with — deriving it here rather than in each caller means one derivation, and
one that matches the playback. The rest-at-both-ends part is load-bearing: knot 0 is what
the capture splice has to reach, and it can only reach 15 °/s, so a differentiated start
velocity is what turns a good path into a refused submission. Pass `qd` whenever your
planner already produced it — those are the tangents it shaped the path with. A derived
velocity is validated exactly like a supplied one, and a path whose knot spacing outruns
the arm is refused with a message saying the velocities were derived.

**`move_j`** plans the timeline for you from the current pose to `q` under the profile's
jerk-limited envelope, then submits it through `move_trajectory`. **It requires the arm to
be at rest** and refuses a moving one with `TrajectoryValidationError` — call `hold()` or
`stop_j()` and `wait_until_steady()` first. `joint_speed` is the leading-axis speed in
**rad/s**; every joint is time-synchronised to it, so they arrive together. `None` resolves to
a quarter of the profile's *slowest* joint velocity limit — on a fast arm that is not slow, so
name it explicitly for anything but a nudge.

**`servo_j`** is replace-not-queue: each target supersedes the last, and the core plans a
fresh profile to it under the servo limits and follows it best-effort. `duration` is in
**seconds**, and it is the intended spacing to the next target rather than a tick period. There is **no
distance rejection** — a far target is chased, not refused — and **no collision check**.
`qd`/`qdd` are validated for shape and finiteness but **currently ignored** by the core.
A servo stream has no terminal condition: see `hold()`.

**`stop_j()`** is the universal preempt. Callable from any thread in any state, never
raises, takes effect within one tick, and is a limit-respecting deceleration on the 8 ms
path — not the controller's own backstop and not an E-stop. It preempts everything
submitted before it, but a target submitted *after* it is accepted normally, so a servo
loop that keeps feeding restarts the arm a tick later.

**`hold()`** brakes to rest and holds the commanded pose, resolving the active motion as
`PREEMPTED`. **This is the only way a servo stream ends.** Stop feeding one and the core
holds the last target in SERVO indefinitely — status keeps flowing, nothing faults, and
`is_steady()` stays `False` however long you wait.

**`kick()`** feeds the per-motion deadman watchdog, for callers using `deadman_s`.

**`arm()`** clears the ARM gate after an E-stop or an operator-required fault. Nothing
else clears it — see [safety](safety.md).

### Exceptions from a command

| Raises | When |
|---|---|
| `RobotFaultedError` | not STREAMING, or ARM-gated. Carries `reason` and `operator_hint` |
| `TrajectoryValidationError` | any argument failed validation: shape, finiteness, non-monotonic times, over-limit velocity, a target outside the position limits, a moving arm for `move_j`, an unusable guard value, or a collision-check rejection |
| `RejectedStartMismatch` | the capture gate refused the first knot — it sits outside the 5° capture window, or shedding the velocity change would need more travel than the splice has. Only when `asynchronous=False`; otherwise the reject resolves on the handle as `MotionResult.REJECTED` |

A first knot whose `|qd|` exceeds the 15°/s the splice can reach is a
`TrajectoryValidationError`, not a `RejectedStartMismatch`: it is refused during validation,
before the capture gate runs at all.

## `MotionHandle`

Returned by every motion command. A pure accessor over core state — no thread, no
callback, nothing to clean up.

| Member | Blocks | Notes |
|---|---|---|
| `wait(timeout=None)` | yes | Returns the terminal `MotionResult`. A **positive `timeout` that elapses raises `TimeoutError`** — a caller-supervision concern, not an outcome: the motion keeps running, poll `result()` |
| `result()` | no | The terminal `MotionResult`, or `None` while pending/running |
| `done()` | no | `bool` |
| `motion_id`, `submit_mono_ns` | no | identity and submission time |
| `trajectory_start_mono_ns` | no | monotonic ns of the CAPTURE→TRAJECTORY edge, host-observed within one watch interval; `None` until seen |

`MotionResult` is one of `DONE`, `SETTLE_TIMEOUT`, `STOPPED`, `PREEMPTED`, `FAULTED`,
`REJECTED`. `SETTLE_TIMEOUT` means the commanded timeline finished but the measured joints
never confirmed arrival within the settle window — the path was sent, the arm did not
agree it got there.

**Not calling `.wait()` is fine.** The motion runs in the C++ core regardless of whether
Python ever polls it. `asynchronous=` is unrelated: it only decides whether a capture-gate
rejection is raised at the call or resolved on the handle.

---

## Receive

| Method | Blocks | Returns |
|---|---|---|
| `get_state()` | no | `dict` — the whole published state, see below |
| `get_joint_configuration()` | no | measured joints (rad); `None` before the first status packet |
| `get_joint_velocities()` | no | estimated joint velocities (rad/s); `None` before the first status packet |
| `get_flange_pose()` | no | `[X, Y, Z, W, P, R]` in **mm and degrees**, faceplate frame; `None` before the first status packet. Whether this field follows the *active* UTOOL is unverified — see [portability](portability.md#open-questions-stated-as-such) |
| `get_tcp_pose()` | **yes** | the same six numbers at the **tool tip**, with the controller's active UTOOL applied; `None` if the read fails |
| `get_wrench()` | no | `[fx, fy, fz, mx, my, mz]` in N and Nm; `None` on a controller that streams no force block |
| `joints_at_wall(t_wall_ns)` | no | joints (rad) nearest a wall-clock stamp — for pairing with a camera shutter; `None` if no match |
| `timing_stats()` | no | `dict` of RT loop health, see below |
| `is_steady()` | no | `bool` — in HOLD and below ~2°/s |
| `wait_until_steady(timeout=5.0)` | yes | `bool`; `False` on timeout |
| `preflight_report` | no | the structured report from bring-up |

`get_tcp_pose()` is the only getter that blocks on I/O: it is an RMI round trip costing tens
of milliseconds, because it is the controller's own answer rather than something derived here.
(`wait_until_steady` blocks too, but on a poll loop you asked for.) The core-state reads are
lock-free against the real-time thread; `get_state()` additionally takes the supervisor's
mutex for the lifecycle half, which the RT thread never holds. **This package
does no kinematics** — there is no FK, no IK, no URDF — so these two poses are what the
controller reports, and nothing more.

`get_joint_configuration()` / `get_joint_velocities()` are `q_meas` / `qd_est` from
`get_state()`, behind the same "no data yet" gate the pose getters apply — they answer
`None` until a status packet has landed, rather than handing out the zero-initialised
snapshot as if all-zero joints were a reading. They are *measured*; `q_cmd` in `get_state()`
is what the driver last put on the wire, and the two differ by the servo's tracking lag.
Each takes its own snapshot, so read `q_meas` and `qd_est` out of one `get_state()` when a
position/velocity pair must be the same tick.

### The pose convention

Both pose getters return `[X, Y, Z, W, P, R]` in **millimetres and degrees** — wire units,
exactly what the pendant's POSITION screen shows. **W/P/R are fixed-axis XYZ angles:**

```
R = Rz(R) · Ry(P) · Rx(W)
```

W about the world X axis, then P about world Y, then R about world Z. That is the one thing
a consumer converting to a homogeneous matrix must get right and cannot guess from the six
numbers. It holds for both getters: at one standstill pose the streamed faceplate and the
controller's own `FRC_ReadCartesianPosition` reported orientation bit-identically, and the
composition itself was resolved against that pose's 175 mm tool lever arm to better than
0.1° — measured, `controller-notes.md` §1.10, not assumed.

### `get_state()`

44 keys: the core snapshot, the supervisor's lifecycle view, and the ownership record.

**Joint state** — all six-element lists, radians and rad/s:

| Key | Meaning |
|---|---|
| `q_meas` | measured position, from the status packet |
| `qd_est` | estimated velocity (five-sample finite difference at ingest) |
| `q_cmd`, `qd_cmd`, `qdd_cmd` | last *commanded* position, velocity, acceleration — what went on the wire |
| `cmd_tick` | tick counter the command above was issued on; advances every tick whether or not anything is transmitting |

**Enumerated state** — plain integers, each with an exported decoder. These are pybind11
enums and **not `int` subclasses**, so `state["mode"] == Mode.HOLD` is always `False`.
Convert first, and compare with `==` rather than `is`:

```python
from airo_fanuc import Condition, FaultReason, Mode, MotionStatus
Mode(int(state["mode"])) == Mode.HOLD
bool(int(state["conditions"]) & Condition.E_STOP)      # a bitmask, not an ordinal
```

| Key | Decoder |
|---|---|
| `mode` | `Mode` — STREAM_DOWN, PREROLL, HOLD, CAPTURE, TRAJECTORY, SERVO, BRAKE, SAFE_FOLLOW, RX_SILENT |
| `fault` | `FaultReason` |
| `active_motion_status` | `MotionStatus` |
| `conditions` | `Condition` — bit flags |

**Controller gates** — `e_stopped`, `in_error`, `tp_enabled`, `motion_possible`,
`motion_in_progress` (bool), `contact_stop_status` (int), `safety_scale` (float, the DCS
speed clamp fraction).

**Cartesian and force** — `cart` (nine floats, raw wire units: mm and degrees, six pose
plus three extended axes), `fx`/`fy`/`fz`/`mx`/`my`/`mz` (N, Nm), `fs_type` (int),
`wrench_valid` (bool — gates the six force values; `False` on a controller with no force
block).

**Freshness and sequence** — `rx_age_ms` (float, ms since the last status packet;
**this is the staleness annotation** — read it before trusting `q_meas`), `rx_fresh`
(bool, was there a fresh packet at this publish), `rx_mono_ns` / `tick_mono_ns` (int64 ns,
`CLOCK_MONOTONIC`), `ctrl_time_stamp_ms`, `rx_seq`, `tx_seq`.

**`rx_age_ms` alone cannot tell a fresh feed from one that never started**: it reads `0.0`
both when the last packet arrived this tick and when no packet has ever arrived — in which case
`q_meas` is all zeros. Check `rx_mono_ns > 0` before trusting either. That is what
`get_flange_pose()` does internally, and why it returns `None` rather than a pose of zeros.

**Motion and lifecycle** — `active_motion_id`, `epoch` (the command epoch; a command
against a stale one is refused), `total_slew_clips` (diagnostic — non-zero means the core
trimmed the planned profile, so the executed path was not the planned one),
`lifecycle_state` (string, decode with `LifecycleState`; the values are **lowercase** —
`"disconnected"`, `"preflight"`, `"rmi_connecting"`, `"tp_launch"`, `"sm_handshake"`,
`"streaming"`, `"degraded"`, `"faulted"`, `"recovering"`, `"lost"`, `"shutting_down"`), `fault_reason` (string, `"none"`
when nothing is wrong — never empty), `faulted`, `operator_hint` (the actionable pendant
instruction, or `None`), `operator_required`, `motion_inhibited` (the ARM gate),
`recovery_count`, `owner` (`{pid, mode, since}` or `None`).

### `timing_stats()`

Whether the host is holding the 8 ms deadline. `tx_interval_p50_ms` / `_p99_ms` /
`_p999_ms` / `_max_ms` is the measurement that matters; `rx2tx_*_us` is the
receive-to-transmit latency within a tick. `tick_count`, `tx_count`, `parked_ticks`,
`missed_rx_ticks`, `rx_seq_gaps`, `rx_nonfinite_drops`, `cpu_migrations`,
`skipped_tick_windows` are counters. `double_send_guard` and `tau_advance_count` are core
invariants, not tuning — a non-zero `double_send_guard` is a driver bug.
[Troubleshooting](troubleshooting.md) has what good numbers look like.

---

## Exceptions

```
FanucError
├── FanucConnectionError          transport / bring-up: cannot reach the controller
├── FanucPreflightError           a preflight gate refused, carrying the report
├── RobotFaultedError             a command while not commandable (.reason, .operator_hint)
├── RejectedStartMismatch         the capture gate refused a trajectory's first knot
├── OwnershipError                another process holds the lock (.pid, .mode, .since)
├── RmiSessionDown                a live RMI session died and bounded reopen failed
├── TrajectoryValidationError     a motion argument failed validation
└── CalibrationError              a calibration capture was refused
    ├── CalibrationSourceError            (.source, .fact)
    └── CalibrationVelocityUnavailable

RmiError(RuntimeError)            a controller-reported non-zero ErrorID (.error_id, .text)
```

`except FanucError` catches everything the driver owns. **`RmiError` is deliberately
outside that tree** and derives from `RuntimeError`: it is a protocol-level signal rather
than a driver verdict, and the RMI ladders need to catch it together with raw transport
failures as `except (RmiError, OSError)`. If you want controller errors handled uniformly
with driver errors, name it explicitly.

---

## `FanucReceiveInterface`

A separate, **motion-free** reader for calibration work. It polls joint angles and status
over RMI and never calls `FRC_Initialize`, so it does not take the motion group and does
not disable hand-guidance. Use it when you want to read the arm while a human moves it —
not alongside `FanucDriver`, which owns the controller exclusively.

```python
from airo_fanuc.receive_interface import FanucReceiveInterface

with FanucReceiveInterface(ip="192.168.1.100") as rx:
    sample = rx.latest_joint_sample()
```

It builds its own RMI client from `ip` — ports, poll rates and the ownership mode are keyword
arguments — and the context manager is `start()` / `stop()`.

| Method | Blocks | Returns |
|---|---|---|
| `start()` / `stop()` | briefly | `None`; also a context manager |
| `latest_joint_sample()` | no | `JointSample` (**degrees**, wall-clock ns) or `None` |
| `get_controller_status()` / `get_extended_status()` | no | the cached RMI status, or `None` |
| `estimate_velocity_deg_s(window_s=None)` | no | per-joint least-squares slope in **deg/s**, or `None` |
| `settled(threshold=None, window=None, *, previous_q=None)` | no | `SettleResult` with a `reason` string |
| `capture_calibration_sample(*, threshold=None, window=None)` | no | a `JointSample`, **or raises** |
| `reset_calibration_dataset()` | no | `None` |

`capture_calibration_sample` is the exception to "getters never raise", deliberately. A
stale or frozen joint feed reads exactly like a perfectly settled robot, so an unguarded
capture folds a wrong pose into a calibration dataset silently and nothing downstream can
detect it. Every guard therefore *raises* rather than returning `None`:
`CalibrationError` (no samples, still moving, or a position identical to the last accepted
sample), `CalibrationVelocityUnavailable` (velocity could not be estimated — refusing
beats fabricating a zero, which a frozen feed would pass), and `CalibrationSourceError`
(RMI-sourced joints while the J2/J3 representation is unverified for your installation, or
mixing sources within one dataset). See [portability](portability.md) for that last one.

---

See also: [configuration](configuration.md) for every field of `DriverConfig` and
`DriverPolicy`, [architecture](architecture.md) for what happens between a submit and the
wire, and [safety](safety.md) for the lifecycle and fault model these calls sit on.
