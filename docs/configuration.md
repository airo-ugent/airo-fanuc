# Configuration

Three objects, nested:

```python
DriverPolicy(config=DriverConfig(profile=RobotProfile(...)))
```

- **`RobotProfile`** — the arm. Its motion envelope. **Required, no default.**
- **`DriverConfig`** — the facts: endpoints, the profile, timing, RT hygiene.
- **`DriverPolicy`** — the behaviour: retries, recovery, the ARM gate, peripherals.

Every default below is what you get by writing nothing. A live driver reads the whole nest
back as `driver.policy`, which is how a caller recovers the settle window, the gripper
protocol or `itp_s` without being handed them separately.

---

## `RobotProfile` — the arm's envelope

The one thing the package will not guess. `DriverConfig(profile=...)` is required, because
these limits are the ceiling the real-time core clamps against, the driver cannot ask the
controller which robot is attached, and a default would silently apply limits somebody else
measured on a different arm.

| Field | Required | Unit |
|---|---|---|
| `name`, `model` | yes | free text |
| `velocity_limits` | yes | rad/s |
| `acceleration_limits` | yes | rad/s² |
| `jerk_limits` | yes | rad/s³ |
| `position_limits_lower` / `_upper` | yes | rad |
| `ndof` | no (6) | must be 6 — see [portability](portability.md) |
| `max_payload_kg` | no (`None`) | kg, reference only |
| `source` | no (`"unspecified"`) | provenance, reported at start-up |

Write it in **degrees** — the unit of both the datasheet and the controller's own system
variables — and let it convert:

```python
from airo_fanuc import RobotProfile

MY_ARM = RobotProfile.from_degrees(
    name="my-arm", model="...",
    velocity_limits_deg_s=[...], acceleration_limits_deg_s2=[...], jerk_limits_deg_s3=[...],
    position_limits_lower_deg=[...], position_limits_upper_deg=[...],
    source="velocity from the controller; accel/jerk derived 2x/8x",
)
```

`from_degrees` is keyword-only on purpose: five same-shaped vectors in a row is exactly the
argument list where a positional swap is easy to make and impossible to see. Construction
validates finiteness, strict positivity of the three clamp vectors, `lower < upper` per
joint, and the joint count; anything else raises `ProfileError`. The arrays are frozen after
validation, because one profile is shared by everything the driver builds.

### Deriving a profile for your arm

Most of it you should not type, because the controller already knows:

```bash
python -m airo_fanuc.controller_probe --ip 192.168.1.100 --emit-profile
```

This prints a paste-ready `RobotProfile` call, plus the controller's model, software
P-level, ordered options (so option **S636** presence is a lookup rather than a claim), and
which TP programs are installed. It is **read-only**: it fetches diagnostic files from the
controller's virtual `md:` device and touches neither motion, RMI, nor Stream Motion, so it
is safe against a live cell. Run it first on any arm that is not ours.

What it supplies from the controller: **velocity limits and joint position limits**, read
from the active configuration.

What it cannot supply: **acceleration and jerk**, because the controller publishes no clamp
equivalent. The probe derives them as 2× velocity and 8× acceleration, and records that in
the profile's `source` string so the derived half is never mistaken for a read one. Whether
that derivation is right for your arm is a question for measurement — see
[the open questions](portability.md#the-accelerationjerk-question).

Then close the loop at bring-up with `DriverPolicy(preflight_full=True)`: it re-reads the
controller and warns when the supplied profile has drifted from what the controller says it
is enforcing, naming the direction — a profile *wider* than the controller is the dangerous
one. It flags and never adopts: the controller is authoritative about what it will allow,
but overwriting a configured clamp mid-bring-up would change the envelope under a caller
who never asked.

`examples/crx10ial.py` is a worked profile with the provenance of every number, and is the
file to copy.

---

## `DriverConfig`

### The arm and the endpoints

| Field | Default | Unit | |
|---|---|---|---|
| `profile` | **required** | | see above |
| `sm_port` | 60015 | | Stream Motion UDP (J519). Fixed on this controller class |
| `rmi_port` | 16001 | | RMI bootstrap TCP (R912) |
| `rmi_connect_timeout` | 5.0 | s | |
| `rmi_request_timeout` | 2.0 | s | per RMI request |

### Protocol and timing

| Field | Default | Unit | |
|---|---|---|---|
| `sm_version` | 3 | | the version *requested* at handshake; the session adopts what the controller reports it can do |
| `itp_s` | 0.008 | s | the controller's interpolation period. **Bring-up refuses a mismatch** with what the controller reports — every per-tick quantity is scaled by this, so a wrong value means limits silently wrong by that ratio |
| `preroll_timeout_s` | 5.0 | s | how long the handshake waits for `motion_possible` |
| `gripdisp_probe_timeout_s` | 6.0 | s | ceiling on the gripper-dispatcher liveliness probe |
| `supervisor_lost_s` | 3.0 | s | the core holds if the Python heartbeat lapses this long |

### The motion envelope's fractions

These are **fractions of the profile's limits, not limits**, which is why they live in the
package rather than in your profile: they carry across arms unchanged.

| Field | Default | Controls |
|---|---|---|
| `stop_scale_va` | 0.4 | the brake runs at this × the velocity and acceleration limits |
| `stop_scale_j` | 0.15 | and this × jerk — scaled harder than acceleration, deliberately: a collaborative-stop monitor infers contact force from motor disturbance torque, so a sharp jerk ramp reads as a phantom contact |
| `slew_factor` | 1.2 | the per-tick position clip is `slew_factor × v × itp_s` |
| `movej_scale_a` | 0.5 | `move_j` plans under this × the acceleration limit… |
| `movej_scale_j` | 0.375 | …and this × jerk. Python-only: the core still clamps against the full limits, and these keep the *plan* clear of that ceiling. A cell that finds the default too slow raises them here rather than editing the package |

### RT hygiene

| Field | Default | |
|---|---|---|
| `sched_fifo` | `False` | raise the RT thread to `SCHED_FIFO`. **Best-effort** — a denied request is logged and tolerated, never fatal |
| `rt_priority` | 80 | the priority used if it is |
| `mlock` | `False` | lock the process's pages. Also best-effort |

Both are off by default so an unprivileged process runs unchanged. Turning them on without
the privileges (`CAP_SYS_NICE`, a `MEMLOCK` rlimit) fails silently and improves nothing —
check `timing_stats()` rather than assuming.

---

## `DriverPolicy`

### Bring-up

| Field | Default | Unit | |
|---|---|---|---|
| `config` | **required** | | |
| `connect_retries` | 3 | | attempts at the whole ladder. **Leave it at 3 with the gripper enabled** — the attempt that forks the dispatcher can consume itself, and the next adopts the now-running fork |
| `hold_wait_s` | 3.0 | s | grace for the core to publish HOLD |
| `bringup_settle_s` | 2.0 | s | how long `motion_possible` must stay asserted before bring-up calls itself done |
| `preflight_full` | `False` | | add the FTP-based checks: option S636, TP programs, profile cross-check — **and a software-version floor that hard-blocks anything below V9.40P81**, which was chosen for our arm's software line. Read [portability](portability.md#tier-2--the-controller-class) before turning this on for a controller on an older line. Off the per-connect path anyway because it reads ~650 kB, about 3 s |

### Recovery

| Field | Default | Unit | |
|---|---|---|---|
| `auto_recover` | `True` | | run the recovery ladder on a fault without being asked |
| `recovery_cooldown_s` | 15.0 | s | minimum between automatic attempts |
| `recovery_delay_s` | 3.0 | s | settle before an attempt |
| `ready_wait_s` | 60.0 | s | grace for `motion_possible` and HOLD after a relaunch |
| `recovery_motion_probe_s` | 5.0 | s | short probe before escalating to a cold reconnect |
| `recovery_reconnect_attempts` | 1 | | cold-reconnect fallbacks when the ladder dead-ends. Kept small: RMI churn is itself a way to wedge the Stream Motion daemon. `0` disables the fallback |
| `arm_gate` | `True` | | **read [safety](safety.md) before changing this.** Recovery from an E-stop or an operator-required fault ends with motion inhibited until an explicit `arm()` |

### Peripherals and hooks

| Field | Default | |
|---|---|---|
| `capture_check` | `None` | your collision check — signature below |
| `enable_gripper` | `True` | build the gripper worker at bring-up |
| `gripper_protocol` | `ROBOTIQ_2F85` | which registers your dispatcher watches — see [gripper](gripper.md) |
| `publisher` | `None` | a duck-typed state sink for the republisher threads. `None` = no republish, no threads |

### The collision-check hook

`Callable[[np.ndarray, np.ndarray], bool]`:

```python
import numpy as np

def my_check(q: np.ndarray, qd: np.ndarray) -> bool:
    """q, qd: (N, 6) float64, rad and rad/s — the synthesized splice knots the core will
    execute, on the interpolation grid. Return True iff they are safe."""
    return True
```

Called synchronously inside `move_trajectory` / `move_j` before submission, so it runs on your
thread and not the real-time one: a slow check delays the submit, never the loop. Returning
`False` raises `TrajectoryValidationError`. These are **the exact knots the core will
execute**, produced by the same function it uses — which is what makes the checked path the
executed one.

Not called for `servo_j`: there is no hook anywhere on the servo path.

### The state sink

`publisher` is duck-typed to a zenoh session — one method,
`declare_publisher(topic: str)`, returning a handle with `put(payload: bytes) -> None`. The
republisher declares once per topic at start-up and reuses each handle.

### Ownership

| Field | Default | |
|---|---|---|
| `enable_ownership` | `True` | take the advisory `flock`. **Leave it on** — one Stream Motion peer per controller is a hardware constraint, not a policy. The lock carries no controller identity, so it means *one driver per host*: a two-arm cell needs a distinct `lock_path` per arm, or the second driver is refused for no real reason |
| `ownership_mode` | `"control"` | `control` / `receive` / `tool`, recorded in the lock file so a blocked process can say who holds it |
| `lock_path` | `/run/lock/airo-fanuc/owner.lock` | |

### Settle and the loop

`settle` is a `SettlePolicy(tol_deg=0.5, vel_eps_deg_s=2.0, timeout_s=2.0)`: how close the
*measured* joints must come to the commanded endpoint, and how slowly they must be moving,
before a motion resolves `DONE`. Exceeding `timeout_s` resolves `SETTLE_TIMEOUT` instead —
the path was commanded, the arm never confirmed arrival. Overridable per call:
`move_trajectory(..., settle=SettlePolicy(tol_deg=1.0))`.

`watch_interval_s` (0.02) is the supervisor's poll period. It bounds how quickly a fault is
observed and how stale `trajectory_start_mono_ns` can be.

---

## What is not configurable

Three values are settable but **not mirrored** from `DriverConfig`, so in ordinary use they
stay at the C++ defaults: the PLL's lead time and gain, and the minimum DCS speed-clamp
fraction that counts as a clamp. Reach them on an `airo_fanuc._core.RtCoreConfig` if you must.

Genuinely unreachable, because they are mechanism rather than policy: the sustained slew-clip
threshold, and the brake's own settle epsilon and duration cap. The joint count is a
compile-time constant — see [portability](portability.md).

---

See also: [the API reference](api.md), [gripper](gripper.md) for the protocol object, and
[portability](portability.md) for which of these numbers are properties of your controller
rather than of ours.
