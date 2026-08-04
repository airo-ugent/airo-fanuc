# Architecture

Two protocols, two languages, one thread that must not miss a deadline.

```
              your code
                  │
        ┌─────────▼──────────┐
        │    FanucDriver     │  Python: validation, lifecycle, recovery policy
        │  ┌──────────────┐  │
        │  │  Supervisor  │  │  bring-up ladder, fault classification, ARM gate
        │  └──────┬───────┘  │
        └─────┬───┴──────────┘
              │              └────────────► RmiClient ──── TCP 16001 ──► controller
              │                              (JSON)
     ┌────────▼─────────┐
     │  _core (pybind11)│
     │  ┌────────────┐  │
     │  │RealtimeCore│  │  C++: the RT thread — timerfd/epoll, PLL, sockets
     │  │ ┌────────┐ │  │
     │  │ │TickCore│ │  │  the mode + fault state machine
     │  │ │┌──────┐│ │  │
     │  │ ││ tick ││ │  │  I/O-free math: Hermite, brake, capture, servo, slew, settle
     │  │ ││engine││ │  │
     │  │ │└──────┘│ │  │
     │  │ └────────┘ │  │
     │  └─────┬──────┘  │
     └────────┼─────────┘
              └──── UDP 60015, every 8 ms ──────────────► controller
```

The dividing line: **everything on the 8 ms path is C++, everything about policy is
Python.** Python is never in the control loop and the RT thread never calls into Python —
not for logging, not for a callback. Python *polls*.

---

## The two protocols, and why both

Neither is sufficient alone, which is the single most important structural fact about this
driver.

| | Stream Motion | RMI |
|---|---|---|
| Transport | UDP, port 60015 | TCP, port 16001 → redirect port |
| Shape | one packet out, one in, every interpolation period | JSON request/response |
| Rate | 125 Hz (8 ms) | on demand |
| Carries | commanded joint angles; measured joints, Cartesian pose, controller gates | everything else |

**Only Stream Motion can move the arm** at rate, and it reports the safety gates
(`e_stopped`, `in_error`, `motion_possible`, contact stop, DCS speed clamp) on every one of
those 125 packets per second.

**Only RMI can start it.** The `motion_possible` bit the Stream Motion session needs is
armed by a Teach-Pendant program (`STREAM_MOTN`), and launching a TP program is an RMI
operation. RMI also owns fault reset, register I/O (which is how the gripper works), and
the controller's own tool-tip pose.

So bring-up is necessarily: RMI first, then Stream Motion. And the asymmetry runs the other
way at shutdown — no RMI verb terminates a running `STREAM_MOTN`, so the session ends by
sending a Stop packet and ceasing to transmit.

### Packets actually used

| Direction | Type | Size | Purpose |
|---|---|---|---|
| → controller | 7 | 28 B | GetCapability — negotiates version and sampling rate |
| → controller | 200 | 8 B | Start |
| → controller | **201** | **344 B** | **CommandPacket — the one that matters, every 8 ms** |
| → controller | 205 | 16 B | ForceSensorConfig (v4 only; this driver never sends it) |
| → controller | 2 | 8 B | Stop |
| ← controller | 202 | 388 B | RobotStatus, v3 — **no force block** |
| ← controller | 204 | 416 B | RobotStatus, v4 — with force/moment |

The wire carries nine axis slots; joints 7-9 are zero-filled.

One field in the command packet deserves naming because getting it wrong is dangerous
rather than merely broken: a reserved 16-bit word selects how the controller interprets the
payload. It must carry `0xFFFF` for joint data. Written as `0`, the controller reads the six
joint angles as a Cartesian XYZWPR pose, slews at full speed toward it, and hits the E-stop.

---

## Layers, in C++

Each depends only on the ones below it.

| Layer | Target | Owns | Depends on |
|---|---|---|---|
| bindings | `_core` | the Python surface; two offline planners that bypass the RT path (`generate_capture_path`, `plan_joint_move`) | rt_core |
| RT core | `rt_core` | `RealtimeCore` (the thread, sockets, timerfd, PLL, snapshot publishing) and `TickCore` (the mode and fault state machine) | tick_engine, codec |
| tick engine | `tick_engine` | the per-tick math, **I/O-free**: cubic-Hermite playback, Ruckig brake/servo/capture profiles, the slew clip, settle detection | Ruckig |
| codec | `fanuc_sm_codec` | encode/decode of the packets above | the vendored FANUC headers |

`tick_engine` has no clock, no socket and no thread. That is what makes the whole motion
path testable as pure functions against byte-exact goldens, and it is where the tests are
densest.

**C++ standard:** the source targets C++17, but Ruckig declares `cxx_std_20` publicly and
that propagates through every target linking it, so `tick_engine`, `rt_core` and `_core`
compile at C++20 in practice. Only the codec is genuinely C++17.

### What is vendored

The line is drawn at the wire: **vendor FANUC's protocol structs, write our own real-time
loop.**

The repo carries FANUC's `fanuc_driver` as a submodule and compiles exactly two of its
headers — the `#pragma pack(1)` Stream Motion wire structs and the byte-swap template. A
packet layout has to match the controller byte for byte, and a header published by the
protocol's own implementer is the authoritative statement of that layout; compiling it
directly means the struct we send cannot drift from a transcription of it. `PATCHES.md`
records what is vendored and that the patch set against it is empty; attribution is in
`NOTICE`.

Nothing above the codec is adopted. FANUC's driver reaches ROS 2 through a general-purpose
client layer and a `ros2_control` interface; this stack is not ROS, and the real-time loop,
the safety state machine and the interpolation layer are exactly the parts an application
with its own planner has to own — so they are written here, on the tick.

---

## Threads

One C++ thread, several Python ones, and a strict rule about what crosses between them.

| Thread | Language | Rate | Job |
|---|---|---|---|
| RT thread | C++ | **125 Hz** | epoll on the UDP socket + a timerfd: decode status → tick → encode and send → publish the snapshot |
| supervisor watch | Python | 50 Hz | poll the snapshot, classify lifecycle state, trigger recovery |
| heartbeat | Python | 10 Hz | feed the core's SUPERVISOR_LOST watchdog |
| RMI receive | Python | event-driven | the sole reader of the RMI socket; dispatches replies and stashes async pushes |
| gripper worker | Python | 20 Hz while active | one command at a time, polling for completion |
| recovery ladder | Python | one-shot | runs off the watch thread so recovery never blocks polling |
| republisher | Python | 1-100 Hz, 4 threads | only if a `publisher` is injected |

### What crosses the boundary

| Direction | Mechanism | Carries |
|---|---|---|
| Python → RT | single-producer ring, 16 slots, drained **latest-wins** each tick | submitted targets (trajectory, servo, brake, hold) |
| RT → Python | **seqlock** | the state snapshot, republished every tick |
| RT → Python | ring, 1024 slots | events (mode changes, faults, motion terminals) |
| RT → Python | ring, 64 slots | trajectory buffers to free — allocation never happens on the RT thread |
| RT → Python | per-slot seqlock ring, 256 deep | joint history, for `joints_at_wall()` |

The seqlock is why getters never block the RT thread and never tear: the writer stamps a
sequence counter odd before the copy and even after, and a reader that sees an odd or a changed
counter retries. ThreadSanitizer reports this as a data race by design — it cannot model
version counters — which is why `tests/tsan.supp` names it and the joint-history ring, plus
the two diagnostic counters that are accepted-torn for a different reason, and nothing else.

The GIL is released around precisely the three calls that can block on I/O: starting the
core, stopping it, and waiting for the handshake. Everything else is O(1) and lock-free.

---

## Bring-up

Construction runs this in order and returns only at the end of it.

1. **Ownership** — take an advisory `flock`. Another holder raises `OwnershipError` naming
   its pid rather than racing it.
2. **RMI connect** — bootstrap on 16001, follow the redirect to the session port.
3. **Preflight** — AUTO mode, drives powered, E-stop released, alarm classification; with
   `preflight_full`, also the controller's P-level, option S636 and TP programs read off
   its file server.
4. **Initialize**, then a best-effort `FRC_Continue`.
5. **TP launch** — reset; if the gripper is enabled, probe for a running dispatcher and
   fork its launcher if absent; reseed the sequence id; `FRC_Call(STREAM_MOTN)`.
6. **Stream Motion handshake** — open the socket, start the RT thread, negotiate version and
   sampling rate, wait for `motion_possible`, then for the core to publish HOLD.
7. **Settle** — re-apply the `STREAM_MOTN` launch if `motion_possible` drops, which it does
   about a second after the first call.
8. **Verify the interpolation period** the controller reported against the configured one,
   and refuse a mismatch: every per-tick limit is scaled by it.
9. Start the watch and heartbeat threads; build the gripper worker and republisher.

The whole ladder is retried `connect_retries` times (default 3). Lifecycle states along the
way are `DISCONNECTED → PREFLIGHT → RMI_CONNECTING → TP_LAUNCH → SM_HANDSHAKE → STREAMING`,
with `DEGRADED`, `FAULTED`, `RECOVERING`, `LOST` and `SHUTTING_DOWN` as the branches.
**`STREAMING` is the only commandable state.** `get_state()["lifecycle_state"]` publishes these
as **lowercase** strings — see [the API reference](api.md#get_state).

---

## From a submit to the wire

Tracing one `move_trajectory`, because where the executed path is computed is the crux of
the safety argument:

1. **Python validates** — knot count, strictly increasing int64 times, finiteness,
   `|qd|` against the profile's velocity limits.
2. **Python runs the capture gate** — it calls the *same* `generate_capture_path` the core
   will run to synthesize the splice from the current commanded state to your first knot,
   and hands those exact knots to `capture_check` if you supplied one.
3. **Submit** — the knots are copied into a buffer allocated off the RT thread, stamped
   with a motion id, the command epoch and a stop generation, and pushed onto the mailbox.
   The call returns.
4. **Next tick, the core drains the mailbox** (latest wins; a superseded target resolves as
   `PREEMPTED`), gates the submission, and on acceptance synthesizes the splice **once** and
   enters CAPTURE.
5. **When the splice completes**, the caller's knots are installed and the core samples them
   with cubic Hermite interpolation, once per tick, advancing its trajectory clock exactly
   once per tick.
6. **The slew clip** applies a per-tick position bound as a last line of defence.
7. **Encode and send**, radians to degrees, one packet.

Two properties fall out of this and both matter:

- **The collision-checked path is the executed path.** The splice is synthesized once, in
  C++, from a deterministic function that Python can call with the same inputs and get the
  same knots. There is no second implementation to drift.
- **The core never re-times or re-plans your trajectory.** Between knots it interpolates;
  it does not smooth, retime or optimise. What you submit is what runs, clipped only if it
  would exceed a per-tick bound — and that clipping is counted and published, so a
  non-zero `total_slew_clips` tells you the executed path was not the planned one.

`move_j` differs only in where the knots come from: an offline Ruckig call synthesizes them
before submission, and they then travel this identical path.

---

## Scope: two things this package does not do

**No kinematics.** No FK, no IK, no URDF, no link geometry. Stream Motion speaks joint
angles and so does RMI; the safety-critical work here is timing, fault handling and the
splice. This is a *scope* argument, not a dependency one — hand-rolled FK for a 6-DOF
serial arm would add no dependency at all — and the point is that baking in link geometry
would make an otherwise arm-agnostic package newly arm-specific. FANUC's own stack draws the
same line, shipping a solver configuration rather than a kinematics implementation. The seam
for a consumer is `capture_check`, which receives real joint knots.

`get_tcp_pose()` and `get_flange_pose()` are not exceptions: both report a pose the
*controller* computed. Nothing here derives a pose from joint angles.

**One runtime dependency: numpy.** That is the whole list, and it is enforced statically —
`tests/test_standalone_contract.py` walks the AST of every file in the package and asserts
that nothing imports a messaging middleware, a logging shim, a planner or a tensor stack
anywhere, including inside function bodies, and that the third-party import surface equals
the declared dependencies. AST-based rather than import-based on purpose: a package that
merely happens to be installed in the dev environment cannot satisfy an AST assertion, so
the environment cannot launder a violation.

The consequence is that a consumer installs this driver without inheriting a robotics
stack. Everything environment-specific is injected: the collision check, a state sink, the
arm's envelope, the gripper protocol. Adding a dependency here is a decision, not a
convenience.

---

See also: [configuration](configuration.md), [the API reference](api.md),
[portability](portability.md) for what is generic and what is not, and
[safety](safety.md) for the fault and recovery model.
