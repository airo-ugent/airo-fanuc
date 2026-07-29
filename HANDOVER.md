# Handover — finalising `airo-fanuc` as a standalone repo

You are working in a fresh repo containing **only** the FANUC driver, just copied out of a
monorepo (a wheel-of-fortune grasp-and-handover demo built on a FANUC CRX-10iA/L). Your job is
to finish turning it into a mature, self-contained package. This document is the context you
need that the code does not state — the design decisions, the invariants that were paid for in
damaged hardware, and the specific things the extraction broke.

Consider keeping this file as the repo's `CLAUDE.md` once you have worked through it.

---

## 1. What this package is, and where its boundary sits

A driver for **FANUC controllers**: a C++17 real-time core speaking Stream Motion (UDP, 125 Hz
/ 8 ms ITP) plus an RMI JSON client (TCP 16001), behind a `ur_rtde`-shaped Python API. About
133 tracked files.

**Scope of this repo: the driver and nothing above it.**

The eventual airo-mono integration — `PositionManipulator`, `ParallelPositionGripper`,
`AwaitableAction` adapters — is **deliberately not here**. It will live in
[airo-mono](https://github.com/airo-ugent/airo-mono), which will depend on this package. Do
not add it, and do not add an `airo_robots` import "to make integration easier": that import
is one of the things a test actively forbids (§2).

Nor does this package contain **kinematics** — no FK, no IK, no URDF, no link geometry. That is
a deliberate scope decision, not an omission (§3), and FANUC's own ROS driver draws the same
line.

### 1.1 Why this driver exists at all — README material

FANUC ships an official ROS 2 driver. A stranger's first question will be *"why not just use
it?"*, so the README must answer it. The honest answer is not "theirs is bad" — it is that the
decision was **narrower than a rewrite**:

> **Vendor FANUC's protocol codecs. Write our own real-time loop.**

That was the recorded verdict of a research pass over `FANUC-CORPORATION/fanuc_driver@main`
(post-v2.2.0), 2026-07-03/04, which assessed their stack module by module and concluded:
*"wrap the protocol layers vendored with a small permanent patch set; do NOT adopt
`fanuc_client`; keep our own loop."* The final plan sharpened that to the Stream Motion codecs
only, with RMI reimplemented in Python.

**What was kept.** FANUC's wire definitions — the `#pragma pack(1)` packet structs and the
byte-swap template. Those encode knowledge transcribed from proprietary manuals (the Stream
Motion and RMI OPERATOR'S MANUALs) that we have no independent access to, and their protocol
layer was assessed as thin and correct. That is exactly why this repo vendors two of their
headers instead of re-deriving them, and why `NOTICE` attributes them (§2, §4.1).

**What was not kept: their loop** (`fanuc_client`). Source-verified reasons, all from the same
research pass:

- **No real-time hygiene.** Plain `std::thread`; no `SCHED_FIFO`, no CPU affinity, no
  `mlockall` anywhere in their repo — their own troubleshooting directs users to fix host RT
  themselves.
- **Hot-path hazards.** Blocking iostream logging on the seq-skip and timeout paths; a 100 µs
  sleep-poll receive loop; and `robot_state_queue_.enqueue()` allocates as the queue grows, so
  a consumer stall means an allocation every tick.
- **A process-wide `SIGINT` sigaction plus a static singleton** in `FanucClient` — a direct
  hazard for a pybind extension.
- **~16 ms of interpolation buffer latency** by default (`out_cmd_interp_buff_target_ = 8`).
- **Stream drop means full teardown.** One second of RX silence sets `is_streaming_ = false`
  and exits the thread; recovery is a complete rebuild, not a reconnect.
- **Async fault detection is dead code.** `checkPushPacket()` loops on
  `while (packet.has_value())` over a default-empty optional, so pushed `FRC_SystemFault` /
  `FRC_Terminate` notifications are unreachable.
- **It is ROS 2.** Their integration point is a `ros2_control` `SystemInterface`; this stack is
  not ROS, and adopting it would mean adopting ROS.
- **FANUC accepts no external pull requests** (their `CONTRIBUTING.md`, verbatim: *"we are
  unable to accept external pull requests at this time"*). So every fix you need is a permanent
  local patch — a maintenance cost that applies to their loop but not to two stable headers.

**The most persuasive single data point** is from FANUC's own issue tracker, and it is not a
criticism: issue **#45**, a CRX-10iA overshoot report filed by a user running cuRobo MPC — our
exact workload. The maintainer's answer, verbatim: *"It is suggested that commands are filtered
in your application before inputting them to `forward_position_controller`."* There is no
servo-grade smoother in their stack **by design**; smoothing is the application's job. So the
interpolation and trajectory layer was always going to be ours.

**And what we needed that their stack has no concept of:** brake-then-replan as the collision
response, starvation handling on the 125 Hz stream, SYST-348 / OPERATOR_REQUIRED recovery, the
ARM gate after an E-stop, and the CAPTURE-or-REJECT collision-check splice where the checked
path is provably the executed path (§6).

**Why C++ and not Python**, which the README should also cover, since the predecessor was a
pure-Python bridge: measured `time.sleep` returning up to **176 ms** late under GIL contention,
pub/sub calls blocking 3–7.8 ms, and 150–185 ms gaps in the command stream — enough to drop
`motion_possible`. That bridge also published a **frozen `joint_states` cache at 100 Hz whenever
RX went silent**, which is the failure the driver's `rx_age_ms` staleness gate now exists to
prevent. A 125 Hz hard-real-time loop is not a thing to run in CPython.

**Tone guidance for the README.** This repo vendors FANUC's Apache-2.0 headers and credits them
in `NOTICE`; the framing should be *different design goals*, not *their code is bad*. Say that
their driver targets ROS 2 with a general-purpose client layer, that this one targets a
non-ROS, hard-real-time application with its own planner and safety state machine, and that we
build on their protocol work. Every claim above is factual and checkable, but a public README
does not need the full list — one paragraph plus the issue-#45 point carries it.

> The research reports behind this section stayed in the monorepo and were not copied here, so
> this document is now their only record — the same situation as the probe-day facts in §10.

---

## 2. The prime directive: the numpy-only contract

```toml
dependencies = ["numpy>=1.26"]
```

That is the entire runtime dependency list, and it is **statically enforced** by
`tests/test_standalone_contract.py`: an AST sweep over every `src/airo_fanuc/**/*.py` asserting

1. no `grocery_bot` / `zenoh` / `loguru` / `curobo` / `torch` / `airo_robots` import
   **anywhere**, including inside function bodies, and
2. the package's whole third-party import surface equals its declared dependencies.

It is AST-based rather than import-based on purpose — a merely-installed package cannot satisfy
an AST assertion, so the check cannot be defeated by the environment.

**Why this is the most important property of the package.** It is what makes the driver
upstreamable at all, and what lets a consumer install it without inheriting a robotics stack.
In the monorepo, house style was `from loguru import logger` in every module; a single
copy-paste of that line into this package breaks the contract. If you want logging here, use
`logging` from the stdlib or an injected callback.

Treat any proposal that adds a dependency as needing an explicit decision, not a convenience
call.

---

## 3. No kinematics in this package — that is a scope decision

This package contains no FK, no IK, no URDF and no link geometry, and it should stay that way.
The owner's call, and it is a defensible line: **this is a protocol and real-time motion
package.** Stream Motion speaks joint angles, RMI speaks joint angles, and the safety-critical
work here is timing, fault handling and the CAPTURE splice. Geometric modelling is a different
concern that lives one layer up.

Two supporting arguments worth knowing, because they are not obvious:

**FANUC's own stack draws the same line.** The vendored `fanuc_driver` (their ROS 2 driver)
ships **no kinematics implementation at all** — a grep for `inverse_kinematics`,
`forwardKinematics`, `IKFast`, `KinematicsBase` across every `.cpp/.hpp/.py` in it returns
nothing. What it ships is a solver *configuration*:
`fanuc_moveit_config/config/kinematics.yaml` points MoveIt at
`kdl_kinematics_plugin/KDLKinematicsPlugin`. The robot geometry lives in a **separate,
un-vendored** package (`fanuc_crx_description`, declared as an `<exec_depend>`), and the FK/IK
math comes from MoveIt/KDL. So "hardware interface" and "kinematic model" are separate packages
upstream too — keeping them separate here matches FANUC's architecture rather than fighting it.

**Baking geometry would make the package newly arm-specific.** Today every CRX-specific value
is either an injectable default (`DriverConfig.velocity_limits` and friends) or a documented
controller fact. Link geometry is not injectable in the same way — it would hard-code one arm
variant into a package that otherwise aims to serve any 6-axis FANUC on an R-30iB (§5). That is
a real regression in scope, distinct from the dependency question.

Note that the numpy-only contract (§2) is **not** the reason. Hand-rolled FK for a 6-DOF serial
arm is ~50 lines of numpy and seeded damped-least-squares IK is ~80, neither of which would add
a dependency. The contract rules out a *solver library* (pinocchio, KDL, curobo) or a runtime
URDF parser — not kinematics as such. Do not defend this boundary with the wrong argument; if
it is ever revisited, the question is scope, not dependencies.

**Where kinematics goes instead.** The consumer supplies it. The monorepo passed a curobo
`Kinematics` in via `make_facade(fk_in_base=...)`, and the future airo-mono adapter will do
something equivalent — it can take the model from `fanuc_crx_description` plus KDL or IKFast,
exactly as MoveIt does.

So if you find yourself wanting `get_tcp_pose()`, `forward_kinematics()` or a URDF in this
package, that is the signal you are writing the consumer layer in the wrong repo. Also still
out of scope: the `PositionManipulator` / `ParallelPositionGripper` / `AwaitableAction` adapter
classes, and any `airo_robots` import.

---

## 4. What the extraction broke — the concrete TODO

### 4.1 `.gitmodules` must be re-authored (highest priority — the build depends on it)

`vendor/fanuc_driver` is a **build dependency**, not reference material: `CMakeLists.txt:65`
points at it and `pyproject.toml`'s `sdist.include` lists
`vendor/fanuc_driver/fanuc_libs/stream_motion`. Only two header files are actually compiled
(the `#pragma pack(1)` wire structs and a byte-swap template) — `PATCHES.md` documents exactly
what is vendored and why everything else upstream is unused.

The gitlink survives a copy/split but `.gitmodules` lived at the **monorepo root** and does
not. Recreate it:

```ini
[submodule "vendor/fanuc_driver"]
	path = vendor/fanuc_driver
	url = https://github.com/driesmarzougui/fanuc_driver.git
	branch = itf-2026-v2.2.0
```

Verify with a genuinely clean clone — `git clone --recursive`, then `uv sync` must compile the
extension. This is the single most likely way the repo is broken for everyone but you.

### 4.2 `tests/test_grocery_shim.py` must not be here

It resolves `Path(__file__).parents[3] / "src"` to import `grocery_bot.robot.fanuc_backend`
from inside this venv — a path that does not exist outside the monorepo. Delete it if the copy
brought it along.

Expected suite size here: **320 tests** (the monorepo's 342 minus that file's 22). If you see
342, the file is still present. Say so in the README, so its absence does not read as an
oversight later.

### 4.3 Citations that now dangle

- **`PLAN.md` is cited ~79 times** — 71 in source (about 20 files, including C++ headers) and 8
  in the package's own docs. `PLAN.md` lived *outside* the package and is not travelling. Of
  those, 55 are bare provenance tags (`PLAN.md`) that can simply be dropped, keeping the
  surrounding explanation — the docstrings already state the behaviour; the tag only recorded
  where it was specified. The other 12 name real contracts (`PLAN §5.1` ×7 — the
  `move_trajectory` / `MotionHandle` contract; `§5.4` ×4; `§5.3` ×1) and should be repointed at
  `docs/successor-invariants.md` (motion + CAPTURE invariants) or `docs/controller-notes.md`
  (controller facts) rather than deleted.
- `CMakeLists.txt:75` — the submodule-init error message tells the user to run
  `git submodule update --init packages/airo_fanuc/vendor/fanuc_driver`. Now just
  `vendor/fanuc_driver`. This one is operator-facing.
- `src/airo_fanuc/testing/wire.py:18` — docstring cites a `packages/airo_fanuc/...` path.
- `pyproject.toml` line ~52 comment says the lint config is *"consumed by the CI `lint` job"*.
  There is no CI (the monorepo's was deleted). Either add CI (§4.5) or fix the comment.

A quick sweep for `grep -rn "PLAN.md\|packages/airo_fanuc" src/ tests/ CMakeLists.txt` should
come back empty when you are done.

### 4.4 Reframe two docs written for a monorepo

- `docs/successor-invariants.md` (96 lines) opens *"Draft of the invariants that become the
  successor `CLAUDE.md` section when the grocery demo cuts over"* — meaningless standalone.
  Reframe it as this driver's own binding invariants. It is already the distilled design
  record, which is what makes it the right home for the `§5.x` citations above.
- `docs/p5-grocery-integration.md` (322 lines) — keep it, retitled as a worked
  consumer-integration example rather than a migration milestone. It is genuinely useful: it
  shows how a real consumer wired the injected `capture_check` and kinematics.

### 4.5 Missing repo-level files

- **`README.md`** — the package has never had one. It has two jobs a stranger needs answered:
  *why does this exist when FANUC ships an official driver* (§1.1) and *which robots does it
  actually work on* (§5).
- **CI** — one Linux job: checkout with submodules, `uv sync` (compiles the C++ extension),
  `uv run pytest -q`, `uvx ruff check src tests`. All 320 tests are hardware-free, so this is
  fast and complete. Its real value is proving the submodule build dependency is fetchable from
  a clean checkout — the failure mode nothing else catches.

### 4.6 Audit `deploy/` — some of it is dead, and one file is misnamed

`deploy/GRPRUN.LS` is **load-bearing**: it is the Teach-Pendant launcher for the gripper
dispatcher and is referenced from code and docs. Keep it.

The rest — `grocery-demo.slice`, `eno2-rt-setup.sh`, `limits.d/`, `system.slice.d/`,
`user.slice.d/` — is referenced by **nothing** in the package, its docs, or the monorepo. These
look like remnants of a host-wide CPU-core-reservation approach that was measured and then
**reverted** in favour of doing real-time work on the C++ thread instead. Verify that reading,
then either delete them or document what they are for. At minimum, a file called
`grocery-demo.slice` should not ship in a driver repo.

---

## 5. The scope claim the README must make

Be precise here; it is the first thing a stranger reads, and overclaiming wastes their day. The
driver is **not** usable on all FANUC manipulators. Four tiers.

(The README's *other* obligation — explaining why this exists when FANUC ships an official ROS 2
driver — is §1.1.)

**Generic across FANUC.** Stream Motion UDP and RMI JSON are FANUC's own protocols. Needs
controller option **S636** (bundles J519 Stream Motion + R912 RMI). `sm_version` is a config
field (3 = no force telemetry, 4 = FSConfig/force). The 8 ms ITP is an **R-30iB** fact — that
is FANUC's mainstream controller, so it covers many arms, not just collaborative ones.

**Injectable, CRX-defaulted.** `DriverConfig.velocity_limits` / `acceleration_limits` /
`jerk_limits` default to `CRX10IAL_*` from `controller_facts.py` but are ordinary constructor
fields — swap them per arm. Velocity limits come from FANUC Europe datasheet MDS-04018;
acceleration and jerk are derived (2× and 8× respectively), because FANUC publishes only joint
*speeds* for the CRX.

**CRX-specific and baked in.** `kNumJoints = 6` is a C++ **compile-time constant**
(`tick_engine_config.hpp`) alongside `_NDOF = 6` in `driver.py` — fine for almost every FANUC
arm, but it excludes 7-axis and extended/positioner axes. The ITP is read as a module constant
in Python (`controller_facts.ITP_S`) even though the C++ side already has `itp_s` as a config
field, so a different controller period is a small, contained change rather than a rewrite.

**Not portable at all.** `gripper.py` drives *our* `GRIPDISP` Teach-Pendant program via RMI
registers. GRIPDISP exists **only on controller flash and is not version-controlled** — only
its `GRPRUN.LS` launcher is in this repo. For any other user this is a **site-installation
prerequisite**, not a Python dependency. Say that plainly.

Worth noting in the README too: the gripper contract is **action-dependent** — on open, `R[3]`
is a width selector (`OPEN_FULL` / `OPEN_MID` / `OPEN_NARROW`); on close it is a force class
(`FORCE_LIGHT` / `MEDIUM` / `HARD`). Three discrete buckets each, no continuous width, and **no
width feedback at all**. That is why the airo-mono `ParallelPositionGripper` adapter (elsewhere)
will have to quantize and will not be able to implement `get_current_width()` from hardware.

---

## 6. Invariants paid for in damage — do not regress these

Each of these cost real debugging, several of them with hardware consequences.

**The Stream Motion `unused` field must be written as `0xFFFF`.** Writing `0` makes the
controller interpret the payload as Cartesian XYZWPR instead of joint angles → slew → E-stop.
This actually happened. It is documented at `controller_facts.py:39-40`.

**`stop_j()` is the universal preempt.** Any thread, ≤1 tick, never raises. Every watchdog,
manual stop and fault path converges there. A `stop_j()` during a blocking `MotionHandle.wait()`
resolves it as `MotionResult.STOPPED` — that is the brake-then-replan contract. It is a *clean
preempt, not a fault*: it does not bump the epoch and does not arm the ARM gate.

**The ARM gate is real.** After an E-stop or an OPERATOR_REQUIRED (SYST-348) recovery the
driver ends in `MOTION_INHIBITED` and motion methods raise `RobotFaultedError` until an explicit
`driver.arm()`. **Never auto-`arm()` in a retry loop** — that moves the robot while an operator
is at the pendant.

**The checked path IS the executed path.** `move_trajectory` synthesizes the exact CAPTURE
splice the RT core will execute via `airo_fanuc._core.generate_capture_path` (one C++ code path)
and hands it to `policy.capture_check` when set. Do not add a second, Python-side approximation
of the splice — the whole point is that there is one.

**No numeric external force on v3 / type-202.** `get_wrench()` is always `None`, and
`force_stop_n > 0` is rejected with a typed error explaining why. Contact detection relies on
the controller's **collaborative contact-stop** (the motion resolves FAULTED with reason
`CONTACT_STOP`). Arming a force guard would be a silent safety no-op. This needs v4 / type-204
firmware to change.

**Contact-force phantom stops come from high JERK, not speed.** The CRX collaborative-stop
monitor estimates contact force from motor disturbance torque, so a steep jerk ramp reads as a
contact. This is why driver clamps and planner limits diverge (8× accel here vs 3× on the
planner side) — that split is deliberate, not drift.

**Brakes are planned from `qd_planned`, not `qd_meas`.** A `qd_meas`-seeded brake tripped
collaborative contact-stops. `STOP_LIMIT_SCALE` is split `_VA=0.4` / `_J=0.15` for the same
reason.

**`FRC_Call(GRIPDISP)` never returns** — which is why the RUN-fork launcher (`GRPRUN`) exists.
And the fork **must** stay gated on the `_gripdisp_alive()` probe: a per-process latch alone
stacked un-killable GRIPDISP tasks that wedged Stream Motion at `program_status=2`.
`gripdisp_probe_timeout_s` must remain ≥ the 5 s gripper dispatch timeout, or a false "absent"
verdict re-forks and recreates the wedge. Recovery from a wedged controller is TP
**FCTN → ABORT ALL**; a power-cycle is *not* required.

**The 125 Hz loop had to leave Python.** Measured: `time.sleep` returning up to **176 ms** late
under GIL contention, pub/sub calls blocking 3–7.8 ms, 150–185 ms chunk gaps → `motion_possible:
false`. That is the basis for the C++ RT thread. Do not "simplify" the hot path back into
Python.

**Never log from a signal handler** — a re-entrant lock in the logging library deadlocked the
process.

**One Stream Motion peer per controller.** Two processes talking Stream Motion at once is a
hardware-level conflict, not a software error. Any tool in this repo that opens the driver must
be documented as exclusive.

---

## 7. Hardware preconditions (for the README's "running against a real robot" section)

- Controller option **S636** (J519 Stream Motion + R912 RMI). Verifiable over HTTP:
  `curl http://<controller>/MD/orderfil.dat | grep -E 'S636|J519|R912'`
- Controller in **AUTO**, drives powered, E-STOP released, errors reset
- No other Stream Motion peer
- `GRIPDISP` on controller flash if you want the gripper (see §5)

---

## 8. Test rig and verification

The suite is **entirely hardware-free**: `airo_fanuc.testing` ships an in-process `FakeCRX` — a
wall-paced 125 Hz controller emulation with an RMI server and a simple plant — so the real C++
RT core is exercised against it. That is why CI is worth having.

```bash
git clone --recursive <repo> && cd airo-fanuc
uv sync                                   # compiles the C++17 extension (scikit-build-core)
uv run pytest -q                          # expect 320 passed
uvx ruff@0.15.20 check src tests
uv run python examples/move_joints.py --fake   # zero-hardware end-to-end smoke
```

`examples/move_joints.py` is also the best single file to read to understand the API: it shows
the **2-knot rest-to-rest trajectory** idiom (`qd = [0, 0]`, the C++ core cubic-Hermite
interpolates between knots), which is how a point-to-point joint move is expressed given that
`move_trajectory` demands a fully time-parameterised trajectory (≥2 knots, strictly-increasing
int64 ns times, `|s·qd| ≤ v_lim`, `s ≤ 1.0`). There is no `move_to_joint_configuration`
primitive, and it does not need one.

Also present: byte-exact wire goldens, `tests/goldens/rmi/*.json` (14) and `goldens/sm/*.bin`
(10) — every basename is referenced from a test. Regenerate deliberately, never reflexively:
`AIRO_FANUC_REGEN_GOLDENS=1 uv run pytest tests/test_wire_stream_motion.py tests/test_wire_rmi_json.py`.

C++ tests exist behind `AIRO_FANUC_BUILD_TESTS` (OFF for the wheel, ON for a standalone
`cmake && ctest`), with a ThreadSanitizer suppressions file at `tests/tsan.supp`.

---

## 9. Tooling notes

- **`uv` for everything.** Never system `python` or `pip`.
- **ruff** config is already in `pyproject.toml`: `select = ["E", "F", "I", "UP", "B"]`,
  line-length 110. Stricter than the monorepo root, and it was clean — keep it that way.
- **`pytest-timeout` is a required dev dependency, not optional.** 17 tests carry
  `@pytest.mark.timeout` guards, including a self-deadlock guard that silently does nothing
  without the plugin — it would hang the suite instead of failing in 30 s. The marker is
  registered in `[tool.pytest.ini_options]`.
- **`uv run mypy` fails to start as configured** (`python_version = "3.10"` vs numpy 2.5 stubs).
  Pass `--python-version 3.12`, or fix the config. Worth fixing, because it currently hides one
  real stub gap: `StreamCore.submit_servo_ff` exists in the pybind bindings but is missing from
  `src/airo_fanuc/_core.pyi`.

---

## 10. Facts that are still unverified — do not present them as settled

`controller_facts.py` distinguishes measured facts from assumptions, and two safe-default
guesses are riding on unverified ground. Preserve that honesty:

- **`rmi_j3_plus_j2_conversion = True`** — the J2/J3 angle representation. Flagged
  **UNVERIFIED**; a wrong guess is a silent J2-sized FK error. Nobody knows whether this is
  CRX-specific or FANUC-wide.
- **`estop_continuation_path = "B"`** — path A is unproven.
- **`tx_silence_backstop_ok = False`** — this one *is* measured, and it is a NO-GO: on TX
  silence the controller does **not** fast-decelerate within 2–3 ITPs. Measured decel onset
  ~86.7 ms (≈10.8 ITPs) with 4.63° overrun at 49.9°/s. `deviation_watchdog_deg = 5.0` is sized
  from that worst overrun. Host-side watchdogs exist because the controller-side backstop
  cannot be relied on.

The provenance for all of these is a hardware probe day whose scripts and results stayed in the
monorepo (they were not copied here, by choice). So `controller_facts.py`'s comments are now the
sole record — do not thin them out, and if you restate a number in the README, cite the field
rather than the value.

### Open question: the acceleration limits contradict a first-party FANUC source

`controller_facts.py:51` states *"FANUC publishes only joint SPEEDS for the CRX"*, and on that
premise derives the rest:

```python
CRX10IAL_VELOCITY_LIMITS     = [2.094, 2.094, 3.142, 3.142, 3.142, 3.142]   # datasheet MDS-04018
CRX10IAL_ACCELERATION_LIMITS = 2.0 * CRX10IAL_VELOCITY_LIMITS               # derived
CRX10IAL_JERK_LIMITS         = 8.0 * CRX10IAL_ACCELERATION_LIMITS           # derived
```

**That premise is wrong.** FANUC publishes accelerations too, in a file sitting inside this
repo's own vendored submodule:
`vendor/fanuc_driver/fanuc_moveit_config/config/joint_limits.yaml`, © FANUC America Corporation
/ FANUC CORPORATION 2025, headed *"Joint limits for CRX-10iA and CRX-10iA/L"* — our exact arm.

|  | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| velocity, ours (rad/s) | 2.094 | 2.094 | 3.142 | 3.142 | 3.142 | 3.142 |
| velocity, FANUC | 2.0944 | 2.0944 | 3.1416 | 3.1416 | 3.1416 | 3.1416 |
| **accel, ours** (2×v) | 4.188 | 4.188 | 6.284 | 6.284 | 6.284 | 6.284 |
| **accel, FANUC** | **0.4** | **0.4** | **0.4** | **1.0** | **1.0** | **1.0** |
| ratio ours/FANUC | 10.5× | 10.5× | 15.7× | 6.3× | 6.3× | 6.3× |

The **velocities match exactly** — good independent confirmation of the datasheet values. The
**accelerations differ by 6–16×**, and because jerk is `8 × acceleration`, any error in the
accel base is multiplied straight through into the jerk limit.

Why this matters rather than being a curiosity: the monorepo's phantom contact-stops were caused
by **too much jerk** (the CRX collaborative-stop monitor reads a steep ramp as contact force),
and the fix was lowering accel/jerk. FANUC's own numbers being an order of magnitude lower
points in exactly that direction — i.e. the derivation may have been too aggressive from the
start.

**Do not simply swap the numbers in.** MoveIt `joint_limits.yaml` values are *planning* limits,
not necessarily hardware capability, and the uniform 0.4 across J1–J3 / 1.0 across J4–J6 looks
like engineering judgement rather than a datasheet transcription. Taken literally, 0.4 rad/s²
would need ~5 s to reach J1's max velocity, which is implausibly slow for this arm. Treat it as
a lead:

1. Fix the false claim in the `controller_facts.py:51` comment regardless — it is the premise
   the derivation rests on, and it is disproven by a file in this repo.
2. Decide, with the numbers written down, whether the driver's clamps should move. Note the
   driver's limits are deliberately *looser* than the planner's (they are clamps, not targets),
   so "FANUC's planning limit" and "our clamp" are not the same quantity — that is part of why
   this needs a decision rather than an edit.
3. If in doubt, prefer the more conservative value on the motion path and say why in the
   comment.

---

## 11. Suggested order of work

1. `.gitmodules` + verify a `--recursive` clone builds (§4.1) — everything else is cosmetic if
   this is broken
2. Remove `test_grocery_shim.py`, confirm 320 tests, ruff clean (§4.2)
3. Fix the operator-facing `CMakeLists.txt` message and the dangling citations (§4.3)
4. `README.md` with the four-tier scope claim (§5)
5. CI (§4.5)
6. Reframe the two monorepo-shaped docs (§4.4)
7. Audit `deploy/` (§4.6)
8. Optionally fix the mypy config and close the `_core.pyi` gap (§9)
9. Correct the false "FANUC publishes only joint SPEEDS" premise in `controller_facts.py:51`,
   and decide whether the derived acceleration/jerk limits should change (§10) — this one
   touches the motion path, so decide it deliberately rather than in passing

Every item above is *finishing the extraction* — there is no substantial new feature work in
this repo. Do **not** add kinematics (§3), the airo-mono adapter layer, an `airo_robots` import,
or any dependency beyond numpy.
