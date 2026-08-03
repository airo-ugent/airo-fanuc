# airo-fanuc

A standalone driver for FANUC controllers: a C++17 real-time core speaking **Stream Motion**
(UDP, 125 Hz / 8 ms ITP) plus an **RMI** JSON client (TCP 16001), behind a Python API.

The Python surface is split the conventional way for an industrial-arm client library: a
*receive* half (state getters that never block, never raise, and never lie — every value comes
with its age) and a *control* half (motion submission, usable blocking or non-blocking).
`FanucDriver` presents both halves on one construct-and-go object: the constructor brings the
robot up to *commandable* or raises with a real reason, `move_trajectory` / `move_j` / `servo_j`
are the only motion surfaces, and `stop_j()` is the universal preempt.

Everything timing-critical lives in C++: the 125 Hz tick loop, cubic-Hermite trajectory
playback, the Ruckig-based brake/servo/capture profiles, the slew clip and the safety state
machine (`src/cpp/rt_core/`, `src/cpp/tick_engine/`). Python owns lifecycle, recovery policy,
validation and the RMI session.

---

## What is vendored, and what is ours

The line is drawn at the wire: **vendor FANUC's protocol codecs, write our own real-time loop.**

This repo vendors FANUC's `fanuc_driver` as a submodule and compiles exactly two of its headers —
the `#pragma pack(1)` Stream Motion wire structs and the byte-swap template. A packet layout has
to match the controller byte for byte, and a header published by the protocol's own implementer
is the authoritative statement of that layout; compiling it directly means the struct we send
cannot drift from a transcription of ours. `PATCHES.md` records exactly what is vendored, that
the patch set against it is empty, and why nothing else upstream is compiled. Attribution is in
`NOTICE`.

Nothing above the codec is adopted. FANUC's driver reaches ROS 2 through a general-purpose
client layer and a `ros2_control` `SystemInterface`; this stack is not ROS, and the real-time
loop, the safety state machine and the interpolation layer are the parts an application with its
own planner has to own — so they are written here, in C++, on the tick. Same protocol work
underneath, different thing built on top.

---

## Which robots this actually works on

Be precise about this before you plan a bring-up. Four tiers, from portable to not portable at
all.

### Generic across FANUC

Stream Motion and RMI are FANUC's own protocols, not a CRX feature. What is required is
controller option **S636** (the External Control Package, which bundles J519 Stream Motion +
R912 RMI). Default endpoints are UDP 60015 and TCP 16001 (`DriverConfig.sm_port` /
`rmi_port`), and the negotiated Stream Motion version is configurable via
`DriverConfig.sm_version` (3 = no force telemetry, 4 = FSConfig/force). On v3 the controller
streams type-202 packets, `get_wrench()` returns `None`, and a numeric `force_stop_n` is
rejected with a typed error rather than armed as a silent no-op.

The 8 ms ITP is an **R-30iB** fact, not a collaborative-robot fact — that is FANUC's mainstream
controller class, so this covers many arms rather than only cobots. It is also a config field
(`DriverConfig.itp_s`, defaulting to `controller_facts.ITP_S`) rather than something to edit in
the package, and bring-up refuses to run if it disagrees with the period the controller reports
in its GetCapability reply — every per-tick quantity is scaled by it, so a mismatch means limits
that are silently wrong by that ratio.

### Injected, with no default: the arm

The arm's motion envelope — velocity, acceleration and jerk clamps plus joint position limits —
is a `RobotProfile`, and `DriverConfig(profile=...)` **requires** one. The package ships the type
and no instance, deliberately: those limits are the ceiling the RT core clamps against, this
driver cannot ask the controller which robot is attached, and a default would silently apply
limits somebody else measured on a different arm.

Most of a profile does not have to be written by hand, because the controller already knows the
answer. It serves its own configuration over FTP, and `airo_fanuc.controller_probe` reads it:

```bash
python -m airo_fanuc.controller_probe --ip 192.168.1.100 --emit-profile
```

That prints the controller's model, software P-level, ordered options (so **S636** presence is a
lookup rather than a claim), which teach-pendant programs are installed, and its **active joint
velocity and position limits** — followed by a paste-ready `RobotProfile`. Read-only: it fetches
diagnostic files from the virtual `md:` device and writes nothing, touching neither motion nor
RMI nor Stream Motion. What it *cannot* supply is the acceleration and jerk clamps, because the
controller publishes no clamp equivalent; those are derived from the measured velocity by an
explicit rule and the profile's `source` string records which half is which.

`DriverPolicy(preflight_full=True)` closes the loop at bring-up: it re-reads the controller, bands
the P-level, checks the option and TP programs, and warns when the supplied profile has drifted
from the limits the controller says it is enforcing — naming the direction, since a profile
*wider* than the controller is the dangerous one. It flags and never adopts: the controller is
authoritative about what it will allow, but overwriting a configured clamp mid-bring-up would
change the envelope under a caller who never asked. The probe is deliberately off the per-connect
path — `symotn.va` is ~650 kB, about 3 s on a point-to-point link.

`examples/crx10ial.py` is the written-down result for the CRX-10iA/L this driver has been run
against, and is the file to copy for another model. Keeping it in the repo rather than probing at
start-up is what makes a `--fake` run possible with no controller to ask, the numbers reviewable
in a diff, and a run reproducible against a controller whose settings have since changed.

Read it before trusting the numbers anywhere. In summary: velocity is the controller's own
`$PARAM_GROUP` value, which happens to match FANUC Europe datasheet **MDS-04018** exactly;
acceleration and jerk are engineering *derivations* from it (2× velocity and 8× acceleration).
FANUC does publish accelerations for this
arm too — in a file inside this repo's own vendored submodule,
`vendor/fanuc_driver/fanuc_moveit_config/config/joint_limits.yaml` — and those are 6–16× lower,
with velocities matching exactly. The profile records why that is not simply a bug to patch:
FANUC's figures are *planning* limits (a profile a planner shapes trajectories to) while these
are *clamps* (the ceiling above which the RT core refuses to pass a command through), so they
are deliberately looser. Whether the gap is the right size is an **open question** flagged there,
to be resolved by measuring this controller rather than by picking one of the two numbers.

One related caveat, also documented there: a planner feeding this driver should shape
trajectories with a *softer* jerk than the clamp (~3× acceleration rather than 8×). The CRX
collaborative-stop monitor infers contact force from motor disturbance torque, so a sharp jerk
ramp reads as a phantom contact mid-transit. Jerk is the trip trigger; acceleration is not.

Profiles are written in degrees (`RobotProfile.from_degrees`), the unit of both the datasheet and
the controller's system variables, and stored in radians. `DriverConfig.to_rt_core_config()`
carries them into the C++ tick engine, which is the thing that actually enforces them; the C++
struct's own defaults are a synthetic envelope that exists only so the tick-engine math is
testable stand-alone.

What stays in the package is everything that is *not* a property of the arm:
`controller_facts.py` holds the controller-class protocol facts (the 8 ms interpolation period,
the Stream Motion dataStyle word), the facts measured on our specific controller, and this
driver's own tuning — the brake scales, the capture window, the RX-silence ladder and watchdog
thresholds, which are fractions of a profile's limits rather than limits themselves and so carry
across arms unchanged.

### Baked in: six joints

`kNumJoints = 6` is a **C++ compile-time constant** (`src/cpp/tick_engine/tick_engine_config.hpp`,
mirrored by `_NDOF = 6` in `driver.py`). Fine for almost every FANUC arm, but it does exclude
7-axis arms and extended / positioner axes.

The reason is deliberate rather than incidental: the online trajectory generator's
degrees-of-freedom is a compile-time template parameter (`ruckig::Ruckig<kNumJoints>` in
`src/cpp/tick_engine/brake.hpp`), and the engine's per-joint type is
`Vec6 = std::array<double, kNumJoints>`, so the whole tick path is stack-allocated and
allocation-free. Making the DOF count dynamic means giving that up on the 8 ms path. (The Stream
Motion wire itself carries nine axis slots; the trailing three are zero-filled.)

### Not portable at all: the gripper

`src/airo_fanuc/gripper.py` and `gripper_worker.py` drive one specific teach-pendant program,
`GRIPDISP`, by writing RMI registers (R[1] trigger, R[2] action, R[3] modifier) and polling R[1]
for completion. **GRIPDISP lives only on controller flash and is not version-controlled here** —
only its RUN-fork launcher, `deploy/GRPRUN.LS`, is in this repo. For anyone else this is a
**site-installation prerequisite, not a Python dependency**: no `pip`/`uv` invocation can supply
it, and without it you run the driver with `DriverPolicy(enable_gripper=False)`.

The register contract is also **action-dependent** and coarse. On open, R[3] selects a width
bucket (`OPEN_FULL` / `OPEN_MID` / `OPEN_NARROW`); on close it selects a force class
(`FORCE_LIGHT` / `FORCE_MEDIUM` / `FORCE_HARD`). Three discrete buckets each — there is no
continuous width command and **no width feedback at all**. The only readable gripper state is
R[1] (mid-action vs done), which is why completion means "the TP program cleared the trigger",
not "a measured width was reached".

---

## Install, build, test

`uv` for everything — never system `python`, never `pip`. The one detail worth knowing up front:
the dev tools are an **extra** (`[project.optional-dependencies] dev`), not a dependency group,
so a plain `uv sync` does *not* install pytest. Use `--extra dev`.

```bash
git clone --recursive <repo-url> && cd airo-fanuc
uv sync --extra dev            # compiles the C++17 extension (scikit-build-core + CMake)
uv run pytest -q               # 328 tests, all hardware-free
uvx ruff check src tests
uv run mypy
uv run python examples/move_joints.py --fake   # zero-hardware end-to-end smoke
```

`--recursive` is not optional: `vendor/fanuc_driver` is a **build dependency**, and CMake fails
at configure time with an explicit `git submodule update --init vendor/fanuc_driver` message
without it.

One trap when working on the C++ side: **editing C++ does not rebuild the extension.** The
editable install redirects imports to `src/`, so Python edits are picked up immediately, but
`uv run pytest` will happily test the previously compiled `_core`. After touching anything under
`src/cpp/`, force a rebuild:

```bash
uv sync --extra dev --reinstall-package airo-fanuc
```

Otherwise a passing suite means only that the *old* binary still passes.

The whole Python suite is hardware-free because `airo_fanuc.testing` ships an in-process
`FakeCRX` — a wall-paced 125 Hz controller emulation with an RMI server and a first-order-lag
plant — so the real C++ RT core is exercised against it, plus byte-exact wire goldens under
`tests/goldens/`. `airo_fanuc.testing` ships in the wheel, so downstream code can be tested the
same way.

The C++ gtest suite is separate and the wheel build does **not** compile it
(`AIRO_FANUC_BUILD_TESTS=OFF` via `[tool.scikit-build.cmake.define]`). For a standalone build it
defaults to ON:

```bash
cmake -S . -B build/cpp -DAIRO_FANUC_BUILD_TESTS=ON \
      -DPython_EXECUTABLE="$(readlink -f .venv/bin/python)"
cmake --build build/cpp -j
ctest --test-dir build/cpp     # 95 tests
```

The `Python_EXECUTABLE` hint is only there because this build also configures pybind11, which
needs the interpreter's development headers; resolving the venv symlink points CMake at the real
installation. Drop it if `find_package(Python)` already finds a suitable one.

There is also a ThreadSanitizer variant, `-DAIRO_FANUC_TSAN=ON`, with a suppressions file at
`tests/tsan.supp` — the data-race gate for the RT core.

---

## Runtime dependencies: numpy, and only numpy

```toml
dependencies = ["numpy>=1.26"]
```

That is the entire runtime dependency list, and it is enforced statically.
`tests/test_standalone_contract.py` walks the AST of every file under `src/airo_fanuc/` and
asserts that (1) nothing imports a messaging middleware, a logging shim, a planner or a tensor
stack anywhere — including inside function bodies — and (2) the package's whole third-party
import surface equals its declared dependencies.

The check is AST-based rather than import-based on purpose: a package that merely happens to be
installed in the dev environment cannot satisfy an AST assertion, so the environment cannot
launder a violation.

Why it matters: a consumer installs this driver without inheriting a robotics stack. Everything
environment-specific is injected instead — the collision-check hook
(`DriverPolicy.capture_check`), a state sink (`DriverPolicy.publisher`, duck-typed), logging via
stdlib `logging`. Adding a dependency here is a decision, not a convenience.

---

## No kinematics — a scope decision

This package contains no FK, no IK, no URDF and no link geometry, and that is deliberate. Stream
Motion speaks joint angles, RMI speaks joint angles, and the safety-critical work here is
timing, fault handling and the collision-check splice. Geometric modelling is a different
concern that lives one layer up.

FANUC's own stack draws the same line: their driver ships no kinematics implementation, only a
solver *configuration* pointing MoveIt at KDL, with the robot geometry in a separate,
un-vendored description package. Keeping the hardware interface and the kinematic model apart
matches that architecture rather than fighting it.

Note this is **not** a dependency argument. Hand-rolled FK for a 6-DOF serial arm and a
damped-least-squares IK would add no dependency at all; the numpy-only contract rules out a
solver *library*, not kinematics as such. The argument is scope — and that baking in link
geometry would make an otherwise arm-agnostic package newly arm-specific.

The consumer supplies kinematics. The natural seam is `DriverPolicy.capture_check`, which
receives the exact synthesized splice knots the RT core will execute (from
`airo_fanuc._core.generate_capture_path` — the checked path is the executed path) and returns
whether they are safe. If you find yourself wanting `get_tcp_pose()` in this package, you are
writing the consumer layer in the wrong repo.

---

## Hardware preconditions

Before running against a real robot:

- **Controller option S636** (J519 Stream Motion + R912 RMI). It is verified once on the bench
  and recorded in `docs/controller-notes.md` §1.1 rather than re-fetched on every connect;
  `run_preflight(rmi, full=True)` is the hook for re-reading the P-level / S636 / orderfile facts
  off the controller's file server, and currently reports that it is not implemented. The
  per-connect gate does run on every bring-up: AUTO mode, servo ready, drives powered, general
  override, DCS speed clamp, and alarm classification (SYST-348 / SYST-322 are hard blocks with
  operator instructions).
- **Controller in AUTO**, drives powered, E-stop released, alarms reset. T1/T2 is a soft
  DEGRADED warning, not a hard block.
- **No other Stream Motion peer.** One peer per controller is a hardware-level constraint, not a
  policy choice: a second peer receives no status at all, a foreign StopPacket silences the live
  session mid-motion, and a second RMI connect leaves the redirect port timing out. The driver
  enforces single ownership with an advisory `flock` (`src/airo_fanuc/ownership.py`) and fails
  loudly with `OwnershipError` naming the holder rather than racing it.
- **`GRIPDISP` on controller flash** if you want the gripper; otherwise
  `DriverPolicy(enable_gripper=False)`.

Two binding behaviours worth reading before an operator is at the pendant. An E-stop, a latched
controller alarm or an OPERATOR_REQUIRED (SYST-348) condition sets `MOTION_INHIBITED` the moment
the driver observes it, and motion methods raise `RobotFaultedError` until an explicit
`driver.arm()` — so `driver.recover()` returning True means "back to STREAMING", not "commandable",
and nothing may auto-`arm()` in a retry loop. And contact detection on a v3 controller relies on
the controller's collaborative contact-stop: the motion resolves `FAULTED` with reason
`CONTACT_STOP`, not on a numeric force threshold. The full set is in `docs/invariants.md`.

---

## Usage

`move_trajectory` takes a **fully time-parameterised** trajectory: at least 2 knots, strictly
increasing int64 nanosecond times (relative), joint positions and velocities in rad and rad/s.
Every validation failure raises a typed error naming the offending joint or knot.

A rest-to-rest point-to-point move can therefore be written as a **2-knot trajectory with zero
endpoint velocities** — the C++ core cubic-Hermite-interpolates between the knots. That shape is
fine for a slow nudge, but a single cubic segment steps the acceleration at both endpoints
(`a(t) = 6Δq/T²·(1 − 2t/T)`, so jerk is an impulse there), which the core clips against the jerk
clamp. `driver.move_j(q, joint_speed=...)` is the real point-to-point path: it shapes a
jerk-limited profile from the current commanded state with `_core.plan_joint_move` (offline
Ruckig, under this driver's own limits) and submits the knots through `move_trajectory`, so it
inherits the capture gate, the collision hook and the settle policy unchanged. `joint_speed` is
the **leading-axis** speed — it caps every joint and the profile is time-synchronised, so the
joint travelling furthest runs at it and the rest scale down to land together.

```python
import numpy as np
from airo_fanuc import DriverConfig, DriverPolicy, FanucDriver, MotionResult

# The arm's envelope has no default and must be supplied — see "Injected, with no
# default" above. `examples/crx10ial.py` is a written-down CRX-10iA/L profile; for
# another model, emit one with `python -m airo_fanuc.controller_probe --emit-profile`.
from crx10ial import CRX10IAL

policy = DriverPolicy(config=DriverConfig(profile=CRX10IAL), enable_gripper=False)

# construct-and-go: blocks until the robot is commandable, or raises with a real reason
driver = FanucDriver("192.168.1.100", policy)
try:
    # start from the current *commanded* pose so the capture splice is a no-op
    q0 = np.asarray(driver.get_state()["q_cmd"], dtype=float)[:6]
    q1 = q0.copy()
    q1[5] += np.radians(10.0)                 # J6 by +10 deg

    times = [0, int(4.0 * 1e9)]               # int64 ns, relative, strictly increasing
    q = [q0.tolist(), q1.tolist()]            # (2, 6) rad
    qd = [[0.0] * 6, [0.0] * 6]               # rest to rest

    result = driver.move_trajectory(times, q, qd).wait(timeout=9.0)
    print(result, np.degrees(driver.get_state()["q_meas"][:6]))
    assert result == MotionResult.DONE
finally:
    driver.close()
```

`move_trajectory` returns a `MotionHandle` immediately; `.wait(timeout)` is the blocking form and
resolves to a non-raising `MotionResult` (`DONE`, `SETTLE_TIMEOUT`, `STOPPED`, `PREEMPTED`,
`FAULTED`, `REJECTED`). Skip `.wait()` for the non-blocking form and poll `.done()` /
`.result()`. `driver.stop_j()` is callable from any thread, takes effect within one tick, never
raises, and resolves a waiting handle as `STOPPED` — a clean preempt, not a fault, so the caller
can replan from rest. For streaming an externally planned
sequence, `driver.servo_j(q, duration)` is the replace-not-queue path. It is **best-effort** in
the sense UR's `servoj` is: head for this pose, with this long to do it, tracked under the servo
limits with no promise of arriving on time or at rest, and no rejection of a target for being far
away — falling behind means keep going, not stop. `duration` is the spacing between targets, not
the tick. The caller owns the loop, which has two consequences worth knowing: `stop_j()` preempts
the targets already submitted but *not* the ones the loop sends next, and ending a stream means
letting it drain onto its last target and then calling `driver.hold()` (stop feeding alone leaves
the core resting in SERVO indefinitely). Unlike `move_trajectory` there is no collision-check hook
anywhere on the servo path, so setpoint sanity is the caller's job.

Runnable versions of the above, including an offline `--fake` mode that needs no hardware:

```bash
uv run python examples/move_joints.py --fake      # single-joint rest-to-rest move
uv run python examples/sine_wave.py --fake        # dense-knot trajectory, all joints
uv run python examples/servo_stream.py --fake     # the same path streamed via servo_j
```

All three are also bring-up validation scripts: each one reports what the controller said, moves,
and ends in a `PASS`/`FAIL` verdict with the real-time loop's measured timing.
[`examples/README.md`](examples/README.md) is the step-by-step procedure for a first run against a
real controller.

---

## Repository layout

| Path | What |
|---|---|
| `src/airo_fanuc/` | Python API: driver, config, lifecycle/supervisor, RMI client, preflight, ownership, gripper, receive interface |
| `src/airo_fanuc/testing/` | `FakeCRX` controller emulation + byte-exact wire oracle (ships in the wheel) |
| `src/cpp/rt_core/` | RT thread: timerfd/epoll, PLL clock, seqlock snapshot, sockets |
| `src/cpp/tick_engine/` | I/O-free tick math: Hermite, brake, capture, servo, slew, settle |
| `src/cpp/codec/` | Wrapper TU compiled against the vendored FANUC Stream Motion headers |
| `vendor/fanuc_driver/` | FANUC's driver as a submodule; two headers compiled (see `PATCHES.md`) |
| `examples/` | Runnable validation scripts (`--fake` or real hardware) + the bring-up procedure |
| `docs/invariants.md` | The driver's binding safety / motion invariants |
| `docs/controller-notes.md` | Measured controller behaviour, alarm texts, recovery procedures |
| `deploy/GRPRUN.LS` | Teach-pendant launcher that RUN-forks `GRIPDISP` |

## License

Apache-2.0 (`LICENSES/Apache-2.0.txt`). The vendored FANUC Stream Motion codec headers are
Apache-2.0 and attributed in `NOTICE`; modifications to them: none — see `PATCHES.md`.
