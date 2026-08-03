# airo-fanuc — orientation for coding agents

A driver for FANUC industrial arms: a compiled real-time core speaking Stream Motion (UDP,
125 Hz / 8 ms) plus an RMI JSON client (TCP), behind a Python API. It commands a real 10 kg
arm, so parts of this repo are safety-relevant code rather than ordinary code.

**Read `docs/invariants.md` before changing anything under `src/cpp/` or in the motion path
of `src/airo_fanuc/driver.py`.** Read `examples/README.md` before touching `examples/`.

---

## Where things live

| Path | What, and what to be careful of |
|---|---|
| `src/cpp/tick_engine/` | The per-tick math. **I/O-free**: no clock, no socket, no thread. Keep it that way — it is why the motion path is testable as pure functions. |
| `src/cpp/rt_core/` | The RT thread (`realtime_core`) and the mode/fault state machine (`tick_core`). The 8 ms path. |
| `src/cpp/codec/` | Wire encode/decode, compiled against vendored FANUC headers. |
| `src/cpp/bindings.cpp` | The pybind11 surface. Also two *offline* planners that bypass the RT path. |
| `src/airo_fanuc/driver.py` | `FanucDriver` — the facade. Validation, the public surface. |
| `src/airo_fanuc/supervisor.py` | Bring-up and recovery ladders, fault classification, the ARM gate. |
| `src/airo_fanuc/config.py` | `DriverConfig` / `DriverPolicy`, and the mirror into the C++ config. |
| `src/airo_fanuc/controller_facts.py` | Measured facts and this driver's tuning. **Comments distinguish measured from assumed — do not thin them, and do not restate an unverified value as settled.** |
| `src/airo_fanuc/testing/` | The in-process controller emulation. Ships in the wheel. |
| `examples/` | An **ordered hardware validation ladder**, not documentation. See below. |
| `docs/` | Start at `docs/README.md`. |

Documentation depth lives in `docs/`: `architecture.md`, `api.md`, `configuration.md`,
`portability.md`, `safety.md`, `gripper.md`, `troubleshooting.md`, `development.md`,
`invariants.md`, `controller-notes.md`.

---

## Tooling

`uv` for everything. Never system `python`, never `pip`.

```bash
uv sync --extra dev            # dev tools are an EXTRA, not a group
uv run pytest -q
uv run ruff check src tests examples
uv run ruff format --check src tests examples
uv run mypy
```

Those four are the CI gates. All of them must pass.

The C++ suite is separate:

```bash
cmake -S . -B build/cpp -DAIRO_FANUC_BUILD_TESTS=ON \
      -DPython_EXECUTABLE="$(readlink -f .venv/bin/python)"
cmake --build build/cpp -j && ctest --test-dir build/cpp
```

`-DAIRO_FANUC_TSAN=ON` is the data-race gate, with suppressions at `tests/tsan.supp`. That
file covers three specific functions whose lock-free design ThreadSanitizer cannot model.
**Anything else it reports is a real finding. Never broaden the suppressions to make a run
pass.**

### Two traps that will waste your time

- **Editing C++ does not rebuild the extension.** `uv run pytest` happily tests the
  previously compiled `_core`. Run
  `uv sync --extra dev --reinstall-package airo-fanuc` after any change under `src/cpp/`, or
  a green suite means only that the old binary still passes.
- **`ctest` runs stale executables** if the build failed. Confirm the build succeeded before
  believing the result.

---

## Hard constraints

**One runtime dependency: numpy.** `dependencies = ["numpy>=1.26"]` is the entire list, and
`tests/test_standalone_contract.py` enforces it by walking the AST of every file in the
package — including imports inside function bodies. **Adding a runtime dependency is a
decision for the maintainer, never a convenience.** Everything environment-specific is
injected instead: the collision check, the state sink, the arm's envelope, the gripper
protocol.

**No kinematics.** No FK, no IK, no URDF, no link geometry — a scope decision, not a
dependency one. The two pose getters report poses the *controller* computed; nothing here
derives a pose from joint angles. If a task seems to need FK, it belongs in the consumer
layer.

**Controller-generic vs robot-specific.** The package keeps what is true of the FANUC
controller class. A specific arm's numbers live in a `RobotProfile` the caller injects
(`examples/crx10ial.py` is a worked one) and the package ships no instance. A specific
gripper's registers live in a `RegisterGripperProtocol`. Do not hardcode either into `src/`.

**Six joints is compile-time.** The joint count is a C++ template parameter, so the tick
path is allocation-free. A different count needs a rebuild, not a config change.

---

## Invariants, and the symptom of breaking each

The full statements are in `docs/invariants.md`. These are the ones whose breakage is
silent:

| Invariant | Symptom if broken |
|---|---|
| The command packet's reserved style word carries `0xFFFF` | The controller reads joint angles as a Cartesian pose, slews at full speed, and E-stops |
| The collision-checked path IS the executed path — the splice is synthesized once, in C++ | A checker approves knots the arm never executes; no error anywhere |
| `stop_j()` outranks a command submitted in the same 8 ms window | A stop is swallowed by a trajectory that raced it; the arm keeps going |
| The universal preempt is a clean stop, not a fault | Callers must run a recovery ladder to replan from rest |
| Motion stays inhibited after an E-stop until an explicit `arm()` | A retry loop resumes motion into a cell someone is standing in |
| Brake trajectories are planned from *planned*, not measured, velocity | The brake plans from a lagging measurement and overshoots |
| The trajectory clock advances exactly once per tick | Playback runs fast or slow, with nothing reporting it |
| Getters never raise, never block (one documented exception), and publish the age of anything stale | A stale value reads as fresh; a caller acts on a frozen feed |
| No numeric force guard on a controller with no force telemetry — typed rejection instead | A force limit is armed that can never trip |
| One Stream Motion peer per controller, enforced by an advisory lock | A second peer silences the live session mid-motion |
| The 125 Hz loop stays in C++; the RT thread never calls into Python | Missed deadlines; the controller coasts and drops motion capability |
| No logging or allocation from the RT thread or a signal handler | A latency spike where it matters most |
| The gripper dispatcher is probed once, then forked at most once | Stacked, un-killable TP tasks on the controller |

A test that cannot fail is not a guard. If you add one, mutate the behaviour it names and
watch it fail before believing it — several tests here were rewritten after they turned out
to pass against code that did not work.

---

## `examples/` is a hardware acceptance test

Treat it as safety-relevant code, not as documentation. It is a deliberately ordered
validation ladder that gets run against a physical arm, so:

- **Do not change motion semantics as part of a cleanup.** Default joint, amplitude, speed,
  ramp, trajectory shape, limit checks and safety gates stay as they are unless there is a
  defect. Tidying a comment or extracting a shared helper is fine.
- **Preserve the CLI surface.** Flag names and defaults are what an operator types.
- **Keep the ladder intact and in order**, and keep the hardware-free `--fake` path working
  — it is the pre-check before anything touches the robot.
- Every script must still import, parse its arguments and run its dry path. An example that
  only breaks with a robot attached is the worst failure mode here.

---

## Writing style in this repo

Comments say what the code does and why, justified by facts in the repo. They do not
describe what the code used to do, name predecessors, or narrate history — that is what git
is for. Brevity counts: a shorter comment that says the same thing is better.

Where a fact is unverified, the code says so. Do not upgrade a hedge into a claim without a
measurement to point at.
