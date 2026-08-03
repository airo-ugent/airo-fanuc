# airo-fanuc

A standalone driver for FANUC industrial arms: a compiled real-time core speaking **Stream
Motion** (UDP, 125 Hz / 8 ms) plus an **RMI** JSON client (TCP), behind a Python API.

The Python surface is split the conventional way for an industrial-arm client library: a
*receive* half — state getters that never block, never raise, and never lie, every value
carrying its age — and a *control* half, usable blocking or non-blocking. `FanucDriver`
presents both on one construct-and-go object: the constructor brings the robot up to
*commandable* or raises with a real reason, `move_trajectory` / `move_j` / `servo_j` are the
only motion surfaces, and `stop_j()` is the universal preempt.

Everything timing-critical is C++: the 125 Hz tick loop, cubic-Hermite trajectory playback,
the Ruckig-based brake/servo/capture profiles, the slew clip and the safety state machine.
Python owns lifecycle, recovery policy, validation and the RMI session, and is never in the
control loop.

**One runtime dependency: numpy.** Everything environment-specific is injected — the
collision check, a state sink, the arm's motion envelope, the gripper's register protocol —
so installing this driver does not drag in a robotics stack.

---

## Install

```bash
pip install airo-fanuc
```

A download on **Linux x86_64 and aarch64, CPython 3.10-3.13**: the published wheels carry
the compiled real-time core, so no compiler, no CMake and no headers are involved.

Linux only, and not a preference — the RT thread is `timerfd_create` + `epoll` +
`SCHED_FIFO`, with no portable fallback. Anywhere else, pip falls back to the sdist and
compiles the extension, which needs a C++20 compiler, CMake ≥ 3.22, the Python development
headers, and network access to fetch ruckig and pybind11.

The package ships **no arm profile** — you supply a `RobotProfile`, and most of it comes out
of the controller itself. See [configuration](docs/configuration.md).

## Will it work on your robot?

Stream Motion and RMI are FANUC's own protocols, not a collaborative-robot feature; what is
required is controller option **S636**. The 8 ms interpolation period is an R-30iB fact, so
this covers FANUC's mainstream controller class rather than only cobots.

Four things are not generic: the arm's envelope (injected, no default), the interpolation
period (configured, and bring-up refuses a mismatch), the six-joint count (compile-time),
and the gripper (parameterised, but its Teach-Pendant program is yours to install).
[**Read this before planning a bring-up.**](docs/portability.md)

Developed and measured against a FANUC CRX-10iA/L on an R-30iB-class controller.

## Usage

```python
import numpy as np
from airo_fanuc import DriverConfig, DriverPolicy, FanucDriver, MotionResult

from crx10ial import CRX10IAL   # your arm's RobotProfile — examples/crx10ial.py

policy = DriverPolicy(config=DriverConfig(profile=CRX10IAL), enable_gripper=False)

# construct-and-go: blocks until the robot is commandable, or raises with a real reason
with FanucDriver("192.168.1.100", policy) as driver:
    q0 = np.asarray(driver.get_state()["q_cmd"], dtype=float)
    q1 = q0.copy()
    q1[5] += np.radians(10.0)                       # J6 by +10 deg

    result = driver.move_j(q1, joint_speed=0.2).wait(timeout=9.0)
    assert result == MotionResult.DONE
```

`move_j` plans a jerk-limited point-to-point profile and submits it; `joint_speed` is the
leading-axis speed in rad/s, and every joint is time-synchronised to land together. It
requires the arm to be at rest. `move_trajectory(times, q, qd)` takes a fully
time-parameterised timeline instead — int64 nanoseconds, radians — and the C++ core owns
playback from submission, so a late Python thread costs nothing. `servo_j(q, duration)` is
the replace-not-queue streaming path, best-effort in the sense UR's `servoj` is.

Every motion returns a `MotionHandle` immediately. `.wait(timeout)` blocks and resolves to a
non-raising `MotionResult`; skip it and poll `.done()` / `.result()`.

Runnable, including a `--fake` mode that needs no hardware:

```bash
uv run python examples/move_joints.py --fake      # single-joint rest-to-rest move
uv run python examples/sine_wave.py --fake        # dense-knot trajectory, all joints
uv run python examples/servo_stream.py --fake     # the same path streamed via servo_j
```

Those are also bring-up validation scripts: each reports what the controller said, moves,
and ends in a PASS/FAIL verdict with the real-time loop's measured timing.

## Documentation

**[docs/README.md](docs/README.md)** is the index. The ones you are most likely to want:

- [**Portability**](docs/portability.md) — will this work on my robot, and what to check first
- [**API reference**](docs/api.md) — every method, what blocks, what it raises, every `get_state()` key
- [**Configuration**](docs/configuration.md) — `RobotProfile`, `DriverConfig`, `DriverPolicy`
- [**Safety**](docs/safety.md) — preconditions, the fault model, the ARM gate, limitations
- [**Gripper**](docs/gripper.md) — how to drive your own
- [**Troubleshooting**](docs/troubleshooting.md) — symptom-first
- [**Architecture**](docs/architecture.md) and [**Development**](docs/development.md) — if you are working on the driver
- [**`examples/README.md`**](examples/README.md) — the ordered bring-up ladder for a first run against real hardware

## License

Apache-2.0 — see `LICENSE`. The vendored FANUC Stream Motion codec headers are Apache-2.0
too, with no modifications (`PATCHES.md`). Ruckig (MIT) and pybind11 (BSD-3-Clause) are
compiled into the `_core` extension, so the wheel redistributes both; their texts are in
`LICENSES/` and every component is attributed in `NOTICE`.
