# Development

`uv` for everything — never system `python`, never `pip`.

```bash
git clone --recursive https://github.com/airo-ugent/airo-fanuc.git && cd airo-fanuc
uv sync --extra dev            # compiles the extension (scikit-build-core + CMake)
uv run pytest -q
```

`--recursive` is not optional: `vendor/fanuc_driver` is a **build dependency**, and CMake
fails at configure time with an explicit `git submodule update --init` message without it.

The dev tools are an **extra**, not a dependency group, so a plain `uv sync` installs no
pytest. Use `--extra dev`.

---

## The gates

These four are the lint and test gates CI runs — its `python` job also builds the extension
and checks the vendored header is present, a `dist` job builds the sdist and wheel and installs
the sdist into an empty venv, and a `cpp` job runs the C++ suite below:

```bash
uv run ruff check src tests examples
uv run ruff format --check src tests examples
uv run mypy
uv run pytest -q
```

`uv run ruff` rather than `uvx ruff` on purpose, so the version is the one pinned in the
`dev` extra and the config in `pyproject.toml` applies — which is what excludes `vendor/`
and `build/` from linting.

CI **lints** `examples/` but does not execute it: those scripts assert the real-time loop
held an 8 ms deadline, and a shared runner cannot be relied on to do that. Running the
`--fake` ladder is yours to do locally, and it is the pre-check before anything touches a
robot — see [`examples/README.md`](../examples/README.md).

### The C++ suite

Separate, and the *wheel* build does not compile it (`AIRO_FANUC_BUILD_TESTS=OFF`) — CI runs
it in a job of its own. For a standalone build the option defaults to ON:

```bash
cmake -S . -B build/cpp -DAIRO_FANUC_BUILD_TESTS=ON \
      -DPython_EXECUTABLE="$(readlink -f .venv/bin/python)"
cmake --build build/cpp -j
ctest --test-dir build/cpp
```

The `Python_EXECUTABLE` hint is only there because this build also configures pybind11,
which needs the interpreter's development headers; resolving the venv symlink points CMake
at the real installation. Drop it if `find_package(Python)` already finds a suitable one.

There is also a ThreadSanitizer variant, `-DAIRO_FANUC_TSAN=ON`, with a suppressions file
at `tests/tsan.supp`. That file is narrow on purpose — four named symbols covering two
by-design patterns TSan cannot model: the version-counter seqlocks (`Seqlock`,
`JointsAtRing`), where a reader retries on a torn copy, and the accepted-torn diagnostic
counters (`Histogram`, `RealtimeCore::timing`), where a torn bucket is a cosmetic percentile
blip and never a control decision. **Everything else must stay TSan-clean; a race outside
those four is a real finding, not a false positive.** Do not broaden the suppressions to make
a run pass — one test that deliberately provokes a benign race skips itself under
instrumentation instead, and says why in the file.

### The trap worth internalising

**Editing C++ does not rebuild the extension.** The editable install redirects imports to
`src/`, so Python edits are picked up immediately, but `uv run pytest` will happily test the
previously compiled `_core`:

```bash
uv sync --extra dev --reinstall-package airo-fanuc
```

Otherwise a passing suite means only that the *old* binary still passes.

---

## The test rig

The whole Python suite is hardware-free because `airo_fanuc.testing` ships an in-process
`FakeCRXController`: a wall-paced 125 Hz controller emulation with an RMI server and a
first-order-lag joint model. The **real** C++ RT core is exercised against it over real
loopback sockets — this is not a mock of the driver, it is a stand-in for the controller.

`airo_fanuc.testing` ships in the wheel, so downstream code can be tested the same way:

```python
from airo_fanuc.testing import FakeCRXConfig, FakeCRXController
```

Alongside it, `tests/goldens/` holds byte-exact wire goldens for both protocols, checked
against an independent codec implementation in `airo_fanuc.testing.wire`. That independence
is the point — the oracle re-derives the packet layout rather than importing the driver's
own encoder, so the two cannot agree by construction. **Goldens are regenerated
deliberately, never reflexively:** a golden that changed because the wire changed is either
a deliberate protocol decision or a bug, and regenerating hides the difference.

The fake controller keeps its own copy of the gripper register numbers for the same reason.
It stands in for the Teach-Pendant program; importing the driver's constants would make the
two sides unable to disagree, and a driver poking the wrong register is exactly what the
fake exists to catch.

---

## What a test here is expected to do

The bar is not coverage, it is **failing when the behaviour is removed.** Several tests in
this repo were rewritten after they turned out to pass against code that did not work:

- Config mirrors asserted against C++ defaults that were bit-identical to the Python
  constants they mirrored, so every assertion held against an empty config. They now perturb
  the source value first.
- The seqlock's sequence discipline is invisible to a single-threaded round trip — dropping
  the counter updates changes nothing until a reader and the writer overlap — so its test
  runs a real concurrent writer.

If you add a guard, mutate it and watch the test fail before you believe it. If you cannot
make it fail, the test is documentation, not a guard, and should say so.

---

## Repository layout

| Path | What |
|---|---|
| `src/airo_fanuc/` | the Python package: driver, config, lifecycle/supervisor, RMI client, preflight, ownership, gripper, receive interface |
| `src/airo_fanuc/testing/` | the `FakeCRX` controller emulation + the byte-exact wire oracle (ships in the wheel) |
| `src/cpp/rt_core/` | the RT thread: timerfd/epoll, PLL clock, seqlock snapshot, sockets, the mode/fault state machine |
| `src/cpp/tick_engine/` | I/O-free tick math: Hermite, brake, capture, servo, slew, settle |
| `src/cpp/codec/` | a wrapper translation unit compiled against the vendored FANUC headers |
| `src/cpp/bindings.cpp` | the pybind11 surface |
| `vendor/fanuc_driver/` | FANUC's driver as a submodule; two headers compiled (see `PATCHES.md`) |
| `tests/` | the hardware-free pytest suite, wire goldens, and the C++ gtests under `tests/cpp/` |
| `examples/` | the runnable validation ladder (`--fake` or real hardware) |
| `deploy/` | the Teach-Pendant launcher that RUN-forks the gripper dispatcher |
| `docs/` | this documentation |

---

## Releasing

Wheels are built by `.github/workflows/release.yml`, not by hand: a local build produces a
`linux_x86_64` tag that PyPI refuses, so the manylinux image is what makes a publishable
wheel. cp310-cp313 on x86_64 and aarch64, each imported and run against two test suites
before it is kept.

`workflow_dispatch` builds the matrix and keeps the artifacts without releasing anything.
Pushing a `v*` tag is what publishes, via PyPI trusted publishing — no API token lives in
this repository, and it requires a one-time publisher configured on PyPI for this repo,
workflow and environment.

The version lives in `pyproject.toml` and nowhere else: `airo_fanuc.__version__` reads it
from installed distribution metadata, and the standalone CMake build parses it out of the
manifest, so there is no second literal to forget.

---

See also: [architecture](architecture.md) for how the layers fit together, and
[invariants.md](invariants.md) before changing anything on the 8 ms path.
