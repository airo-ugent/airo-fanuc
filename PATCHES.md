# PATCHES.md — vendored `fanuc_driver`

Tracks every modification we carry against the upstream FANUC `fanuc_driver`
sources vendored under `vendor/fanuc_driver/`. The submodule points directly at
`FANUC-CORPORATION/fanuc_driver` and is pinned by commit to tag `v2.2.0`; no
`branch` is recorded in `.gitmodules`, so `git submodule update --remote` cannot
move the pin silently — bumps are deliberate (see the procedure below).

Tracking upstream directly is only possible while the patch set below is empty.
If a patch ever becomes necessary, the submodule must move to a fork carrying it.

## What we vendor

We vendor and compile **only the Stream Motion packet codec**:

| File | Role | Compiled? |
|---|---|---|
| `fanuc_libs/stream_motion/include/stream_motion/packets.hpp` | `#pragma pack(1)` wire structs (Command 344 B, RobotStatus-204 416 B, Start/Stop 8 B, Capability 28 B, ForceSensorConfig 16 B, …) | header-only |
| `fanuc_libs/stream_motion/include/stream_motion/byte_ops.hpp` | `swapBytesIfNeeded<T>` big-endian swap template | header-only |

Everything else in the upstream tree is **unused**:

- `fanuc_libs/stream_motion/src/stream.cpp` + `include/stream_motion/stream.hpp` —
  their UDP socket layer (depends on `sockpp`). We own the RT socket loop (P3), so this is not compiled.
- `fanuc_libs/stream_motion/src/byte_ops.cpp` — defines only `isLittleEndian()` and carries an
  **unused** `#include <fmt/format.h>`. We do NOT compile it (see build note B1); our codec supplies
  its own endianness probe. This avoids pulling `fmt` into the codec target.
- `fanuc_libs/fanuc_client`, `fanuc_libs/rmi` — RMI stays in Python (dries `RmiClient`); no C++ RMI target.
- All ROS2 packages (`fanuc_hardware_interface`, `fanuc_controllers`, `fanuc_msgs`, `fanuc_moveit_config`, …) — no ROS.
- No `Eigen`, no `reflect-cpp`, no `yaml-cpp`, no `sockpp` in our build (a configure-time D6 check
  asserts the two vendored headers we compile against are Eigen-free).

## Patch set

**EMPTY.** As of P0 we carry **zero source patches** against the vendored FANUC files —
the codec is consumed header-only and needs no edits. The SPDX headers on the vendored
files are intact and unmodified.

If a patch ever becomes necessary, record it here as:

> ### P-SM<n> — <one-line symptom>
> - **Files:** `<paths>`
> - **Symptom / why:** <what breaks without it>
> - **Change:** <what the patch does> (+ SPDX `// Modified by AIRO, <date>: <reason>` in the file)
> - **Upstream issue:** <link — file even though FANUC refuses external PRs; they reply in 2-6 days>
> - **Re-validation tests:** <test ids that must pass after the patch>

## Build-integration notes (NOT source patches)

- **B1 — `byte_ops.cpp` excluded from the codec target.** The vendored `byte_ops.cpp` only
  defines `stream_motion::isLittleEndian()` and includes `<fmt/format.h>` (unused). Compiling it
  would drag `fmt` into an otherwise dependency-free codec. Instead, the `fanuc_sm_codec` static
  target compiles our own thin wrapper TU (`src/cpp/codec/codec.cpp`) against the vendored headers,
  and that wrapper defines a local endianness probe. This keeps `swapBytesIfNeeded` calls out of the
  default-argument path that would ODR-use `isLittleEndian()`, so nothing from `byte_ops.cpp` is linked.
  This is a choice in **our** CMake/wrapper, not an edit to a vendored file → not a patch.

## Submodule bump procedure

1. Move the pin to the target upstream tag. If any P-SM patch exists, first create a fork
   branch off that tag, cherry-pick the patches, and repoint the submodule at the fork.
2. Re-run L0 (codec goldens) + L1 (C++ unit) + L2 (FakeCRX matrix); fix drift.
3. `sizeof` static-asserts in `tests/cpp/test_codec_smoke.cpp` guard silent struct-layout changes.
4. Dedicated PR + HIL retest before the pin moves on `main`.
