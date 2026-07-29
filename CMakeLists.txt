# SPDX-License-Identifier: Apache-2.0
#
# airo_fanuc — C++17 FANUC CRX-10iA/L Stream Motion driver core + pybind11.
# Builds the full driver stack: the vendored Stream Motion packet codec (structs
# only, no sockets / no Eigen / no RMI), the tick_engine (Ruckig cubic-Hermite /
# brake / servo / settle / slew) and the rt_core (PLL + RealtimeCore), and exposes
# the codec + the real-time StreamCore + generate_capture_path through the
# `airo_fanuc._core` module.
#
# Design refs: docs/reference/airo-fanuc/PLAN.md §6,
# docs/reference/airo-fanuc/reports/09-design-packaging-migration-testing.md (§2.3).

cmake_minimum_required(VERSION 3.22)  # exact host minimum (cmake 3.22.1)
project(airo_fanuc LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)  # C++17 — reflect-cpp/C++20 dropped with the rmi lib (D2)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # the codec is linked into a shared module

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------------------
# ThreadSanitizer variant (P3c; B11 race gate). OFF by default so neither the
# wheel build nor the normal ctest build is affected. When ON, every target
# below (codec/tick_engine/rt_core/_core + the gtest suite) is instrumented, so
# the L1/L2-lite gtests run under TSan. Must be set BEFORE any target is defined
# (add_compile_options only applies to targets declared afterwards). The seqlock
# StateSnapshot payload + the RT-owned timing histograms race by design; scope
# those away with tests/tsan.supp (TSAN_OPTIONS=suppressions=...), NOT by
# disabling TSan. -O1 + frame pointers keep the reports symbolic.
# ---------------------------------------------------------------------------
option(AIRO_FANUC_TSAN "Build the C++ targets + gtest suite with ThreadSanitizer" OFF)
if(AIRO_FANUC_TSAN)
  message(STATUS "airo_fanuc: ThreadSanitizer ENABLED (-fsanitize=thread).")
  add_compile_options(-fsanitize=thread -g -O1 -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
endif()

include(FetchContent)

# ---------------------------------------------------------------------------
# Python hint normalisation.
#
# The standalone verify command passes -DPython3_EXECUTABLE / -DPYTHON_EXECUTABLE;
# scikit-build-core passes Python_EXECUTABLE. pybind11 (PYBIND11_FINDPYTHON=ON,
# recommended by scikit-build-core) drives find_package(Python) which honours
# Python_EXECUTABLE — normalise so all three invocation styles agree.
# ---------------------------------------------------------------------------
if(NOT DEFINED Python_EXECUTABLE)
  if(DEFINED PYTHON_EXECUTABLE)
    set(Python_EXECUTABLE "${PYTHON_EXECUTABLE}")
  elseif(DEFINED Python3_EXECUTABLE)
    set(Python_EXECUTABLE "${Python3_EXECUTABLE}")
  endif()
endif()
set(PYBIND11_FINDPYTHON ON)

# ---------------------------------------------------------------------------
# Vendored FANUC driver source location (D3 — kept behind a cache var so the
# submodule can be relocated/pointed elsewhere without editing this file).
# ---------------------------------------------------------------------------
set(FANUC_DRIVER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/fanuc_driver"
    CACHE PATH "Path to the vendored fanuc_driver fork (in-package submodule by default).")
set(FANUC_LIBS "${FANUC_DRIVER_SOURCE_DIR}/fanuc_libs")
set(FANUC_SM_INCLUDE "${FANUC_LIBS}/stream_motion/include")

set(_sm_packets "${FANUC_SM_INCLUDE}/stream_motion/packets.hpp")
set(_sm_byteops "${FANUC_SM_INCLUDE}/stream_motion/byte_ops.hpp")
if(NOT EXISTS "${_sm_packets}")
  message(FATAL_ERROR
    "Vendored Stream Motion codec not found at ${_sm_packets}. "
    "Run: git submodule update --init vendor/fanuc_driver")
endif()

# ---------------------------------------------------------------------------
# D6: verify at configure time that the codec headers we compile pull in NO
# Eigen. We vendor ONLY packets.hpp (structs) + byte_ops.hpp (swap template);
# both must remain dependency-light (<array>, <cstdint>, <algorithm>).
# ---------------------------------------------------------------------------
foreach(_hdr "${_sm_packets}" "${_sm_byteops}")
  file(READ "${_hdr}" _hdr_contents)
  string(FIND "${_hdr_contents}" "Eigen" _eigen_pos)
  if(NOT _eigen_pos EQUAL -1)
    message(FATAL_ERROR "D6 violation: Eigen reference found in vendored codec header ${_hdr}")
  endif()
endforeach()
message(STATUS "airo_fanuc: vendored Stream Motion codec is Eigen-free (D6 check passed).")

# ---------------------------------------------------------------------------
# fanuc_sm_codec — the Stream Motion codec as OUR static target (D2).
#
# The vendored codec is effectively header-only: packets.hpp holds the
# #pragma pack(1) wire structs and byte_ops.hpp holds the swap template. We do
# NOT compile their byte_ops.cpp (it only defines isLittleEndian() and carries
# an unused <fmt/format.h> include) nor stream.cpp (sockpp sockets). This target
# compiles our thin wrapper translation unit against the vendored headers, so
# the codec logic (struct layout + byte swap) is exercised as a real static lib.
# ---------------------------------------------------------------------------
add_library(fanuc_sm_codec STATIC
  src/cpp/codec/codec.cpp
)
target_include_directories(fanuc_sm_codec PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cpp"  # so #include "codec/codec.hpp" resolves
  "${FANUC_SM_INCLUDE}"                  # so #include "stream_motion/packets.hpp" resolves
)
target_compile_features(fanuc_sm_codec PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# pybind11 (pinned) → airo_fanuc._core module.
# ---------------------------------------------------------------------------
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.6
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pybind11)

# scikit-build-core exposes the CMake-safe version (X.Y.Z) in
# SKBUILD_PROJECT_VERSION and the full PEP 440 version (e.g. 0.1.0.dev0) in
# SKBUILD_PROJECT_VERSION_FULL — prefer the latter so the module __version__
# matches the installed distribution version exactly.
if(DEFINED SKBUILD_PROJECT_VERSION_FULL)
  set(AIRO_FANUC_VERSION "${SKBUILD_PROJECT_VERSION_FULL}")
elseif(DEFINED SKBUILD_PROJECT_VERSION)
  set(AIRO_FANUC_VERSION "${SKBUILD_PROJECT_VERSION}")
else()
  set(AIRO_FANUC_VERSION "0.1.0.dev0")
endif()

pybind11_add_module(_core MODULE src/cpp/bindings.cpp)
target_link_libraries(_core PRIVATE fanuc_sm_codec)
target_compile_definitions(_core PRIVATE AIRO_FANUC_CORE_VERSION="${AIRO_FANUC_VERSION}")

# scikit-build-core installs CMake outputs into the wheel; place _core inside
# the import package so it lands at airo_fanuc/_core*.so.
install(TARGETS _core LIBRARY DESTINATION airo_fanuc)

# ---------------------------------------------------------------------------
# C++ tests (gtest). Default ON for standalone builds so `ctest` has targets;
# the wheel build turns this OFF via [tool.scikit-build.cmake.define] to skip
# the googletest fetch (the codec runtime never needs gtest).
# ---------------------------------------------------------------------------
option(AIRO_FANUC_BUILD_TESTS "Build the C++ gtest suite" ON)
if(AIRO_FANUC_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  add_subdirectory(tests/cpp)
endif()

# ---------------------------------------------------------------------------
# --- P3a/P3b add Ruckig FetchContent + rt_core/tick_engine sources below ---
#
# P3a lands here: Ruckig (pinned to v0.17.3, matching the repo's Python ruckig
# 0.17.3) + the pure, I/O-free `tick_engine` static lib (cubic-Hermite resample,
# Ruckig brake/servo/capture, slew clip, settle, qd_end blend). P3b adds the
# rt_core (sockets/timerfd/threads) on top and links tick_engine into it.
# The codec target, Python hint normalisation, pybind11 pin, install rule and
# test gating above are reused unchanged.
# ---------------------------------------------------------------------------

# Ruckig, pinned to v0.17.3 (== the repo's Python `ruckig` so brake/servo/capture
# math is identical to the Python side). DOF-templated Ruckig<6> is used
# everywhere so the tick path is stack-allocated / allocation-free
# (StandardVector<double,6> == std::array<double,6>). Build only the library:
# examples/tests/benchmark/cloud-client/python-module OFF, and static so it links
# into our static tick_engine + the _core module (PIC is on globally, line 16).
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  ruckig
  GIT_REPOSITORY https://github.com/pantor/ruckig.git
  GIT_TAG        v0.17.3
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(ruckig)

# tick_engine — the pure tick-engine math (no sockets, no threads, no clocks).
add_library(tick_engine STATIC
  src/cpp/tick_engine/hermite.cpp
  src/cpp/tick_engine/brake.cpp
  src/cpp/tick_engine/capture.cpp
  src/cpp/tick_engine/servo.cpp
  src/cpp/tick_engine/slew.cpp
  src/cpp/tick_engine/settle.cpp
)
target_include_directories(tick_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/cpp")
target_link_libraries(tick_engine PUBLIC fanuc_sm_codec ruckig::ruckig)
target_compile_features(tick_engine PUBLIC cxx_std_17)
# Bit-exact cubic-Hermite vs the dries Python oracle requires NO FP contraction:
# an FMA fusing `a*b + c` would re-round and diverge from interpolator.py, which
# evaluates each `*` and `+` as a separately-rounded IEEE op. PRIVATE so we do
# not perturb P0's _core / bindings compilation.
target_compile_options(tick_engine PRIVATE -ffp-contract=off)

# ---------------------------------------------------------------------------
# rt_core — the P3b real-time core: TickCore (pure I/O-free tick) + RealtimeCore
# (RT thread: epoll/timerfd/PLL-clocked TX/sockets, seqlock snapshot, rings).
# Links tick_engine (Hermite/brake/servo/capture/slew/settle) + fanuc_sm_codec
# (wire encode/decode). Depends on pthread (RT thread + affinity).
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)
add_library(rt_core STATIC
  src/cpp/rt_core/pll.cpp
  src/cpp/rt_core/tick_core.cpp
  src/cpp/rt_core/realtime_core.cpp
)
target_include_directories(rt_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/cpp")
target_link_libraries(rt_core PUBLIC tick_engine fanuc_sm_codec ruckig::ruckig Threads::Threads)
target_compile_features(rt_core PUBLIC cxx_std_17)

# Link the RT core into the Python module so StreamCore is importable.
target_link_libraries(_core PRIVATE rt_core)
