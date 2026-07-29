# SPDX-License-Identifier: Apache-2.0
#
# Standalone build for the airo_fanuc P2' timing spike.
#
# This CMake is DELIBERATELY independent of the parent airo_fanuc package build
# (per the P2' self-containment decision): it pulls in nothing from src/ or
# vendor/ and has no dependency on the C++ core, pybind11, or numpy. Build it on
# its own:
#
#   cd packages/airo_fanuc/spike
#   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
#   cmake --build build
#   ./build/rt_loop --help
#
# Requires: C++17 (gcc-11 verified), cmake >= 3.16, a Linux with epoll/timerfd.

cmake_minimum_required(VERSION 3.16)
project(airo_fanuc_rt_spike LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)  # keep GNU extensions (_GNU_SOURCE, sched_getcpu)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# -O2 (not -march=native) so a binary built on one host stays valid on another;
# the per-tick work is trivial and does not need aggressive vectorisation.
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

find_package(Threads REQUIRED)

add_executable(rt_loop rt_loop.cpp)
target_compile_options(rt_loop PRIVATE -Wall -Wextra)
target_link_libraries(rt_loop PRIVATE Threads::Threads)

# timerfd / clock_gettime live in glibc on modern Linux; link rt defensively for
# older toolchains. (harmless no-op on glibc >= 2.17)
target_link_libraries(rt_loop PRIVATE rt)
