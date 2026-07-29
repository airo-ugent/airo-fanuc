# SPDX-License-Identifier: Apache-2.0
#
# gtest suite for the Stream Motion codec. googletest is provided via
# FetchContent by the top-level CMakeLists (guarded behind AIRO_FANUC_BUILD_TESTS).

add_executable(test_codec_smoke test_codec_smoke.cpp)
target_link_libraries(test_codec_smoke PRIVATE fanuc_sm_codec GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_codec_smoke)

# ---------------------------------------------------------------------------
# Pure tick-engine math goldens. Each links the `tick_engine` static lib
# (defined in the top-level CMakeLists; resolved at generate time)
# and gtest_main. test_hermite is compiled with -ffp-contract=off so any Hermite
# value it recomputes inline matches the -ffp-contract=off engine bit-for-bit.
# ---------------------------------------------------------------------------
foreach(_t hermite brake capture servo slew settle)
  add_executable(test_${_t} test_${_t}.cpp)
  target_link_libraries(test_${_t} PRIVATE tick_engine GTest::gtest_main)
  gtest_discover_tests(test_${_t})
endforeach()

target_compile_options(test_hermite PRIVATE -ffp-contract=off)

# ---------------------------------------------------------------------------
# RT core unit + loopback tests. Each links the `rt_core` static lib (TickCore
# + RealtimeCore + Pll) and gtest_main. test_pll/test_epoch/test_modes/
# test_tick_core exercise the PURE TickCore + Pll; test_rt_loopback drives the
# full RealtimeCore over UDP loopback.
# ---------------------------------------------------------------------------
foreach(_t pll tick_core epoch modes rt_loopback)
  add_executable(test_${_t} test_${_t}.cpp)
  target_link_libraries(test_${_t} PRIVATE rt_core GTest::gtest_main)
  gtest_discover_tests(test_${_t})
endforeach()

# ---------------------------------------------------------------------------
# Additional goldens (gate autonomy, TRAJECTORY→BRAKE qdd-seed continuity,
# big-anchor int64 timestamps, epoch bump-table completeness). Same rt_core link
# surface as the tests above.
# ---------------------------------------------------------------------------
foreach(_t gate_autonomy traj_brake_continuity big_anchor bump_table)
  add_executable(test_${_t} test_${_t}.cpp)
  target_link_libraries(test_${_t} PRIVATE rt_core GTest::gtest_main)
  gtest_discover_tests(test_${_t})
endforeach()

# ---------------------------------------------------------------------------
# Same-tick STOP precedence: a stop_j drained in a tick outranks a
# trajectory/servo consumed in the SAME tick, so a preempt can never be lost to a
# submission that landed in the same ITP window. Same rt_core link.
# ---------------------------------------------------------------------------
add_executable(test_stop_precedence test_stop_precedence.cpp)
target_link_libraries(test_stop_precedence PRIVATE rt_core GTest::gtest_main)
gtest_discover_tests(test_stop_precedence)
