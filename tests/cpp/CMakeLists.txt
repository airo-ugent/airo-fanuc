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
# Everything that links the `rt_core` static lib (TickCore + RealtimeCore + Pll)
# and gtest_main. Most drive the PURE TickCore; test_rt_loopback and
# test_rx_finiteness drive the full RealtimeCore over UDP loopback. Each file's
# own header comment says what it covers and why — restating it here would be a
# second copy that only this list can make stale.
# ---------------------------------------------------------------------------
foreach(_t pll tick_core epoch modes rt_loopback seqlock
         gate_autonomy traj_brake_continuity big_anchor bump_table
         stop_precedence rx_finiteness latch_and_handles motion_profile capture_seam)
  add_executable(test_${_t} test_${_t}.cpp)
  target_link_libraries(test_${_t} PRIVATE rt_core GTest::gtest_main)
  gtest_discover_tests(test_${_t})
endforeach()

