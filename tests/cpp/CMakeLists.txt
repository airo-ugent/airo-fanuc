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

# ---------------------------------------------------------------------------
# Ingest finiteness gate: a status packet carrying a non-finite wire float is
# dropped at ingest, so no NaN reaches the clamps and thresholds that cannot
# reject one. Drives the full RealtimeCore over UDP loopback, like
# test_rt_loopback, against a peer that poisons individual fields.
# ---------------------------------------------------------------------------
add_executable(test_rx_finiteness test_rx_finiteness.cpp)
target_link_libraries(test_rx_finiteness PRIVATE rt_core GTest::gtest_main)
gtest_discover_tests(test_rx_finiteness)

# ---------------------------------------------------------------------------
# Exactly-once tick-core invariants: one SUPERVISOR_LOST fault per heartbeat
# lapse, a stop_j answered in BRAKE, and a terminal event for every motion that
# consume displaces from the single active-motion slot.
# ---------------------------------------------------------------------------
add_executable(test_latch_and_handles test_latch_and_handles.cpp)
target_link_libraries(test_latch_and_handles PRIVATE rt_core GTest::gtest_main)
gtest_discover_tests(test_latch_and_handles)

# ---------------------------------------------------------------------------
# Commanded-position PROFILES across the CAPTURE splice head, the SAFE_FOLLOW fault
# ramp head, the qd_end blend handoff and the RX_SILENT park exit. These assert the
# SEQUENCE of commanded positions rather than any single state, because each defect
# they cover is "one tick commanded the pose it had already commanded" — invisible in
# every scalar the snapshot exposes. Same rt_core link surface as the tests above.
# ---------------------------------------------------------------------------
add_executable(test_motion_profile test_motion_profile.cpp)
target_link_libraries(test_motion_profile PRIVATE rt_core GTest::gtest_main)
gtest_discover_tests(test_motion_profile)

# ---------------------------------------------------------------------------
# The CAPTURE -> TRAJECTORY seam: the splice's sub-tick terminal residue, and the
# commanded travel the handoff tick carries at every grid phase. Asserts the position
# SEQUENCE, because the wire carries positions and the controller differentiates them.
# Links rt_core (which carries tick_engine) like the tests above.
# ---------------------------------------------------------------------------
add_executable(test_capture_seam test_capture_seam.cpp)
target_link_libraries(test_capture_seam PRIVATE rt_core GTest::gtest_main)
gtest_discover_tests(test_capture_seam)

# ---------------------------------------------------------------------------
# Joining a submitted plan at the phase the caller's declared staleness implies: that a
# continuation replan is joined with no splice at all, that a reachable gap still
# splices, and the staleness ceiling. Same rt_core link surface as the tests above.
# ---------------------------------------------------------------------------
add_executable(test_join_phase test_join_phase.cpp)
target_link_libraries(test_join_phase PRIVATE rt_core GTest::gtest_main)
gtest_discover_tests(test_join_phase)
