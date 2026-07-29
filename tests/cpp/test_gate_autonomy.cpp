// SPDX-License-Identifier: Apache-2.0
//
// Unit test — C++-gate-autonomy invariant. Every autonomy gate carried in
// a type-204 status reacts inside ONE TickCore::tick() call, computed purely from
// the RxSample, with NO Python and NO callback: TickCore has no callback surface
// at all, and RealtimeCore (the only caller) never invokes Python from the RT
// thread. This test is the regression wall against "simplifying" any gate back
// into a Python watchdog — the drift/e-stop/TEACH/contact/safety_scale/
// motion_possible reactions must stay GIL-free in the C++ tick.
//
// It also covers the gates that test_epoch/test_modes do not reach at the
// TickCore level: TEACH_MODE, SAFETY_CLAMP and MOTION_NOT_POSSIBLE. (DRIFT is a bound FaultReason enum value but is NOT wired as a C++ gate
// in the frozen core — it is derived host-side, plan@now−lag — so it is not part
// of the C++ autonomy table and is deliberately not asserted here.)

#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "rt_core/rt_core_config.hpp"
#include "rt_core/tick_core.hpp"

using namespace airo_fanuc::rt_core;
using airo_fanuc::tick_engine::Vec6;

namespace {

RxSample clean_rx(const Vec6& q) {
  RxSample rx{};
  rx.q_meas = q;
  rx.gates.motion_possible = true;  // all other gates clear
  rx.safety_scale = 1.0;
  return rx;
}

// Drive to steady HOLD (streaming, one clean RX baseline processed → gate edges
// are measured against a fully-clear baseline).
void init_holding(TickCore& tc, const Vec6& q0) {
  tc.begin_streaming(q0);
  RxSample rx = clean_rx(q0);
  tc.tick(&rx, nullptr);
}

// Whether the mode/fault reaction landed WITHOUT draining any event ring — i.e.
// the reaction is intrinsic to tick(), not deferred to a Python consumer.
struct GateCase {
  const char* name;
  void (*apply)(RxSample&);
  FaultReason fault;
  std::uint32_t cond_bit;
  BumpReason bump;
};

}  // namespace

// One tick with a single gate raised (from an all-clear baseline) must, purely
// from the RxSample: enter SAFE_FOLLOW, latch the right FaultReason, set the
// right condition bit, and bump the epoch under the right BumpReason — all
// before any event is drained.
TEST(GateAutonomy, EveryGateReactsInOneTickNoPython) {
  const std::array<GateCase, 6> cases{{
      {"e_stop", [](RxSample& r) { r.gates.e_stopped = true; }, FaultReason::E_STOP, kCondEStop,
       BumpReason::kEStopEdge},
      {"in_error", [](RxSample& r) { r.gates.in_error = true; }, FaultReason::IN_ERROR, kCondInError,
       BumpReason::kFaultEntry},
      {"teach", [](RxSample& r) { r.gates.tp_enabled = true; }, FaultReason::TEACH_MODE, kCondTeach,
       BumpReason::kKillDegradedEntry},
      {"contact_stop",
       [](RxSample& r) {
         r.gates.contact_stop_active = true;
         r.contact_stop_status = 2;
       },
       FaultReason::CONTACT_STOP, kCondContactStop, BumpReason::kKillDegradedEntry},
      {"safety_clamp", [](RxSample& r) { r.gates.safety_clamp = true; }, FaultReason::SAFETY_CLAMP,
       kCondSafetyClamp, BumpReason::kKillDegradedEntry},
      {"motion_not_possible", [](RxSample& r) { r.gates.motion_possible = false; },
       FaultReason::MOTION_NOT_POSSIBLE, kCondMotionNotPossible, BumpReason::kKillDegradedEntry},
  }};

  for (const GateCase& gc : cases) {
    RtCoreConfig cfg;
    Vec6 q0{};
    TickCore tc(cfg);
    init_holding(tc, q0);
    const std::uint64_t e0 = tc.epoch();

    RxSample rx = clean_rx(q0);
    gc.apply(rx);
    tc.tick(&rx, nullptr);  // ← the ONLY thing that runs; no callbacks exist

    EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW) << gc.name << ": gate kills active/held motion";
    EXPECT_EQ(tc.fault(), gc.fault) << gc.name << ": correct latched fault";
    EXPECT_TRUE(tc.conditions() & gc.cond_bit) << gc.name << ": condition bit set (SET semantics)";
    EXPECT_EQ(tc.epoch(), e0 + 1) << gc.name << ": exactly one epoch bump";
    EXPECT_EQ(tc.bump_count(gc.bump), 1u) << gc.name << ": bump attributed to the right reason";
  }
}

// The reaction is synchronous with the tick, not gated on event drain: the mode
// is already SAFE_FOLLOW when tick() returns, before pop_event() is ever called.
TEST(GateAutonomy, ReactionPrecedesEventDrain) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);
  init_holding(tc, q0);

  RxSample rx = clean_rx(q0);
  rx.gates.tp_enabled = true;  // TEACH
  tc.tick(&rx, nullptr);
  // State reflects the reaction immediately (no drain performed yet).
  EXPECT_EQ(tc.mode(), Mode::SAFE_FOLLOW);
  EXPECT_EQ(tc.fault(), FaultReason::TEACH_MODE);

  // Events are ADDITIONALLY available for observability, but they are a
  // consequence — not the mechanism — of the reaction.
  bool saw_gate_edge = false;
  Event e{};
  while (tc.pop_event(e)) {
    if (e.type == EventType::kGateEdge && e.reason == FaultReason::TEACH_MODE) saw_gate_edge = true;
  }
  EXPECT_TRUE(saw_gate_edge) << "gate edge is emitted for observability";
}

// Gates before begin_streaming are informational only (no reaction) — autonomy
// is scoped to the streaming window.
TEST(GateAutonomy, GatesInertBeforeStreaming) {
  RtCoreConfig cfg;
  Vec6 q0{};
  TickCore tc(cfg);  // NOT begin_streaming'd
  RxSample rx = clean_rx(q0);
  rx.gates.e_stopped = true;
  tc.tick(&rx, nullptr);
  EXPECT_NE(tc.mode(), Mode::SAFE_FOLLOW) << "no reaction before STREAMING";
  EXPECT_EQ(tc.epoch(), 0u) << "no bump before STREAMING";
}
