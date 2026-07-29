# SPDX-License-Identifier: Apache-2.0
"""Tests for :class:`airo_fanuc.republisher.Republisher`.

Covers the published payload shapes, the never-publish-an-unknown-velocity-as-zeros
rule, the staleness annotation, the exact toast triples, the payload size guard and
the wall-clock joints accessor contract.
"""

from __future__ import annotations

import time
from typing import Any

import pytest

from airo_fanuc.republisher import (
    MAX_PAYLOAD_BYTES,
    TOPIC_EXT_STATUS,
    TOPIC_EXTERNAL_FORCE,
    TOPIC_JOINT_STATES,
    TOPIC_LOG_EVENT,
    TOPIC_ROBOT_STATUS,
    FakePublisher,
    Republisher,
)


def _base_snapshot() -> dict[str, Any]:
    now = time.time_ns()
    return {
        "q_meas_deg": [10.0, 20.0, 30.0, 40.0, 50.0, 60.0],
        "qd_meas_deg": [0.1, 0.2, 0.0, -0.1, 0.0, 0.0],
        "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
        "t_meas_ns": now,
        "source": "stream",
        "e_stopped": False,
        "in_error": False,
        "tp_enabled": False,
        "motion_possible": True,
        "motion_in_progress": False,
        "contact_stop_mode": 0,
        "safety_scale": 1.0,
        "rx_silent": False,
        "recovery_event_id": 3,
        "starvation_active": False,
        "system_fault_event_id": 1,
        "gen_override_pct": 100,
        "speed_clamp_limit_pct": 100.0,
        "lifecycle_state": "STREAMING",
        "fault_reason": None,
        "operator_hint": None,
        "command_epoch": 7,
        "owner": "pid=1234 mode=control",
        "tick_p99_ms": 8.2,
        "wrench": {"force": [1.0, 2.0, 3.0], "moment": [0.1, 0.2, 0.3], "magnitude_n": 3.74},
        "control_mode": "RemoteControl",
        "in_motion": False,
        "drives_powered": True,
    }


def _repub(snap: dict[str, Any]) -> tuple[Republisher, FakePublisher]:
    pub = FakePublisher()
    r = Republisher(pub, lambda: snap)
    # Declare handles without starting the threads (deterministic one-shot tests).
    for topic in (
        TOPIC_JOINT_STATES,
        TOPIC_ROBOT_STATUS,
        TOPIC_EXTERNAL_FORCE,
        TOPIC_EXT_STATUS,
        TOPIC_LOG_EVENT,
    ):
        r._handles[topic] = pub.declare_publisher(topic)
    return r, pub


# ---------------------------------------------------------------------------
# Payload shape
# ---------------------------------------------------------------------------


def test_joint_states_payload_shape() -> None:
    r, pub = _repub(_base_snapshot())
    r.publish_joint_states()
    (payload,) = pub.payloads(TOPIC_JOINT_STATES)
    assert payload["positions_deg"] == [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    assert payload["velocities_deg_s"] == [0.1, 0.2, 0.0, -0.1, 0.0, 0.0]
    assert payload["velocity_valid"] is True
    assert payload["source"] == "stream"
    assert payload["stale"] is False
    assert payload["age_ms"] is not None


def test_robot_status_payload_has_status_and_lifecycle_keys() -> None:
    r, pub = _repub(_base_snapshot())
    r.publish_robot_status()
    (payload,) = pub.payloads(TOPIC_ROBOT_STATUS)
    for key in (
        "e_stopped",
        "in_error",
        "tp_enabled",
        "motion_possible",
        "motion_in_progress",
        "contact_stop_mode",
        "safety_scale",
        "rx_silent",
        "recovery_event_id",
        "starvation_active",
        "system_fault_event_id",
        "gen_override_pct",
        "speed_clamp_limit_pct",
    ):
        assert key in payload, f"missing status key {key}"
    for key in (
        "lifecycle_state",
        "fault_reason",
        "operator_hint",
        "command_epoch",
        "owner",
        "tick_p99_ms",
    ):
        assert key in payload, f"missing lifecycle key {key}"
    assert payload["lifecycle_state"] == "STREAMING"
    assert payload["command_epoch"] == 7


def test_external_force_payload_shape() -> None:
    r, pub = _repub(_base_snapshot())
    r.publish_external_force()
    (payload,) = pub.payloads(TOPIC_EXTERNAL_FORCE)
    assert payload["force"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert payload["magnitude_n"] == pytest.approx(3.74)
    assert "age_ms" in payload


def test_external_force_skipped_when_wrench_none() -> None:
    snap = _base_snapshot()
    snap["wrench"] = None
    r, pub = _repub(snap)
    r.publish_external_force()
    assert pub.payloads(TOPIC_EXTERNAL_FORCE) == []


def test_ext_status_payload_shape() -> None:
    r, pub = _repub(_base_snapshot())
    r.publish_ext_status()
    (payload,) = pub.payloads(TOPIC_EXT_STATUS)
    assert payload["gen_override_pct"] == 100
    assert payload["control_mode"] == "RemoteControl"
    assert payload["drives_powered"] is True


# ---------------------------------------------------------------------------
# None-velocity is NEVER published as zeros (2026-05-17)
# ---------------------------------------------------------------------------


def test_none_velocity_not_published_as_zeros() -> None:
    snap = _base_snapshot()
    snap["qd_meas_deg"] = None
    r, pub = _repub(snap)
    r.publish_joint_states()
    (payload,) = pub.payloads(TOPIC_JOINT_STATES)
    assert payload["velocities_deg_s"] is None
    assert payload["velocity_valid"] is False
    # Explicitly NOT a zeros list.
    assert payload["velocities_deg_s"] != [0.0] * 6


def test_no_publish_when_no_joint_measurement() -> None:
    snap = _base_snapshot()
    snap["q_meas_deg"] = None
    r, pub = _repub(snap)
    r.publish_joint_states()
    assert pub.payloads(TOPIC_JOINT_STATES) == []


# ---------------------------------------------------------------------------
# Staleness annotation: a stale sample is still published, but flagged — a consumer
# must be able to tell "old reading" from "no reading".
# ---------------------------------------------------------------------------


def test_stale_annotation() -> None:
    snap = _base_snapshot()
    snap["t_meas_ns"] = time.time_ns() - 500_000_000  # 500 ms old > 200 ms
    r, pub = _repub(snap)
    r.publish_joint_states()
    (payload,) = pub.payloads(TOPIC_JOINT_STATES)
    assert payload["stale"] is True
    assert payload["age_ms"] > 200.0


# ---------------------------------------------------------------------------
# Toast triples — exact (kind, tag, status). Subscribers match on the literal triple,
# so these identifiers are wire contract: a renamed tag silently stops a toast.
# ---------------------------------------------------------------------------


def test_motion_possible_fail_toast_triple() -> None:
    snap = _base_snapshot()
    r, pub = _repub(snap)
    snap["motion_possible"] = True
    r.publish_robot_status()  # seed prev=True
    snap["motion_possible"] = False
    r.publish_robot_status()  # edge True->False
    events = pub.payloads(TOPIC_LOG_EVENT)
    triples = [(e["kind"], e["tag"], e["status"]) for e in events]
    assert ("SYSTEM", "BRIDGE:MOTION_POSSIBLE", "FAIL") in triples


def test_bridge_fault_toast_triple() -> None:
    snap = _base_snapshot()
    r, pub = _repub(snap)
    r.publish_robot_status()  # seed prev_faulted=False
    snap["lifecycle_state"] = "FAULTED"
    snap["fault_reason"] = "E-stop pressed"
    r.publish_robot_status()  # edge into FAULTED
    events = pub.payloads(TOPIC_LOG_EVENT)
    # BRIDGE:FAULT whitelist matches ANY status (None wildcard) — assert the
    # (kind, tag) pair is emitted exactly.
    pairs = [(e["kind"], e["tag"]) for e in events]
    assert ("SYSTEM", "BRIDGE:FAULT") in pairs


def test_no_toast_without_edge() -> None:
    snap = _base_snapshot()
    r, pub = _repub(snap)
    r.publish_robot_status()
    r.publish_robot_status()  # steady STREAMING, motion_possible True — no toast
    assert pub.payloads(TOPIC_LOG_EVENT) == []


# ---------------------------------------------------------------------------
# < 3072 B guard: the pub/sub transport silently drops samples at or above its
# shared-memory slot size, so every payload must stay under MAX_PAYLOAD_BYTES.
# ---------------------------------------------------------------------------


def test_all_payloads_under_shm_guard() -> None:
    snap = _base_snapshot()
    # Stress the string fields a bit (fault reason / operator hint / owner).
    snap["fault_reason"] = "SRVO-023 Stop error excess (J2, step 5.00 deg); " * 5
    snap["operator_hint"] = "Release E-stop; then RESET on the TP if in_error persists. " * 3
    r, pub = _repub(snap)
    r.publish_joint_states()
    r.publish_robot_status()  # steady (no toast)
    r.publish_external_force()
    r.publish_ext_status()
    r.emit_event("SYSTEM", "BRIDGE:FAULT", "FAIL", snap["fault_reason"])
    assert pub.all_bytes(), "expected some payloads"
    for topic, data in pub.all_bytes():
        assert len(data) < MAX_PAYLOAD_BYTES, f"{topic} payload {len(data)} B >= guard"


def test_oversized_payload_is_dropped_not_published() -> None:
    snap = _base_snapshot()
    snap["fault_reason"] = "x" * 4000  # blow past the guard
    r, pub = _repub(snap)
    r.publish_robot_status()
    # The status payload exceeds the guard and must be dropped (never emitted).
    assert pub.payloads(TOPIC_ROBOT_STATUS) == []


# ---------------------------------------------------------------------------
# joints_at wall-clock accessor contract: the republisher never owns the joint ring,
# so the lookup must be injected — and must raise, not guess, when it is not.
# ---------------------------------------------------------------------------


def test_joints_at_wall_requires_injected_accessor() -> None:
    r, _pub = _repub(_base_snapshot())
    with pytest.raises(NotImplementedError):
        r.joints_at_wall(time.time_ns())


def test_joints_at_wall_delegates_to_injected_accessor() -> None:
    seen: list[int] = []

    def accessor(t_wall_ns: int) -> str:
        seen.append(t_wall_ns)
        return "q@wall"

    pub = FakePublisher()
    r = Republisher(pub, _base_snapshot, joints_at_wall=accessor)
    stamp = time.time_ns()
    assert r.joints_at_wall(stamp) == "q@wall"
    assert seen == [stamp]


# ---------------------------------------------------------------------------
# Threaded liveness — the injected FakePublisher receives payloads
# ---------------------------------------------------------------------------


def test_threads_produce_payloads() -> None:
    pub = FakePublisher()
    r = Republisher(
        pub,
        _base_snapshot,
        joint_states_hz=200.0,
        robot_status_hz=100.0,
        external_force_hz=100.0,
        ext_status_hz=100.0,
    )
    r.start()
    try:
        time.sleep(0.15)
    finally:
        r.stop()
    topics = {t for (t, _b) in pub.all_bytes()}
    assert TOPIC_JOINT_STATES in topics
    assert TOPIC_ROBOT_STATUS in topics
    assert TOPIC_EXTERNAL_FORCE in topics
    # Every payload the running threads emitted respects the guard.
    for _topic, data in pub.all_bytes():
        assert len(data) < MAX_PAYLOAD_BYTES
