# SPDX-License-Identifier: Apache-2.0
"""Zenoh-free state republisher for the driver-hosting process.

Re-publishes the driver's live state as telemetry on a set of fixed Zenoh
topics, so independent consumers (an operator UI, a perception capture stage, a
recorder) can observe the robot without holding a handle on the driver:

* ``fanuc/joint_states``  100 Hz — ``positions_deg`` / ``velocities_deg_s`` +
  ``t_meas_ns`` / ``age_ms`` / ``stale`` / ``source``.
* ``fanuc/robot_status``  10 Hz — the controller safety/status keys +
  ``lifecycle_state`` / ``fault_reason`` / ``faulted`` / ``operator_hint`` /
  ``command_epoch`` / ``recovery_count`` / ``owner`` / ``tick_p99_ms``.
* ``fanuc/external_force`` 50 Hz — force/moment/magnitude + ``age_ms``.
* ``fanuc/ext_status``     1 Hz — override / speed-clamp / control-mode. These come
  off an RMI round trip rather than the Stream Motion snapshot, so they arrive
  through their own injected :data:`ExtStatusFn` and are read on this 1 Hz thread
  only; with no accessor the topic stays silent.
* :data:`TOPIC_LOG_EVENT` — typed toast events. A consumer's toast whitelist
  matches on the exact ``(kind, tag, status)`` triple, so the two triples this
  module emits are a pinned contract, not a formatting choice:
  ``("SYSTEM","BRIDGE:MOTION_POSSIBLE","FAIL")`` and
  ``("SYSTEM","BRIDGE:FAULT", None)`` (``tag = "{system}:{feature}"``).

**No zenoh import.** The package's only runtime dependency is numpy, so the
concrete sink is injected as a :class:`Publisher` (duck-typed to a
``zenoh.Session``: ``declare_publisher(key)`` → handle with ``put(payload)``).
The hosting application supplies the real session; tests inject
:class:`FakePublisher`.

Snapshot contract
-----------------
The driver passes a ``snapshot() -> dict`` callable returning the current state —
``FanucDriver._republish_snapshot`` assembles exactly these keys. All keys are
optional and None-safe, and the republisher is **publish-always, annotate**: a
missing or None value is published as an explicit annotation rather than
suppressed, so a consumer can always tell "no data" from "stale data" and never
has to infer either from silence.

    q_meas_deg, qd_meas_deg (None ⇒ velocity_valid=False, NEVER zeros),
    joint_names, t_meas_ns, source,
    e_stopped, in_error, tp_enabled, motion_possible, motion_in_progress,
    contact_stop_status, safety_scale, rx_silent,
    lifecycle_state, fault_reason, faulted, operator_hint, command_epoch,
    recovery_count, owner, tick_p99_ms,
    wrench = {"force":[x,y,z], "moment":[x,y,z], "magnitude_n":float} | None.

``t_meas_ns`` and the ``now_ns`` ages are taken against MUST be the same clock —
wall-clock nanoseconds by default; see :meth:`_age_ms`. ``faulted`` is the
driver's own lifecycle classification (:data:`airo_fanuc.lifecycle.FAULT_STATES`)
travelling as data: fault classification has one definition, in
:mod:`airo_fanuc.lifecycle`, and this module does not re-derive it from a status
string.

Every republished payload serializes to < 3072 B: zenoh silently drops samples
above its shared-memory threshold, so an oversized payload would vanish without
an error. A unit test asserts the bound. ``joints_at_wall`` is the Python-side
accessor contract for the C++-owned (mono_ns, wall_ns) ring — see
:meth:`joints_at_wall`.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections.abc import Callable
from datetime import datetime, timezone
from typing import Any, Protocol

logger = logging.getLogger("airo_fanuc.republisher")

# -- Topics ------------------------------------------------------------------
TOPIC_JOINT_STATES = "fanuc/joint_states"
TOPIC_ROBOT_STATUS = "fanuc/robot_status"
TOPIC_EXTERNAL_FORCE = "fanuc/external_force"
TOPIC_EXT_STATUS = "fanuc/ext_status"
TOPIC_LOG_EVENT = "fanuc/log/event"

# -- Rates (Hz) --------------------------------------------------------------
JOINT_STATES_HZ = 100.0
ROBOT_STATUS_HZ = 10.0
EXTERNAL_FORCE_HZ = 50.0
EXT_STATUS_HZ = 1.0

#: Staleness annotation threshold: age > this ⇒ ``stale=True``. The sample is
#: still published — annotated, never suppressed.
STALE_MS = 200.0

#: Payload ceiling: zenoh silently drops samples above its shared-memory
#: threshold, so every republished payload must stay under it.
MAX_PAYLOAD_BYTES = 3072

#: Age ceiling above which :meth:`Republisher._age_ms` reports "no age" instead of a
#: number. Nothing on these topics is telemetry a minute after it was measured, so a
#: result beyond this means the stamp is not in the ``now_ns`` clock domain — a
#: CLOCK_MONOTONIC stamp is the near miss, and it subtracts to the host's uptime.
MAX_AGE_MS = 60_000.0

# -- Pinned toast triple identifiers -----------------------------------------
# A consumer's toast whitelist matches on the exact (kind, tag, status) triple,
# so these strings cannot be reworded without breaking it. tag = "{system}:{feature}".
EVENT_KIND_SYSTEM = "SYSTEM"
TAG_BRIDGE_MOTION_POSSIBLE = "BRIDGE:MOTION_POSSIBLE"
TAG_BRIDGE_FAULT = "BRIDGE:FAULT"
STATUS_FAIL = "FAIL"
STATUS_SUCCESS = "SUCCESS"


class PublisherHandle(Protocol):
    """A per-topic publisher handle (duck-typed to ``zenoh.Publisher``)."""

    def put(self, payload: bytes) -> None: ...


class Publisher(Protocol):
    """A zenoh-session-shaped sink (duck-typed to ``zenoh.Session``).

    The republisher calls :meth:`declare_publisher` once per topic at
    :meth:`Republisher.start` and reuses the returned handle for every
    subsequent ``put(bytes)`` — re-declaring per sample would repeat the
    session-side setup at up to 100 Hz. The hosting application passes the real
    ``zenoh.Session``; tests pass :class:`FakePublisher`.
    """

    def declare_publisher(self, topic: str) -> PublisherHandle: ...


SnapshotFn = Callable[[], dict[str, Any]]
JointsAtWallFn = Callable[[int], Any]
#: Injected reader for the RMI extended-status block (``gen_override_pct``,
#: ``speed_clamp_limit_pct``, ``control_mode``, ``in_motion``, ``drives_powered``,
#: optional wall-clock ``t_read_ns``). ``None`` means "not readable now" and
#: publishes nothing.
ExtStatusFn = Callable[[], "dict[str, Any] | None"]


class Republisher:
    """Threads that re-publish the driver snapshot onto the surviving topics."""

    def __init__(
        self,
        publisher: Publisher,
        snapshot: SnapshotFn,
        *,
        joints_at_wall: JointsAtWallFn | None = None,
        ext_status: ExtStatusFn | None = None,
        now_ns: Callable[[], int] = time.time_ns,
        joint_states_hz: float = JOINT_STATES_HZ,
        robot_status_hz: float = ROBOT_STATUS_HZ,
        external_force_hz: float = EXTERNAL_FORCE_HZ,
        ext_status_hz: float = EXT_STATUS_HZ,
    ) -> None:
        self._publisher = publisher
        self._snapshot = snapshot
        self._joints_at_wall_fn = joints_at_wall
        self._ext_status_fn = ext_status
        self._now_ns = now_ns
        self._periods = {
            TOPIC_JOINT_STATES: 1.0 / float(joint_states_hz),
            TOPIC_ROBOT_STATUS: 1.0 / float(robot_status_hz),
            TOPIC_EXTERNAL_FORCE: 1.0 / float(external_force_hz),
            TOPIC_EXT_STATUS: 1.0 / float(ext_status_hz),
        }

        self._handles: dict[str, PublisherHandle] = {}
        self._stop_evt = threading.Event()
        self._threads: list[threading.Thread] = []
        self._started = False

        # Toast edge-detection state (robot_status loop).
        self._prev_motion_possible: bool | None = None
        self._prev_faulted: bool | None = None
        self._age_domain_warned = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Declare publishers and start the per-topic republish threads."""
        if self._started:
            return
        for topic in (
            TOPIC_JOINT_STATES,
            TOPIC_ROBOT_STATUS,
            TOPIC_EXTERNAL_FORCE,
            TOPIC_EXT_STATUS,
            TOPIC_LOG_EVENT,
        ):
            self._handles[topic] = self._publisher.declare_publisher(topic)
        self._stop_evt.clear()
        self._threads = [
            threading.Thread(
                target=self._run_loop,
                args=(TOPIC_JOINT_STATES, self.publish_joint_states),
                name="airo-fanuc-repub-joints",
                daemon=True,
            ),
            threading.Thread(
                target=self._run_loop,
                args=(TOPIC_ROBOT_STATUS, self.publish_robot_status),
                name="airo-fanuc-repub-status",
                daemon=True,
            ),
            threading.Thread(
                target=self._run_loop,
                args=(TOPIC_EXTERNAL_FORCE, self.publish_external_force),
                name="airo-fanuc-repub-force",
                daemon=True,
            ),
            threading.Thread(
                target=self._run_loop,
                args=(TOPIC_EXT_STATUS, self.publish_ext_status),
                name="airo-fanuc-repub-ext",
                daemon=True,
            ),
        ]
        for t in self._threads:
            t.start()
        self._started = True

    def stop(self) -> None:
        """Stop the republish threads (idempotent)."""
        self._stop_evt.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads = []
        self._started = False

    def __enter__(self) -> Republisher:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def _run_loop(self, topic: str, publish_once: Callable[[], None]) -> None:
        period = self._periods[topic]
        while not self._stop_evt.is_set():
            try:
                publish_once()
            except Exception as exc:  # noqa: BLE001 - a publish blip must not kill the loop
                logger.debug("republisher: %s publish failed: %s", topic, exc)
            self._stop_evt.wait(period)

    # ------------------------------------------------------------------
    # Per-topic publish (one-shot; also the deterministic test entry points)
    # ------------------------------------------------------------------

    def publish_joint_states(self) -> None:
        snap = self._snapshot()
        q = snap.get("q_meas_deg")
        if q is None:
            return  # no measurement yet — nothing to annotate
        t_meas_ns = snap.get("t_meas_ns")
        age_ms = self._age_ms(t_meas_ns)
        payload: dict[str, Any] = {
            "joint_names": list(snap.get("joint_names") or []),
            "positions_deg": [float(v) for v in q],
            "t_meas_ns": int(t_meas_ns) if t_meas_ns is not None else None,
            "age_ms": age_ms,
            # An unknown age is stale: a consumer gating on freshness must not read
            # "age unavailable" as "fresh".
            "stale": age_ms is None or age_ms > STALE_MS,
            "source": snap.get("source", "stream"),
        }
        qd = snap.get("qd_meas_deg")
        if qd is None:
            # NEVER publish zeros for an unknown velocity (2026-05-17). Annotate.
            payload["velocities_deg_s"] = None
            payload["velocity_valid"] = False
        else:
            payload["velocities_deg_s"] = [float(v) for v in qd]
            payload["velocity_valid"] = True
        self._put(TOPIC_JOINT_STATES, payload)

    def publish_robot_status(self) -> None:
        snap = self._snapshot()
        t_meas_ns = snap.get("t_meas_ns")
        payload: dict[str, Any] = {
            # Controller safety / status keys
            "e_stopped": bool(snap.get("e_stopped", False)),
            "in_error": bool(snap.get("in_error", False)),
            "tp_enabled": bool(snap.get("tp_enabled", False)),
            "motion_possible": bool(snap.get("motion_possible", False)),
            "motion_in_progress": bool(snap.get("motion_in_progress", False)),
            # The status packet's contact-stop enum as the controller reports it
            # (2 = STOP, 4 = ESCP — rt_core_config.hpp decode_gates), not a mode
            # selector: the vendored ROS driver derives a separate mode enum from it
            # (fanuc_client.cpp ToContactStopMode), so the two names are different
            # values and this one is the raw status.
            "contact_stop_status": int(snap.get("contact_stop_status", 0)),
            "safety_scale": float(snap.get("safety_scale", 1.0)),
            "rx_silent": bool(snap.get("rx_silent", False)),
            # Driver lifecycle keys
            "lifecycle_state": snap.get("lifecycle_state"),
            "fault_reason": snap.get("fault_reason"),
            "faulted": bool(snap.get("faulted", False)),
            "operator_hint": snap.get("operator_hint"),
            "command_epoch": _opt_int(snap.get("command_epoch")),
            "recovery_count": _opt_int(snap.get("recovery_count")),
            "owner": snap.get("owner"),
            "tick_p99_ms": _opt_float(snap.get("tick_p99_ms")),
            "age_ms": self._age_ms(t_meas_ns),
        }
        self._put(TOPIC_ROBOT_STATUS, payload)
        self._emit_status_toasts(snap)

    def publish_external_force(self) -> None:
        snap = self._snapshot()
        wrench = snap.get("wrench")
        if wrench is None:
            return  # fs_type unavailable — there is no wrench to publish
        force = list(wrench.get("force", (0.0, 0.0, 0.0)))
        moment = list(wrench.get("moment", (0.0, 0.0, 0.0)))
        mag = wrench.get("magnitude_n")
        payload: dict[str, Any] = {
            "force": {"x": float(force[0]), "y": float(force[1]), "z": float(force[2])},
            "moment": {"x": float(moment[0]), "y": float(moment[1]), "z": float(moment[2])},
            "magnitude_n": float(mag) if mag is not None else 0.0,
            "age_ms": self._age_ms(snap.get("t_meas_ns")),
        }
        self._put(TOPIC_EXTERNAL_FORCE, payload)

    def publish_ext_status(self) -> None:
        """Publish the RMI extended-status block from the injected :data:`ExtStatusFn`.

        Nothing is published without an accessor, or when it returns ``None`` (the
        driver refuses the read unless the RMI session is steady): a topic that says
        nothing is honest, an all-null payload is not. The read happens here, on the
        1 Hz thread, and never on the 100 Hz joint path.
        """
        read = self._ext_status_fn
        if read is None:
            return
        ext = read()
        if ext is None:
            return
        payload: dict[str, Any] = {
            "gen_override_pct": _opt_int(ext.get("gen_override_pct")),
            "speed_clamp_limit_pct": _opt_float(ext.get("speed_clamp_limit_pct")),
            "control_mode": ext.get("control_mode"),
            "in_motion": _opt_bool(ext.get("in_motion")),
            "drives_powered": _opt_bool(ext.get("drives_powered")),
            # Age of the RMI read itself, not of a Stream Motion sample: the two
            # planes are read independently and one cannot date the other.
            "age_ms": self._age_ms(ext.get("t_read_ns")),
        }
        self._put(TOPIC_EXT_STATUS, payload)

    # ------------------------------------------------------------------
    # Toast events (TOPIC_LOG_EVENT) — exact pinned triples
    # ------------------------------------------------------------------

    def _emit_status_toasts(self, snap: dict[str, Any]) -> None:
        """Edge-detect motion_possible drops and fault entries; emit the pinned triples."""
        motion_possible = bool(snap.get("motion_possible", False))
        if self._prev_motion_possible is not None and self._prev_motion_possible and not motion_possible:
            self.emit_event(
                EVENT_KIND_SYSTEM,
                TAG_BRIDGE_MOTION_POSSIBLE,
                STATUS_FAIL,
                self._motion_possible_specifics(snap),
            )
        elif self._prev_motion_possible is not None and not self._prev_motion_possible and motion_possible:
            self.emit_event(
                EVENT_KIND_SYSTEM,
                TAG_BRIDGE_MOTION_POSSIBLE,
                STATUS_SUCCESS,
                "motion_possible recovered to TRUE",
            )
        self._prev_motion_possible = motion_possible

        # The snapshot's own classification (airo_fanuc.lifecycle.FAULT_STATES). Not
        # re-derived here: fault_reason carries the string "none" when there is no
        # fault, so any local truthiness test latches "faulted" on a healthy robot and
        # the rising edge below never arrives.
        faulted = bool(snap.get("faulted", False))
        if self._prev_faulted is not None and not self._prev_faulted and faulted:
            reason = snap.get("fault_reason") or "fault"
            self.emit_event(EVENT_KIND_SYSTEM, TAG_BRIDGE_FAULT, STATUS_FAIL, str(reason))
        self._prev_faulted = faulted

    @staticmethod
    def _motion_possible_specifics(snap: dict[str, Any]) -> str:
        return (
            f"motion_possible→FALSE (e_stop={bool(snap.get('e_stopped', False))}, "
            f"in_error={bool(snap.get('in_error', False))}, "
            f"tp_enabled={bool(snap.get('tp_enabled', False))})"
        )

    def emit_event(self, kind: str, tag: str, status: str, specifics: str = "") -> None:
        """Publish one structured event on :data:`TOPIC_LOG_EVENT`.

        The payload carries ``kind`` / ``tag`` / ``status`` / ``specifics`` plus
        the three timestamp forms (ns, ms, ISO-8601) consumers index on. A
        consumer's toast whitelist matches on the exact ``(kind, tag, status)``
        triple, so this shape and the triple constants above are part of the
        published contract. The shape is reproduced here rather than imported so
        the package carries no dependency on any consumer.
        """
        ts_ns = int(self._now_ns())
        payload = {
            "ts_ns": ts_ns,
            "ts_ms": ts_ns // 1_000_000,
            "ts_iso": _format_ts(ts_ns),
            "kind": kind,
            "tag": tag,
            "status": status,
            "specifics": specifics,
        }
        self._put(TOPIC_LOG_EVENT, payload)

    # ------------------------------------------------------------------
    # joints_at wall-clock accessor contract
    # ------------------------------------------------------------------

    def joints_at_wall(self, t_wall_ns: int) -> Any:
        """Joint configuration at a CAMERA-GRAB WALL timestamp.

        The C++ core owns the ``(mono_ns, wall_ns)`` ring (wall captured off the
        RT thread at packet ingest); this is the Python-side accessor contract
        for it. Consumers pass a camera grab-shutter wall stamp
        (``time.time_ns()`` domain) — the FK-at-shutter contract that guards the
        phantom-voxel class — and the driver interpolates on the paired mono
        stamps. ``StreamCore.joints_at_wall`` implements the ring and the driver
        wires the callable in at construction; :class:`NotImplementedError` fires
        only when a :class:`Republisher` is built without it (e.g. a bare unit
        test).
        """
        if self._joints_at_wall_fn is None:
            raise NotImplementedError(
                "joints_at_wall requires the driver's (mono_ns, wall_ns) ring "
                "accessor (C++ core, wired by the driver at construction)"
            )
        return self._joints_at_wall_fn(int(t_wall_ns))

    # ------------------------------------------------------------------
    # Encoding + guard
    # ------------------------------------------------------------------

    def _put(self, topic: str, payload: dict[str, Any]) -> None:
        data = _encode(payload)
        if len(data) >= MAX_PAYLOAD_BYTES:
            # A payload at/over the SHM threshold would be dropped by zenoh with
            # no error, so drop it here loudly instead.
            logger.error(
                "republisher: %s payload is %d B (>= %d B SHM guard) — dropping",
                topic,
                len(data),
                MAX_PAYLOAD_BYTES,
            )
            return
        handle = self._handles.get(topic)
        if handle is None:
            return
        handle.put(data)

    def _age_ms(self, t_meas_ns: Any) -> float | None:
        """Age of ``t_meas_ns`` against ``now_ns``, or ``None`` if it cannot be one.

        Both stamps must be read from the SAME clock — wall-clock nanoseconds with the
        default ``now_ns``. A stamp from another clock still subtracts to a number, and
        a published age is used to decide whether a pose may be trusted, so a result
        outside ``0 .. MAX_AGE_MS`` is reported as "no age" and logged once instead.
        """
        if t_meas_ns is None:
            return None
        age_ms = (int(self._now_ns()) - int(t_meas_ns)) / 1e6
        if age_ms < 0.0 or age_ms > MAX_AGE_MS:
            if not self._age_domain_warned:
                self._age_domain_warned = True
                logger.error(
                    "republisher: age %.0f ms is outside 0..%.0f ms — t_meas_ns is not in the "
                    "now_ns clock domain (or the wall clock stepped); publishing age_ms=None",
                    age_ms,
                    MAX_AGE_MS,
                )
            return None
        return age_ms


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _encode(payload: dict[str, Any]) -> bytes:
    """Compact JSON bytes (separators trimmed to keep payloads under the guard)."""
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def _format_ts(ts_ns: int) -> str:
    dt = datetime.fromtimestamp(ts_ns / 1_000_000_000, tz=timezone.utc)
    return dt.isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _opt_int(v: Any) -> int | None:
    return int(v) if v is not None else None


def _opt_float(v: Any) -> float | None:
    return float(v) if v is not None else None


def _opt_bool(v: Any) -> bool | None:
    return bool(v) if v is not None else None


class FakePublisher:
    """In-memory :class:`Publisher` for tests — records every ``(topic, bytes)``."""

    def __init__(self) -> None:
        self.records: list[tuple[str, bytes]] = []
        self._lock = threading.Lock()

    def declare_publisher(self, topic: str) -> _FakeHandle:
        return _FakeHandle(self, topic)

    def _record(self, topic: str, payload: bytes) -> None:
        with self._lock:
            self.records.append((topic, payload))

    def payloads(self, topic: str) -> list[dict[str, Any]]:
        """Decoded payloads recorded for ``topic``."""
        with self._lock:
            recs = [p for (t, p) in self.records if t == topic]
        return [json.loads(p.decode("utf-8")) for p in recs]

    def all_bytes(self) -> list[tuple[str, bytes]]:
        with self._lock:
            return list(self.records)


class _FakeHandle:
    def __init__(self, parent: FakePublisher, topic: str) -> None:
        self._parent = parent
        self._topic = topic

    def put(self, payload: bytes) -> None:
        self._parent._record(self._topic, payload)


__all__ = [
    "Republisher",
    "Publisher",
    "PublisherHandle",
    "FakePublisher",
    "TOPIC_JOINT_STATES",
    "TOPIC_ROBOT_STATUS",
    "TOPIC_EXTERNAL_FORCE",
    "TOPIC_EXT_STATUS",
    "TOPIC_LOG_EVENT",
    "MAX_AGE_MS",
    "MAX_PAYLOAD_BYTES",
    "STALE_MS",
]
