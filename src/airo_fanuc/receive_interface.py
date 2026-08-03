# SPDX-License-Identifier: Apache-2.0
"""RMI-poll read-only state interface for calibration / no-motion tools.

``FanucReceiveInterface`` is the motion-free way to read the controller when
nothing is driving it: it polls the
:class:`~airo_fanuc.rmi_client.RmiClient` *commands-only* session (it NEVER
calls :meth:`~airo_fanuc.rmi_client.RmiClient.initialize`, so it works in T1 and
with no program running) and derives joint velocity by least squares. It is the
structural fix for the T1-freeze calibration-corruption incident.

Two guards make hand-eye calibration safe (both raise, never silently return a
bad sample — "surface latent issues loudly"):

* **Velocity gate.** Velocity is a least-squares slope over a
  ``>= CALIB_LSQ_WINDOW_S`` (0.5 s) window, NOT a single instantaneous sample.
  The stillness threshold is ``CALIB_STILLNESS_DEG_S`` (0.1 deg/s), matching the
  hand-eye calibration ground truth. If velocity is unavailable (too few
  samples / insufficient time base / a frozen feed with duplicate stamps) the
  estimate is ``None`` and a capture is **rejected** — never fabricated as zero
  (fabricating 0 deg/s is exactly the corruption: a frozen feed then reads as
  "settled"). A capture also asserts the measured pose actually changed vs the
  previous accepted sample (a frozen feed at rest would otherwise pass).

* **J2/J3 source gate.** RMI-sourced joints are tagged
  :data:`SOURCE_RMI_UNCONVERTED` and calibration **HARD-REJECTS** them while
  ``controller_facts.INTERIM_FACTS.rmi_to_stream_j3_plus_j2_verified`` is
  ``False``. On a controller serving the J2/J3-coupled representation RMI reports
  J3 one J2 below the Stream Motion value, so reaching the stream frame takes
  ``J3 += J2``; whether a controller does that is a per-installation
  configuration, and being wrong either way is a silent J2-sized FK error.
  :meth:`_apply_rmi_joint_policy` is the single per-model conversion point. RMI
  and Stream Motion joints are never mixed in one dataset.

Joint source
------------
Joints arrive through an injected :class:`JointReader` (dependency injection),
which isolates the safety-critical velocity/settled/source logic from the
joint-read transport — the gates above are testable without a controller, and a
new transport cannot quietly bypass them. :class:`RmiClientJointReader` is the
RMI-backed implementation — it reads ``FRC_ReadJointAngles`` off an
:class:`~airo_fanuc.rmi_client.RmiClient` and tags every sample
:data:`SOURCE_RMI_UNCONVERTED` (it applies no J3 correction, so calibration
HARD-REJECTS it). The optional ``rmi`` client passed here is used only for
status polling (``get_status`` / ``get_extended_status``), which the
commands-only session already supports.

Dependency-light on purpose: stdlib ``logging`` / ``threading`` / ``time`` +
numpy (LSQ).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Protocol

import numpy as np

from airo_fanuc.controller_facts import (
    CALIB_LSQ_WINDOW_S,
    CALIB_STILLNESS_DEG_S,
    INTERIM_FACTS,
    P1Facts,
)
from airo_fanuc.exceptions import (
    CalibrationError,
    CalibrationSourceError,
    CalibrationVelocityUnavailable,
    RmiError,
    RmiSessionDown,
)

if TYPE_CHECKING:  # pragma: no cover - typing only
    from airo_fanuc.ownership import OwnershipLock
    from airo_fanuc.rmi_client import ControllerStatus, ExtendedStatus

logger = logging.getLogger("airo_fanuc.receive")

# Joint-source tags. RMI reads are UNCONVERTED — RMI J3 sits one J2 below the Stream
# Motion J3 and nothing corrects it; Stream Motion joints are the calibration reference.
SOURCE_STREAM = "stream"
SOURCE_RMI_UNCONVERTED = "rmi_unconverted"
SOURCE_RMI_CONVERTED = "rmi_converted"

# LSQ / capture defaults.
_DEFAULT_MIN_SAMPLES = 3  # a degree-1 fit needs >=2; 3 rejects a 2-point fluke
_DEFAULT_MIN_SPAN_FRACTION = 0.8  # window must be ~covered (discrete sampling slack)
# Minimum position change (deg) between two accepted calibration samples. Well
# above RMI angle quantization (~0.001 deg, INTERIM_FACTS.rmi_angle_resolution_deg)
# so a genuinely new jogged pose passes, but a byte-identical frozen feed (0.0
# change — the frozen-feed signature) is caught.
_DEFAULT_MIN_CHANGE_DEG = 0.05


@dataclass(frozen=True)
class JointSample:
    """One measured joint reading.

    ``q_deg`` is the measured joint vector in **degrees**; ``t_wall_ns`` is the
    wall-clock capture stamp in nanoseconds (the same clock the cameras stamp
    grab time with, so a pose can be paired with an image); ``source`` is one of
    the ``SOURCE_*`` tags.
    """

    q_deg: np.ndarray
    t_wall_ns: int
    source: str = SOURCE_STREAM


class JointReader(Protocol):
    """Duck-typed joint source injected into :class:`FanucReceiveInterface`.

    :meth:`read` returns the latest :class:`JointSample` or ``None`` if no
    reading is currently available (transport blip / not connected). The
    RMI-backed implementation is :class:`RmiClientJointReader` below
    (``FRC_ReadJointAngles`` → :data:`SOURCE_RMI_UNCONVERTED`).
    """

    def read(self) -> JointSample | None: ...


class _JointReadRmi(Protocol):
    """The subset of :class:`~airo_fanuc.rmi_client.RmiClient` :class:`RmiClientJointReader` needs."""

    def read_joint_angles(self) -> list[float]: ...


class RmiClientJointReader:
    """:class:`JointReader` backed by ``RmiClient.read_joint_angles``.

    Each :meth:`read` issues ``FRC_ReadJointAngles`` on the client's
    commands-only session and wraps the reply in a :class:`JointSample` stamped
    with a monotonic wall-clock time (the same clock the cameras stamp grab time
    with) and tagged :data:`SOURCE_RMI_UNCONVERTED`.

    The tag is load-bearing: on a controller serving the J2/J3-coupled
    representation RMI J3 is one J2 below the Stream Motion value
    (``controller_facts.INTERIM_FACTS.rmi_to_stream_j3_plus_j2_measured``), so
    these joints are HARD-rejected for calibration by
    :meth:`FanucReceiveInterface._apply_source_policy` until the installation
    confirms it via ``rmi_to_stream_j3_plus_j2_verified``. This reader NEVER
    converts — conversion is the single per-model job of
    ``_apply_rmi_joint_policy``, downstream and gated on that fact.

    Per the :class:`JointReader` contract a transport blip / dead session
    surfaces as ``None`` (logged at DEBUG), not an exception, so the receive
    poll loop keeps running.
    """

    def __init__(self, rmi: _JointReadRmi, *, now_ns: Callable[[], int] = time.time_ns) -> None:
        self._rmi = rmi
        self._now_ns = now_ns

    def read(self) -> JointSample | None:
        try:
            q_deg = self._rmi.read_joint_angles()
        except (RmiError, RmiSessionDown) as exc:
            logger.debug("receive: RMI joint read unavailable: %s", exc)
            return None
        if not q_deg:
            return None
        return JointSample(
            q_deg=np.asarray(q_deg, dtype=np.float64),
            t_wall_ns=int(self._now_ns()),
            source=SOURCE_RMI_UNCONVERTED,
        )


class _StatusRmi(Protocol):
    """The subset of :class:`~airo_fanuc.rmi_client.RmiClient` used for status."""

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def get_status(self) -> ControllerStatus: ...

    def get_extended_status(self, *, quiet: bool = ...) -> ExtendedStatus: ...


@dataclass(frozen=True)
class SettleResult:
    """Outcome of :meth:`FanucReceiveInterface.settled`.

    ``settled`` is the predicate; ``sample_count`` / ``age_s`` describe the
    window used; ``max_velocity_deg_s`` is the LSQ velocity magnitude (``None``
    when unavailable); ``reason`` is a machine-readable tag
    (``settled`` / ``moving`` / ``velocity_unavailable`` / ``position_unchanged``
    / ``no_samples``).
    """

    settled: bool
    sample_count: int
    age_s: float
    max_velocity_deg_s: float | None
    reason: str


class FanucReceiveInterface:
    """RMI-poll read-only state + calibration velocity gate.

    Feed it joints via an injected :class:`JointReader` (polled on
    :meth:`start`) or, in tests, directly via :meth:`ingest_sample`. Query
    :meth:`settled` (soft UI predicate) or :meth:`capture_calibration_sample`
    (the hard, raising calibration path).
    """

    def __init__(
        self,
        joint_reader: JointReader | None = None,
        *,
        rmi: _StatusRmi | None = None,
        ownership: OwnershipLock | None = None,
        joint_poll_hz: float = 15.0,
        status_poll_hz: float = 2.0,
        lsq_window_s: float = CALIB_LSQ_WINDOW_S,
        stillness_deg_s: float = CALIB_STILLNESS_DEG_S,
        min_samples: int = _DEFAULT_MIN_SAMPLES,
        min_span_fraction: float = _DEFAULT_MIN_SPAN_FRACTION,
        min_change_deg: float = _DEFAULT_MIN_CHANGE_DEG,
        facts: P1Facts = INTERIM_FACTS,
        ring_size: int = 256,
        now_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        self._joint_reader = joint_reader
        self._rmi = rmi
        self._ownership = ownership
        self._joint_poll_period_s = 1.0 / float(joint_poll_hz)
        self._status_poll_period_s = 1.0 / float(status_poll_hz)
        self._lsq_window_s = float(lsq_window_s)
        self._stillness_deg_s = float(stillness_deg_s)
        self._min_samples = int(min_samples)
        self._min_span_fraction = float(min_span_fraction)
        self._min_change_deg = float(min_change_deg)
        self._facts = facts
        self._now_ns = now_ns

        self._lock = threading.Lock()
        self._ring: deque[JointSample] = deque(maxlen=int(ring_size))

        # Accepted-calibration bookkeeping (changed-position + no-mixing guards).
        self._last_accepted_q: np.ndarray | None = None
        self._accepted_source: str | None = None

        # Latest status snapshots (populated by the status poll thread).
        self._status_lock = threading.Lock()
        self._latest_status: ControllerStatus | None = None
        self._latest_ext_status: ExtendedStatus | None = None

        self._stop_evt = threading.Event()
        self._threads: list[threading.Thread] = []
        self._started = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Acquire ownership (if any), open the RMI session, start poll threads.

        Ownership is taken FIRST; the RMI commands-only session is opened only
        after we hold the lock. Never calls ``initialize()``.
        """
        if self._started:
            return
        if self._ownership is not None:
            self._ownership.acquire()
        try:
            if self._rmi is not None:
                self._rmi.start()
        except Exception:
            if self._ownership is not None:
                self._ownership.release()
            raise
        self._stop_evt.clear()
        self._threads = []
        if self._joint_reader is not None:
            self._threads.append(
                threading.Thread(target=self._joint_poll_loop, name="airo-fanuc-recv-joints", daemon=True)
            )
        if self._rmi is not None:
            self._threads.append(
                threading.Thread(target=self._status_poll_loop, name="airo-fanuc-recv-status", daemon=True)
            )
        for t in self._threads:
            t.start()
        self._started = True

    def stop(self) -> None:
        """Stop poll threads, then close RMI, then release ownership.

        The order is load-bearing: the RMI close completes BEFORE the flock is
        released, so the next process to take the lock can never race the dying
        RMI session (the controller allows only one).
        """
        if not self._started:
            return
        self._stop_evt.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads = []
        if self._rmi is not None:
            try:
                self._rmi.stop()
            except Exception as exc:  # noqa: BLE001 - shutdown is best-effort
                logger.warning("receive: RMI stop failed on teardown: %s", exc)
        if self._ownership is not None:
            self._ownership.release()
        self._started = False

    def __enter__(self) -> FanucReceiveInterface:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Sample ingestion
    # ------------------------------------------------------------------

    def ingest_sample(self, sample: JointSample) -> None:
        """Append a joint sample to the ring (thread-safe).

        Normally driven by the joint poll thread; exposed so tests (and a future
        push-based source) can feed samples deterministically.
        """
        with self._lock:
            self._ring.append(sample)

    def latest_joint_sample(self) -> JointSample | None:
        with self._lock:
            return self._ring[-1] if self._ring else None

    def get_controller_status(self) -> ControllerStatus | None:
        with self._status_lock:
            return self._latest_status

    def get_extended_status(self) -> ExtendedStatus | None:
        with self._status_lock:
            return self._latest_ext_status

    # ------------------------------------------------------------------
    # Velocity gate
    # ------------------------------------------------------------------

    def estimate_velocity_deg_s(self, window_s: float | None = None) -> np.ndarray | None:
        """Least-squares joint velocity (deg/s) over the last ``window_s`` seconds.

        Returns per-joint slopes, or ``None`` when velocity is *unavailable*:
        fewer than ``min_samples`` in the window, a time base shorter than
        ``min_span_fraction * window_s``, or a degenerate time base (duplicate
        stamps — a frozen feed). ``None`` is a hard signal — callers must reject
        it, never treat it as zero.
        """
        window_s = self._lsq_window_s if window_s is None else float(window_s)
        now = self._now_ns()
        window_ns = window_s * 1e9
        with self._lock:
            samples = [s for s in self._ring if (now - s.t_wall_ns) <= window_ns]
        if len(samples) < self._min_samples:
            return None
        samples.sort(key=lambda s: s.t_wall_ns)
        t_ns = np.array([s.t_wall_ns for s in samples], dtype=np.float64)
        span_s = float(t_ns[-1] - t_ns[0]) / 1e9
        if span_s < window_s * self._min_span_fraction:
            return None
        q = np.stack([np.asarray(s.q_deg, dtype=np.float64) for s in samples])  # (N, D)
        t_s = (t_ns - t_ns[0]) / 1e9
        t_centered = t_s - t_s.mean()
        var_t = float(t_centered @ t_centered)
        if var_t <= 0.0:
            return None  # degenerate time base (all stamps equal)
        q_centered = q - q.mean(axis=0, keepdims=True)
        slope = (t_centered @ q_centered) / var_t  # (D,) deg/s
        return slope

    def settled(
        self,
        threshold: float | None = None,
        window: float | None = None,
        *,
        previous_q: np.ndarray | None = None,
    ) -> SettleResult:
        """Stillness predicate over the LSQ window.

        ``settled`` is True iff the max per-joint LSQ velocity magnitude is
        ``<= threshold`` (default :data:`CALIB_STILLNESS_DEG_S`). Velocity
        unavailable ⇒ NOT settled with reason ``velocity_unavailable`` (callers
        reject; never publish zeros). When ``previous_q`` is supplied, a pose
        that has not changed vs it (a frozen feed) is also NOT settled
        (``position_unchanged``).
        """
        threshold = self._stillness_deg_s if threshold is None else float(threshold)
        window = self._lsq_window_s if window is None else float(window)
        now = self._now_ns()
        with self._lock:
            n_in_window = sum(1 for s in self._ring if (now - s.t_wall_ns) <= window * 1e9)
            newest = self._ring[-1] if self._ring else None
        if newest is None:
            return SettleResult(False, 0, math.inf, None, "no_samples")
        age_s = float(now - newest.t_wall_ns) / 1e9

        vel = self.estimate_velocity_deg_s(window)
        if vel is None:
            return SettleResult(False, n_in_window, age_s, None, "velocity_unavailable")
        max_vel = float(np.max(np.abs(vel)))
        if max_vel > threshold:
            return SettleResult(False, n_in_window, age_s, max_vel, "moving")
        if previous_q is not None:
            change = self._max_abs_change(newest.q_deg, previous_q)
            if change < self._min_change_deg:
                return SettleResult(False, n_in_window, age_s, max_vel, "position_unchanged")
        return SettleResult(True, n_in_window, age_s, max_vel, "settled")

    # ------------------------------------------------------------------
    # Calibration capture (the hard, raising path)
    # ------------------------------------------------------------------

    def capture_calibration_sample(
        self, *, threshold: float | None = None, window: float | None = None
    ) -> JointSample:
        """Return the current sample as a validated calibration capture, or RAISE.

        Raises (never returns a bad sample):

        * :class:`CalibrationSourceError` — RMI joints while
          ``rmi_to_stream_j3_plus_j2_verified`` is False (the J2/J3 hard-reject),
          or an attempt to mix sources in one dataset.
        * :class:`CalibrationVelocityUnavailable` — velocity is ``None``.
        * :class:`CalibrationError` — no samples, robot still moving, or the
          pose is unchanged vs the previous accepted sample (frozen feed).
        """
        threshold = self._stillness_deg_s if threshold is None else float(threshold)
        window = self._lsq_window_s if window is None else float(window)

        newest = self.latest_joint_sample()
        if newest is None:
            raise CalibrationError("no joint samples available to capture")

        # Source policy: hard-reject rmi_unconverted until the conversion is verified.
        accepted = self._apply_source_policy(newest)

        # Never mix joint representations within one calibration dataset.
        if self._accepted_source is not None and accepted.source != self._accepted_source:
            raise CalibrationSourceError(
                f"cannot mix joint sources in one calibration dataset: already "
                f"accepted {self._accepted_source!r}, now {accepted.source!r}",
                source=accepted.source,
            )

        # Velocity gate — None is a hard reject (never fabricate zeros).
        result = self.settled(threshold=threshold, window=window)
        if result.reason == "velocity_unavailable":
            raise CalibrationVelocityUnavailable(
                f"joint velocity unavailable (samples={result.sample_count}, "
                f"age={result.age_s:.3f}s): refusing to capture — publishing zeros "
                "for an unknown velocity is exactly the corruption this guards against"
            )
        if not result.settled:
            raise CalibrationError(
                f"robot still moving: max joint velocity "
                f"{result.max_velocity_deg_s:.3f} deg/s > {threshold} deg/s "
                "(wait for the robot to stop, then capture)"
            )

        # Frozen-feed / duplicate guard: the accepted pose must differ from the
        # previous accepted one (an operator jogs between captures).
        if self._last_accepted_q is not None:
            change = self._max_abs_change(accepted.q_deg, self._last_accepted_q)
            if change < self._min_change_deg:
                raise CalibrationError(
                    f"measured joints unchanged vs the previous accepted sample "
                    f"(max change {change:.4f} deg < {self._min_change_deg} deg) — "
                    "frozen feed? (the calibration corruption). Jog to a new pose."
                )

        self._last_accepted_q = np.asarray(accepted.q_deg, dtype=np.float64).copy()
        self._accepted_source = accepted.source
        return accepted

    def reset_calibration_dataset(self) -> None:
        """Forget the accepted-sample history (start a fresh calibration run)."""
        self._last_accepted_q = None
        self._accepted_source = None

    # ------------------------------------------------------------------
    # J2/J3 source policy — the single per-model conversion point
    # ------------------------------------------------------------------

    def _apply_source_policy(self, sample: JointSample) -> JointSample:
        if sample.source != SOURCE_RMI_UNCONVERTED:
            return sample
        if not self._facts.rmi_to_stream_j3_plus_j2_verified:
            # HARD reject: whether THIS controller serves the coupled representation is
            # a per-installation configuration, so the conversion is off until it is
            # confirmed. Rejecting is the loud failure; converting on an uncoupled
            # controller would be a silent J2-sized FK error in every derived pose.
            raise CalibrationSourceError(
                "RMI-sourced joints are hard-rejected for calibration until "
                "rmi_to_stream_j3_plus_j2_verified: on a controller serving the "
                "J2/J3-coupled representation RMI J3 is one J2 below the Stream Motion "
                "J3, and applying that conversion unconfirmed — or leaving it off when "
                "it is needed — is a silent J2-sized FK error. Run "
                "examples/verify_j2j3_coupling.py --move to settle it for your cell",
                source=sample.source,
                fact="rmi_to_stream_j3_plus_j2_verified",
            )
        return self._apply_rmi_joint_policy(sample)

    def _apply_rmi_joint_policy(self, sample: JointSample) -> JointSample:
        """Move RMI joints into the Stream Motion frame. SINGLE per-model point.

        Reached ONLY when ``rmi_to_stream_j3_plus_j2_verified`` is True. On a
        controller serving the J2/J3-coupled representation the RMI plane reports J3
        relative to J2 — RMI J3 = SM J3 − J2 — so the conversion is ``q[2] += q[1]``,
        and this method is the one place it belongs.

        Whether a given controller serves that representation is a per-installation
        configuration rather than a property of the arm, which is what the gate is
        for; :mod:`airo_fanuc.controller_facts` records how to settle it.
        """
        q = np.asarray(sample.q_deg, dtype=np.float64).copy()
        q[2] += q[1]
        return replace(sample, q_deg=q, source=SOURCE_RMI_CONVERTED)

    # ------------------------------------------------------------------
    # Poll loops
    # ------------------------------------------------------------------

    def _joint_poll_loop(self) -> None:
        reader = self._joint_reader
        assert reader is not None
        period = self._joint_poll_period_s
        while not self._stop_evt.is_set():
            try:
                sample = reader.read()
            except Exception as exc:  # noqa: BLE001 - a read blip must not kill the poller
                logger.debug("receive: joint reader.read() failed: %s", exc)
                sample = None
            if sample is not None:
                self.ingest_sample(sample)
            self._stop_evt.wait(period)

    def _status_poll_loop(self) -> None:
        rmi = self._rmi
        assert rmi is not None
        period = self._status_poll_period_s
        # GetExtStatus at ~1 Hz regardless of the (faster) GetStatus cadence.
        ext_every = max(1, int(round((1.0 / max(period, 1e-6)) / 1.0)))
        i = 0
        while not self._stop_evt.is_set():
            try:
                status = rmi.get_status()
                with self._status_lock:
                    self._latest_status = status
            except Exception as exc:  # noqa: BLE001 - status poll must never crash
                logger.debug("receive: get_status failed: %s", exc)
            if i % ext_every == 0:
                try:
                    ext = rmi.get_extended_status(quiet=True)
                    with self._status_lock:
                        self._latest_ext_status = ext
                except Exception as exc:  # noqa: BLE001
                    logger.debug("receive: get_extended_status failed: %s", exc)
            i += 1
            self._stop_evt.wait(period)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _max_abs_change(a: np.ndarray, b: np.ndarray) -> float:
        return float(np.max(np.abs(np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64))))


__all__ = [
    "FanucReceiveInterface",
    "JointSample",
    "JointReader",
    "RmiClientJointReader",
    "SettleResult",
    "SOURCE_STREAM",
    "SOURCE_RMI_UNCONVERTED",
    "SOURCE_RMI_CONVERTED",
]
