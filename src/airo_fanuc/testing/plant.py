# SPDX-License-Identifier: Apache-2.0
"""First-order-lag joint plant + deviation watchdog for the FakeCRXController.

Given a stream of commanded joint angles at the controller ITP (8 ms), it
produces the *measured* joint angles a controller of this shape would report back
in its status packet (type-202 at the default v3; type-204 at v4), so tests that
need a closed loop have one.

**This model is a stand-in, not an authority.** Each effect below is pinned to a
number measured on the physical controller, but three first-order effects are not
a CRX: it has no joint coupling, no gravity or payload term, no friction, no
drive-train compliance and no controller-side interpolation of its own. Where the
model and the arm disagree, the ARM is right. So it is sound to test *logic*
against — did the driver react to a fault, did the lifecycle advance, was the
command well-formed — and it settles nothing quantitative about tracking,
following error, servo fidelity or timing. Those belong on hardware, or in the
C++ suite where the core can be driven deterministically in-process.

Three modeled effects, each pinned to a value measured on the physical
controller:

* **First-order servo lag** — ``q_meas`` relaxes toward ``q_cmd`` with time
  constant ``tau_s`` (measured servo lag: 25 ms, held in
  ``MEASURED_FACTS.tracking_lag_s``).
* **Deviation watchdog** — a commanded per-tick position *step* larger than
  ``deviation_watchdog_deg`` (``MEASURED_FACTS.deviation_watchdog_deg`` = 5.0 deg;
  worst measured overrun was 4.63° at 49.9°/s, so 5.0° is well-supported) trips a
  controller fault. This emulates the CRX deviation watchdog that the per-tick
  slew clip + C1-continuity exist to avoid; a smooth ramp (small per-tick delta)
  never trips it, an un-ramped step does.
* **TX-silence backstop** — with no fresh command, the controller does NOT
  fast-decel (measured: ``MEASURED_FACTS.tx_silence_backstop_ok`` is False). It
  coasts at the last commanded velocity ~120 ms then hard-stops on the deviation
  watchdog. The knob defaults to the measured (coast) case; set it True only to
  model a hypothetical fast-backstop controller.

This module has no I/O and no controller-protocol knowledge; it also hosts the
low-level simulation primitives (the injectable :class:`Clock`) shared by the
UDP/TCP servers so those modules import only from this leaf.
"""

from __future__ import annotations

import math
import time
from typing import Protocol, runtime_checkable

import numpy as np

from airo_fanuc.controller_facts import ITP_S, MEASURED_FACTS

# CRX-10iA/L is 6-DOF; Stream Motion carries 9 axes (MAX_AXES) with the trailing
# three zero-padded. The plant works in the 9-axis wire frame so the SM server
# can hand its arrays straight to the wire codec.
N_AXES: int = 9


# ---------------------------------------------------------------------------
# Injectable clock (shared low-level primitive).
# ---------------------------------------------------------------------------


@runtime_checkable
class Clock(Protocol):
    """Minimal monotonic clock the fake reads for timestamps / spacing checks."""

    def now(self) -> float:
        """Return monotonic seconds (float)."""
        ...


class WallClock:
    """Real ``time.monotonic`` clock — used by the optional real-time driver."""

    __slots__ = ()

    def now(self) -> float:
        return time.monotonic()


class ManualClock:
    """Steppable clock for deterministic, wall-time-free scenario tests.

    Scenario/lifecycle agents drive the fake by advancing this clock in ITP
    increments (usually via :meth:`FakeCRXController.tick` /
    :meth:`~FakeCRXController.advance`, which advance the clock then pump one
    controller tick). Nothing in the fake sleeps on wall time while a
    ``ManualClock`` is installed.
    """

    __slots__ = ("_t",)

    def __init__(self, start: float = 0.0) -> None:
        self._t = float(start)

    def now(self) -> float:
        return self._t

    def advance(self, dt: float) -> float:
        """Advance by ``dt`` seconds; return the new time."""
        self._t += float(dt)
        return self._t

    def set(self, t: float) -> None:
        self._t = float(t)


# ---------------------------------------------------------------------------
# Joint plant.
# ---------------------------------------------------------------------------


class JointPlant:
    """Per-joint first-order-lag servo model with a deviation watchdog.

    All angles are **degrees** (wire units). Deterministic given a ``dt`` — no
    RNG, no wall-clock — so replaying the same command sequence yields
    bit-identical measured trajectories.
    """

    def __init__(
        self,
        *,
        tau_s: float | None = None,
        deviation_watchdog_deg: float | None = None,
        deviation_watchdog_enabled: bool = True,
        silence_decel_ticks: int = 3,
        itp_s: float = ITP_S,
        initial_q_deg: np.ndarray | list[float] | None = None,
    ) -> None:
        # τ — servo tracking lag; reads MEASURED_FACTS.tracking_lag_s
        # (measured servo lag: 25 ms).
        self.tau_s: float = float(tau_s if tau_s is not None else MEASURED_FACTS.tracking_lag_s)
        if self.tau_s <= 0.0:
            raise ValueError(f"tau_s must be > 0 (got {self.tau_s})")

        # Deviation-watchdog per-tick step threshold; reads
        # MEASURED_FACTS.deviation_watchdog_deg (5.0°; worst measured overrun
        # 4.63° at 49.9°/s). Modeled as a per-tick step bound.
        self.deviation_watchdog_deg: float = float(
            deviation_watchdog_deg
            if deviation_watchdog_deg is not None
            else MEASURED_FACTS.deviation_watchdog_deg
        )
        self.deviation_watchdog_enabled = bool(deviation_watchdog_enabled)
        self.silence_decel_ticks = max(1, int(silence_decel_ticks))
        self.itp_s = float(itp_s)

        self.q_meas: np.ndarray = np.zeros(N_AXES, dtype=np.float64)
        self.qd_meas: np.ndarray = np.zeros(N_AXES, dtype=np.float64)
        self._target: np.ndarray = np.zeros(N_AXES, dtype=np.float64)
        self._prev_cmd: np.ndarray | None = None

        # Deviation-watchdog trip latch.
        self.watchdog_tripped: bool = False
        self.watchdog_joint: int | None = None
        self.watchdog_step_deg: float = 0.0

        # TX-silence backstop state.
        self._in_silence: bool = False
        self._silence_qd0: np.ndarray = np.zeros(N_AXES, dtype=np.float64)
        self._silence_ticks: int = 0

        if initial_q_deg is not None:
            self.set_initial(initial_q_deg)

    # -- setup ------------------------------------------------------------

    def set_initial(self, q_deg: np.ndarray | list[float]) -> None:
        """Place the plant at rest at ``q_deg`` (9-axis, degrees)."""
        q = self._as9(q_deg)
        self.q_meas = q.copy()
        self.qd_meas = np.zeros(N_AXES, dtype=np.float64)
        self._target = q.copy()
        # Seed prev_cmd so the FIRST real command's step is measured against the
        # rest pose (a large first command IS an un-ramped step and should trip).
        self._prev_cmd = q.copy()
        self._in_silence = False
        self._silence_ticks = 0

    def reset_watchdog(self) -> None:
        """Clear the deviation-watchdog latch (post-recovery)."""
        self.watchdog_tripped = False
        self.watchdog_joint = None
        self.watchdog_step_deg = 0.0

    # -- stepping ---------------------------------------------------------

    def command_step(self, q_cmd_deg: np.ndarray | list[float], dt: float) -> None:
        """Apply one commanded setpoint and integrate the lag for ``dt`` seconds.

        Runs the deviation watchdog against the previous *command* (per-tick
        step), then relaxes ``q_meas`` toward the command with the first-order
        response. Clears any TX-silence backstop.
        """
        q_cmd = self._as9(q_cmd_deg)

        if self.deviation_watchdog_enabled and self._prev_cmd is not None:
            step = np.abs(q_cmd - self._prev_cmd)
            j = int(np.argmax(step))
            if step[j] > self.deviation_watchdog_deg and not self.watchdog_tripped:
                self.watchdog_tripped = True
                self.watchdog_joint = j
                self.watchdog_step_deg = float(step[j])

        self._prev_cmd = q_cmd.copy()
        self._target = q_cmd.copy()
        self._in_silence = False
        self._silence_ticks = 0
        self._integrate_lag(dt)

    def silence_step(self, dt: float, *, backstop_ok: bool) -> None:
        """Integrate one tick with NO fresh command (TX-silence).

        ``backstop_ok=False`` is the measured controller behavior and the default:
        no active backstop — the plant keeps tracking the last command with the
        servo lag (the controller coasts, does not fast-decel). ``backstop_ok=True``
        models a hypothetical fast-backstop controller that decelerates ``qd``
        linearly to zero over ``silence_decel_ticks`` and holds the frozen pose.
        """
        if not backstop_ok:
            # No safe backstop: keep relaxing toward the last commanded target.
            self._integrate_lag(dt)
            return

        if not self._in_silence:
            self._in_silence = True
            self._silence_ticks = 0
            self._silence_qd0 = self.qd_meas.copy()

        self._silence_ticks += 1
        n = self.silence_decel_ticks
        if self._silence_ticks >= n:
            # Fully decelerated: hold in place.
            self.qd_meas = np.zeros(N_AXES, dtype=np.float64)
        else:
            frac = 1.0 - (self._silence_ticks / n)
            self.qd_meas = self._silence_qd0 * frac
        self.q_meas = self.q_meas + self.qd_meas * dt
        # Freeze the tracking target at the (decelerating) current pose so a
        # later resume captures from where we actually stopped.
        self._target = self.q_meas.copy()

    def _integrate_lag(self, dt: float) -> None:
        alpha = 1.0 - math.exp(-dt / self.tau_s)
        q_prev = self.q_meas.copy()
        self.q_meas = self.q_meas + (self._target - self.q_meas) * alpha
        self.qd_meas = (self.q_meas - q_prev) / dt if dt > 0.0 else np.zeros(N_AXES)

    # -- helpers ----------------------------------------------------------

    @property
    def in_silence(self) -> bool:
        return self._in_silence

    def snapshot_deg(self) -> tuple[np.ndarray, np.ndarray]:
        """Return copies of ``(q_meas, qd_meas)`` in degrees (9-axis)."""
        return self.q_meas.copy(), self.qd_meas.copy()

    @staticmethod
    def _as9(q: np.ndarray | list[float]) -> np.ndarray:
        arr = np.asarray(q, dtype=np.float64).ravel()
        if arr.shape[0] == N_AXES:
            return arr.astype(np.float64, copy=True)
        if arr.shape[0] < N_AXES:
            out = np.zeros(N_AXES, dtype=np.float64)
            out[: arr.shape[0]] = arr
            return out
        raise ValueError(f"joint vector longer than {N_AXES} axes: {arr.shape[0]}")
