# SPDX-License-Identifier: Apache-2.0
"""Serialized Robotiq 2F-85 gripper worker over the driver's ``RmiClient``.

Drives the controller's ``GRIPDISP`` TP program by writing its trigger registers
and polling for completion. The register contract itself — and the fact that the
R[3] modifier means a *width bucket* on open and a *force class* on close — is in
:mod:`airo_fanuc.gripper`.

Sequence (one physical command):

1. ``rmi.write_register(REG_R3, modifier)`` — action-dependent modifier
   (open → open-state selector; close → close-force selector).
2. ``rmi.write_register(REG_ACTION, ACTION_OPEN | ACTION_CLOSE)``.
3. ``rmi.write_register(REG_CMD, 1)`` — trigger the dispatcher.
4. Sleep ``trigger_settle_s`` (0.1 s) so the controller sees the trigger
   before we start polling.
5. Poll ``rmi.read_register(REG_CMD)`` at ``poll_hz`` (20 Hz) until it clears
   to 0 (success) or ``dispatch_timeout_s`` (5 s) elapses (timeout).

Polling R[1] is the only completion signal available: the protocol exposes no
width feedback, so "done" means the TP program cleared the trigger, not that a
measured width was reached.

Design invariants:

* **Serialized** — one physical gripper, so a single-worker
  :class:`~concurrent.futures.ThreadPoolExecutor` runs the sequence; concurrent
  ``open``/``close`` calls are ordered, never interleaved on the wire.
* **Fail-fast during recovery** — the supervisor toggles :meth:`set_recovery`;
  while recovery is active a command is *rejected fast* (never queued behind the
  5 s poll loop, which would otherwise hold the RMI path and stall the
  recovery ladder's ``FRC_Reset``).
* **Never hang** — a stuck GRIPDISP times out into ``{"success": False, ...}``.
* **Result is dict-or-None** ``{"success": bool, "message": str}``;
  :meth:`wait_gripper_done` returns ``None`` only if it times out waiting for the
  worker, which is a distinct outcome from the worker's own dict-fail and must
  stay distinguishable by callers.
* Bad ``open_state`` / ``close_force`` (out of range, or a ``bool``) raise
  :class:`ValueError` synchronously — programmer error, fail loud rather than
  silently gripping at some other bucket.

Dependency-light: stdlib ``logging`` / ``threading`` / ``concurrent.futures``
only. The injected RMI client is duck-typed (:class:`GripperRmi`), so the real
:class:`~airo_fanuc.rmi_client.RmiClient` and a test double both satisfy it.
"""

from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Protocol

from airo_fanuc.gripper import (
    ACTION_CLOSE,
    ACTION_OPEN,
    DEFAULT_CLOSE_FORCE,
    OPEN_FULL,
    REG_ACTION,
    REG_CMD,
    REG_R3,
    VALID_CLOSE_FORCES,
    VALID_OPEN_STATES,
)

logger = logging.getLogger("airo_fanuc.gripper")

# Gripper protocol timing — the worker defaults for the settle window before the
# first poll, the poll cadence, and the bound on one dispatch (see the module
# docstring's sequence).
GRIPPER_TRIGGER_SETTLE_S = 0.1
GRIPPER_POLL_HZ = 20.0
GRIPPER_DISPATCH_TIMEOUT_S = 5.0

#: Result type: dict-or-None ``{"success": bool, "message": str}``.
GripperResult = dict[str, object]


class GripperRmi(Protocol):
    """The subset of :class:`~airo_fanuc.rmi_client.RmiClient` the worker needs.

    Duck-typed so the worker can be unit-tested against a fake and never has to
    import the real client. Both methods are worker-safe on the commands-only
    session and may raise (``RmiError`` / ``RmiSessionDown`` / ``OSError``); the
    worker converts any raise into a ``{"success": False}`` result.
    """

    def read_register(self, register_number: int) -> float: ...

    def write_register(self, register_number: int, value: float | int) -> None: ...


class GripperWorker:
    """Serialized GRIPDISP register worker over an injected ``RmiClient``.

    Construct once with the driver's RMI client, then call :meth:`open_gripper`
    / :meth:`close_gripper` (non-blocking submit) followed by
    :meth:`wait_gripper_done`, or the blocking :meth:`open_gripper_and_wait` /
    :meth:`close_gripper_and_wait`. :meth:`close` shuts the worker down.
    """

    def __init__(
        self,
        rmi: GripperRmi,
        *,
        trigger_settle_s: float = GRIPPER_TRIGGER_SETTLE_S,
        poll_hz: float = GRIPPER_POLL_HZ,
        dispatch_timeout_s: float = GRIPPER_DISPATCH_TIMEOUT_S,
    ) -> None:
        self._rmi = rmi
        self._trigger_settle_s = float(trigger_settle_s)
        self._poll_period_s = 1.0 / float(poll_hz)
        self._dispatch_timeout_s = float(dispatch_timeout_s)

        # Single worker => at most one physical command in flight (serialized).
        self._exec = ThreadPoolExecutor(max_workers=1, thread_name_prefix="airo-fanuc-gripper")
        self._stop_evt = threading.Event()

        # Latest-command completion signalling + result. Starts "done" so a
        # wait_gripper_done before any command returns instead of blocking.
        self._done = threading.Event()
        self._done.set()
        self._result_lock = threading.Lock()
        self._result: GripperResult | None = None

        # Recovery gate (supervisor-toggled). While set, commands fail fast.
        self._recovery_lock = threading.Lock()
        self._recovering = False

    # ------------------------------------------------------------------
    # Recovery gate (supervisor-facing)
    # ------------------------------------------------------------------

    def set_recovery(self, active: bool) -> None:
        """Toggle the fail-fast recovery gate (supervisor-only).

        While ``active``, :meth:`open_gripper` / :meth:`close_gripper` reject
        immediately with a ``{"success": False}`` result instead of submitting
        the 5 s poll loop, so the RMI path stays free for the recovery ladder's
        ``FRC_Reset``.
        """
        with self._recovery_lock:
            self._recovering = bool(active)

    def _is_recovering(self) -> bool:
        with self._recovery_lock:
            return self._recovering

    # ------------------------------------------------------------------
    # Non-blocking submit API
    # ------------------------------------------------------------------

    def open_gripper(self, open_state: int = OPEN_FULL) -> None:
        """Open the gripper (non-blocking). Result via :meth:`wait_gripper_done`.

        ``open_state`` selects the open width via R[3] — one of three discrete
        buckets, never a width in millimetres: ``OPEN_FULL`` (0, POSITION 0,
        ~85 mm), ``OPEN_MID`` (1, POSITION 75, ~60 mm), or ``OPEN_NARROW``
        (2, POSITION 150, ~35 mm).

        Raises :class:`ValueError` for an out-of-range value or a ``bool``:
        ``open_state=True`` would otherwise silently mean ``OPEN_MID`` (1),
        opening to a width the caller never asked for.
        """
        self._validate_selector(open_state, VALID_OPEN_STATES, "open_state")
        self._submit("open", ACTION_OPEN, int(open_state))

    def close_gripper(self, close_force: int = DEFAULT_CLOSE_FORCE) -> None:
        """Close the gripper (non-blocking). Result via :meth:`wait_gripper_done`.

        ``close_force`` selects the close target/force pair via R[3] — one of
        three discrete force classes, never a force in newtons: ``FORCE_LIGHT``
        (0, POSITION 220 / FORCE 100; rigid or easily-crushed objects),
        ``FORCE_MEDIUM`` (1, POSITION 220 / FORCE 150; default), ``FORCE_HARD``
        (2, POSITION 255 / FORCE 255; compressible objects).

        Raises :class:`ValueError` for an out-of-range value or a ``bool``.
        """
        self._validate_selector(close_force, VALID_CLOSE_FORCES, "close_force")
        self._submit("close", ACTION_CLOSE, int(close_force))

    def wait_gripper_done(self, timeout: float | None = None) -> GripperResult | None:
        """Block until the latest command completes; return its dict-or-None result.

        Returns the worker's ``{"success", "message"}`` dict, or ``None`` if the
        wait times out (the worker is still running — distinct from a
        ``{"success": False}`` dict-fail the worker itself produced).
        """
        if not self._done.wait(timeout=timeout):
            logger.warning("gripper: wait_gripper_done timed out after %ss", timeout)
            return None
        with self._result_lock:
            return None if self._result is None else dict(self._result)

    def is_gripper_done(self) -> bool:
        """True iff the latest submitted command has completed."""
        return self._done.is_set()

    # ------------------------------------------------------------------
    # Blocking convenience API
    # ------------------------------------------------------------------

    def open_gripper_and_wait(
        self, open_state: int = OPEN_FULL, timeout: float | None = None
    ) -> GripperResult | None:
        """:meth:`open_gripper` then :meth:`wait_gripper_done`.

        ``timeout=None`` waits ``dispatch_timeout_s`` + settle + 1 s margin so
        the wait reliably captures the worker's own timeout dict-fail rather
        than returning a premature ``None`` (the poll loop is bounded by
        ``dispatch_timeout_s`` and always sets a result).
        """
        self.open_gripper(open_state=open_state)
        return self.wait_gripper_done(timeout=self._default_wait_timeout(timeout))

    def close_gripper_and_wait(
        self, close_force: int = DEFAULT_CLOSE_FORCE, timeout: float | None = None
    ) -> GripperResult | None:
        """:meth:`close_gripper` then :meth:`wait_gripper_done` (see timeout note)."""
        self.close_gripper(close_force=close_force)
        return self.wait_gripper_done(timeout=self._default_wait_timeout(timeout))

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Stop the worker: signal any in-flight poll loop and drain the executor. Idempotent."""
        self._stop_evt.set()
        self._exec.shutdown(wait=False, cancel_futures=True)

    def __enter__(self) -> GripperWorker:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _default_wait_timeout(self, timeout: float | None) -> float | None:
        if timeout is not None:
            return timeout
        return self._dispatch_timeout_s + self._trigger_settle_s + 1.0

    @staticmethod
    def _validate_selector(value: object, valid: tuple[int, ...], field: str) -> None:
        # bool is an int subclass — reject explicitly so True/False can't sneak
        # in as 1/0 and select a bucket the caller never named.
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{field} must be an int in {valid}, got {type(value).__name__} {value!r}")
        if value not in valid:
            raise ValueError(f"{field} must be one of {valid}, got {value!r}")

    def _submit(self, action: str, action_reg: int, modifier: int) -> None:
        """Reset the done-latch and hand the sequence to the single worker.

        Fast-rejects (no executor submit) while recovery is active or the worker
        is shutting down.
        """
        if self._is_recovering():
            self._finish(
                False,
                "RMI recovery in progress — refusing gripper command (retry once recovery has completed)",
            )
            return
        if self._stop_evt.is_set():
            self._finish(False, "gripper worker is shutting down")
            return
        # Arm the completion latch BEFORE submitting so a wait started right
        # after this call observes the in-flight state, not a stale result.
        self._done.clear()
        with self._result_lock:
            self._result = None
        try:
            self._exec.submit(self._execute, action, action_reg, modifier)
        except RuntimeError:
            # Executor already shut down (close() raced the submit).
            self._finish(False, "gripper worker is shutting down")

    def _execute(self, action: str, action_reg: int, modifier: int) -> None:
        """Run the GRIPDISP poke→settle→poll sequence on the worker thread."""
        # Re-check the recovery gate on the worker thread: the supervisor may
        # have entered recovery between submit and execution.
        if self._is_recovering():
            self._finish(False, "RMI recovery in progress — refusing gripper command")
            return
        logger.info("gripper: command %s (R[%d]=%d)", action, REG_R3, modifier)
        try:
            self._rmi.write_register(REG_R3, modifier)
            self._rmi.write_register(REG_ACTION, action_reg)
            self._rmi.write_register(REG_CMD, 1)
        except Exception as exc:  # noqa: BLE001 - any RMI failure → dict-fail, never crash the worker
            logger.error("gripper: register write failed: %s", exc)
            self._finish(False, f"failed to write trigger registers: {exc}")
            return

        # Settle window before polling (interruptible on shutdown).
        if self._stop_evt.wait(timeout=self._trigger_settle_s):
            self._finish(False, "gripper worker stopped during settle")
            return

        deadline = time.monotonic() + self._dispatch_timeout_s
        while time.monotonic() < deadline:
            if self._stop_evt.is_set():
                self._finish(False, "gripper worker stopped during poll")
                return
            try:
                cmd = self._rmi.read_register(REG_CMD)
            except Exception as exc:  # noqa: BLE001 - any RMI failure → dict-fail
                logger.error("gripper: poll read of R[%d] failed: %s", REG_CMD, exc)
                self._finish(False, f"failed to read R[{REG_CMD}]: {exc}")
                return
            if cmd == 0.0:
                logger.info("gripper: %s complete", action)
                self._finish(True, f"gripper {action} complete")
                return
            if self._stop_evt.wait(timeout=self._poll_period_s):
                self._finish(False, "gripper worker stopped during poll")
                return

        logger.error("gripper: %s timed out (R[%d] not cleared)", action, REG_CMD)
        self._finish(
            False,
            f"gripper {action} timed out (R[{REG_CMD}] not cleared after "
            f"{self._dispatch_timeout_s:.1f}s — is GRIPDISP running?)",
        )

    def _finish(self, success: bool, message: str) -> None:
        with self._result_lock:
            self._result = {"success": success, "message": message}
        self._done.set()


__all__ = [
    "GripperWorker",
    "GripperRmi",
    "GripperResult",
    "GRIPPER_TRIGGER_SETTLE_S",
    "GRIPPER_POLL_HZ",
    "GRIPPER_DISPATCH_TIMEOUT_S",
]
