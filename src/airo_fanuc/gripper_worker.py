# SPDX-License-Identifier: Apache-2.0
"""Serialized register-dispatcher gripper worker over the driver's ``RmiClient``.

Executes a :class:`~airo_fanuc.gripper.RegisterGripperProtocol` against a dispatcher
TP program running on the controller: write the command into registers, then poll the
trigger for completion. Which registers and which values is the protocol's business,
not this module's — everything here is true of any dispatcher of that shape.

Sequence (one physical command):

1. ``write_register(modifier_reg, modifier)`` — the verb's argument, first.
2. ``write_register(action_reg, open_action | close_action)`` — the verb.
3. ``write_register(trigger_reg, 1)`` — LAST, so the dispatcher cannot read a
   half-written command: it may act the moment it sees this.
4. Sleep ``trigger_settle_s`` (0.1 s) so the dispatcher has seen the trigger before
   the first poll, which would otherwise read the pre-trigger 0 and report done.
5. Poll ``read_register(trigger_reg)`` at ``poll_hz`` (20 Hz) until it clears to 0
   (success) or ``dispatch_timeout_s`` (5 s) elapses (timeout).

Polling the trigger is the only completion signal available: the mechanism carries no
feedback, so "done" means the dispatcher cleared the trigger, not that any physical
quantity was measured.

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

from airo_fanuc.gripper import ROBOTIQ_2F85, RegisterGripperProtocol

logger = logging.getLogger("airo_fanuc.gripper")

# Gripper protocol timing. How long a dispatcher takes to answer is the DISPATCHER's
# property, so the values live on the protocol and these are names for the shipped
# preset's — read from it rather than repeated, so there is one place to change.
GRIPPER_TRIGGER_SETTLE_S = ROBOTIQ_2F85.trigger_settle_s
GRIPPER_POLL_HZ = ROBOTIQ_2F85.poll_hz
GRIPPER_DISPATCH_TIMEOUT_S = ROBOTIQ_2F85.dispatch_timeout_s

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

    ``protocol`` defaults to :data:`~airo_fanuc.gripper.ROBOTIQ_2F85`, the shipped
    preset. A different gripper passes its own — the worker's behaviour, timing and
    guarantees are unchanged by which one.
    """

    def __init__(
        self,
        rmi: GripperRmi,
        *,
        protocol: RegisterGripperProtocol = ROBOTIQ_2F85,
        trigger_settle_s: float | None = None,
        poll_hz: float | None = None,
        dispatch_timeout_s: float | None = None,
    ) -> None:
        self._rmi = rmi
        self._proto = protocol
        # The protocol carries the dispatcher's timing; an explicit argument still wins, so a
        # caller holding a worker can tune one knob without rebuilding a protocol for it.
        self._trigger_settle_s = float(
            protocol.trigger_settle_s if trigger_settle_s is None else trigger_settle_s
        )
        self._poll_period_s = 1.0 / float(protocol.poll_hz if poll_hz is None else poll_hz)
        self._dispatch_timeout_s = float(
            protocol.dispatch_timeout_s if dispatch_timeout_s is None else dispatch_timeout_s
        )

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

    def open_gripper(self, open_state: int | None = None) -> None:
        """Open the gripper (non-blocking). Result via :meth:`wait_gripper_done`.

        ``open_state`` is the modifier the dispatcher reads for its open verb — a
        bucket index it defines, never a width in millimetres. ``None`` uses the
        protocol's default. For the shipped Robotiq preset the buckets are
        ``OPEN_FULL`` (~85 mm), ``OPEN_MID`` (~60 mm) and ``OPEN_NARROW`` (~35 mm).

        Raises :class:`ValueError` for a value the protocol does not list, or a
        ``bool``: ``open_state=True`` would otherwise silently select bucket 1,
        opening to a width the caller never asked for.
        """
        if open_state is None:
            open_state = self._proto.default_open_modifier
        self._validate_selector(open_state, self._proto.open_modifiers, "open_state")
        self._submit("open", self._proto.open_action, int(open_state))

    def close_gripper(self, close_force: int | None = None) -> None:
        """Close the gripper (non-blocking). Result via :meth:`wait_gripper_done`.

        ``close_force`` is the modifier the dispatcher reads for its close verb — a
        class index it defines, never a force in newtons. ``None`` uses the protocol's
        default. For the shipped Robotiq preset the classes are ``FORCE_LIGHT``
        (rigid or easily-crushed objects), ``FORCE_MEDIUM`` (the default) and
        ``FORCE_HARD`` (compressible objects that only hold once squeezed).

        Raises :class:`ValueError` for a value the protocol does not list, or a ``bool``.
        """
        if close_force is None:
            close_force = self._proto.default_close_modifier
        self._validate_selector(close_force, self._proto.close_modifiers, "close_force")
        self._submit("close", self._proto.close_action, int(close_force))

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

    @property
    def last_result(self) -> GripperResult | None:
        """The latest completed command's result, or ``None`` if none has completed
        since the last submit. Never blocks.

        The same dict :meth:`wait_gripper_done` returns, read without waiting for it —
        ``wait_gripper_done(timeout=0.0)`` gets there too, but reads like a blocking
        call given a degenerate timeout. Note the two ``None``\\ s differ: this one
        means "no verdict yet", that one means "I stopped waiting".
        """
        with self._result_lock:
            return None if self._result is None else dict(self._result)

    @property
    def protocol(self) -> RegisterGripperProtocol:
        """The :class:`~airo_fanuc.gripper.RegisterGripperProtocol` this worker
        executes — which registers it writes and which modifier values they accept.

        A caller wrapping the worker in something that assumes particular buckets (a
        width in millimetres, a force in newtons) can check that assumption against
        the protocol instead of trusting it.
        """
        return self._proto

    # ------------------------------------------------------------------
    # Blocking convenience API
    # ------------------------------------------------------------------

    def open_gripper_and_wait(
        self, open_state: int | None = None, timeout: float | None = None
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
        self, close_force: int | None = None, timeout: float | None = None
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

    def _validate_selector(self, value: object, valid: tuple[int, ...], field: str) -> None:
        # bool is an int subclass — reject explicitly so True/False can't sneak
        # in as 1/0 and select a bucket the caller never named.
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{field} must be an int in {valid}, got {type(value).__name__} {value!r}")
        if value not in valid:
            raise ValueError(f"{field} must be one of {valid} for {self._proto.name}, got {value!r}")

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
        logger.info("gripper: command %s (R[%d]=%d)", action, self._proto.modifier_reg, modifier)
        try:
            self._rmi.write_register(self._proto.modifier_reg, modifier)
            self._rmi.write_register(self._proto.action_reg, action_reg)
            self._rmi.write_register(self._proto.trigger_reg, 1)
        except Exception as exc:  # noqa: BLE001 - any RMI failure → dict-fail, never crash the worker
            logger.error("gripper: register write failed: %s", exc)
            self._finish(False, f"failed to write trigger registers: {exc}")
            return

        # Settle window before polling (interruptible on shutdown).
        if self._stop_evt.wait(timeout=self._trigger_settle_s):
            self._finish(False, "gripper worker stopped during settle")
            return

        trigger = self._proto.trigger_reg
        deadline = time.monotonic() + self._dispatch_timeout_s
        while time.monotonic() < deadline:
            if self._stop_evt.is_set():
                self._finish(False, "gripper worker stopped during poll")
                return
            try:
                cmd = self._rmi.read_register(trigger)
            except Exception as exc:  # noqa: BLE001 - any RMI failure → dict-fail
                logger.error("gripper: poll read of R[%d] failed: %s", trigger, exc)
                self._finish(False, f"failed to read R[{trigger}]: {exc}")
                return
            if cmd == 0.0:
                logger.info("gripper: %s complete", action)
                self._finish(True, f"gripper {action} complete")
                return
            if self._stop_evt.wait(timeout=self._poll_period_s):
                self._finish(False, "gripper worker stopped during poll")
                return

        logger.error("gripper: %s timed out (R[%d] not cleared)", action, trigger)
        self._finish(
            False,
            f"gripper {action} timed out (R[{trigger}] not cleared after "
            f"{self._dispatch_timeout_s:.1f}s — is {self._proto.dispatcher_program} running?)",
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
