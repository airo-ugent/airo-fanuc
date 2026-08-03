# SPDX-License-Identifier: Apache-2.0
"""JSON-over-TCP client for the FANUC Remote Motion Interface (option R912 / S636).

Every request is built with the pinned :mod:`airo_fanuc.testing.wire`
serializers, so the bytes this client puts on the wire are exactly the ones the
goldens in ``tests/goldens/rmi/`` assert. Responsibilities are split along one
axis: **a worker's socket blip must never abort a live trajectory.**

Controller behaviors this client encodes
----------------------------------------
* The full ``ERROR_ID → text`` table (2556929–2556981). The block is easy to
  mis-transcribe from 2556941 onward: 2556946 is "Invalid Motion Option", NOT
  "Wait for Command Done".
* Correlate-by-echo request/response matching (the controller echoes the
  request's ``Command`` / ``Communication`` / ``Instruction`` identifier).
* The async push ring: ``FRC_SystemFault`` / ``FRC_Terminate`` /
  ``FRC_AsbnReady`` pushes and orphan replies (e.g. a lingering fire-and-forget
  ``FRC_Call`` ack) are stashed off-band and drained by
  :meth:`RmiClient.poll_async_packets`.
* ``Connect_STMO`` → redirect-port hop.
* SequenceID reseed after Initialize (``FRC_Initialize`` does NOT reset the
  controller's ``NextSequenceID`` on R-30iB+ v9.0 — it persists across sessions
  until power-cycle, so we re-anchor from ``FRC_GetStatus.NextSequenceID``).
* The Init recovery ladder: GetStatus→Reset→GetStatus→Initialize; on error
  Abort→Reset→GetStatus→Initialize.

The session split (the central design constraint)
-------------------------------------------------
1. **Commands-only session.** :meth:`start` establishes a session with
   ``Connect_STMO`` + the redirect-port hop and *nothing else*. ``GetStatus`` /
   ``ReadError`` / ``ReadRegister`` / ``WriteRegister`` / ``GetExtStatus`` /
   ``program_call`` all work pre-Initialize (proven by the bring-up ladder
   itself, which issues GetStatus/Reset *before* Initialize). Transport
   auto-reopen re-establishes the socket **without** issuing
   Initialize/Abort — a bounded, single-flight transport concern.
2. **Explicit** :meth:`initialize`. Runs the Init recovery ladder + seq reseed.
   Invoked ONLY by the lifecycle supervisor, never implicitly on reconnect.
3. **Typed** :class:`~airo_fanuc.exceptions.RmiSessionDown`. Workers that hit a
   persistently dead session get this instead of an implicit reopen (avoids the
   concurrent-reopen 2556954 self-infliction and the gripper TOCTOU).
4. :meth:`abort` is **supervisor-only** and is *not* on the auto-reopen path —
   a worker's socket blip must never abort a live ``STREAM_MOTN``.

Threading model
---------------
A single long-lived **background RX thread** is the sole reader of the redirect
session socket: it reads + buffers + dispatches every packet (push → async ring,
matched reply → the one in-flight waiter, orphan → async ring). Reading on a
dedicated thread rather than synchronously inside each request is what surfaces
``FRC_SystemFault`` pushes promptly — the supervisor drains the ring at a ~1 s
cadence and a fault must not wait for the next request to be noticed. The ring
is *preserved across session rebuilds* (a fault that killed the session is
exactly the one worth reporting, so reopening must not discard it).

Two locks with a strict order (``_request_lock`` → ``_state_lock``, never the
reverse) keep it deadlock-free: ``_request_lock`` serializes request/response
cycles (so at most one waiter is ever pending), ``_state_lock`` guards the
shared socket / pending-waiter / async-ring / seq-id state and is held only
briefly. The RX thread only ever takes ``_state_lock``.

The bootstrap ``Connect_STMO`` handshake is the *one* synchronous read outside
the RX thread: it runs on a throwaway bootstrap socket while ``self._sock`` is
``None`` (RX thread idle-spinning), so there is no contention for the redirect
socket.

Dependency-light: stdlib ``logging`` / ``socket`` / ``json`` / ``threading`` /
``queue`` only — the package's sole runtime dependency is numpy. Sockets are
O_CLOEXEC.
"""

from __future__ import annotations

import argparse
import json
import logging
import queue
import socket
import threading
from collections import deque
from dataclasses import dataclass
from typing import Any

from airo_fanuc.exceptions import FanucConnectionError, RmiError, RmiSessionDown
from airo_fanuc.testing import wire

logger = logging.getLogger("airo_fanuc.rmi")


# ---------------------------------------------------------------------------
# Push-packet identifiers.  Anything matching one of these is consumed by the
# RX thread independently of the request flow and stashed for diagnostics.
# Mirrors fanuc_libs/rmi/include/rmi/packets.hpp lines 106-116, 1008-1013.
# ---------------------------------------------------------------------------
_PUSH_PACKET_COMMS = frozenset({"FRC_SystemFault", "FRC_Terminate", "FRC_AsbnReady"})
# Request-reply correlation lookup keys, in priority order.  The controller
# echoes the *same* field on the response that the request used as its top-
# level identifier ("Command" first, then "Communication" / "Instruction").
_REQ_REPLY_KEYS: tuple[str, ...] = ("Command", "Communication", "Instruction")

# FRC_Continue of a program that is not paused returns this ErrorID; the
# supervisor recovery ladder treats it as a no-op success (an already-running
# program is the state the ladder was trying to reach).
_ERR_TP_NOT_PAUSED: int = 2556938  # "TP Program is Not Paused." (see _RMI_ERROR_CODES)


# ---------------------------------------------------------------------------
# FANUC RMI error-code → human-readable.  Verbatim from
# fanuc_libs/rmi/src/rmi.cpp lines 25-81 (``kErrorCodes`` table) of the
# FANUC America reference driver (Apache-2.0).  A hand-curated table drifts
# out of alignment easily from 2556941 onward, which produces misleading log
# lines like "2556946 = Wait for Command Done" when the controller actually
# means "Invalid Motion Option."  Keep IN SYNC with the reference and do NOT
# edit individual entries; if FANUC extends the table, regenerate from the
# reference.
# ---------------------------------------------------------------------------
_RMI_ERROR_CODES: dict[int, str] = {
    2556929: "Internal System Error.",
    2556930: "Invalid UTool Number.",
    2556931: "Invalid UFrame Number.",
    2556932: "Invalid Position Register.",
    2556933: "Invalid Speed Override.",
    2556934: "Cannot Execute TP program.",
    2556935: "Controller Servo is Off.",
    2556936: "Cannot Execute TP program.",
    2556937: "RMI is Not Running.",
    2556938: "TP Program is Not Paused.",
    2556939: "Cannot Resume TP Program.",
    2556940: "Cannot Reset Controller.",
    2556941: "Invalid RMI Command.",
    2556942: "RMI Command Fail.",
    2556943: "Invalid Controller State.",
    2556944: "Please Cycle Power.",
    2556945: "Invalid Payload Schedule.",
    2556946: "Invalid Motion Option.",
    2556947: "Invalid Vision Register.",
    2556948: "Invalid RMI Instruction.",
    2556949: "Invalid Value.",
    2556950: "Invalid Text String",
    2556951: "Invalid Position Data",
    2556952: "RMI is In HOLD State",
    2556953: "Remote Device Disconnected.",
    2556954: "Robot is Already Connected.",
    2556955: "Wait for Command Done.",
    2556956: "Wait for Instruction Done.",
    2556957: "Invalid sequence ID number.",
    2556958: "Invalid Speed Type.",
    2556959: "Invalid Speed Value.",
    2556960: "Invalid Term Type.",
    2556961: "Invalid Term Value.",
    2556962: "Invalid LCB Port Type.",
    2556963: "Invalid ACC Value.",
    2556964: "Invalid Destination Position",
    2556965: "Invalid VIA Position.",
    2556966: "Invalid Port Number.",
    2556967: "Invalid Group Number",
    2556968: "Invalid Group Mask",
    2556969: "Joint motion with COORD",
    2556970: "Incremental motn with COORD",
    2556971: "Robot in Single Step Mode",
    2556972: "Invalid Position Data Type",
    2556973: "Ready for ASCII Packet",
    2556974: "ASCII Conversion Failed",
    2556975: "Invalid ASCII Instruction",
    2556976: "Invalid Number of Groups",
    2556977: "Invalid Instruction packet",
    2556978: "Invalid ASCII String packet",
    2556979: "Invalid ASCII string size",
    2556980: "Invalid Application Tool",
    2556981: "Invalid Call Program Name",
    7015: ("RMI_MOVE is the selected TP program. Select a different TP program and re-run RMI PC code."),
    7004: "The specific program is in use.",
}


def describe_rmi_error_id(error_id: int) -> str:
    """Map a FANUC RMI ErrorID to a short human-readable description.

    Returns ``"Unknown Error Code: <n>"`` for IDs not in the table —
    matches the C++ reference's ``LookupErrorCode`` (rmi.cpp:93) so
    operators reading the log line still see the numeric ID.
    """
    eid = int(error_id)
    text = _RMI_ERROR_CODES.get(eid)
    if text is not None:
        return text
    return f"Unknown Error Code: {eid}"


@dataclass(frozen=True)
class ExtendedStatus:
    """Decoded ``FRC_GetExtStatus`` reply."""

    in_motion: bool
    drives_powered: bool
    gen_override_pct: int
    speed_clamp_limit_pct: float | None
    error_code: str | None
    control_mode: str | None


@dataclass(frozen=True)
class ControllerStatus:
    """Decoded ``FRC_GetStatus`` reply (FANUC RMI §2.3.7).

    Mirrors ``rmi/include/rmi/packets.hpp`` ``StatusRequestPacket::Response``.
    The most operationally-important field is :attr:`next_sequence_id` — the
    controller's authoritative next-expected SequenceID for instruction
    packets.  Use it to re-anchor :attr:`RmiClient._instruction_seq_id` after
    session recovery so a fresh ``FRC_Call`` is not silently dropped as a
    duplicate.
    """

    servo_ready: bool
    tp_mode: int
    rmi_motion_status: int
    program_status: int
    single_step_mode: bool
    number_utool: int
    next_sequence_id: int
    number_uframe: int
    override_pct: int


@dataclass(frozen=True)
class CartesianPosition:
    """Decoded ``FRC_ReadCartesianPosition`` reply (FANUC RMI §2.3.14).

    Mirrors ``rmi/include/rmi/packets.hpp`` ``GetCartesianPositionPacket::Response``.
    :attr:`xyzwpr` is the controller's own FK result — X, Y, Z in mm, W, P, R in
    degrees — and :attr:`uframe_number` / :attr:`utool_number` name the frame it is
    expressed in and the tool whose TCP it reports.

    Those two numbers are why this read exists. The Stream Motion status packet
    carries a Cartesian pose with **no frame tag at all**, so RMI is the only
    surface that states which frame a pose belongs to.

    :attr:`front` / :attr:`up` / :attr:`left` / :attr:`flip` / :attr:`turn` are the
    arm's configuration branch — the same pose is reachable in several of them, so
    they matter to anyone converting a pose back into joints.

    Unlike :meth:`RmiClient.read_joint_angles`, this read carries **no
    representation caveat**: the controller runs the FK internally, so the J2/J3
    question (``controller_facts.INTERIM_FACTS.rmi_to_stream_j3_plus_j2_measured``)
    cannot reach the result.
    """

    xyzwpr: tuple[float, ...]  # X, Y, Z (mm), W, P, R (deg), then any ext axes
    utool_number: int
    uframe_number: int
    front: int
    up: int
    left: int
    flip: int
    turn: tuple[int, int, int]
    time_tag: int


@dataclass(frozen=True)
class ControllerError:
    """One ``FRC_ReadError`` row (FANUC RMI §2.3.5).

    The reply may carry up to 5 strings (``ErrorData``..``ErrorData5``); we
    expose them as a tuple so callers can iterate without juggling optional
    fields.  Strings are the controller's literal alarm text (e.g.
    ``"SRVO-002 Teach pendant E-stop"``).
    """

    error_id: int
    count: int  # number of valid entries in ``messages`` (1..5)
    messages: tuple[str, ...]


@dataclass
class _Pending:
    """A single in-flight request awaiting its correlated reply.

    ``identifier`` is the ``Command``/``Communication``/``Instruction`` string
    the controller will echo.  The RX thread delivers ``(reply, None)`` on a
    match or ``(None, exc)`` on a socket death via the one-shot queue.
    """

    identifier: str | None
    inbox: queue.Queue[tuple[dict[str, Any] | None, BaseException | None]]


class RmiClient:
    """JSON-over-TCP client for the FANUC RMI (option R912 / S636).

    Thread-safe. A background RX thread is the sole reader of the redirect
    session socket; ``_request_lock`` serializes request/response cycles.

    Worker-safe methods (auto-reopen the transport without Initialize/Abort;
    raise :class:`RmiSessionDown` on persistent failure):
        :meth:`read_register`, :meth:`write_register`, :meth:`get_status`,
        :meth:`get_extended_status`, :meth:`read_error`,
        :meth:`read_joint_angles`, :meth:`reseed_sequence_id_from_controller`,
        :meth:`reset`, :meth:`program_call`.

    Supervisor-only methods (NOT on the auto-reopen path — a worker must never
    call these from a live-stream state):
        :meth:`initialize`       — runs the Init recovery ladder + seq reseed;
        :meth:`abort`            — aborts all TP programs; guarded to run only
                                   when STREAM_MOTN is known-down;
        :meth:`program_continue` — resumes a paused TP program (recovery ladder).
    """

    # Cap on the RX buffer's pending bytes.  A non-newline-terminating firmware
    # bug or a very long unmatched response should never blow up host memory —
    # well above any realistic single-packet size (~1 KB).
    _MAX_RXBUF_BYTES: int = 64 * 1024

    # Cap on stashed async/orphan packets.  Above this we evict oldest with a
    # warning — bounded so a runaway push-packet stream can't drive memory
    # growth in a long-running driver process.
    _MAX_ASYNC_QUEUE: int = 64

    def __init__(
        self,
        controller_ip: str = "192.168.1.100",
        bootstrap_port: int = 16001,
        *,
        connect_timeout: float = 5.0,
        request_timeout: float = 2.0,
    ) -> None:
        self._controller_ip = controller_ip
        self._bootstrap_port = bootstrap_port

        self._connect_timeout = connect_timeout
        self._request_timeout = request_timeout

        # _request_lock: serializes full request/response cycles (one in-flight
        # request => at most one pending waiter).  _state_lock: guards the
        # shared fields below and the pending waiter; held only briefly.
        # Lock order is ALWAYS _request_lock -> _state_lock (never the reverse).
        self._request_lock = threading.Lock()
        self._state_lock = threading.RLock()

        self._sock: socket.socket | None = None
        # The single in-flight request awaiting its correlated reply.
        self._pending: _Pending | None = None
        # Async / orphan packets parsed off-band by the RX thread.  Push packets
        # (FRC_SystemFault / FRC_Terminate / FRC_AsbnReady), unsolicited FRC_Call
        # acks that lingered after a fire-and-forget write, and any reply that
        # did not match the in-flight request all land here.  Drained by
        # ``poll_async_packets``.  PRESERVED across session rebuilds: the fault
        # that killed the session is the one most worth reporting.
        self._async_packets: deque[dict[str, Any]] = deque(maxlen=self._MAX_ASYNC_QUEUE)
        # Set after the first Connect_STMO; reused on auto-reopen so we don't
        # re-log the redirect port every time.
        self._redirect_port: int | None = None
        # Monotonic counter for instruction packets (FRC_Call & friends).  The
        # controller silently drops a duplicate/stale SequenceID, so after
        # Initialize we re-anchor to the controller's NextSequenceID (which
        # FRC_Initialize does NOT reset on R-30iB+ v9.0).
        self._instruction_seq_id = 1
        # Surfaced by ``transport_retry_count()`` for status-path alarming.
        self._transport_retry_count = 0

        # Background RX thread + shutdown signal.
        self._rx_thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def controller_ip(self) -> str:
        """The controller this session addresses. Read by the preflight gate, which
        reaches the same controller over FTP for its one-time version/option checks."""
        return self._controller_ip

    def start(self) -> None:
        """Establish the commands-only session: Connect_STMO + redirect hop.

        Does NOT Initialize — that is :meth:`initialize`, driven only by the
        lifecycle supervisor.  After ``start()`` the full commands-only
        surface (GetStatus / ReadError / registers / GetExtStatus /
        program_call) is available pre-Initialize.

        Raises :class:`RmiError` on a controller-refused connect (e.g. 2556954
        "Robot is Already Connected." — a prior session not yet freed, the
        reacquire race; see controller-notes.md §1.4) and
        :class:`FanucConnectionError` on a transport-level failure. NOTE: a *genuine*
        concurrent live session does NOT surface as 2556954 on this controller — the
        2nd Connect_STMO returns ErrorID 0 and the redirect-port connect TIMES OUT
        (→ :class:`FanucConnectionError`), per controller-notes.md §1.4.
        """
        self._stop_evt.clear()
        with self._request_lock:
            try:
                self._open_session_locked()
            except OSError as exc:
                raise FanucConnectionError(
                    f"RMI: could not connect to {self._controller_ip}:{self._bootstrap_port}: {exc}"
                ) from exc

    def stop(self) -> None:
        """FRC_Disconnect (best-effort), stop the RX thread, close socket. Idempotent."""
        with self._request_lock:
            if self._sock is not None:
                try:
                    # Best-effort: don't raise if the controller already closed
                    # the connection.
                    self._send_and_wait_locked(wire.rmi_disconnect_request(), raise_on_error_id=False)
                except Exception as exc:  # noqa: BLE001 - shutdown is best-effort
                    logger.debug("RMI: FRC_Disconnect failed (often benign on shutdown): %s", exc)
            self._stop_evt.set()
            self._close_socket_locked()
        # Join the RX thread outside the request lock (it only takes _state_lock).
        self._join_rx_thread()
        logger.info("RMI: disconnected")

    # ------------------------------------------------------------------
    # Explicit initialize — SUPERVISOR-ONLY
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        """Run the FANUC Init recovery ladder + SequenceID reseed. SUPERVISOR-ONLY.

        Ladder (mirrors the FANUC reference driver,
        ``fanuc_libs/fanuc_client/src/fanuc_client.cpp`` lines 443-459):

            pass 1:  GetStatus -> Reset -> GetStatus -> Initialize
            on error (incl. 2556943 "Invalid Controller State"):
            pass 2:  Abort -> Reset -> GetStatus -> Initialize

        GetStatus/Reset/Abort non-zero ErrorIDs are best-effort (ignored); we
        only care that Initialize succeeds.  Immediately after Initialize we
        reseed ``_instruction_seq_id`` from ``FRC_GetStatus.NextSequenceID``.

        This is the ONLY method that issues ``FRC_Initialize`` — transport
        auto-reopen deliberately does not.  It MUST be invoked only by
        the lifecycle supervisor from a state where STREAM_MOTN is known-down
        (TP_LAUNCH / RECOVERING); calling it from STREAMING would restart the
        session under a live trajectory.
        """
        with self._request_lock:
            self._ensure_open_locked()
            self._initialize_with_recovery_locked()
            logger.info("RMI: FRC_Initialize OK — session ready")

    def abort(self) -> None:
        """FRC_Abort; controller aborts all running TP programs. SUPERVISOR-ONLY.

        Equivalent to the operator pressing ``FCTN → ABORT ALL`` on the teach
        pendant.  Kills any running ``STREAM_MOTN`` (smooth controller-side
        decel, not a hard E-stop) and any running ``GRIPDISP``, and flushes the
        controller-side instruction queue.

        CONTRACT: this is **supervisor-only** and is deliberately NOT on
        the transport auto-reopen path.  A worker whose socket blipped must
        never abort a live trajectory — auto-reopen re-establishes the socket
        *without* Abort/Initialize.  The supervisor may call this only in a
        state where STREAM_MOTN is known-down.

        Best-effort: a non-zero ErrorID here is not fatal; callers may catch
        and continue.
        """
        self._exchange_with_retry(wire.rmi_command_request("FRC_Abort"))

    def program_continue(self) -> None:
        """FRC_Continue; resume a paused Remote-Motion TP program. SUPERVISOR-ONLY.

        The recovery ladder's resume step: after a SYST-348 payload-confirm
        clears the pause, ``FRC_Continue`` resumes the RMI-launched program. ErrorID
        ``2556938`` ("TP Program is Not Paused.") is TOLERATED as a no-op
        success — the program is already running (or was never paused), which is
        the desired post-recovery end state; treating it as fatal would wedge
        the ladder.

        SUPERVISOR-ONLY, like :meth:`abort`: a worker must never issue it from a
        live-stream state. Any other non-zero ErrorID raises :class:`RmiError`;
        a persistent transport failure raises :class:`RmiSessionDown`.
        """
        try:
            self._exchange_with_retry(wire.rmi_continue_request())
        except RmiError as exc:
            if exc.error_id == _ERR_TP_NOT_PAUSED:
                logger.info(
                    "RMI: FRC_Continue — TP program not paused (%d); treating as no-op",
                    _ERR_TP_NOT_PAUSED,
                )
                return
            raise

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def poll_async_packets(self) -> list[dict[str, Any]]:
        """Drain any async / orphan packets the RX thread has stashed.

        Returns and removes everything currently buffered.  The supervisor
        calls this from its ~1 s status loop so SystemFault / Terminate
        notifications are surfaced as warnings instead of silently piling up.
        The ring is preserved across session rebuilds (only drained here).
        """
        with self._state_lock:
            if not self._async_packets:
                return []
            drained = list(self._async_packets)
            self._async_packets.clear()
            return drained

    def transport_retry_count(self) -> int:
        """Cumulative retries triggered by transient socket errors."""
        with self._state_lock:
            return self._transport_retry_count

    # ------------------------------------------------------------------
    # Numeric registers (the gripper uses R[1], R[2], R[3])
    # ------------------------------------------------------------------

    def read_register(self, register_number: int) -> float:
        """Send FRC_ReadRegister and return the numeric value as a ``float``.

        The wire packet's ``RegisterValue`` is a ``std::variant<int, float>``; both
        are coerced to ``float`` here. Raises :class:`RmiError` if the controller
        returns a non-zero ErrorID, :class:`RmiSessionDown` if the session is
        persistently dead.
        """
        resp = self._exchange_with_retry(wire.rmi_read_register_request(register_number))
        value = resp.get("RegisterValue")
        if value is None:
            raise RmiError(
                f"FRC_ReadRegister missing RegisterValue, response={resp}",
                response=resp,
            )
        # Both int and float are allowed by the packet (std::variant<int, float>).
        return float(value)

    def write_register(self, register_number: int, value: float | int) -> None:
        """Send FRC_WriteRegister with DataType inferred from ``value`` type.

        int/bool → lowercase ``"integer"``, float → lowercase ``"float"``
        (matching FANUC reference ``rmi.cpp:462``).  Capitalised
        "Integer"/"Float" is silently coerced to Integer by the controller.
        Type inference + the lowercase invariant live in
        :func:`airo_fanuc.testing.wire.rmi_write_register_request` so this
        client inherits the pinned goldens byte-for-byte.

        Raises :class:`TypeError` for a non-int/float value, :class:`RmiError`
        on non-zero ErrorID, :class:`RmiSessionDown` on persistent session death.
        """
        # The builder does the bool/int/float inference and TypeError on bad type.
        self._exchange_with_retry(wire.rmi_write_register_request(register_number, value))

    # ------------------------------------------------------------------
    # Extended status — used to detect drives_powered changes and surface the
    # FANUC alarm code on motion_possible drops.
    # ------------------------------------------------------------------

    def get_extended_status(self, *, quiet: bool = False) -> ExtendedStatus:
        """Send ``FRC_GetExtStatus`` (RMI §2.3.4) and decode the reply.

        ``quiet=True`` suppresses the per-call DEBUG send/recv log — used by the
        1 Hz ext-status poll so the two log lines per call don't bury the journal.
        """
        resp = self._exchange_with_retry(wire.rmi_command_request("FRC_GetExtStatus"), quiet=quiet)
        return ExtendedStatus(
            in_motion=bool(resp.get("InMotion", 0)),
            drives_powered=bool(resp.get("DrivesPowered", 0)),
            gen_override_pct=int(resp.get("GenOverride", 0)),
            speed_clamp_limit_pct=(
                float(resp["SpeedClampLimit"]) if resp.get("SpeedClampLimit") is not None else None
            ),
            error_code=resp.get("ErrorCode"),
            control_mode=resp.get("ControlMode"),
        )

    def get_status(self) -> ControllerStatus:
        """Send ``FRC_GetStatus`` (RMI §2.3.7), parse all reply fields.

        Distinct from :meth:`get_extended_status` (``FRC_GetExtStatus``):
        ``GetStatus`` returns the scheduler-relevant state — the authoritative
        ``NextSequenceID`` for instruction packets, TP mode, program status,
        override percentage.  Works on the commands-only session (pre-Initialize)
        so :class:`FanucReceiveInterface` can poll it without ever initializing.
        """
        resp = self._exchange_with_retry(wire.rmi_command_request("FRC_GetStatus"))
        return ControllerStatus(
            servo_ready=bool(resp.get("ServoReady", 0)),
            tp_mode=int(resp.get("TPMode", 0)),
            rmi_motion_status=int(resp.get("RMIMotionStatus", 0)),
            program_status=int(resp.get("ProgramStatus", 0)),
            single_step_mode=bool(resp.get("SingleStepMode", 0)),
            number_utool=int(resp.get("NumberUTool", 0)),
            next_sequence_id=int(resp.get("NextSequenceID", 1)),
            number_uframe=int(resp.get("NumberUFrame", 0)),
            override_pct=int(resp.get("Override", 0)),
        )

    def read_error(self, count: int = 1) -> ControllerError:
        """Send ``FRC_ReadError`` (RMI §2.3.5) and return the alarm strings.

        ``count`` is the controller's "how many recent errors do you want" knob
        (valid range 1..5, validated by the wire builder).  Use this on
        motion_possible→False events to capture the actual FANUC alarm text.

        Raises :class:`ValueError` for ``count`` out of range; :class:`RmiError`
        on non-zero ErrorID; :class:`RmiSessionDown` on persistent session death.
        """
        # The builder validates 1 <= count <= 5 (ValueError otherwise).
        resp = self._exchange_with_retry(wire.rmi_read_error_request(count))
        msgs: list[str] = []
        for key in ("ErrorData", "ErrorData2", "ErrorData3", "ErrorData4", "ErrorData5"):
            v = resp.get(key)
            if isinstance(v, str) and v:
                msgs.append(v)
        return ControllerError(
            error_id=int(resp.get("ErrorID", 0)),
            count=int(resp.get("Count", len(msgs))),
            messages=tuple(msgs),
        )

    def read_joint_angles(self) -> list[float]:
        """Send ``FRC_ReadJointAngles`` (RMI §2.3.15), return joint angles in degrees.

        Worker-safe: runs on the commands-only session and auto-reopens the
        transport WITHOUT Initialize/Abort. Returns the joints present in the
        reply's ``JointAngle`` block in ascending order (J1..J9); a 6-DOF CRX
        still reports nine axes with the trailing three zero, so callers get the
        contiguous run starting at J1 (six or nine values).

        **UNCONVERTED joints.** RMI reports J3 one J2 below the Stream Motion
        value (``controller_facts.INTERIM_FACTS.rmi_to_stream_j3_plus_j2_measured``),
        so these are NOT interchangeable with Stream Motion joints: reaching that
        frame takes ``J3 += J2``. A caller feeding them to calibration must tag them
        :data:`~airo_fanuc.receive_interface.SOURCE_RMI_UNCONVERTED`
        (:class:`~airo_fanuc.receive_interface.RmiClientJointReader` does), which
        the calibration path hard-rejects until that conversion is verified at a
        second J2.

        Raises :class:`RmiError` on a non-zero ErrorID or a malformed reply
        (missing / empty ``JointAngle``), :class:`RmiSessionDown` on persistent
        session death.
        """
        resp = self._exchange_with_retry(wire.rmi_read_joint_angles_request())
        joint_angle = resp.get("JointAngle")
        if not isinstance(joint_angle, dict):
            raise RmiError(
                f"FRC_ReadJointAngles reply missing JointAngle block, response={resp}",
                response=resp,
            )
        joints: list[float] = []
        for i in range(1, 10):
            value = joint_angle.get(f"J{i}")
            if value is None:
                break  # first gap ends the contiguous J1..Jn run
            joints.append(float(value))
        if not joints:
            raise RmiError(
                f"FRC_ReadJointAngles JointAngle carried no J1..J9 fields, response={resp}",
                response=resp,
            )
        return joints

    def read_cartesian_position(self) -> CartesianPosition:
        """Send ``FRC_ReadCartesianPosition`` (RMI §2.3.14), return the decoded pose.

        Worker-safe, like :meth:`read_joint_angles`: runs on the commands-only
        session and auto-reopens the transport WITHOUT Initialize/Abort.

        The returned pose is the controller's own FK, tagged with the active
        UFRAME/UTOOL numbers — see :class:`CartesianPosition`. This is a
        request/reply read at RMI cadence (tens of ms, not deterministic), so it is
        a **verification and setup instrument, not a telemetry source**: for a pose
        on the 125 Hz timeline, sampled on the same packet as the joints, use
        :meth:`airo_fanuc.driver.FanucDriver.get_tcp_pose`.

        Raises :class:`RmiError` on a non-zero ErrorID or a malformed reply (missing
        / empty ``Position``), :class:`RmiSessionDown` on persistent session death.
        """
        resp = self._exchange_with_retry(wire.rmi_read_cartesian_position_request())
        position = resp.get("Position")
        if not isinstance(position, dict):
            raise RmiError(
                f"FRC_ReadCartesianPosition reply missing Position block, response={resp}",
                response=resp,
            )
        pose: list[float] = []
        for key in ("X", "Y", "Z", "W", "P", "R", "Ext1", "Ext2", "Ext3"):
            value = position.get(key)
            if value is None:
                break  # first gap ends the contiguous run
            pose.append(float(value))
        if len(pose) < 6:
            raise RmiError(
                f"FRC_ReadCartesianPosition Position carried {len(pose)} of the 6 required "
                f"X/Y/Z/W/P/R fields, response={resp}",
                response=resp,
            )
        config = resp.get("Configuration")
        if not isinstance(config, dict):
            config = {}
        return CartesianPosition(
            xyzwpr=tuple(pose),
            utool_number=int(config.get("UToolNumber", 0)),
            uframe_number=int(config.get("UFrameNumber", 0)),
            front=int(config.get("Front", 0)),
            up=int(config.get("Up", 0)),
            left=int(config.get("Left", 0)),
            flip=int(config.get("Flip", 0)),
            turn=(
                int(config.get("Turn4", 0)),
                int(config.get("Turn5", 0)),
                int(config.get("Turn6", 0)),
            ),
            time_tag=int(resp.get("TimeTag", 0)),
        )

    def reseed_sequence_id_from_controller(self) -> int:
        """Re-anchor ``_instruction_seq_id`` to the controller's ``NextSequenceID``.

        Calls :meth:`get_status` and sets ``_instruction_seq_id`` to the
        controller's ``NextSequenceID``.  Returns the new value.

        After a session recovery the controller may have a different
        "next expected sequence ID" than our local counter; without this
        resync the next ``FRC_Call`` is silently dropped as a duplicate.
        (Lock-free variant of the in-handshake :meth:`_reseed_seq_id_from_controller_locked`.)
        """
        status = self.get_status()
        with self._state_lock:
            self._instruction_seq_id = int(status.next_sequence_id)
        return int(status.next_sequence_id)

    # ------------------------------------------------------------------
    # Fault clear
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """FRC_Reset; controller clears recoverable faults.

        Worker-safe: unlike :meth:`abort` / :meth:`initialize`, FRC_Reset does
        not tear down or restart the RMI/Stream-Motion session, so it is part of
        the recovery ladders driven by both the supervisor and the bring-up TP
        ladder.
        """
        self._exchange_with_retry(wire.rmi_command_request("FRC_Reset"))

    def program_call(self, program_name: str) -> None:
        """Fire-and-forget ``FRC_Call`` instruction (RMI §2.4.6).

        Used at startup to launch ``STREAM_MOTN`` (which flips the Stream Motion
        ``status & 0x1`` motion_possible bit) and by the gripper to fork
        ``GRPRUN``.  We deliberately do NOT read the synchronous ``FRC_Call``
        ack:

        - The first response can carry ErrorID 2556956 ("Wait for Instruction
          Done"), a non-fatal "accepted, in progress" status; treating it as
          fatal blocks bring-up after any prior crash left a stale instruction
          marker.
        - The FANUC America reference driver uses ``programCallNonBlocking``
          (fire-and-forget) for ``STREAM_MOTN`` on this path.
        - Stream Motion's ``motion_possible`` bit is the authoritative "did
          STREAM_MOTN start" signal; the bring-up loop waits on it directly.

        The orphan ack arrives later and is stashed by the RX thread as an
        out-of-band orphan (no protocol corruption).

        Auto-reopens the transport WITHOUT Initialize/Abort on transport error
        and retries once; raises :class:`RmiSessionDown` on persistent failure.
        """
        with self._request_lock:
            for attempt in (1, 2):
                try:
                    self._ensure_open_locked()
                    with self._state_lock:
                        assert self._sock is not None
                        sock = self._sock
                        seq_id = self._instruction_seq_id
                        self._instruction_seq_id += 1
                    req = wire.rmi_call_request(seq_id, program_name)
                    logger.debug("RMI <- FRC_Call(%s, seq=%s) [fire-and-forget]", program_name, seq_id)
                    sock.sendall(wire.rmi_wire_bytes(req))
                    return
                except OSError as exc:
                    self._bump_retry_locked()
                    logger.warning(
                        "RMI: program_call(%s) transport error on attempt %d/2: %s",
                        program_name,
                        attempt,
                        exc,
                    )
                    self._close_socket_locked()
                    if attempt == 2:
                        raise RmiSessionDown(
                            f"RMI: program_call({program_name}) failed after 2 attempts: {exc}"
                        ) from exc

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> RmiClient:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Request/response core (caller of _exchange_with_retry never holds locks)
    # ------------------------------------------------------------------

    def _exchange_with_retry(self, req: dict[str, Any], *, quiet: bool = False) -> dict[str, Any]:
        """Send a request and return its correlated reply, retrying once on socket error.

        Transport auto-reopen re-establishes the socket via
        ``Connect_STMO`` + the redirect hop WITHOUT issuing Initialize/Abort.
        A non-zero ErrorID raises :class:`RmiError` (a controller-level problem,
        not transient — not retried, not converted).  A persistent transport
        failure raises :class:`RmiSessionDown` (workers do not implicitly
        reopen — that is the supervisor's concern).  The single-flight reopen
        (serialized by ``_request_lock``) also prevents concurrent-reopen
        2556954 self-infliction.
        """
        with self._request_lock:
            for attempt in (1, 2):
                try:
                    self._ensure_open_locked()
                    return self._send_and_wait_locked(req, quiet=quiet)
                except RmiError:
                    # Controller-level error: a real protocol problem (incl. a
                    # single-session 2556954 on reopen).  Do NOT retry — retrying
                    # a controller-level refusal is the self-reopen storm the
                    # session split exists to prevent.
                    raise
                except OSError as exc:
                    self._bump_retry_locked()
                    logger.warning(
                        "RMI: transport error on attempt %d/2 (%s): %s",
                        attempt,
                        self._request_reply_identifier(req),
                        exc,
                    )
                    self._close_socket_locked()
                    if attempt == 2:
                        raise RmiSessionDown(
                            f"RMI: session down after 2 attempts "
                            f"({self._request_reply_identifier(req)}): {exc}"
                        ) from exc
            # Unreachable — the loop either returns or raises.
            raise RmiSessionDown("unreachable")

    def _send_and_wait_locked(
        self, req: dict[str, Any], *, raise_on_error_id: bool = True, quiet: bool = False
    ) -> dict[str, Any]:
        """Send one JSON line and block for the RX thread to deliver its reply.

        Caller holds ``_request_lock`` (guaranteeing a single in-flight waiter)
        and ``self._sock`` is expected open.  Correlation-by-echo and the
        push/orphan split are performed by the RX thread; this only registers
        the waiter, sends, and validates the delivered reply's ErrorID.

        ``raise_on_error_id=False`` returns the reply without raising on a
        non-zero ErrorID — used by the recovery-ladder prep commands and the
        best-effort FRC_Disconnect.

        Raises :class:`OSError` if the socket is not open or the reply never
        lands within the bounded deadline (a missing reply must surface as a
        transport failure, not an indefinite block on the request lock);
        :class:`RmiError` on a non-zero ErrorID when ``raise_on_error_id``.
        """
        identifier = self._request_reply_identifier(req)
        inbox: queue.Queue[tuple[dict[str, Any] | None, BaseException | None]] = queue.Queue(maxsize=1)
        pending = _Pending(identifier=identifier, inbox=inbox)
        with self._state_lock:
            if self._sock is None:
                raise OSError("RMI: socket not open")
            sock = self._sock
            self._pending = pending
        if not quiet:
            logger.debug("RMI <- %s", req)
        try:
            sock.sendall(wire.rmi_wire_bytes(req))
        except OSError:
            with self._state_lock:
                if self._pending is pending:
                    self._pending = None
            raise

        deadline_s = max(self._request_timeout * 4, 4.0)
        try:
            reply, exc = inbox.get(timeout=deadline_s)
        except queue.Empty:
            with self._state_lock:
                if self._pending is pending:
                    self._pending = None
                n_async = len(self._async_packets)
            raise OSError(
                f"RMI: no response matching {identifier!r} within {deadline_s:.1f}s; async_drained={n_async}"
            ) from None
        with self._state_lock:
            if self._pending is pending:
                self._pending = None
        if exc is not None:
            raise exc
        assert reply is not None
        if not quiet:
            logger.debug("RMI -> %s", reply)
        error_id = reply.get("ErrorID")
        if error_id is not None and int(error_id) != 0 and raise_on_error_id:
            cmd = self._request_reply_identifier(req)
            eid = int(error_id)
            raise RmiError(
                f"{cmd} failed: ErrorID={eid} ({describe_rmi_error_id(eid)}), response={reply}",
                error_id=eid,
                text=describe_rmi_error_id(eid),
                response=reply,
            )
        return reply

    # ------------------------------------------------------------------
    # Session open (commands-only) + Initialize ladder
    # ------------------------------------------------------------------

    def _ensure_open_locked(self) -> None:
        """Ensure the commands-only session socket is open. Caller holds ``_request_lock``.

        No-op if already connected.  Otherwise runs the bootstrap+redirect
        handshake (NO Initialize/Abort — that is :meth:`initialize`).
        """
        with self._state_lock:
            if self._sock is not None:
                return
        self._open_session_locked()

    def _open_session_locked(self) -> None:
        """Run the ``FRC_Connect_STMO`` → redirect-hop handshake. Caller holds ``_request_lock``.

        Establishes the *commands-only* session — deliberately NO
        ``FRC_Initialize``.  On success installs ``self._sock`` (the
        redirect socket) and starts the RX thread; on any failure leaves
        ``self._sock = None`` so the next attempt re-bootstraps cleanly.

        The Connect_STMO reply is read *synchronously* on a throwaway bootstrap
        socket — the ONE read outside the RX thread — while ``self._sock`` is
        ``None`` (RX thread idle-spinning), so there is no contention.
        """
        # Step 1: bootstrap connect + Connect_STMO (synchronous read).
        boot = self._tcp_connect(self._controller_ip, self._bootstrap_port)
        try:
            resp = self._bootstrap_exchange(boot, wire.rmi_connect_stmo_request())
        finally:
            self._safe_close(boot)

        error_id = resp.get("ErrorID")
        if error_id is not None and int(error_id) != 0:
            eid = int(error_id)
            # e.g. 2556954 "Robot is Already Connected." — a prior session not yet
            # freed (the reacquire race, controller-notes.md §1.4; a genuine
            # concurrent session times out on the redirect port instead). This is a
            # controller-level RmiError — the caller must NOT retry-reopen (that
            # would be a self-reopen storm against a controller already refusing).
            raise RmiError(
                f"FRC_Connect_STMO failed: ErrorID={eid} ({describe_rmi_error_id(eid)}), response={resp}",
                error_id=eid,
                text=describe_rmi_error_id(eid),
                response=resp,
            )
        redirect = resp.get("PortNumber")
        if not isinstance(redirect, int) or redirect <= 0:
            raise RmiError(f"FRC_Connect_STMO returned no PortNumber, response={resp}", response=resp)
        if self._redirect_port is not None and self._redirect_port != int(redirect):
            logger.info("RMI: redirect port changed %s -> %s on reconnect", self._redirect_port, redirect)
        self._redirect_port = int(redirect)
        major = resp.get("MajorVersion")
        minor = resp.get("MinorVersion")
        logger.info("RMI: controller version %s.%s, redirect to port %s", major, minor, self._redirect_port)

        # Step 2: reconnect to the redirect port and hand the socket to the RX thread.
        sock = self._tcp_connect(self._controller_ip, self._redirect_port)
        with self._state_lock:
            self._sock = sock
            # NOTE: _async_packets is intentionally NOT cleared here — the
            # push/orphan ring is preserved across session rebuilds, since the
            # fault that forced the rebuild is the one worth reporting.
        self._ensure_rx_thread()
        logger.info("RMI: commands-only session up on %s:%s", self._controller_ip, self._redirect_port)

    def _initialize_with_recovery_locked(self) -> None:
        """FANUC-reference Initialize with state-recovery fallback. Caller holds ``_request_lock``.

        Only called from :meth:`initialize` (supervisor-only).  The Abort in
        pass 2 is therefore never issued from a live-stream state.
        """
        prep_pass1 = ("FRC_GetStatus", "FRC_Reset", "FRC_GetStatus")
        prep_pass2 = ("FRC_Abort", "FRC_Reset", "FRC_GetStatus")

        # Pass 1: gentle clear (handles a "stuck Initialize" from a clean exit).
        for cmd in prep_pass1:
            self._send_prep_command_locked(cmd)
        try:
            self._send_and_wait_locked(wire.rmi_command_request("FRC_Initialize"))
            self._reseed_seq_id_from_controller_locked()
            return
        except (RmiError, OSError) as exc:
            # OSError covers a wedged redirect-port handler silently dropping
            # Initialize — pass 2 (Abort + Reset) is the last lever before a
            # controller power-cycle.
            logger.warning(
                "RMI: FRC_Initialize after GetStatus/Reset failed (%s); escalating to Abort + Reset",
                exc,
            )

        # Pass 2: harder clear (stale STREAM_MOTN call or a latched fault).
        for cmd in prep_pass2:
            self._send_prep_command_locked(cmd)
        self._send_and_wait_locked(wire.rmi_command_request("FRC_Initialize"))
        self._reseed_seq_id_from_controller_locked()

    def _reseed_seq_id_from_controller_locked(self) -> None:
        """Re-anchor ``_instruction_seq_id`` from ``NextSequenceID``. Caller holds ``_request_lock``.

        In-handshake variant of :meth:`reseed_sequence_id_from_controller` (which
        acquires ``_request_lock`` itself and so must not run inside the ladder).

        ``FRC_Initialize`` does NOT reset the controller's ``NextSequenceID`` on
        R-30iB+ v9.0 — it persists across sessions until a power-cycle (or a
        successful ``FRC_Disconnect``).  Without re-anchoring, every driver
        restart after the first silently drops ``FRC_Call(STREAM_MOTN, seq=1)``.
        """
        resp = self._send_and_wait_locked(wire.rmi_command_request("FRC_GetStatus"))
        next_seq = int(resp.get("NextSequenceID", 1))
        if next_seq < 1:
            next_seq = 1
        with self._state_lock:
            current = self._instruction_seq_id
            self._instruction_seq_id = next_seq
        if next_seq != current:
            logger.info(
                "RMI: re-anchoring SequenceID %s -> %s from controller "
                "(FRC_Initialize did not reset NextSequenceID)",
                current,
                next_seq,
            )

    def _send_prep_command_locked(self, command: str) -> None:
        """Send one ``{"Command": <cmd>}`` and swallow non-zero ErrorIDs / transport errors."""
        try:
            resp = self._send_and_wait_locked(wire.rmi_command_request(command), raise_on_error_id=False)
            err = resp.get("ErrorID")
            if err is not None and int(err) != 0:
                logger.debug("RMI: prep %s returned ErrorID=%s (ignored)", command, err)
        except OSError as exc:
            # The controller can drop the connection between Connect and
            # Initialize on some firmwares; surface as a soft warning so we
            # still attempt Initialize on whatever socket we have.
            logger.warning("RMI: prep %s transport error: %s", command, exc)

    # ------------------------------------------------------------------
    # Background RX thread — sole reader of the redirect session socket
    # ------------------------------------------------------------------

    def _ensure_rx_thread(self) -> None:
        """Start the RX reader thread if not already running (idempotent)."""
        with self._state_lock:
            if self._rx_thread is not None and self._rx_thread.is_alive():
                return
            self._stop_evt.clear()
            t = threading.Thread(target=self._rx_loop, name="airo-fanuc-rmi-rx", daemon=True)
            self._rx_thread = t
        t.start()

    def _join_rx_thread(self) -> None:
        with self._state_lock:
            t = self._rx_thread
            self._rx_thread = None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=2.0)

    def _rx_loop(self) -> None:
        """Read/buffer/dispatch every packet off the redirect session socket.

        The RX buffer is a thread-LOCAL bytearray keyed to the current socket
        identity: when ``self._sock`` changes (reopened or dropped) the buffer
        is reset, so bytes from a dead socket can never bleed into a fresh one.
        """
        cur_sock: socket.socket | None = None
        rxbuf = bytearray()
        while not self._stop_evt.is_set():
            with self._state_lock:
                sock = self._sock
            if sock is not cur_sock:
                cur_sock = sock
                rxbuf = bytearray()
            if sock is None:
                self._stop_evt.wait(0.02)
                continue
            try:
                pkt = self._read_one_json(sock, rxbuf)
            except TimeoutError:
                continue  # recv idle timeout — normal, keep reading
            except OSError as exc:
                self._on_socket_error(sock, exc)
                continue
            except ValueError as exc:
                # Malformed JSON (json.JSONDecodeError) or a non-ASCII byte
                # (UnicodeDecodeError) — both ValueError subclasses, neither an
                # OSError. _read_one_json consumed the offending line from rxbuf
                # (``del rxbuf[:nl+1]``) BEFORE decoding, so reading resumes cleanly
                # at the next line with no desync. Log + skip: a raised ValueError
                # would kill this SOLE RX reader thread and silently disable async
                # fault-push surfacing (FRC_SystemFault / FRC_Terminate would never
                # reach poll_async_packets until an unrelated later request timed out).
                logger.warning("RMI: skipping malformed packet line from controller: %s", exc)
                continue
            try:
                self._dispatch(pkt)
            except Exception:  # noqa: BLE001 - the sole RX reader must never die on a dispatch bug
                logger.exception("RMI: RX dispatch raised (continuing; reader kept alive)")

    def _dispatch(self, pkt: dict[str, Any]) -> None:
        """Route one parsed packet: push → async ring, matched reply → waiter, else orphan."""
        comm = pkt.get("Communication")
        cmd = pkt.get("Command")
        if comm in _PUSH_PACKET_COMMS or cmd == "Unknown":
            self._stash_async_packet(pkt)
            return
        with self._state_lock:
            pending = self._pending
        if pending is not None and self._packet_matches_request(pkt, pending.identifier):
            pending.inbox.put((pkt, None))
            return
        # Orphan (e.g. a stale fire-and-forget FRC_Call ack).  Stash for
        # diagnostics rather than mis-returning it as someone's reply.
        logger.debug(
            "RMI: stashing orphan packet (pending=%s): %s",
            None if pending is None else pending.identifier,
            pkt,
        )
        self._stash_async_packet(pkt)

    def _on_socket_error(self, sock: socket.socket, exc: OSError) -> None:
        """Handle a dead socket: drop it (if still current) and fail any pending waiter."""
        with self._state_lock:
            waiter = self._pending
            self._pending = None
            if self._sock is sock:
                self._safe_close(sock)
                self._sock = None
        if waiter is not None:
            waiter.inbox.put((None, exc))

    def _stash_async_packet(self, pkt: dict[str, Any]) -> None:
        """Push ``pkt`` onto the async/orphan ring with a structured log."""
        ident = pkt.get("Communication") or pkt.get("Command") or pkt.get("Instruction")
        if ident in _PUSH_PACKET_COMMS:
            logger.warning("RMI: async push packet received: %s", pkt)
        with self._state_lock:
            if (
                len(self._async_packets) == self._async_packets.maxlen
                and self._async_packets.maxlen is not None
            ):
                logger.warning(
                    "RMI: async-queue full (%d entries); evicting oldest: %s",
                    self._async_packets.maxlen,
                    self._async_packets[0],
                )
            self._async_packets.append(pkt)

    def _read_one_json(self, sock: socket.socket, rxbuf: bytearray) -> dict[str, Any]:
        """Read one ``\\n``-terminated JSON object from ``sock`` into ``rxbuf``.

        Treats ``\\r\\n`` and ``\\n`` as equivalent.  ``rxbuf`` is owned by the
        RX thread (persists across calls so a partial line survives a recv
        timeout).  Bounded growth: a runaway non-newline-terminated stream
        raises :class:`OSError` rather than exhausting host memory.
        """
        while True:
            nl = rxbuf.find(b"\n")
            if nl >= 0:
                raw = bytes(rxbuf[:nl])
                del rxbuf[: nl + 1]
                if raw.endswith(b"\r"):
                    raw = raw[:-1]
                return json.loads(raw.decode("ascii"))  # type: ignore[no-any-return]
            if len(rxbuf) > self._MAX_RXBUF_BYTES:
                raise OSError(
                    f"RMI: receive buffer exceeded {self._MAX_RXBUF_BYTES} B "
                    "without a newline (controller streaming garbage?)"
                )
            chunk = sock.recv(4096)
            if not chunk:
                raise OSError("RMI: controller closed the connection")
            rxbuf.extend(chunk)

    # ------------------------------------------------------------------
    # Correlation helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _request_reply_identifier(req: dict[str, Any]) -> str | None:
        """The string the controller will echo back on the response."""
        for key in _REQ_REPLY_KEYS:
            value = req.get(key)
            if isinstance(value, str):
                return value
        return None

    @staticmethod
    def _packet_matches_request(pkt: dict[str, Any], expected_value: str | None) -> bool:
        """True iff ``pkt`` is the response to a request with the given identifier.

        Falls back to "any non-push packet" when ``expected_value`` is None
        (defensive against a malformed request with no identifier key).
        """
        if expected_value is None:
            return True
        for key in _REQ_REPLY_KEYS:
            value = pkt.get(key)
            if isinstance(value, str) and value == expected_value:
                return True
        return False

    # ------------------------------------------------------------------
    # Socket helpers
    # ------------------------------------------------------------------

    def _bootstrap_exchange(self, sock: socket.socket, req: dict[str, Any]) -> dict[str, Any]:
        """Synchronously send ``req`` and read one JSON reply on the bootstrap socket.

        Used ONLY for the Connect_STMO handshake, before the RX thread owns any
        socket.  A local buffer keeps this fully self-contained.
        """
        sock.sendall(wire.rmi_wire_bytes(req))
        buf = bytearray()
        return self._read_one_json(sock, buf)

    def _bump_retry_locked(self) -> None:
        with self._state_lock:
            self._transport_retry_count += 1

    def _tcp_connect(self, ip: str, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # O_CLOEXEC on every socket fd (Python defaults fds non-inheritable
        # per PEP 446; set it explicitly so a fork in the robot process can never
        # leak the RMI connection into a child).
        sock.set_inheritable(False)
        sock.settimeout(self._connect_timeout)
        try:
            sock.connect((ip, port))
        except Exception:
            self._safe_close(sock)
            raise
        sock.settimeout(self._request_timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # TCP keepalive — detects half-open connections (controller power-cycle,
        # switch outage) without waiting for the next request to discover the
        # dead socket.  The three TCP_KEEPxxx tunings are Linux-specific and
        # silently fall through on other OSes.
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            for opt_name, opt_value in (("TCP_KEEPIDLE", 10), ("TCP_KEEPINTVL", 5), ("TCP_KEEPCNT", 3)):
                opt = getattr(socket, opt_name, None)
                if opt is not None:
                    sock.setsockopt(socket.IPPROTO_TCP, opt, opt_value)
        except OSError:
            # Setting keepalive must never block bring-up.
            pass
        return sock

    def _close_socket_locked(self) -> None:
        """Drop ``self._sock`` (the RX thread resets its buffer on the identity change)."""
        with self._state_lock:
            if self._sock is not None:
                self._safe_close(self._sock)
                self._sock = None

    @staticmethod
    def _safe_close(sock: socket.socket) -> None:
        # shutdown() first so a blocked recv on the RX thread wakes immediately
        # (a bare close() does NOT reliably interrupt a peer thread's recv on
        # Linux — it would otherwise hang until the socket's own timeout).
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except Exception:  # noqa: BLE001 - close is best-effort
            pass


# ---------------------------------------------------------------------------
# Smoke test — hardware-in-the-loop against a real controller.
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description="FANUC RMI hardware-in-the-loop smoke test.")
    parser.add_argument("--ip", default="192.168.1.100", help="Controller IP")
    parser.add_argument(
        "--read-register",
        type=int,
        default=1,
        metavar="N",
        help="Numeric register R[N] to read (default: 1).",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    with RmiClient(args.ip) as rmi:
        value = rmi.read_register(args.read_register)
        print(f"R[{args.read_register}] = {value}")
        status = rmi.get_extended_status()
        print(f"Extended status: {status}")


if __name__ == "__main__":
    main()
