# SPDX-License-Identifier: Apache-2.0
"""RMI JSON-over-TCP server for the FakeCRXController.

Emulates the controller's Remote Motion Interface as
:mod:`airo_fanuc.rmi_client` drives it:

* **Bootstrap → redirect port-hop**: ``FRC_Connect_STMO`` on port 16001 replies
  with a ``PortNumber`` redirect; the client reconnects there for everything
  else.
* **FRC_* vocabulary**: Initialize / Abort / Reset / GetStatus / GetExtStatus /
  ReadError / ReadRegister / WriteRegister / Call (Instruction) / Continue /
  Disconnect. Replies echo the request's identifier key and carry ``ErrorID``.
* **NextSequenceID bookkeeping**: ``FRC_GetStatus`` reports the authoritative
  ``NextSequenceID``; an ``FRC_Call`` with a stale/duplicate SequenceID is
  silently dropped (no ack, no launch) — modeling why the driver must reseed
  from ``FRC_GetStatus.NextSequenceID`` after every Initialize/Reset.
* **Single-session** (``INTERIM_FACTS.rmi_single_session``): a second concurrent
  ``FRC_Connect_STMO`` while a session is live returns error ``2556954``.
* **Error injection**: arm any command to return a specific ErrorID (2556938 TP
  not paused, 2556943 stale session, 2556954 multi-session, 2556934/2556936 →
  SYST-348) — deterministic and scriptable, so the recovery ladders can be
  exercised without provoking a real controller fault.
* **Async push** (``FRC_SystemFault`` / ``FRC_Terminate`` / ``FRC_AsbnReady``):
  out-of-band packets the client's async ring drains.
* **GRIPDISP register contract**: ``FRC_Call("GRPRUN")`` starts the dispatcher;
  with the dispatcher running, writing ``R[1]=1`` executes the ``R[2]``/``R[3]``
  action and clears ``R[1]`` back to 0. A direct ``FRC_Call("GRIPDISP")`` wedges
  the RMI instruction queue instead of forking the dispatcher, so ``R[1]`` never
  clears — the fake reproduces that dead end to prove the driver never calls it
  directly.

Reply shapes carry the field names the client parses (ServoReady /
NextSequenceID / DrivesPowered / GenOverride / ErrorData…); the exact *request*
JSON is pinned by the goldens in ``tests/goldens/rmi/``.
"""

from __future__ import annotations

import json
import socket
import threading
from collections import defaultdict, deque
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover - typing only
    from airo_fanuc.testing.fake_crx import ControllerState, FakeCRXConfig

# RMI error IDs used by the injection hooks (subset of the client's error table).
ERR_CANNOT_EXECUTE_TP_A = 2556934  # "Cannot Execute TP program." (SYST-348 family)
ERR_CANNOT_EXECUTE_TP_B = 2556936  # "Cannot Execute TP program." (SYST-348 family)
ERR_TP_NOT_PAUSED = 2556938  # "TP Program is Not Paused." (FRC_Continue of unpaused)
ERR_INVALID_CONTROLLER_STATE = 2556943  # "Invalid Controller State." (stale session)
ERR_ALREADY_CONNECTED = 2556954  # "Robot is Already Connected." (multi-session)

# Async push packet identifiers the client's async ring drains.
PUSH_COMMS = ("FRC_SystemFault", "FRC_Terminate", "FRC_AsbnReady")
_ECHO_KEYS = ("Command", "Communication", "Instruction")

# GRIPDISP register map (gripper.py): R[1]=trigger, R[2]=action, R[3]=modifier.
REG_CMD = 1
REG_ACTION = 2
REG_R3 = 3


class FakeRmiServer:
    """RMI emulator with bootstrap+redirect TCP sockets and fault injection."""

    def __init__(
        self,
        state: ControllerState,
        cfg: FakeCRXConfig,
        *,
        host: str = "127.0.0.1",
        bootstrap_port: int = 0,
        redirect_port: int = 0,
    ) -> None:
        self._state = state
        self._cfg = cfg
        self._host = host

        self._boot_sock = self._listen(host, bootstrap_port)
        self.bootstrap_port = self._boot_sock.getsockname()[1]
        self._redir_sock = self._listen(host, redirect_port)
        self.redirect_port = self._redir_sock.getsockname()[1]

        self._lock = threading.RLock()
        self._session_active = False
        self._redir_conn: socket.socket | None = None

        # NextSequenceID persists across sessions — only a power-cycle or a
        # successful FRC_Disconnect resets it. Seeded non-1 by default so a client
        # that fails to reseed from GetStatus sends a stale FRC_Call and is dropped.
        self._next_sequence_id = int(cfg.seq_seed)

        # Registers R[1..]; the gripper uses 1/2/3.
        self._registers: dict[int, float] = {}

        # GRIPDISP dispatcher state.
        self._gripdisp_running = False
        #: Cumulative FRC_Call("GRPRUN") RUN-fork count. Distinct from
        #: ``_gripdisp_running`` (a bool the fake treats as idempotent): this counts
        #: every fork so a test can assert the supervisor's at-most-once anti-stacking
        #: guard holds across a multi-attempt bring-up (a RUN-fork can't be un-launched
        #: on real hardware, so each extra call stacks an un-killable task).
        self._grprun_call_count = 0
        self.gripdisp_direct_called = False
        self._gripper_pending: dict[str, int] | None = None
        self._gripper_reads_since_trigger = 0
        self.last_gripper_command: dict[str, int] | None = None

        # Error injection: command name -> queue of ErrorIDs to return.
        self._error_queue: dict[str, deque[int]] = defaultdict(deque)

        # FRC_Continue paused-state model (RMI §2.3.4). Default paused=True so a
        # bare FRC_Continue is a clean no-op (ErrorID 0). set_program_paused(False)
        # makes it return 2556938 "TP Program is Not Paused." — the unpaused case
        # the client tolerates as a no-op success. Scriptable, and also reachable
        # via arm_error("FRC_Continue", ERR_TP_NOT_PAUSED).
        self._program_paused = True
        # Monotonic controller time tag echoed in FRC_ReadJointAngles replies.
        self._joint_time_tag = 0

        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        for target, name in (
            (self._bootstrap_accept_loop, "fake-crx-rmi-boot"),
            (self._redirect_accept_loop, "fake-crx-rmi-redir"),
        ):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def close(self) -> None:
        self._stop.set()
        for sock in (self._boot_sock, self._redir_sock, self._redir_conn):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()

    @staticmethod
    def _listen(host: str, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.listen(4)
        sock.settimeout(0.05)
        return sock

    # -- accept loops -----------------------------------------------------

    def _bootstrap_accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _addr = self._boot_sock.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            with conn:
                conn.settimeout(2.0)
                buf = bytearray()
                try:
                    req = self._read_line(conn, buf)
                except (TimeoutError, OSError):
                    continue
                if req is None:
                    continue
                self._handle_bootstrap(conn, req)

    def _handle_bootstrap(self, conn: socket.socket, req: dict[str, Any]) -> None:
        comm = req.get("Communication")
        if comm != "FRC_Connect_STMO":
            self._send(conn, {"Communication": comm, "ErrorID": 0})
            return
        with self._lock:
            if self._cfg.single_session and self._session_active:
                # A second concurrent connect is refused: the controller serves
                # one RMI session at a time (INTERIM_FACTS.rmi_single_session).
                self._send(
                    conn,
                    {"Communication": "FRC_Connect_STMO", "ErrorID": ERR_ALREADY_CONNECTED},
                )
                return
        self._send(
            conn,
            {
                "Communication": "FRC_Connect_STMO",
                "ErrorID": 0,
                "PortNumber": self.redirect_port,
                "MajorVersion": self._cfg.major_version,
                "MinorVersion": self._cfg.minor_version,
            },
        )

    def _redirect_accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _addr = self._redir_sock.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            conn.settimeout(0.2)
            with self._lock:
                self._session_active = True
                self._redir_conn = conn
            try:
                self._serve_redirect(conn)
            finally:
                with self._lock:
                    self._session_active = False
                    self._redir_conn = None
                try:
                    conn.close()
                except OSError:
                    pass

    def _serve_redirect(self, conn: socket.socket) -> None:
        # Persistent per-connection buffer: a single recv() can carry multiple
        # pipelined lines (e.g. a fire-and-forget FRC_Call immediately followed
        # by FRC_GetStatus); leftover bytes MUST survive to the next read.
        buf = bytearray()
        while not self._stop.is_set():
            try:
                req = self._read_line(conn, buf)
            except TimeoutError:
                continue
            except OSError:
                return
            if req is None:
                return  # peer closed
            reply = self._dispatch(req)
            if reply is not None:
                self._send(conn, reply)

    # -- dispatch ---------------------------------------------------------

    def _dispatch(self, req: dict[str, Any]) -> dict[str, Any] | None:
        cmd = req.get("Command")
        instr = req.get("Instruction")
        comm = req.get("Communication")

        if instr == "FRC_Call":
            return self._handle_call(req)
        if comm == "FRC_Disconnect":
            return self._handle_disconnect()
        if cmd is None:
            return {"ErrorID": 0}

        # Injected error for this command takes precedence.
        with self._lock:
            q = self._error_queue.get(cmd)
            injected = q.popleft() if q else None
        if injected is not None:
            return self._echo(req, error_id=injected)

        handler = {
            "FRC_Initialize": self._handle_initialize,
            "FRC_Reset": self._handle_reset,
            "FRC_Abort": self._handle_abort,
            "FRC_Continue": self._handle_continue,
            "FRC_GetStatus": self._handle_get_status,
            "FRC_GetExtStatus": self._handle_get_ext_status,
            "FRC_ReadError": self._handle_read_error,
            "FRC_ReadRegister": self._handle_read_register,
            "FRC_WriteRegister": self._handle_write_register,
            "FRC_ReadJointAngles": self._handle_read_joint_angles,
        }.get(cmd)
        if handler is None:
            return self._echo(req, error_id=0)
        return handler(req)

    def _handle_initialize(self, req: dict[str, Any]) -> dict[str, Any]:
        # A successful Initialize does NOT reset NextSequenceID (it persists till
        # power-cycle). It does clear the deviation-fault latch / in_error.
        with self._lock:
            self._state.clear_recoverable_faults()
        return self._echo(req, error_id=0)

    def _handle_reset(self, req: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._state.clear_recoverable_faults()
        return self._echo(req, error_id=0)

    def _handle_abort(self, req: dict[str, Any]) -> dict[str, Any]:
        # Kills any running STREAM_MOTN / GRIPDISP (motion_possible drops).
        with self._lock:
            self._state.stream_motn_launched = False
            self._gripdisp_running = False
        return self._echo(req, error_id=0)

    def _handle_continue(self, req: dict[str, Any]) -> dict[str, Any]:
        # FRC_Continue resumes a paused TP program: ErrorID 0 when paused,
        # 2556938 "TP Program is Not Paused." otherwise. Default paused=True
        # makes a bare FRC_Continue a clean no-op; set_program_paused(False)
        # models the unpaused case the client tolerates. An injected error
        # (arm_error) still takes precedence in _dispatch.
        with self._lock:
            paused = self._program_paused
        if paused:
            return self._echo(req, error_id=0)
        return self._echo(req, error_id=ERR_TP_NOT_PAUSED)

    def _handle_read_joint_angles(self, req: dict[str, Any]) -> dict[str, Any]:
        # Report the plant's true measured joints, but with the vendor RMI
        # conversion J3 += J2 applied (INTERIM_FACTS.rmi_j3_plus_j2_conversion):
        # RMI joint reads are UNCONVERTED relative to Stream Motion. Wired to the
        # shared plant so the reply tracks whatever the plant is currently at.
        with self._state.lock:
            q_deg, _qd = self._state._plant.snapshot_deg()
        j = [float(v) for v in q_deg.tolist()]
        if len(j) >= 3:
            j[2] = j[2] + j[1]  # J3 += J2 (vendor behavior on RMI reads)
        joint_angle = {f"J{idx + 1}": j[idx] for idx in range(len(j))}
        with self._lock:
            tag = self._joint_time_tag
            self._joint_time_tag += 1
        return {
            "Command": "FRC_ReadJointAngles",
            "ErrorID": 0,
            "TimeTag": tag,
            "JointAngle": joint_angle,
        }

    def _handle_get_status(self, req: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            st = self._state
            reply = {
                "Command": "FRC_GetStatus",
                "ErrorID": 0,
                "ServoReady": 1 if st.servo_ready() else 0,
                # 1=T1, 0=AUTO on this SOP-less CRX (measured on the controller).
                "TPMode": 1 if st.tp_enabled else 0,
                "RMIMotionStatus": 0,
                "ProgramStatus": 1 if st.stream_motn_launched else 0,
                "SingleStepMode": 0,
                "NumberUTool": 1,
                "NextSequenceID": self._next_sequence_id,
                "NumberUFrame": 1,
                "Override": int(st.gen_override_pct),
            }
        return reply

    def _handle_get_ext_status(self, req: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            st = self._state
            reply = {
                "Command": "FRC_GetExtStatus",
                "ErrorID": 0,
                "InMotion": 1 if st.in_motion else 0,
                "DrivesPowered": 1 if st.drives_powered() else 0,
                "GenOverride": int(st.gen_override_pct),
                "SpeedClampLimit": float(st.speed_clamp_limit_pct),
                "ErrorCode": st.error_code,
                "ControlMode": st.control_mode,
            }
        return reply

    def _handle_read_error(self, req: dict[str, Any]) -> dict[str, Any]:
        count = int(req.get("Count", 1))
        with self._lock:
            texts = list(self._state.alarm_texts)
        texts = texts[: max(1, min(5, count))]
        reply: dict[str, Any] = {
            "Command": "FRC_ReadError",
            "ErrorID": 0,
            "Count": len(texts),
        }
        keys = ("ErrorData", "ErrorData2", "ErrorData3", "ErrorData4", "ErrorData5")
        for key, text in zip(keys, texts, strict=False):
            reply[key] = text
        if not texts:
            reply["ErrorData"] = ""
        return reply

    def _handle_read_register(self, req: dict[str, Any]) -> dict[str, Any]:
        n = int(req.get("RegisterNumber", 0))
        with self._lock:
            if n == REG_CMD:
                self._advance_gripdisp_on_read()
            value = self._registers.get(n, 0)
        return {
            "Command": "FRC_ReadRegister",
            "ErrorID": 0,
            "RegisterNumber": n,
            "RegisterValue": value,
        }

    def _handle_write_register(self, req: dict[str, Any]) -> dict[str, Any]:
        n = int(req.get("RegisterNumber", 0))
        value = req.get("RegisterValue", 0)
        with self._lock:
            self._registers[n] = value
            if n == REG_CMD and int(value) == 1:
                self._gripper_triggered()
        return self._echo(req, error_id=0)

    def _handle_call(self, req: dict[str, Any]) -> dict[str, Any] | None:
        seq = int(req.get("SequenceID", 0))
        program = req.get("ProgramName", "")
        with self._lock:
            if seq < self._next_sequence_id:
                # Stale / duplicate SequenceID — silently dropped (no ack, no
                # launch). This is why the client reseeds from NextSequenceID.
                return None
            self._next_sequence_id = seq + 1
            if program == "STREAM_MOTN":
                self._state.stream_motn_launched = True
            elif program == "GRPRUN":
                self._gripdisp_running = True
                self._grprun_call_count += 1
            elif program == "GRIPDISP":
                # Direct call wedges the RMI instruction queue: the dispatcher is
                # NOT forked, so R[1] never clears. Never do this.
                self.gripdisp_direct_called = True
        # FRC_Call is fire-and-forget on the client; the controller still sends
        # an ack, which lands in the client's async/orphan ring. Configurable so
        # simpler test clients can disable it.
        if self._cfg.send_call_ack:
            return {"Instruction": "FRC_Call", "SequenceID": seq, "ErrorID": 0}
        return None

    def _handle_disconnect(self) -> dict[str, Any]:
        with self._lock:
            # A successful Disconnect resets NextSequenceID to 1.
            self._next_sequence_id = 1
            self._state.stream_motn_launched = False
        return {"Communication": "FRC_Disconnect", "ErrorID": 0}

    # -- GRIPDISP dispatcher ---------------------------------------------

    def _gripper_triggered(self) -> None:
        """R[1] was written 1 — latch the pending R[2]/R[3] action."""
        action = int(self._registers.get(REG_ACTION, 0))
        modifier = int(self._registers.get(REG_R3, 0))
        self._gripper_pending = {"action": action, "modifier": modifier}
        self._gripper_reads_since_trigger = 0

    def _advance_gripdisp_on_read(self) -> None:
        """Advance the dispatcher when the client polls R[1].

        Only runs if GRIPDISP was started via GRPRUN. After
        ``gripdisp_clear_after_reads`` polls it executes the action and clears
        R[1]=0 (the poll-until-0 completion the client waits on). A direct
        GRIPDISP call leaves the dispatcher un-forked, so R[1] stays 1 forever.
        """
        if not self._gripdisp_running or self._gripper_pending is None:
            return
        self._gripper_reads_since_trigger += 1
        if self._gripper_reads_since_trigger >= self._cfg.gripdisp_clear_after_reads:
            self.last_gripper_command = dict(self._gripper_pending)
            self._gripper_pending = None
            self._registers[REG_CMD] = 0  # dispatcher clears the trigger

    # -- injection API ----------------------------------------------------

    def arm_error(self, command: str, error_id: int, *, times: int = 1) -> None:
        """Arm ``command`` to return ``error_id`` on its next ``times`` calls."""
        with self._lock:
            for _ in range(int(times)):
                self._error_queue[command].append(int(error_id))

    def arm_syst_348(self, *, on: str = "FRC_Initialize") -> None:
        """Arm a SYST-348 payload-monitor block: ``on`` returns 2556936 and
        ReadError surfaces the SYST-348 alarm text."""
        with self._lock:
            self._error_queue[on].append(ERR_CANNOT_EXECUTE_TP_B)
            self._state.raise_alarm("SYST-348", "SYST-348 Payload monitor detected")

    def push(self, packet: dict[str, Any]) -> bool:
        """Send an out-of-band async packet on the live redirect session."""
        with self._lock:
            conn = self._redir_conn
        if conn is None:
            return False
        try:
            self._send(conn, packet)
            return True
        except OSError:
            return False

    def push_system_fault(self, *, event_id: int | None = None) -> bool:
        pkt: dict[str, Any] = {"Communication": "FRC_SystemFault", "ErrorID": 0}
        if event_id is not None:
            pkt["EventID"] = int(event_id)
        return self.push(pkt)

    def set_next_sequence_id(self, value: int) -> None:
        with self._lock:
            self._next_sequence_id = int(value)

    def set_program_paused(self, paused: bool) -> None:
        """Script the FRC_Continue paused-state: True → ErrorID 0, False → 2556938."""
        with self._lock:
            self._program_paused = bool(paused)

    @property
    def next_sequence_id(self) -> int:
        with self._lock:
            return self._next_sequence_id

    @property
    def gripdisp_running(self) -> bool:
        with self._lock:
            return self._gripdisp_running

    @property
    def grprun_call_count(self) -> int:
        """Number of FRC_Call("GRPRUN") RUN-forks the controller has received."""
        with self._lock:
            return self._grprun_call_count

    def read_register_value(self, n: int) -> float:
        with self._lock:
            return self._registers.get(n, 0)

    # -- transport helpers ------------------------------------------------

    def _echo(self, req: dict[str, Any], *, error_id: int) -> dict[str, Any]:
        for key in _ECHO_KEYS:
            if key in req:
                return {key: req[key], "ErrorID": error_id}
        return {"ErrorID": error_id}

    def _send(self, conn: socket.socket, obj: dict[str, Any]) -> None:
        line = (json.dumps(obj) + "\r\n").encode("ascii")
        with self._lock:
            conn.sendall(line)

    @staticmethod
    def _read_line(conn: socket.socket, buf: bytearray) -> dict[str, Any] | None:
        """Read one ``\\n``-terminated JSON object, consuming from ``buf``.

        ``buf`` is owned by the caller and persists across calls so pipelined
        lines are never dropped. Raises ``socket.timeout`` when no complete line
        is buffered and the recv times out (caller loops to re-check shutdown).
        """
        while True:
            nl = buf.find(b"\n")
            if nl >= 0:
                raw = bytes(buf[:nl]).rstrip(b"\r")
                del buf[: nl + 1]
                return json.loads(raw.decode("ascii"))
            chunk = conn.recv(4096)
            if not chunk:
                if buf:
                    raw = bytes(buf).rstrip(b"\r")
                    buf.clear()
                    return json.loads(raw.decode("ascii"))
                return None
            buf.extend(chunk)
