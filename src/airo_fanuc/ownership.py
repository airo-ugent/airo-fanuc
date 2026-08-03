# SPDX-License-Identifier: Apache-2.0
"""Single-owner controller lock via ``flock``.

The FANUC controller accepts exactly ONE Stream Motion peer and one RMI session,
so at most one process may talk to it on either protocol at a time. This is a
hardware-level constraint, not a policy choice:

* the controller does not serve two Stream Motion peers — a second peer gets no
  status packets at all, and its packets are not harmlessly ignored: a foreign
  StopPacket silences the *live* session mid-motion, and a foreign or malformed
  StartPacket can wedge the Stream Motion daemon until a controller power-cycle;
* a second RMI connect is not cleanly refused either — the ``FRC_Connect_STMO``
  itself succeeds and the redirect-port connect then times out, so the loser of
  the race learns about the conflict only as a stall.

This module enforces the exclusion with a ``flock``-based advisory lock on a lock
file whose payload names the current holder (``pid`` / ``mode`` / ``since``).

Why ``flock``: the kernel releases the lock automatically when the holder dies, so
a crashed process never wedges the next start — no stale-PID file to clean up, no
timeout to tune. There is no yield/negotiation protocol; a second acquirer gets a
loud, typed :class:`~airo_fanuc.exceptions.OwnershipError` naming the holder
("kill <PID>"), because silently taking the controller away from a process that
may be mid-trajectory is exactly the conflict the lock exists to prevent.

Modes: ``control`` (Stream Motion + STREAM_MOTN —
:class:`~airo_fanuc.driver.FanucDriver`), ``receive`` (RMI-poll state for
calibration / :class:`~airo_fanuc.receive_interface.FanucReceiveInterface`),
``tool`` (gripper register pokes). All three contend for the *same* lock — only
one process holds the controller regardless of what it intends to do with it.

The lock fd is ``O_CLOEXEC`` so a fork in the robot process cannot leak the lock
into a child (a leaked fd would keep the lock alive past the holder's death and
defeat the kernel-release guarantee above). The lock directory's owner is also
checked: a foreign-owned lock dir would let another user pre-seed or replace the
lock file and hijack ownership.

Wheel-standalone: stdlib ``fcntl`` / ``os`` / ``json`` / ``logging`` only.
Tests override :data:`DEFAULT_LOCK_PATH` to a tmp path — acquiring the lock
never requires root.
"""

from __future__ import annotations

import fcntl
import json
import logging
import os
import sys
import time
from collections.abc import Callable
from pathlib import Path

from airo_fanuc.exceptions import OwnershipError

logger = logging.getLogger("airo_fanuc.ownership")

#: Production lock path. Under ``/run/lock`` so it lives on tmpfs (cleared on
#: reboot) and the kernel-released flock is the whole crash-safety story.
DEFAULT_LOCK_PATH = "/run/lock/airo-fanuc/owner.lock"

#: Valid ownership modes.
VALID_MODES = ("control", "receive", "tool")


class OwnershipLock:
    """A ``flock``-based single-owner controller lock (context-manager).

    Usage::

        with OwnershipLock("receive") as lock:
            ...  # exactly one process holds the controller here

    or explicitly via :meth:`acquire` / :meth:`release`. Acquiring while another
    process holds the lock raises :class:`OwnershipError` naming the holder.
    """

    def __init__(
        self,
        mode: str,
        *,
        path: str | os.PathLike[str] = DEFAULT_LOCK_PATH,
        pid: int | None = None,
        check_dir_owner: bool = True,
        clock: Callable[[], float] = time.time,
    ) -> None:
        if mode not in VALID_MODES:
            raise ValueError(f"mode must be one of {VALID_MODES}, got {mode!r}")
        self._mode = mode
        self._path = Path(path)
        self._pid = int(pid) if pid is not None else os.getpid()
        self._check_dir_owner = bool(check_dir_owner)
        self._clock = clock
        self._fd: int | None = None
        self._since: float | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def path(self) -> Path:
        return self._path

    @property
    def pid(self) -> int:
        """The pid recorded as the holder — the same value written into the holder file, so
        a consumer reading ``owner.pid`` off a published snapshot sees what a second process
        would be told is holding the lock."""
        return self._pid

    @property
    def fd(self) -> int | None:
        """The open lock fd while held (``None`` otherwise). Exposed for tests
        (O_CLOEXEC assertion)."""
        return self._fd

    @property
    def since(self) -> float | None:
        """Epoch acquisition time while held, else ``None``."""
        return self._since

    def acquire(self) -> OwnershipLock:
        """Take the lock. Raises :class:`OwnershipError` if another process holds it.

        Non-blocking (``LOCK_NB``): on contention we read the current holder's
        ``pid`` / ``mode`` / ``since`` from the lock file and raise immediately —
        no waiting, no yield negotiation. A caller that blocked here would sit
        for the controller's idle timeout behind a hung (SIGSTOP'd) holder, which
        the kernel will not release; failing loudly with the holder's PID is the
        only actionable outcome.
        """
        if self._fd is not None:
            return self  # already held by this instance (idempotent)

        self._ensure_lock_dir()
        # O_CLOEXEC: the lock must not leak across a fork/exec.
        fd = os.open(self._path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            holder = self._read_holder(fd)
            os.close(fd)
            raw_pid = holder.get("pid") if holder else None
            raw_mode = holder.get("mode") if holder else None
            raw_since = holder.get("since") if holder else None
            pid = int(raw_pid) if isinstance(raw_pid, (int, float)) else None
            mode = str(raw_mode) if isinstance(raw_mode, str) else None
            since = float(raw_since) if isinstance(raw_since, (int, float)) else None
            hint = f" — kill {pid}" if pid is not None else ""
            raise OwnershipError(
                pid=pid,
                mode=mode,
                since=since,
                message=(
                    f"FANUC controller already owned by pid={pid} mode={mode} "
                    f"since={since} (lock {self._path}){hint}"
                ),
            ) from exc

        # We hold it: stamp the holder record so the next contender can name us.
        self._fd = fd
        self._since = float(self._clock())
        self._write_holder(fd)
        logger.info("ownership: acquired %s lock (pid=%d, %s)", self._mode, self._pid, self._path)
        return self

    def release(self) -> None:
        """Release the lock (idempotent). Kernel also releases it on crash."""
        fd = self._fd
        if fd is None:
            return
        self._fd = None
        self._since = None
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        except OSError as exc:
            logger.debug("ownership: LOCK_UN failed (benign on close): %s", exc)
        finally:
            try:
                os.close(fd)
            except OSError:
                pass
        logger.info("ownership: released %s lock (%s)", self._mode, self._path)

    def __enter__(self) -> OwnershipLock:
        return self.acquire()

    def __exit__(self, *exc: object) -> None:
        self.release()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _ensure_lock_dir(self) -> None:
        """Create the lock directory if needed and verify its owner.

        A lock dir owned by a *different* non-root user could let that user
        pre-seed or replace the lock file to hijack ownership, so we refuse to
        use it. Owned by us or by root is fine (root sets up ``/run/lock``).
        """
        parent = self._path.parent
        try:
            parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            # If it already exists we can still proceed; otherwise surface it.
            if not parent.is_dir():
                raise OwnershipError(message=f"ownership: cannot create lock dir {parent}: {exc}") from exc
        if self._check_dir_owner:
            owner_uid = self._dir_owner_uid(parent)
            my_uid = os.getuid()
            if owner_uid not in (my_uid, 0):
                raise OwnershipError(
                    message=(
                        f"ownership: lock dir {parent} is owned by uid={owner_uid}, "
                        f"not root or the current user (uid={my_uid}) — refusing to "
                        "use a foreign-owned lock directory"
                    )
                )

    @staticmethod
    def _dir_owner_uid(path: Path) -> int:
        """Owning uid of ``path`` (isolated for test monkeypatching)."""
        return os.stat(path).st_uid

    def _write_holder(self, fd: int) -> None:
        record = {
            "pid": self._pid,
            "mode": self._mode,
            "since": self._since,
            "cmdline": " ".join(sys.argv),
        }
        payload = (json.dumps(record) + "\n").encode("utf-8")
        os.ftruncate(fd, 0)
        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, payload)

    @staticmethod
    def _read_holder(fd: int) -> dict[str, object] | None:
        """Best-effort read of the holder record from the lock file."""
        try:
            os.lseek(fd, 0, os.SEEK_SET)
            raw = os.read(fd, 4096)
        except OSError:
            return None
        if not raw:
            return None
        try:
            data = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return None
        return data if isinstance(data, dict) else None


__all__ = ["OwnershipLock", "DEFAULT_LOCK_PATH", "VALID_MODES"]
