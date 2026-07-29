# SPDX-License-Identifier: Apache-2.0
"""Tests for :class:`airo_fanuc.ownership.OwnershipLock`.

The controller accepts one Stream Motion peer and one RMI session, so at most one
process may hold it — in any mode. The lock is a ``flock`` whose file payload names
the holder, so a second acquirer fails loudly with a "kill <PID>" hint instead of
quietly contending for the arm. Covered here: the roundtrip, the named refusal, the
``O_CLOEXEC`` fd (a fork must not inherit ownership) and the lock-directory owner
check (a foreign-owned directory could be pre-seeded to hijack the lock).
"""

from __future__ import annotations

import fcntl
import os
from pathlib import Path

import pytest

from airo_fanuc.exceptions import OwnershipError
from airo_fanuc.ownership import OwnershipLock


def _lock_path(tmp_path: Path) -> Path:
    return tmp_path / "lockdir" / "owner.lock"


def test_acquire_release_roundtrip(tmp_path: Path) -> None:
    lock = OwnershipLock("receive", path=_lock_path(tmp_path))
    assert lock.fd is None and lock.since is None
    lock.acquire()
    try:
        assert lock.fd is not None
        assert lock.since is not None
        assert _lock_path(tmp_path).exists()
    finally:
        lock.release()
    assert lock.fd is None and lock.since is None


def test_context_manager(tmp_path: Path) -> None:
    with OwnershipLock("control", path=_lock_path(tmp_path)) as lock:
        assert lock.fd is not None
    assert lock.fd is None


def test_second_acquirer_raises_naming_holder(tmp_path: Path) -> None:
    path = _lock_path(tmp_path)
    holder = OwnershipLock("receive", path=path)
    holder.acquire()
    try:
        contender = OwnershipLock("control", path=path, pid=424242)
        with pytest.raises(OwnershipError) as ei:
            contender.acquire()
        # The error names the actual holder (this process, receive mode).
        assert ei.value.pid == os.getpid()
        assert ei.value.mode == "receive"
        assert ei.value.since is not None
        assert f"kill {os.getpid()}" in str(ei.value)
        # The contender took no fd.
        assert contender.fd is None
    finally:
        holder.release()


def test_release_frees_for_next_acquirer(tmp_path: Path) -> None:
    path = _lock_path(tmp_path)
    first = OwnershipLock("receive", path=path)
    first.acquire()
    first.release()
    # Now a second process-like acquirer succeeds.
    second = OwnershipLock("control", path=path)
    second.acquire()
    try:
        assert second.fd is not None
    finally:
        second.release()


def test_fd_is_cloexec(tmp_path: Path) -> None:
    with OwnershipLock("receive", path=_lock_path(tmp_path)) as lock:
        assert lock.fd is not None
        flags = fcntl.fcntl(lock.fd, fcntl.F_GETFD)
        assert flags & fcntl.FD_CLOEXEC


def test_invalid_mode_raises() -> None:
    with pytest.raises(ValueError, match="mode must be one of"):
        OwnershipLock("bogus")


def test_idempotent_acquire_and_release(tmp_path: Path) -> None:
    lock = OwnershipLock("tool", path=_lock_path(tmp_path))
    lock.acquire()
    fd = lock.fd
    lock.acquire()  # idempotent — same fd, no new lock
    assert lock.fd == fd
    lock.release()
    lock.release()  # idempotent — no raise
    assert lock.fd is None


def test_holder_record_written(tmp_path: Path) -> None:
    import json

    path = _lock_path(tmp_path)
    with OwnershipLock("receive", path=path, pid=777):
        data = json.loads(path.read_text(encoding="utf-8"))
    assert data["pid"] == 777
    assert data["mode"] == "receive"
    assert isinstance(data["since"], (int, float))


def test_foreign_lock_dir_owner_rejected(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # A lock dir owned by a different non-root user is refused: that user could
    # pre-seed or replace the lock file and hijack ownership of the controller.
    foreign_uid = os.getuid() + 12345
    monkeypatch.setattr(OwnershipLock, "_dir_owner_uid", staticmethod(lambda _p: foreign_uid))
    lock = OwnershipLock("receive", path=_lock_path(tmp_path))
    with pytest.raises(OwnershipError, match="foreign-owned"):
        lock.acquire()
    assert lock.fd is None


def test_foreign_lock_dir_owner_check_can_be_disabled(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    foreign_uid = os.getuid() + 12345
    monkeypatch.setattr(OwnershipLock, "_dir_owner_uid", staticmethod(lambda _p: foreign_uid))
    lock = OwnershipLock("receive", path=_lock_path(tmp_path), check_dir_owner=False)
    lock.acquire()
    try:
        assert lock.fd is not None
    finally:
        lock.release()
