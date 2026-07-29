#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Adversarial-load generator for the P2' timing spike.

Run this ALONGSIDE ``rt_loop`` to stress the host while the RT loop is measured.
It reproduces the contention classes the RT core must survive in-process
(PLAN.md §5.5, design doc 07 §3.4):

  * GIL storm    -- N threads spinning tight Python bytecode loops, plus
                    optional periodic *held-GIL* stalls (a C-extension-free way
                    to emulate the "deliberate 200 ms GIL holds").
  * malloc / gc  -- N threads churning the allocator + forcing gc, to provoke
                    heap-lock contention and page reclaim.
  * page cache   -- a large anonymous mmap that is sequentially touched to
                    apply page-cache / reclaim pressure (used because ``fio``
                    may be absent). This is the scenario that would evict RT
                    text pages without mlock (R1-E3).
  * torch        -- OPTIONAL, behind ``try: import torch``. Not present in the
                    dev venv; DO NOT rely on it. The REAL 1 h soak must instead
                    co-locate the actual grocery curobo/voxel/torch process (see
                    README.md) -- this flag is only a stand-in for local probing.

This is throwaway load: it imports nothing from airo_fanuc.
"""
from __future__ import annotations

import argparse
import ctypes
import gc
import mmap
import os
import sys
import threading
import time

_stop = threading.Event()


def gil_storm(hold_ms: float, hold_period_s: float) -> None:
    """Tight bytecode loop -> monopolises the GIL. Every hold_period_s, hold the
    GIL uninterrupted for hold_ms (emulates a long no-yield section)."""
    x = 0
    last_hold = time.monotonic()
    while not _stop.is_set():
        # tight loop; CPython releases the GIL only every few thousand bytecodes
        for _ in range(20000):
            x = (x * 1103515245 + 12345) & 0x7FFFFFFF
        if hold_ms > 0.0:
            now = time.monotonic()
            if now - last_hold >= hold_period_s:
                # busy-hold: a pure-Python spin that never yields voluntarily
                deadline = time.perf_counter() + hold_ms * 1e-3
                y = 0
                while time.perf_counter() < deadline:
                    y += 1
                last_hold = now
    globals()["_gil_sink"] = x


def malloc_gc_churn() -> None:
    """Allocate and drop lists + force gc -> heap-lock + reclaim contention."""
    while not _stop.is_set():
        junk = [bytearray(4096) for _ in range(512)]  # ~2 MB churn
        junk = [b for b in junk if len(b) > 1]         # touch + rebind
        del junk
        gc.collect()
        time.sleep(0.001)


def page_cache_pressure(mb: int) -> None:
    """Sequentially touch a large anonymous mapping to drive reclaim pressure."""
    if mb <= 0:
        return
    size = mb * 1024 * 1024
    mm = mmap.mmap(-1, size)
    pagesz = 4096
    while not _stop.is_set():
        i = 0
        while i < size and not _stop.is_set():
            mm[i] = (mm[i] + 1) & 0xFF
            i += pagesz
    mm.close()


def try_torch_load() -> bool:
    """Optional torch churn. Returns False (with a note) if torch is absent."""
    try:
        import torch  # noqa: F401
    except Exception as e:  # pragma: no cover - torch not in dev venv
        print(f"[load] torch unavailable ({e.__class__.__name__}); skipping. "
              "The real soak MUST include the actual curobo/voxel/torch process.",
              flush=True)
        return False

    def worker() -> None:
        import torch
        dev = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"[load] torch churn on {dev}", flush=True)
        while not _stop.is_set():
            a = torch.randn(1024, 1024, device=dev)
            b = torch.randn(1024, 1024, device=dev)
            c = a @ b
            _ = float(c.sum().item())
            del a, b, c
    threading.Thread(target=worker, daemon=True, name="torch").start()
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description="adversarial load for the RT spike")
    ap.add_argument("--gil-threads", type=int, default=2)
    ap.add_argument("--malloc-threads", type=int, default=2)
    ap.add_argument("--page-cache-mb", type=int, default=1024,
                    help="size of the touched anonymous mmap (0 disables)")
    ap.add_argument("--gil-hold-ms", type=float, default=200.0,
                    help="periodic uninterrupted GIL hold length (0 disables)")
    ap.add_argument("--gil-hold-period-s", type=float, default=2.0)
    ap.add_argument("--torch", action="store_true",
                    help="also run torch matmul churn if torch importable")
    ap.add_argument("--duration-s", type=float, default=0.0,
                    help="stop after S seconds (0 = run until Ctrl-C)")
    args = ap.parse_args()

    print(f"[load] pid={os.getpid()} gil={args.gil_threads} malloc={args.malloc_threads} "
          f"page_cache={args.page_cache_mb}MB gil_hold={args.gil_hold_ms}ms/"
          f"{args.gil_hold_period_s}s torch={args.torch}", flush=True)
    print("[load] NOTE: this Python load is intentionally UNPINNED. On olifant it "
          "must be confined to cores 0-14,16-30 by the demo slice so it cannot "
          "touch the RT sibling pair 15/31 (README.md).", flush=True)

    threads = []
    for i in range(args.gil_threads):
        threads.append(threading.Thread(
            target=gil_storm, args=(args.gil_hold_ms, args.gil_hold_period_s),
            daemon=True, name=f"gil{i}"))
    for i in range(args.malloc_threads):
        threads.append(threading.Thread(target=malloc_gc_churn, daemon=True, name=f"malloc{i}"))
    if args.page_cache_mb > 0:
        threads.append(threading.Thread(
            target=page_cache_pressure, args=(args.page_cache_mb,), daemon=True, name="pagecache"))
    for t in threads:
        t.start()
    if args.torch:
        try_torch_load()

    _ = ctypes  # (kept importable in case a caller wants to extend with C stalls)
    t0 = time.monotonic()
    try:
        while True:
            if args.duration_s and (time.monotonic() - t0) >= args.duration_s:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        _stop.set()
        print(f"\n[load] stopping after {time.monotonic() - t0:.1f}s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
