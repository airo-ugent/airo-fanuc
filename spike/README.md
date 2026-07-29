# P2′ timing spike — in-process 125 Hz RT-loop go/no-go

**Question this answers (PLAN.md §9, P2′ row):** can an **in-process** C++ Stream
Motion RT loop, pinned to core 31 inside the same CUDA/voxel/GIL-heavy grocery
process, hold 8 ms TX cadence under a full hour of adversarial load? If yes, the
driver's RT core lives in-process (design doc 07). If no, the pre-committed
fallback re-homes the identical API into a dedicated process (PLAN.md §10 risk 2).

This is a **self-contained** harness (P2′ decision): it depends on nothing else
in `airo_fanuc` — a throwaway `status_gen.py` feeds it packets, not `FakeCRX` or
`wire.py`. The packet **sizes** match the real wire (status 416 B, command
344 B) but the loopback is native-endian; wire correctness is a later phase.

```
spike/
  rt_loop.cpp       RT-loop skeleton: epoll{UDP, timerfd} + PLL-clocked TX + metrics + VERDICT
  status_gen.py     throwaway 125 Hz 416-byte status generator (--jitter-ms, --drop-rate)
  load.py           adversarial load: GIL storm + malloc/gc churn + page-cache pressure + optional torch
  CMakeLists.txt    standalone build (no parent-package coupling)
  README.md         this file
```

---

## Privilege warning (why CI / the agent sandbox cannot produce the verdict)

Three of the measured knobs need privileges an unprivileged shell (and the agent
sandbox) lacks:

| Flag | Needs | Without it |
|---|---|---|
| `--pin-core 31` | membership in a cpuset that includes core 31 | `EINVAL`/`EPERM`, warns, runs unpinned → **migrations ≠ 0** |
| `--sched-fifo 80` | `CAP_SYS_NICE` (root) or `RLIMIT_RTPRIO` | `EPERM`, warns, runs `SCHED_OTHER` |
| `--mlock` | root or `RLIMIT_MEMLOCK` | warns, runs unlocked → majflt risk under page-cache pressure |

`rt_loop` degrades gracefully on each (prints a `[warn]` and continues), and when
pin+fifo+mlock are **not** all active it stamps the report:
`*** NOT A GO/NO-GO VERDICT ***`. **Only a run with all three, on the isolated
host, under real load, for ≥ 1 h, is the go/no-go.**

---

## 1. Build (on olifant)

```
cd packages/airo_fanuc/spike
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/rt_loop --help
```

Requires C++17 (gcc-11 verified), cmake ≥ 3.16. No CUDA, no numpy, no vendored
sources.

---

## 2. Host isolation (the load-bearing part — PLAN.md §5.5, R1 E1/E2/E3)

The whole point of the spike is that core 31 is **exclusively** the RT thread's,
and its **SMT sibling 15 is kept idle** (verified `thread_siblings_list=15,31`).
Everything else in the system is confined to `0-14,16-30`.

> These are temporary changes for the soak. Note the current production units
> (`fanuc.slice`, `user.slice.d/99-reserve-core-31.conf`, etc.) reserve only 31
> with others on `0-30` — for this spike the **pair 15+31** must be reserved, so
> the drop-ins below use `0-14,16-30`. **Revert after** (see §8).

**2a. Reserve the 15+31 pair from every other slice (cgroup v2 cpuset):**

```
sudo mkdir -p /etc/systemd/system/user.slice.d /etc/systemd/system/system.slice.d
printf '[Slice]\nAllowedCPUs=0-14,16-30\n' | sudo tee /etc/systemd/system/user.slice.d/99-spike-reserve-pair.conf
printf '[Slice]\nAllowedCPUs=0-14,16-30\n' | sudo tee /etc/systemd/system/system.slice.d/99-spike-reserve-pair.conf
sudo systemctl daemon-reload
```

(The RT spike itself runs in `grocery-demo.slice` with `AllowedCPUs=0-31`, granted
inline in §4, so it — and only it — can touch core 31.)

**2b. NIC: kill RX interrupt coalescing on the robot link (R1 A2/E2):**

```
sudo ethtool -C eno2 rx-usecs 0 adaptive-rx off   # some NICs reject one flag; apply what sticks
```

**2c. Pin the eno2 RX IRQ to core 31 and ban irqbalance from moving it:**

```
sudo systemctl stop irqbalance
grep eno2 /proc/interrupts                          # note the IRQ number(s), call it N
echo 31 | sudo tee /proc/irq/N/smp_affinity_list    # repeat per eno2 queue IRQ
```

(For this spike `status_gen` streams over **loopback**, not eno2, so IRQ/ethtool
tuning is exercising the procedure/host, not this run's packet path. Keep it in
because the real driver's RX arrives on eno2 and P6 HIL repeats the soak there.)

---

## 3. Launch the adversarial load (confined to 0-14,16-30 by §2a)

Run these from a normal shell (→ `user.slice` → cannot touch 15/31):

```
# throwaway packet source (light; lives with the load, not on the RT core)
.venv-dev/bin/python spike/status_gen.py --port 60115 &

# synthetic contention: GIL storm + 200 ms GIL holds + malloc/gc + 4 GB page-cache pressure
.venv-dev/bin/python spike/load.py --gil-threads 4 --malloc-threads 2 --page-cache-mb 4096 --gil-hold-ms 200 --gil-hold-period-s 2 &
```

**The synthetic load is NOT sufficient on its own.** `load.py`'s torch path is a
stand-in (torch is not even in the dev venv). The real 1 h soak MUST also run the
**actual grocery curobo/voxel/torch workload** concurrently — e.g. a live
`grocery-test-planner --voxels-mode live --voxels-cameras d415,zed` or a full
`grocery-robot` demo loop — so the RT loop contends with genuine CUDA host
allocations, ESDF updates, and Zenoh traffic (design doc 07 §3.4). Start that in
`user.slice` too so it stays off 15/31.

---

## 4. Run the spike — the real go/no-go (root, pinned, FIFO, mlock, ≥ 1 h)

Run **inside** `grocery-demo.slice` so the process may use core 31, with the
rtprio + memlock limits granted inline:

```
sudo systemd-run --slice=grocery-demo.slice -p AllowedCPUs=0-31 -p LimitRTPRIO=99 -p LimitMEMLOCK=infinity --pty --wait --collect \
  "$PWD/build/rt_loop" --pin-core 31 --sched-fifo 80 --mlock --warmup-s 30 --duration-s 3600 --status-port 60115
```

`--warmup-s 30` faults in code pages, warms caches, and lets the PLL lock before
metrics start; `--duration-s 3600` is the 1 h soak. Watch the live start line
confirm `pin=on fifo=on mlock=on`. If any says `off`, fix privileges before
trusting the numbers.

(Equivalent without systemd-run, if you have made `grocery-demo.slice` persistent
and your login session is in it: `sudo chrt -f 80 ./build/rt_loop ...` — but the
cpuset must include 31 or `--pin-core 31` will `EINVAL`.)

---

## 5. Read the VERDICT — exact PASS/FAIL gates

At the end `rt_loop` prints a `VERDICT` block. **GO** requires every gate PASS
*and* the header NOT showing `NOT A GO/NO-GO VERDICT`:

| Gate | Threshold | Source |
|---|---|---|
| TX interval **p99** | ≤ 8.5 ms | PLAN.md §9 |
| TX interval **p999** | ≤ 9 ms | PLAN.md §9 |
| TX interval **max** | < 16 ms | PLAN.md §9 (L3 superset) |
| **rx→tx p99** | < 500 µs | design doc 07 §3.4 |
| **majflt** (post-warmup) | = 0 | R1 E3 |
| **cpu migrations** | = 0 | R1 E1 |
| **hot-path heap allocs** | = 0 | zero-alloc invariant |
| **one TX / one τ per window** | `double_send_guard = 0`, `tx_count == tau_advance_count` | R1 A1/A2 |

Also sanity-check the report body: `nonvol_ctxsw` ≈ 0/min (R1 E4), `seq_gaps`/
`missed_rx_ticks` ≈ 0 on loopback (a nonzero count means the PLL lost lock or the
generator dropped — expected only when you deliberately set `--drop-rate`).

---

## 6. Ablations (do these too — they decide the config defaults)

- **Partition is load-bearing (design doc 07 §3.4):** repeat §4 after *removing*
  the §2a drop-ins (`sudo rm .../99-spike-reserve-pair.conf && sudo systemctl
  daemon-reload`) so other work can land on 31. Expect migrations/jitter to
  worsen — that proves the reservation matters, mirroring the dries measurement
  (p99 11.75 → 17.95 ms without isolation).
- **FIFO/mlock drop-criterion (PLAN.md §5.5 L3):** run **affinity-only** (drop
  `--sched-fifo` and `--mlock`, keep `--pin-core 31`). If that alone holds
  p99 ≤ 8.5 ms and max < 16 ms for the full adversarial hour, FIFO+mlock may
  default **off** in the shipped driver. If not, they stay on.
- Probe packet loss / phase with `status_gen --jitter-ms … --drop-rate …` (feeds
  the H2 phase-sweep intuition) — this stresses the rx→tx gate, not the TX gate.

---

## 7. If it FAILS — the pre-committed fallback (PLAN.md §10 risk 2)

A FAIL is not a redesign. The `FanucDriver` API is **declared
process-location-agnostic**: the RT core re-homes into its **own dedicated
process** (as today's systemd-isolated `grocery-fanuc-bridge` is), talking to the
demo over local IPC, with the ur_rtde-shaped API unchanged. Cost is re-importing
a thin IPC surface (not the old chunk protocol); the in-process latency wins in
design doc 07 §6 partially regress, but correctness and the API are preserved.
Record which gate failed and by how much in `docs/controller-notes.md` before
flipping to the IPC layout.

---

## 8. Teardown / revert

```
sudo rm -f /etc/systemd/system/user.slice.d/99-spike-reserve-pair.conf /etc/systemd/system/system.slice.d/99-spike-reserve-pair.conf
sudo systemctl daemon-reload
sudo systemctl start irqbalance
# kill the backgrounded status_gen / load.py / grocery workload
```

---

## Appendix — unprivileged plumbing sanity check (what the harness author ran)

On a 28-core dev box (NOT olifant, no core-31 isolation, no real curobo load), an
unprivileged run `./build/rt_loop --duration-s 30 --warmup-s 5` against
`status_gen.py`, and again with `load.py` (GIL storm + malloc churn + 512 MB
page-cache) alongside, produced:

```
TX interval:  p50 8.000 ms  p99 8.01 ms  p999 8.04 ms  max 8.07 ms
rx->tx:       p50 316 us     p99 347 us    max 380 us      (PLL lock at the 300 us target lead)
majflt 0   hot-path allocs 0   double_send_guard 0   tx_count == tau_advance_count
cpu_migrations 43   -> FAIL, EXPECTED: nothing was pinned (proves the counter is live)
```

These prove the **measurement plumbing** — cadence, PLL lock, freshness,
fault/migration/alloc counters, the one-TX-per-window invariant — is correct and
populated. **They are not the verdict:** the sandbox has no core-31 reservation,
no SMT-sibling isolation, no SCHED_FIFO/mlock, and no genuine CUDA/voxel
contention. The go/no-go is the §4 run on olifant.
