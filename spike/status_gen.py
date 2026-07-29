#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Throwaway 125 Hz Stream Motion *status* generator for the P2' timing spike.

Streams 416-byte packets over loopback UDP with a monotonic sequence number and
a millisecond timestamp, mimicking the SIZE and a few field offsets of the real
``RobotStatusPacket`` (type 204) -- see
``vendor/fanuc_driver/fanuc_libs/stream_motion/include/stream_motion/packets.hpp``.

Deliberately standalone: it does NOT import ``wire.py`` / ``FakeCRX`` / anything
from the airo_fanuc package (P2' self-containment decision). It is ~throwaway
scaffolding whose only job is to feed ``rt_loop`` a realistic packet cadence so
the RT loop's timing can be characterised.

The generator learns the client's address from the first datagram the client
sends (``rt_loop`` fires a small StartPacket on connect), then streams status to
that address at 125 Hz until Ctrl-C.

Knobs for probing robustness:
  --jitter-ms J   uniform +-J ms jitter added to each send instant
  --drop-rate R   fraction [0,1] of packets randomly dropped (packet loss)

Run it BEFORE rt_loop, on the same --port:

    python3 status_gen.py --port 60115
"""
from __future__ import annotations

import argparse
import random
import socket
import struct
import sys
import time

STATUS_BYTES = 416
PACKET_TYPE_204 = 204
VERSION = 3

# Field byte offsets inside RobotStatusPacket (#pragma pack(1)):
#   packet_type u32 @0, version u32 @4, sequence_no u32 @8,
#   status/robot_status/contact/unused (4x u8) @12..15, time_stamp u32 @16,
#   position[9] f32 @20, joint_angle[9] f32 @56, current[9] f32 @92,
#   safety_scale f32 @128, forces 6x f32 @132, fs_type u32 @156,
#   io_status[256] @160  -> total 416.
OFF_SEQ = 8
OFF_TS = 16
OFF_JA = 56


def build_packet(buf: bytearray, seq: int, t_ms: int, t_s: float) -> None:
    """Fill a preallocated 416-byte buffer in place (avoids per-send alloc)."""
    struct.pack_into("<I", buf, 0, PACKET_TYPE_204)
    struct.pack_into("<I", buf, 4, VERSION)
    struct.pack_into("<I", buf, OFF_SEQ, seq & 0xFFFFFFFF)
    struct.pack_into("<I", buf, OFF_TS, t_ms & 0xFFFFFFFF)
    # A few plausible, moving joint angles so the client's decode reads live data.
    for j in range(6):
        ang = 30.0 * (0.5 + 0.5 * ((seq // 8) % 2)) * (1.0 + 0.1 * j)
        struct.pack_into("<f", buf, OFF_JA + 4 * j, ang * (0.5 + 0.5 * (t_s % 1.0)))


def main() -> int:
    ap = argparse.ArgumentParser(description="125 Hz throwaway SM status generator")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=60115)
    ap.add_argument("--rate-hz", type=float, default=125.0)
    ap.add_argument("--jitter-ms", type=float, default=0.0,
                    help="uniform +-J ms jitter on each send instant")
    ap.add_argument("--drop-rate", type=float, default=0.0,
                    help="fraction [0,1] of packets randomly dropped")
    ap.add_argument("--duration-s", type=float, default=0.0,
                    help="stop after S seconds (0 = run until Ctrl-C)")
    args = ap.parse_args()

    period = 1.0 / args.rate_hz
    jitter = max(0.0, args.jitter_ms) * 1e-3

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    sock.settimeout(5.0)

    print(f"[status_gen] listening {args.host}:{args.port}, "
          f"waiting for client StartPacket ...", flush=True)
    client = None
    while client is None:
        try:
            _data, addr = sock.recvfrom(2048)
            client = addr
        except socket.timeout:
            print("[status_gen] still waiting for a client ...", flush=True)
    print(f"[status_gen] client {client}; streaming {args.rate_hz:.0f} Hz "
          f"({STATUS_BYTES} B), jitter={args.jitter_ms}ms drop={args.drop_rate}",
          flush=True)

    sock.setblocking(False)  # never let a full client buffer stall our cadence
    buf = bytearray(STATUS_BYTES)
    seq = 0
    sent = 0
    dropped = 0
    t0 = time.monotonic()
    next_t = t0
    try:
        while True:
            now = time.monotonic()
            if args.duration_s and (now - t0) >= args.duration_s:
                break
            sleep = next_t - now
            if sleep > 0:
                time.sleep(sleep)
            seq += 1
            fire = next_t + (random.uniform(-jitter, jitter) if jitter else 0.0)
            # busy-ish wait for the jittered instant (jitter is small)
            while jitter and time.monotonic() < fire:
                pass
            if args.drop_rate and random.random() < args.drop_rate:
                dropped += 1
            else:
                t_now = time.monotonic()
                build_packet(buf, seq, int((t_now - t0) * 1000.0) & 0xFFFFFFFF, t_now - t0)
                try:
                    sock.sendto(buf, client)
                    sent += 1
                except (BlockingIOError, OSError):
                    dropped += 1
            next_t += period
            # if we fell badly behind, resync so we don't spiral
            if next_t < time.monotonic() - period:
                next_t = time.monotonic() + period
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        elapsed = time.monotonic() - t0
        rate = sent / elapsed if elapsed > 0 else 0.0
        print(f"\n[status_gen] sent={sent} dropped={dropped} "
              f"elapsed={elapsed:.1f}s eff_rate={rate:.1f} Hz", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
