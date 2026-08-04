# FANUC CRX-10iA/L controller notes — measured behaviour + hard-won detail

> The controller is reached over a **dedicated point-to-point Ethernet link**: the
> host and the controller are the only two nodes on the segment, so no third party
> can inject Stream Motion or RMI traffic and no other traffic competes with the
> 8 ms ITP. Both are on a private /24; the controller's address is a deployment
> detail, not a driver constant.
>
> Option **S636** (External Control Package = J519 Stream Motion + R912 RMI).
>
> **Everything below was measured on the physical controller** unless a row says
> otherwise. Three questions this file used to leave open are now measured: the **e-stop
> continuation path** (§1.3 — the Stream Motion session survives an E-stop and resumes
> without a re-handshake), the **command-to-report offset** vs `tracking_lag_s` (§1.9a —
> duty-dependent over 84–180 ms, which is what made two sessions appear to disagree), and
> the **acceleration/jerk clamps** (§1.11 — the controller executes the profile's full
> derived clamp, measured on J6 only). The **J2/J3 representation** (§1.5) is measured
> including the form of the offset, and its conversion is written but off by default,
> because whether a controller serves that representation is a per-installation
> configuration. Still open: whether the streamed Cartesian block follows the active UTOOL
> (§1.10), and the clamps on **J1/J2**, which carry the arm's mass and were not swept.

---

## 1. Measured controller facts

### 1.1 Static facts

| Fact | Value | Gate / use |
|------|-------|------------|
| Controller software version / P-level | **V9.40/P82** (orderfile "Deliver Ver"); TP shows core fw V9.40P/84, boot monitor V9.40P/77, root V9.40534 | `>= V9.40P84` GO; `< P81` HARD FAIL → **WARN band (P82)**, not fail |
| Option S636 present | **PRESENT** ✓ | required |
| Options J519 / R912 present | Not separate order codes (bundled in S636); functional: `stream_motn.tp` + `rmi_move.tp` present, RMI + SM both work | required (bundled in S636) |
| RMI protocol Major.Minor (Connect_STMO) | **9.0** (redirect port 16002) | RMI version record |
| SM `available_version` / `sampling_rate_ms` | **v3 / 8 ms** (125 Hz). NOTE: v3 < 4 → **FSConfig(v4) N/A**; controller streams **type-202 (no force)** | version echo; 8 ms ITP |
| J2/J3 coupled envelope (`$JNT23_UPLIM/$JNT23_LOWLI`) | **0.0 / 0.0 → INACTIVE** | context for §1.5 |
| Joint vel limits (deg/s and rad/s) | deg/s `[120,120,180,180,180,180]`; rad/s `[2.0944,2.0944,3.1416,3.1416,3.1416,3.1416]` — **exact match to `examples/crx10ial.py`** ✓ | cross-check the `RobotProfile` in `examples/crx10ial.py` |
| Joint pos limits (active vs URDF) | active lower `[-179.999,-179.999,-270,-190,-179.999,-225]` upper `[179.999,179.999,270,190,179.999,225]` — note **±179.999, not ±180**, on J1/J2/J5; J1–J5 == URDF, **J6 URDF (±190) narrower than ctrl (±225)** = safe | SRVO-115 (flag URDF-wider) |
| DCS joint / Cartesian envelopes | joint DCS ±9999° (LIM_ENB=1, wide-open); Cartesian ±3000 mm (LIM_ENB=1, whole cell) — enabled but permissive | fallback if the §1.2 coast is ever judged insufficient (zone tightening) — **DCS is enabled, so tightening is viable** |
| Master position / max payload | MASTER_POS `[0, 55.21, -104.852, 0, -75.148, 0]`°; MAX_PAYLOAD **10.0 kg**; TRKERRLIM 524288; `$RMI_CFG.$DISCNT_TIM = 60 min` | reference |

#### 1.1a Where each of these comes from, and which are read automatically

Most of §1.1 does not need transcribing: the controller serves generated diagnostic and
variable dumps from its virtual `md:` device over FTP (anonymous, read-only), and
`airo_fanuc.controller_probe` parses them. `python -m airo_fanuc.controller_probe --ip <addr>`
prints the lot; `--emit-profile` prints a paste-ready `RobotProfile`.

| Fact | File on `md:` | Field |
|---|---|---|
| P-level ("Deliver Ver") | `orderfil.dat` | `!SOF Ref5: Deliver Ver` |
| Option list, S636 presence, model order code | `orderfil.dat` | `1A05B-…-S636 ! External Control Pkg` |
| Model name | `version.dg` | line after `Default Personality (from FD)` |
| Software edition / root / boot monitor / servo / DCS versions, serial | `version.dg` | labelled `Label : Value` lines |
| `stream_motn`, `rmi_move`, `gripdisp`, `grprun` installed | `md:` listing | `*.tp` names |
| Joint velocity limits | `symotn.va` | `$PARAM_GROUP[1].$JNTVELLIM` (deg/s, active) |
| Joint position limits | `symotn.va` | `$PARAM_GROUP[1].$LOWERLIMS` / `$UPPERLIMS` (deg, active) |
| Max payload | `symotn.va` | `$MRR_GRP[1].$MAX_PAYLOAD` |
| J2/J3 coupled envelope | `symotn.va` | `$PARAM_GROUP[1].$JNT23_UPLIM` / `$JNT23_LOWLI` |

Two details worth knowing:

* **`$PARAM_GROUP` is degrees and RW (the active limits, what binds); `$MRR_GRP` is the same
  three quantities in radians and RO (the model master).** The probe reads both and reports any
  disagreement rather than picking a side — on this controller they agree exactly.
* **The order file and `version.dg` disagree about the P-level** (V9.40/P82 vs V9.40P/84), and so
  does the FTP login banner (`[LR V9.40P/84]`). They describe different components — the order
  file's Deliver Ver is the system software, `version.dg` and the banner report the pendant
  firmware. The gate bands on Deliver Ver, which is the value this section records, and the
  probe reports all of them rather than picking one, because which component a P-level refers
  to is exactly what a single reported number loses.

**Still not on the controller in any form:** the **acceleration and jerk clamps**. Nothing here
publishes a clamp equivalent, so those remain a decision — derived from the measured velocity at
2× and 8×, recorded in `examples/crx10ial.py`, and still the open item below.

**Verdict: WARN** — S636 present (options GO), but system software P-level (V9.40/P82) is in the
P81–P83 vibration-risk WARN band, not the ≥P84 GO band. Not a hard fail (all ≥P81). Non-blocking.
The TP *pendant* firmware is P84 but that is a separate component. Firmware update deferred (see §5).

### 1.2 TX-silence backstop — what the controller does when commands stop

| Fact | Value |
|------|-------|
| **Backstop verdict** | **NOT a fast decel** — the controller does NOT decelerate within ≤3 ITPs on TX-silence |
| Decel onset after silence (ms / ITPs) | 15.3 °/s: none (abrupt, mp-drop 121 ms); **49.9 °/s: 86.7 ms = 10.8 ITPs** (mp-drop 71 ms) |
| Overrun at speed ~0.35 (deg + commanded vel) | **2.10°** @ 15.3 °/s |
| Overrun at speed 1.0 | **4.63°** @ 49.9 °/s |
| Alarm fired on silence (code/text from FRC_ReadError) | **None** (RMI ReadError = "No Error"); stop signalled by `motion_possible` drop |
| Resting behaviour (hold vs drift) | coasts at last commanded velocity ~120 ms (deviation-triggered) → abrupt hard-stop; **no runaway**; rest at 110–130 ms |
| Fallbacks available if the coast is judged insufficient | external RMI-abort watchdog and/or DCS zone tightening (§1.1 envelope) |

**Consequence for the design:** the driver may **NOT** rely on the controller as a *fast* host-death
backstop. It **is** a bounded, deviation-triggered backstop (overrun sub-linear in speed: 2.10°→4.63°
for a 3.3× speed increase; the controller reacts *sooner* at higher speed as position error accrues
faster). `deviation_watchdog_deg ≈ 5.0°` is well-supported (worst overrun 4.63° @ 49.9 °/s). Caller-fed
deadman must be sized for ~120 ms of coast. Missed-packet tolerance empirically ~9–15 ITPs (71–121 ms);
cross-check against the J519 manual (§3).

> **Driver-side handling.** The driver **accepts the bounded ~120 ms deviation-triggered
> coast as the host-death backstop** and wires an **in-process C++ RT-core guard**
> (`SUPERVISOR_LOST`, see `rt_core_config.hpp`) rather than an external RMI-abort
> watchdog: a watchdog process on the same host dies *with* the host and is slower than
> the coast even when it survives (see `docs/invariants.md` "Host-death &
> liveness watchdogs"). DCS zone tightening (§1.1 envelope) remains available as a
> further mitigation but is not wired.
>
> The same deviation monitor is what covers commanded↔measured divergence generally,
> not just on TX silence: there is **no host-side divergence guard** in the driver. An
> arm that stops following the stream accrues position error until the controller's own
> monitor hard-stops it and drops `motion_possible` / raises `in_error` — gates the C++
> tick reacts to within one ITP. Measured here at 71–121 ms with an overrun of 2.10°
> @ 15.3 °/s and 4.63° @ 49.9 °/s.

### 1.3 E-stop continuation

| Fact | Value |
|------|-------|
| SM session survives e-stop (status keeps flowing) | **YES, measured.** `rx_age_ms` held 0.17–0.52 ms across the whole latched period, worst 8.3 ms (one tick) over the entire drill |
| Sequence numbers across the fault | **No gaps** — `rx_seq_gaps=0` through fault, release and recovery |
| `motion_possible` re-asserts after release w/o a new StartPacket | **YES.** Recovery ran `faulted → recovering → streaming` in **1.25 s**, never entering `rmi_connecting` / `tp_launch` / `sm_handshake`; bring-up in the same run took ~11 s, so no handshake fits in the gap |
| What recovery actually does | The RMI ladder re-**calls** the STREAM_MOTN TP program (`FRC_Reset` → best-effort Continue → reseed → `FRC_Call`) and then `core.recover()`. The **UDP session is never torn down** — the controller keeps streaming status regardless of the TP program's state, which is why the feed survives |
| Alarm strings on TP e-stop (SRVO-001/002 exact text) | **SRVO-002 (Teach Pendant E-stop) + SRVO-289** (RMI FRC_ReadError, independent of SM) |
| `FRC_Continue` reply | **ErrorID 0 (OK)** — accepted (not 2556938) |
| **Chosen path** | **A works** — the session resumes without a re-handshake, and that is what the ladder does. The recovery refuses to start while the E-stop is still held ("cannot recover — e-stop is held; release it first"), so it waits for the operator by construction. |

**How the fault arrives matters, and it is not one event.** Sampling at 10 Hz through a deliberate
E-stop on a moving arm, the flags land in **three stages over ~500 ms**:

| t (s) | fault | `in_error` | `motion_possible` | `e_stopped` | mode |
|---|---|---|---|---|---|
| 12.52 | NONE | False | True | False | TRAJECTORY |
| **12.63** | **IN_ERROR** | **True** | **True** | **False** | SAFE_FOLLOW |
| 12.93 | MOTION_NOT_POSSIBLE | True | **False** | False | SAFE_FOLLOW |
| 13.13 | E_STOP | True | False | **True** | SAFE_FOLLOW |

This is the measurement the ARM gate's wording rests on: the E-stop first appears as `in_error`
with `e_stopped=False` and `motion_possible=True`. **A classifier watching only `e_stopped` would be
blind for 500 ms while the arm is already decelerating.** The driver catches it at the first stage,
110 ms after the last healthy tick. `motion_inhibited` went True on that same first faulted sample,
`safety_scale` held 1.00 throughout, `operator_required` stayed False, and the operator hint read
"Release the E-stop; then press RESET on the TP if the fault persists."

**Motion stays inhibited after a *successful* recovery.** With `recovery_count=1`,
`lifecycle_state=streaming`, `mode=HOLD`, `fault=NONE` and `in_error=False`, `motion_inhibited` was
still True and a `move_j` was refused with `RobotFaultedError: motion_inhibited — call arm() to
permit motion after a fault/e-stop recovery`. That is the case where breaking the gate would be
silent.

### 1.4 RMI single-session + ownership

| Fact | Value |
|------|-------|
| Single-session confirmed (concurrent → 2556954) | **Confirmed = True**, but **NOT via 2556954**: 2nd `FRC_Connect_STMO` succeeds (ErrorID 0, same redirect port 16002), then the redirect connect **TIMES OUT**. `concurrent_connect_errorid = None`. Driver must treat "already connected" as a **redirect-port TCP timeout**. |
| Reacquire after clean FRC_Disconnect (s) | **2.7 s** |
| Reacquire after SIGKILL (s) | **0.2 s** (kernel FIN frees session near-instantly) |
| Reacquire after SIGSTOP / hung holder (s) | **None within 180 s** — confirms hung-holder worst case up to `$discnt_tim` |
| `$rmi_cfg.$discnt_tim` (idle timeout) | **60 (minutes)** |

Mitigation: flock ownership + operator-facing "kill <PID>" hint + documented worst-case wait.

### 1.5 J2/J3 representation

| Fact | Value |
|------|-------|
| RMI joint-read command name + reply schema | **`FRC_ReadJointAngles`** → `{ErrorID, TimeTag:<int>, JointAngle:{J1..J9 deg}}`, 0.001° precision (see §1.7) |
| Verdict: identical / RMI applies `J3 += J2` / other | **MEASURED — not identical: RMI reports J3 one J2 BELOW the Stream Motion value.** `RMI J3 = SM J3 − J2`, so the RMI→stream conversion is `J3 += J2`. |
| Conversion formula | `q_stream[2] = q_rmi[2] + q_rmi[1]` |
| ε achieved (deg) | **0.0001** — the RMI wire quantization itself (§1.7). Every other joint agreed to 0.000. |
| Form of the offset: tracks J2 / fixed | **MEASURED — tracks J2.** Two standstill poses in one session, 25° apart in J2: residual **0.0000°** against "tracks J2", **24.9970°** against "fixed offset". Reproduced in a later session, again 0.0000° against 24.9970°. |
| Which plane does the **pendant** show? | **Both, and FANUC names the distinction.** The Position display in JOINT coordinates carries a `J3` field *and* one labelled **`J2/J3 interaction`**. At one pose: pendant `J3` = 1.843 against RMI's 1.849, and pendant `J2/J3 interaction` = −21.734 against Stream Motion's −21.734, exact to three decimals. |

A two-plane read at one pose is available from inside a `FanucDriver` session: the driver
holds an initialized RMI session *and* the SM stream at once, so both planes can be read
back to back at a standstill pose with nothing commanded. Four consecutive read pairs at
one pose:

```
        J1        J2        J3        J4        J5        J6
SM      92.678     2.595    -1.380   -45.464   -27.230   -11.037
RMI     92.678     2.595    -3.975   -45.464   -27.230   -11.037
Δ       -0.000     0.000    -2.595     0.000     0.000    -0.000     ← Δ J3 = −(J2), to 0.0001°
```

**The form of the offset, settled.** One pose could not distinguish "−J2" from "a fixed
−2.595° offset", because at J2 = 2.595° they predict the same number. `examples/verify_j2j3_coupling.py`
reads both planes at one standstill, moves J2, and reads both again. Two poses 25° apart:

```
              J2        J3 stream   J3 RMI     offset (RMI − stream)
pose A     +11.6950      7.7610     -3.9340         -11.6950
pose B     +36.6920      7.7570    -28.9350         -36.6920
Δ          +24.9970     -0.0040    -25.0010         -24.9970
```

The offset moved by exactly −ΔJ2: residual **0.0000°** for "tracks J2", **24.9970°** for
"fixed offset". Stream-plane J3 held still to 0.004° (gravity sag) while the RMI plane's J3
moved a full 25°, so the coupling is on the RMI plane. Every other joint agreed to 0.0000 at
both poses.

**Still open:** **which plane matches the pendant.** This fixes the RMI↔stream relationship,
not which one is "true J3". It does not matter for converting RMI into the stream frame (what
the driver needs), but it does for anything trusting absolute J3.

**Rule.** The conversion is written, in the single conversion point
`FanucReceiveInterface._apply_rmi_joint_policy`, and stays **off by default**: whether a
controller serves the coupled representation is a per-installation *configuration*, not a
property of the arm. While `rmi_to_stream_j3_plus_j2_verified` is False, RMI-sourced joints
stay tagged `rmi_unconverted` and calibration **hard-rejects** them. The asymmetry is
deliberate — off fails loud, on-where-it-does-not-apply is a silent J2-sized error in every
derived pose. Settle it for your own cell with `verify_j2j3_coupling.py --move`.

### 1.6 RMI reads during manual motion

| Fact | Value |
|------|-------|
| RMI reads succeed in T1 | **N/A — this CRX has no T1 access** (no AUTO/T1/T2 mode switch on browser iRProgrammer or physical TP; operates permanently in AUTO). Measured the equivalent in AUTO instead. |
| Reads track a live jog (not frozen) | **YES** — connect-only `FRC_ReadJointAngles` during hand-guided motion: **450/450 reads, 0 failures, 15.0 Hz, all distinct, TimeTag monotonic**, per-joint span up to J1=103°. Reads follow the motion; they never latch a frozen value. |
| `FRC_Initialize` succeeds fresh (else AUTO-first) | Succeeds in AUTO. (T1 case N/A.) Observed `2556955` once = Initialize rejected when `ServoReady=0` (servos momentarily off). |
| Sustained poll rate (Hz) | **15.0 Hz** |

**Key finding:** RMI `FRC_Initialize` **locks the motion group → disables TP hand-guidance**. The
calibration-while-moving read path MUST be **connect-only RMI** (no Initialize) or SM status — never
motion-initialized RMI. Manual motion on this robot = **hand-guidance in AUTO** (no T1).

### 1.7 RMI angle quantization

| Fact | Value |
|------|-------|
| Read resolution (deg) | **0.001°** (`FRC_ReadJointAngles` 3-decimal format precision) |
| Worst instantaneous 15 Hz velocity noise (deg/s) | **0.0** (reads perfectly constant when dead-still; reads confirmed LIVE via advancing TimeTag) |
| Worst 0.5 s-LSQ velocity noise (deg/s) | 0.0 |
| Verdict: instantaneous OK / LSQ mandatory | **instantaneous OK** — LSQ NOT mandatory (worst-case 1-LSB flip = 0.015 °/s ≪ 0.1 °/s gate) |

### 1.8 Second SM peer

| Fact | Value |
|------|-------|
| Foreign StopPacket honored (kills session-A) | **YES** — session-A went silent (max_age 2955 ms). A cross-host Stop is dangerous → mitigation is the point-to-point link + flock ownership (structural). |
| Foreign StartPacket reject observable (signal) | **YES** — TP raised **`HOST-380 "System error 0x30,0x0"`**; a rogue Start is detectable via that alarm. |
| Controller serves two SM peers | **No** (`controller_serves_two_peers = False`; the 2nd peer received no status). |

**⚠️ Incidental but significant:** at negotiated SM **v3 the controller streams type-202 status
(NO force/torque)**; type-204 (force) requires **v4 + ForceSensorConfigPacket**, which
`available_version=3` cannot negotiate. So **native external force in the Stream Motion feed is
UNAVAILABLE** on this controller and any force-threshold guard has no signal to act on. This is a
direct input to the firmware decision (§5): obtaining type-204 likely needs a controller SW update
and/or a force option — confirm with FANUC.
**Also:** a foreign/malformed SM StartPacket can **wedge the Stream Motion daemon** — see §2.5.

> **Driver-side handling.** The C++ RT core decodes **both** type-202 (388 B, this
> controller) and type-204 (416 B, a v4/P84 controller); at v3 `wrench_valid` is
> false so `get_wrench()` returns `None`. `move_trajectory(force_stop_n>0)` is
> **rejected** (`TrajectoryValidationError`) when force is unavailable, and grasps
> terminate on the collaborative **contact-stop** bit instead — see
> `docs/invariants.md` (Force / grasping).

### 1.9 Servo lag

| Fact | Value |
|------|-------|
| Measured command-to-report offset | **84–180 ms**, moving with recent duty — §1.9a has the series |
| `MEASURED_FACTS.tracking_lag_s` | **0.125** — the midpoint of that range: typical, not exact |
| An earlier 20–25 ms by the same xcorr method | **does not reproduce.** Per-tick xcorr on this controller does not go below ~84 ms in any duty state measured, and the ratio metric agrees with it to within 8 ms. Treat the low figure as superseded rather than as a second data point. |
| Amplitude ratio | **1.000** (commanded 10.000° pk-pk, measured 9.997°) |
| Post-stop settle: overshoot / time-to-rest | **NOT captured** — the lag measurement samples during motion only (~1 post-end sample). During-motion tracking is clean. Settle defaults (0.5° / 2 °/s / 2 s) stand pending a dedicated move-then-observe capture. |

**Uses:** FakeCRX first-order-lag τ, and the figure the validation examples report a motion's peak
`|q_cmd − q_meas|` against. It gates nothing — no fault threshold is derived from it and it is not
mirrored into `DriverConfig`, so a wrong value costs accuracy in the fake and in a printed
comparison, not safety.

#### 1.9a RESOLVED: the command-to-report offset is duty-dependent, 84–180 ms

**The offset is not a constant of this controller.** It rises with recent motion and falls back with
idleness, over slightly more than a 2× range — which is why two sessions measuring the same quantity
disagreed without either being wrong. Dividing the steady `|q_cmd − q_meas|` by the concurrent
measured speed expresses it as a time, comparable across runs:

| Run | Peak speed | Peak \|q_cmd − q_meas\| | Implied offset |
|---|---|---|---|
| J6 +10° rest-to-rest | 3.74 °/s | 0.306° | 82 ms |
| ±5° sine, all joints | 3.45 °/s | 0.291° | 84 ms |
| ±10° sine, 6 s period | 11.38 °/s | 1.031° | 91 ms |
| J6 +10°, then −10° (repeat) | 3.68 / 3.81 °/s | 0.376° / 0.396° | 102 / 104 ms |
| ±5° sine, all joints (repeat) | 3.68 °/s | 0.337° | 92 ms |
| ±10° sine, 6 s period (repeat) | 11.42 °/s | 1.120° | 98 ms |

**82–104 ms across seven runs spanning a 3.3× speed range** — proportional to speed, which is what a
fixed delay looks like and not what noise looks like.

A later session, on the same controller and arm, measured the same metric **higher**:

| Run | Peak speed | Implied offset |
|---|---|---|
| J6 +10° rest-to-rest, out / back | 3.92 / 3.90 °/s | 125 / 127 ms |
| `move_j` J6 +20° at 12 °/s | 12.15 °/s | 138 ms |
| `move_j` `--multi`, three joints | 12.07 °/s | 139 ms |
| ±5° sine, all joints, 10 s period | 3.62 °/s | 121 ms |
| ±10° sine, 6 s period | 10.55 °/s | 130 ms |
| ±10° sine, 6 s period (repeat) | 10.95 °/s | 127 ms |

**121–139 ms across six runs**, against 82–104 in the table above. The two sessions disagree, and no
fixed property of the servo explains it.

**What explains it is duty.** A third session measured the *same* motion — J6 raised cosine, 20 °/s,
two cycles, 20 ms knots — repeatedly, by cross-correlating per-tick logged `q_cmd` against `q_meas`
(the method §1.9 uses, not the ratio above):

| Preceding activity | xcorr offset | ticks |
|---|---|---|
| After a bring-up ladder | 123.6 ms | 15 |
| After an acceleration sweep and servo streaming | 163.9 ms | 20 |
| Immediately after a 12 s continuous move | **179.9 ms** | 22 |
| After ~4 minutes with the joints idle | **83.9 ms** | 10 |
| Immediately after the previous run | 91.9 ms | 11 |
| Immediately after that one | 99.9 ms | 12 |

Each successive run adds close to exactly **one interpolation period**, and rest returns it to the
floor. The steps are integer ticks (10 → 11 → 12 back to back; 15, 20, 22 earlier), which points at
pipeline buffering accumulating one ITP per motion session and draining when idle rather than at
motor heating — heating would drift continuously. The mechanism is not observable from outside; the
behaviour is.

Excluded as causes, each by measurement: **host load** (load average 0.10 on 28 cores, `tx interval`
p50 8.000 ms and `skipped_tick_windows` 0 in every run), **packet staleness** (`rx_age_ms` p50
0.32 ms early against 0.33 ms late — packets arrive equally fresh, so it is the *content* that lags,
not the delivery), **pose and gravity** (J6 at −1.758° measured 179.9 ms, and after rest at the same
pose 83.9 ms), and **controller alarms** (`alarms=['No Error']` throughout).

Two consequences. Session 1's 82 ms is a rested arm and session 2's 121–139 ms a worked one, so both
tables are correct as measurements of different duty states. And **speed dependence is not
established**: the 123.6 → 131.6 ms step across a 3× speed change came from two consecutive runs,
which is exactly the one-tick-per-run pattern.

**An independent corroboration of the second session's figure**, from a completely different
observable. `stop_j()` fired at 10.43 °/s took 0.507 s and 3.367° to reach standstill. The brake is
jerk-limited at these speeds — reaching `a_max` = 96 °/s² at `j` = 288 °/s³ would take 0.333 s and
shed 16 °/s, more than the 10.43 on hand, so the accel ramp is purely triangular — giving a
*commanded* stop of `2√(v/j)` = 0.381 s over `v·t/2` = 1.99°. Adding one 127 ms offset and the
1.395° of standing lag the arm had to catch up predicts 0.508 s and 3.39°, against 0.507 s and
3.367° measured. Two unrelated observables agreeing to ~1% is much stronger evidence for the
offset's magnitude than the ratio metric alone, and it also confirms the brake profile is running
at the `STOP_LIMIT_SCALE_J` clamps it is supposed to.

Reproduced in a later session with the offset measured that day: `stop_j()` fired at 10.62 °/s took
**0.525 s over 3.449°**, against `2√(v/j)` = 0.384 s plus a 128 ms offset = 0.512 s, and 2.039° of
commanded travel plus 1.359° of standing lag = 3.398° predicted. **2.5% on time, 1.5% on distance.**

The metric is an *instantaneous* offset between the setpoint for the current tick and the most recent
status packet, so unlike the cross-correlation figure above it also contains the command→report
pipeline (command buffering, the controller's own status generation, up to one ITP of packet age).

**The ratio metric is roughly honest about a known offset, but has a low-speed noise floor.** Against
the FakeCRX, whose plant is a first-order lag with τ set to `MEASURED_FACTS.tracking_lag_s`, it
over-reads that τ by only ~4 ms, and an independent per-tick cross-correlation of the same fake read
29.9 ms against a τ of 25 ms — so the method is sound. What it cannot do is separate the servo from
the status pipeline, and its floor dominates when the deviation is small: four measurements of one
identical 3.7 °/s command gave 71 / 86 / 102 / 87 ms, where the same method at 12 °/s repeated to
within 1 ms. It also samples every 250 ms and so misses the true peak — for one motion it printed
118 ms where a per-tick trace of the same run gave 131 ms. **Below ~10 °/s, prefer xcorr.**

**What it costs.** Nothing in the driver is gated on this number (see §1.9 **Uses**).
`tracking_lag_s` is the 125 ms midpoint of the measured range, so the FakeCRX plant represents a
typical arm rather than a rested or a worked one — no single τ can reproduce a quantity that moves,
and a fake calibrated to the floor would make every tracking comparison optimistic. The examples'
printed NOTE fires only past 2× the recorded value, i.e. 250 ms, which is outside the measured range:
it now marks a genuine anomaly instead of firing on every run.

### 1.10 The streamed Cartesian pose is NOT the TCP (measured)

The Stream Motion status packet carries a Cartesian pose (`position[9]`, XYZWPR + 3 ext, float32,
in the shared header ahead of the force fields — so it is present at v3/type-202, unlike the
wrench). It carries **no frame tag**. `examples/verify_tcp_frame.py` reads it along
`FRC_ReadCartesianPosition` (§2.3.14, which *does* name its UFRAME/UTOOL) at one standstill pose:

| Fact | Value |
|------|-------|
| Joints at the read | `[92.678, 2.595, -1.380, -45.464, -27.230, -11.037]`° |
| Stream Motion `position` | `X 20.166  Y 703.511  Z 504.091  W -115.210  P -26.319  R 35.132` |
| `FRC_ReadCartesianPosition` | `X -43.921  Y 852.017  Z 437.279  W -115.210  P -26.319  R 35.132` |
| Orientation difference | **exactly 0.000° on all three angles** |
| Position difference | **175.0001 mm**, aligned with the **tool Z axis to 0.000°** (0.0009 mm perpendicular residual) |
| `FRC_ReadCartesianPosition` Configuration | UFRAME **0**, UTOOL **1** |
| `FRC_GetStatus` | UFRAME **9**, UTOOL **10** ← *different numbers, same session, see 1.10a* |

**The two planes differ by a pure 175.000 mm translation along tool +Z and nothing else.** A
different UFRAME would rotate or translate the orientation too; it is bit-identical. So this is a
*tool* difference, not a frame difference: the streamed pose sits 175 mm short of the pose RMI
reports, i.e. **Stream Motion does not apply the tool offset that RMI applies.**

**The 175 mm is the cell's Robotiq gripper** (operator-confirmed). So the RMI read is
the gripper TCP and the Stream Motion stream is the faceplate. The driver exposes both, named for
what they are, and neither derives the other:

| Getter | Source | Cadence | Point |
|---|---|---|---|
| `get_tcp_pose()` | `FRC_ReadCartesianPosition` | one RMI round trip, **blocks** ~tens of ms | **tool tip**, the controller's own arithmetic with its active UTOOL |
| `get_flange_pose()` | Stream Motion status `position` | 125 Hz seqlock read, non-blocking | **faceplate**, same packet as `q_meas` |

Nothing about the tool is configured driver-side: `get_tcp_pose()` asks the control box, so changing
the UTOOL entry on the pendant changes what it returns, with no code change and nothing to keep in
sync. The driver therefore ships **no pose algebra at all** — the controller does the conversion, so
duplicating it would only add a second definition of the tool that could drift from the pendant's.
The cost is that this is the one getter on `FanucDriver` that blocks, so it does not belong in a
control loop; a caller who needs a TCP at tick rate applies its own tool transform to
`get_flange_pose()`, using the convention below.

**Bonus, free from the same measurement: the W/P/R convention is fixed-axis XYZ**, i.e.
`R = Rz(R)·Ry(P)·Rx(W)`. The 175 mm offset lies along that matrix's third column to 0.000°; under
the intrinsic `Rx·Ry·Rz` reading it is 4.92° off. A 175 mm lever arm resolves the convention to
better than a tenth of a degree, so this is settled, not assumed.

#### 1.10a OPEN: does the streamed pose track the *active* UTOOL?

Unresolved, and it decides how dangerous the field is. `FRC_GetStatus` reported UFRAME 9 / UTOOL 10
while `FRC_ReadCartesianPosition` reported UFRAME 0 / UTOOL 1 **in the same session** — so "the
active tool" is not one number on this controller, and two readings of it disagree. Two hypotheses
fit the data equally well:

* **(a) Stream Motion always reports the faceplate**, ignoring UTOOL entirely. Then the flange/TCP
  gap is fixed at whatever UTOOL 1 holds, and `get_flange_pose()` means one thing forever.
* **(b) Stream Motion applies the controller-active UTOOL** (10 per GetStatus), which happens to be
  empty. Then **the meaning of the streamed pose changes the moment someone switches UTOOL**, with
  nothing on the wire to signal it.

`get_tcp_pose()` is unaffected either way — it asks the controller, so it tracks the active tool by
construction. What is at risk is `get_flange_pose()` and any caller that composes a *fixed* tool
transform onto it: under (b) that transform silently stops matching reality.

**To resolve:** read UTOOL 1, UTOOL 10 and UFRAME 9 off the pendant (SETUP → Frames). If UTOOL 10 is
all zeros both hypotheses still fit, so the decisive test is *active*: change the active UTOOL at the
pendant and re-run `examples/verify_tcp_frame.py`. If the reported offset changes, it is (b). That
run also re-measures the offset, so it is the check to repeat after any end-effector change.

### 1.11 Acceleration and jerk — what the controller actually tolerates (measured on J6)

`examples/crx10ial.py` derives its acceleration clamp as 2× velocity and its jerk clamp as 8×
acceleration, while the vendored MoveIt configuration publishes accelerations 6–16× lower with
identical velocities. Nothing distinguished the two until this sweep, because both are permissive
enough that ordinary motion runs clean under either.

A raised cosine has peak velocity `A·ω`, acceleration `A·ω²` and jerk `A·ω³`, so choosing `ω`
separates acceleration from velocity instead of escalating both. On **J6**, two cycles each,
20 ms knots:

| Peak velocity | Peak acceleration | Peak jerk | Slew clips | Fault |
|---|---|---|---|---|
| 20 °/s | 20 °/s² | 20 °/s³ | 0 | none |
| 60 °/s | 60 °/s² | 60 °/s³ | 0 | none |
| 80 °/s | 120 °/s² | 180 °/s³ | 0 | none |
| 90 °/s | 180 °/s² | 360 °/s³ | 0 | none |
| 96 °/s | 240 °/s² | 600 °/s³ | 0 | none |
| **121 °/s** | **360 °/s² — the profile's own clamp** | **1080 °/s³** | **0** | **none** |

**The controller executes the full derived J6 clamp.** FANUC's published figure for J4–J6
(1.0 rad/s² ≈ 57 °/s²) is therefore a *planning* target, not a tolerance: it is 6.3× below what the
controller demonstrably runs without trimming a single commanded step and without a `CONTACT_STOP` in
a clear cell. So the derived clamps are not invalidated by the controller faulting earlier — it does
not fault.

**Scope, stated plainly: J6 only.** Wrist roll is the lowest-inertia joint. **J1/J2 carry the arm's
mass and have the widest gap to FANUC's numbers (240 against 23 °/s², ~10×), and were not swept.**
Do not read this table as a property of the arm. Jerk remains the trip trigger for the
collaborative-stop monitor, so the ≤3× acceleration guidance in `crx10ial.py` stands — the highest
jerk reached here, 1080 °/s³, is exactly 3× its acceleration.

---

## 2. Recovery / fault procedures

### 2.1 SYST-348 / SYST-328 stuck-state recovery
- Symptom: `FRC_Initialize` fails with "Cannot Execute TP program"
  (ErrorID 2556934/2556936); `FRC_Call` rejected.
- Clear **SYST-348** via the TP **payload-confirmation** dialog — **not** by
  writing `$PLMR_GRP[1].PYLD_ENB`, and **not** via `$PMON_*`.
- Then resume the paused STREAM_MOTN with `FRC_Continue` over RMI.
- **Never jog during recovery** — jogging re-raises **SYST-328**.
- `[ TBD — exact TP menu path not recorded ]`

### 2.2 FRC_Continue semantics
- Invariant: **seq reseed is the FINAL step before any FRC_Call**.
- Issue FRC_Continue only after SAFE_FOLLOW has converged (commanded == measured).
- Tolerate ErrorID `2556938` ("TP Program is Not Paused") on an unpaused program.
- `motion_possible` is the only success oracle.
- Measured: **`FRC_Continue` → ErrorID 0 (OK)** (accepted, not 2556938).

### 2.3 UI[2]
- `UI[2] = 0` is **FANUC-normal** on this SOP-less CRX; log-only, not a fault.
  **Measured: `UI[2]=0` and `UI[8]=0` throughout (all GetStatus reads).**

### 2.4 MOTN-607
- Raised on Stream Motion reconnect **without** a preceding StopPacket. The
  defensive StopPacket + drain in `StreamMotionClient.start()` mitigates it.
  `[ TBD — not specifically re-exercised; no MOTN-607 observed during the measurements ]`

### 2.5 Stream Motion daemon wedge → controller power-cycle
- Symptom: RMI healthy (servos ready, drives powered, No Error) but **UDP 60015 silent** — no
  GetCapability reply, no status packets, `motion_possible` never asserts. The startup triage
  (`Supervisor._triage`) detects this "RMI-healthy-but-SM-silent" branch and recommends a
  power-cycle. Often coincides with a flaky/unresponsive Teach Pendant.
- **Observed trigger:** a foreign/malformed SM StartPacket (HOST-380, §1.8) plus repeated
  FRC_Abort/Reset cycling wedged the SM daemon.
- **Observed trigger, second and easier to hit:** **aborting the STREAM_MOTN TP program from the
  pendant** — `FCTN > ABORT ALL` does this, because it aborts *every* task and not just the one you
  were aiming at. The controller pushes `FRC_SystemFault`; `FRC_Continue` is then refused with
  `ErrorID=2556939 (Cannot Resume TP Program.)`, the RMI ladder cannot re-arm `motion_possible`
  within its probe window, and `reconnect()`'s three bring-up attempts each `FRC_Call` and each fail
  with `program_status` 2 then 0. **Only RMI can launch STREAM_MOTN, the driver tries exactly that,
  and it does not come back.** The driver's triage names this case and the required action; nothing
  short of a power-cycle cleared it. If you need to stop only the gripper dispatcher, abort the
  `GRIPDISP` task specifically.
- **Fix (confirmed):** **power-cycle the controller**, then restart the driver process. After the
  power-cycle, `motion_possible=True` on the first bring-up. The SM-silent triage branch and this
  power-cycle guidance must stay in the driver — nothing short of a power-cycle cleared it.

### 2.6 TP RESET has no authority in AUTO on this CRX
- The TP RESET button did **not** clear alarms (HOST-380 system error, and SRVO-002/289
  e-stop). RMI `FRC_Reset` (and `FRC_Abort` + `FRC_Reset` for the paused-program / system-error case)
  cleared them immediately (`ServoReady` 0→1). Because this CRX is **permanently in AUTO** (no T1
  access), fault RESET must come via **RMI / remote**, not the pendant. RMI (port 16001) is ground truth.
- `2556955` = `FRC_Initialize` rejected when `ServoReady=0` (robot not ready / servos off).

### 2.7 `FRC_Abort` — what it does and does not terminate

Measured against a live, streaming session, with no motion in flight and auto-recovery
disabled so nothing could relaunch behind the measurement:

| | before `FRC_Abort` | after (10 samples over 10 s) |
|---|---|---|
| `program_status` | 0 | **2**, every sample |
| `rmi_motion_status` | 1 | 0 |
| `motion_possible` | True | False |
| SM status `rx_age` | 0.4 ms | **0.2–0.7 ms — the stream never stopped** |

- **It does not terminate `STREAM_MOTN`.** Motion is disarmed, but status keeps flowing at
  the full rate and `program_status` is *left* at 2. Re-arming took only the
  re-`FRC_Call` that every bring-up already issues: a plain bring-up straight afterwards
  reached STREAMING with `alarms=['No Error']` and needed no operator action.
- **It does not kill a RUN-forked `GRIPDISP`.** With a dispatcher forked, the register
  handshake still answered after the Abort (twice, in 0.27 s and 0.33 s — a live
  dispatcher's latency, not the probe's 6 s timeout), and the gripper still actuated.
  Clearing the fork took `FCTN → ABORT ALL` at the pendant.
- **So it is not the equivalent of `FCTN → ABORT ALL`**, and a RUN-fork outlives every
  RMI verb this driver has.

One caveat on the first point: continued *status* alone would not prove the TP program
survived, if status were served by something independent of it. It is the combination with
`program_status` being left at 2 — the signature §4.2 already relies on — that makes
"not terminated" the supported reading.

---

## 3. Missed-packet tolerance (J519 manual) — partially measured

The exact "N missed ITPs → alarm" comes from the J519 Stream Motion manual (B-83734EN class) and is
characterised empirically by the TX-silence measurement in §1.2. Record both:
- J519 manual value: `[ TBD — manual not on hand ]`
- **Empirical decel-onset / motion_possible-drop:** deviation-triggered, ~9–15 ITPs
  (71 ms @ 49.9 °/s … 121 ms @ 15.3 °/s). The controller coasts at the last commanded velocity, then
  drops `motion_possible` and hard-stops; overrun 2.10° @ 15.3 °/s, 4.63° @ 49.9 °/s; no alarm.

---

## 4. Bring-up ordering (confirmed working on hardware)

`RMI connect → FRC_Reset → FRC_Call(GRPRUN) → sleep 0.2 → FRC_Reset →
full RMI reconnect → FRC_Call(STREAM_MOTN) → SM defensive-Stop+GetCapability+Start
→ (FSConfig if v≥4 — N/A here, v3) → preroll wait for motion_possible`.
`FRC_Call(STREAM_MOTN)` **must** precede the StartPacket or `motion_possible`
never asserts. `GRPRUN` is a one-shot RUN-fork launcher for `GRIPDISP` (never
`FRC_Call` GRIPDISP directly — it wedges the RMI queue).
- **Confirmed:** this full ladder reaches `motion_possible=True` (post power-cycle). A reduced
  hand-rolled ladder (no GRPRUN / no double reconnect) did **not** reliably launch STREAM_MOTN
  streaming — every step above is load-bearing.

### 4.1 Streaming continuity — starvation must resume, not latch

A setpoint source that stalls and then resumes must resume *tracking*. An interpolator that
re-anchors to the frozen pose and stays there is indistinguishable from a healthy hold when viewed
from outside — status keeps flowing, no fault is raised — while the robot has silently stopped
following. Exercise starvation-resume with real trajectory motion: a per-tick hold target is not a
substitute, because holding a pose produces no motion to observe in the first place.

### 4.2 Post-bring-up `motion_possible` transient (measured)

On a bring-up over a controller that has recently had `STREAM_MOTN` launched, `motion_possible`
asserts, then **drops roughly 1 s after bring-up reports complete**, and the recovery ladder
restores it in under a second:

```
bring-up complete (attempt 1) → streaming (fault=none)
lifecycle → faulted (fault=motion_not_possible)
lifecycle → recovering → streaming
recovery complete → STREAMING (from MOTION_NOT_POSSIBLE)
```

Streaming was then stable for the rest of the run (12 s observed, `fault=none`, `rx_age` 0.1–0.5 ms).

- **No alarm accompanies it**: `FRC_ReadError` says `No Error`, and an RMI probe taken between runs
  reports the controller entirely healthy (`servo_ready=True`, `drives_powered=True`, AUTO,
  `override=100%`, `control_mode=LOCAL`).
- **Reproducible on re-bring-up, absent on the first**: 5 consecutive re-connects showed it; the
  first bring-up after the robot had been powered on and left idle did not.
- **Cause — this is the re-`FRC_Call` transient already documented in `_bringup_once`**, not a new
  phenomenon. `STREAM_MOTN` cannot be un-launched over RMI (§2.7: `FRC_Abort` and `FRC_Reset` both
  leave `program_status=2`), and a pure SM handshake to an already-running instance does not re-arm
  `motion_possible` — so every bring-up re-Calls it, and the re-Call is what drops
  `motion_possible` briefly. Corroborated here: a read-only RMI probe taken between runs reported
  `program_status=2`, i.e. `STREAM_MOTN` still running from the previous session. This is why the
  first post-power-on bring-up is clean and every later one is not.
- **Consequence for callers:** wait for streaming to *hold* before commanding rather than trusting
  the first post-bring-up sample. Both example scripts do this (`_wait_streaming`, 2 s of stable
  streaming), and any consumer should. A validation check that reads the first sample will report
  this as a fault.
- **RESOLVED — bring-up absorbs it** (`Supervisor._settle_stream_motn`, `bringup_settle_s`,
  default 2 s). The fault path is the wrong place for it: `recovery_count` would be 1 before the
  caller did anything, and a policy with `auto_recover=False` would be left FAULTED by an ordinary
  startup. Bring-up therefore requires `motion_possible` to *hold* for `bringup_settle_s` and
  re-applies the same relaunch step if it drops, so the constructor keeps its contract: it returns
  a robot that is commandable, not one about to fault.

  Verified on hardware. One re-apply was enough, which is what the handover explanation
  predicts — the first `FRC_Call` lands while the previous instance is still live, the second sticks:

  ```
  lifecycle → sm_handshake
  motion_possible dropped within 2.0s of preroll (settle attempt 1/2) — re-applying the relaunch
  lifecycle → streaming
  bring-up complete (attempt 1)
  ```

  No `faulted` transition, `recovery_count` 0. Also verified with `auto_recover=False`: 6 s of stable
  streaming, `recovery_count` 0 — a configuration with no fault path to fall back on, so the settle
  is what makes it startable at all.

- **Why the settle cannot be replaced by doing something up front.** The natural question is which
  step of the second application ends the previous instance, so it can be done first instead. There
  is no such step: both applications are the same four calls (`reset` → `Continue` → reseed →
  `FRC_Call`), the `FRC_Call` is already first, and neither `FRC_Abort` nor `FRC_Reset`
  terminates STREAM_MOTN at all — only an operator at the pendant (FCTN → ABORT ALL) does
  (§2.7).

  What the hardware runs add: the **second Call never drops `motion_possible`** (every run needed
  exactly one settle attempt) even though it too is issued while an instance is running — ours. So
  re-Calling per se is not what drops it; the drop is the *previous session's* instance being torn
  down, asynchronously, about a second after our first Call triggers it. That is a delay to outlast,
  not a step to reorder.

  Two options remain open, neither taken: (a) gate the settle on `FRC_GetStatus.ProgramStatus` — it
  read 2 with a prior instance live, and preflight already calls `get_status()`, so it would cost
  nothing — but the enum's clean-state value is unconfirmed (needs the first bring-up after a
  power-cycle) and it would only save the settle on that one bring-up per power cycle; (b) issue a
  deliberate sacrificial `FRC_Call` at the start of the ladder to move the disturbance ahead of the
  SM handshake — rejected, because it adds an `FRC_Call` to every bring-up and RMI churn is itself a
  documented SM-daemon wedge vector (§2.5).

---

## 5. Firmware update — deferred decision

Software is V9.40/P82 (WARN band). A P82 → ≥P84 update would clear the vibration WARN and *may* also
enable SM v4 (type-204 native force — currently unavailable, see §1.8). It is a FANUC-channel operation
(FANUC BeneLux; software image + controlled-start version-up + full image backup + possible re-master),
not doable remotely. **Deferred** (operator decision): the measured TX-silence backstop
(§1.2) and servo lag (§1.9) are both workable as they stand, so P84 buys only the vibration-WARN
clearance and possible v4 force; order the P84 media **and release notes** through FANUC BeneLux
(same channel as the J519 manual) if that changes.
