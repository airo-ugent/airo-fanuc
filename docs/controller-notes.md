# FANUC CRX-10iA/L controller notes — measured behaviour + hard-won detail

> The controller is reached over a **dedicated point-to-point Ethernet link**: the
> host and the controller are the only two nodes on the segment, so no third party
> can inject Stream Motion or RMI traffic and no other traffic competes with the
> 8 ms ITP. Both are on a private /24; the controller's address is a deployment
> detail, not a driver constant.
>
> Option **S636** (External Control Package = J519 Stream Motion + R912 RMI).
>
> **Everything below was measured on the physical controller (2026-07-06)** unless a
> row says otherwise. Two questions could not be settled on hardware and keep their
> safe defaults: the **e-stop continuation path** (§1.3 — inconclusive, the status
> stream was down) and the **J2/J3 representation** (§1.5 — deferred).

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
| Joint vel limits (deg/s and rad/s) | deg/s `[120,120,180,180,180,180]`; rad/s `[2.094,2.094,3.142,3.142,3.142,3.142]` — **exact match to `controller_facts.py`** ✓ | cross-check `controller_facts.py` |
| Joint pos limits (active vs URDF) | active lower `[-180,-180,-270,-190,-180,-225]` upper `[180,180,270,190,180,225]`; J1–J5 == URDF, **J6 URDF (±190) narrower than ctrl (±225)** = safe | SRVO-115 (flag URDF-wider) |
| DCS joint / Cartesian envelopes | joint DCS ±9999° (LIM_ENB=1, wide-open); Cartesian ±3000 mm (LIM_ENB=1, whole cell) — enabled but permissive | fallback if the §1.2 coast is ever judged insufficient (zone tightening) — **DCS is enabled, so tightening is viable** |
| Master position / max payload | MASTER_POS `[0, 55.21, -104.852, 0, -75.148, 0]`°; MAX_PAYLOAD **10.0 kg**; TRKERRLIM 524288; `$RMI_CFG.$DISCNT_TIM = 60 min` | reference |

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
> coast as the host-death backstop** and wires **in-process C++ RT-core guards**
> (`SUPERVISOR_LOST` + `DRIFT`, see `rt_core_config.hpp`) rather than an external
> RMI-abort watchdog: a watchdog process on the same host dies *with* the host and is
> slower than the coast even when it survives (see `docs/successor-invariants.md`
> "Host-death & liveness watchdogs"). DCS zone tightening (§1.1 envelope) remains
> available as a further mitigation but is not wired.

### 1.3 E-stop continuation

| Fact | Value |
|------|-------|
| SM session survives e-stop (status keeps flowing) | **Not proven** — status stream never came up during the measurement (STREAM_MOTN aborted during an earlier HOST-380 recovery); could not test path A |
| Sequence number resets on e-stop | N/A (no live stream at the time) |
| `motion_possible` re-asserts after release w/o new StartPacket | **Not observed** (stream down) |
| Path-B full re-handshake time (s) | ~4.82 s client start-sequence (of a non-streaming session; not a clean timing) |
| Alarm strings on TP e-stop (SRVO-001/002 exact text) | **SRVO-002 (Teach Pendant E-stop) + SRVO-289** (RMI FRC_ReadError, independent of SM) |
| `FRC_Continue` reply | **ErrorID 0 (OK)** — accepted (not 2556938) |
| **Chosen path** | **B (full SM re-handshake)** — the always-safe default; path A neither proven nor refuted. Re-test A when a live `motion_possible` session is available. |

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
| Verdict: identical / RMI applies `J3 += J2` / other | **DEFERRED — not run on hardware.** Single RMI session + single SM peer + AUTO-only (no T1) + RMI-init locks hand-guidance made a clean simultaneous stream-vs-RMI capture impractical. |
| Conversion formula (if any) | Unverified. Vendor-behavior default (`J3 += J2` on RMI) retained but UNCONFIRMED. |
| ε achieved across the J2 sweep (deg) | N/A (deferred) |

**Rule:** RMI-sourced joints stay tagged `rmi_unconverted`; calibration **hard-rejects**
them. Never mix RMI and stream joints. Resolve definitively when STREAM_MOTN and RMI can be
co-managed and the robot moved between poses.

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
> `docs/successor-invariants.md` (Force / grasping).

### 1.9 Servo lag

| Fact | Value |
|------|-------|
| Measured tracking lag (ms) | **25.0 ms** (xcorr; a later verify run gave 20 ms → ~20–25 ms) |
| `DriverConfig.tracking_lag_s` | **0.025** |
| Amplitude ratio | **1.000** (commanded 10.000° pk-pk, measured 9.997°) |
| Post-stop settle: overshoot / time-to-rest | **NOT captured** — the lag measurement samples during motion only (~1 post-end sample). During-motion tracking is clean. Settle defaults (0.5° / 2 °/s / 2 s) stand pending a dedicated move-then-observe capture. |

**Uses:** drift guard (`plan @ now − lag`), FakeCRX first-order-lag τ, C++ settle constants. The
drift guard predicts the measured pose as `commanded @ (now − tracking_lag_s)`, so an overstated
`tracking_lag_s` makes it over-predict: at roughly 4× the true lag a spurious DRIFT fault becomes
possible above ~57–61 °/s. Hence `tracking_lag_s = 0.025`, matching the measurement.

#### 1.9a OPEN: the observed command-to-report offset is ~3.4× `tracking_lag_s` (2026-07-30)

Every motion run on 2026-07-30 showed a steady `|q_cmd − q_meas|` far above what 25 ms predicts.
Dividing by the concurrent measured speed expresses it as a time, which is comparable across runs:

| Run | Peak speed | Peak \|q_cmd − q_meas\| | Implied offset |
|---|---|---|---|
| J6 +10° rest-to-rest | 3.74 °/s | 0.306° | 82 ms |
| ±5° sine, all joints | 3.45 °/s | 0.291° | 84 ms |
| ±10° sine, 6 s period | 11.38 °/s | 1.031° | 91 ms |

**~85 ms, stable across a 3.3× speed range** — proportional to speed, which is what a fixed delay
looks like and not what noise looks like.

This does **not** by itself contradict the 25 ms above: that figure is a *cross-correlation* of the
commanded and measured series, whereas this is the instantaneous offset between the setpoint for the
current tick and the most recent status packet, so it also contains the command→report pipeline
(command buffering, the controller's own status generation, and up to one ITP of packet age). The two
measure different things and ~60 ms of pipeline would reconcile them. Separating the two needs the
xcorr method on logged series, not this ratio.

**Why it matters anyway.** The drift guard compares `q_cmd @ (now − drift_lag_ticks)` against
`q_meas @ now`, with `drift_lag_ticks = round(0.025 / 0.008) = 3`. If the *empirical* offset is ~11
ticks, the guard's reference sits ~8 ticks ahead of what the measurement reflects, and that residual
counts as drift: `8 × 8 ms × v`. At the tested speeds it is negligible (0.7° at 11 °/s against
`DRIFT_FAULT_DEG = 10`), but at this arm's 120 °/s ceiling it is ~7.7° — inside 25% of a false DRIFT
fault, in the false-positive direction. **Resolve before running fast moves (>50 °/s):** log q_cmd
and q_meas through a swept-speed move, cross-correlate, and re-derive `tracking_lag_s` (and hence
`drift_lag_ticks`) from that. Until then the guard is a bound with less margin than its constants
imply, not a wrong bound.

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
  `[ TBD — not specifically re-exercised; no MOTN-607 observed on the measurement day ]`

### 2.5 Stream Motion daemon wedge → controller power-cycle
- Symptom: RMI healthy (servos ready, drives powered, No Error) but **UDP 60015 silent** — no
  GetCapability reply, no status packets, `motion_possible` never asserts. The startup triage
  (`Supervisor._triage`) detects this "RMI-healthy-but-SM-silent" branch and recommends a
  power-cycle. Often coincides with a flaky/unresponsive Teach Pendant.
- **Observed trigger:** a foreign/malformed SM StartPacket (HOST-380, §1.8) plus repeated
  FRC_Abort/Reset cycling wedged the SM daemon.
- **Fix (confirmed):** **power-cycle the controller**, then restart the driver process. After the
  power-cycle, `motion_possible=True` on the first bring-up. The SM-silent triage branch and this
  power-cycle guidance must stay in the driver — nothing short of a power-cycle cleared it.

### 2.6 TP RESET has no authority in AUTO on this CRX
- The TP RESET button did **not** clear alarms (HOST-380 system error, and SRVO-002/289
  e-stop). RMI `FRC_Reset` (and `FRC_Abort` + `FRC_Reset` for the paused-program / system-error case)
  cleared them immediately (`ServoReady` 0→1). Because this CRX is **permanently in AUTO** (no T1
  access), fault RESET must come via **RMI / remote**, not the pendant. RMI (port 16001) is ground truth.
- `2556955` = `FRC_Initialize` rejected when `ServoReady=0` (robot not ready / servos off).

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

### 4.2 Post-bring-up `motion_possible` transient (measured 2026-07-30)

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
- **Reproducible on re-bring-up, absent on the first**: 3 consecutive re-connects showed it; the
  first bring-up after the robot had been powered on and left idle did not. The mechanism is not
  measured — that `STREAM_MOTN` state carries across sessions is the hypothesis the pattern fits,
  not something confirmed.
- **Consequence for callers:** wait for streaming to *hold* before commanding rather than trusting
  the first post-bring-up sample. Both example scripts do this (`_wait_streaming`, 2 s of stable
  streaming), and any consumer should. A validation check that reads the first sample will report
  this as a fault.

---

## 5. Firmware update — deferred decision

Software is V9.40/P82 (WARN band). A P82 → ≥P84 update would clear the vibration WARN and *may* also
enable SM v4 (type-204 native force — currently unavailable, see §1.8). It is a FANUC-channel operation
(FANUC BeneLux; software image + controlled-start version-up + full image backup + possible re-master),
not doable remotely. **Deferred** (operator decision 2026-07-06): the measured TX-silence backstop
(§1.2) and servo lag (§1.9) are both workable as they stand, so P84 buys only the vibration-WARN
clearance and possible v4 force; order the P84 media **and release notes** through FANUC BeneLux
(same channel as the J519 manual) if that changes.
