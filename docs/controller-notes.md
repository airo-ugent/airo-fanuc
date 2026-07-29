# FANUC CRX-10iA/L controller notes (tribal knowledge + P-1 measured facts)

> Controller `192.168.1.100` on `eno2` (`192.168.1.101/24`, point-to-point).
> Option **S636** (External Control Package = J519 Stream Motion + R912 RMI).
>
> **Status: P-1 MEASURED (2026-07-06).** Values below were measured on the physical
> controller during the P-1 HIL probe day (`docs/reference/hil-probe-runbook.md`).
> Raw probe output archived under `p_minus_1/results/` (copied to `results/` next to this file).
> Two items are unresolved on hardware and retain their safe defaults: **E1** (e-stop path,
> inconclusive — SM stream was down) and **E3** (J2/J3, deferred — see §1.5).

---

## 1. P-1 measured facts (design-freeze gates)

### 1.1 Static facts — E5 (HIL-L9)

| Fact | Value | Gate / use |
|------|-------|------------|
| Controller software version / P-level | **V9.40/P82** (orderfile "Deliver Ver"); TP shows core fw V9.40P/84, boot monitor V9.40P/77, root V9.40534 | `>= V9.40P84` GO; `< P81` HARD FAIL → **WARN band (P82)**, not fail |
| Option S636 present | **PRESENT** ✓ | required |
| Options J519 / R912 present | Not separate order codes (bundled in S636); functional: `stream_motn.tp` + `rmi_move.tp` present, RMI + SM both work | required (bundled in S636) |
| RMI protocol Major.Minor (Connect_STMO) | **9.0** (redirect port 16002) | RMI version record |
| SM `available_version` / `sampling_rate_ms` | **v3 / 8 ms** (125 Hz). NOTE: v3 < 4 → **FSConfig(v4) N/A**; controller streams **type-202 (no force)** | version echo; 8 ms ITP |
| J2/J3 coupled envelope (`$JNT23_UPLIM/$JNT23_LOWLI`) | **0.0 / 0.0 → INACTIVE** | context for E3 |
| Joint vel limits (deg/s and rad/s) | deg/s `[120,120,180,180,180,180]`; rad/s `[2.094,2.094,3.142,3.142,3.142,3.142]` — **exact match to `controller_facts.py`** ✓ | cross-check `controller_facts.py` |
| Joint pos limits (active vs URDF) | active lower `[-180,-180,-270,-190,-180,-225]` upper `[180,180,270,190,180,225]`; J1–J5 == URDF, **J6 URDF (±190) narrower than ctrl (±225)** = safe | SRVO-115 (flag URDF-wider) |
| DCS joint / Cartesian envelopes | joint DCS ±9999° (LIM_ENB=1, wide-open); Cartesian ±3000 mm (LIM_ENB=1, whole cell) — enabled but permissive | E6 NO-GO fallback (zone tightening) — **DCS is enabled, so tightening is viable** |
| Master position / max payload | MASTER_POS `[0, 55.21, -104.852, 0, -75.148, 0]`°; MAX_PAYLOAD **10.0 kg**; TRKERRLIM 524288; `$RMI_CFG.$DISCNT_TIM = 60 min` | reference |

**E5 VERDICT: WARN** — S636 present (options GO), but system software P-level (V9.40/P82) is in the
P81–P83 vibration-risk WARN band, not the ≥P84 GO band. Not a hard fail (all ≥P81). Non-blocking.
The TP *pendant* firmware is P84 but that is a separate component. Firmware update deferred (see §5).

### 1.2 TX-silence backstop — E6 (H3 / HIL-L10) — **THE GO/NO-GO**

| Fact | Value |
|------|-------|
| **Backstop verdict** | **NO-GO / MARGINAL** — controller does NOT fast-decel within ≤3 ITPs on TX-silence |
| Decel onset after silence (ms / ITPs) | 15.3 °/s: none (abrupt, mp-drop 121 ms); **49.9 °/s: 86.7 ms = 10.8 ITPs** (mp-drop 71 ms) |
| Overrun at speed ~0.35 (deg + commanded vel) | **2.10°** @ 15.3 °/s |
| Overrun at speed 1.0 | **4.63°** @ 49.9 °/s |
| Alarm fired on silence (code/text from FRC_ReadError) | **None** (RMI ReadError = "No Error"); stop signalled by `motion_possible` drop |
| Resting behaviour (hold vs drift) | coasts at last commanded velocity ~120 ms (deviation-triggered) → abrupt hard-stop; **no runaway**; rest at 110–130 ms |
| If NO-GO: chosen fallback (ext RMI-abort watchdog / DCS tightening) | **external RMI-abort watchdog and/or DCS zone tightening (E5 envelope)** — REQUIRED |

**Decision gated:** the in-process design may **NOT** rely on the controller as a *fast* host-death
backstop. It **is** a bounded, deviation-triggered backstop (overrun sub-linear in speed: 2.10°→4.63°
for a 3.3× speed increase; the controller reacts *sooner* at higher speed as position error accrues
faster). `deviation_watchdog_deg ≈ 5.0°` is well-supported (worst overrun 4.63° @ 49.9 °/s). Caller-fed
deadman must be sized for ~120 ms of coast. Missed-packet tolerance empirically ~9–15 ITPs (71–121 ms);
cross-check against the J519 manual (§3).

> **Driver-side handling (decided at finalization).** The "external RMI-abort watchdog
> … REQUIRED" cell above records the *pre-finalization contingency*. Finalization
> **ACCEPTED the bounded ~120 ms deviation-triggered coast as the host-death backstop**
> and wired **in-process C++ RT-core guards** (`SUPERVISOR_LOST` + `DRIFT`, see
> `rt_core_config.hpp`) instead of an external RMI-abort watchdog: a same-host process
> dies *with* the host and is slower than the coast (see `docs/successor-invariants.md`
> "Host-death & liveness watchdogs"). DCS zone tightening (E5 envelope) remains available
> as a further mitigation but is not wired.

### 1.3 E-stop continuation — E1 (HIL-L1)

| Fact | Value |
|------|-------|
| SM session survives e-stop (status keeps flowing) | **Not proven** — status stream never came up during the probe (STREAM_MOTN aborted during an earlier HOST-380 recovery); could not test path A |
| Sequence number resets on e-stop | N/A (no live stream during probe) |
| `motion_possible` re-asserts after release w/o new StartPacket | **Not observed** (stream down) |
| Path-B full re-handshake time (s) | ~4.82 s client start-sequence (of a non-streaming session; not a clean timing) |
| Alarm strings on TP e-stop (SRVO-001/002 exact text) | **SRVO-002 (Teach Pendant E-stop) + SRVO-289** (RMI FRC_ReadError, independent of SM) |
| `FRC_Continue` reply | **ErrorID 0 (OK)** — accepted (not 2556938) |
| **Chosen path** | **B (full SM re-handshake)** — the always-safe default; path A neither proven nor refuted. Re-test A now that a live `motion_possible` session is available (post power-cycle). |

### 1.4 RMI single-session + ownership — E2 (HIL-L3)

| Fact | Value |
|------|-------|
| Single-session confirmed (concurrent → 2556954) | **Confirmed = True**, but **NOT via 2556954**: 2nd `FRC_Connect_STMO` succeeds (ErrorID 0, same redirect port 16002), then the redirect connect **TIMES OUT**. `concurrent_connect_errorid = None`. Driver must treat "already connected" as a **redirect-port TCP timeout**. |
| Reacquire after clean FRC_Disconnect (s) | **2.7 s** |
| Reacquire after SIGKILL (s) | **0.2 s** (kernel FIN frees session near-instantly) |
| Reacquire after SIGSTOP / hung holder (s) | **None within 180 s** — confirms hung-holder worst case up to `$discnt_tim` |
| `$rmi_cfg.$discnt_tim` (idle timeout) | **60 (minutes)** |

Mitigation: flock ownership + operator-facing "kill <PID>" hint + documented worst-case wait.

### 1.5 J2/J3 representation — E3 (HIL-L7 / H4)

| Fact | Value |
|------|-------|
| RMI joint-read command name + reply schema | **`FRC_ReadJointAngles`** → `{ErrorID, TimeTag:<int>, JointAngle:{J1..J9 deg}}`, 0.001° precision (discovered in E8) |
| Verdict: identical / RMI applies `J3 += J2` / other | **DEFERRED — not run on hardware.** Single RMI session + single SM peer + AUTO-only (no T1) + RMI-init locks hand-guidance made a clean simultaneous stream-vs-RMI capture impractical. |
| Conversion formula (if any) | Unverified. Vendor-behavior default (`J3 += J2` on RMI) retained but UNCONFIRMED. |
| ε achieved across the J2 sweep (deg) | N/A (deferred) |

**Rule (RETAINED):** RMI-sourced joints stay tagged `rmi_unconverted`; calibration **hard-rejects**
them. Never mix RMI and stream joints. Resolve definitively at the P2′/L3 C++ driver bring-up when
STREAM_MOTN + RMI can be co-managed and moved between poses.

### 1.6 RMI in T1 — E4 (HIL-L8)

| Fact | Value |
|------|-------|
| RMI reads succeed in T1 | **N/A — this CRX has no T1 access** (no AUTO/T1/T2 mode switch on browser iRProgrammer or physical TP; operates permanently in AUTO). Measured the equivalent in AUTO instead. |
| Reads track a live jog (not frozen) | **YES** — connect-only `FRC_ReadJointAngles` during hand-guided motion: **450/450 reads, 0 failures, 15.0 Hz, all distinct, TimeTag monotonic**, per-joint span up to J1=103°. Immune to the 2026-05-17 freeze. |
| `FRC_Initialize` succeeds fresh (else AUTO-first) | Succeeds in AUTO. (T1 case N/A.) Observed `2556955` once = Initialize rejected when `ServoReady=0` (servos momentarily off). |
| Sustained poll rate (Hz) | **15.0 Hz** |

**Key finding:** RMI `FRC_Initialize` **locks the motion group → disables TP hand-guidance**. The
calibration-while-moving read path MUST be **connect-only RMI** (no Initialize) or SM status — never
motion-initialized RMI. Manual motion on this robot = **hand-guidance in AUTO** (no T1).

### 1.7 RMI angle quantization — E8

| Fact | Value |
|------|-------|
| Read resolution (deg) | **0.001°** (`FRC_ReadJointAngles` 3-decimal format precision) |
| Worst instantaneous 15 Hz velocity noise (deg/s) | **0.0** (reads perfectly constant when dead-still; reads confirmed LIVE via advancing TimeTag) |
| Worst 0.5 s-LSQ velocity noise (deg/s) | 0.0 |
| Verdict: instantaneous OK / LSQ mandatory | **instantaneous OK** — LSQ NOT mandatory (worst-case 1-LSB flip = 0.015 °/s ≪ 0.1 °/s gate) |

### 1.8 Second SM peer — E7 (H9)

| Fact | Value |
|------|-------|
| Foreign StopPacket honored (kills session-A) | **YES** — session-A went silent (max_age 2955 ms). Cross-host Stop is dangerous → mitigation is the eno2 point-to-point network + flock (structural). |
| Foreign StartPacket reject observable (signal) | **YES** — TP raised **`HOST-380 "System error 0x30,0x0"`**; a rogue Start is detectable via that alarm. |
| Controller serves two SM peers | **No** (`controller_serves_two_peers = False`; the 2nd peer received no status). |

**⚠️ Incidental but significant (E7 + `packets.py:47-48`):** at negotiated SM **v3 the controller streams
type-202 status (NO force/torque)**; type-204 (force) requires **v4 + ForceSensorConfigPacket**, which
`available_version=3` cannot do. So **native external force in the Stream Motion feed is UNAVAILABLE** and
`RealRobot.linear_move_with_force_threshold`'s force watchdog is **inert**. This contradicts the prior
"type-204 provides external force natively" assumption and is a **third input to the firmware decision**
(getting type-204 likely needs a controller SW update and/or force option — confirm with FANUC).
**Also:** a foreign/malformed SM StartPacket (this probe) can **wedge the Stream Motion daemon** — see §2.5.

> **Driver-side handling (implemented at finalization).** The C++ RT core decodes
> **both** type-202 (388 B, this controller) and type-204 (416 B, a future v4/P84
> controller); at v3 `wrench_valid` is false so `get_wrench()` returns `None`.
> `move_trajectory(force_stop_n>0)` is **rejected** (`TrajectoryValidationError`) when
> force is unavailable, and grasps terminate on the collaborative **contact-stop**
> bit instead. See `docs/p5-grocery-integration.md` (grasp_execution) and
> `docs/successor-invariants.md` (Force / grasping).

### 1.9 Servo lag — E9 (H11)

| Fact | Value |
|------|-------|
| Measured tracking lag (ms) | **25.0 ms** (xcorr; a later verify run gave 20 ms → ~20–25 ms) |
| Recommended `DriverConfig.tracking_lag_s` | **0.025** (was 0.107 interim — a ~4× change) |
| Amplitude ratio | **1.000** (commanded 10.000° pk-pk, measured 9.997°) |
| Post-stop settle: overshoot / time-to-rest | **NOT captured** (measure_servo_lag records during-motion only; ~1 post-end sample). During-motion tracking clean. Executor settle defaults (0.5°/2°/s/2 s) retained pending a dedicated move-then-observe capture. |

**Uses:** drift guard (`plan @ now − lag`), FakeCRX first-order-lag τ, C++ settle constants. The 25 ms lag
vs the 107 ms assumption means the drift guard was over-predicting; `measure_servo_lag` warns a drift fault
becomes possible above ~57–61 °/s with the mismatch — hence the update to 0.025.

---

## 2. Recovery / fault procedures *(P-1 measured + interim)*

### 2.1 SYST-348 / SYST-328 stuck-state recovery
- Symptom: bridge `FRC_Initialize` fails with "Cannot Execute TP program"
  (ErrorID 2556934/2556936); `FRC_Call` rejected.
- Clear **SYST-348** via the TP **payload-confirmation** dialog — **not** by
  writing `$PLMR_GRP[1].PYLD_ENB`, and **not** via `$PMON_*`.
- Then resume the paused STREAM_MOTN with `FRC_Continue` over RMI.
- **Never jog during recovery** — jogging re-raises **SYST-328**.
- `[ TBD — exact TP menu path; not re-exercised at P-1 ]`

### 2.2 FRC_Continue semantics *(HIL-L4/L12 — partially from E1)*
- Invariant: **seq reseed is the FINAL step before any FRC_Call**.
- Issue FRC_Continue only after SAFE_FOLLOW has converged (commanded == measured).
- Tolerate ErrorID `2556938` ("TP Program is Not Paused") on an unpaused program.
- `motion_possible` is the only success oracle.
- **P-1 E1: `FRC_Continue` → ErrorID 0 (OK)** observed (accepted, not 2556938).

### 2.3 UI[2] *(P-1 confirmed)*
- `UI[2] = 0` is **FANUC-normal** on this SOP-less CRX; log-only, not a fault.
  **Confirmed at P-1: `UI[2]=0` and `UI[8]=0` throughout (all GetStatus reads).**

### 2.4 MOTN-607
- Raised on Stream Motion reconnect **without** a preceding StopPacket. The
  defensive StopPacket + drain in `StreamMotionClient.start()` mitigates it.
  `[ TBD — not specifically re-exercised at P-1; no MOTN-607 seen this day ]`

### 2.5 Stream Motion daemon wedge → controller power-cycle *(P-1 measured — NEW)*
- Symptom: RMI healthy (servos ready, drives powered, No Error) but **UDP 60015 silent** — no
  GetCapability reply, no status packets, `motion_possible` never asserts. The bridge's own triage
  (`motion_subprocess:_log_streammotion_failure_triage`) detects "RMI-healthy-but-SM-silent" and
  recommends a power-cycle. Often coincides with a flaky/unresponsive Teach Pendant.
- **Trigger observed at P-1:** an E7-style foreign/malformed SM StartPacket (HOST-380) plus repeated
  FRC_Abort/Reset cycling wedged the SM daemon.
- **Fix (confirmed):** **power-cycle the controller**, then `sudo systemctl reset-failed grocery-fanuc-bridge
  && sudo systemctl start grocery-fanuc-bridge`. After the power-cycle, `motion_possible=True` on the first
  bring-up. The new `airo_fanuc` driver MUST retain this SM-silent triage + power-cycle guidance.

### 2.6 TP RESET has no authority in AUTO on this CRX *(P-1 measured — NEW)*
- The TP RESET button did **not** clear alarms this session (HOST-380 system error, and SRVO-002/289
  e-stop). RMI `FRC_Reset` (and `FRC_Abort` + `FRC_Reset` for the paused-program / system-error case)
  cleared them immediately (`ServoReady` 0→1). Because this CRX is **permanently in AUTO** (no T1
  access), fault RESET must come via **RMI / remote**, not the pendant. RMI (port 16001) is ground truth.
- `2556955` = `FRC_Initialize` rejected when `ServoReady=0` (robot not ready / servos off).

---

## 3. Missed-packet tolerance (J519 manual) — **partially measured (E6)**

The exact "N missed ITPs → alarm" comes from the J519 Stream Motion manual (B-83734EN class, ordered at
P-1) and is characterised empirically by **E6**. Record both:
- J519 manual value: `[ TBD — manual on order, long lead ]`
- **E6 empirical decel-onset / motion_possible-drop:** deviation-triggered, ~9–15 ITPs
  (71 ms @ 49.9 °/s … 121 ms @ 15.3 °/s). The controller coasts at the last commanded velocity, then
  drops `motion_possible` and hard-stops; overrun 2.10° @ 15.3 °/s, 4.63° @ 49.9 °/s; no alarm.

---

## 4. Bring-up ordering (from the dries bridge — confirmed working at P-1)

`RMI connect → FRC_Reset → FRC_Call(GRPRUN) → sleep 0.2 → FRC_Reset →
full RMI reconnect → FRC_Call(STREAM_MOTN) → SM defensive-Stop+GetCapability+Start
→ (FSConfig if v≥4 — N/A here, v3) → preroll wait for motion_possible`.
`FRC_Call(STREAM_MOTN)` **must** precede the StartPacket or `motion_possible`
never asserts. `GRPRUN` is a one-shot RUN-fork launcher for `GRIPDISP` (never
`FRC_Call` GRIPDISP directly — it wedges the RMI queue).
- **P-1 confirmed:** this full ladder reaches `motion_possible=True` (post power-cycle). A reduced
  hand-rolled ladder (no GRPRUN / no double reconnect) did **not** reliably launch STREAM_MOTN streaming.
- **P-1 bench-setup note:** the host-local **`grocery-zenoh-router`** (tcp/127.0.0.1:7447) must be running
  before the bridge/executor, or the bridge dies with `zenoh.ZError: Unable to connect to [tcp/127.0.0.1:7447]`.
- **P-1 dev-bridge note:** after a chunk-starvation + publisher-restart re-anchor, the dev bridge's
  interpolator can get stuck at the frozen pose (no motion) until a bridge restart; the new driver's
  starvation-resume must actually resume tracking. (`probe_streaming_sine`'s per-tick HoldTarget did not
  drive motion with this executor at all — use TrajectoryTarget-based motion.)

---

## 5. Firmware update — deferred decision *(P-1)*

Software is V9.40/P82 (WARN band). A P82 → ≥P84 update would clear the vibration WARN and *may* also
enable SM v4 (type-204 native force — currently unavailable, see §1.8). It is a FANUC-channel operation
(FANUC BeneLux; software image + controlled-start version-up + full image backup + possible re-master),
not doable remotely. **Deferred** (operator decision 2026-07-06): let E6/E9 results inform whether P84 is
warranted; order the P84 media **and release notes** through FANUC BeneLux (same channel as the J519 manual).
