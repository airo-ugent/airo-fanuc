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
> row says otherwise; rows dated **2026-07-30** are from the first bring-up of the
> standalone driver. One question is still unsettled and keeps its safe default: the
> **e-stop continuation path** (§1.3 — inconclusive, the status stream was down). The
> **J2/J3 representation** (§1.5) is now measured, but its conversion stays disabled
> pending confirmation at a second J2 value. Open items raised on 2026-07-30 and not
> yet resolved: the **command-to-report offset** vs `tracking_lag_s` (§1.9a) and the
> **acceleration/jerk clamps** (see `examples/crx10ial.py`).

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
  does the FTP login banner (`[LR V9.40P/84]`). The gate bands on the order file's Deliver Ver,
  which is the value this section records; the probe reports the divergence rather than quietly
  taking the more flattering number.

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
| Verdict: identical / RMI applies `J3 += J2` / other | **MEASURED 2026-07-30 — RMI omits the J2 coupling that Stream Motion carries.** `RMI J3 = SM J3 − J2`, so the RMI→stream conversion is `J3 += J2`, the documented vendor default and sign. |
| Conversion formula | `q_stream[2] = q_rmi[2] + q_rmi[1]` |
| ε achieved (deg) | **0.0001** — the RMI wire quantization itself (§1.7). Every other joint agreed to 0.000. |

The capture §1.5 originally called impractical is available from inside a `FanucDriver`
session: the driver holds an initialized RMI session *and* the SM stream at once, so both
planes can be read back to back at a standstill pose with nothing commanded. Four
consecutive read pairs at one pose:

```
        J1        J2        J3        J4        J5        J6
SM      92.678     2.595    -1.380   -45.464   -27.230   -11.037
RMI     92.678     2.595    -3.975   -45.464   -27.230   -11.037
Δ       -0.000     0.000    -2.595     0.000     0.000    -0.000     ← Δ J3 = −(J2), to 0.0001°
```

**Still open, and why it is not yet enough to enable the conversion:**

1. **One pose, one J2 value** (2.595°). The offset equalling −J2 exactly is not plausibly a
   coincidence, but a second pose with a materially different J2 makes it airtight.
   `examples/check_joint_limits.py` prints the stream-frame value at its J3 extremes, so a
   hand-guided run at another J2 confirms it for free.
2. **Which plane matches the pendant** is unmeasured — this fixes the RMI↔stream
   relationship, not which one is "true J3". It does not matter for converting RMI into the
   stream frame (what the driver needs), but it does for anything trusting absolute J3.

**Rule (unchanged until 1 is done):** RMI-sourced joints stay tagged `rmi_unconverted`;
calibration **hard-rejects** them. The single conversion point is
`FanucReceiveInterface._apply_rmi_joint_policy`, whose disabled `q[2] += q[1]` line is now
confirmed correct in formula and sign — enable it together with the
`rmi_joints_identical_to_stream` gate, not before.

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
| Measured tracking lag (ms) | **25.0 ms** (xcorr; a later verify run gave 20 ms → ~20–25 ms) |
| `INTERIM_FACTS.tracking_lag_s` | **0.025** |
| Amplitude ratio | **1.000** (commanded 10.000° pk-pk, measured 9.997°) |
| Post-stop settle: overshoot / time-to-rest | **NOT captured** — the lag measurement samples during motion only (~1 post-end sample). During-motion tracking is clean. Settle defaults (0.5° / 2 °/s / 2 s) stand pending a dedicated move-then-observe capture. |

**Uses:** FakeCRX first-order-lag τ, and the figure the validation examples report a motion's peak
`|q_cmd − q_meas|` against. It gates nothing — no fault threshold is derived from it and it is not
mirrored into `DriverConfig`, so a wrong value costs accuracy in the fake and in a printed
comparison, not safety.

#### 1.9a OPEN: the observed command-to-report offset is ~3.4× `tracking_lag_s` (2026-07-30)

Every motion run on 2026-07-30 showed a steady `|q_cmd − q_meas|` far above what 25 ms predicts.
Dividing by the concurrent measured speed expresses it as a time, which is comparable across runs:

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

The metric is an *instantaneous* offset between the setpoint for the current tick and the most recent
status packet, so unlike the cross-correlation figure above it also contains the command→report
pipeline (command buffering, the controller's own status generation, up to one ITP of packet age).

**But the pipeline does not account for it.** The same metric against the FakeCRX, whose plant is a
first-order lag with τ set to exactly `INTERIM_FACTS.tracking_lag_s` = 25 ms, reads **29 ms** — so
this measurement over-reads a known 25 ms by only ~4 ms. Add the wire (ping RTT to this controller is
1–6 ms) and the pipeline plausibly explains ~30 ms of the ~85 ms, leaving **~50 ms unaccounted for**.
That is the part worth resolving, and it wants the xcorr method on logged series rather than this
ratio.

**What it costs.** Nothing in the driver is gated on this number (see §1.9 **Uses**), so the ~50 ms
is an accuracy question, not a safety one: the FakeCRX plant is τ = 25 ms where the real command→report
path measures ~85 ms, and the examples' printed lag comparison is correspondingly optimistic. **To
settle it:** log `q_cmd` and `q_meas` through a swept-speed move and cross-correlate, which separates
the servo lag from the status pipeline instead of lumping them as this ratio does.

### 1.10 The streamed Cartesian pose is NOT the TCP (measured 2026-07-30)

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

**The 175 mm is the cell's Robotiq gripper** (operator-confirmed, 2026-07-30). So the RMI read is
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

#### 1.10a OPEN: does the streamed pose track the *active* UTOOL? (2026-07-30)

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
- **Reproducible on re-bring-up, absent on the first**: 5 consecutive re-connects showed it; the
  first bring-up after the robot had been powered on and left idle did not.
- **Cause — this is the re-`FRC_Call` transient already documented in `_bringup_once`**, not a new
  phenomenon. `STREAM_MOTN` cannot be un-launched over RMI (§ the 2026-07-07 observation:
  `FRC_Abort` and `FRC_Reset` both leave `program_status=2`), and a pure SM handshake to an
  already-running instance does not re-arm `motion_possible` — so every bring-up re-Calls it, and
  the re-Call is what drops `motion_possible` briefly. Corroborated here: a read-only RMI probe
  taken between runs reported `program_status=2`, i.e. `STREAM_MOTN` still running from the
  previous session. This is why the first post-power-on bring-up is clean and every later one is not.
- **Consequence for callers:** wait for streaming to *hold* before commanding rather than trusting
  the first post-bring-up sample. Both example scripts do this (`_wait_streaming`, 2 s of stable
  streaming), and any consumer should. A validation check that reads the first sample will report
  this as a fault.
- **RESOLVED — bring-up now absorbs it** (`Supervisor._settle_stream_motn`, `bringup_settle_s`,
  default 2 s). It had been left to the fault path, which meant `recovery_count` was already 1 before
  the caller did anything and a policy with `auto_recover=False` was left FAULTED by an ordinary
  startup. Bring-up now requires `motion_possible` to *hold* for `bringup_settle_s` and re-applies
  the same relaunch step if it drops, so the constructor keeps its contract: it returns a robot that
  is commandable, not one about to fault.

  Verified on hardware 2026-07-30. One re-apply was enough, which is what the handover explanation
  predicts — the first `FRC_Call` lands while the previous instance is still live, the second sticks:

  ```
  lifecycle → sm_handshake
  motion_possible dropped within 2.0s of preroll (settle attempt 1/2) — re-applying the relaunch
  lifecycle → streaming
  bring-up complete (attempt 1)
  ```

  No `faulted` transition, `recovery_count` 0. Also verified with `auto_recover=False`: 6 s of stable
  streaming, `recovery_count` 0, where before the fix that configuration could not start at all.

- **Why the settle cannot be replaced by doing something up front.** The natural question is which
  step of the second application ends the previous instance, so it can be done first instead. There
  is no such step: both applications are the same four calls (`reset` → `Continue` → reseed →
  `FRC_Call`), the `FRC_Call` is already first, and per the 2026-07-07 measurement neither
  `FRC_Abort` nor `FRC_Reset` terminates STREAM_MOTN at all — only an operator at the pendant
  (FCTN → ABORT ALL) does.

  What the 2026-07-30 runs add: the **second Call never drops `motion_possible`** (every run needed
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
not doable remotely. **Deferred** (operator decision 2026-07-06): the measured TX-silence backstop
(§1.2) and servo lag (§1.9) are both workable as they stand, so P84 buys only the vibration-WARN
clearance and possible v4 force; order the P84 media **and release notes** through FANUC BeneLux
(same channel as the J519 manual) if that changes.
