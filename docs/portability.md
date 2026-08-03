# Portability: what works on your robot

Be precise about this before you plan a bring-up. Four tiers, from portable to not portable
at all.

---

## Tier 1 — generic across FANUC

Stream Motion and RMI are FANUC's own protocols, not a collaborative-robot feature. What is
required is controller option **S636**, the External Control Package, which bundles J519
Stream Motion and R912 RMI. Default endpoints are UDP 60015 and TCP 16001, both
configurable.

`sm_version` is the version *requested* at handshake; the session then adopts whatever the
controller reports it can do. Either way **this driver sends no ForceSensorConfig packet**,
so no force block is ever streamed — on a v4 controller as much as a v3 one. `get_wrench()`
returns `None`, and a numeric `force_stop_n` is refused with a typed error rather than armed
as a silent no-op.

## Tier 2 — the controller class

The **8 ms interpolation period is an R-30iB fact**, not a cobot fact. That is FANUC's
mainstream controller class, so this covers many arms rather than only cobots — and a
different controller class has a different period (an R-50iA with the right option is
faster).

It is a config field, `DriverConfig.itp_s`, not something to edit in the package, and
bring-up **refuses to run if it disagrees with the period the controller reports** in its
GetCapability reply. Every per-tick quantity is scaled by it, so a mismatch would mean
limits silently wrong by that ratio. Pass `--itp-ms` in the examples, or `itp_s=` in code.

Also in this tier: the RMI error-code table, the alarm classes preflight treats as hard
blocks, and the TP program names the External Control Package installs.

## Tier 3 — injected, with no default: the arm

The whole motion envelope. `DriverConfig(profile=...)` is required and the package ships no
instance. See [configuration](configuration.md#robotprofile--the-arms-envelope) for how to
derive one for your arm — most of it comes out of the controller.

### What stays in the package, and how portable it really is

`controller_facts.py` holds this driver's own tuning, and it is worth being exact about
which parts survive a change of arm unedited:

**Scale factors — genuinely arm-independent**, because they are fractions of *your*
profile's limits rather than limits:

| | |
|---|---|
| `STOP_LIMIT_SCALE_VA` / `_J` | the brake's fraction of your v/a and jerk |
| `SLEW_FACTOR` | the per-tick clip, as a multiple of `v × itp_s` |
| `SERVO_LIMIT_SCALE` | the servo path's fraction of your limits |
| `MOVEJ_LIMIT_SCALE_A` / `_J` | what `move_j` plans under |

**Absolute quantities — this driver's tuning, and not scaled by your profile.** They are
degrees, degrees per second and milliseconds, so an arm much faster or much slower than a
CRX-10iA/L may want them revisited:

| | | Consequence if it does not suit your arm |
|---|---|---|
| `CAPTURE_TOL_DEG` | 5° | how far a submitted first knot may sit from the current commanded pose before the trajectory is refused |
| `CAPTURE_RATE_DEG_S` | 15°/s | the splice's velocity ceiling, and the SAFE_FOLLOW re-anchor rate. A trajectory whose first knot needs more velocity than this is refused |
| `RX_SILENCE_*`, `ANTIFLAP_DWELL_MS`, `QD_END_BLEND_MIN_MS` | ms | the RX-silence ladder and the fault-flap dwell |
| `SUPERVISOR_LOST_S` | 3 s | how long the Python heartbeat may lapse |
| `deviation_watchdog_deg` | 5° | chosen above the worst overrun measured on our arm, not measured on yours |
| `tracking_lag_s` | 25 ms | the servo lag measured on our controller |

The first two are the ones you will meet first, because they are what refuse a submission.
Both are single-sourced: changing the constant moves the refusal, the error message and the
core's own gate together.

## Tier 4a — baked in: six joints

`kNumJoints = 6` is a **C++ compile-time constant**, mirrored by a Python-side check that
rejects any other `ndof` at profile construction with a message saying so. Fine for almost
every FANUC arm, but it does exclude 7-axis arms and extended or positioner axes.

The reason is deliberate rather than incidental: the online trajectory generator's
degrees-of-freedom is a compile-time template parameter, and the tick engine's per-joint type
is a fixed-size `std::array`, so the whole 8 ms path is stack-allocated and
allocation-free. Making the joint count dynamic means giving that up on the tick. A
different joint count needs a rebuild, not a config change.

(The Stream Motion wire itself carries nine axis slots; the trailing three are zero-filled.)

## Tier 4b — the gripper

Not portable as shipped, but **parameterised** rather than hardcoded: which registers a
dispatcher watches and which values it understands is a `RegisterGripperProtocol` you can
supply. What is not portable is the Teach-Pendant program on the other end, which you write
and install. [The gripper document](gripper.md) is the specification for doing that.

The one thing no configuration can supply: the dispatcher lives on controller flash and is a
**site-installation prerequisite, not a Python dependency**. Without it, run with
`DriverPolicy(enable_gripper=False)`.

---

## Open questions, stated as such

Two facts this driver depends on are not settled, and both are flagged in the code rather
than smoothed over. Neither is a bug; both are things only your controller can answer.

### The acceleration/jerk question

`RobotProfile` velocity limits come from the controller. Acceleration and jerk are
**derivations** — 2× velocity and 8× acceleration, in both the hand-written CRX profile and
the probe's `--emit-profile` output.

Published accelerations for the same arm also exist inside this repo's own vendored
submodule, in the MoveIt configuration's joint limits, and those are **6-16× lower**, with
the velocities matching exactly.

The profile records why the two are not directly comparable rather than declaring one wrong:
those are *planning* limits, a profile a planner shapes trajectories to, while these are
*clamps* — the ceiling above which the core refuses to pass a command through. A looser
clamp than a planning limit is the expected relationship. Whether this particular gap is the
right size is unresolved, and the way to resolve it is to measure your controller, not to
pick one of the two numbers. `examples/sine_wave.py` is where you would do it: raise the
commanded speed and watch for slew clips, vibration, or a servo alarm.

One related caveat: **a planner feeding this driver should shape trajectories with a softer
jerk than the clamp** — roughly 3× acceleration rather than 8×. A collaborative-stop monitor
infers contact force from motor disturbance torque, so a sharp jerk ramp reads as a phantom
contact mid-transit. Jerk is the trip trigger; acceleration is not. That is also why
`move_j` plans at a fraction of the clamp rather than at it.

### The J2/J3 representation

A FANUC controller can report J3 absolutely, or relative to J2 — the "interlinked"
representation. **Which one a given plane serves is a controller configuration, not a
property of the arm**, and this driver reads joints on two planes that need not agree:
Stream Motion status packets, and RMI `FRC_ReadJointAngles`.

On our controller they disagree, measured at two poses 25° apart: the RMI plane reports J3
one J2 lower than Stream Motion does. The conversion is written, and **off by default**,
gated by a `controller_facts` flag — because guessing wrong injects a silent J2-sized error
into anything that consumes RMI joints.

This affects only the RMI-sourced receive and calibration path, not Stream Motion motion.
While the flag is off, `FanucReceiveInterface.capture_calibration_sample()` **hard-rejects**
RMI joints rather than folding a possibly-wrong angle into a calibration dataset.
`examples/verify_j2j3_coupling.py` settles it for your controller against the pendant's
displayed angles; then set the flag.

---

## A checklist for a new arm

1. `python -m airo_fanuc.controller_probe --ip <controller> --emit-profile` — read-only,
   safe against a live cell. Confirms S636, the P-level, the TP programs, and emits most of
   a profile.
2. Fill in acceleration and jerk, and record in `source` how you got them.
3. Check the interpolation period the probe and the bring-up report agree on; pass `itp_s`
   if it is not 8 ms.
4. Run the `--fake` pre-check, then the validation ladder in order —
   [`examples/README.md`](../examples/README.md). Step 5 checks your position limits against
   the arm itself, by hand-guiding it.
5. Decide the J2/J3 question with `verify_j2j3_coupling.py` if you will use the calibration
   path.
6. If you have a gripper, write its dispatcher against [gripper.md](gripper.md).

---

See also: [configuration](configuration.md), [safety](safety.md), and
`docs/controller-notes.md` for the measured behaviour of the controller these numbers came
from.
