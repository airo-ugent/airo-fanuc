# Hardware validation still owed before publishing

**Do not publish until this file is worked through.** Everything below needs the physical
CRX-10iA/L; none of it can be settled against the fake controller, and some of it is code
that has never executed on a real arm.

Two hardware sessions settled a great deal — see `docs/controller-notes.md` for what they
measured. What follows is what they did *not* settle, plus everything that landed after the
robot went back.

---

## 0. Before you touch the robot

Preconditions: controller in **AUTO**, drives powered, no active alarm, override **100%**,
and **nothing else talking to the controller**. An operator stays at the robot with the
E-stop in hand for every step.

The hardware-free pre-check must pass first — all nine, from `examples/README.md`:

```bash
uv sync --extra dev
uv run python examples/move_joints.py --fake
uv run python examples/move_j.py --fake
uv run python examples/move_j.py --fake --multi
uv run python examples/sine_wave.py --fake --period 4 --cycles 1
uv run python examples/sine_wave.py --fake --period 8 --cycles 1 --stop-after 2
uv run python examples/servo_stream.py --fake --period 4 --cycles 1
uv run python examples/verify_tcp_frame.py --fake
uv run python examples/verify_j2j3_coupling.py --fake
uv run python examples/verify_j2j3_coupling.py --fake --move
```

Then run the eight-step ladder **in order**. It is a ladder on purpose: each step assumes
the one before it passed.

---

## 1. Code that has never run on the robot

The 12 passing hardware runs exercised the knot-0 splice. Everything here landed
afterwards, so the ladder is not a re-run — it is a first run for these paths.

- [ ] **The capture handoff and its feasibility gate.** The splice's sub-tick terminal
      residue and the phase-exact resume. Watch for a velocity notch at the
      CAPTURE→TRAJECTORY seam. Covered by `test_capture_seam` in C++, by nothing on
      hardware. **Steps 1-3.**
- [ ] **The arrival-velocity gate term.** A trajectory whose first knot needs more velocity
      than the splice can reach is now refused rather than accepted and clipped. Confirm a
      legitimate submission is *not* refused: a plain sine start (which demands peak
      velocity at t=0) should still be refused above ~15°/s, and a raised cosine should
      still be accepted. **Step 3.**
- [ ] **`move_j`'s at-rest precondition.** It now raises rather than planning from a moving
      anchor. Confirm the normal path is unaffected, and that the refusal fires if you call
      it mid-motion. **Step 2b.**
- [ ] **The submitted-brake mode gate.** A submitted brake is now refused in the RX-silent
      park and while faulted. Not directly reachable from the public API — `stop_j()` takes
      a different path in — so this is a "nothing regressed" check rather than a test.
      **Steps 4 and 6.**
- [ ] **The servo force-guard disarm.** A `move_trajectory(force_stop_n=...)` followed by
      `servo_j` no longer leaves the trajectory's guard policing the stream. Inert on a v3
      controller (no force telemetry, so `force_stop_n` is refused outright) — record that
      it stays inert. **Step 7.**
- [ ] **The rejected-submission isolation.** A refused trajectory no longer disturbs the
      motion already in flight. Submit a deliberately out-of-window trajectory mid-motion
      and confirm the running one completes normally. **Step 3.**
- [ ] **The gripper protocol refactor.** The registers the worker writes now come from a
      protocol object rather than module constants. For the shipped Robotiq preset the
      values are identical and the existing tests pass untouched, but **the gripper path has
      not been run on hardware since.** Needs its own run with the tool mounted — see §3.

---

## 2. Questions only the robot can answer

Each of these ships with a documented safe default. None is a blocker for a *correct*
driver; all of them are things the documentation currently has to hedge about.

- [ ] **The acceleration and jerk clamps.** Our profile derives them as 2× velocity and 8×
      acceleration. The vendored MoveIt configuration publishes accelerations **6-16×
      lower**, with velocities matching exactly. Both are permissive enough that 63°/s ran
      clean, so nothing so far distinguishes them.
      **How:** step 3, raising *amplitude* at a long period rather than shortening the
      period (peak accel and jerk go as ω² and ω³). Watch **slew clips staying at zero** and
      no `CONTACT_STOP` in a clear cell. Decide by measurement, not by picking one of the two
      numbers. Documented in `docs/portability.md`.
- [ ] **E-stop continuation path A.** Whether a Stream Motion session can resume after an
      E-stop, or must be torn down and re-handshaked. **Inconclusive in both sessions**
      because the status feed was down each time. Needs a live session with `motion_possible`
      asserted — available after a power-cycle. Ships as the safe path (full re-handshake),
      which is now the only implemented one.
      **How:** step 6, with the stream confirmed up first.
- [ ] **Does a contact stop surface as `in_error`?** Never triggered on hardware — the
      ladder showed zero `CONTACT_STOP` up to 12°/s. The fault classification treats it as
      DEGRADED (controller-managed decel), which is wrong if the controller actually latches
      it as an error.
      **How:** a deliberate, gentle contact during step 3. Record which gate bits set.
- [ ] **The servo lag disagreement.** Session 1 measured 82-104 ms; session 2 measured
      121-139 ms. The within-session scatter exceeds the across-speed trend, so neither
      number is trustworthy, and both are 3-5× the recorded `tracking_lag_s` of 25 ms.
      **How:** log `q_cmd` and `q_meas` through a swept-speed move and cross-correlate,
      rather than reading the peak-lag figure the examples print. Noted as open in
      `docs/controller-notes.md` §1.9a.
- [ ] **SERVO path fidelity.** Under `--fake`, `qd_cmd` periodically reverses against a
      forward-moving target on a ~1.3 s cycle that does **not** scale with target rate.
      `move_trajectory` shows none of it. This may be an artefact of the fake's plant or a
      real servo-planner behaviour; the fake cannot settle it either way.
      **How:** step 7. `servo_stream.py` already prints `commanded`, `plan error` and
      `send spacing` every run — nothing asserts them, so read them. `commanded` well above
      1.0×, or a *cyclic* rather than steady `plan error`, is the signature.
- [ ] **`get_flange_pose()` vs the active UTOOL.** Whether the streamed Cartesian block
      tracks the active tool offset or reports the faceplate regardless. The docstring says
      "open" and `docs/controller-notes.md` §1.10a keeps it open.
      **How:** `verify_tcp_frame.py --move`, with a non-identity UTOOL set on the pendant.
- [ ] **The capture residue notch.** Whether the sub-tick residue at the seam is visible in
      the measured joint trace at speed.
      **How:** step 3 at the highest validated speed, then look at the logged trace across
      the handoff rather than at the summary numbers.

---

## 3. Never validated at all

- [ ] **The whole gripper path.** Every script in `examples/` runs with
      `enable_gripper=False`, so passing all eight steps says nothing about it. Needs its own
      run once the tool is mounted: bring-up with `enable_gripper=True`, the dispatcher
      liveliness probe, open and close at each bucket, and the timeout path (stop the
      dispatcher and confirm a clean `{"success": False}` rather than a hang).
      Expect to need `connect_retries > 1`, which is why the default is 3.
- [ ] **The J2/J3 representation on RMI reads.** Measured on our controller — RMI J3 reads
      one J2 lower than Stream Motion, at two poses 25° apart, residual 0.0000° — but the
      conversion ships **off by default** because whether a controller serves that
      representation is per-installation. Confirm against the pendant's displayed angles
      before turning the flag on.
      **How:** `verify_j2j3_coupling.py --move`.
- [ ] **Recovery from a collision-induced controller SystemFault.** A different path from
      the E-stop drill: it can leave RMI unresponsive and forces the cold-reconnect
      escalation. Provoking it deliberately is not something to do casually — decide whether
      it is worth doing at all, and if not, say so in `docs/safety.md` rather than leaving
      it implied.

---

## 4. Settled — do not re-litigate

Recorded so a hardware session does not spend time re-deriving these:

- `FRC_Abort` terminates **neither** `STREAM_MOTN` nor a RUN-forked gripper dispatcher.
- The J2/J3 relation and its *form* are measured; only whether your installation serves it
  is per-site.
- `safety_scale` reads 1.0 at rest.
- The ARM gate's wording is justified by measurement: the E-stop arrived as `in_error` with
  `e_stopped=False` and `motion_possible=True`.
- Gripper bring-up needs `connect_retries > 1`.
- `MOTN-603 ST: Receiving interval over` **after** a run ends is benign teardown, not a
  missed deadline. During a run it is not.
- TP `RESET` has no authority in AUTO on this controller; the driver clears faults over RMI.

---

## 5. Before you tag a release

Not hardware, but in the same "has never actually run" category:

- [ ] **`.github/workflows/release.yml` has never executed.** Trigger it with
      `workflow_dispatch` first and confirm eight wheels build, `auditwheel` retags them as
      manylinux, and each passes its import and test step. The manylinux build was never
      exercised locally — no container runtime on the dev machine.
- [ ] **Configure PyPI trusted publishing** for this repository, the `Release` workflow and
      the `pypi` environment. Without it the publish step fails at the last moment.
- [ ] **Make the repository public**, or `https://github.com/airo-ugent/airo-fanuc.git` in
      the README 404s for everyone who reads it.
- [ ] Re-run the whole gate set on the final commit: `uv run pytest`, `ctest`, the TSAN
      build, ruff, mypy, and the nine `--fake` example runs.
