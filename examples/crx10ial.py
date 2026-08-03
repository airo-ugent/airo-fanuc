# SPDX-License-Identifier: Apache-2.0
"""The FANUC CRX-10iA/L profile these examples run against.

This is the one arm the driver has been exercised on, and it lives here rather than in
the package because the package does not know which robot is attached: a
:class:`~airo_fanuc.robot_profile.RobotProfile` is injected by whoever does know.
Copy this file as the starting point for another arm — what to change is the numbers
and their provenance, not the shape.

**Most of this file does not have to be written by hand.** The controller reports its
own active velocity and joint position limits, and
:mod:`airo_fanuc.controller_probe` reads them::

    python -m airo_fanuc.controller_probe --ip 192.168.1.100 --emit-profile

That prints exactly the call below, ready to paste, with the numbers taken from
``$PARAM_GROUP`` rather than a datasheet. The velocity and position values here were
generated that way and then annotated. What the probe *cannot* supply is the
acceleration and jerk clamps — the controller publishes no equivalent — so those stay a
decision, made once here and explained below.

Keeping the result written down rather than probing at start-up buys three things: a
``--fake`` run works with no controller to ask, the numbers are reviewable in a diff,
and a run is reproducible against a controller whose settings someone has since
changed. ``DriverPolicy(preflight_full=True)`` closes the loop by re-reading the
controller at bring-up and warning if this file has drifted from it.

Every value is in DEGREES, the unit of the datasheet and of the controller's own
``$PARAM_GROUP`` system variables, so each one can be diffed against the pendant
without arithmetic. ``RobotProfile.from_degrees`` converts to radians on the way in.

The measurements referenced below were taken on our controller and are recorded in
``docs/controller-notes.md`` §1.1.
"""

from __future__ import annotations

from airo_fanuc.robot_profile import RobotProfile

# ---------------------------------------------------------------------------
# Velocity — read from the controller: $PARAM_GROUP[1].$JNTVELLIM.
#
# Identical to FANUC Europe datasheet MDS-04018 (CRX-10iA family), which specifies
# J1,J2 = 120°/s and J3-J6 = 180°/s. The controller's ACTIVE limits are the ones that
# bind, which is why these come from the box rather than the datasheet; the agreement
# is a bonus, not the source.
# ---------------------------------------------------------------------------
VELOCITY_LIMITS_DEG_S = [120.0, 120.0, 180.0, 180.0, 180.0, 180.0]

# ---------------------------------------------------------------------------
# Acceleration = 2× velocity (~1 s to reach max velocity, typical for cobots).
# Universal Robots' joint-health guidance recommends ≤300 °/s² and Franka Panda specs
# ~860 °/s², so these sit at the conservative end.
#
# FANUC publishes accelerations for this arm too, and they are much lower: the
# vendored MoveIt config
# (`vendor/fanuc_driver/fanuc_moveit_config/config/joint_limits.yaml`, headed "Joint
# limits for CRX-10iA and CRX-10iA/L") gives 0.4 rad/s² (≈23 °/s²) for J1-J3 and
# 1.0 rad/s² (≈57 °/s²) for J4-J6, with velocities identical to these — 6-16× below
# the values here. The two numbers are not the same kind of thing: FANUC's are
# *planning* limits, a target profile a planner shapes trajectories to, whereas a
# profile's are *clamps* — the ceiling above which the RT core refuses to pass a
# command through — so they are deliberately looser, to avoid silently mangling a
# legitimate planned motion.
#
# Whether that gap is the right size is an OPEN QUESTION: what this controller
# actually tolerates has not been measured, and if a measurement lands near FANUC's
# planning figures then these clamps are too permissive to be a useful net. Resolve it
# with hardware measurement, not by picking one of the two numbers.
# ---------------------------------------------------------------------------
ACCELERATION_LIMITS_DEG_S2 = [240.0, 240.0, 360.0, 360.0, 360.0, 360.0]

# ---------------------------------------------------------------------------
# Jerk = 8× acceleration (~0.125 s to max accel). Conservative; some aggressive cobot
# configs target ~33× accel.
#
# A planner feeding this driver should shape trajectories with a SOFTER jerk than the
# clamp here (~3× accel rather than 8×). The CRX collaborative-stop monitor infers
# contact force from motor disturbance torque, so a sharp jerk ramp reads as a phantom
# contact mid-transit. Jerk is the trip trigger; acceleration is not.
# ---------------------------------------------------------------------------
JERK_LIMITS_DEG_S3 = [1920.0, 1920.0, 2880.0, 2880.0, 2880.0, 2880.0]

# ---------------------------------------------------------------------------
# Joint position limits — read from the controller:
# $PARAM_GROUP[1].$LOWERLIMS / $UPPERLIMS. These are that controller's ACTIVE
# configuration rather than a property of the model, so they are worth re-reading on
# any new cell.
#
# Note ±179.999 on J1/J2/J5, not the ±180 a datasheet would round it to. That is what
# the controller actually enforces, and the 0.001° is the kind of difference a
# hand-transcribed table loses.
#
# §1.1 also records that the vendored URDF's J6 (±190°) is narrower than the
# controller's own (±225°). The controller's values are authoritative and are the ones
# here; a URDF *wider* than the controller is the dangerous direction, because a
# planner would emit poses the controller answers with SRVO-115.
# ---------------------------------------------------------------------------
POSITION_LIMITS_LOWER_DEG = [-179.999, -179.999, -270.0, -190.0, -179.999, -225.0]
POSITION_LIMITS_UPPER_DEG = [179.999, 179.999, 270.0, 190.0, 179.999, 225.0]

#: The profile the examples pass to ``DriverConfig``.
CRX10IAL: RobotProfile = RobotProfile.from_degrees(
    name="crx10ial",
    model="FANUC CRX-10iA/L",
    velocity_limits_deg_s=VELOCITY_LIMITS_DEG_S,
    acceleration_limits_deg_s2=ACCELERATION_LIMITS_DEG_S2,
    jerk_limits_deg_s3=JERK_LIMITS_DEG_S3,
    position_limits_lower_deg=POSITION_LIMITS_LOWER_DEG,
    position_limits_upper_deg=POSITION_LIMITS_UPPER_DEG,
    # $MRR_GRP[1].$MAX_PAYLOAD. Recorded for reference — the driver enforces nothing
    # against it; payload is set on the pendant, and the controller's own payload
    # monitor (SYST-348) is what acts on it.
    max_payload_kg=10.0,
    source=(
        "velocity + position limits read from the controller's $PARAM_GROUP "
        "(controller_probe --emit-profile, 2026-07-30); acceleration = 2x velocity and "
        "jerk = 8x acceleration derived, not measured"
    ),
)
