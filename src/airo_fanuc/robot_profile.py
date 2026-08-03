# SPDX-License-Identifier: Apache-2.0
"""The per-arm motion envelope, injected by the caller.

A :class:`RobotProfile` carries the facts that change when the robot *model* changes
— velocity, acceleration and jerk clamps plus the joint position limits — and
nothing else. This package ships the type and no instance: the numbers for any
particular arm are the caller's to supply, because this driver cannot verify them and
a default would be a set of limits someone else measured on a different robot.
``examples/crx10ial.py`` builds one for the FANUC CRX-10iA/L these examples run
against, and is the model to copy.

Everything else stays in the package because it is not a property of the arm:
:mod:`airo_fanuc.controller_facts` holds the controller-class protocol facts (the 8 ms
interpolation period, the Stream Motion dataStyle word) and this driver's own tuning
(brake scales, capture windows, watchdog dwells), all of which apply unchanged across
FANUC models.

Profiles are written in DEGREES via :meth:`RobotProfile.from_degrees` — the unit of
the datasheet and of the controller's own ``$PARAM_GROUP`` system variables, so a
value can be diffed against the pendant without arithmetic — and stored in RADIANS,
the unit the whole engine works in. The stored arrays are read-only: one profile is
shared by everything a driver builds from it, so an in-place edit would silently
change the clamps the RT core is already running.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Sequence

__all__ = ["SUPPORTED_NDOF", "ProfileError", "RobotProfile"]

#: Joint count the C++ core is compiled for (``tick_engine::kNumJoints``). The online
#: trajectory generator's DOF is a compile-time template parameter, so a profile with
#: a different ``ndof`` cannot be executed and is rejected on construction rather than
#: truncated to fit.
SUPPORTED_NDOF = 6


class ProfileError(ValueError):
    """A profile is malformed, or describes an arm this build cannot run."""


@dataclass(frozen=True)
class RobotProfile:
    """One arm's motion envelope, in radians.

    Build it with :meth:`from_degrees` (datasheet units) or directly in radians.
    Either way the vectors are validated and frozen: lengths must match
    :data:`SUPPORTED_NDOF`, the v/a/j clamps must be finite and positive (a zero clamp
    is a robot that cannot move, which is a config error rather than a safe default),
    and each joint's lower position limit must sit below its upper.

    The v/a/j values are *clamps* — the ceiling above which the RT core refuses to
    pass a command through — not a planning profile. They belong at or above what a
    planner shapes trajectories to, or a legitimate planned motion gets mangled on the
    tick; see the discussion in ``examples/crx10ial.py``, where a real arm's numbers
    and their provenance are worked through.
    """

    name: str
    model: str
    velocity_limits: np.ndarray
    acceleration_limits: np.ndarray
    jerk_limits: np.ndarray
    position_limits_lower: np.ndarray
    position_limits_upper: np.ndarray
    ndof: int = SUPPORTED_NDOF
    max_payload_kg: float | None = None
    #: Free-text provenance — where these numbers came from. Reported in the driver's
    #: STARTUP event, so a log records which envelope actually ran.
    source: str = "unspecified"

    _ARRAY_FIELDS = (
        "velocity_limits",
        "acceleration_limits",
        "jerk_limits",
        "position_limits_lower",
        "position_limits_upper",
    )

    def __post_init__(self) -> None:
        if self.ndof != SUPPORTED_NDOF:
            raise ProfileError(
                f"profile {self.name!r} declares ndof={self.ndof}, but this build's C++ core is "
                f"compiled for {SUPPORTED_NDOF} joints (tick_engine::kNumJoints). A different joint "
                f"count needs a rebuild, not a config change."
            )
        for attr in self._ARRAY_FIELDS:
            arr = np.asarray(getattr(self, attr), dtype=np.float64).reshape(-1)
            if arr.shape != (self.ndof,):
                raise ProfileError(
                    f"profile {self.name!r}: {attr} has {arr.size} values, expected {self.ndof}"
                )
            if not np.all(np.isfinite(arr)):
                raise ProfileError(f"profile {self.name!r}: {attr} contains non-finite values")
            arr.flags.writeable = False
            object.__setattr__(self, attr, arr)

        for attr in ("velocity_limits", "acceleration_limits", "jerk_limits"):
            arr = getattr(self, attr)
            if np.any(arr <= 0.0):
                bad = np.where(arr <= 0.0)[0].tolist()
                raise ProfileError(
                    f"profile {self.name!r}: {attr} must be positive on every joint; "
                    f"joint(s) {bad} are ≤ 0"
                )

        crossed = np.where(self.position_limits_lower >= self.position_limits_upper)[0].tolist()
        if crossed:
            raise ProfileError(
                f"profile {self.name!r}: position limits cross on joint(s) {crossed} — "
                f"lower must be below upper"
            )

    @classmethod
    def from_degrees(
        cls,
        *,
        name: str,
        model: str,
        velocity_limits_deg_s: Sequence[float],
        acceleration_limits_deg_s2: Sequence[float],
        jerk_limits_deg_s3: Sequence[float],
        position_limits_lower_deg: Sequence[float],
        position_limits_upper_deg: Sequence[float],
        ndof: int = SUPPORTED_NDOF,
        max_payload_kg: float | None = None,
        source: str = "unspecified",
    ) -> RobotProfile:
        """Build a profile from degree-based values, converting to radians.

        Keyword-only: five same-shaped vectors in a row are exactly the argument list
        where a positional swap is both easy to make and impossible to see.
        """
        return cls(
            name=name,
            model=model,
            velocity_limits=np.radians(np.asarray(velocity_limits_deg_s, dtype=np.float64)),
            acceleration_limits=np.radians(np.asarray(acceleration_limits_deg_s2, dtype=np.float64)),
            jerk_limits=np.radians(np.asarray(jerk_limits_deg_s3, dtype=np.float64)),
            position_limits_lower=np.radians(np.asarray(position_limits_lower_deg, dtype=np.float64)),
            position_limits_upper=np.radians(np.asarray(position_limits_upper_deg, dtype=np.float64)),
            ndof=ndof,
            max_payload_kg=max_payload_kg,
            source=source,
        )

    @property
    def position_limits_lower_deg(self) -> np.ndarray:
        """Lower joint position limits in degrees (the unit the pendant reports)."""
        return np.degrees(self.position_limits_lower)

    @property
    def position_limits_upper_deg(self) -> np.ndarray:
        """Upper joint position limits in degrees (the unit the pendant reports)."""
        return np.degrees(self.position_limits_upper)

    def describe(self) -> str:
        """One line for a log or a bring-up report."""
        v_deg = np.degrees(self.velocity_limits)
        return (
            f"{self.model} (profile {self.name!r}, {self.ndof} joints, "
            f"v≤{np.max(v_deg):.0f}°/s; {self.source})"
        )
