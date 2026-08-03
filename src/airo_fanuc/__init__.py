# SPDX-License-Identifier: Apache-2.0
"""airo_fanuc — FANUC driver: a C++17 real-time core behind a Python API.

The public surface is split the conventional way for a robot client library: a
*receive* half (state getters that never raise and never lie — every value comes
with its age) and a *control* half (motion commands), with each motion available
both blocking and non-blocking. :class:`FanucDriver` presents both halves on one
construct-and-go object and reaches through to its parts (``driver.core``,
``driver.rmi``, ``driver.gripper``).

The arm's motion envelope is not built in: a :class:`RobotProfile` carrying its
velocity, acceleration and jerk clamps and its joint position limits is injected
through :class:`DriverConfig`. ``examples/crx10ial.py`` builds one for the FANUC
CRX-10iA/L this driver has been run against.
"""

from importlib.metadata import PackageNotFoundError as _PackageNotFoundError
from importlib.metadata import version as _metadata_version

#: Re-exported from the extension because :meth:`FanucDriver.get_state` publishes
#: ``mode``, ``fault``, ``active_motion_status`` and the ``conditions`` bitmask as plain
#: integers, and :class:`LifecycleState` deliberately collapses
#: HOLD/CAPTURE/TRAJECTORY/SERVO/BRAKE into ``STREAMING`` — so without
#: these a caller cannot decode values the driver hands it, and the examples had to reach
#: into ``airo_fanuc._core`` to do it. One decoder each: ``Mode`` for ``mode``,
#: ``FaultReason`` for ``fault``, ``MotionStatus`` for ``active_motion_status``, and
#: ``Condition`` — bit flags, not ordinals — for ``conditions``.
#:
#: These are pybind11 enumerations and NOT ``int`` subclasses, so ``snap["mode"] ==
#: Mode.HOLD`` is always False. Convert first, and compare with ``==`` rather than ``is``:
#: ``Mode(int(snap["mode"])) == Mode.HOLD``.
from ._core import Condition, FaultReason, Mode, MotionStatus
from .config import DriverConfig, DriverPolicy, MotionResult, SettlePolicy
from .driver import FanucDriver, MotionHandle
from .exceptions import (
    CalibrationError,
    CalibrationSourceError,
    CalibrationVelocityUnavailable,
    FanucConnectionError,
    FanucError,
    FanucPreflightError,
    OwnershipError,
    RejectedStartMismatch,
    RmiError,
    RmiSessionDown,
    RobotFaultedError,
    TrajectoryValidationError,
)
from .lifecycle import LifecycleState
from .robot_profile import ProfileError, RobotProfile

#: Read from the installed distribution metadata so ``pyproject.toml`` stays the
#: single source of truth — a hardcoded literal here silently disagrees with the
#: version the wheel is published under. The fallback covers an uninstalled source
#: tree, where there is no metadata to read.
try:
    __version__ = _metadata_version("airo-fanuc")
except _PackageNotFoundError:  # pragma: no cover - only when not installed at all
    __version__ = "0.0.0+unknown"

__all__ = [
    "CalibrationError",
    "CalibrationSourceError",
    "CalibrationVelocityUnavailable",
    "Condition",
    "DriverConfig",
    "DriverPolicy",
    "FanucConnectionError",
    "FanucDriver",
    "FanucError",
    "FanucPreflightError",
    "FaultReason",
    "LifecycleState",
    "Mode",
    "MotionHandle",
    "MotionResult",
    "MotionStatus",
    "OwnershipError",
    "ProfileError",
    "RejectedStartMismatch",
    "RmiError",
    "RmiSessionDown",
    "RobotFaultedError",
    "RobotProfile",
    "SettlePolicy",
    "TrajectoryValidationError",
]
