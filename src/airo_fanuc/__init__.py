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

__version__ = "0.1.0.dev0"

__all__ = [
    "CalibrationError",
    "CalibrationSourceError",
    "CalibrationVelocityUnavailable",
    "DriverConfig",
    "DriverPolicy",
    "FanucConnectionError",
    "FanucDriver",
    "FanucError",
    "FanucPreflightError",
    "LifecycleState",
    "MotionHandle",
    "MotionResult",
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
