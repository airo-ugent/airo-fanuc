# SPDX-License-Identifier: Apache-2.0
"""airo_fanuc — FANUC CRX-10iA/L driver: C++17 real-time core + ur_rtde-shaped Python API.

Plan of record: docs/reference/airo-fanuc/PLAN.md (FINAL PLAN v2).
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
    "RejectedStartMismatch",
    "RmiError",
    "RmiSessionDown",
    "RobotFaultedError",
    "SettlePolicy",
    "TrajectoryValidationError",
]
