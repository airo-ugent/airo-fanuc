# SPDX-License-Identifier: Apache-2.0
"""airo_fanuc.testing — wire-protocol oracle + FakeCRXController.

Ships in the wheel, not just the source tree, so downstream code can be tested
against the emulator without a controller on the bench.

* :mod:`airo_fanuc.testing.wire` — byte-exact Stream Motion + RMI codec oracle.
* :class:`FakeCRXController` — strict-conformance software emulator of the FANUC
  CRX-10iA/L controller (Stream Motion UDP + RMI TCP + first-order-lag plant,
  with fault injection). The executable spec the C++ core + Python lifecycle are
  tested against: where the emulator's behavior model and a test disagree, the
  behavior model is normative.
"""

from airo_fanuc.testing.fake_crx import (
    FS_TYPE_EMBEDDED,
    FS_TYPE_UNAVAILABLE,
    ControllerState,
    FakeCRXConfig,
    FakeCRXController,
    FakeRmiServer,
    FakeStreamMotionServer,
)
from airo_fanuc.testing.fake_crx_sm import StrictConformanceError
from airo_fanuc.testing.plant import Clock, JointPlant, ManualClock, WallClock

__all__ = [
    "FakeCRXController",
    "FakeCRXConfig",
    "ControllerState",
    "FakeStreamMotionServer",
    "FakeRmiServer",
    "StrictConformanceError",
    "JointPlant",
    "ManualClock",
    "WallClock",
    "Clock",
    "FS_TYPE_EMBEDDED",
    "FS_TYPE_UNAVAILABLE",
]
