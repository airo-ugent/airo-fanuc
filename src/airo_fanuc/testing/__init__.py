# SPDX-License-Identifier: Apache-2.0
"""airo_fanuc.testing — wire-protocol oracle + FakeCRXController (ships in the wheel; see PLAN.md D11).

* :mod:`airo_fanuc.testing.wire` — byte-exact Stream Motion + RMI codec oracle.
* :class:`FakeCRXController` — strict-conformance software emulator of the FANUC
  CRX-10iA/L controller (Stream Motion UDP + RMI TCP + first-order-lag plant,
  with fault injection). The executable spec the C++ core + Python lifecycle are
  tested against (PLAN.md §8 "L2 (FakeCRX): behavior-model spec is normative").
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
