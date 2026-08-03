# SPDX-License-Identifier: Apache-2.0
"""Pytest bootstrap for the airo_fanuc test suite.

Puts the repository's ``src`` directory on ``sys.path`` so ``import airo_fanuc...``
works WITHOUT installing the package. This matters because installing the
package builds the C++ extension (pybind11 core), which is not available in
the pure-Python test sandbox — the codec golden tests here import only
``airo_fanuc.testing.wire`` + ``airo_fanuc.controller_facts``, both pure
Python (stdlib + numpy).

Also defines :data:`TEST_PROFILE`, the arm the suite runs against. The package ships
no arm profile, so every :class:`~airo_fanuc.config.DriverConfig` in the suite needs
one; sourcing it from here keeps the whole suite on a single envelope.
"""

from __future__ import annotations

import sys
from pathlib import Path

# tests/ -> package root -> src/
_SRC = Path(__file__).resolve().parent.parent / "src"
if _SRC.is_dir():
    src_str = str(_SRC)
    if src_str not in sys.path:
        sys.path.insert(0, src_str)

from airo_fanuc.robot_profile import RobotProfile  # noqa: E402  (needs the sys.path above)

#: The arm the suite runs against. FakeCRX emulates our CRX-10iA/L, so this matches
#: the profile the examples use for it (``examples/crx10ial.py`` carries the
#: provenance of every number): velocities off the FANUC datasheet, acceleration and
#: jerk derived at 2× and 8×. Tests that assert on a limit should read it from here
#: rather than restating a number, so retuning the envelope cannot silently invalidate
#: an expectation.
TEST_PROFILE = RobotProfile.from_degrees(
    name="test-arm",
    model="FANUC CRX-10iA/L (test)",
    velocity_limits_deg_s=[120.0, 120.0, 180.0, 180.0, 180.0, 180.0],
    acceleration_limits_deg_s2=[240.0, 240.0, 360.0, 360.0, 360.0, 360.0],
    jerk_limits_deg_s3=[1920.0, 1920.0, 2880.0, 2880.0, 2880.0, 2880.0],
    # ±179.999 on J1/J2/J5, not the ±180 a datasheet rounds to: these mirror the
    # controller's own active limits as recorded in ``examples/crx10ial.py``, and that
    # file says outright that the 0.001° is the kind of difference a hand-transcribed
    # table loses. It had been lost here.
    position_limits_lower_deg=[-179.999, -179.999, -270.0, -190.0, -179.999, -225.0],
    position_limits_upper_deg=[179.999, 179.999, 270.0, 190.0, 179.999, 225.0],
    max_payload_kg=10.0,
    source="tests/conftest.py",
)
