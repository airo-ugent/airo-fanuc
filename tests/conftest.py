# SPDX-License-Identifier: Apache-2.0
"""Pytest bootstrap for the airo_fanuc test suite.

The suite runs against whichever ``airo_fanuc`` is importable, and only falls back to
the repository's ``src`` directory when none is. That order matters in both directions:
``uv sync --extra dev`` installs the package editable, so the working tree already *is*
what gets imported, while cibuildwheel runs this suite against a built wheel with the
repository mounted beside it — and unconditionally inserting ``src`` there would shadow
the wheel with a source tree carrying no compiled ``_core``, failing the wheel's own
test step over a wheel that was fine.

The suite is NOT hardware-free by being pure Python — it needs the built extension.
``airo_fanuc/__init__.py`` imports ``._core``, and several test modules import it
outright, so an install that compiled it is a precondition for collection. What makes
the suite hardware-free is ``airo_fanuc.testing``: an in-process controller emulation
the real C++ core is driven against.

Also defines :data:`TEST_PROFILE`, the arm the suite runs against. The package ships
no arm profile, so every :class:`~airo_fanuc.config.DriverConfig` in the suite needs
one; sourcing it from here keeps the whole suite on a single envelope.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

# Nothing installed to import: fall back to the working tree. tests/ -> repo root -> src/.
if importlib.util.find_spec("airo_fanuc") is None:
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
