# SPDX-License-Identifier: Apache-2.0
"""Pytest bootstrap for the airo_fanuc test suite.

Puts ``packages/airo_fanuc/src`` on ``sys.path`` so ``import airo_fanuc...``
works WITHOUT installing the package. This matters because installing the
package builds the C++ extension (pybind11 core), which is not available in
the pure-Python test sandbox — the L0 wire-oracle tests here import only
``airo_fanuc.testing.wire`` + ``airo_fanuc.controller_facts``, both pure
Python (stdlib + numpy).
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
