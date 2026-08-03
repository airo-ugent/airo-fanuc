# SPDX-License-Identifier: Apache-2.0
"""Keep README.md's Python blocks executable against the current API.

The README is the landing page and its usage block is the first code a new user runs,
so a name that has been renamed or an argument that has become required is a defect in
the most expensive place. Nothing else checks it: the blocks are prose to pytest and
prose to ruff.

What is checked is what can be checked without a controller: every block parses, and
every name a block imports from ``airo_fanuc`` or from ``examples/`` actually resolves.
Executing a block is not possible — it opens sockets to a robot — so a wrong argument
name or a wrong value inside a call still gets past this. Renames and removed exports,
which is what actually happens, do not.
"""

from __future__ import annotations

import ast
import importlib
import pathlib
import re
import sys

import pytest

_README = pathlib.Path(__file__).resolve().parents[1] / "README.md"
_EXAMPLES = pathlib.Path(__file__).resolve().parents[1] / "examples"

#: ```python ... ``` fenced blocks, capturing the body.
_BLOCK_RE = re.compile(r"^```python\n(.*?)^```", re.MULTILINE | re.DOTALL)


def _blocks() -> list[str]:
    return _BLOCK_RE.findall(_README.read_text())


def test_readme_has_python_blocks() -> None:
    """Guard the guard: a regex that matches nothing makes every check below vacuous."""
    assert len(_blocks()) >= 1, "no ```python blocks found in README.md"


@pytest.mark.parametrize("src", _blocks(), ids=lambda s: s.strip().splitlines()[0][:40])
def test_block_parses(src: str) -> None:
    ast.parse(src)


@pytest.mark.parametrize("src", _blocks(), ids=lambda s: s.strip().splitlines()[0][:40])
def test_imported_names_exist(src: str) -> None:
    """Every ``from <mod> import <name>`` in a block must resolve for real.

    Covers both halves of the usage example: the package's own exports, and the
    ``examples/`` profile module it tells the reader to import — which is not
    importable by default, hence the sys.path insert, mirroring how an example script
    runs with its own directory on the path.
    """
    if str(_EXAMPLES) not in sys.path:
        sys.path.insert(0, str(_EXAMPLES))
    for node in ast.walk(ast.parse(src)):
        if not isinstance(node, ast.ImportFrom) or node.module is None:
            continue
        if node.module != "airo_fanuc" and not (_EXAMPLES / f"{node.module}.py").exists():
            continue  # third-party (numpy) or stdlib — not this test's business
        mod = importlib.import_module(node.module)
        missing = [a.name for a in node.names if not hasattr(mod, a.name)]
        assert not missing, f"README imports {missing} from {node.module}, which does not export them"
