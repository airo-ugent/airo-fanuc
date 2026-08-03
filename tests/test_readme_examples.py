# SPDX-License-Identifier: Apache-2.0
"""Keep the Python blocks in README.md and docs/ executable against the current API.

The README's usage block is the first code a new user runs and the reference in ``docs/``
is what they read next, so a name that has been renamed or an export that has been removed
is a defect in the most expensive place. Nothing else checks either: the blocks are prose to
pytest and prose to ruff.

What is checked is what can be checked without a controller: every block parses, and every
name a block imports from ``airo_fanuc`` or from ``examples/`` actually resolves. Executing a
block is not possible — it opens sockets to a robot — so a wrong argument name or a wrong
value inside a call still gets past this. Renames and removed exports, which is what
actually happens, do not.
"""

from __future__ import annotations

import ast
import importlib
import pathlib
import re
import sys

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[1]
_EXAMPLES = _ROOT / "examples"
#: Every prose file that carries runnable Python: the landing page and the doc set.
_PROSE = [_ROOT / "README.md", *sorted((_ROOT / "docs").glob("*.md"))]

#: ```python ... ``` fenced blocks, capturing the body.
_BLOCK_RE = re.compile(r"^```python\n(.*?)^```", re.MULTILINE | re.DOTALL)


def _blocks() -> list[tuple[str, str]]:
    """(where, source) for every fenced Python block across the prose files."""
    found = []
    for path in _PROSE:
        for src in _BLOCK_RE.findall(path.read_text()):
            found.append((path.name, src))
    return found


def test_the_prose_files_carry_python_blocks() -> None:
    """Guard the guard: a regex that matches nothing makes every check below vacuous.

    Counted per file rather than in total, because a rename that silently stopped matching
    one file would otherwise hide behind the others' blocks.
    """
    assert _PROSE[0].name == "README.md"
    files = {where for where, _src in _blocks()}
    assert "README.md" in files, "no ```python blocks found in README.md"
    assert len(files) >= 3, f"only {sorted(files)} carry Python blocks — did a path change?"


_IDS = [f"{where}:{src.strip().splitlines()[0][:34]}" for where, src in _blocks()]


@pytest.mark.parametrize(("where", "src"), _blocks(), ids=_IDS)
def test_block_parses(where: str, src: str) -> None:
    ast.parse(src)


@pytest.mark.parametrize(("where", "src"), _blocks(), ids=_IDS)
def test_imported_names_exist(where: str, src: str) -> None:
    """Every ``from <mod> import <name>`` in a block must resolve for real.

    Covers both halves of the usage example: the package's own exports, and the
    ``examples/`` profile module the prose tells the reader to import — which is not
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
        assert not missing, f"{where} imports {missing} from {node.module}, which does not export them"
