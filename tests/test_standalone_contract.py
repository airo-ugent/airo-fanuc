"""Guard the standalone-distribution contract that keeps this package upstreamable.

``airo_fanuc`` is intended to be contributed to airo-mono
(https://github.com/airo-ugent/airo-mono) as a FANUC CRX manipulator alongside the
other ``airo_robots.manipulators.hardware`` drivers. That only stays cheap while the
package installs and imports with **numpy alone**: no messaging middleware, no
third-party logging shim, no planner or tensor stack, and — above all — nothing from
whatever application happens to be deployed on top of the driver. The enforced list
is :data:`_FORBIDDEN` below.

Declared deps and docstrings cannot hold that line on their own: they say
``numpy>=1.26`` and "numpy only", but a single convenience
``from loguru import logger`` breaks the contract silently, and the failure would
surface only at upstreaming time.

These are static (AST) checks on purpose — they cannot be satisfied by an
import that merely happens to be installed in the dev venv.
"""

from __future__ import annotations

import ast
import pathlib
import sys

import pytest

_PKG_ROOT = pathlib.Path(__file__).resolve().parents[1] / "src" / "airo_fanuc"

#: The only third-party runtime import the distribution is allowed to have.
#: Keep in sync with ``[project] dependencies`` in pyproject.toml.
_ALLOWED_THIRD_PARTY = frozenset({"numpy"})

#: Importing any of these would tie the distribution to a consumer application or to
#: a heavyweight optional stack, and break the numpy-alone install.
_FORBIDDEN = ("grocery_bot", "zenoh", "loguru", "curobo", "torch", "airo_robots")


def _source_files() -> list[pathlib.Path]:
    return sorted(_PKG_ROOT.rglob("*.py"))


def _imported_top_level_modules(path: pathlib.Path) -> set[str]:
    """Top-level module names imported by *path*, including inside functions."""
    names: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text(), filename=str(path))):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.add(node.module.split(".")[0])
    return names


def test_source_tree_is_discoverable() -> None:
    """Guard the guard: a bad path would make every check below vacuous."""
    files = _source_files()
    assert len(files) > 5, f"only found {len(files)} source files under {_PKG_ROOT}"


@pytest.mark.parametrize("path", _source_files(), ids=lambda p: p.name)
def test_no_forbidden_imports(path: pathlib.Path) -> None:
    """No module may import a consumer application or a heavyweight optional stack."""
    imported = _imported_top_level_modules(path)
    offenders = sorted(imported.intersection(_FORBIDDEN))
    assert not offenders, (
        f"{path.name} imports {offenders}, which breaks the standalone contract. "
        "airo_fanuc must install and import with numpy alone so it can be "
        "upstreamed to airo-mono; use stdlib logging or dependency injection."
    )


def test_only_numpy_is_imported_from_outside_the_stdlib() -> None:
    """The whole package's third-party surface must equal its declared deps."""
    stdlib = set(sys.stdlib_module_names)
    third_party: dict[str, list[str]] = {}
    for path in _source_files():
        for name in _imported_top_level_modules(path):
            if name in stdlib or name == "airo_fanuc":
                continue
            third_party.setdefault(name, []).append(path.name)

    unexpected = {k: v for k, v in third_party.items() if k not in _ALLOWED_THIRD_PARTY}
    assert not unexpected, (
        "undeclared third-party imports found: "
        + "; ".join(f"{k} (in {', '.join(sorted(set(v)))})" for k, v in sorted(unexpected.items()))
        + f". Allowed: {sorted(_ALLOWED_THIRD_PARTY)}. Either vendor the behaviour, "
        "inject it, or add the dependency to pyproject.toml AND update this test."
    )
