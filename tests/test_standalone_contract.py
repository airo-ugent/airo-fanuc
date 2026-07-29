"""Guard the standalone-distribution contract that keeps this package upstreamable.

``airo_fanuc`` is intended to be contributed to airo-mono
(https://github.com/airo-ugent/airo-mono) as a FANUC CRX manipulator alongside
``airo_robots.manipulators.hardware.ur_rtde``. That only stays cheap while the
package installs and imports with **numpy alone** — no zenoh, no loguru, no
curobo, and above all no ``grocery_bot``.

Nothing enforced that before this test: the package's declared deps said
``numpy>=1.26`` and its docstrings said "numpy only (D14)", but a single
convenience ``from loguru import logger`` (the house style everywhere else in
the parent repo) would have broken the contract silently, and the failure would
only have surfaced at upstreaming time.

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

#: Importing any of these would tie the package to its current host repo.
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
    """No module may import the host repo or its heavyweight stack."""
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
