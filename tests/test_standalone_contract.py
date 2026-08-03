"""Guard the numpy-only contract, the package's most important distribution property.

``airo_fanuc`` installs and imports with **numpy alone**: no messaging middleware, no
third-party logging shim, no planner or tensor stack, and nothing from whatever
application is deployed on top of the driver. That is what lets any consumer adopt it
without inheriting a dependency stack, and what keeps the environment-specific parts —
a status sink, a collision checker, a kinematics provider — injected by the caller
rather than imported here. The enforced list is :data:`_FORBIDDEN` below.

Declared deps and docstrings cannot hold that line on their own: they say
``numpy>=1.26`` and "numpy only", but a single convenience
``from loguru import logger`` breaks the contract silently, and it would not surface
in a dev venv that happens to have loguru installed.

Static (AST) checks for exactly that reason — they cannot be satisfied by an import
that merely happens to be importable where the suite runs.
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
_FORBIDDEN = ("zenoh", "loguru", "curobo", "torch", "airo_robots")


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
        "airo_fanuc must install and import with numpy alone; use stdlib logging "
        "or dependency injection."
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


def test_declared_dependencies_match_the_allowed_set() -> None:
    """``pyproject.toml`` and this test's allow-list must not drift apart.

    Without this, adding a dependency to the manifest and forgetting to widen
    ``_ALLOWED_THIRD_PARTY`` leaves the check above silently passing on a package that
    no longer installs with numpy alone.
    """
    import re

    import tomllib

    root = pathlib.Path(__file__).resolve().parents[1]
    manifest = tomllib.loads((root / "pyproject.toml").read_text())
    declared = manifest["project"]["dependencies"]
    # Strip version specifiers / extras: "numpy>=1.26" -> "numpy".
    names = {re.split(r"[<>=!~\[; ]", spec, maxsplit=1)[0].strip().lower() for spec in declared}
    assert names == _ALLOWED_THIRD_PARTY, (
        f"pyproject declares {sorted(names)} but this test allows {sorted(_ALLOWED_THIRD_PARTY)}. "
        "The numpy-only contract is the package's most important distribution property — "
        "widening it is a deliberate decision, not a drive-by."
    )


def test_version_is_read_from_the_distribution_metadata() -> None:
    """``__version__`` must not be a second, drifting source of truth.

    A literal in ``__init__.py`` can disagree with ``[project] version`` indefinitely
    without anything failing: the wheel is published under the manifest's version while
    ``airo_fanuc.__version__`` reports the literal, and a user comparing the two has no
    way to tell which is authoritative. Deriving it from the installed metadata makes
    the manifest the only source; this pins that it stays so.
    """
    import tomllib

    import airo_fanuc

    root = pathlib.Path(__file__).resolve().parents[1]
    declared = tomllib.loads((root / "pyproject.toml").read_text())["project"]["version"]
    assert airo_fanuc.__version__ == declared, (
        f"__version__ is {airo_fanuc.__version__!r} but pyproject declares {declared!r}. "
        "Reinstall the package if this is a stale editable install; otherwise the literal "
        "has come back."
    )
