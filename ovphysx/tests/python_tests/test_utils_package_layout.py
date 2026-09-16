# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Guard the ovphysx.utils package's shape and its USD-free import path.

Two things live here because both must hold in an environment without `pxr`:
the package must not be shadowed by a module file, and the one submodule that
authors nothing must stay reachable with no USD installed at all.

The first is a filesystem check rather than an import check, so it keeps working
where importing the authoring surface would not.

"Shadowed" is the way round it reads from the module file's point of view. The
package directory wins every resolution, for CPython and for type checkers
alike: mypy documents its per-directory order as package, then `foo.pyi`, then
`foo.py`, and pyright's resolver looks for `foo/__init__.pyi` and then
`foo/__init__.py` before it ever considers a sibling `foo.pyi`. So the module
file is not a competitor that might win; it is a file nothing reads, which is
worse, because it looks authoritative and drifts silently.
"""

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-1a AC-2

from __future__ import annotations

import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

# Runs in a child interpreter with `pxr` forced to be unimportable, so the
# result is the same whether or not the test environment happens to have USD.
_BLOCK_PXR = """
import sys

class _Blocker:
    def find_module(self, name, path=None):
        return None

    def find_spec(self, name, path=None, target=None):
        if name == "pxr" or name.startswith("pxr."):
            raise ModuleNotFoundError("No module named 'pxr'")
        return None

sys.meta_path.insert(0, _Blocker())
"""


# A module file of the same name loses to the package silently rather than
# loudly, and each of these is dead to a different consumer of it.
SHADOWING_SUFFIXES = (
    (
        ".py",
        "CPython resolves the package directory first, so this file is never "
        "imported: any name only it declares is unreachable, and any name it "
        "duplicates becomes a second, silently dead copy",
    ),
    (
        ".pyi",
        "type checkers resolve the package first too -- mypy tries the package "
        "ahead of a same-named .pyi, and pyright looks for utils/__init__.pyi "
        "then utils/__init__.py before any sibling stub -- so this file is "
        "never consulted for ovphysx.utils and describes nothing, and it would "
        "take over only if utils/__init__.py disappeared",
    ),
)


@pytest.fixture
def pkg_dir() -> Path:
    """Directory of the installed/editable ovphysx package."""
    import ovphysx

    return Path(ovphysx.__file__).resolve().parent


def test_utils_is_a_package(pkg_dir: Path):
    """ovphysx.utils must be a package directory, not a module file."""
    utils_dir = pkg_dir / "utils"
    assert utils_dir.is_dir(), f"ovphysx.utils is not a package directory under {pkg_dir}"
    assert (utils_dir / "__init__.py").is_file(), f"{utils_dir} has no __init__.py"


def test_no_module_file_shadows_the_utils_package(pkg_dir: Path):
    """No `utils.*` module file may sit beside the `utils/` package.

    Both can exist at once: they are distinct filesystem names, so git merges
    a branch adding `utils.py` into a branch adding `utils/` with no conflict
    at all, and nothing fails at build time. The package then wins every
    resolution, which is why this has to be caught here rather than by any
    import or build step.
    """
    for suffix, consequence in SHADOWING_SUFFIXES:
        shadow = pkg_dir / f"utils{suffix}"
        assert not shadow.is_file(), (
            f"{shadow} sits beside the ovphysx/utils/ package and loses to it: "
            f"{consequence}. Both files can sit in the tree with the build "
            f"staying green, and git merges them without a conflict. Fix: move "
            f"its contents into a submodule of ovphysx/utils/ (re-exported from "
            f"utils/__init__.py, stub as utils/<submodule>.pyi) and delete "
            f"utils{suffix}."
        )


def _run_without_pxr(body: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, "-c", _BLOCK_PXR + textwrap.dedent(body)],
        capture_output=True,
        text=True,
        timeout=120,
    )


def test_ovstage_helper_imports_without_pxr():
    """`step_and_write_to_ovstage` must resolve in a process that has no USD.

    It is the one submodule here that authors nothing, and the `output_read`
    sample reaches it through `ovphysx.utils` in a venv with `ovstage` and no
    `pxr`. That worked while `ovphysx.utils` was a module file holding only this
    helper; as a package it works only because submodules load on first
    attribute access. Importing them eagerly in `__init__` pulls in `authoring`,
    which imports `pxr` at module scope, and the sample dies on import.
    """
    result = _run_without_pxr("""
        from ovphysx.utils import OvStageOutputCache, step_and_write_to_ovstage
        from ovphysx.utils.simulation import OvStageOutputCache as SimulationCache
        assert OvStageOutputCache is SimulationCache
        print(OvStageOutputCache.__name__, step_and_write_to_ovstage.__name__)
        """)
    assert result.returncode == 0, (
        "importing the ovstage helper from ovphysx.utils requires pxr, so any "
        "consumer that has ovstage but no USD -- the output_read sample among "
        f"them -- fails on import:\n{result.stderr}"
    )
    assert "step_and_write_to_ovstage" in result.stdout
    assert "OvStageOutputCache" in result.stdout


def test_dir_degrades_without_pxr(pkg_dir: Path):
    """`dir(ovphysx.utils)` must answer in a process that has no USD.

    Computing `__all__` imports every flattened submodule, so a `__dir__` built
    on it raised `ModuleNotFoundError: pxr` and prevented module discovery in
    exactly the environment the `output_read` sample runs in.
    `__dir__` skips what it cannot import instead, leaving the submodule names
    and whatever the importable ones export. `help()` and pydoc are outside this
    guarantee because pydoc reads `__all__`.

    The absence assertion is this case's own guard: with `pxr` reachable, `dir()`
    would report the whole flat surface and the rest would pass vacuously.
    """
    # Public submodules only: a leading underscore marks a module `__init__` does
    # not list in `__all__` and so never reports from `__dir__` either, such as
    # `_deprecation`, which holds the alias factory and exports nothing.
    submodules = sorted(path.stem for path in (pkg_dir / "utils").glob("*.py") if not path.stem.startswith("_"))
    result = _run_without_pxr("""
        import ovphysx.utils

        print(" ".join(dir(ovphysx.utils)))
        """)
    assert result.returncode == 0, (
        "dir(ovphysx.utils) requires pxr, so consumers of the module directory "
        "break in a process that has ovstage and no "
        f"USD:\n{result.stderr}"
    )
    names = set(result.stdout.split())
    assert "OvStageOutputCache" in names, sorted(names)
    assert "step_and_write_to_ovstage" in names, (
        "dir(ovphysx.utils) dropped the one public name reachable without USD; " f"got: {sorted(names)}"
    )
    assert set(submodules) <= names, f"dir(ovphysx.utils) is missing submodule names {sorted(set(submodules) - names)}"
    assert "add_rigid_box" not in names, (
        "an authoring name reached dir(ovphysx.utils) with pxr blocked, so the "
        "block is not working and this case proves nothing"
    )


def test_unknown_attribute_degrades_without_pxr():
    """Unknown lookups must keep the module protocol when USD is unavailable."""
    result = _run_without_pxr("""
        import ovphysx.utils

        missing = object()
        assert getattr(ovphysx.utils, "nope", missing) is missing
        assert getattr(ovphysx.utils, "__nope__", missing) is missing
        assert not hasattr(ovphysx.utils, "nope")

        try:
            ovphysx.utils.__nope__
        except AttributeError as exc:
            assert exc.__cause__ is None
        else:
            raise AssertionError("unknown dunder lookup did not raise AttributeError")

        try:
            ovphysx.utils.nope
        except AttributeError as exc:
            assert exc.__cause__ is None
        else:
            raise AssertionError("unknown attribute lookup did not raise AttributeError")
        """)
    assert result.returncode == 0, (
        "unknown ovphysx.utils attributes do not degrade to AttributeError with " f"pxr blocked:\n{result.stderr}"
    )


def test_known_authoring_attribute_reports_missing_pxr():
    """A known helper must not masquerade as absent when its owner cannot load."""
    result = _run_without_pxr("""
        import ovphysx.utils

        def from_import():
            from ovphysx.utils import add_rigid_box
            return add_rigid_box

        missing = object()
        for lookup in (
            lambda: ovphysx.utils.add_rigid_box,
            lambda: getattr(ovphysx.utils, "add_rigid_box", missing),
            lambda: hasattr(ovphysx.utils, "add_rigid_box"),
            from_import,
        ):
            try:
                lookup()
            except ModuleNotFoundError as exc:
                assert "pxr" in str(exc)
            else:
                raise AssertionError("known authoring helper was reported as absent")
        """)
    assert result.returncode == 0, (
        "a known ovphysx.utils helper did not report its missing pxr dependency:\n" f"{result.stderr}"
    )


def test_all_stays_strict_without_pxr():
    """`__all__` must keep raising without USD, where `__dir__` degrades.

    The asymmetry is deliberate. `__all__` is what `from ovphysx.utils import *`
    binds, so a tolerant one would hand a caller a partial authoring surface and
    let the missing helpers surface later as `NameError` on their own names;
    failing here names the dependency that is actually absent.
    """
    result = _run_without_pxr("""
        import ovphysx.utils

        print(len(ovphysx.utils.__all__))
        """)
    assert result.returncode != 0, (
        "ovphysx.utils.__all__ resolved with pxr blocked, so `from "
        "ovphysx.utils import *` yields a partial surface and every authoring "
        "name missing from it fails later as a NameError instead"
    )
    assert "pxr" in result.stderr, (
        "ovphysx.utils.__all__ failed without naming pxr, so a star-import "
        f"consumer cannot see what to install:\n{result.stderr}"
    )


def test_pxr_block_is_effective():
    """Guard the guard: prove the child really cannot import `pxr`.

    Without this, `test_ovstage_helper_imports_without_pxr` would pass just as
    happily in an environment where the block silently stopped working.
    """
    result = _run_without_pxr("""
        from ovphysx.utils import add_rigid_box
        print(add_rigid_box.__name__)
        """)
    assert result.returncode != 0, (
        "an authoring helper imported successfully with pxr blocked, so the "
        "block is not working and the companion test proves nothing"
    )
    assert "pxr" in result.stderr
