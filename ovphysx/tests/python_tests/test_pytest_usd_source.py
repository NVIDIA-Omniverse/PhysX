# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Guards for how the python-test suite sources USD (REQ-PACKAGING-PYTESTUSD-001).

The python tests resolve python USD from stock pip ``usd-core``, the only supported
source; no internal USD monolith is fetched. That is only safe while no test imports
``pxr`` in-process next to ovstage's own USD -- two different USD builds in one process
double-register USD's process-wide singletons and abort. These checks pin the
declaration (AC-1) and the one-USD-per-process invariant (AC-3) so a regression fails
here rather than as a hard crash on some later build.

Suites that run in their own pytest process without ever loading ovstage (see
``_ISOLATED_PXR_DIRS``) are exempt from the in-process-``pxr`` scan: pxr there is the
only USD in the process, so the rule cannot be violated.
"""

import re
from pathlib import Path


_HERE = Path(__file__).resolve().parent
_PYPROJECT = _HERE / "pyproject.toml"
_UV_LOCK = _HERE / "uv.lock"

# The only modules allowed to name ``pxr``, keyed by path relative to this directory
# (``as_posix``), NOT by bare filename -- a future ``subdir/foo.py`` must not inherit a
# top-level exemption without its own review. test_documentation_contracts.py names pxr
# only inside strings handed to a clean child interpreter, never in-process. No test may
# import pxr in-process: it would resolve to stock usd-core and collide with ovstage's
# resident USD. A new module that reaches for pxr must be reviewed against the one-USD
# rule and added here deliberately -- do not widen this set to make a failing check pass.
_PXR_ALLOWED = frozenset(
    {
        "test_documentation_contracts.py",
    }
)

# Top-level directories whose tests run in a dedicated pytest process that never loads
# ovstage (scripts/test_python_runtime.cmake runs each separately), so pxr in that
# process is the only USD and the one-USD-per-process rule cannot be violated there.
# utils_tests/ is the pure-`pxr` ovphysx.utils suite -- it needs no PhysX instance
# ("the subpackage is pure pxr", per test_python_runtime.cmake). These are exempt from
# the in-process-pxr scan below. A directory here must genuinely never attach ovstage.
_ISOLATED_PXR_DIRS = frozenset(
    {
        "utils_tests",
    }
)

# Literal ``from pxr`` / ``import pxr``, plus the dynamic forms that dodge it:
# ``importlib.import_module("pxr")`` and ``__import__("pxr")``. A re-export through a
# helper (``from .helper import stage`` where ``stage`` is a pxr type) is beyond a
# source grep and is not claimed to be caught here.
_PXR_REFERENCE = re.compile(
    r"\b(?:from|import)\s+pxr\b"
    # import_module may be called bare after `from importlib import import_module`,
    # so the importlib. prefix is optional; still requires a literal "pxr" argument.
    r"|\b(?:(?:importlib\.)?import_module|__import__)\s*\(\s*['\"]pxr(?:['\".])"
)


def test_pyproject_declares_stock_usd_core():
    """AC-1: the test project depends on stock usd-core, marker-gated off aarch64."""
    text = _PYPROJECT.read_text(encoding="utf-8")
    assert re.search(
        r'"usd-core;\s*platform_machine\s*!=\s*\'aarch64\'"', text
    ), "pyproject.toml must declare usd-core gated 'platform_machine != aarch64'"


def test_uv_lock_records_marked_usd_core():
    """AC-1: the lock carries usd-core as a leaf with the aarch64-exclusion marker."""
    text = _UV_LOCK.read_text(encoding="utf-8")
    assert 'name = "usd-core"' in text, "uv.lock does not record usd-core"
    assert re.search(
        r'name\s*=\s*"usd-core",\s*marker\s*=\s*"platform_machine\s*!=\s*\'aarch64\'"',
        text,
    ), "uv.lock must gate the usd-core edge on 'platform_machine != aarch64'"


def test_no_unexpected_in_process_pxr_users():
    """AC-3: only the allow-listed modules may name pxr at all.

    Enforces the one-USD-per-process rule at the source level: a test that adds an
    in-process ``from pxr import`` would resolve to stock usd-core in the default
    run and collide with ovstage's resident monolith. Comments are stripped so a
    prose mention of pxr does not trip the check.
    """
    offenders = {}
    skip_dirs = {".venv", "__pycache__"}
    for path in _HERE.rglob("*.py"):
        if any(part in skip_dirs or part.endswith(".egg-info") for part in path.parts):
            continue
        if path.resolve() == Path(__file__).resolve():
            continue  # this guard names pxr in its own patterns/strings
        rel = path.relative_to(_HERE).as_posix()
        if rel.split("/", 1)[0] in _ISOLATED_PXR_DIRS:
            continue  # dedicated ovstage-free process; pxr is the only USD there
        for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            line = raw.split("#", 1)[0]
            if _PXR_REFERENCE.search(line):
                offenders.setdefault(rel, []).append(lineno)

    unexpected = {rel: lines for rel, lines in offenders.items() if rel not in _PXR_ALLOWED}
    assert not unexpected, (
        "modules name pxr but are not in the one-USD allow-list "
        f"(_PXR_ALLOWED): {unexpected}. An in-process `from pxr import` resolves to "
        "stock usd-core in the default run and collides with ovstage's monolith; "
        "review against REQ-PACKAGING-PYTESTUSD-001 AC-3 before allow-listing."
    )


def test_isolated_pxr_dirs_are_excluded_from_the_main_session():
    """AC-3: every scan-exempt directory is kept out of the main pytest session.

    The exemption is only sound while test_python_runtime.cmake runs that directory
    in its own process; if the ``--ignore`` is dropped its pxr imports land next to
    ovstage again with nothing left to notice, so pin the two together.
    """
    cmake = (_HERE.parent.parent / "scripts" / "test_python_runtime.cmake").read_text(
        encoding="utf-8"
    )
    for name in sorted(_ISOLATED_PXR_DIRS):
        assert f"--ignore={name}" in cmake, (
            f"{name}/ imports pxr in-process and is scan-exempt only because it runs in "
            f"a separate pytest process; test_python_runtime.cmake must pass "
            f"--ignore={name} to the main session"
        )
