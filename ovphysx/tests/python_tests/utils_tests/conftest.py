# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Pytest configuration for the ovphysx.utils authoring tests.

These tests run in a SEPARATE pytest invocation from the main suite, for the
same class of reason cpu_tests/ does: USD builds its schema registry once, on
first access, and a registration that arrives after that is silently
ineffective. The codeless PhysX schemas therefore have to be registered before
anything in the process opens a stage, which is only guaranteed in a process
that runs nothing else. See test_python_runtime.cmake for the orchestration.

Nothing here needs a PhysX instance, a GPU, or the ovphysx native library: the
subpackage under test is pure `pxr` and the standard library.
"""

# @implements REQ-PYTHON-UTILS-001
# @covers AC-10 AC-14

import importlib.util
import os
import shutil

import pytest

# Set by the CMake test step, which runs this suite against a staged install
# tree. There, a schema tree that cannot be registered is a broken test stage
# rather than an environment the suite has to tolerate, and skipping would take
# every stage-opening case -- all the behavioral coverage this subpackage has --
# out of the run while still reporting a pass.
REQUIRE_CODELESS_SCHEMAS = os.environ.get("OVPHYSX_REQUIRE_CODELESS_SCHEMAS") == "1"

# The two codeless schema roots ovphysx ships, which register as independent USD
# plugins. `codeless._SENTINEL_IDENTIFIERS` carries one always-present applied
# API from each.
PHYSX_SCHEMA_MODULE = "PhysxSchema"
DEFORMABLE_SCHEMA_MODULE = "OmniUsdPhysicsDeformableSchema"


def _unavailable(reason):
    """Skip in a developer checkout, fail where the environment is guaranteed.

    Every degradation in this suite goes through here. A skip is right in a raw
    source checkout, where there is no staged schema tree and nothing is wrong
    with the subpackage; against the staged install tree the CMake step runs it
    on, the same condition is a broken test stage, and a skip would take the
    coverage out of the run while still reporting a pass.
    """
    if REQUIRE_CODELESS_SCHEMAS:
        pytest.fail(reason)
    pytest.skip(reason)


@pytest.fixture
def codeless_unavailable():
    """`_unavailable` as a fixture, for a case that decides part-way through.

    The registration-ordering cases cannot know whether their branch is
    reachable until their child interpreter has reported back, so they cannot
    express the decision as a fixture dependency. Without this they called
    `pytest.skip` directly and escaped the require-guard entirely -- which is
    the whole of AC-14 and the unregistered-schemas half of AC-10 able to
    vanish from a green run.
    """
    return _unavailable


def pytest_configure(config):
    """Fail the CI-mode run outright when there is no USD runtime to author into.

    The per-file `pytest.importorskip("pxr")` runs at collection, before any
    fixture, so the skip would otherwise empty the suite before the fixture
    below gets a chance to object.
    """
    if REQUIRE_CODELESS_SCHEMAS and importlib.util.find_spec("pxr") is None:
        raise pytest.UsageError(
            "OVPHYSX_REQUIRE_CODELESS_SCHEMAS=1 but `pxr` is not importable. The "
            "test venv is expected to carry stock usd-core, and the test stage must "
            "not launch this suite where PyPI ships none (linux-aarch64); see "
            "scripts/test_python_runtime.cmake."
        )


@pytest.fixture(scope="session")
def codeless_schemas():
    """Register ovphysx's codeless PhysX schemas before any stage is opened.

    Deliberately not autouse: the cases that only inspect the subpackage's
    surface need no schemas, and would otherwise skip alongside the authoring
    cases in an environment that cannot register them.

    Skips rather than fails in the two environments where registration cannot
    work, because neither is a defect in the subpackage under test:

    - No `pxr`. The utils subpackage requires a caller-supplied `usd-core` or
      equivalent, which the wheel deliberately does not depend on: ovphysx's USD
      runtime is the py-less ovstage one, so a declared dependency would put a
      second USD library in the same process. See REQ-PYTHON-UTILS-001, "Why
      `pxr` is caller-supplied".
    - No staged schema tree. `codeless_schema_paths()` resolves the wheel or
      `_install/` layout, so a raw source checkout has nothing to register.

    Under `OVPHYSX_REQUIRE_CODELESS_SCHEMAS=1` both become failures instead: see
    REQUIRE_CODELESS_SCHEMAS above.
    """
    pytest.importorskip("pxr", reason="ovphysx.utils requires a caller-supplied USD runtime")

    from ovphysx.utils import codeless

    staging_error = None
    try:
        codeless.register_schemas()
    except FileNotFoundError as exc:
        # Not fatal on its own: the schemas may already be registered through
        # ovphysx's own plugin path in this process.
        staging_error = exc

    if not codeless.schema_is_registered():
        if staging_error is not None:
            _unavailable(f"codeless schemas not staged in this layout: {staging_error}")
        _unavailable(
            "the USD schema registry was built before registration could take "
            "effect, so the codeless PhysX schemas are not visible"
        )


@pytest.fixture
def stage(codeless_schemas):
    """A fresh in-memory stage, with the codeless schemas already registered."""
    from pxr import Usd

    return Usd.Stage.CreateInMemory()


@pytest.fixture(scope="session")
def half_staged_schema_root(tmp_path_factory):
    """A schema tree carrying one of ovphysx's two codeless roots and not the other.

    No install layout produces this state, so it is built here: the PhysX root is
    copied in and the Omni deformable one is left out. It is what shows that a
    partial registration is reported as a registration failure rather than
    blamed on the caller's identifier spelling, which is the reason
    `codeless` probes a sentinel per root instead of one overall.

    Returns the tree rather than registering it: USD's schema registry is
    process-global and one-way, and this session's was built from the complete
    tree, so only a child interpreter can register this one.

    Skips, or fails under `OVPHYSX_REQUIRE_CODELESS_SCHEMAS=1`, on the same terms
    as `codeless_schemas` -- and additionally when the staged tree does not carry
    both roots, since a tree missing one already is not a half of anything.
    """
    pytest.importorskip("pxr", reason="ovphysx.utils requires a caller-supplied USD runtime")

    import ovphysx

    try:
        staged = ovphysx.codeless_schema_paths()
    except FileNotFoundError as exc:
        _unavailable(f"codeless schemas not staged in this layout: {exc}")

    by_module = {path.parent.name: path for path in staged}
    for module in (PHYSX_SCHEMA_MODULE, DEFORMABLE_SCHEMA_MODULE):
        if module not in by_module:
            _unavailable(f"the staged schema tree has no {module} root: {sorted(by_module)}")

    root = tmp_path_factory.mktemp("half_staged_schemas")
    shutil.copytree(by_module[PHYSX_SCHEMA_MODULE], root / PHYSX_SCHEMA_MODULE / "resources")
    return root
