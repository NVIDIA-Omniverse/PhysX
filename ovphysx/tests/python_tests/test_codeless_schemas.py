# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Unit tests for the codeless USD schema exposure API (OMPE-86833).

These are pure-Python path/packaging checks and do not require usd-core. The
end-to-end registration of the codeless schemas against a stock usd-core
runtime is covered by the tests/python_samples_extra/codeless_schemas
regression sample.
"""

import json
import os
import subprocess
import sys

import pytest

import ovphysx


def _schema_paths_or_skip():
    try:
        return ovphysx.codeless_schema_paths()
    except FileNotFoundError as exc:
        pytest.skip(f"codeless schemas not staged in this layout: {exc}")


def test_codeless_schema_root_is_dir():
    try:
        root = ovphysx.codeless_schema_root()
    except FileNotFoundError as exc:
        pytest.skip(f"codeless schemas not staged in this layout: {exc}")
    assert root.is_dir()
    assert root.name == "physx"


def test_codeless_schema_paths_contain_expected_modules():
    paths = _schema_paths_or_skip()
    assert paths, "expected at least one codeless schema package"
    # Module directory names mirror the packaged runtime schema (derived from
    # _install/plugins/usd), matched case-insensitively. Require each PhysX
    # schema module ovphysx ships; the set may grow, so this is a subset check
    # rather than an equality check.
    module_names = {path.parent.name.lower() for path in paths}
    for expected in ("physxschema", "omniusdphysicsdeformableschema"):
        assert expected in module_names, (expected, module_names)


def test_codeless_schema_packages_have_required_files():
    paths = _schema_paths_or_skip()
    for path in paths:
        assert path.name == "resources", f"expected a resources/ dir, got {path}"
        assert (path / "plugInfo.json").is_file(), f"missing plugInfo.json in {path}"
        assert (path / "generatedSchema.usda").is_file(), f"missing generatedSchema.usda in {path}"


def test_shipped_pluginfo_is_codeless():
    """Each shipped plugInfo.json must satisfy the codeless resource-plugin contract."""
    paths = _schema_paths_or_skip()
    for path in paths:
        plug_info = json.loads((path / "plugInfo.json").read_text())
        plugins = plug_info.get("Plugins", [])
        assert plugins, f"no Plugins in {path}"
        for plugin in plugins:
            assert plugin.get("Type") == "resource", (path, plugin.get("Type"))
            assert plugin.get("Root") == "..", (path, plugin.get("Root"))
            assert plugin.get("ResourcePath") == "resources", (path, plugin.get("ResourcePath"))
            assert not plugin.get("LibraryPath"), (path, plugin.get("LibraryPath"))
            assert plugin.get("Info", {}).get("Types"), f"no Info.Types in {path}"


def test_codeless_schema_api_does_not_trigger_native_bootstrap():
    """The discovery helpers must never load the native runtime."""
    script = """
import sys
import ovphysx
try:
    ovphysx.codeless_schema_root()
    ovphysx.codeless_schema_paths()
except FileNotFoundError:
    pass
assert "ovphysx._bindings" not in sys.modules
assert getattr(ovphysx, "_native_bootstrapped") is False
"""
    subprocess.run([sys.executable, "-c", script], env=os.environ.copy(), check=True)
