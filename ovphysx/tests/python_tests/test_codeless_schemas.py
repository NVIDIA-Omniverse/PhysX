# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-1

"""Tests for independently installed codeless USD schemas.

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
from schema_support import physx_schema_paths


def test_codeless_schema_paths_contain_expected_modules():
    paths = physx_schema_paths()
    assert paths, "expected at least one codeless schema package"
    module_names = {path.parent.name.lower() for path in paths}
    for expected in ("physxschema", "omniusdphysicsdeformableschema"):
        assert expected in module_names, (expected, module_names)


def test_codeless_schema_packages_have_required_files():
    paths = physx_schema_paths()
    for path in paths:
        assert path.name == "resources", f"expected a resources/ dir, got {path}"
        assert (path / "plugInfo.json").is_file(), f"missing plugInfo.json in {path}"
        assert (path / "generatedSchema.usda").is_file(), f"missing generatedSchema.usda in {path}"


def test_external_provider_pluginfo_is_codeless():
    """Each provider plugInfo.json must satisfy the codeless resource-plugin contract."""
    paths = physx_schema_paths()
    for path in paths:
        plug_info = json.loads("\n".join(line for line in (path / "plugInfo.json").read_text().splitlines() if not line.lstrip().startswith("#")))
        plugins = plug_info.get("Plugins", [])
        assert plugins, f"no Plugins in {path}"
        for plugin in plugins:
            assert plugin.get("Type") == "resource", (path, plugin.get("Type"))
            assert plugin.get("Root") == "..", (path, plugin.get("Root"))
            assert plugin.get("ResourcePath") == "resources", (path, plugin.get("ResourcePath"))
            assert not plugin.get("LibraryPath"), (path, plugin.get("LibraryPath"))
            assert plugin.get("Info", {}).get("Types"), f"no Info.Types in {path}"


def test_newton_schema_root_locates_the_installed_package_without_importing_it():
    """newton-usd-schemas is a test dependency; the helper must find its plugInfo.json
    through find_spec alone, since importing the package needs pxr and registers the
    schema with that stock USD runtime rather than with ovstage's."""
    import sys

    sys.modules.pop("newton_usd_schemas", None)
    root = ovphysx.newton_schema_root()
    assert (root / "plugInfo.json").is_file(), root
    assert (root / "generatedSchema.usda").is_file(), root
    assert "newton_usd_schemas" not in sys.modules


def test_newton_schema_root_reports_a_missing_package_with_the_install_hint(monkeypatch):
    import importlib.util

    from ovphysx import schemas

    real_find_spec = importlib.util.find_spec
    monkeypatch.setattr(
        importlib.util,
        "find_spec",
        lambda name, *a, **k: None if name == "newton_usd_schemas" else real_find_spec(name, *a, **k),
    )
    assert schemas.find_newton_schema_root() is None
    with pytest.raises(FileNotFoundError) as excinfo:
        schemas.newton_schema_root()
    message = str(excinfo.value)
    assert "pip install newton-usd-schemas" in message
    assert "github.com/newton-physics/newton-usd-schemas" in message
    assert "register_usd_schemas" in message


def test_schema_discovery_does_not_trigger_usd_or_native_bootstrap():
    script = """
import sys
import ovphysx
from schema_support import physx_schema_paths
assert len(physx_schema_paths()) == 2
assert "physx_usd_schemas" not in sys.modules
assert "pxr" not in sys.modules
assert "ovphysx._bindings" not in sys.modules
assert getattr(ovphysx, "_native_bootstrapped") is False
assert not hasattr(ovphysx, "codeless_schema_root")
assert not hasattr(ovphysx, "codeless_schema_paths")
"""
    subprocess.run([sys.executable, "-c", script], env=os.environ.copy(), check=True)
