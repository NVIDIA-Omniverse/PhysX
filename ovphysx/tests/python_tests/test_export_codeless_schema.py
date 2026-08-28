# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Regression tests for scripts/export_codeless_schema.py (OMPE-86833).

These cover the transform/validation logic that derives the exposed codeless
PhysX USD schemas from the packaged runtime schema. The end-to-end registration
of the exported tree into a stock usd-core is covered by the
tests/python_samples_extra/codeless_schemas sample.
"""

import json
import sys
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import export_codeless_schema as ec  # noqa: E402


def _plug_info(name="physxSchema", type_="library", library_path="../../libusd.dll",
               types=None, root="..", resource_path="resources"):
    plugin = {
        "Name": name,
        "Type": type_,
        "Root": root,
        "ResourcePath": resource_path,
        "Info": {"Types": types if types is not None else {"PhysxSchemaFooAPI": {"schemaKind": "singleApplyAPI"}}},
    }
    if library_path is not None:
        plugin["LibraryPath"] = library_path
    return {"Plugins": [plugin]}


# ---------------------------------------------------------------------------
# _is_physics_module
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["physxSchema", "physxSchemaAddition", "omniUsdPhysicsDeformableSchema"])
def test_is_physics_module_accepts_physics_names(name):
    assert ec._is_physics_module(_plug_info(name=name)) is True


@pytest.mark.parametrize("name", ["usdPhysics", "usdPhysicsValidators", "usdGeom", "ar", "hdStorm"])
def test_is_physics_module_rejects_core_usd(name):
    assert ec._is_physics_module(_plug_info(name=name)) is False


# ---------------------------------------------------------------------------
# _to_codeless
# ---------------------------------------------------------------------------

def test_to_codeless_flips_type_and_drops_library_path():
    data = ec._to_codeless(_plug_info(type_="library", library_path="../../libusd.dll"))
    plugin = data["Plugins"][0]
    assert plugin["Type"] == "resource"
    assert "LibraryPath" not in plugin
    assert plugin["Root"] == ".." and plugin["ResourcePath"] == "resources"


def test_to_codeless_drops_unsubstituted_library_path_placeholder():
    data = ec._to_codeless(_plug_info(type_="resource", library_path="@PLUG_INFO_LIBRARY_PATH@"))
    assert "LibraryPath" not in data["Plugins"][0]


# ---------------------------------------------------------------------------
# _validate_codeless
# ---------------------------------------------------------------------------

def test_validate_codeless_passes_on_compliant():
    ec._validate_codeless(ec._to_codeless(_plug_info()), "physxSchema")  # no raise


def test_validate_codeless_rejects_library_type():
    bad = _plug_info(type_="library", library_path=None)
    with pytest.raises(SystemExit):
        ec._validate_codeless(bad, "physxSchema")


def test_validate_codeless_rejects_leftover_library_path():
    bad = _plug_info(type_="resource", library_path="../../libusd.dll")
    with pytest.raises(SystemExit):
        ec._validate_codeless(bad, "physxSchema")


def test_validate_codeless_rejects_empty_types():
    bad = _plug_info(type_="resource", library_path=None, types={})
    with pytest.raises(SystemExit):
        ec._validate_codeless(bad, "physxSchema")


# ---------------------------------------------------------------------------
# export_codeless_schemas (end-to-end on a tmp fixture)
# ---------------------------------------------------------------------------

def _write_module(plugins_usd: Path, dirname, plug_info, with_generated=True):
    res = plugins_usd / dirname / "resources"
    res.mkdir(parents=True)
    (res / "plugInfo.json").write_text(json.dumps(plug_info, indent=4))
    if with_generated:
        (res / "generatedSchema.usda").write_text("#usda 1.0\n")


def test_export_selects_physics_normalizes_and_skips_core(tmp_path):
    plugins_usd = tmp_path / "plugins" / "usd"
    out_dir = tmp_path / "schemas" / "physx"
    _write_module(plugins_usd, "PhysxSchema", _plug_info(name="physxSchema", type_="library"))
    _write_module(plugins_usd, "OmniUsdPhysicsDeformableSchema",
                  _plug_info(name="omniUsdPhysicsDeformableSchema", type_="resource",
                             library_path="@PLUG_INFO_LIBRARY_PATH@"))
    _write_module(plugins_usd, "usdPhysics", _plug_info(name="usdPhysics", type_="library"))

    exported = ec.export_codeless_schemas(plugins_usd, out_dir)

    assert sorted(exported) == ["OmniUsdPhysicsDeformableSchema", "PhysxSchema"]
    assert not (out_dir / "usdPhysics").exists(), "core USD module must not be exposed"
    for module in exported:
        plug = json.loads((out_dir / module / "resources" / "plugInfo.json").read_text())
        plugin = plug["Plugins"][0]
        assert plugin["Type"] == "resource"
        assert "LibraryPath" not in plugin
        assert (out_dir / module / "resources" / "generatedSchema.usda").is_file()


def test_export_raises_when_generated_schema_missing(tmp_path):
    plugins_usd = tmp_path / "plugins" / "usd"
    out_dir = tmp_path / "schemas" / "physx"
    _write_module(plugins_usd, "PhysxSchema", _plug_info(name="physxSchema"), with_generated=False)
    with pytest.raises(SystemExit):
        ec.export_codeless_schemas(plugins_usd, out_dir)


def test_export_raises_on_unparseable_physics_module(tmp_path):
    plugins_usd = tmp_path / "plugins" / "usd"
    out_dir = tmp_path / "schemas" / "physx"
    res = plugins_usd / "PhysxSchema" / "resources"
    res.mkdir(parents=True)
    (res / "plugInfo.json").write_text("{ this is not valid json")
    (res / "generatedSchema.usda").write_text("#usda 1.0\n")
    with pytest.raises(SystemExit):
        ec.export_codeless_schemas(plugins_usd, out_dir)


def test_export_skips_unparseable_non_physics_module(tmp_path):
    plugins_usd = tmp_path / "plugins" / "usd"
    out_dir = tmp_path / "schemas" / "physx"
    # A broken core-USD plugInfo we never expose must not fail the export.
    bad = plugins_usd / "usdGeom" / "resources"
    bad.mkdir(parents=True)
    (bad / "plugInfo.json").write_text("{ broken")
    _write_module(plugins_usd, "PhysxSchema", _plug_info(name="physxSchema"))
    exported = ec.export_codeless_schemas(plugins_usd, out_dir)
    assert exported == ["PhysxSchema"]


def test_export_clears_stale_output_on_rerun(tmp_path):
    plugins_usd = tmp_path / "plugins" / "usd"
    out_dir = tmp_path / "schemas" / "physx"
    _write_module(plugins_usd, "PhysxSchema", _plug_info(name="physxSchema"))
    _write_module(plugins_usd, "PhysxSchemaAddition", _plug_info(name="physxSchemaAddition"))
    ec.export_codeless_schemas(plugins_usd, out_dir)
    assert (out_dir / "PhysxSchemaAddition").exists()

    # Drop a module from the runtime and re-export: stale output must disappear.
    import shutil
    shutil.rmtree(plugins_usd / "PhysxSchemaAddition")
    exported = ec.export_codeless_schemas(plugins_usd, out_dir)
    assert exported == ["PhysxSchema"]
    assert not (out_dir / "PhysxSchemaAddition").exists()
