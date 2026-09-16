# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-CLOSURE-001
# @covers AC-3

# @implements REQ-PACKAGING-USDFREE-001
# @covers AC-2 AC-4 AC-7

# @implements REQ-PACKAGING-OMNICLIENT-001
# @covers AC-1 AC-2

"""Tests for the ovphysx package isolation verifier.

The verifier is a packaging gate. It scans the staged SDK/wheel native files
and rejects files or binary imports that would make the package depend on
Python runtime libraries, any OpenUSD library (namespaced monolith or classic
modular), USD's dependency closure, or core Carbonite, and it rejects a shipped
USD plugin registry. With --require-schemas it also demands the codeless PhysX
schema tree ovphysx ships as data.
"""

import subprocess
import sys
from pathlib import Path

import json

import pytest

OVPHYSX_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(OVPHYSX_ROOT / "scripts"))

import verify_pyless_closure  # noqa: E402


def test_verify_rejects_classic_usd_library_files(tmp_path):
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    (plugins_dir / "libusd_tf.so").write_text("classic usd\n")
    (plugins_dir / "usd_sdf.dll").write_text("classic usd\n")

    violations = verify_pyless_closure.verify(tmp_path)

    assert "forbidden file: plugins/libusd_tf.so" in violations
    assert "forbidden file: plugins/usd_sdf.dll" in violations


def test_verify_rejects_forbidden_symlink_names(tmp_path, monkeypatch):
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    forbidden_link = plugins_dir / "libpython3.12.so"
    try:
        forbidden_link.symlink_to(plugins_dir / "missing-libpython3.12.so.1.0")
    except (OSError, NotImplementedError) as exc:
        pytest.skip(f"symlink creation unavailable: {exc}")

    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])

    violations = verify_pyless_closure.verify(tmp_path)

    assert "forbidden file: plugins/libpython3.12.so" in violations


def test_verify_rejects_namespaced_usd_monolith_and_its_closure(tmp_path, monkeypatch):
    """ovphysx ships no OpenUSD: the ovstage-owned monolith, the resolver, and
    USD's dependency closure must all stay out of the payload."""
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    names = (
        "libov_25.11usd_ms.so",
        "ov_25.11usd_ms.dll",
        "libomni_usd_resolver.so",
        "libtbb.so.12.13",
        "tbb12.dll",
        "libMaterialXCore.so.1.39.3",
        "libAlembic.so.1.8.5",
        "libImath-3_1.so.29.11.0",
        "libosdCPU.so.3.6.0",
        "libdraco.so.1",
        "hdStorm.dll",
    )
    for name in names:
        (plugins_dir / name).write_text("usd closure\n")

    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])
    monkeypatch.setattr(verify_pyless_closure, "_check_pe_imports_windows", lambda root: [])

    violations = verify_pyless_closure.verify(tmp_path)

    for name in names:
        assert f"forbidden file: plugins/{name}" in violations


def test_verify_rejects_ovstage_owned_omniclient_payload(tmp_path, monkeypatch):
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    names = (
        "libomniclient.so",
        "libomniverse_connection.so",
        "omniclient.dll",
        "omniverse_connection.dll",
        "ovstage-omniclient.version",
    )
    for name in names:
        (plugins_dir / name).write_text("OVStage-owned asset runtime\n")

    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])
    monkeypatch.setattr(verify_pyless_closure, "_check_pe_imports_windows", lambda root: [])

    violations = verify_pyless_closure.verify(tmp_path)

    for name in names:
        assert f"forbidden file: plugins/{name}" in violations


def test_verify_rejects_usd_plugin_registry_directory(tmp_path, monkeypatch):
    registry = tmp_path / "plugins" / "usd" / "PhysxSchema" / "resources"
    registry.mkdir(parents=True)
    (registry / "plugInfo.json").write_text("{}\n")

    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])

    violations = verify_pyless_closure.verify(tmp_path)

    assert any(v.startswith("USD plugin registry shipped: plugins/usd") for v in violations)


@pytest.mark.skipif(sys.platform != "linux", reason="ELF dynamic checks are Linux-only")
def test_verify_rejects_direct_usd_monolith_import(tmp_path, monkeypatch):
    plugin = tmp_path / "lib" / "libovphysx_internal.so"
    plugin.parent.mkdir()
    plugin.write_text("not a real elf\n")

    def fake_run(args, **kwargs):
        assert args[:2] == ["readelf", "-d"]
        return subprocess.CompletedProcess(
            args,
            0,
            stdout=" 0x0000000000000001 (NEEDED) Shared library: [libov_25.11usd_ms.so]\n",
            stderr="",
        )

    monkeypatch.setattr(subprocess, "run", fake_run)

    violations = verify_pyless_closure.verify(tmp_path)

    assert "forbidden DT_NEEDED in lib/libovphysx_internal.so: libov_25.11usd_ms.so" in violations


_MODULE_PLUG_INFO = {
    "Plugins": [
        {
            "Name": "physxSchema",
            "Type": "resource",
            "Root": "..",
            "ResourcePath": "resources",
            "Info": {"Types": {"PhysxSchemaPhysxRigidBodyAPI": {"schemaKind": "singleApplyAPI"}}},
        }
    ]
}


def _write_codeless_schema_tree(root):
    schema_root = root / "schemas" / "physx"
    module = schema_root / "PhysxSchema" / "resources"
    module.mkdir(parents=True)
    (schema_root / "plugInfo.json").write_text('{"Includes": ["*/resources/"]}\n')
    (module / "plugInfo.json").write_text(json.dumps(_MODULE_PLUG_INFO) + "\n")
    (module / "generatedSchema.usda").write_text("#usda 1.0\n")
    return schema_root


def test_verify_require_schemas_accepts_complete_codeless_tree(tmp_path, monkeypatch):
    _write_codeless_schema_tree(tmp_path)
    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])

    assert verify_pyless_closure.verify(tmp_path, require_schemas=True) == []


def test_verify_rejects_upstream_modular_usd_library_names(tmp_path, monkeypatch):
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    (plugins_dir / "libusdGeom.so").write_text("stock usd\n")
    (plugins_dir / "tf.dll").write_text("stock usd\n")
    (plugins_dir / "libomni.tbb.globalcontrol.plugin.so").write_text("allowed\n")
    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])

    violations = verify_pyless_closure.verify(tmp_path)

    assert "forbidden file: plugins/libusdGeom.so" in violations
    assert "forbidden file: plugins/tf.dll" in violations
    assert not [v for v in violations if "globalcontrol" in v]


def test_verify_rejects_usd_runtime_config_file(tmp_path, monkeypatch):
    lib_dir = tmp_path / "lib"
    lib_dir.mkdir()
    (lib_dir / "config.toml").write_text("[usd]\n")
    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])

    assert verify_pyless_closure.verify(tmp_path) == ["USD runtime configuration shipped: lib/config.toml"]


def test_verify_require_schemas_rejects_nonfunctional_registry_and_module(tmp_path, monkeypatch):
    schema_root = _write_codeless_schema_tree(tmp_path)
    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])

    (schema_root / "plugInfo.json").write_text("{}\n")
    module_plug_info = schema_root / "PhysxSchema" / "resources" / "plugInfo.json"
    module_plug_info.write_text("{}\n")
    violations = verify_pyless_closure.verify(tmp_path, require_schemas=True)
    assert any(v.startswith("codeless schema root registry does not include the module resources:") for v in violations)
    assert "codeless schema module without a Plugins entry: schemas/physx/PhysxSchema/resources/plugInfo.json" in violations

    (schema_root / "plugInfo.json").write_text('{"Includes": ["*/resources/"]}\n')
    compiled = json.loads(json.dumps(_MODULE_PLUG_INFO))
    compiled["Plugins"][0]["Type"] = "library"
    compiled["Plugins"][0]["LibraryPath"] = "../../libphysxSchema.so"
    module_plug_info.write_text(json.dumps(compiled) + "\n")
    violations = verify_pyless_closure.verify(tmp_path, require_schemas=True)
    assert (
        "codeless schema module is not a resource plugin: schemas/physx/PhysxSchema/resources/plugInfo.json (Type 'library')"
        in violations
    )
    assert "codeless schema module names a LibraryPath: schemas/physx/PhysxSchema/resources/plugInfo.json" in violations


def test_verify_require_schemas_rejects_missing_root_registry_and_module_payload(tmp_path, monkeypatch):
    schema_root = _write_codeless_schema_tree(tmp_path)
    (schema_root / "plugInfo.json").unlink()
    (schema_root / "PhysxSchema" / "resources" / "generatedSchema.usda").unlink()
    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])

    violations = verify_pyless_closure.verify(tmp_path, require_schemas=True)

    assert "missing codeless schema root registry: schemas/physx/plugInfo.json" in violations
    assert any("without generatedSchema.usda" in v for v in violations)
    # Without the flag the schema tree is optional (a wheel staging tree is checked with it).
    assert verify_pyless_closure.verify(tmp_path) == []


def test_verify_rejects_obsolete_runtime_plugin_artifacts(tmp_path, monkeypatch):
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    for name in (
        "libomni.physx.plugin.so",
        "omni.physx.plugin.dll",
        "libomni.physx.fabric.plugin.so",
        "omni.physx.fabric.plugin.dll",
        "libomni.cubric.plugin.so",
        "omni.cubric.plugin.dll",
        "libomni.gpucompute-cuda.plugin.so",
        "omni.gpucompute-cuda.plugin.dll",
        "libomni.physics.tensors.plugin.so",
        "omni.physics.tensors.plugin.dll",
        "libomni.physx.tensors.plugin.so",
        "omni.physx.tensors.plugin.dll",
        "libomni.physx.cooking.plugin.so",
        "omni.physx.cooking.plugin.dll",
        "libomni.physx.foundation.plugin.so",
        "omni.physx.foundation.plugin.dll",
    ):
        (plugins_dir / name).write_text("obsolete plugin artifact\n")

    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])
    monkeypatch.setattr(verify_pyless_closure, "_check_pe_imports_windows", lambda root: [])

    violations = verify_pyless_closure.verify(tmp_path)

    for name in (
        "libomni.physx.plugin.so",
        "omni.physx.plugin.dll",
        "libomni.physx.fabric.plugin.so",
        "omni.physx.fabric.plugin.dll",
        "libomni.cubric.plugin.so",
        "omni.cubric.plugin.dll",
        "libomni.gpucompute-cuda.plugin.so",
        "omni.gpucompute-cuda.plugin.dll",
        "libomni.physics.tensors.plugin.so",
        "omni.physics.tensors.plugin.dll",
        "libomni.physx.tensors.plugin.so",
        "omni.physx.tensors.plugin.dll",
        "libomni.physx.cooking.plugin.so",
        "omni.physx.cooking.plugin.dll",
        "libomni.physx.foundation.plugin.so",
        "omni.physx.foundation.plugin.dll",
    ):
        assert f"forbidden file: plugins/{name}" in violations


@pytest.mark.skipif(sys.platform != "linux", reason="ELF dynamic checks are Linux-only")
def test_verify_rejects_forbidden_elf_dynamic_entries_with_one_readelf_call(tmp_path, monkeypatch):
    plugin = tmp_path / "plugins" / "libomni.physics.tensors.plugin.so"
    plugin.parent.mkdir()
    plugin.write_text("not a real elf\n")

    readelf_calls = []

    def fake_run(args, **kwargs):
        assert args[:2] == ["readelf", "-d"]
        readelf_calls.append(args)
        return subprocess.CompletedProcess(
            args,
            0,
            stdout=(
                " 0x000000000000001d (RUNPATH) Library runpath: "
                "[/tmp/schemas/physx/_build/target-deps/python/lib]\n"
                " 0x0000000000000001 (NEEDED) Shared library: [libusd_tf.so]\n"
            ),
            stderr="",
        )

    monkeypatch.setattr(subprocess, "run", fake_run)

    violations = verify_pyless_closure.verify(tmp_path)

    assert len(readelf_calls) == 1
    assert any(
        "forbidden RPATH in plugins/libomni.physics.tensors.plugin.so: contains 'target-deps/python'" in v for v in violations
    )
    assert "forbidden DT_NEEDED in plugins/libomni.physics.tensors.plugin.so: libusd_tf.so" in violations


def test_verify_rejects_core_carbonite_library_files(tmp_path, monkeypatch):
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    (plugins_dir / "libcarb.so").write_text("core carbonite\n")
    (plugins_dir / "libcarb.so.0").write_text("core carbonite\n")
    (plugins_dir / "carb.dll").write_text("core carbonite\n")

    # Filename check only. The fake files are not real ELF/PE, so binary parsing is skipped.
    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])
    monkeypatch.setattr(verify_pyless_closure, "_check_pe_imports_windows", lambda root: [])

    violations = verify_pyless_closure.verify(tmp_path)

    assert "forbidden file: plugins/libcarb.so" in violations
    assert "forbidden file: plugins/libcarb.so.0" in violations
    assert "forbidden file: plugins/carb.dll" in violations


def test_verify_allows_no_libcarb_static_carb_plugin_shims(tmp_path, monkeypatch):
    # The per-plugin no-libcarb shims ovphysx legitimately ships must NOT be
    # flagged by the core-carb patterns (false-positive guard).
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    for name in (
        "libcarb.datastore.plugin.so",
        "libcarb.stats.plugin.so",
        "libcarb.profiler-cpu.plugin.so",
        "libcarb.profiler-mux.plugin.so",
        "libcarb.ujitso.default.plugin.so",
        "libcarb.ujitsoagent.plugin.so",
    ):
        (plugins_dir / name).write_text("no-libcarb shim\n")

    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])
    monkeypatch.setattr(verify_pyless_closure, "_check_pe_imports_windows", lambda root: [])

    assert verify_pyless_closure.verify(tmp_path) == []


@pytest.mark.skipif(sys.platform != "linux", reason="ELF dynamic checks are Linux-only")
def test_verify_rejects_libcarb_dt_needed(tmp_path, monkeypatch):
    plugin = tmp_path / "plugins" / "libcarb.stats.plugin.so"
    plugin.parent.mkdir()
    plugin.write_text("not a real elf\n")

    def fake_run(args, **kwargs):
        assert args[:2] == ["readelf", "-d"]
        return subprocess.CompletedProcess(
            args,
            0,
            stdout=(
                " 0x0000000000000001 (NEEDED) Shared library: [libcarb.so]\n"
                " 0x0000000000000001 (NEEDED) Shared library: [libcarb.so.0]\n"
            ),
            stderr="",
        )

    monkeypatch.setattr(subprocess, "run", fake_run)

    violations = verify_pyless_closure.verify(tmp_path)

    assert "forbidden DT_NEEDED in plugins/libcarb.stats.plugin.so: libcarb.so" in violations
    assert "forbidden DT_NEEDED in plugins/libcarb.stats.plugin.so: libcarb.so.0" in violations


@pytest.mark.skipif(sys.platform != "linux", reason="ELF dynamic checks are Linux-only")
@pytest.mark.parametrize("soname", ["libcarb.so", "libcarb.so.0"])
def test_verify_rejects_libcarb_soname(tmp_path, monkeypatch, soname):
    plugin = tmp_path / "plugins" / "libunexpected_core.so.0"
    plugin.parent.mkdir()
    plugin.write_text("not a real elf\n")

    def fake_run(args, **kwargs):
        assert args[:2] == ["readelf", "-d"]
        return subprocess.CompletedProcess(
            args,
            0,
            stdout=f" 0x000000000000000e (SONAME) Library soname: [{soname}]\n",
            stderr="",
        )

    monkeypatch.setattr(subprocess, "run", fake_run)

    violations = verify_pyless_closure.verify(tmp_path)

    assert f"forbidden SONAME in plugins/libunexpected_core.so.0: {soname}" in violations


def test_verify_rejects_upstream_usd_and_resolver_pe_imports(tmp_path, monkeypatch):
    plugin = tmp_path / "plugins" / "ovphysx.dll"
    plugin.parent.mkdir()
    plugin.write_text("not a real pe\n")

    def _fake_read_pe_imports(path):
        assert path == plugin
        return ["KERNEL32.dll", "tf.dll", "usdGeom.dll", "omni_usd_resolver.dll", "omni.tbb.globalcontrol.plugin.dll"]

    monkeypatch.setattr(verify_pyless_closure.platform, "system", lambda: "Windows")
    monkeypatch.setattr(verify_pyless_closure, "read_pe_imports", _fake_read_pe_imports)

    violations = verify_pyless_closure.verify(tmp_path)

    assert "forbidden PE import in plugins/ovphysx.dll: tf.dll" in violations
    assert "forbidden PE import in plugins/ovphysx.dll: usdGeom.dll" in violations
    assert "forbidden PE import in plugins/ovphysx.dll: omni_usd_resolver.dll" in violations
    assert not [v for v in violations if "globalcontrol" in v]


def test_verify_rejects_carb_dll_pe_import(tmp_path, monkeypatch):
    plugin = tmp_path / "plugins" / "ovphysx.dll"
    plugin.parent.mkdir()
    plugin.write_text("not a real pe\n")

    def _fake_read_pe_imports(path):
        assert path == plugin
        return ["KERNEL32.dll", "carb.dll"]

    monkeypatch.setattr(verify_pyless_closure.platform, "system", lambda: "Windows")
    monkeypatch.setattr(verify_pyless_closure, "read_pe_imports", _fake_read_pe_imports)

    violations = verify_pyless_closure.verify(tmp_path)

    assert "forbidden PE import in plugins/ovphysx.dll: carb.dll" in violations
