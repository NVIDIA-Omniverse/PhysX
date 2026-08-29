# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for the namespaced ovphysx package isolation verifier.

The verifier is a packaging gate, not a USD-version check. It scans the staged
SDK/wheel native files and rejects files or binary imports that would make the
package depend on Python runtime libraries, classic modular USD, or core
Carbonite. Namespaced monolithic USD is allowed; classic `libusd_*`/`usd_*.dll`
libraries and core `libcarb.so`/`libcarb.so.*`/`carb.dll` are not.
"""

import subprocess
import sys
from pathlib import Path

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


def test_verify_allows_namespaced_usd_monolith(tmp_path, monkeypatch):
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    (plugins_dir / "libov_25.11usd_ms.so").write_text("namespaced usd\n")

    monkeypatch.setattr(verify_pyless_closure, "_check_elf_dynamic_linux", lambda root: [])

    assert verify_pyless_closure.verify(tmp_path) == []


def test_verify_rejects_obsolete_physx_runtime_plugin_artifacts(tmp_path, monkeypatch):
    plugins_dir = tmp_path / "plugins"
    plugins_dir.mkdir()
    for name in (
        "libomni.physx.plugin.so",
        "omni.physx.plugin.dll",
        "libomni.physx.fabric.plugin.so",
        "omni.physx.fabric.plugin.dll",
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

    # Filename check only; fake files are not real ELF/PE, so skip binary parsing.
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
