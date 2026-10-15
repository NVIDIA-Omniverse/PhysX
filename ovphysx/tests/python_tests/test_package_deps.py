# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-USDFREE-001
# @covers AC-4
# @implements REQ-PACKAGING-CLOSURE-001
# @covers AC-1
# @implements REQ-PACKAGING-OMNICLIENT-001
# @covers AC-1

"""Regression tests for scripts/package_deps.py.

These tests cover the source of truth for the SDK/wheel native files:
external schema delivery, py-less and USD-free dependency staging,
and the OVStage runtime validation. They intentionally exercise the Python
packaging helper directly so policy failures are caught before a full install or
wheel build.
"""

import json
import os
import platform
import sys
import zipfile
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import package_deps as package_deps_module  # noqa: E402


def test_resolver_and_client_have_no_direct_packman_coordinates():
    repo_root = Path(__file__).resolve().parents[3]
    manifests = [
        repo_root / "ovphysx" / "deps" / "ovruntime-deps-import.packman.xml",
        repo_root / "ovphysx" / "deps" / "carb-sdk-deps-import.packman.xml",
        repo_root / "ovphysx" / "ovruntime" / "deps" / "ovruntime-deps-import.packman.xml",
    ]
    combined = "\n".join(path.read_text() for path in manifests)

    assert "omniusdresolver_ov_openusd_0.25.11_nopy" not in combined
    assert "omni_client_library" not in combined
    assert "2.72.0-mr1112.7072+gl.253c8c1e" not in combined


def test_load_json_like_accepts_usd_plug_info_json_extensions(tmp_path):
    """USD plugInfo.json can use comments and URLs that strict JSON rejects.

    The package step rewrites plugInfo LibraryPath values after staging. If this
    parser rejects USD's lax comment styles or strips `//` inside a URL string,
    the rewrite is skipped and runtime schema discovery can fail.
    """
    plug_info = tmp_path / "plugInfo.json"
    plug_info.write_text(
        "{\n"
        "  // USD-style comment\n"
        '  "Plugins": [\n'
        "    {\n"
        '      "Name": "test", /* inline block comment */\n'
        '      "LibraryPath": "http://example.invalid/libusd_test.so",\n'
        "    },\n"
        "  ],\n"
        "}\n"
    )

    data = package_deps_module._load_json_like(plug_info)

    assert data is not None
    assert data["Plugins"][0]["Name"] == "test"
    assert data["Plugins"][0]["LibraryPath"] == "http://example.invalid/libusd_test.so"


def test_load_json_like_keeps_comma_brace_text_inside_strings(tmp_path):
    """Trailing-comma cleanup must not edit quoted plugInfo metadata strings."""
    plug_info = tmp_path / "plugInfo.json"
    plug_info.write_text(
        "{\n"
        '  "Plugins": [\n'
        "    {\n"
        '      "Name": "test",\n'
        '      "Description": "literal text ,} and ,] stays unchanged",\n'
        '      "LibraryPath": "../lib/libusd_test.so",\n'
        "    },\n"
        "  ],\n"
        "}\n"
    )

    data = package_deps_module._load_json_like(plug_info)

    assert data is not None
    assert data["Plugins"][0]["Description"] == "literal text ,} and ,] stays unchanged"


def test_load_json_like_accepts_unterminated_trailing_block_comment(tmp_path):
    """Treat a malformed trailing block comment as comment text, not JSON data."""
    plug_info = tmp_path / "plugInfo.json"
    plug_info.write_text('{"Plugins": []} /* generated comment was truncated')

    data = package_deps_module._load_json_like(plug_info)

    assert data == {"Plugins": []}


def _write_stub_manifest(path: Path) -> None:
    path.write_text("[carbonite_plugins]\n" "include = []\n\n" "[packaging]\n" "skip_shared_libs = []\n")


def test_copy_carbonite_plugins_errors_when_source_dir_missing(tmp_path):
    missing_source = tmp_path / "missing-carbonite-shims"

    with pytest.raises(RuntimeError, match="Static Carbonite plugin shim directory not found"):
        package_deps_module.copy_carbonite_plugins(
            source_dir=missing_source,
            plugins_dest=tmp_path / "plugins",
            plugin_names=["carb.profiler-cpu.plugin"],
            verbose=False,
        )


def test_copy_carbonite_plugins_errors_when_required_plugin_missing(tmp_path):
    source_dir = tmp_path / "carbonite-shims"
    source_dir.mkdir()

    with pytest.raises(RuntimeError, match="Missing required Carbonite bootstrap plugin shims"):
        package_deps_module.copy_carbonite_plugins(
            source_dir=source_dir,
            plugins_dest=tmp_path / "plugins",
            plugin_names=["carb.profiler-cpu.plugin"],
            verbose=False,
        )


def _create_package_deps_layout(tmp_path: Path) -> dict[str, Path]:
    platform_build = package_deps_module._preferred_platform_build()
    build_dir = tmp_path / "build"
    install_dir = tmp_path / "install"
    install_dir.mkdir()
    license_dir = build_dir / "PACKAGE-LICENSES"
    license_dir.mkdir(parents=True)
    with zipfile.ZipFile(license_dir / "ovphysx-LICENSES.zip", "w") as archive:
        archive.writestr("base-LICENSE.txt", "base-license")

    manifest_path = tmp_path / "deps_manifest.toml"
    _write_stub_manifest(manifest_path)

    ovphysx_root = tmp_path / "ovphysx"
    ovruntime_root = ovphysx_root / "ovruntime"
    (ovruntime_root / "_install" / "release").mkdir(parents=True)
    (ovruntime_root / "_install" / "debug").mkdir(parents=True)

    ovruntime_tdeps = ovruntime_root / "_build" / "target-deps"
    # package_deps.py picks the ovruntime_deps_<config> variant matching the
    # build, so set up both release and debug payloads. Real pull_dependencies
    # pulls both. The per-config plugin binaries differ in DT_NEEDED libtbb
    # variant (release links libtbb.so.12, debug links libtbb_debug.so.12).
    for cfg in ("release", "debug"):
        ovruntime_deps_root = ovruntime_tdeps / f"ovruntime_deps_{cfg}" / "_build" / platform_build / cfg / "plugins"
        for subdir in (
            "carb.datastore",
            "omni.blobkey",
            "carb.ujitsoagent",
            "carb.ujitso.default",
            "omni.fabric",
            "usdrt",
            "scenegraph",
            "omni.cubric",
            "gpucompute",
            "omni.usd",
        ):
            (ovruntime_deps_root / subdir).mkdir(parents=True, exist_ok=True)

    target_deps = build_dir / "target-deps"

    # Exact OVStage package layout used as the indivisible provider. Keep stale
    # direct-package directories beside it to prove packaging never falls back.
    ovstage_dir = tmp_path / "ovstage"
    ovstage_plugins = ovstage_dir / "bin" / "plugins"
    client_lib_dir = ovstage_plugins / "omni.client.lib"
    resolver_lib_dir = ovstage_plugins / "omni.usd_resolver"
    resolver_resources = resolver_lib_dir / "usd" / "omni_usd_resolver" / "resources"
    client_lib_dir.mkdir(parents=True)
    resolver_resources.mkdir(parents=True)
    (client_lib_dir / "libomniclient.so").write_text("ovstage-client")
    (client_lib_dir / "libomniverse_connection.so").write_text("ovstage-connection")
    (resolver_lib_dir / "libomni_usd_resolver.so").write_text("ovstage-resolver")
    (resolver_resources / "plugInfo.json").write_text(
        json.dumps(
            {
                "Plugins": [
                    {
                        "Name": "Omniverse USD Plugin",
                        "Type": "library",
                        "LibraryPath": "../../../libomni_usd_resolver.so",
                        "Root": ".",
                    }
                ]
            }
        )
    )
    # OVStage ships its own USD runtime; ovphysx packaging neither stages nor
    # inspects it (REQ-PACKAGING-USDFREE-001 AC-4).
    (ovstage_plugins / "libov_25.11usd_ms.so").write_text("ovstage-monolith")
    (ovstage_dir / "THIRD-PARTY-NOTICES.txt").write_text("ovstage-notice")

    stale_client = target_deps / "client-library" / "release"
    stale_resolver = target_deps / "omni_usd_resolver" / "lib"
    stale_client.mkdir(parents=True)
    stale_resolver.mkdir(parents=True)
    (stale_client / "libomniclient.so").write_text("stale-direct-client")
    (stale_resolver / "libomni_usd_resolver.so").write_text("stale-direct-resolver")

    omni_physics_link = target_deps / "omni_physics"
    omni_physics_link.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.symlink(ovruntime_root, omni_physics_link, target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"symlink support required for package_deps layout test: {exc}")

    return {
        "build_dir": build_dir,
        "install_dir": install_dir,
        "manifest_path": manifest_path,
        "ovruntime_root": ovruntime_root,
        "ovstage_dir": ovstage_dir,
    }


def _patch_package_deps_copy_steps(
    monkeypatch,
    recorded_plugin_labels: list[str] | None = None,
    recorded_shared_lib_labels: list[str] | None = None,
) -> None:
    monkeypatch.setattr(package_deps_module, "copy_carbonite_plugins", lambda *args, **kwargs: 0)

    def _fake_copy_plugin_libs(source_dir, dest_dir, skip_patterns=None, verbose=False, label=""):
        if recorded_plugin_labels is not None:
            recorded_plugin_labels.append(label)
        return 0

    monkeypatch.setattr(package_deps_module, "_copy_plugin_libs", _fake_copy_plugin_libs)

    def _fake_copy_shared_libs(
        source_dir,
        dest_dir,
        skip_patterns=None,
        include_patterns=None,
        verbose=False,
        label="",
    ):
        if recorded_shared_lib_labels is not None:
            recorded_shared_lib_labels.append(label)
        if label == "client-library":
            return 1
        return 0

    monkeypatch.setattr(package_deps_module, "_copy_shared_libs", _fake_copy_shared_libs)


def test_package_deps_has_no_python_runtime_copy_helper():
    assert not hasattr(package_deps_module, "copy_python_runtime_libs")


def test_package_deps_excludes_non_owned_runtime_payloads(tmp_path, monkeypatch):
    if platform.system() == "Windows":
        pytest.skip("package_deps layout test exercises POSIX symlink semantics")

    layout = _create_package_deps_layout(tmp_path)

    recorded_plugin_labels: list[str] = []
    _patch_package_deps_copy_steps(
        monkeypatch,
        recorded_plugin_labels=recorded_plugin_labels,
    )

    success = package_deps_module.package_deps(
        manifest_path=layout["manifest_path"],
        build_dir=layout["build_dir"],
        install_dir=layout["install_dir"],
        ovruntime_install_dir=layout["ovruntime_root"] / "_install" / "release",
        ovstage_dir=layout["ovstage_dir"],
        ovstage_runtime_dir=layout["ovstage_dir"] / "bin",
        config="release",
        verbose=False,
    )

    assert success is True
    assert any(label.startswith("ovruntime_deps/carb.datastore") for label in recorded_plugin_labels)
    assert any(label.startswith("ovruntime_deps/omni.usd") for label in recorded_plugin_labels)
    assert not any(label.startswith("ovruntime_deps/omni.cubric") for label in recorded_plugin_labels)
    assert not any(label.startswith("ovruntime_deps/gpucompute") for label in recorded_plugin_labels)
    assert not any(label.startswith("ovruntime_deps/omni.fabric") for label in recorded_plugin_labels)
    assert not any(label.startswith("ovruntime_deps/usdrt") for label in recorded_plugin_labels)
    assert not any(label.startswith("ovruntime_deps/scenegraph") for label in recorded_plugin_labels)
    plugins = layout["install_dir"] / "plugins"
    # OmniClient, its transport, and the resolver all stay with the
    # application's OVStage. ovphysx neither stages nor loads them.
    assert not (plugins / "libomniclient.so").exists()
    assert not (plugins / "libomniverse_connection.so").exists()
    assert not (plugins / "ovstage-omniclient.version").exists()
    assert not (plugins / "libomni_usd_resolver.so").exists()
    assert not (plugins / "usd" / "omni_usd_resolver").exists()


def test_installed_cmake_config_omits_retired_gpu_plugin_path():
    repo_root = Path(__file__).resolve().parents[2]
    config_template = (repo_root / "cmake" / "ovphysxConfig.cmake.in").read_text()

    assert "${ovphysx_PLUGINS_DIR}/gpu" not in config_template


def test_package_deps_stages_no_usd_payload(tmp_path, monkeypatch):
    """REQ-PACKAGING-USDFREE-001 AC-4: no USD library, USD dependency closure, or
    USD plugin registry is staged. OVStage brings its own USD runtime.
    Schema definitions are application-owned too."""
    if platform.system() == "Windows":
        pytest.skip("package_deps layout test exercises POSIX symlink semantics")

    layout = _create_package_deps_layout(tmp_path)

    recorded_shared_lib_labels: list[str] = []
    _patch_package_deps_copy_steps(monkeypatch, recorded_shared_lib_labels=recorded_shared_lib_labels)

    success = package_deps_module.package_deps(
        manifest_path=layout["manifest_path"],
        build_dir=layout["build_dir"],
        install_dir=layout["install_dir"],
        ovruntime_install_dir=layout["ovruntime_root"] / "_install" / "release",
        ovstage_dir=layout["ovstage_dir"],
        ovstage_runtime_dir=layout["ovstage_dir"] / "bin",
        config="release",
        verbose=False,
    )

    assert success is True
    assert recorded_shared_lib_labels, "no shared-library copy step ran"
    assert not [label for label in recorded_shared_lib_labels if label.startswith("usd/")], recorded_shared_lib_labels
    plugins = layout["install_dir"] / "plugins"
    assert not list(plugins.glob("libov_*usd_ms.so*"))
    assert not list(plugins.glob("ov_*usd_ms.dll"))
    assert not (plugins / "usd").exists()
    for removed_helper in ("copy_usd_registry", "fixup_usd_plugin_info_library_paths", "_find_usd_monolith"):
        assert not hasattr(package_deps_module, removed_helper)


def test_package_deps_removes_obsolete_bundled_schemas(tmp_path, monkeypatch):
    """The SDK no longer delivers schemas, including on incremental installs."""
    if platform.system() == "Windows":
        pytest.skip("package_deps layout test exercises POSIX symlink semantics")

    layout = _create_package_deps_layout(tmp_path)
    _patch_package_deps_copy_steps(monkeypatch)

    obsolete = layout["install_dir"] / "schemas" / "physx"
    obsolete.mkdir(parents=True)
    (obsolete / "generatedSchema.usda").write_text("obsolete")

    success = package_deps_module.package_deps(
        manifest_path=layout["manifest_path"],
        build_dir=layout["build_dir"],
        install_dir=layout["install_dir"],
        ovruntime_install_dir=layout["ovruntime_root"] / "_install" / "release",
        ovstage_dir=layout["ovstage_dir"],
        ovstage_runtime_dir=layout["ovstage_dir"] / "bin",
        config="release",
        verbose=False,
    )

    assert success is True
    assert not obsolete.exists()
    assert not (layout["install_dir"] / "plugins" / "usd").exists()


def test_package_deps_errors_when_infra_payload_is_empty(tmp_path, monkeypatch):
    if platform.system() == "Windows":
        pytest.skip("package_deps layout test exercises POSIX symlink semantics")

    layout = _create_package_deps_layout(tmp_path)

    ovruntime_deps_root = (
        layout["ovruntime_root"]
        / "_build"
        / "target-deps"
        / "ovruntime_deps_release"
        / "_build"
        / package_deps_module._preferred_platform_build()
        / "release"
        / "plugins"
    )
    for child in list(ovruntime_deps_root.iterdir()):
        if child.is_dir():
            child.rmdir()

    monkeypatch.setattr(package_deps_module, "copy_carbonite_plugins", lambda *a, **k: 0)
    monkeypatch.setattr(package_deps_module, "_copy_plugin_libs", lambda *a, **k: 0)
    monkeypatch.setattr(package_deps_module, "_copy_shared_libs", lambda *a, **k: 0)

    with pytest.raises(RuntimeError, match="infrastructure plugin payload"):
        package_deps_module.package_deps(
            manifest_path=layout["manifest_path"],
            build_dir=layout["build_dir"],
            install_dir=layout["install_dir"],
            ovruntime_install_dir=layout["ovruntime_root"] / "_install" / "release",
            ovstage_dir=layout["ovstage_dir"],
            ovstage_runtime_dir=layout["ovstage_dir"] / "bin",
            config="release",
            verbose=False,
        )


def test_package_deps_does_not_require_omniclient_payload(tmp_path, monkeypatch):
    if platform.system() == "Windows":
        pytest.skip("package_deps layout test exercises POSIX symlink semantics")

    layout = _create_package_deps_layout(tmp_path)
    monkeypatch.setattr(package_deps_module, "copy_carbonite_plugins", lambda *a, **k: 0)
    monkeypatch.setattr(package_deps_module, "_copy_plugin_libs", lambda *a, **k: 0)
    monkeypatch.setattr(package_deps_module, "_copy_shared_libs", lambda *a, **k: 0)
    client_dir = layout["ovstage_dir"] / "bin" / "plugins" / "omni.client.lib"
    for client_file in client_dir.iterdir():
        client_file.unlink()
    client_dir.rmdir()

    success = package_deps_module.package_deps(
        manifest_path=layout["manifest_path"],
        build_dir=layout["build_dir"],
        install_dir=layout["install_dir"],
        ovruntime_install_dir=layout["ovruntime_root"] / "_install" / "release",
        ovstage_dir=layout["ovstage_dir"],
        ovstage_runtime_dir=layout["ovstage_dir"] / "bin",
        config="release",
        verbose=False,
    )

    assert success is True
    assert not (layout["install_dir"] / "plugins" / "libomniclient.so").exists()


@pytest.mark.parametrize(
    ("relative_path", "expected_error"),
    [
        (
            Path("bin/plugins/omni.usd_resolver/libomni_usd_resolver.so"),
            r"OVStage USD resolver payload is incomplete.*libomni_usd_resolver.so",
        ),
        (
            Path("bin/plugins/omni.usd_resolver/usd/omni_usd_resolver/resources/plugInfo.json"),
            r"OVStage USD resolver registry is missing",
        ),
    ],
)
def test_package_deps_rejects_incomplete_ovstage_runtime(tmp_path, monkeypatch, relative_path, expected_error):
    if platform.system() == "Windows":
        pytest.skip("package_deps layout test exercises Linux OVStage filenames")

    layout = _create_package_deps_layout(tmp_path)
    _patch_package_deps_copy_steps(monkeypatch)
    (layout["ovstage_dir"] / relative_path).unlink()

    with pytest.raises(RuntimeError, match=expected_error):
        package_deps_module.package_deps(
            manifest_path=layout["manifest_path"],
            build_dir=layout["build_dir"],
            install_dir=layout["install_dir"],
            ovruntime_install_dir=layout["ovruntime_root"] / "_install" / "release",
            ovstage_dir=layout["ovstage_dir"],
            ovstage_runtime_dir=layout["ovstage_dir"] / "bin",
            config="release",
            verbose=False,
        )


def test_merge_ovstage_notices_is_exact_and_idempotent(tmp_path):
    ovstage_dir = tmp_path / "ovstage"
    install_dir = tmp_path / "install"
    ovstage_dir.mkdir()
    install_dir.mkdir()
    notice = ovstage_dir / "THIRD-PARTY-NOTICES.txt"
    notice.write_bytes(b"exact-ovstage-notice\n")
    license_archive = tmp_path / "ovphysx-LICENSES.zip"
    with zipfile.ZipFile(license_archive, "w") as archive:
        archive.writestr("base-LICENSE.txt", "base")

    for _ in range(2):
        entries = package_deps_module.merge_ovstage_notices(
            ovstage_dir, ovstage_dir / "bin", license_archive, install_dir
        )
        assert entries == ["ovstage/THIRD-PARTY-NOTICES.txt"]

    with zipfile.ZipFile(license_archive) as archive:
        assert archive.namelist().count("ovstage/THIRD-PARTY-NOTICES.txt") == 1
        assert archive.read("ovstage/THIRD-PARTY-NOTICES.txt") == notice.read_bytes()
    assert (install_dir / "ovstage-THIRD-PARTY-NOTICES.txt").read_bytes() == notice.read_bytes()


def test_merge_ovstage_notices_accepts_legacy_package_layout(tmp_path):
    ovstage_dir = tmp_path / "ovstage"
    install_dir = tmp_path / "install"
    legacy_dir = ovstage_dir / "PACKAGE-LICENSES"
    legacy_dir.mkdir(parents=True)
    install_dir.mkdir()
    (legacy_dir / "rendering-LICENSES.txt").write_text("legacy-index")
    with zipfile.ZipFile(legacy_dir / "rendering-LICENSES.zip", "w") as archive:
        archive.writestr("omni-client-LICENSE.txt", "client-license")
    license_archive = tmp_path / "ovphysx-LICENSES.zip"
    with zipfile.ZipFile(license_archive, "w") as archive:
        archive.writestr("base-LICENSE.txt", "base")

    entries = package_deps_module.merge_ovstage_notices(ovstage_dir, ovstage_dir / "bin", license_archive, install_dir)

    assert entries == ["ovstage/rendering-LICENSES.txt", "ovstage/rendering-LICENSES.zip"]
    with zipfile.ZipFile(license_archive) as archive:
        assert archive.read("ovstage/rendering-LICENSES.txt") == b"legacy-index"
        assert archive.read("ovstage/rendering-LICENSES.zip") == (legacy_dir / "rendering-LICENSES.zip").read_bytes()


def test_nested_wheel_runtime_uses_matching_plugins_and_notices(tmp_path):
    ovstage_dir = tmp_path / "ovstage_pip"
    nested_root = ovstage_dir / "ovstage"
    runtime_dir = nested_root / "bin"
    plugins_dir = runtime_dir / "plugins"
    (plugins_dir / "omni.client.lib").mkdir(parents=True)
    (plugins_dir / "omni.usd_resolver").mkdir()

    # A competing outer layout must not redirect binary selection away from the
    # runtime directory CMake resolved.
    (ovstage_dir / "bin" / "plugins" / "omni.client.lib").mkdir(parents=True)
    (ovstage_dir / "bin" / "plugins" / "omni.usd_resolver").mkdir()
    assert package_deps_module._resolve_ovstage_plugins_dir(runtime_dir) == plugins_dir

    legacy_dir = nested_root / "PACKAGE-LICENSES"
    legacy_dir.mkdir()
    (legacy_dir / "rendering-LICENSES.txt").write_text("nested-wheel-index")
    with zipfile.ZipFile(legacy_dir / "rendering-LICENSES.zip", "w") as archive:
        archive.writestr("omni-client-LICENSE.txt", "nested-client-license")
    (ovstage_dir / "THIRD-PARTY-NOTICES.txt").write_text("wrong-outer-notice")

    install_dir = tmp_path / "install"
    install_dir.mkdir()
    license_archive = tmp_path / "ovphysx-LICENSES.zip"
    with zipfile.ZipFile(license_archive, "w") as archive:
        archive.writestr("base-LICENSE.txt", "base")

    entries = package_deps_module.merge_ovstage_notices(ovstage_dir, runtime_dir, license_archive, install_dir)

    assert entries == ["ovstage/rendering-LICENSES.txt", "ovstage/rendering-LICENSES.zip"]
    assert (install_dir / "ovstage-THIRD-PARTY-NOTICES.txt").read_text() == "nested-wheel-index"


def test_merge_ovstage_notices_rejects_empty_notice(tmp_path):
    ovstage_dir = tmp_path / "ovstage"
    install_dir = tmp_path / "install"
    ovstage_dir.mkdir()
    install_dir.mkdir()
    (ovstage_dir / "THIRD-PARTY-NOTICES.txt").write_text("")
    license_archive = tmp_path / "ovphysx-LICENSES.zip"
    with zipfile.ZipFile(license_archive, "w") as archive:
        archive.writestr("base-LICENSE.txt", "base")

    with pytest.raises(RuntimeError, match="empty files"):
        package_deps_module.merge_ovstage_notices(ovstage_dir, ovstage_dir / "bin", license_archive, install_dir)
