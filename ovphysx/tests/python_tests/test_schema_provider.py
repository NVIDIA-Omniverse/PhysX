# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-1 AC-2 AC-3

"""External schema discovery in fresh and relocated application installations."""

import importlib.util
import shutil
import sys
from importlib import metadata
from pathlib import Path

import pytest

_source = Path(__file__).resolve().parents[1] / "python_samples/schema_support/__init__.py"
_spec = importlib.util.spec_from_file_location("schema_support_under_test", _source)
provider = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(provider)


def _install(root, version="25.11.1"):
    info = root / f"physx_usd_schemas-{version}.dist-info"
    info.mkdir(parents=True)
    (info / "METADATA").write_text(
        f"Metadata-Version: 2.1\nName: physx-usd-schemas\nVersion: {version}\n", encoding="utf-8"
    )
    package = root / "physx_usd_schemas"
    package.mkdir()
    (package / "__init__.py").write_text("raise AssertionError('provider must not be imported')\n")
    for module in provider.SCHEMA_MODULES:
        resources = package / module / "resources"
        resources.mkdir(parents=True)
        (resources / "plugInfo.json").write_text('{"Plugins": []}')
        (resources / "generatedSchema.usda").write_text("#usda 1.0\n")
    return info


def test_fresh_and_relocated_schema_installation_does_not_import_usd(tmp_path, monkeypatch):
    root = tmp_path / "original"
    _install(root)
    monkeypatch.syspath_prepend(str(root))
    before = set(sys.modules)
    assert provider.physx_schema_paths() == [
        root / "physx_usd_schemas" / module / "resources" for module in provider.SCHEMA_MODULES
    ]
    relocated = tmp_path / "relocated installation"
    shutil.move(str(root), str(relocated))
    monkeypatch.syspath_prepend(str(relocated))
    assert all(path.is_relative_to(relocated) for path in provider.physx_schema_paths())
    assert not any(name == "pxr" or name.startswith(("pxr.", "physx_usd_schemas"))
                   for name in set(sys.modules) - before)


def test_missing_schema_package_reports_install_action(monkeypatch):
    def missing(_name):
        raise metadata.PackageNotFoundError("physx-usd-schemas")
    monkeypatch.setattr(provider.metadata, "distribution", missing)
    with pytest.raises(FileNotFoundError, match="Install physx-usd-schemas"):
        provider.physx_schema_paths()


@pytest.mark.parametrize("version", ["25.11.2", "26.1.0", "25.11.1+local"])
def test_provider_version_is_selected_by_application_dependencies(tmp_path, monkeypatch, version):
    _install(tmp_path, version)
    monkeypatch.syspath_prepend(str(tmp_path))
    assert all(path.is_relative_to(tmp_path) for path in provider.physx_schema_paths())


@pytest.mark.parametrize("filename", ["plugInfo.json", "generatedSchema.usda"])
def test_incomplete_schema_package_fails_before_registration(tmp_path, monkeypatch, filename):
    _install(tmp_path)
    monkeypatch.syspath_prepend(str(tmp_path))
    (tmp_path / "physx_usd_schemas/OmniUsdPhysicsDeformableSchema/resources" / filename).unlink()
    with pytest.raises(FileNotFoundError, match="Incomplete physx-usd-schemas"):
        provider.physx_schema_paths()


def test_dependency_resolver_rejects_provider_below_manifest_minimum(tmp_path):
    """Version policy is enforced while installing, without a runtime equality check."""
    import os
    import re
    import subprocess
    from zipfile import ZipFile

    wheel = tmp_path / "physx_usd_schemas-25.11.0-py3-none-any.whl"
    with ZipFile(wheel, "w") as archive:
        archive.writestr("physx_usd_schemas-25.11.0.dist-info/METADATA",
                         "Metadata-Version: 2.1\nName: physx-usd-schemas\nVersion: 25.11.0\n")
        archive.writestr("physx_usd_schemas-25.11.0.dist-info/WHEEL",
                         "Wheel-Version: 1.0\nRoot-Is-Purelib: true\nTag: py3-none-any\n")
    uv = shutil.which("uv")
    assert uv, "uv is required by the dependency-resolution check"
    venv = tmp_path / "venv"
    subprocess.run([uv, "venv", "--python", sys.executable, str(venv)], check=True,
                   capture_output=True, text=True, timeout=30)
    python = venv / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    manifest = Path(__file__).with_name("pyproject.toml").read_text(encoding="utf-8")
    constraint = re.search(r'"(physx-usd-schemas[^"]+)"', manifest).group(1)
    result = subprocess.run([
        uv, "pip", "install", "--python", str(python), "--no-index",
        "--find-links", str(tmp_path), constraint,
    ], capture_output=True, text=True, timeout=30)
    assert result.returncode != 0
    assert constraint in result.stderr
    assert "No solution found" in result.stderr


def test_real_provider_registration_and_fallbacks_after_relocation(tmp_path):
    """Fresh processes resolve real external data and populate schema fallbacks."""
    import os
    import subprocess

    package = metadata.distribution("physx-usd-schemas")
    root = tmp_path / "fresh"
    shutil.copytree(package.locate_file("physx_usd_schemas"), root / "physx_usd_schemas")
    info = f"physx_usd_schemas-{package.version}.dist-info"
    shutil.copytree(package.locate_file(info), root / info)
    scene = tmp_path / "fallback.usda"
    scene.write_text('''#usda 1.0
 def Xform "Body" (
     prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI"]
 ) {}
''', encoding="utf-8")
    script = r'''
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from schema_support import physx_schema_paths
paths = physx_schema_paths()
assert all(p.is_relative_to(Path(sys.argv[1])) for p in paths), paths
assert "pxr" not in sys.modules and "physx_usd_schemas" not in sys.modules
import ovstage
ovstage.population.register_usd_schemas([str(p) for p in paths])
with ovstage.Stage("external-schema-fallback") as stage:
    ovstage.population.open_usd(stage, sys.argv[2], ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
    stage.advance_write_floor(ordinal=1).wait()
    with ovstage.PathDictionary(stage) as dictionary:
        token = dictionary.intern_token("physxRigidBody:linearDamping")
        with dictionary.create_path_list_from_strings(["/Body"]) as path_list:
            with stage.query_from_path_list(path_list) as query:
                values = []
                with stage.read_attributes(query, [token], ovstage.OrdinalRange.latest(1)) as read:
                    read.wait()
                    for group in read.groups():
                        with group:
                            for index in range(group.tensor_count):
                                values.extend(group.array(index).reshape(-1).tolist())
                assert values == [0.0], values
'''
    env = dict(os.environ)
    env.pop("PXR_PLUGINPATH_NAME", None)
    env.pop("OV_PXR_PLUGINPATH_2511", None)
    for location in (root, tmp_path / "relocated provider"):
        if location != root:
            root.rename(location)
        result = subprocess.run([sys.executable, "-c", script, str(location), str(scene)],
                                env=env, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout + result.stderr
