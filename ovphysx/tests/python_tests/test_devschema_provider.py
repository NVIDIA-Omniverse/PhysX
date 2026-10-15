# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-6

"""A devschema build must drive native and Python consumers with its local data."""

import hashlib
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
from zipfile import ZipFile

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def installer():
    spec = importlib.util.spec_from_file_location(
        "schema_installer", ROOT / "tests/c_samples/common/fetch_physx_schemas.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _provider(root, version="26.1.0", name="physx-usd-schemas"):
    info = root / f"physx_usd_schemas-{version}.dist-info"
    info.mkdir(parents=True)
    (info / "METADATA").write_text(
        f"Metadata-Version: 2.1\nName: {name}\nVersion: {version}\n", encoding="utf-8"
    )
    for module in ("PhysxSchema", "OmniUsdPhysicsDeformableSchema"):
        resources = root / "physx_usd_schemas" / module / "resources"
        resources.mkdir(parents=True)
        (resources / "plugInfo.json").write_text('{"Plugins": []}', encoding="utf-8")
        (resources / "generatedSchema.usda").write_text("#usda 1.0\n# local edit\n", encoding="utf-8")


def _provider_wheel(tmp_path, name="physx-usd-schemas"):
    tree = tmp_path / "tree"
    _provider(tree, name=name)
    wheel = tmp_path / "provider.whl"
    with ZipFile(wheel, "w") as archive:
        for path in tree.rglob("*"):
            if path.is_file():
                archive.write(path, path.relative_to(tree).as_posix())
    return wheel, hashlib.sha256(wheel.read_bytes()).hexdigest()


def _native_driver(tmp_path, local=None, build_dir="_build"):
    project = tmp_path / "project"
    build = project / build_dir
    build.mkdir(parents=True)
    if build_dir != "_build":
        default_build = project / "_build"
        default_build.mkdir()
        (default_build / "ovphysx_dev_schema.stamp").write_text("stale-default-provider", encoding="utf-8")
    if local is not None:
        (build / "ovphysx_dev_schema.stamp").write_text(local.as_posix() + "\n", encoding="utf-8")
    # A selected local provider must bypass this offline download stand-in.
    fetch = project / "tests/c_samples/common/fetch_physx_schemas.py"
    fetch.parent.mkdir(parents=True)
    fetch.write_text("import sys\nfrom pathlib import Path\nPath('published-fetch-ran').touch()\n"
                     "print(Path(sys.argv[sys.argv.index('--cache-dir') + 1]) / 'physx-usd-schemas/locked-digest')\n",
                     encoding="utf-8")
    script = tmp_path / "select.cmake"
    output = tmp_path / "provider.txt"
    pythonpath = tmp_path / "pythonpath.txt"
    script.write_text(f'''
cmake_minimum_required(VERSION 3.18)
set(PROJECT_ROOT "{project.as_posix()}")
set(BUILD_PATH "{build.as_posix()}")
set(Python3_EXECUTABLE "{Path(sys.executable).as_posix()}")
include("{ROOT.as_posix()}/scripts/fetch_test_schemas.cmake")
file(WRITE "{output.as_posix()}" "$ENV{{PHYSX_USD_SCHEMAS_ROOT}}")
file(WRITE "{pythonpath.as_posix()}" "$ENV{{PYTHONPATH}}")
''', encoding="utf-8")
    cmake = shutil.which("cmake")
    assert cmake, "CMake is required by the devschema integration checks"
    result = subprocess.run([cmake, "-P", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=30)
    return result, output, pythonpath


@pytest.mark.parametrize("build_dir", ["_build", "custom build"])
def test_native_and_python_drivers_select_local_schema_data(tmp_path, build_dir):
    local = tmp_path / "local provider"
    _provider(local)
    result, output, pythonpath = _native_driver(tmp_path, local, build_dir)
    assert result.returncode == 0, result.stdout + result.stderr
    assert Path(output.read_text()) == local
    assert not (tmp_path / "published-fetch-ran").exists()
    env = dict(os.environ, PYTHONPATH=pythonpath.read_text())
    probe = subprocess.run([
        sys.executable, "-c",
        "import importlib.metadata as m, sys; "
        "print(m.distribution('physx-usd-schemas').locate_file('')); "
        "assert 'pxr' not in sys.modules and 'physx_usd_schemas' not in sys.modules",
    ], env=env, capture_output=True, text=True, timeout=30)
    assert probe.returncode == 0, probe.stderr
    assert Path(probe.stdout.strip()) == local


@pytest.mark.parametrize("build_dir", ["_build", "custom build"])
def test_missing_local_provider_does_not_fall_back_to_published_data(tmp_path, build_dir):
    result, _, _ = _native_driver(tmp_path, tmp_path / "missing local provider", build_dir)
    assert result.returncode != 0
    assert "devschema" in (result.stdout + result.stderr).lower()
    assert not (tmp_path / "published-fetch-ran").exists()


@pytest.mark.parametrize("build_dir", ["_build", "custom build"])
def test_non_devschema_driver_keeps_published_provider(tmp_path, build_dir):
    result, output, _ = _native_driver(tmp_path, build_dir=build_dir)
    assert result.returncode == 0, result.stdout + result.stderr
    assert Path(output.read_text()).parts[-2:] == ("physx-usd-schemas", "locked-digest")
    assert (tmp_path / "published-fetch-ran").is_file()


@pytest.mark.parametrize("build_dir", ["_build", "custom build", "absolute"])
def test_focused_pytest_wrapper_uses_local_provider(tmp_path, monkeypatch, build_dir):
    spec = importlib.util.spec_from_file_location("devschema_pytest_runner", ROOT / "scripts/run_uv_pytest.py")
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    project = tmp_path / "project"
    tests = project / "tests/python_tests"
    tests.mkdir(parents=True)
    (tests / "pyproject.toml").write_text("", encoding="utf-8")
    if build_dir == "absolute":
        build_dir = str(tmp_path / "absolute build")
    monkeypatch.setenv("OVPHYSX_BUILD_DIR", build_dir)
    local = project / build_dir / "test-deps/local provider"
    _provider(local)
    (project / build_dir / "ovphysx_dev_schema.stamp").write_text(local.as_posix(), encoding="utf-8")
    if build_dir != "_build":
        (project / "_build").mkdir()
        (project / "_build/ovphysx_dev_schema.stamp").write_text("stale-default-provider", encoding="utf-8")
    monkeypatch.setattr(runner, "__file__", str(project / "scripts/run_uv_pytest.py"))
    monkeypatch.setattr(sys, "argv", ["run_uv_pytest.py", str(tests), "test_lifecycle.py"])
    calls = []
    monkeypatch.setattr(runner.subprocess, "call", lambda cmd, **kwargs: calls.append((cmd, kwargs)) or 0)
    assert runner.main() == 0
    cmd, kwargs = calls.pop()
    assert "--locked" in cmd
    assert Path(kwargs["env"]["PYTHONPATH"].split(os.pathsep)[0]) == local
    assert Path(kwargs["env"]["PHYSX_USD_SCHEMAS_ROOT"]) == local


@pytest.mark.parametrize("source", ["default", "argument", "environment", "argument-over-environment"])
def test_cmake_build_directory_selection(tmp_path, monkeypatch, source):
    build = tmp_path / "custom build"
    monkeypatch.delenv("OVPHYSX_BUILD_DIR", raising=False)
    if "environment" in source:
        monkeypatch.setenv("OVPHYSX_BUILD_DIR", str(build if source == "environment" else tmp_path / "wrong build"))
    output = tmp_path / "build-path.txt"
    script = tmp_path / "build-path.cmake"
    # Path selection needs no host-tool discovery from the full build setup.
    script.write_text(f'''
cmake_minimum_required(VERSION 3.18)
set(PROJECT_ROOT "{ROOT.as_posix()}")
include("{ROOT.as_posix()}/scripts/build_directory.cmake")
file(WRITE "{output.as_posix()}" "${{BUILD_PATH}}")
''', encoding="utf-8")
    args = [f"-DBUILD_DIR={build}"] if source.startswith("argument") else []
    result = subprocess.run([shutil.which("cmake"), *args, "-P", str(script)],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert Path(output.read_text()) == (ROOT / "_build" if source == "default" else build)


def test_rebuilding_same_version_refreshes_the_external_provider(tmp_path):
    """Regenerate a cached tree before wheel extraction after a local source edit."""
    project = tmp_path / "ovphysx"
    common = project / "tests/c_samples/common"
    common.mkdir(parents=True)
    for filename in ("fetch_physx_schemas.py", "physx_schemas.lock.json"):
        shutil.copyfile(ROOT / "tests/c_samples/common" / filename, common / filename)
    schema = tmp_path / "schemas/physx"
    (schema / "scripts").mkdir(parents=True)
    (schema / "generate.py").write_text('''
import json
from pathlib import Path
root = Path(__file__).resolve().parent
for module in ("PhysxSchema", "OmniUsdPhysicsDeformableSchema"):
    resources = root / "_build/schema/share/usd/plugins" / module / "resources"
    resources.mkdir(parents=True, exist_ok=True)
    (resources / "plugInfo.json").write_text(json.dumps({"Plugins": [{
        "Name": module, "Type": "resource", "ResourcePath": "resources", "Info": {"Types": {"Test": {}}}
    }]}))
    (resources / "generatedSchema.usda").write_bytes((root / "local.usda").read_bytes())
''', encoding="utf-8")
    suffix = ".bat" if os.name == "nt" else ".sh"
    command = f'"{sys.executable}" "{schema / "generate.py"}"'
    builder = schema / f"build{suffix}"
    builder.write_text(("@echo off\n" if os.name == "nt" else "#!/bin/sh\nexec ") + command + "\n",
                       encoding="utf-8")
    builder.chmod(0o755)
    # Stand in for the schema wheel builder; the handoff must use its exact bytes.
    (schema / "scripts/build_wheel.py").write_text('''
from pathlib import Path
from zipfile import ZipFile
root = Path(__file__).resolve().parents[1]
(root / "_dist").mkdir(exist_ok=True)
with ZipFile(root / "_dist/physx_usd_schemas-26.1.0-py3-none-any.whl", "w") as wheel:
    wheel.writestr("physx_usd_schemas-26.1.0.dist-info/METADATA",
                  "Metadata-Version: 2.1\\nName: physx-usd-schemas\\nVersion: 26.1.0\\n")
    for module in ("PhysxSchema", "OmniUsdPhysicsDeformableSchema"):
        prefix = f"physx_usd_schemas/{module}/resources/"
        wheel.writestr(prefix + "plugInfo.json", '{"Plugins": []}')
        generated = root / "_build/schema/share/usd/plugins" / module / "resources/generatedSchema.usda"
        wheel.writestr(prefix + "generatedSchema.usda", generated.read_bytes())
''', encoding="utf-8")
    build = tmp_path / "ovphysx-build"
    build.mkdir()
    stamp = build / "ovphysx_dev_schema.stamp"
    script = tmp_path / "build-provider.cmake"
    build_script = (ROOT / "scripts/build.cmake").read_text(encoding="utf-8")
    start = build_script.index("if(DEV_SCHEMA)\n", build_script.index("# Build a separate local provider wheel"))
    end = build_script.index("\n# ============================================================================", start)
    script.write_text(f'''
cmake_minimum_required(VERSION 3.18)
set(PROJECT_ROOT "{project.as_posix()}")
set(BUILD_PATH "{build.as_posix()}")
set(DEV_SCHEMA ON)
set(SCRIPT_DIR "{ROOT.as_posix()}/scripts")
set(SCRIPT_SUFFIX "{suffix}")
set(TARGET_PYTHON "{Path(sys.executable).as_posix()}")
set(OVPHYSX_DEV_SCHEMA_STAMP "{stamp.as_posix()}")
''' + build_script[start:end], encoding="utf-8")
    providers = []
    for content in ("#usda 1.0\n# local edit one\n", "#usda 1.0\n# local edit two\n"):
        (schema / "local.usda").write_text(content, encoding="utf-8")
        result = subprocess.run([shutil.which("cmake"), "-P", str(script)],
                                capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        provider = Path(stamp.read_text().strip())
        assert provider.is_relative_to(build / "test-deps")
        assert (provider / "physx_usd_schemas/PhysxSchema/resources/generatedSchema.usda").read_text() == content
        providers.append(provider)
    assert providers[0] != providers[1]
    assert not (build / "_install").exists()


def test_local_wheel_override_accepts_new_version_and_keeps_checksum_validation(tmp_path, installer):
    wheel, digest = _provider_wheel(tmp_path)
    with pytest.raises(RuntimeError, match="SHA-256 mismatch"):
        installer.install(tmp_path / "default", wheel=wheel)
    destination = installer.install(tmp_path / "custom", wheel=wheel, sha256=digest)
    assert (destination / "physx_usd_schemas-26.1.0.dist-info/METADATA").is_file()
    # A cached installation must also accept the application's selected version.
    assert installer.install(destination, wheel=wheel, sha256=digest) == destination
    (destination / "physx_usd_schemas/PhysxSchema/resources/generatedSchema.usda").unlink()
    with pytest.raises(RuntimeError, match="Incomplete schema provider"):
        installer.install(destination, wheel=wheel, sha256=digest)


def test_local_wheel_override_rejects_wrong_package_identity(tmp_path, installer):
    wheel, digest = _provider_wheel(tmp_path, name="unrelated-schema-package")
    with pytest.raises(RuntimeError, match="Incompatible schema provider"):
        installer.install(tmp_path / "provider", wheel=wheel, sha256=digest)
    assert not (tmp_path / "provider").exists()


@pytest.mark.skipif(os.name != "nt", reason="Windows path limit")
def test_provider_install_and_cache_reuse_beyond_max_path(tmp_path, monkeypatch, installer):
    import builtins
    import io

    wheel, digest = _provider_wheel(tmp_path)
    destination = tmp_path / ("deep-" + "x" * max(1, 210 - len(str(tmp_path)))) / "provider"
    original_open = builtins.open

    def limited_open(file, *args, **kwargs):
        # Reproduce MAX_PATH even when the host enables long paths.
        if not isinstance(file, int):
            path = os.fsdecode(file)
            if len(path) >= 260 and not path.startswith("\\\\?\\"):
                raise FileNotFoundError(2, "MAX_PATH exceeded", path)
        return original_open(file, *args, **kwargs)

    monkeypatch.setattr(builtins, "open", limited_open)
    monkeypatch.setattr(io, "open", limited_open)
    assert installer.install(destination, wheel=wheel, sha256=digest) == destination
    installed = Path("\\\\?\\" + str(destination))
    for module in ("PhysxSchema", "OmniUsdPhysicsDeformableSchema"):
        resource = installed / "physx_usd_schemas" / module / "resources/generatedSchema.usda"
        assert len(str(resource)) - 4 >= 260
        assert resource.read_text() == "#usda 1.0\n# local edit\n"
    wheel.unlink()
    assert installer.install(destination, wheel=wheel, sha256=digest) == destination
