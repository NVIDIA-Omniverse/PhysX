# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OVD-INVESTIGATION-001
# @covers AC-5

"""Run the sample driver with offline stand-ins and observe its workflow argv."""

import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("explicit,environment,require_cuda,expected", [
    ("ON", None, False, ["--cpu-only"]),
    (None, "ON", False, ["--cpu-only"]),
    ("OFF", "ON", False, []),
    (None, None, True, ["--require-gpu"]),
    ("ON", None, True, ["--cpu-only"]),
    (None, "ON", True, ["--cpu-only"]),
    ("OFF", "ON", True, ["--require-gpu"]),
    (None, None, False, []),
])
@pytest.mark.parametrize("inspector_path", ["absolute", "relative", "relative-other-cwd"])
def test_sample_driver_routes_workflow_mode(tmp_path, explicit, environment, require_cuda, expected, inspector_path):
    cmake = shutil.which("cmake")
    assert cmake, "CMake is required by the sample driver argument checks"
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    shutil.copyfile(ROOT / "scripts/test_python_samples.cmake", scripts / "test_python_samples.cmake")
    shutil.copyfile(ROOT / "scripts/build_directory.cmake", scripts / "build_directory.cmake")
    samples = tmp_path / "tests/python_samples"
    samples.mkdir(parents=True)
    for sample in (ROOT / "tests/python_samples").glob("*.py"):
        (samples / sample.name).touch()
    (tmp_path / "_dist").mkdir()
    (tmp_path / "_dist/ovphysx-fixture.whl").touch()
    inspector = tmp_path / "SDK tools/inspector"
    inspector.parent.mkdir()
    inspector.touch()
    launch_dir = tmp_path / "launch" if inspector_path == "relative-other-cwd" else tmp_path
    launch_dir.mkdir(exist_ok=True)
    installed_sample = tmp_path / "installed_sample.py"
    installed_sample.touch()
    output = tmp_path / "workflow-argv.txt"
    (scripts / "dev_schema_provider.cmake").touch()
    (scripts / "ovstage_index.cmake").write_text(
        "function(ovphysx_ovstage_index_env output)\nset(${output} \"\" PARENT_SCOPE)\nendfunction()\n",
        encoding="utf-8",
    )
    # Dependency provisioning and unrelated sample launches are offline stand-ins.
    # The real driver still chooses and submits the workflow arguments.
    (scripts / "build_common.cmake").write_text(f'''
cmake_minimum_required(VERSION 3.18)
include("${{CMAKE_CURRENT_LIST_DIR}}/build_directory.cmake")
set(OVPHYSX_UV_COMMAND offline-uv)
macro(ensure_uv_managed_python version output)
    set(${{output}} offline-python)
endmacro()
function(execute_process)
    cmake_parse_arguments(CALL "ERROR_QUIET;OUTPUT_STRIP_TRAILING_WHITESPACE;ECHO_OUTPUT_VARIABLE;ECHO_ERROR_VARIABLE"
        "WORKING_DIRECTORY;RESULT_VARIABLE;OUTPUT_VARIABLE;ERROR_VARIABLE;TIMEOUT" "COMMAND" ${{ARGN}})
    if(CALL_RESULT_VARIABLE)
        set(${{CALL_RESULT_VARIABLE}} 0 PARENT_SCOPE)
    endif()
    if(CALL_OUTPUT_VARIABLE)
        set(${{CALL_OUTPUT_VARIABLE}} "{installed_sample.as_posix()}" PARENT_SCOPE)
    endif()
    foreach(argument IN LISTS CALL_COMMAND)
        if(argument MATCHES "omnipvd_inspection[.]py" OR argument MATCHES "test_workflow[.]py$")
            list(FIND CALL_COMMAND "UV_PROJECT_ENVIRONMENT=${{_VENV_DIR}}" environment_index)
            if(environment_index LESS 0)
                message(FATAL_ERROR "OVD workflow command did not select its sample environment")
            endif()
            file(APPEND "{output.as_posix()}.environments" "${{_VENV_DIR}}\n")
        endif()
        if(argument MATCHES "test_workflow[.]py$")
            set(flags)
            foreach(option IN LISTS CALL_COMMAND)
                if(option STREQUAL "--cpu-only" OR option STREQUAL "--require-gpu")
                    list(APPEND flags "${{option}}")
                endif()
            endforeach()
            file(WRITE "{output.as_posix()}" "${{flags}}")
            list(FIND CALL_COMMAND "--inspector" inspector_index)
            math(EXPR inspector_index "${{inspector_index}} + 1")
            list(GET CALL_COMMAND ${{inspector_index}} inspector)
            file(WRITE "{output.as_posix()}.inspector" "${{CALL_WORKING_DIRECTORY}}\n${{inspector}}")
        endif()
    endforeach()
endfunction()
''', encoding="utf-8")
    env = os.environ.copy()
    for variable in ("OVPHYSX_SKIP_GPU_PASSES", "OVPHYSX_TEST_REQUIRE_CUDA"):
        env.pop(variable, None)
    env["OVPHYSX_OVD_INSPECTOR"] = (str(inspector) if inspector_path == "absolute"
                                   else os.path.relpath(inspector, launch_dir))
    if environment is not None:
        env["OVPHYSX_SKIP_GPU_PASSES"] = environment
    if require_cuda:
        env["OVPHYSX_TEST_REQUIRE_CUDA"] = "1"
    command = [cmake]
    if explicit is not None:
        command.append(f"-DOVPHYSX_SKIP_GPU_PASSES={explicit}")
    result = subprocess.run(command + ["-P", str(scripts / "test_python_samples.cmake")],
                            cwd=launch_dir, env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert output.is_file(), "Driver did not invoke the installed workflow"
    flags = output.read_text(encoding="utf-8")
    assert (flags.split(";") if flags else []) == expected
    child_cwd, child_inspector = Path(str(output) + ".inspector").read_text(encoding="utf-8").splitlines()
    assert (Path(child_cwd) / child_inspector).resolve(strict=True) == inspector.resolve()
    environments = Path(str(output) + ".environments").read_text(encoding="utf-8").splitlines()
    assert environments == [str(tmp_path / "_build/venv/tests/python_samples/python-3.12").replace("\\", "/")] * 2


@pytest.fixture
def inspector_project(tmp_path):
    """Keep directory resolution and child processes real; replace SDK inputs."""
    project = tmp_path / "ovphysx"
    scripts = project / "scripts"
    scripts.mkdir(parents=True)
    for name in ("build_common.cmake", "build_directory.cmake", "host_toolchain.cmake",
                 "crossplatform_helpers.cmake", "build_ovd_inspector.cmake", "test_cpp_samples.cmake",
                 "test_python_samples.cmake"):
        shutil.copyfile(ROOT / "scripts" / name, scripts / name)
    (project / "cmake").mkdir()
    shutil.copyfile(ROOT / "cmake/RequireCMakeVersion.cmake", project / "cmake/RequireCMakeVersion.cmake")
    # These dependency inputs otherwise fetch schemas or require an installed
    # OVStage. Neither participates in inspector provisioning/directory selection.
    (scripts / "fetch_test_schemas.cmake").touch()
    (scripts / "dev_schema_provider.cmake").touch()
    dependency = project / "ovruntime/cmake"
    dependency.mkdir(parents=True)
    (dependency / "OvstageDependency.cmake").touch()
    config = project / "_install/lib/cmake/ovphysx"
    config.mkdir(parents=True)
    (config / "ovphysxConfig.cmake").touch()
    (project / "tests/c_samples").mkdir(parents=True)
    # Header compilation belongs to the installed SDK tests, not directory routing.
    headers = project / "tests/installed_headers"
    headers.mkdir()
    (headers / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.21)\nproject(offline_headers LANGUAGES NONE)\n", encoding="utf-8")
    tool = tmp_path / "physx/pvddom"
    tool.mkdir(parents=True)
    # The real build script must configure, build and install this project.
    # The installed marker is never executed and needs no compiler or runtime.
    (tool / "CMakeLists.txt").write_text('''
cmake_minimum_required(VERSION 3.21)
project(offline_inspector LANGUAGES NONE)
set(name ovd-inspect)
if(WIN32)
    string(APPEND name ".exe")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${name}" "offline inspector installation")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${name}" DESTINATION bin)
''', encoding="utf-8")
    return project


def inspector_build_env():
    env = os.environ.copy()
    for name in ("OVPHYSX_BUILD_DIR", "OVPHYSX_OVD_INSPECTOR", "CMAKE_GENERATOR",
                 "CMAKE_GENERATOR_PLATFORM", "CMAKE_GENERATOR_TOOLSET"):
        env.pop(name, None)
    # Ninja supports compiler-free CMake projects on both Linux and Windows.
    ninja = shutil.which("ninja")
    if ninja is None:
        name = "ninja.exe" if sys.platform == "win32" else "ninja"
        ninja = str(ROOT / "_build/host-deps/ninja" / name)
    assert Path(ninja).is_file(), "Ninja is required for inspector provisioning checks"
    env["CMAKE_GENERATOR"] = "Ninja"
    env["PATH"] = str(Path(ninja).parent) + os.pathsep + env.get("PATH", "")
    return env


@pytest.mark.parametrize("platform,name", [("linux", "ninja"), ("win32", "ninja.exe")])
def test_inspector_ninja_uses_packaged_platform_executable(tmp_path, monkeypatch, platform, name):
    monkeypatch.setattr(sys, "platform", platform)
    monkeypatch.setattr(sys.modules[__name__], "ROOT", tmp_path)
    monkeypatch.setenv("PATH", "")
    ninja = tmp_path / "_build/host-deps/ninja" / name
    ninja.parent.mkdir(parents=True)
    ninja.touch()
    env = inspector_build_env()
    assert env["CMAKE_GENERATOR"] == "Ninja"
    assert env["PATH"].split(os.pathsep)[0] == str(ninja.parent)
    assert os.environ["PATH"] == ""


def test_inspector_ninja_prefers_path_executable(tmp_path, monkeypatch):
    project = tmp_path / "packaged-tools"
    monkeypatch.setattr(sys.modules[__name__], "ROOT", project)
    ninja = tmp_path / ("ninja.exe" if os.name == "nt" else "ninja")
    ninja.touch()
    ninja.chmod(0o755)
    packaged = project / "_build/host-deps/ninja" / ninja.name
    packaged.parent.mkdir(parents=True)
    packaged.touch()
    packaged.chmod(0o755)
    monkeypatch.setenv("PATH", str(tmp_path))
    assert inspector_build_env()["PATH"].split(os.pathsep)[0] == str(tmp_path)


@pytest.mark.parametrize("platform,wrong_name", [("linux", "ninja.exe"), ("win32", "ninja")])
def test_inspector_ninja_missing_platform_tool_is_diagnostic(tmp_path, monkeypatch, platform, wrong_name):
    monkeypatch.setattr(sys, "platform", platform)
    monkeypatch.setattr(sys.modules[__name__], "ROOT", tmp_path)
    monkeypatch.setenv("PATH", "")
    wrong_tool = tmp_path / "_build/host-deps/ninja" / wrong_name
    wrong_tool.parent.mkdir(parents=True)
    wrong_tool.touch()
    with pytest.raises(AssertionError, match="Ninja is required for inspector provisioning checks"):
        inspector_build_env()


def run_cmake(arguments, project, env):
    cmake = shutil.which("cmake")
    assert cmake, "CMake is required for inspector provisioning checks"
    result = subprocess.run([cmake, *arguments], cwd=project, env=env,
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr


def assert_inspector_installed(project, build_dir):
    name = "ovd-inspect.exe" if os.name == "nt" else "ovd-inspect"
    installed = project / build_dir / "ovd-inspector-install/bin" / name
    assert installed.is_file(), f"Inspector was not installed in selected build directory: {installed}"
    assert installed.read_text(encoding="utf-8") == "offline inspector installation"
    if build_dir != "_build":
        assert not (project / "_build/ovd-inspector-install").exists(), "Default build was provisioned instead"


@pytest.mark.parametrize("path_source", ["hostile", "absent", "fallback"])
def test_copied_ninja_source_selection(inspector_project, tmp_path, monkeypatch, path_source):
    original = tmp_path / "original"
    name = "ninja.exe" if os.name == "nt" else "ninja"
    supplied = original / "_build/host-deps/ninja" / name
    supplied.parent.mkdir(parents=True)
    if path_source != "fallback":
        supplied.write_bytes(b"packaged standalone fixture")
    fallback = tmp_path / "path-tools" / name
    fallback.parent.mkdir()
    fallback.write_bytes(b"PATH shim fixture")
    monkeypatch.setattr(sys.modules[__name__], "ROOT", original)
    monkeypatch.setenv("PATH", str(fallback.parent) if path_source != "absent" else "")
    monkeypatch.setattr(shutil, "which", lambda tool, **kw: (str(fallback) if str(fallback.parent) in os.environ["PATH"] else None) if tool == "ninja" else "/usr/bin/uname")
    monkeypatch.setattr(subprocess, "run", lambda *args, **kw: subprocess.CompletedProcess(args[0], 0, "1.10.2", ""))
    env = copied_ninja_env(inspector_project, monkeypatch)
    copied = inspector_project / "_build/host-deps/ninja" / name
    assert copied.read_bytes() == (b"PATH shim fixture" if path_source == "fallback" else b"packaged standalone fixture")
    assert env["PATH"].split(os.pathsep)[0] == str(copied.parent)


def test_copied_ninja_probe_diagnostic(inspector_project, tmp_path, monkeypatch):
    name = "ninja.exe" if os.name == "nt" else "ninja"
    original = tmp_path / "original"
    supplied = original / "_build/host-deps/ninja" / name
    supplied.parent.mkdir(parents=True)
    supplied.write_bytes(b"fixture, never executed")
    monkeypatch.setattr(sys.modules[__name__], "ROOT", original)
    monkeypatch.setattr(subprocess, "run", lambda *args, **kw: subprocess.CompletedProcess(args[0], 7, "shim stdout", "shim stderr"))
    with pytest.raises(AssertionError) as error:
        copied_ninja_env(inspector_project, monkeypatch)
    for text in (str(supplied), str(inspector_project / "_build/host-deps/ninja" / name), "exit=7", "shim stdout", "shim stderr"):
        assert text in str(error.value)


def copied_ninja_env(project, monkeypatch):
    name = "ninja.exe" if os.name == "nt" else "ninja"
    source = ROOT / "_build/host-deps/ninja" / name
    if not source.is_file():
        found = shutil.which("ninja")
        assert found, "Ninja is required for inspector provisioning checks"
        source = Path(found)
    packaged = project / "_build/host-deps/ninja" / name
    packaged.parent.mkdir(parents=True)
    shutil.copy2(source, packaged)
    # Retain the platform query / command shell, without CMake, Ninja or compilers.
    if os.name == "nt":
        clean_path = str(Path(os.environ["SystemRoot"]) / "System32")
    else:
        tools = project / "platform-tools"
        tools.mkdir()
        uname = shutil.which("uname")
        assert uname, "uname is required by the CMake platform query"
        (tools / "uname").symlink_to(uname)
        clean_path = str(tools)
    monkeypatch.setenv("PATH", clean_path)
    monkeypatch.setattr(sys.modules[__name__], "ROOT", project)
    assert shutil.which("ninja") is None
    env = inspector_build_env()
    try:
        probe = subprocess.run([str(packaged), "--version"], env=env, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as error:
        pytest.fail(f"Copied Ninja probe failed: source={source}, copied={packaged}: {error}")
    assert probe.returncode == 0, (f"Copied Ninja is not self-contained: source={source}, copied={packaged}, "
                                   f"exit={probe.returncode}, stdout={probe.stdout!r}, stderr={probe.stderr!r}")
    return env


def test_inspector_provisions_with_packaged_ninja_off_path(inspector_project, monkeypatch):
    cmake = shutil.which("cmake")
    assert cmake, "CMake is required for inspector provisioning checks"
    project = inspector_project
    env = copied_ninja_env(project, monkeypatch)
    result = subprocess.run([cmake, "-DBUILD_DIR=_build/custom build", "-P", "scripts/build_ovd_inspector.cmake"],
                            cwd=project, env=env, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    assert_inspector_installed(project, "_build/custom build")
    assert str(project / "_build/host-deps/ninja") == env["PATH"].split(os.pathsep)[0]


def test_missing_inspector_hint_provisions_selected_build_with_spaces(inspector_project):
    project = inspector_project
    env = inspector_build_env()
    env["OVPHYSX_BUILD_DIR"] = "_build/environment build"
    cmake = shutil.which("cmake")
    assert cmake, "CMake is required for inspector provisioning checks"
    result = subprocess.run([cmake, "-DBUILD_DIR=_build/custom build", "-P", "scripts/test_python_samples.cmake"],
                            cwd=project, env=env, capture_output=True, text=True, timeout=60)
    assert result.returncode != 0
    hint = re.search(r"Run (cmake\b.*?),\s*or set", result.stderr, re.S)
    assert hint, result.stdout + result.stderr
    command = shlex.split(re.sub(r"\s+", " ", hint.group(1)))
    suggested = subprocess.run(command, cwd=project, env=env, capture_output=True, text=True, timeout=60)
    assert suggested.returncode == 0, suggested.stdout + suggested.stderr
    assert_inspector_installed(project, "_build/custom build")
    assert not (project / "_build/environment build/ovd-inspector-install").exists()


def test_missing_inspector_override_has_actionable_diagnostic(inspector_project):
    env = inspector_build_env()
    env["OVPHYSX_OVD_INSPECTOR"] = "missing SDK/ovd-inspect"
    result = subprocess.run([shutil.which("cmake"), "-P", "scripts/test_python_samples.cmake"],
                            cwd=inspector_project, env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode != 0
    diagnostic = re.sub(r"\s+", " ", result.stderr)
    assert "OVPHYSX_OVD_INSPECTOR" in diagnostic
    assert "unset" in diagnostic
    assert "build_ovd_inspector.cmake" not in diagnostic


@pytest.mark.parametrize("build_dir,environment", [
    ("_build", None),
    ("_build/custom build", None),
    ("_build/custom build", "_build/environment build"),
])
def test_inspector_custom_target_retains_configured_directory(inspector_project, build_dir, environment):
    project = inspector_project
    source = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    # Execute the exact production argument setup and target declaration in a
    # minimal project, avoiding the unrelated OvPhysX/OVStage dependency graph.
    arguments = source[source.index("if(CMAKE_CONFIGURATION_TYPES)\n    set(_SCRIPT_BUILD_ARGS"):
                       source.index("set(_SCRIPT_RUNTIME_DEPS_ARG")]
    target = re.search(r"add_custom_target\(build_ovd_inspector\b.*?\n\)", source, re.S)
    assert target, "Inspector target declaration was not found"
    (project / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.21)\nproject(provisioning LANGUAGES NONE)\n"
        + arguments + target.group(0) + "\n", encoding="utf-8")
    env = inspector_build_env()
    if environment is not None:
        env["OVPHYSX_BUILD_DIR"] = environment
    run_cmake(["-S", ".", "-B", build_dir, "-DCMAKE_BUILD_TYPE=Release"], project, env)
    run_cmake(["--build", build_dir, "--target", "build_ovd_inspector"], project, env)
    assert_inspector_installed(project, build_dir)
    if environment is not None:
        assert not (project / environment / "ovd-inspector-install").exists()


@pytest.mark.parametrize("explicit,environment,selected", [
    (None, None, "_build"),
    ("_build/custom build", None, "_build/custom build"),
    (None, "_build/environment build", "_build/environment build"),
    ("_build/custom build", "_build/environment build", "_build/custom build"),
])
def test_cpp_samples_provisions_selected_inspector_directory(inspector_project, explicit, environment, selected):
    env = inspector_build_env()
    if environment is not None:
        env["OVPHYSX_BUILD_DIR"] = environment
    arguments = ["-DSAMPLES_BUILD_ONLY=ON", "-DSKIP_STALENESS_CHECK=ON", "-DOVSTAGE_DIR=offline"]
    if explicit is not None:
        arguments.append(f"-DBUILD_DIR={explicit}")
    run_cmake([*arguments, "-P", "scripts/test_cpp_samples.cmake"], inspector_project, env)
    assert_inspector_installed(inspector_project, selected)
    if environment is not None and environment != selected:
        assert not (inspector_project / environment / "ovd-inspector-install").exists()


@pytest.mark.parametrize("external,run_only", [(True, False), (False, True)])
def test_cpp_samples_skips_inspector_provisioning(inspector_project, external, run_only):
    env = inspector_build_env()
    if external:
        inspector = inspector_project / "external-inspector"
        inspector.touch()
        env["OVPHYSX_OVD_INSPECTOR"] = str(inspector)
    # No SDK tool sources: a mistaken provisioning launch now fails, even if
    # it targets some other directory instead of the expected custom build.
    (inspector_project.parent / "physx/pvddom/CMakeLists.txt").unlink()
    mode = "SAMPLES_RUN_ONLY" if run_only else "SAMPLES_BUILD_ONLY"
    run_cmake([f"-D{mode}=ON", "-DBUILD_DIR=_build/custom build", "-DSKIP_STALENESS_CHECK=ON",
               "-DOVSTAGE_DIR=offline", "-P", "scripts/test_cpp_samples.cmake"], inspector_project, env)
    assert not (inspector_project / "_build/custom build/ovd-inspector-install").exists()
