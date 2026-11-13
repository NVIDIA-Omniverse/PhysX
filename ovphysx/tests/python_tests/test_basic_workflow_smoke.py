# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-OVSTAGE-001
# @covers AC-5

"""Offline checks for the installed basic-workflow sample smoke driver."""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("basic_workflow_smoke", ROOT / "scripts/test_basic_workflow.py")
WORKFLOW = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WORKFLOW)


def smoke_args(tmp_path):
    sdk = tmp_path / "SDK with spaces"
    sample = sdk / "samples/c_samples/basic_workflow_c"
    sample.mkdir(parents=True)
    (sample / "main.c").write_text("int main(void) { return 0; }\n", encoding="utf-8")
    (sample / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\nproject(fixture C)\n"
        "add_executable(basic_workflow_c main.c)\n", encoding="utf-8",
    )
    build = tmp_path / "build with spaces"
    build.mkdir()
    generator = "Ninja" if os.name == "nt" else "Unix Makefiles"
    (build / "CMakeCache.txt").write_text(
        f"CMAKE_GENERATOR:INTERNAL={generator}\n", encoding="utf-8",
    )
    return SimpleNamespace(sdk_root=sdk, ovstage_root=tmp_path / "OVStage with spaces",
                           build_root=build, cmake="cmake", build_only=True, run_only=False)


@pytest.mark.parametrize("skill_text", [None, "# Reworded workflow\n", "```c\ninvalid C\n```\n"])
def test_build_uses_installed_sample_independently_of_skill_markdown(tmp_path, monkeypatch, skill_text):
    args = smoke_args(tmp_path)
    if skill_text is not None:
        skill = args.sdk_root / "skills/ovphysx-basic-workflow/SKILL.md"
        skill.parent.mkdir(parents=True)
        skill.write_text(skill_text, encoding="utf-8")
    calls = []
    monkeypatch.setattr(WORKFLOW.subprocess, "run", lambda command, **kwargs: calls.append((command, kwargs)))
    WORKFLOW.run(args)
    consumer = args.build_root / "basic_workflow"
    sample = args.sdk_root / "samples/c_samples/basic_workflow_c"
    configure = calls[0][0]
    assert configure[configure.index("-S") + 1] == str(sample)
    assert configure[configure.index("-B") + 1] == str(consumer)
    assert f"-DCMAKE_PREFIX_PATH={args.sdk_root.as_posix()};{args.ovstage_root.as_posix()}" in configure
    assert calls[1][0] == ["cmake", "--build", str(consumer), "--config", "Release"]
    assert len(calls) == 2
    assert all(options["check"] for _, options in calls)
    assert not (consumer / "main.c").exists()
    assert not (consumer / "CMakeLists.txt").exists()


@pytest.mark.parametrize("failing_call", [0, 1])
def test_build_failure_stops_the_smoke(tmp_path, monkeypatch, failing_call):
    args = smoke_args(tmp_path)
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if len(calls) - 1 == failing_call:
            raise subprocess.CalledProcessError(7, command)

    monkeypatch.setattr(WORKFLOW.subprocess, "run", run)
    with pytest.raises(subprocess.CalledProcessError) as failure:
        WORKFLOW.run(args)
    assert failure.value.returncode == 7
    assert len(calls) == failing_call + 1


@pytest.mark.parametrize("returncode", [0, 7])
@pytest.mark.parametrize("generator", ["Ninja", "Visual Studio 17 2022"] if os.name == "nt" else ["Unix Makefiles"])
def test_run_only_executes_launcher_and_preserves_failure(tmp_path, monkeypatch, returncode, generator):
    args = smoke_args(tmp_path)
    args.build_only, args.run_only, args.cmake = False, True, "missing-cmake"
    (args.build_root / "CMakeCache.txt").write_text(
        f"CMAKE_GENERATOR:INTERNAL={generator}\n", encoding="utf-8",
    )
    consumer = args.build_root / "basic_workflow"
    consumer.mkdir()
    if os.name == "nt":
        output = consumer / "Release" if generator.startswith("Visual Studio ") else consumer
        output.mkdir(exist_ok=True)
        launcher = output / "run_basic_workflow_c.cmd"
        launcher.write_text(f"@exit /b {returncode}\n", encoding="ascii")
    else:
        launcher = consumer / "basic_workflow_c"
        launcher.write_text(f"#!/bin/sh\nexit {returncode}\n", encoding="ascii")
        launcher.chmod(0o755)
    monkeypatch.setenv("PHYSX_USD_SCHEMAS_ROOT", str(tmp_path / "external-schemas"))
    if returncode:
        with pytest.raises(subprocess.CalledProcessError) as failure:
            WORKFLOW.run(args)
        assert failure.value.returncode == returncode
    else:
        WORKFLOW.run(args)


def test_run_only_requires_the_built_launcher(tmp_path):
    args = smoke_args(tmp_path)
    args.build_only, args.run_only = False, True
    with pytest.raises(FileNotFoundError, match="basic_workflow_c"):
        WORKFLOW.run(args)


def test_vs_generator_preserves_the_portable_instance(tmp_path):
    cache = tmp_path / "CMakeCache.txt"
    cache.write_text(
        "CMAKE_GENERATOR:INTERNAL=Visual Studio 17 2022\n"
        "CMAKE_GENERATOR_PLATFORM:INTERNAL=x64\n"
        "CMAKE_GENERATOR_INSTANCE:INTERNAL=C:/portable-msvc,version=17.0.0.0\n"
        "CMAKE_VS_SDK_INCLUDE_DIRECTORIES:STRING=C:/sdk/ucrt;C:/sdk/shared\n"
        "CMAKE_MAKE_PROGRAM:FILEPATH=C:/ninja.exe\n", encoding="utf-8",
    )
    assert WORKFLOW.toolchain_args(WORKFLOW.read_cache(cache)) == [
        "-G", "Visual Studio 17 2022", "-DCMAKE_GENERATOR_PLATFORM=x64",
        "-DCMAKE_GENERATOR_INSTANCE=C:/portable-msvc,version=17.0.0.0",
        "-DCMAKE_VS_SDK_INCLUDE_DIRECTORIES=C:/sdk/ucrt;C:/sdk/shared",
    ]


@pytest.mark.parametrize("system_root", ["SystemRoot", "SYSTEMROOT"])
def test_windows_runtime_path_has_no_sdk_or_ovstage(monkeypatch, system_root):
    monkeypatch.setattr(WORKFLOW, "os", SimpleNamespace(name="nt"))
    inherited = {
        "Path": "C:/sdk/bin;C:/sdk/plugins;C:/ovstage/bin;C:/toolchain",
        system_root: "C:/Windows", "CUDA_PATH": "C:/CUDA",
        "ComSpec": "C:/Windows/System32/cmd.exe",
        "PHYSX_USD_SCHEMAS_ROOT": "C:/external-schemas",
    }
    environment = WORKFLOW.runtime_env(inherited)
    assert environment["PATH"].replace("\\", "/") == "C:/Windows/System32;C:/Windows;C:/CUDA/bin"
    assert "Path" not in environment
    assert environment["COMSPEC"] == "C:/Windows/System32/cmd.exe"
    assert environment["PHYSX_USD_SCHEMAS_ROOT"] == "C:/external-schemas"
    assert "sdk" in inherited["Path"]


@pytest.mark.skipif(os.name == "nt", reason="The synthetic consumer checks Linux launch behavior")
def test_split_build_and_run_use_installed_source_and_data(tmp_path, monkeypatch):
    if not shutil.which("cmake") or not shutil.which("cc"):
        pytest.skip("CMake and a C compiler are required for the offline consumer")
    args = smoke_args(tmp_path)
    sdk = args.sdk_root
    common = sdk / "samples/c_samples/common"
    common.mkdir(parents=True)
    (common / "marker.h").write_text("#define MARKER 7\n", encoding="utf-8")
    (sdk / "samples/data").mkdir()
    (sdk / "samples/data/simple_physics_scene.usda").write_text("fixture\n", encoding="utf-8")
    c = '''#include "marker.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(void) {
    if (MARKER != 7 || !getenv("PHYSX_USD_SCHEMAS_ROOT") ||
        strcmp(getenv("LD_LIBRARY_PATH"), "") != 0) return 1;
    FILE* scene = fopen(OVPHYSX_TEST_DATA "/simple_physics_scene.usda", "r");
    if (!scene) return 2;
    fclose(scene);
    return 0;
}
'''
    cmake = '''cmake_minimum_required(VERSION 3.16)
project(fixture C)
add_executable(basic_workflow_c main.c)
target_include_directories(basic_workflow_c PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common")
get_filename_component(DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(basic_workflow_c PRIVATE OVPHYSX_TEST_DATA="${DATA_DIR}")
'''
    sample = sdk / "samples/c_samples/basic_workflow_c"
    (sample / "main.c").write_text(c, encoding="utf-8")
    (sample / "CMakeLists.txt").write_text(cmake, encoding="utf-8")
    skill = sdk / "skills/ovphysx-basic-workflow/SKILL.md"
    skill.parent.mkdir(parents=True)
    skill.write_text("# Workflow guidance, with no executable recipe.\n", encoding="utf-8")
    monkeypatch.setenv("PHYSX_USD_SCHEMAS_ROOT", str(tmp_path / "external-schemas"))
    monkeypatch.setenv("LD_LIBRARY_PATH", "/sdk/lib:/ovstage/lib")
    args.cmake = shutil.which("cmake")
    WORKFLOW.run(args)
    consumer = args.build_root / "basic_workflow"
    cache = WORKFLOW.read_cache(consumer / "CMakeCache.txt")
    assert cache["CMAKE_PREFIX_PATH"] == f"{sdk.as_posix()};{args.ovstage_root.as_posix()}"
    skill.write_text("```c\ninvalid C\n```\n", encoding="utf-8")
    args.build_only, args.run_only, args.cmake = False, True, "missing-cmake"
    WORKFLOW.run(args)
    monkeypatch.delenv("PHYSX_USD_SCHEMAS_ROOT")
    with pytest.raises(ValueError, match="PHYSX_USD_SCHEMAS_ROOT"):
        WORKFLOW.run(args)
