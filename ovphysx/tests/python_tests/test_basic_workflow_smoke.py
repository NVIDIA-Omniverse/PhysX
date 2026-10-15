# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-OVSTAGE-001
# @covers AC-5

"""Offline checks for the installed-skill CI smoke driver."""

import importlib.util
import os
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("basic_workflow_smoke", ROOT / "scripts/test_basic_workflow.py")
WORKFLOW = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WORKFLOW)


def test_recipe_ignores_unrelated_cmake_fences():
    text = (ROOT / "skills/basic-workflow/SKILL.md").read_text(encoding="utf-8")
    expected = WORKFLOW.recipe(text)
    text = text.replace("### Build and run the C sample\n", "### Build and run the C sample\n\n```cmake\nmessage(STATUS example)\n```\n")
    assert WORKFLOW.recipe(text) == expected


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
def test_split_build_and_run_use_documented_commands(tmp_path, monkeypatch):
    if not shutil.which("cmake") or not shutil.which("cc"):
        pytest.skip("CMake and a C compiler are required for the offline consumer")
    sdk = tmp_path / "SDK with spaces"
    common = sdk / "samples/c_samples/common"
    common.mkdir(parents=True)
    (common / "marker.h").write_text("#define MARKER 7\n", encoding="utf-8")
    (sdk / "samples/data").mkdir()
    (sdk / "samples/data/simple_physics_scene.usda").write_text("fixture\n", encoding="utf-8")
    c = '''#include "common/marker.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(void) {
    if (MARKER != 7 || !getenv("PHYSX_USD_SCHEMAS_ROOT") ||
        strcmp(getenv("LD_LIBRARY_PATH"), "") != 0) return 1;
    FILE* scene = fopen("samples/data/simple_physics_scene.usda", "r");
    if (!scene) return 2;
    fclose(scene);
    return 0;
}
'''
    cmake = "cmake_minimum_required(VERSION 3.16)\nproject(fixture C)\nadd_executable(my_app main.c)\n"
    skill = sdk / "skills/basic-workflow/SKILL.md"
    skill.parent.mkdir(parents=True)
    skill.write_text(
        f"```c\n{c}```\n### Build and run the C sample\n```cmake\n{cmake}```\n"
        '```bash\ncmake -B build-my-app -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="/path/to/ovphysx;/path/to/ovstage"\n'
        "cmake --build build-my-app --config Release\n```\n"
        "```bash\nexport PHYSX_USD_SCHEMAS_ROOT=/external-schemas\n./build-my-app/my_app\n```\n",
        encoding="utf-8",
    )
    build = tmp_path / "build"
    build.mkdir()
    (build / "CMakeCache.txt").write_text("CMAKE_GENERATOR:INTERNAL=Unix Makefiles\n", encoding="utf-8")
    monkeypatch.setenv("PHYSX_USD_SCHEMAS_ROOT", str(tmp_path / "external-schemas"))
    monkeypatch.setenv("LD_LIBRARY_PATH", "/sdk/lib:/ovstage/lib")
    args = SimpleNamespace(sdk_root=sdk, ovstage_root=tmp_path / "OVStage with spaces",
                           build_root=build, cmake=shutil.which("cmake"), build_only=True, run_only=False)
    WORKFLOW.run(args)
    consumer = build / "basic_workflow"
    assert (consumer / "main.c").read_text(encoding="utf-8") == c
    assert (consumer / "CMakeLists.txt").read_text(encoding="utf-8") == cmake
    cache = WORKFLOW.read_cache(consumer / "build-my-app/CMakeCache.txt")
    assert cache["CMAKE_PREFIX_PATH"] == f"{sdk.as_posix()};{args.ovstage_root.as_posix()}"
    args.build_only, args.run_only, args.cmake = False, True, "missing-cmake"
    WORKFLOW.run(args)
    monkeypatch.delenv("PHYSX_USD_SCHEMAS_ROOT")
    with pytest.raises(ValueError, match="PHYSX_USD_SCHEMAS_ROOT"):
        WORKFLOW.run(args)
