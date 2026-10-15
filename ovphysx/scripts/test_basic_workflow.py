# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-OVSTAGE-001
# @covers AC-5

"""Build and run the installed basic-workflow skill's C example."""

import argparse
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess


def recipe(text):
    c = next(block for block in re.findall(r"^```c\n(.*?)^```", text, re.M | re.S)
             if "int main(" in block)
    section = text.split("### Build and run the C sample\n", 1)[1].split("\n## ", 1)[0]
    cmake = next(block for block in re.findall(r"^```cmake\n(.*?)^```", section, re.M | re.S)
                 if "add_executable(my_app main.c)" in block)
    commands = re.findall(r"^cmake (.+)$", section, re.M)
    if len(commands) != 2:
        raise ValueError("Expected one CMake configure and one build command")
    launchers = dict(re.findall(r"^- (Windows, [^:]+): `([^`]+)`", section, re.M))
    launchers["Linux"] = re.search(r"^\./[^\s]+/my_app$", section, re.M).group(0)
    return c, cmake, commands, launchers


def read_cache(path):
    return dict(re.findall(r"^([A-Za-z_][A-Za-z_0-9]*):[^=\n]*=(.*)$", path.read_text(encoding="utf-8"), re.M))


def toolchain_args(cache):
    generator = cache["CMAKE_GENERATOR"]
    args = ["-G", generator]
    if generator.startswith("Visual Studio "):
        keys = (
            "CMAKE_GENERATOR_PLATFORM", "CMAKE_GENERATOR_INSTANCE", "CMAKE_GENERATOR_TOOLSET",
            "CMAKE_VS_PLATFORM_TOOLSET_VERSION", "CMAKE_VS_SDK_INCLUDE_DIRECTORIES",
            "CMAKE_VS_SDK_LIBRARY_DIRECTORIES",
        )
    else:
        keys = ("CMAKE_C_COMPILER", "CMAKE_MAKE_PROGRAM", "CMAKE_RC_COMPILER")
    args.extend(f"-D{key}={cache[key]}" for key in keys if cache.get(key))
    return args


def runtime_env(environment):
    env = environment.copy()
    if os.name == "nt":
        # Windows environment names ignore case; copied dictionaries do not.
        env = {key.upper(): value for key, value in env.items()}
        system = Path(env["SYSTEMROOT"])
        paths = [str(system / "System32"), str(system)]
        if env.get("CUDA_PATH"):
            paths.append(str(Path(env["CUDA_PATH"]) / "bin"))
        env["PATH"] = ";".join(paths)
    else:
        env["LD_LIBRARY_PATH"] = ""
    if not env.get("PHYSX_USD_SCHEMAS_ROOT"):
        raise ValueError("PHYSX_USD_SCHEMAS_ROOT must point to the separate schema provider")
    return env


def run(args):
    sdk = args.sdk_root.resolve()
    build_root = args.build_root.resolve()
    consumer = build_root / "basic_workflow"
    c, cmake, commands, launchers = recipe(
        (sdk / "skills/basic-workflow/SKILL.md").read_text(encoding="utf-8")
    )
    cache = read_cache(build_root / "CMakeCache.txt")

    if not args.run_only:
        if consumer.exists():
            shutil.rmtree(consumer)
        consumer.mkdir(parents=True)
        (consumer / "main.c").write_text(c, encoding="utf-8")
        (consumer / "CMakeLists.txt").write_text(cmake, encoding="utf-8")
        shutil.copytree(sdk / "samples/c_samples/common", consumer / "common")
        (consumer / "samples/data").mkdir(parents=True)
        shutil.copy2(sdk / "samples/data/simple_physics_scene.usda", consumer / "samples/data")

        configure = shlex.split(commands[0])
        configure = [arg.replace("/path/to/ovphysx", sdk.as_posix())
                     .replace("/path/to/ovstage", args.ovstage_root.resolve().as_posix())
                     for arg in configure]
        configure.extend(toolchain_args(cache))
        subprocess.run([args.cmake, *configure], cwd=consumer, check=True)
        subprocess.run([args.cmake, *shlex.split(commands[1])], cwd=consumer, check=True)

    if not args.build_only:
        label = "Linux"
        if os.name == "nt":
            label = ("Windows, Visual Studio generator" if cache["CMAKE_GENERATOR"].startswith("Visual Studio ")
                     else "Windows, Ninja generator")
        launcher = consumer / launchers[label].replace("\\", "/")
        if not launcher.is_file():
            raise FileNotFoundError(f"Documented launcher not found: {launcher}")
        env = runtime_env(os.environ)
        command = [str(launcher)]
        if os.name == "nt":
            command = [env["COMSPEC"], "/d", "/c", str(launcher)]
        subprocess.run(command, cwd=consumer, env=env, check=True, timeout=60)
        print(f"basic-workflow passed with {cache['CMAKE_GENERATOR']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, required=True)
    parser.add_argument("--ovstage-root", type=Path, required=True)
    parser.add_argument("--build-root", type=Path, required=True)
    parser.add_argument("--cmake", default="cmake")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--build-only", action="store_true")
    mode.add_argument("--run-only", action="store_true")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
