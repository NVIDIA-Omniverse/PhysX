# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-OVSTAGE-001
# @covers AC-5

"""Build and run the installed basic_workflow_c sample."""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess


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
    cache = read_cache(build_root / "CMakeCache.txt")

    if not args.run_only:
        if consumer.resolve().parent != build_root:
            raise ValueError(f"Consumer build directory escapes build root: {consumer}")
        if consumer.exists():
            shutil.rmtree(consumer)
        consumer.mkdir(parents=True)
        sample = sdk / "samples/c_samples/basic_workflow_c"
        configure = [
            args.cmake, "-S", str(sample), "-B", str(consumer), "-DCMAKE_BUILD_TYPE=Release",
            f"-DCMAKE_PREFIX_PATH={sdk.as_posix()};{args.ovstage_root.resolve().as_posix()}",
            *toolchain_args(cache),
        ]
        subprocess.run(configure, cwd=consumer, check=True)
        subprocess.run([args.cmake, "--build", str(consumer), "--config", "Release"],
                       cwd=consumer, check=True)

    if not args.build_only:
        launcher = consumer / "basic_workflow_c"
        if os.name == "nt":
            output = consumer / "Release" if cache["CMAKE_GENERATOR"].startswith("Visual Studio ") else consumer
            launcher = output / "run_basic_workflow_c.cmd"
        if not launcher.is_file():
            raise FileNotFoundError(f"Sample launcher not found: {launcher}")
        env = runtime_env(os.environ)
        command = [str(launcher)]
        if os.name == "nt":
            command = [env["COMSPEC"], "/d", "/c", str(launcher)]
        subprocess.run(command, cwd=consumer, env=env, check=True, timeout=60)
        print(f"basic_workflow_c passed with {cache['CMAKE_GENERATOR']}")


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
