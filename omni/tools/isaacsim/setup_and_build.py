# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import sys
import os
import subprocess
import platform
import builtins
import argparse

### Switches and Parameters

parser = argparse.ArgumentParser(description="Setup and build Omniverse PhysX with IsaacSim integration.")
parser.add_argument("--devphysx", action="store_true", help="Build with local PhysX SDK binaries.")
parser.add_argument("--devschema", action="store_true", help="Build with local PhysX USD schema code.")
parser.add_argument("--isaacsim", required=True, help="Path to IsaacSim repository.")
parser.add_argument("--config", choices=["release", "debug"], nargs="+", default=["release"], help="Build configuration(s). Pass one or both. Default: release.")
parser.add_argument("--ci", action="store_true", help="Non-interactive CI mode. Automatically answers all prompts with their CI defaults: clean rebuild, build PhysX SDK if --devphysx is set, and continue past each step.")
args = parser.parse_args()

print("Welcome to the Setup and Build script for IsaacSim with Omniverse PhysX extensions! This script will download IsaacSim dependencies, build Omniverse PhysX extensions, and build IsaacSim. It will ask for confirmation after each step is completed.")
print("For more information see '/omni/README.md'.\n")

devphysx = args.devphysx
devschema = args.devschema
config = args.config
physx_repo_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))

### CI mode: replace input() with automatic answers so the script runs unattended.
### Answers in order: rebuild=r (clean), physx built=n (build it), then "" for each step confirmation.
if args.ci:
    _ci_answers = iter(
        ["r"]                          # rebuild prompt → clean rebuild
        + [""] * 2                     # step confirmations: source clear + build -g
        + (["n"] if devphysx else [])  # PhysX built? → no, build it now
        + [""] * 50                    # per-step confirmations → continue
    )
    builtins.input = lambda prompt="": next(_ci_answers)

### Helpers
script_ext = ".bat" if sys.platform.startswith("win") else ".sh"
physx_repo_omni_path = os.path.join(physx_repo_path, "omni")
physx_repo_schema_path = os.path.join(physx_repo_path, "schema")
physx_repo_physxsdk_path = os.path.join(physx_repo_path, "physx")
your_platform = "windows-x86_64" if sys.platform.startswith("win") else ("linux-aarch64" if platform.machine() in ("aarch64", "arm64") else "linux-x86_64")
if sys.platform.startswith("win"):
    physx_preset = "vc16win64-carbonite"
    physx_build_script = os.path.join("buildtools", "steps", "build_win_x86_64.bat")
elif platform.machine() in ("aarch64", "arm64"):
    physx_preset = "linux-aarch64-carbonite"
    physx_build_script = os.path.join("buildtools", "steps", "build_linux.sh")
else:
    physx_preset = "linux-carbonite"
    physx_build_script = os.path.join("buildtools", "steps", "build_linux.sh")
current_step = 1

def run_command(cmd, cwd):
    command_line = os.path.join(cwd, ' '.join(cmd))
    print(f"Running: '{command_line}'")
    if sys.platform.startswith("win"):
        subprocess.check_call(cmd, cwd=cwd, shell=True)
    else:
        cmd[0] = f"./{cmd[0]}"
        subprocess.check_call(cmd, cwd=cwd)
    print(f"\n[QUESTION] Check results of Step {current_step} and press Enter to continue or input 'q' to quit:")
    user_input = input().strip().lower()
    if user_input == "q":
        sys.exit(1)

### IsaacSim setup
isaacsim_repo_path = args.isaacsim

# Ask about a clean re-build (default is incremental)
rebuild_arg = []
print("[QUESTION] Do you want to do an incremental build of Omniverse PhysX and IsaacSim (default option) or a clean rebuild? Run a clean rebuild if unsure of the state of the repositories or have encountered errors in a previous run of this script. Input 'i' for incremental or 'r' for rebuild [I/r]:")
clean_isaacsim = input().strip().lower()
if clean_isaacsim not in ("", "i"):
    rebuild_arg = ["-x"]

# Optionally set devphysx, devschema, and config switches for build
config_flags = ["-r" if c == "release" else "-d" for c in config]
build_args = [*rebuild_arg, *config_flags]
if devphysx:
    build_args.append("--devphysx")
if devschema:
    build_args.append("--devschema")

# Clear dependency overrides and download IsaacSim dependencies
print(f"\n[STEP {current_step}] Clearing dependency overrides in the IsaacSim repository\n")
run_command([f"repo{script_ext}", "source", "clear"], cwd=isaacsim_repo_path)
current_step += 1

print(f"\n[STEP {current_step}] Downloading IsaacSim dependencies\n")
build_script = [f"build{script_ext}", "-g", *config_flags, *rebuild_arg]
run_command(build_script, cwd=isaacsim_repo_path)
current_step += 1

# Check if PhysX SDK is built
if devphysx:
    print("\n[QUESTION] You are using --devphysx. Have you already built the PhysX SDK in the 'physx' directory in this repository? [Y/n]:")
    physx_built = input().strip().lower()
    if physx_built not in ("", "y", "yes"):
        print(f"\n[STEP {current_step}] Generating PhysX SDK projects\n")
        run_command([f"generate_projects{script_ext}", physx_preset], cwd=physx_repo_physxsdk_path)
        current_step += 1

        print(f"\n[STEP {current_step}] Building PhysX SDK (checked)\n")
        run_command([physx_build_script, physx_preset, "checked"], cwd=physx_repo_physxsdk_path)
        current_step += 1

# Build Omniverse PhysX extensions
print(f"\n[STEP {current_step}] Building Omniverse PhysX extensions\n")
build_script = [f"build{script_ext}", *build_args]
run_command(build_script, cwd=physx_repo_omni_path)
current_step += 1

# If devphysx, link PhysX SDK headers for IsaacSim build
if devphysx:
    print(f"\n[STEP {current_step}] Linking PhysX SDK headers for IsaacSim build\n")
    run_command([f"repo{script_ext}", "source", "link", "physx", physx_repo_physxsdk_path], cwd=isaacsim_repo_path)
    current_step += 1
# If devschema, link local PhysX schema headers for IsaacSim build
if devschema:
    print(f"\n[STEP {current_step}] Linking local PhysX schema headers for IsaacSim build\n")
    for c in config:
        schema_path = os.path.join(physx_repo_omni_path, "_build", your_platform, c, "schema")
        run_command([f"repo{script_ext}", "source", "link", "usd_ext_physics_${config}", schema_path], cwd=isaacsim_repo_path)
    current_step += 1
# Always link omni_physics to local Omniverse PhysX headers
print(f"\n[STEP {current_step}] Linking local Omniverse PhysX headers for IsaacSim build\n")
run_command([f"repo{script_ext}", "source", "link", "omni_physics", physx_repo_omni_path], cwd=isaacsim_repo_path)
current_step += 1

# Build IsaacSim
print(f"\n[STEP {current_step}] Building IsaacSim\n")
run_command([f"build{script_ext}", *config_flags], cwd=isaacsim_repo_path)
current_step += 1

# Print instructions for running IsaacSim with devFolder
print(f"\nDone!\n")

print("To run IsaacSim with the locally built PhysX extensions, use:")
for c in config:
    isaacsim_path = os.path.join(isaacsim_repo_path, "_build", your_platform, c, f"isaac-sim{script_ext}")
    extsphysics_path = os.path.join(physx_repo_omni_path, "_build", your_platform, c, "extsPhysics")
    print("{0} --/app/exts/devFolders=[{1}]".format(isaacsim_path, extsphysics_path))
print("\nTo run IsaacLab with the locally built PhysX extensions, run the following command in the root of the IsaacLab directory:")
for c in config:
    isaaclab_path = os.path.join(isaacsim_repo_path, "_build", your_platform, c, f"isaaclab{script_ext}")
    extsphysics_path = os.path.join(physx_repo_omni_path, "_build", your_platform, c, "extsPhysics")
    print("isaaclab.[bat|sh] -p script.py --kit_args=\"--/app/exts/devFolders=[{1}] \"".format(isaaclab_path, extsphysics_path))
