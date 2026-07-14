#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set -e

DO_DEV_PHYSX=0
for arg in "$@"; do
    case "$arg" in
        --devphysx) DO_DEV_PHYSX=1 ;;
    esac
done

# Fetch both release and debug configs so that the CMake build can select the
# right USD/TBB libraries for each build type (e.g. usd/release/lib vs usd/debug/lib).
CONFIGS="release debug"

echo "Pulling Physics ovruntime dependencies with packman..."

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

PACKMAN="../ovexts/tools/packman/packman"
if [ "$(uname -m)" = "aarch64" ]; then
    PLATFORM="manylinux_2_35_aarch64"
else
    PLATFORM="manylinux_2_35_x86_64"
fi

# Pull host deps (msvc) — skipped when absent (open-source build uses local toolchain)
if [ -f deps/host-deps.packman.xml ]; then
    $PACKMAN pull deps/host-deps.packman.xml -p "$PLATFORM"
fi

# Pull target deps (PhysX, onnx-mlir, physxdevice, leveldb, snappy, python311)
$PACKMAN pull deps/target-deps.packman.xml -p "$PLATFORM"

# Pull ovruntime_deps package (RTX/fabric plugins, headers)
$PACKMAN pull deps/ovruntime-deps.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM"

# Pull config-dependent deps for both release and debug
for CONFIG in $CONFIGS; do
    # Import dependencies from ovruntime_deps (Carbonite, USD, Python, CUDA, etc.)
    $PACKMAN pull deps/ovruntime-deps-import.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM" -t "config=$CONFIG"

    # Pull kit-kernel for dev headers not yet in ovruntime_deps (omni/timeline, omni/kit/renderer, etc.)
    $PACKMAN pull deps/kit-kernel-deps.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM" -t "config=$CONFIG"

    # Pull schema deps (physxSchema, physicsSchemaTools headers/libs)
    $PACKMAN pull deps/schema-deps.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM" -t "config=$CONFIG"
done

if [ "$DO_DEV_PHYSX" -eq 1 ]; then
    # Pull PhysX source-build dependencies into the current environment so
    # PM_SECURELOADLIBRARY_PATH and related variables are available.
    $PACKMAN pull ../../physx/dependencies.xml --platform linux
fi

# Fetch pip dependencies (Newton USD schemas) using the packman Python.
# The canonical package list lives in deps/pip_newton.toml.
PYTHON="_build/target-deps/python/python"
if [ -x "$PYTHON" ]; then
    "$PYTHON" tools/pip_fetch.py deps/pip_newton.toml
else
    echo "Warning: Python not found at $PYTHON — skipping pip fetch"
fi

echo ""
echo "Dependencies pulled successfully!"
echo ""
