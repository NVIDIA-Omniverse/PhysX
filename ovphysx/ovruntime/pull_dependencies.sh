#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Usage: ./pull_dependencies.sh [--config release|debug|all] [--devphysx] [--devschema]

set -e

DO_DEV_PHYSX=0
DO_DEV_SCHEMA=0
CONFIGS="release"

while [ $# -gt 0 ]; do
    case "$1" in
        --devphysx)
            DO_DEV_PHYSX=1
            ;;
        --devschema)
            DO_DEV_SCHEMA=1
            ;;
        --config)
            shift
            if [ $# -eq 0 ]; then
                echo "Error: --config requires an argument: release, debug, or all"
                exit 1
            fi
            case "$1" in
                release) CONFIGS="release" ;;
                debug) CONFIGS="debug" ;;
                all) CONFIGS="release debug" ;;
                *)
                    echo "Error: unsupported --config value '$1'; expected release, debug, or all"
                    exit 1
                    ;;
            esac
            ;;
        *)
            echo "Error: unknown option '$1'; expected --config, --devphysx, or --devschema"
            exit 1
            ;;
    esac
    shift
done

# Fetch only the requested config by default. Use --config all for workflows
# that need both release and debug dependencies in one pull.

echo "Pulling Physics ovruntime dependencies with packman..."
echo "USD mode: namespaced"
echo "Configs: $CONFIGS"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

PACKMAN="tools/packman/packman"
if [ "$(uname -m)" = "aarch64" ]; then
    PLATFORM="manylinux_2_35_aarch64"
else
    PLATFORM="manylinux_2_35_x86_64"
fi

# Pull host deps (msvc)
$PACKMAN pull deps/host-deps.packman.xml -p "$PLATFORM"

# Pull target deps (PhysX, onnx-mlir, physxdevice, leveldb, snappy, cmake)
$PACKMAN pull deps/target-deps.packman.xml -p "$PLATFORM"

# ovruntime_deps (release variant, config-independent). The config-dependent
# import below reads a manifest inside it, so it comes first.
$PACKMAN pull deps/ovruntime-deps.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM"

# carb_sdk_plugins, then python/cxxopts/doctest imported from it. Config-independent,
# and python must exist before the ovstage fetch below, which uses it as interpreter.
$PACKMAN pull deps/carb-sdk-deps.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM"
$PACKMAN pull deps/carb-sdk-deps-import.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM" -t "platform_target=$PLATFORM" -t "config=release"

PYTHON=""
for PYTHON_CANDIDATE in \
    "_build/target-deps/python/python" \
    "_build/target-deps/python/python3" \
    "_build/target-deps/python/bin/python3"; do
    if [ -x "$PYTHON_CANDIDATE" ]; then
        PYTHON="$PYTHON_CANDIDATE"
        break
    fi
done
if [ -z "$PYTHON" ]; then
    echo "Failed to find packman Python for ovstage dependency fetch!"
    exit 1
fi

# ovstage backend (ADR-0002) into _build/target-deps/ovstage. Two modes:
#   1. Local source checkout: if deps/ovstage-deps.packman.xml declares a <source>
#      path (opt-in, for ovstage development), packman links that local build.
#   2. Default: fetch the released package via fetch_ovstage_release.py.
OVSTAGE_FETCH="../scripts/fetch_ovstage_release.py"
if grep -q '<source path="\([^"]*\)"' deps/ovstage-deps.packman.xml 2>/dev/null; then
    OVSTAGE_SRC=$(sed -n 's/.*<source path="\([^"]*\)".*/\1/p' deps/ovstage-deps.packman.xml | head -1)
    if [ -d "$OVSTAGE_SRC" ]; then
        echo "Linking ovstage source dependency ($OVSTAGE_SRC)..."
        $PACKMAN pull deps/ovstage-deps.packman.xml -p "$PLATFORM"
    else
        echo "Skipping ovstage source link (path not present: $OVSTAGE_SRC)"
    fi
elif [ -f "$OVSTAGE_FETCH" ]; then
    echo "Fetching ovstage release for $PLATFORM ..."
    "$PYTHON" "$OVSTAGE_FETCH" --platform "$PLATFORM" --dest "$PWD/_build/target-deps/ovstage"
else
    echo "ERROR: $OVSTAGE_FETCH not found. ovstage is required to build ovruntime on this branch."
    exit 1
fi

# Pull config-dependent deps
for CONFIG in $CONFIGS; do
    # platform_target and usd_ver are placeholders: they only build version strings
    # for filtered-out entries (nvtx, omniusdresolver), never resolved.
    $PACKMAN pull deps/ovruntime-deps-import.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM" -t "platform_target=$PLATFORM" -t "usd_ver=unused" -t "config=$CONFIG"

    # Pull namespaced schema deps (physxSchema, physicsSchemaTools headers/libs).
    # Skipped in --devschema mode: CMake will point at the local schema build instead.
    if [ "$DO_DEV_SCHEMA" -eq 0 ]; then
        $PACKMAN pull deps/schema-deps.packman.xml -p "$PLATFORM" -t "platform_target_abi=$PLATFORM" -t "config=$CONFIG"
    fi
done

if [ "$DO_DEV_PHYSX" -eq 1 ]; then
    # Pull PhysX source-build dependencies into the current environment so
    # PM_SECURELOADLIBRARY_PATH and related variables are available.
    $PACKMAN pull ../../physx/dependencies.xml --platform linux
fi

# Fetch pip dependencies (Newton USD schemas) using the packman Python.
# The canonical package list lives in deps/pip_newton.toml.
if [ -x "$PYTHON" ]; then
    "$PYTHON" tools/pip_fetch.py deps/pip_newton.toml
else
    echo "Warning: Python not found at $PYTHON -- skipping pip fetch"
fi

echo ""
echo "Dependencies pulled successfully!"
echo ""
