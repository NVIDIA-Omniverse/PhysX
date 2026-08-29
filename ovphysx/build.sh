#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

set -e

# Host-mode build entry point for ovphysx.
#
# This script resolves cmake (packman or system) and calls:
#   cmake -P scripts/build.cmake
# which fetches dependencies, configures the CMake project in _build/,
# and compiles libovphysx plus the required ovruntime subproject targets.
#
# After this script completes, _build/ is populated but _install/ is not.
# Next steps:
#   cmake -P scripts/validate_all.cmake                 # full validation (build + install + wheel + all tests)
#   cmake -P scripts/install.cmake                     # just assemble _install/ (for manual testing)
# Advanced (same validation as validate_all.cmake after a full build; run build.cmake first if needed):
#   cmake --build _build --target validate_all         # install + wheel + all tests
#
# Entry point for building ovphysx from source.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parse unified build flags
source "$SCRIPT_DIR/cmake/parse_build_args.sh" "$@"

# Process project-specific flags from EXTRA_ARGS
DO_DEV_PHYSX=0
DO_DEV_SCHEMA=0
DO_BENCHMARKS=0
DEFAULT_RELEASE_RUNTIME_DEPS=1
USER_SET_RELEASE_RUNTIME_DEPS=0
CMAKE_PASSTHROUGH_ARGS=()
for arg in "${EXTRA_ARGS[@]}"; do
    case "$arg" in
        --devphysx)   DO_DEV_PHYSX=1 ;;
        --devschema)  DO_DEV_SCHEMA=1 ;;
        --benchmarks) DO_BENCHMARKS=1 ;;
        -DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=*)
            USER_SET_RELEASE_RUNTIME_DEPS=1
            CMAKE_PASSTHROUGH_ARGS+=("$arg")
            ;;
        -D*)          CMAKE_PASSTHROUGH_ARGS+=("$arg") ;;  # pass -D flags to cmake
        *)
            echo "Usage: $0 [flags]"
            print_common_usage
            echo ""
            echo "Project-specific flags:"
            echo "  --devphysx         Build PhysX SDK from source"
            echo "  --devschema        Use locally-built physics schema"
            echo "  --benchmarks       Build the opt-in benchmark suite (tests/benchmarks/)"
            echo ""
            echo "Changing --devphysx or --devschema needs a clean rebuild:"
            echo "the flag combination selects a build flavor, and incremental"
            echo "builds across a flavor change are not supported. Add --rebuild"
            echo "whenever the flags differ from the previous build."
            echo ""
            echo "Any -D<VAR>=<VALUE> flags are passed through to cmake."
            exit 1
            ;;
    esac
done

# Use packman-provided cmake when available. ovruntime's packman downloads cmake
# into its target-deps, which is also accessible from ovphysx after dep fetch.
# In minimal build environments this may be the only cmake available.
PACKMAN_CMAKE="$SCRIPT_DIR/ovruntime/_build/target-deps/cmake/bin/cmake"
if [ -x "$PACKMAN_CMAKE" ]; then
    CMAKE="$PACKMAN_CMAKE"
else
    CMAKE="$(command -v cmake 2>/dev/null || true)"
    if [ -z "$CMAKE" ]; then
        echo "Error: cmake not found. Run scripts/fetch_deps.sh first or install cmake."
        exit 1
    fi
fi

# Map unified flags to cmake -D args
cmake_args=()
if [ "$DO_REBUILD" -eq 1 ]; then
    cmake_args+=("-DCLEAN_BUILD=ON" "-DCLEAN_ONLY=OFF")  # clean then continue building
elif [ "$DO_CLEAN" -eq 1 ]; then
    cmake_args+=("-DCLEAN_BUILD=ON")  # CLEAN_ONLY defaults to ON in build.cmake
fi
[ "$BUILD_CONFIG" = "debug" ] && cmake_args+=("-DBUILD_TYPE=Debug")
[ "$DO_GENERATE_ONLY" -eq 1 ] && cmake_args+=("-DGENERATE_ONLY=ON")
[ -n "$BUILD_TARGET" ] && cmake_args+=("-DBUILD_TARGET=$BUILD_TARGET")
[ "$DO_DEV_PHYSX" -eq 1 ] && cmake_args+=("-DDEV_PHYSX=ON")
[ "$DO_DEV_SCHEMA" -eq 1 ] && cmake_args+=("-DDEV_SCHEMA=ON")
[ "$DO_BENCHMARKS" -eq 1 ] && cmake_args+=("-DBENCHMARKS=ON")
if [ "$USER_SET_RELEASE_RUNTIME_DEPS" -eq 0 ] && [ "$DEFAULT_RELEASE_RUNTIME_DEPS" -eq 1 ]; then
    cmake_args+=("-DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON")
fi

exec "$CMAKE" "${cmake_args[@]}" "${CMAKE_PASSTHROUGH_ARGS[@]}" -P "${SCRIPT_DIR}/scripts/build.cmake"
