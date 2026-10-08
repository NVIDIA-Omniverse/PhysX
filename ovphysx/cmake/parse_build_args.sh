#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

#
# Shared argument parser for build scripts (ovphysx, ovruntime).
# Source this file to parse unified build flags.
#
# Usage: source /path/to/parse_build_args.sh "$@"
#
# After sourcing, the following variables are set:
#   BUILD_CONFIG=""        "debug" or "release". Empty means the caller defaults to release
#   DO_CLEAN=0             1 when -c/--clean (clean only, no build)
#   DO_REBUILD=0           1 when -x/--rebuild (clean then build)
#   DO_GENERATE_ONLY=0     1 when -g/--generate (configure only)
#   BUILD_TARGET=""        CMake target name from -t/--target <name>
#   EXTRA_ARGS=()          unrecognized args for project-specific handling

BUILD_CONFIG=""
DO_CLEAN=0
DO_REBUILD=0
DO_GENERATE_ONLY=0
BUILD_TARGET=""
EXTRA_ARGS=()


print_common_usage() {
    echo "Common flags:"
    echo "  -c, --clean        Clean artifacts only (no build)"
    echo "  -x, --rebuild      Clean then build"
    echo "  -d, --debug        Build debug configuration"
    echo "  -r, --release      Build release configuration"
    echo "  -t, --target NAME  Build a specific CMake target"
    echo "  -g, --generate     Configure only (no build)"
}

while [ $# -gt 0 ]; do
    case "$1" in
        -c|--clean)       DO_CLEAN=1 ;;
        -x|--rebuild)     DO_REBUILD=1 ;;
        -d|--debug)       BUILD_CONFIG="debug" ;;
        -r|--release)     BUILD_CONFIG="release" ;;
        -g|--generate)    DO_GENERATE_ONLY=1 ;;
        -t|--target)
            shift
            if [ $# -eq 0 ]; then
                echo "Error: --target requires an argument"
                exit 1
            fi
            BUILD_TARGET="$1"
            ;;
        *)  EXTRA_ARGS+=("$1") ;;
    esac
    shift
done
