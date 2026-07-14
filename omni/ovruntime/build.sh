#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Detect architecture
ARCH="$(uname -m)"
if [ "$ARCH" = "aarch64" ]; then
    BUILD_BASE="_build/linux-aarch64"
    PACKMAN_PLATFORM="manylinux_2_35_aarch64"
    PLATFORM_SUFFIX="linux-aarch64"
else
    BUILD_BASE="_build/linux-x86_64"
    PACKMAN_PLATFORM="manylinux_2_35_x86_64"
    PLATFORM_SUFFIX="linux-x86_64"
fi

COMPILER_CMD="${CXX:-${CC:-c++}}"
COMPILER_BASENAME="$(basename "$COMPILER_CMD")"
if [[ "$COMPILER_BASENAME" == *clang* ]]; then
    COMPILER_FAMILY="clang"
else
    COMPILER_FAMILY="gcc"
fi
COMPILER_BASE="_compiler/${COMPILER_FAMILY}-${PLATFORM_SUFFIX}"

INSTALL_DIR="_install/ovruntime"
if [ -n "${OVRUNTIME_BUILD_BASE:-}" ]; then
    BUILD_BASE="$OVRUNTIME_BUILD_BASE"
fi
if [ -n "${OVRUNTIME_BUILD_DIR:-}" ]; then
    BUILD_BASE="$OVRUNTIME_BUILD_DIR"
fi
if [ -n "${OVRUNTIME_COMPILER_DIR:-}" ]; then
    COMPILER_BASE="$OVRUNTIME_COMPILER_DIR"
fi
if [ -n "${OVRUNTIME_INSTALL_DIR:-}" ]; then
    INSTALL_DIR="$OVRUNTIME_INSTALL_DIR"
fi
PYTHON_DIR_OVERRIDE="${OVRUNTIME_PYTHON_DIR:-}"
SKIP_INSTALL="${OVRUNTIME_SKIP_INSTALL:-0}"
SKIP_VENV="${OVRUNTIME_SKIP_VENV:-0}"

# Parse unified build flags
source "$SCRIPT_DIR/../cmake/parse_build_args.sh" "$@"

# Process project-specific flags from EXTRA_ARGS
DO_DEV_PHYSX=0
DO_DEV_SCHEMA=0
DO_PER_MODULE=0
DO_SETUP_VENV=0
DO_GET_DEPS_AND_CONFIGURE=0
for arg in "${EXTRA_ARGS[@]}"; do
    case "$arg" in
        --devphysx)   DO_DEV_PHYSX=1 ;;
        --devschema)  DO_DEV_SCHEMA=1 ;;
        --per-module) DO_PER_MODULE=1 ;;
        --test-venv)  DO_SETUP_VENV=1 ;;
        *)
            echo "Usage: $0 [flags]"
            print_common_usage
            echo ""
            echo "Project-specific flags:"
            echo "  --devphysx         Build PhysX SDK from source"
            echo "  --devschema        Use locally-built physics schema"
            echo "  --per-module       Install binaries per-module (for ovexts dir linking)"
            echo "  --test-venv        Set up Python test environment only, then quit"
            exit 1
            ;;
    esac
done

# Map unified flags to ovruntime internal variables
DO_CLEAN_INTERNAL=0
if [ "$DO_CLEAN" -eq 1 ] || [ "$DO_REBUILD" -eq 1 ]; then
    DO_CLEAN_INTERNAL=1
fi

# Default to release only; -d for debug, -r for release
if [ "$BUILD_CONFIG" = "debug" ]; then
    DO_DEBUG=1
    DO_RELEASE=0
else
    DO_DEBUG=0
    DO_RELEASE=1
fi

# -g/--generate maps to generate-only
[ "$DO_GENERATE_ONLY" -eq 1 ] && DO_GET_DEPS_AND_CONFIGURE=1

# BUILD_TARGET from CLI takes precedence, fall back to env var
if [ -z "$BUILD_TARGET" ] && [ -n "${OVRUNTIME_BUILD_TARGET:-}" ]; then
    BUILD_TARGET="$OVRUNTIME_BUILD_TARGET"
fi

# Helper: seconds since epoch
now() { date +%s; }

# Helper: format elapsed seconds as human-readable string
format_elapsed() {
    local diff=$1
    local h=$((diff / 3600))
    local m=$(( (diff % 3600) / 60 ))
    local s=$((diff % 60))
    if [ $h -gt 0 ]; then
        printf "%dh %02dm %02ds" $h $m $s
    elif [ $m -gt 0 ]; then
        printf "%02dm %02ds" $m $s
    else
        printf "%ds" $s
    fi
}

# Parent omni/ dir (cmake modules at ../cmake are needed by CMakeLists.txt)
OMNI_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# Repository root (needed when --devphysx mounts physx/ source tree)
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Packman cache (target-deps are symlinks into here)
PACKMAN_CACHE="${PM_PACKAGES_ROOT:-${HOME}/.cache/packman}"

# Helper: run a command, optionally inside Docker
run_cmd() {
    if [ "$USE_DOCKER" -eq 1 ]; then
        # When --devphysx is used, mount the whole repo root so physx/ is accessible
        if [ "$DO_DEV_PHYSX" -eq 1 ]; then
            _DOCKER_MOUNT="$REPO_ROOT:$REPO_ROOT"
        else
            _DOCKER_MOUNT="$OMNI_DIR:$OMNI_DIR"
        fi
        # Collect extra mounts for symlinks under target-deps that point outside
        # the primary mount (e.g. local <source path="..."> packman dependencies).
        _EXTRA_MOUNTS=()
        if [ -d "$SCRIPT_DIR/_build/target-deps" ]; then
            for _link in "$SCRIPT_DIR/_build/target-deps"/*; do
                [ -L "$_link" ] || continue
                _target="$(readlink -f "$_link" 2>/dev/null)" || continue
                [ -d "$_target" ] || continue
                # Skip if already under the primary mount or packman cache
                case "$_target" in
                    "$OMNI_DIR"/*|"$REPO_ROOT"/*|"$PACKMAN_CACHE"/*) continue ;;
                esac
                _EXTRA_MOUNTS+=(-v "$_target:$_target:ro")
            done
        fi
        docker run --rm \
            -v "$_DOCKER_MOUNT" \
            -v "$PACKMAN_CACHE:$PACKMAN_CACHE:ro" \
            "${_EXTRA_MOUNTS[@]}" \
            -w "$SCRIPT_DIR" \
            -u "$(id -u):$(id -g)" \
            "$DOCKER_IMAGE" \
            bash -c '"$@"' _ "$@"
    else
        "$@"
    fi
}

# Record overall start time
START_TIME=$(now)

# Clean _* folders
if [ "$DO_CLEAN_INTERNAL" -eq 1 ]; then
    echo "Cleaning build artifacts..."
    for d in _*/; do
        [ -d "$d" ] && echo "  Removing $d" && rm -rf "$d"
    done
fi

# -c/--clean: clean only, exit
if [ "$DO_CLEAN" -eq 1 ] && [ "$DO_REBUILD" -eq 0 ]; then
    echo "Clean complete."
    exit 0
fi

# Pull dependencies on the host (packman needs access to ../ovexts/tools/)
echo "Pulling dependencies..."
if [ "$DO_DEV_PHYSX" -eq 1 ]; then
    ./pull_dependencies.sh --devphysx
else
    ./pull_dependencies.sh
fi

mkdir -p "$BUILD_BASE"
mkdir -p "$COMPILER_BASE"

# Absolute path for PX_OUTPUT_*_DIR (final libs/bins go here, not in the per-config cmake dir)
OUTPUT_BASE="${OVRUNTIME_OUTPUT_BASE:-$SCRIPT_DIR/$BUILD_BASE}"

# Use packman-provided cmake (compatible with Docker container's glibc)
CMAKE="$SCRIPT_DIR/_build/target-deps/cmake/bin/cmake"
if [ ! -x "$CMAKE" ]; then
    echo "Warning: packman cmake not found at $CMAKE, falling back to system cmake"
    CMAKE="cmake"
fi

# Detect CPU count respecting cgroup limits (Kubernetes / Docker).
# Cap at 12 to avoid OOM from concurrent linker instances.
if [ -f "$REPO_ROOT/ci/helpers/get_build_jobs.sh" ]; then
    source "$REPO_ROOT/ci/helpers/get_build_jobs.sh"
    BUILD_JOBS=$(get_build_jobs 12)
else
    BUILD_JOBS=$(nproc)
fi

if [ "$USE_DOCKER" -eq 1 ]; then
    echo "Building inside Docker ($DOCKER_IMAGE)..."
fi

echo "Building with $BUILD_JOBS jobs..."

PYTHON_CMAKE_ARGS=()
if [ -n "$PYTHON_DIR_OVERRIDE" ]; then
    PYTHON_CMAKE_ARGS+=("-DPYTHON_DIR=$PYTHON_DIR_OVERRIDE")
fi

# Optional: build local schema first (for --devschema)
if [ "$DO_DEV_SCHEMA" -eq 1 ] && [ "$DO_SETUP_VENV" -eq 0 ]; then
    echo "DevSchema mode: building local physics schema..."
    SCHEMA_BUILD_ARGS=""
    [ "$DO_DEBUG"   -eq 1 ] && [ "$DO_RELEASE" -eq 0 ] && SCHEMA_BUILD_ARGS="$SCHEMA_BUILD_ARGS -d"
    [ "$DO_RELEASE" -eq 1 ] && [ "$DO_DEBUG"   -eq 0 ] && SCHEMA_BUILD_ARGS="$SCHEMA_BUILD_ARGS -r"
    SCHEMA_FETCH_ARGS="$SCHEMA_BUILD_ARGS"
    [ "$DO_CLEAN_INTERNAL" -eq 1 ] && SCHEMA_FETCH_ARGS="$SCHEMA_FETCH_ARGS -x"
    SCHEMA_DIR="$(cd "$SCRIPT_DIR/../schema" && pwd)"
    pushd "$SCHEMA_DIR"
    ./repo.sh build --fetch-only $SCHEMA_FETCH_ARGS
    ./repo.sh usd
    ./repo.sh build $SCHEMA_BUILD_ARGS
    popd
fi

# Optional: build PhysX SDK from source
CMAKE_EXTRA_ARGS=""
if [ "$DO_DEV_PHYSX" -eq 1 ]; then
    echo "DevPhysX mode: building PhysX SDK from source..."
    CMAKE_EXTRA_ARGS="-DOVRUNTIME_DEV_PHYSX=ON"
fi
if [ "$DO_DEV_SCHEMA" -eq 1 ]; then
    CMAKE_EXTRA_ARGS="$CMAKE_EXTRA_ARGS -DOVRUNTIME_DEV_SCHEMA=ON"
fi
if [ "$DO_PER_MODULE" -eq 1 ]; then
    echo "Per-module install layout enabled."
    CMAKE_EXTRA_ARGS="$CMAKE_EXTRA_ARGS -DOVRUNTIME_INSTALL_PER_MODULE=ON"
fi
# Configure (separate build dirs so intermediates don't conflict between Debug and Release)
if [ "$DO_SETUP_VENV" -eq 0 ] && [ "$DO_DEBUG" -eq 1 ]; then
    echo "Configuring Debug build..."
    run_cmd "$CMAKE" -S . -B "$COMPILER_BASE/build-debug" -G "Unix Makefiles" \
        -DCMAKE_BUILD_TYPE=Debug -DTARGET_BUILD_PLATFORM=linux \
        "-DPX_OUTPUT_LIB_DIR=$OUTPUT_BASE" "-DPX_OUTPUT_BIN_DIR=$OUTPUT_BASE" \
        "${PYTHON_CMAKE_ARGS[@]}" \
        $CMAKE_EXTRA_ARGS
fi
if [ "$DO_SETUP_VENV" -eq 0 ] && [ "$DO_RELEASE" -eq 1 ]; then
    echo "Configuring Release build..."
    run_cmd "$CMAKE" -S . -B "$COMPILER_BASE/build-release" -G "Unix Makefiles" \
        -DCMAKE_BUILD_TYPE=Release -DTARGET_BUILD_PLATFORM=linux \
        "-DPX_OUTPUT_LIB_DIR=$OUTPUT_BASE" "-DPX_OUTPUT_BIN_DIR=$OUTPUT_BASE" \
        "${PYTHON_CMAKE_ARGS[@]}" \
        $CMAKE_EXTRA_ARGS
fi

# Generate test helper scripts into _build/
if [ "$DO_SETUP_VENV" -eq 0 ]; then
    mkdir -p "_build"
    {
        echo '#!/bin/bash'
        echo 'set -e'
        echo 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"'
        echo 'CTEST="$SCRIPT_DIR/target-deps/cmake/bin/ctest"'
        echo '[ -x "$CTEST" ] || CTEST=ctest'
        echo '"$CTEST" --test-dir "$SCRIPT_DIR/../'"$COMPILER_BASE"'/build-release" -C Release -V'
    } > "_build/test_unit.sh"
    # Only generate test_python.sh when USD has Python support (usd.nopy has no lib/python).
    if [ -d "_build/target-deps/usd/release/lib/python" ]; then
        {
            echo '#!/bin/bash'
            echo 'set -e'
            echo 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"'
            echo 'cd "$SCRIPT_DIR/.."'
            echo '"$SCRIPT_DIR/.venv/bin/python" -m pytest --junitxml="$SCRIPT_DIR/../pytest_results.xml"'
        } > "_build/test_python.sh"
        chmod +x "_build/test_python.sh"
    else
        {
            echo '#!/bin/bash'
            echo 'echo "Skipping Python tests (USD built without Python support)"'
            echo 'exit 0'
        } > "_build/test_python.sh"
        chmod +x "_build/test_python.sh"
        echo "Generated no-op test_python.sh (USD built without Python support)"
    fi
    chmod +x "_build/test_unit.sh"
fi

# -g: stop after generating build files (unless -t also requested — venv setup still runs)
if [ "$DO_GET_DEPS_AND_CONFIGURE" -eq 1 ] && [ "$DO_SETUP_VENV" -eq 0 ]; then
    echo ""
    echo "Generate completed successfully in $(format_elapsed $(($(now) - START_TIME)))"
    exit 0
fi

# Build Debug
if [ "$DO_SETUP_VENV" -eq 0 ] && [ "$DO_DEBUG" -eq 1 ]; then
    T0=$(now)
    if [ -n "$BUILD_TARGET" ]; then
        run_cmd "$CMAKE" --build "$COMPILER_BASE/build-debug" --target "$BUILD_TARGET" -j$BUILD_JOBS
    else
        run_cmd "$CMAKE" --build "$COMPILER_BASE/build-debug" -j$BUILD_JOBS
    fi
    echo "Debug build: $(format_elapsed $(($(now) - T0)))"
    if [ -z "$BUILD_TARGET" ] && [ "$SKIP_INSTALL" -ne 1 ]; then
        run_cmd "$CMAKE" --install "$COMPILER_BASE/build-debug" --prefix "$INSTALL_DIR/debug"
    fi
fi

# Build Release
if [ "$DO_SETUP_VENV" -eq 0 ] && [ "$DO_RELEASE" -eq 1 ]; then
    T0=$(now)
    if [ -n "$BUILD_TARGET" ]; then
        run_cmd "$CMAKE" --build "$COMPILER_BASE/build-release" --target "$BUILD_TARGET" -j$BUILD_JOBS
    else
        run_cmd "$CMAKE" --build "$COMPILER_BASE/build-release" -j$BUILD_JOBS
    fi
    echo "Release build: $(format_elapsed $(($(now) - T0)))"
    if [ -z "$BUILD_TARGET" ] && [ "$SKIP_INSTALL" -ne 1 ]; then
        run_cmd "$CMAKE" --install "$COMPILER_BASE/build-release" --prefix "$INSTALL_DIR/release"
    fi
fi

if [ "$DO_SETUP_VENV" -eq 0 ] && { [ -n "$BUILD_TARGET" ] || [ "$SKIP_VENV" -eq 1 ]; }; then
    echo ""
    echo "Build completed successfully in $(format_elapsed $(($(now) - START_TIME)))"
    exit 0
fi

# Skip venv setup entirely when USD has no Python support (usd.nopy package).
if [ ! -d "_build/target-deps/usd/release/lib/python" ]; then
    echo "Skipping Python test environment setup (USD built without Python support)"
    echo ""
    echo "Build completed successfully in $(format_elapsed $(($(now) - START_TIME)))"
    exit 0
fi

# Setup Python test environment (.venv for VS Code debugging)
# Uses packman Python to ensure version matches native extensions
PACKMAN_PYTHON="_build/target-deps/python/python"
if [ ! -d "_build/.venv" ]; then
    echo "Creating Python test environment..."
    "$PACKMAN_PYTHON" -m venv _build/.venv
    _build/.venv/bin/python -m pip install --quiet pytest numpy warp-lang debugpy
else
    echo "Python test environment already exists."
fi
if [ "$DO_SETUP_VENV" -eq 1 ]; then
    exit 0
fi

# Install Python package paths into the venv via .pth files
# This makes "from pxr import Usd, ..." and "import carb" work for VS Code
VENV_SITE=$(_build/.venv/bin/python -c "import sysconfig; print(sysconfig.get_path('purelib'))")

if [ -d "_build/target-deps/usd/release/lib/python" ]; then
    USD_PYTHON_DIR="$(cd _build/target-deps/usd/release/lib/python && pwd)"
    echo "$USD_PYTHON_DIR" > "$VENV_SITE/usd.pth"
    echo "Installed USD Python path: $VENV_SITE/usd.pth"
fi

CARB_PY_DIR="_build/target-deps/carb_sdk_plugins/$BUILD_BASE/release/bindings-python"
if [ -d "$CARB_PY_DIR" ]; then
    CARB_PY_ABS="$(cd "$CARB_PY_DIR" && pwd)"
    echo "$CARB_PY_ABS" > "$VENV_SITE/carb.pth"
    echo "Installed Carb Python path: $VENV_SITE/carb.pth"
fi

if [ "$DO_DEV_SCHEMA" -eq 1 ]; then
    PHYSX_SCHEMA_PY_DIR="$(cd "$SCRIPT_DIR/../schema" && pwd)/$BUILD_BASE/release/schema/lib/python"
else
    PHYSX_SCHEMA_PY_DIR="_build/target-deps/usd_ext_physics/release/lib/python"
fi
if [ -d "$PHYSX_SCHEMA_PY_DIR" ]; then
    PHYSX_SCHEMA_PY_ABS="$(cd "$PHYSX_SCHEMA_PY_DIR" && pwd)"
    echo "$PHYSX_SCHEMA_PY_ABS" > "$VENV_SITE/physx_schema.pth"
    echo "Installed PhysxSchema Python path: $VENV_SITE/physx_schema.pth"
fi

echo ""
echo "Build completed successfully in $(format_elapsed $(($(now) - START_TIME)))"
