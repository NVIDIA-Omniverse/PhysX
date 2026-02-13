#!/bin/bash
# Alternative PhysX project generator that uses system tools instead of packman
# Usage: ./generate_projects_no_packman.sh <preset-name>
# Example: ./generate_projects_no_packman.sh linux-clang-cpu-only

set -e

SCRIPT_DIR=$(dirname "${BASH_SOURCE[0]}")
PHYSX_ROOT_DIR=$(cd "$SCRIPT_DIR" && pwd)

export PHYSX_ROOT_DIR="$PHYSX_ROOT_DIR"

# Use system paths instead of packman packages
# Set PM_PATHS to /usr for system packages (OpenGL, RapidJSON, etc.)
export PM_PATHS="/usr"

echo "========================================="
echo "PhysX Project Generator (No Packman Mode)"
echo "========================================="
echo "Using system build tools:"
echo "  CMake: $(which cmake) ($(cmake --version | head -1))"
echo "  Clang: $(which clang++) ($(clang++ --version | head -1))"
echo "  Make: $(which make) ($(make --version | head -1))"
echo ""
echo "Using system packages:"
echo "  OpenGL/GLUT: System libraries"
echo "  RapidJSON: /usr/include/rapidjson"
echo ""
echo "PhysX Root: $PHYSX_ROOT_DIR"
echo "CMake Prefix Path: $PM_PATHS"
echo "========================================="
echo ""

if [ $# -eq 0 ]; then
    echo "Running project generator..."
    python3 "$SCRIPT_DIR/buildtools/cmake_generate_projects.py"
else
    echo "Generating project for preset: $1"
    python3 "$SCRIPT_DIR/buildtools/cmake_generate_projects.py" "$1"
fi

echo ""
echo "========================================="
echo "Generation complete!"
echo "Build directories created in: $PHYSX_ROOT_DIR/compiler/"
echo ""
echo "To build, run:"
echo "  cd compiler/<preset-name>-<config>/"
echo "  make -j\$(nproc)"
echo "========================================="
