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

# Check for required system packages
echo "Checking required system packages..."
MISSING_PACKAGES=()

# Check for RapidJSON headers
if [ ! -f "/usr/include/rapidjson/document.h" ]; then
    MISSING_PACKAGES+=("rapidjson-dev")
fi

# Check for OpenGL/GLUT headers
if [ ! -f "/usr/include/GL/glut.h" ]; then
    MISSING_PACKAGES+=("libglut-dev")
fi

if [ ! -f "/usr/include/GL/glu.h" ]; then
    MISSING_PACKAGES+=("libglu1-mesa-dev")
fi

# Check for X11 headers
if [ ! -f "/usr/include/X11/Xlib.h" ]; then
    MISSING_PACKAGES+=("libx11-dev")
fi

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo ""
    echo "ERROR: Missing required system packages!"
    echo ""
    echo "The following packages are required but not installed:"
    for pkg in "${MISSING_PACKAGES[@]}"; do
        echo "  - $pkg"
    done
    echo ""
    echo "Please install them using:"
    echo "  sudo apt-get install ${MISSING_PACKAGES[*]}"
    echo ""
    echo "Full list of required packages:"
    echo "  sudo apt-get install cmake clang build-essential curl \\"
    echo "                       libglut-dev libglu1-mesa-dev libopengl-dev \\"
    echo "                       rapidjson-dev libx11-dev libxext-dev"
    echo ""
    exit 1
fi

echo "✓ All required packages found"
echo ""
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
