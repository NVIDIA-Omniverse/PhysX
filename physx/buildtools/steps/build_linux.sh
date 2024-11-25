#!/bin/bash +x

set -e

# Function to display an error and exit
error_exit() {
    echo "$1" 1>&2
    exit 1
}

# Check if two arguments are passed
if [ "$#" -ne 2 ]; then
    error_exit "Usage: $0 <preset> <build_config>. Example: $0 linux-aarch64-gcc debug"
fi

# Assign arguments
PRESET="$1"
BUILD_CONFIG="$2"

# Validate build configuration
if [[ ! "checked debug profile release all" =~ (^|[[:space:]])$BUILD_CONFIG($|[[:space:]]) ]]; then
    error_exit "Invalid build configuration. Use one of: checked, debug, profile, release, all."
fi

# Get number of CPU cores
if [ -f /proc/cpuinfo ]; then
    CPUS=$(grep processor /proc/cpuinfo | wc -l)
else
    CPUS=1
fi

# Stackoverflow suggests jobs count of (CPU cores + 1) as a good number!
JOBS=$(expr $CPUS + 1)

# Define build function for presets other than linux-carbonite and linux-aarch64-carbonite (no install)
build() {
    CONFIG=$1
    BUILD_DIR="$(dirname "$0")/../../compiler/$PRESET-$CONFIG"
    pushd "$BUILD_DIR" || error_exit "Directory not found for build: $BUILD_DIR"
    make -j$JOBS || error_exit "Build failed for $PRESET-$CONFIG"
    popd
}

# Function to handle file copying for linux-aarch64-carbonite and linux-carbonite debug builds
copy_vhacd_files() {
    CONFIG=$1
    if [[ "$CONFIG" == "debug" ]]; then
        if [[ "$PRESET" == "linux-aarch64-carbonite" ]]; then
            TARGET_PATH="install/$PRESET/VHACD/bin/linux.aarch64/debug/"
            SRC_PATH="bin/linux.aarch64/debug/"
        elif [[ "$PRESET" == "linux-carbonite" ]]; then
            TARGET_PATH="install/$PRESET/VHACD/bin/linux.x86_64/debug/"
            SRC_PATH="bin/linux.x86_64/debug/"
        fi
        mkdir -p "$TARGET_PATH" || error_exit "Failed to create directory $TARGET_PATH"
        
        # Check if VHACD files exist before attempting to copy
        if ls "$SRC_PATH"*VHACD* 1> /dev/null 2>&1; then
            cp "$SRC_PATH"*VHACD* "$TARGET_PATH" || error_exit "Failed to copy VHACD files"
        else
            echo "Warning: No VHACD files found in $SRC_PATH. Skipping copy operation."
        fi
    fi
}

# Define build function for linux-aarch64-carbonite and linux-carbonite (with install)
build_with_install() {
    CONFIG=$1
    BUILD_DIR="$(dirname "$0")/../../compiler/$PRESET-$CONFIG"
    pushd "$BUILD_DIR" || error_exit "Directory not found for build: $BUILD_DIR"
    make -j$JOBS install || error_exit "Build and install failed for $PRESET-$CONFIG"
    popd

    copy_vhacd_files $CONFIG
}

# Build process based on the preset and configuration
if [[ "$PRESET" == "linux-aarch64-carbonite" || "$PRESET" == "linux-carbonite" ]]; then
    # Build with install for linux-aarch64-carbonite and linux-carbonite
    if [ "$BUILD_CONFIG" = "all" ]; then
        build_with_install debug
        build_with_install checked
        build_with_install profile
        build_with_install release
    else
        build_with_install $BUILD_CONFIG
    fi

    # Additional installations not specific to any build
    INSTALL_PATH="install/$PRESET"

    pushd "$(dirname "$0")/../.." || error_exit "Failed to enter base directory"
    mkdir -p "$INSTALL_PATH/PhysX/PACKAGE-LICENSES/" "$INSTALL_PATH/VHACD/" "$INSTALL_PATH/VHACD/include/" "$INSTALL_PATH/VHACD/PACKAGE-LICENSES/" || error_exit "Failed to create installation directories"

    cp "documentation/license/PACKAGE-LICENSES/LICENSE.md" "$INSTALL_PATH/PhysX/PACKAGE-LICENSES/physxsdk-LICENSE.md" || error_exit "Failed to copy PhysX license"
    cp "documentation/license/PACKAGE-LICENSES/vhacd-LICENSE.md" "$INSTALL_PATH/VHACD/PACKAGE-LICENSES/vhacd-LICENSE.md" || error_exit "Failed to copy VHACD license"

    cp "documentation/license/physxsdk-PACKAGE-INFO.yaml" "$INSTALL_PATH/PhysX/PACKAGE-INFO.yaml" || error_exit "Failed to copy PhysX package info"
    cp "documentation/license/vhacd-PACKAGE-INFO.yaml" "$INSTALL_PATH/VHACD/PACKAGE-INFO.yaml" || error_exit "Failed to copy VHACD package info"

    cp "externals/VHACD/public/"* "$INSTALL_PATH/VHACD/include/" || error_exit "Failed to copy VHACD include files"
    popd
else
    # Build without install for other presets
    if [ "$BUILD_CONFIG" = "all" ]; then
        build checked
        build debug
        build profile
        build release
    else
        build $BUILD_CONFIG
    fi
fi
