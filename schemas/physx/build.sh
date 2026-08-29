#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PY="$SCRIPT_DIR/_build/target-deps/python/python"

case "$(uname -m)" in
    aarch64) PACKMAN_PLATFORM="manylinux_2_35_aarch64" ;;
    *)       PACKMAN_PLATFORM="manylinux_2_35_x86_64" ;;
esac

# Fetch release deps (schema is codeless; artifacts are platform- and config-independent).
PACKMAN="$SCRIPT_DIR/tools/packman/packman"
"$PACKMAN" pull "$SCRIPT_DIR/deps/host-deps.packman.xml" -p "$PACKMAN_PLATFORM"
"$PACKMAN" pull "$SCRIPT_DIR/deps/kit-kernel-deps.packman.xml" -p "$PACKMAN_PLATFORM" -t "config=release" -t "platform_target_abi=$PACKMAN_PLATFORM"
"$PACKMAN" pull "$SCRIPT_DIR/deps/usd-deps.packman.xml" -p "$PACKMAN_PLATFORM" -t "config=release" -t "platform_target_abi=$PACKMAN_PLATFORM"

CMAKE="$SCRIPT_DIR/_build/host-deps/cmake/bin/cmake"
if [ ! -x "$CMAKE" ]; then
    echo "Warning: packman cmake not found at $CMAKE, falling back to system cmake"
    CMAKE="cmake"
fi

# Validate USD compatibility before using it.
"$PY" "$SCRIPT_DIR/tools/check_usd_version.py"

# Regenerate the codeless schema files (generatedSchema.usda + plugInfo.json + C++/Python
# wrappers + tokens) from schema.usda via native usdGenSchema (no repo_usd, no premake).
"$SCRIPT_DIR/tools/gen_codeless.sh"

# Install schema artifacts and generate the unit database.
"$CMAKE" -P "$SCRIPT_DIR/tools/install.cmake"
"$PY" "$SCRIPT_DIR/tools/gen_unit_database.py" \
    "$SCRIPT_DIR/_build/schema/lib/python/PhysicsSchemaTools"
