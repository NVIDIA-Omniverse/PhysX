#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

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

# USD version comes from ovruntime's USD pin (stock variant here, py-less
# there), keeping schema aligned with ovruntime and ovphysx.
USD_VER=$(sed -n 's/.*name="usd\.nopy[^"]*"[[:space:]]*version="\([^"]*\)".*/\1/p' \
    "$SCRIPT_DIR/../../ovphysx/ovruntime/deps/usd-deps.packman.xml" | head -1)
if [ -z "$USD_VER" ]; then
    echo "ERROR: could not read the USD version from ovruntime's USD pin." >&2
    exit 1
fi
echo "USD version (from ovruntime): $USD_VER"

"$PACKMAN" pull "$SCRIPT_DIR/deps/usd-deps.packman.xml" -p "$PACKMAN_PLATFORM" -t "config=release" -t "platform_target_abi=$PACKMAN_PLATFORM" -t "usd_ver=$USD_VER"

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
