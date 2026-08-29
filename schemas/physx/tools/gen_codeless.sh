#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Build step for the codeless schemas. Regenerates everything from schema.usda using only
# this tooling + native OpenUSD usdGenSchema (NO repo_usd):
#   1. generatedSchema.usda + plugInfo.json for each codeless schema (tools/gen_schema_data.py
#      -> native usdGenSchema) -- the data USD reads to register the prim/API types.
#   2. physxSchema/tokens.h (header-only C++ tokens) + physxSchema/_tokens.py (pure-Python
#      PhysxSchema.Tokens) -- tools/gen_tokens.py calls usdGenSchema's GatherTokens directly.
#   3. header-only C++ wrappers + Python API (tools/gen_codeless_api.py), base classes resolved
#      from the schema's `inherits` via the USD registry.
#
# Result: a fully codeless schema with NO compiled schema libraries, no codefull generation,
# and no repo_usd dependency.
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT="$SCRIPT_DIR/.."
SRC="$ROOT/source/physxSchema"

USD=""
for cfg in release debug; do
    if [ -d "$ROOT/_build/target-deps/usd/$cfg/lib/python/pxr" ]; then
        USD="$ROOT/_build/target-deps/usd/$cfg"
        break
    fi
done
if [ -z "$USD" ]; then
    echo "[gen_codeless] ERROR: could not find target-deps USD with pxr bindings; run after deps fetch." >&2
    exit 1
fi

USD_PY="$ROOT/_build/target-deps/python/python"
export LD_LIBRARY_PATH="$USD/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# usdGenSchema (imported by gen_tokens.py) needs jinja2. Install it locally via pip
# rather than borrowing it from repo_usd (deprecated). Cached under _build after first run.
JINJA_DIR="$ROOT/_build/codeless_gen_deps"
if [ ! -d "$JINJA_DIR/jinja2" ]; then
    echo "[gen_codeless] installing jinja2 (pip) for usdGenSchema"
    "$USD_PY" -m pip install --quiet --disable-pip-version-check --target "$JINJA_DIR" jinja2
fi
export PYTHONPATH="$JINJA_DIR:$USD/lib/python${PYTHONPATH:+:$PYTHONPATH}"

# 1. schema DATA (generatedSchema.usda + plugInfo.json) via native usdGenSchema, for every
#    codeless schema. Replaces repo.sh usd (repo_usd).
for SCHEMA_DIR in "$SRC" "$ROOT/source/omniUsdPhysicsDeformableSchema"; do
    echo "[gen_codeless] generating schema data (native usdGenSchema): $SCHEMA_DIR"
    "$USD_PY" "$ROOT/tools/gen_schema_data.py" --schema-dir "$SCHEMA_DIR"
done

# 2.+3. header-only tokens (C++) + pure-Python Tokens, then header-only C++ wrappers + Python
#       API, straight from schema.usda -- for every codeless schema. The C++/Python library
#       prefix (e.g. PhysxSchema -> PhysxSchemaTokens / PhysxSchema.<Class>) and the include
#       directory are passed per schema so a single generator serves all codeless schemas.
gen_codeless_one() {
    local DIR="$1" PREFIX="$2" LIBDIR="$3"
    echo "[gen_codeless] regenerating header-only $PREFIX tokens from schema.usda"
    "$USD_PY" "$ROOT/tools/gen_tokens.py" \
        --schema "$DIR/schema.usda" \
        --out-h  "$DIR/tokens.h" \
        --out-py "$DIR/_tokens.py" \
        --prefix "$PREFIX" \
        --lib-dir "$LIBDIR"
    echo "[gen_codeless] generating codeless $PREFIX API from schema.usda (USD: $USD)"
    "$USD_PY" "$ROOT/tools/gen_codeless_api.py" \
        --schema   "$DIR/schema.usda" \
        --tokens-h "$DIR/tokens.h" \
        --out-cpp  "$DIR" \
        --out-py   "$DIR/codeless_api.py" \
        --prefix "$PREFIX" \
        --lib-dir "$LIBDIR"
}

gen_codeless_one "$SRC" "PhysxSchema" "physxSchema"
gen_codeless_one "$ROOT/source/omniUsdPhysicsDeformableSchema" \
    "OmniUsdPhysicsDeformableSchema" "omniUsdPhysicsDeformableSchema"

# 4. derived per-axis instance tokens for physxJointAxis / physxDrivePerformanceEnvelope, built
#    from the codeless PhysxSchemaTokens *_MultipleApplyTemplate_* tokens (replaces the old
#    hand-authored physicsSchemaTools per-axis constants). Must run after physxSchema tokens.
echo "[gen_codeless] generating PhysxAxisInstanceTokens from physxSchema templates"
"$USD_PY" "$ROOT/tools/gen_axis_instance_tokens.py" \
    --tokens-h "$SRC/tokens.h" \
    --out-h    "$SRC/axisInstanceTokens.h"
