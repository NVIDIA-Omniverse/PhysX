#!/usr/bin/env bash
# Compiles TypeScript and assembles the demo site into .vercel/output for deployment.
# WASM artifacts (stress_solver.wasm etc.) must be pre-built if needed.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/.vercel/output/static"

rm -rf "$OUT"
mkdir -p "$OUT"

# ── Compile TypeScript sources ──────────────────────────────
DEMO_SRC="$ROOT/blast/js_stress_example"

echo "Building TypeScript in js_stress_example..."
# noEmitOnError is false in tsconfig so tsc emits JS even with type errors,
# but still exits non-zero. Ignore the exit code since the output is valid.
(cd "$DEMO_SRC" && npm install --ignore-scripts 2>/dev/null; npx -y tsc || true)

# Ensure blast-stress-solver dev deps (three-pinata) are installed for vendor copy
BSS_DIR="$ROOT/blast/blast-stress-solver"
if [ -d "$BSS_DIR" ] && [ ! -d "$BSS_DIR/node_modules/@dgreenheck/three-pinata" ]; then
  echo "Installing blast-stress-solver dependencies for vendor assets..."
  (cd "$BSS_DIR" && npm install --ignore-scripts 2>/dev/null || true)
fi

# ── Demo pages ──────────────────────────────────────────────

# HTML files — use the demo index as the landing page
cp "$DEMO_SRC/demo-index.html" "$OUT/index.html"
for f in "$DEMO_SRC"/*.html; do
  cp "$f" "$OUT/$(basename "$f")"
done

# Compiled JS + WASM
mkdir -p "$OUT/dist"
if [ -d "$DEMO_SRC/dist" ]; then
  cp -r "$DEMO_SRC/dist/." "$OUT/dist/"
fi

# Styles
if [ -d "$DEMO_SRC/styles" ]; then
  mkdir -p "$OUT/styles"
  cp -r "$DEMO_SRC/styles/." "$OUT/styles/"
fi

# ── Vendor libraries (mirror the aliases from serve-demo.mjs) ─
NODE_MODULES="$ROOT/node_modules"

# three.js
if [ -d "$NODE_MODULES/three" ]; then
  mkdir -p "$OUT/vendor/three"
  cp -r "$NODE_MODULES/three/build" "$OUT/vendor/three/build"
  cp -r "$NODE_MODULES/three/examples" "$OUT/vendor/three/examples"
fi

# Rapier
RAPIER_DIR="$NODE_MODULES/@dimforge/rapier3d-compat"
if [ -d "$RAPIER_DIR" ]; then
  mkdir -p "$OUT/vendor/rapier"
  cp "$RAPIER_DIR"/rapier.mjs "$OUT/vendor/rapier/" 2>/dev/null || true
  cp "$RAPIER_DIR"/rapier_wasm3d_bg.wasm "$OUT/vendor/rapier/" 2>/dev/null || true
  # Copy all JS/WASM files in case the package layout differs
  for ext in js mjs wasm; do
    find "$RAPIER_DIR" -maxdepth 1 -name "*.$ext" -exec cp {} "$OUT/vendor/rapier/" \; 2>/dev/null || true
  done
fi

# blast-stress-solver package (high-level rapier + three APIs)
BSS_DIST="$ROOT/blast/blast-stress-solver/dist"
if [ -d "$BSS_DIST" ]; then
  mkdir -p "$OUT/vendor/blast-stress-solver"
  # Copy ESM bundles and WASM — the import maps in the new demos resolve here
  for f in "$BSS_DIST"/*.mjs "$BSS_DIST"/*.wasm "$BSS_DIST"/*.js; do
    [ -f "$f" ] && cp "$f" "$OUT/vendor/blast-stress-solver/"
  done
  echo "Copied blast-stress-solver dist to vendor/"
fi

# three-pinata (optional — used by fractured-wall demo)
PINATA_DIR=""
if [ -d "$NODE_MODULES/@dgreenheck/three-pinata/build" ]; then
  PINATA_DIR="$NODE_MODULES/@dgreenheck/three-pinata/build"
elif [ -d "$ROOT/blast/blast-stress-solver/node_modules/@dgreenheck/three-pinata/build" ]; then
  PINATA_DIR="$ROOT/blast/blast-stress-solver/node_modules/@dgreenheck/three-pinata/build"
fi
if [ -n "$PINATA_DIR" ]; then
  mkdir -p "$OUT/vendor/three-pinata"
  cp "$PINATA_DIR"/*.js "$OUT/vendor/three-pinata/" 2>/dev/null || true
  echo "Copied three-pinata to vendor/"
fi

# ── Other demo directories ──────────────────────────────────
if [ -d "$ROOT/demos" ]; then
  cp -r "$ROOT/demos" "$OUT/demos"
fi

# ── Vercel output config ────────────────────────────────────
mkdir -p "$ROOT/.vercel/output"
cat > "$ROOT/.vercel/output/config.json" <<'EOF'
{
  "version": 3,
  "routes": [
    {
      "src": "/(.*)\\.wasm",
      "headers": { "Content-Type": "application/wasm" }
    },
    {
      "src": "/(.*)",
      "headers": {
        "Cross-Origin-Opener-Policy": "same-origin",
        "Cross-Origin-Embedder-Policy": "require-corp"
      }
    }
  ]
}
EOF

echo "Demo site assembled at $OUT"
ls -la "$OUT"
