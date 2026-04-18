#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
CRATE_DIR="$REPO_ROOT/blast/blast-stress-solver-rs"
DEMO_DIR="$REPO_ROOT/blast/blast-stress-demo-rs"
SUPPORT_DIR="$SCRIPT_DIR/blast-stress-solver-package"
PROOF_DIR="$CRATE_DIR/target/packaging-proof"
HOST_TARGET="$(rustc -vV | awk '/^host: / { print $2 }')"
NATIVE_TARGET="aarch64-apple-darwin"
WASM_TARGET="wasm32-unknown-unknown"
EMSCRIPTEN_SYSROOT_TARGET="wasm32-emscripten"
PACKAGE_SIZE_LIMIT=$((10 * 1024 * 1024))

STAGE_DIR=""
KEEP_STAGE=0
VERIFY_NATIVE=1
VERIFY_WASM=1
VERIFY_DEMO_CONSUMER=0

usage() {
  cat <<'EOF'
Usage: scripts/assemble-blast-stress-solver-package.sh [options]

Assemble a temporary publish-style crate directory for blast-stress-solver with
packaged native and wasm backend artifacts, without changing the monorepo
layout.

Options:
  --stage-dir <dir>      Reuse a specific output directory instead of mktemp.
  --keep-stage           Keep the temporary stage directory on exit.
  --skip-native-verify   Skip the downstream native package proof.
  --skip-wasm-verify     Skip the downstream wasm package proof.
  --verify-demo-consumer Run the real blast-stress-demo-rs headless proof
                         against the staged package.
  -h, --help             Show this help text.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stage-dir)
      STAGE_DIR="$2"
      shift 2
      ;;
    --keep-stage)
      KEEP_STAGE=1
      shift
      ;;
    --skip-native-verify)
      VERIFY_NATIVE=0
      shift
      ;;
    --skip-wasm-verify)
      VERIFY_WASM=0
      shift
      ;;
    --verify-demo-consumer)
      VERIFY_DEMO_CONSUMER=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -d "$CRATE_DIR" ]]; then
  echo "Expected crate directory at $CRATE_DIR" >&2
  exit 1
fi

if [[ ! -f "$SUPPORT_DIR/staged-build.rs" ]]; then
  echo "Missing staged build template under $SUPPORT_DIR" >&2
  exit 1
fi

cleanup() {
  if [[ $KEEP_STAGE -eq 0 && -n "${STAGE_DIR:-}" && -d "$STAGE_DIR" ]]; then
    rm -rf "$STAGE_DIR"
  fi
}

if [[ -z "$STAGE_DIR" ]]; then
  STAGE_DIR="$(mktemp -d "${TMPDIR:-/tmp}/blast-stress-solver-package.XXXXXX")"
else
  rm -rf "$STAGE_DIR"
  mkdir -p "$STAGE_DIR"
fi

trap cleanup EXIT
mkdir -p "$PROOF_DIR"

copy_current_crate_files() {
  local src rel dest
  while IFS= read -r src; do
    rel="${src#$CRATE_DIR/}"
    dest="$STAGE_DIR/$rel"
    mkdir -p "$(dirname "$dest")"
    cp "$src" "$dest"
  done < <(
    find "$CRATE_DIR" \
      \( -path "$CRATE_DIR/target" -o -path "$CRATE_DIR/target/*" -o -path "$CRATE_DIR/.git" -o -path "$CRATE_DIR/.git/*" \) -prune \
      -o -type f -print | sort
  )
}

inject_publish_metadata() {
  env LC_ALL=C perl -0pi -e 's/edition = "2021"\n/edition = "2021"\ninclude = [\n  "Cargo.toml",\n  "build.rs",\n  "README.md",\n  "LICENSE.md",\n  "WASM_FEASIBILITY.md",\n  "examples\/**",\n  "native\/**",\n  "src\/**",\n  "tests\/**",\n  "artifacts\/**"\n]\n/' \
    "$STAGE_DIR/Cargo.toml"
  env LC_ALL=C perl -0pi -e 's/^publish = false\n//m' \
    "$STAGE_DIR/Cargo.toml"

  cat <<'EOF' >> "$STAGE_DIR/Cargo.toml"

[package.metadata.docs.rs]
features = ["scenarios", "rapier"]
default-target = "aarch64-apple-darwin"
targets = ["aarch64-apple-darwin", "wasm32-unknown-unknown"]
EOF
}

find_native_backend() {
  local artifact
  artifact="$(
    find "$CRATE_DIR/target/release/build" -path '*/out/libblast_stress_solver_ffi.a' -print0 \
      | xargs -0 ls -t 2>/dev/null \
      | head -n 1
  )"

  if [[ -z "$artifact" ]]; then
    echo "Failed to locate libblast_stress_solver_ffi.a in $CRATE_DIR/target/release/build" >&2
    exit 1
  fi

  printf '%s\n' "$artifact"
}

build_native_backend() {
  if [[ "$HOST_TARGET" != "$NATIVE_TARGET" ]]; then
    echo "Native packaging currently supports host target $NATIVE_TARGET only; found $HOST_TARGET" >&2
    exit 1
  fi

  echo "Building native backend artifact for $NATIVE_TARGET..."
  cargo build --manifest-path "$CRATE_DIR/Cargo.toml" --release >/dev/null
}

stage_native_backend() {
  local artifact_dir="$STAGE_DIR/artifacts/$NATIVE_TARGET"
  mkdir -p "$artifact_dir"
  cp "$(find_native_backend)" "$artifact_dir/libblast_stress_solver_ffi.a"
}

find_empp() {
  local candidate

  if [[ -n "${BLAST_STRESS_SOLVER_EMPP:-}" && -x "${BLAST_STRESS_SOLVER_EMPP}" ]]; then
    printf '%s\n' "$BLAST_STRESS_SOLVER_EMPP"
    return
  fi

  if [[ -n "${EMSDK:-}" && -x "${EMSDK}/upstream/emscripten/em++" ]]; then
    printf '%s\n' "${EMSDK}/upstream/emscripten/em++"
    return
  fi

  for candidate in \
    "$HOME/Development/emsdk/upstream/emscripten/em++" \
    "/Users/glavin/Development/emsdk/upstream/emscripten/em++" \
    "/opt/emsdk/upstream/emscripten/em++"
  do
    if [[ -x "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return
    fi
  done

  if candidate="$(command -v em++ 2>/dev/null)"; then
    printf '%s\n' "$candidate"
    return
  fi

  echo "Could not locate em++. Set BLAST_STRESS_SOLVER_EMPP or EMSDK." >&2
  exit 1
}

configure_emscripten_env() {
  local empp="$1"
  local emscripten_root llvm_root emsdk_root node_bin cache_dir config_path

  emscripten_root="$(cd -- "$(dirname -- "$empp")" && pwd)"
  llvm_root="$(cd -- "$emscripten_root/../bin" && pwd)"
  emsdk_root="$(cd -- "$emscripten_root/../.." && pwd)"
  cache_dir="$PROOF_DIR/emscripten-cache"
  config_path="$PROOF_DIR/emscripten-config.py"

  mkdir -p "$cache_dir"

  if compgen -G "$emsdk_root/node/*/bin/node" >/dev/null; then
    node_bin="$(printf '%s\n' "$emsdk_root"/node/*/bin/node | head -n 1)"
  else
    node_bin="$(command -v node)"
  fi

  cat >"$config_path" <<EOF
import os
LLVM_ROOT = r'$llvm_root'
BINARYEN_ROOT = r'$(cd -- "$emscripten_root/.." && pwd)'
EMSCRIPTEN_ROOT = r'$emscripten_root'
NODE_JS = r'$node_bin'
CACHE = r'$cache_dir'
PORTS = os.path.join(CACHE, 'ports')
EOF

  export EM_CONFIG="$config_path"
  export EM_CACHE="$cache_dir"
}

build_wasm_backend() {
  local empp llvm_ar build_dir artifact_dir combined_obj
  local -a cxxflags include_dirs sources object_files runtime_libs

  empp="$(find_empp)"
  configure_emscripten_env "$empp"
  llvm_ar="$(cd -- "$(dirname -- "$empp")/../bin" && pwd)/llvm-ar"

  build_dir="$(mktemp -d "$PROOF_DIR/wasm-backend.XXXXXX")"
  artifact_dir="$STAGE_DIR/artifacts/$WASM_TARGET"
  combined_obj="$build_dir/libblast_stress_solver_ffi.o"

  include_dirs=(
    "$REPO_ROOT/blast/include"
    "$REPO_ROOT/blast/include/globals"
    "$REPO_ROOT/blast/include/lowlevel"
    "$REPO_ROOT/blast/include/extensions/stress"
    "$REPO_ROOT/blast/include/shared/NvFoundation"
    "$REPO_ROOT/blast/source/shared"
    "$REPO_ROOT/blast/source/shared/stress_solver"
    "$REPO_ROOT/blast/source/sdk/common"
    "$REPO_ROOT/blast/source/sdk/lowlevel"
    "$REPO_ROOT/blast/source/shared/NsFoundation/include"
    "$REPO_ROOT/blast/rust_stress_example/ffi"
  )

  sources=(
    "$REPO_ROOT/blast/rust_stress_example/ffi/stress_bridge.cpp"
    "$REPO_ROOT/blast/rust_stress_example/ffi/ext_stress_bridge.cpp"
    "$REPO_ROOT/blast/source/shared/stress_solver/stress.cpp"
    "$REPO_ROOT/blast/source/sdk/common/NvBlastAssert.cpp"
    "$REPO_ROOT/blast/source/sdk/common/NvBlastAtomic.cpp"
    "$REPO_ROOT/blast/source/sdk/common/NvBlastTime.cpp"
    "$REPO_ROOT/blast/source/sdk/common/NvBlastTimers.cpp"
    "$REPO_ROOT/blast/source/sdk/globals/NvBlastGlobals.cpp"
    "$REPO_ROOT/blast/source/sdk/globals/NvBlastInternalProfiler.cpp"
    "$REPO_ROOT/blast/source/sdk/lowlevel/NvBlastAsset.cpp"
    "$REPO_ROOT/blast/source/sdk/lowlevel/NvBlastAssetHelper.cpp"
    "$REPO_ROOT/blast/source/sdk/lowlevel/NvBlastFamily.cpp"
    "$REPO_ROOT/blast/source/sdk/lowlevel/NvBlastFamilyGraph.cpp"
    "$REPO_ROOT/blast/source/sdk/lowlevel/NvBlastActor.cpp"
    "$REPO_ROOT/blast/source/sdk/lowlevel/NvBlastActorSerializationBlock.cpp"
    "$REPO_ROOT/blast/source/sdk/extensions/stress/NvBlastExtStressSolver.cpp"
  )

  cxxflags=(
    -std=c++17
    -O2
    -DNDEBUG=1
    -fno-exceptions
    -fno-rtti
    -DSTRESS_SOLVER_FORCE_SCALAR=1
    -DSTRESS_SOLVER_NO_SIMD=1
    -DSTRESS_SOLVER_NO_DEVICE_QUERY=1
    -D__linux__=1
    -D__aarch64__=1
  )

  for dir in "${include_dirs[@]}"; do
    cxxflags+=("-I$dir")
  done

  object_files=()
  for src in "${sources[@]}"; do
    local obj
    obj="$build_dir/$(basename "${src%.cpp}").o"
    "$empp" "${cxxflags[@]}" -c "$src" -o "$obj"
    object_files+=("$obj")
  done

  local emscripten_sysroot filtered_libcxxabi filtered_libcxxabi_dir
  emscripten_sysroot="$(cd -- "$(dirname -- "$empp")/../emscripten/cache/sysroot/lib/$EMSCRIPTEN_SYSROOT_TARGET" && pwd)"
  filtered_libcxxabi_dir="$build_dir/libcxxabi-filter"
  filtered_libcxxabi="$build_dir/libc++abi-no-cxa-allocate.a"
  mkdir -p "$filtered_libcxxabi_dir"
  (
    cd "$filtered_libcxxabi_dir"
    "$llvm_ar" x "$emscripten_sysroot/libc++abi-noexcept.a"
    # `cxa_noexception.o` defines `__cxa_allocate_exception`, which the Rust
    # crate already provides in `src/wasm_cxa_stubs.rs`.
    rm -f cxa_noexception.o
    "$llvm_ar" rcs "$filtered_libcxxabi" ./*.o
  )
  # Keep only the C++ runtime pieces that the packaged backend must carry.
  # libc, dlmalloc, and standalone-wasm define symbols like `abort`,
  # `malloc`, `fwrite`, and `snprintf` that the Rust crate already
  # provides in `src/wasm_runtime_shims.rs` / `src/wasm_cxa_stubs.rs`.
  # Folding those libs into the archive makes downstream `wasm32-unknown-unknown`
  # consumers fail with duplicate-symbol errors.
  runtime_libs=(
    "$emscripten_sysroot/libc++-noexcept.a"
    "$filtered_libcxxabi"
    "$emscripten_sysroot/libunwind-noexcept.a"
    "$emscripten_sysroot/libcompiler_rt.a"
  )

  echo "Building packaged wasm backend artifact for $WASM_TARGET..."
  "$empp" -r -o "$combined_obj" "${object_files[@]}" "${runtime_libs[@]}"

  mkdir -p "$artifact_dir"
  "$llvm_ar" rcs "$artifact_dir/libblast_stress_solver_ffi.a" "$combined_obj"
}

stage_packaging_overrides() {
  cp "$SUPPORT_DIR/staged-build.rs" "$STAGE_DIR/build.rs"
  cp "$CRATE_DIR/README.md" "$STAGE_DIR/README.md"
  cp "$REPO_ROOT/LICENSE.md" "$STAGE_DIR/LICENSE.md"
  cp "$SUPPORT_DIR/WASM_FEASIBILITY.md" "$STAGE_DIR/WASM_FEASIBILITY.md"
}

copy_packaged_native_sources() {
  local native_root="$STAGE_DIR/native/blast"
  mkdir -p "$native_root" "$native_root/rust_stress_example"
  cp -R "$REPO_ROOT/blast/include" "$native_root/"
  cp -R "$REPO_ROOT/blast/source" "$native_root/"
  cp -R "$REPO_ROOT/blast/rust_stress_example/ffi" "$native_root/rust_stress_example/"
}

package_stage() {
  local package_output package_path package_bytes unpack_dir

  cargo package --allow-dirty --offline --manifest-path "$STAGE_DIR/Cargo.toml" >/dev/null
  package_output="$(cargo package --allow-dirty --offline --manifest-path "$STAGE_DIR/Cargo.toml" --list)"
  printf '%s\n' "$package_output" >"$PROOF_DIR/package-contents.txt"

  package_path="$(find "$STAGE_DIR/target/package" -maxdepth 1 -name 'blast-stress-solver-*.crate' | head -n 1)"
  if [[ -z "$package_path" ]]; then
    echo "Failed to locate packaged .crate file under $STAGE_DIR/target/package" >&2
    exit 1
  fi

  package_bytes="$(wc -c <"$package_path" | tr -d ' ')"
  if (( package_bytes > PACKAGE_SIZE_LIMIT )); then
    echo "Packaged crate is too large: $package_bytes bytes (limit $PACKAGE_SIZE_LIMIT)" >&2
    exit 1
  fi

  unpack_dir="$(mktemp -d "$PROOF_DIR/package-unpack.XXXXXX")"
  LC_ALL=C tar -xf "$package_path" -C "$unpack_dir"
  PACKAGE_ROOT="$(find "$unpack_dir" -mindepth 1 -maxdepth 1 -type d | head -n 1)"

  if [[ -z "${PACKAGE_ROOT:-}" ]]; then
    echo "Failed to unpack package contents from $package_path" >&2
    exit 1
  fi

  export PACKAGE_ROOT
  echo "Packaged crate: $package_path"
  echo "Unpacked package root: $PACKAGE_ROOT"
}

verify_docs_stage() {
  echo "Verifying staged docs build..."
  DOCS_RS=1 cargo doc --offline --no-deps --manifest-path "$PACKAGE_ROOT/Cargo.toml" >/dev/null
}

verify_native_stage() {
  local consumer_dir
  echo "Running downstream native smoke test..."

  consumer_dir="$(mktemp -d "$PROOF_DIR/native-consumer.XXXXXX")"
  cargo new --quiet --bin "$consumer_dir/smoke"

  cat <<EOF > "$consumer_dir/smoke/Cargo.toml"
[package]
name = "smoke"
version = "0.1.0"
edition = "2021"

[dependencies]
blast-stress-solver = { path = "$PACKAGE_ROOT" }
EOF

  cat <<'EOF' > "$consumer_dir/smoke/src/main.rs"
use blast_stress_solver::{BondDesc, ExtStressSolver, NodeDesc, SolverSettings, Vec3};

fn main() {
    let nodes = vec![
        NodeDesc {
            centroid: Vec3::new(0.0, 0.0, 0.0),
            mass: 0.0,
            volume: 1.0,
        },
        NodeDesc {
            centroid: Vec3::new(0.0, 1.0, 0.0),
            mass: 1.0,
            volume: 1.0,
        },
    ];
    let bonds = vec![BondDesc {
        centroid: Vec3::new(0.0, 0.5, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
        area: 1.0,
        node0: 0,
        node1: 1,
    }];

    let mut solver =
        ExtStressSolver::new(&nodes, &bonds, &SolverSettings::default()).expect("solver");
    solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
    solver.update();

    assert_eq!(solver.bond_count(), 1);
}
EOF

  CARGO_TARGET_DIR="$consumer_dir/target-prebuilt" \
    cargo run --offline --quiet --manifest-path "$consumer_dir/smoke/Cargo.toml" >/dev/null

  CARGO_TARGET_DIR="$consumer_dir/target-source" \
    BLAST_STRESS_SOLVER_FORCE_SOURCE_BUILD=1 \
    cargo run --offline --quiet --manifest-path "$consumer_dir/smoke/Cargo.toml" >/dev/null
}

verify_wasm_stage() {
  local consumer_dir wasm_path imports_path
  echo "Running downstream wasm smoke test..."

  consumer_dir="$(mktemp -d "$PROOF_DIR/wasm-consumer.XXXXXX")"
  cargo new --quiet --lib "$consumer_dir/smoke"

  cat <<EOF > "$consumer_dir/smoke/Cargo.toml"
[package]
name = "smoke"
version = "0.1.0"
edition = "2021"

[lib]
crate-type = ["cdylib"]

[dependencies]
blast-stress-solver = { path = "$PACKAGE_ROOT" }
EOF

  cat <<'EOF' > "$consumer_dir/smoke/src/lib.rs"
use blast_stress_solver::{BondDesc, ExtStressSolver, NodeDesc, SolverSettings, Vec3};

#[unsafe(no_mangle)]
pub extern "C" fn downstream_blast_smoke() -> u32 {
    let nodes = [
        NodeDesc {
            centroid: Vec3::new(0.0, 0.0, 0.0),
            mass: 0.0,
            volume: 1.0,
        },
        NodeDesc {
            centroid: Vec3::new(0.0, 1.0, 0.0),
            mass: 10.0,
            volume: 1.0,
        },
    ];
    let bonds = [BondDesc {
        centroid: Vec3::new(0.0, 0.5, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
        area: 1.0,
        node0: 0,
        node1: 1,
    }];

    let mut solver = match ExtStressSolver::new(&nodes, &bonds, &SolverSettings::default()) {
        Some(solver) => solver,
        None => return 0,
    };

    solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
    solver.update();

    solver.actor_count() * 100 + solver.node_count() * 10 + solver.overstressed_bond_count()
}
EOF

  cargo build --offline --target "$WASM_TARGET" --release --manifest-path "$consumer_dir/smoke/Cargo.toml" >/dev/null
  wasm_path="$consumer_dir/smoke/target/$WASM_TARGET/release/smoke.wasm"
  imports_path="$PROOF_DIR/wasm-imports.json"

  node -e '
const fs = require("fs");
const path = process.argv[1];
const out = process.argv[2];
const mod = new WebAssembly.Module(fs.readFileSync(path));
const imports = WebAssembly.Module.imports(mod);
if (imports.length !== 0) {
  console.error(JSON.stringify(imports, null, 2));
  process.exit(1);
}
fs.writeFileSync(out, JSON.stringify({
  imports,
  exports: WebAssembly.Module.exports(mod),
}, null, 2));
const inst = new WebAssembly.Instance(mod, {});
const result = inst.exports.downstream_blast_smoke();
if (result !== 121) {
  console.error(`unexpected smoke result: ${result}`);
  process.exit(1);
}
' "$wasm_path" "$imports_path"
}

verify_demo_consumer_stage() {
  local consumer_dir demo_copy manifest_path log_path

  if [[ ! -d "$DEMO_DIR" ]]; then
    echo "Expected demo crate directory at $DEMO_DIR" >&2
    exit 1
  fi

  echo "Running blast-stress-demo packaged consumer proof..."

  consumer_dir="$(mktemp -d "$PROOF_DIR/demo-consumer.XXXXXX")"
  demo_copy="$consumer_dir/demo"
  manifest_path="$demo_copy/Cargo.toml"
  log_path="$PROOF_DIR/demo-consumer-headless.log"

  mkdir -p "$demo_copy"
  rsync -a --exclude 'target/' --exclude '.codex' "$DEMO_DIR/" "$demo_copy/"

  env LC_ALL=C perl -0pi -e 's{blast-stress-solver = \{ path = "\.\./blast-stress-solver-rs", features = \["scenarios", "rapier"\] \}}{blast-stress-solver = { path = "'"$PACKAGE_ROOT"'", features = ["scenarios", "rapier"] }}' \
    "$manifest_path"

  if ! rg -Fq "$PACKAGE_ROOT" "$manifest_path"; then
    echo "Failed to rewrite blast-stress-demo-rs to the staged package root" >&2
    exit 1
  fi

  CARGO_TARGET_DIR="$consumer_dir/target" \
    cargo test --offline --manifest-path "$manifest_path" \
    --test headless_shot_scripts headless_smoke_scripts_emit_summary_and_fracture -- --nocapture \
    >"$log_path" 2>&1 || {
      cat "$log_path"
      exit 1
    }

  echo "blast-stress-demo proof log: $log_path"
}

copy_current_crate_files
inject_publish_metadata
build_native_backend
stage_native_backend
build_wasm_backend
stage_packaging_overrides
copy_packaged_native_sources
package_stage
verify_docs_stage

echo "Assembled staged crate at: $STAGE_DIR"

if [[ $VERIFY_NATIVE -eq 1 ]]; then
  verify_native_stage
fi

if [[ $VERIFY_WASM -eq 1 ]]; then
  verify_wasm_stage
fi

if [[ $VERIFY_DEMO_CONSUMER -eq 1 ]]; then
  verify_demo_consumer_stage
fi

if [[ $KEEP_STAGE -eq 1 ]]; then
  echo "Kept staged crate at: $STAGE_DIR"
fi
