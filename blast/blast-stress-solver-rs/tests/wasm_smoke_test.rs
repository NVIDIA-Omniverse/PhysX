//! Regression tests for the `wasm32-unknown-unknown` build.
//!
//! **Why these exist**: downstream consumers of this crate repeatedly
//! hit two stacked WebAssembly build bugs that the main test suite
//! silently missed:
//!
//! 1. **Bug A — unresolved `env.*` imports.** The C++ backend drags
//!    in dozens of libc symbols via libc++. When downstream builds a
//!    cdylib for `wasm32-unknown-unknown`, those symbols have to be
//!    resolved somewhere. Historically the fix was to link
//!    `wasi-libc`, which resolves them but see Bug B.
//!
//! 2. **Bug B — `command_export` runtime trap.** Linking wasi-libc
//!    pulls in `wasi_snapshot_preview1` imports, which causes
//!    `wasm-bindgen` to treat the module as a *command* (like a main
//!    executable) rather than a *library*. It then wraps every export
//!    in a shim that calls `__wasm_call_dtors → __funcs_on_exit →
//!    __stdio_exit`, walking an uninitialised atexit table — so every
//!    call into the module traps at runtime.
//!
//! The upstream fix is to provide pure-Rust stubs for every libc
//! symbol the C++ backend references (see `src/wasm_runtime_shims.rs`)
//! and **not** link wasi-libc. The final cdylib then has zero `env.*`
//! imports and zero `wasi_snapshot_preview1.*` imports, and
//! wasm-bindgen emits a normal library module.
//!
//! **What these tests do**: Test 1 (`wasm_fixture_has_no_libc_imports`)
//! builds a minimal fixture cdylib under `tests/wasm_smoke/fixture/`,
//! then parses the resulting `.wasm` file and asserts that no import
//! comes from `env` or `wasi_snapshot_preview1`. Test 2
//! (`wasm_fixture_instantiates_and_runs`) additionally runs the fixture
//! through `wasm-pack` + Node to catch any future `command_export`
//! trap. Test 2 is skipped (with a diagnostic message) if wasm-pack
//! or node isn't available; Test 1 requires only `cargo` with the
//! `wasm32-unknown-unknown` target installed plus a wasi C++ toolchain
//! (`libc++-*-dev-wasm32` on Debian/Ubuntu).
//!
//! **Running them**:
//!
//! ```text
//! rustup target add wasm32-unknown-unknown
//! sudo apt-get install -y libc++-18-dev-wasm32   # or equivalent
//! cargo test -p blast-stress-solver \
//!     --features wasm-smoke --test wasm_smoke_test
//! ```

#![cfg(feature = "wasm-smoke")]

use std::path::PathBuf;
use std::process::Command;

/// Imports from `env` that we consider acceptable. **Zero tolerance
/// by default** — every libc symbol should be stubbed in
/// `src/wasm_runtime_shims.rs`. If a genuine wasm-bindgen glue import
/// ever shows up here it belongs on this list, not in the shims.
const ALLOWED_ENV_IMPORTS: &[&str] = &[];

/// Imports from `wasi_snapshot_preview1` that we consider acceptable.
/// **Zero tolerance, no exceptions.** A single wasi import flips
/// wasm-bindgen into command-export mode and causes Bug B.
const ALLOWED_WASI_IMPORTS: &[&str] = &[];

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn fixture_dir() -> PathBuf {
    manifest_dir().join("tests/wasm_smoke/fixture")
}

fn cargo_bin() -> String {
    std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string())
}

/// Ensure the `wasm32-unknown-unknown` target is installed. If it
/// isn't, `cargo build` below would fail with a cryptic "can't find
/// crate for `core`" error; this check surfaces a clearer message.
fn require_wasm_target() {
    let out = Command::new("rustup")
        .args(["target", "list", "--installed"])
        .output();
    if let Ok(out) = out {
        let installed = String::from_utf8_lossy(&out.stdout);
        if !installed.lines().any(|t| t.trim() == "wasm32-unknown-unknown") {
            panic!(
                "the `wasm32-unknown-unknown` target is not installed. \
                 Run `rustup target add wasm32-unknown-unknown` and retry."
            );
        }
    }
    // If `rustup` isn't on PATH we can't check — fall through and
    // let `cargo build` produce its own error.
}

/// Build the fixture cdylib for `wasm32-unknown-unknown` and return
/// the path to the produced `.wasm`.
///
/// We deliberately use an isolated `CARGO_TARGET_DIR` so the fixture
/// build doesn't contend with the outer test run's target dir and
/// nested cargo invocations never see each other's lockfiles.
fn build_fixture(release: bool) -> PathBuf {
    require_wasm_target();

    let fixture = fixture_dir();
    let manifest_path = fixture.join("Cargo.toml");
    let target_dir = fixture.join("target");

    let mut cmd = Command::new(cargo_bin());
    cmd.args([
        "build",
        "--target",
        "wasm32-unknown-unknown",
        "--manifest-path",
    ])
    .arg(&manifest_path)
    .env("CARGO_TARGET_DIR", &target_dir);
    if release {
        cmd.arg("--release");
    }

    // Strip any `RUSTFLAGS` the outer test run might have set — they
    // can inject native-target flags that break the wasm build.
    cmd.env_remove("RUSTFLAGS");
    cmd.env_remove("CARGO_ENCODED_RUSTFLAGS");

    let status = cmd
        .status()
        .expect("failed to spawn cargo to build the wasm smoke fixture");
    assert!(
        status.success(),
        "failed to build the wasm smoke fixture cdylib — see cargo output above"
    );

    let profile = if release { "release" } else { "debug" };
    let wasm = target_dir
        .join("wasm32-unknown-unknown")
        .join(profile)
        .join("blast_wasm_smoke_fixture.wasm");
    assert!(
        wasm.exists(),
        "fixture built but produced no .wasm at {}",
        wasm.display()
    );
    wasm
}

/// Collect every `(module, field)` import in the given wasm module.
fn collect_imports(wasm_bytes: &[u8]) -> Vec<(String, String)> {
    let mut imports = Vec::new();
    for payload in wasmparser::Parser::new(0).parse_all(wasm_bytes) {
        let payload = payload.expect("parse wasm");
        if let wasmparser::Payload::ImportSection(reader) = payload {
            for imp in reader {
                let imp = imp.expect("parse import");
                imports.push((imp.module.to_string(), imp.name.to_string()));
            }
        }
    }
    imports
}

/// **Test 1 — Bug A guard.** Build the fixture and assert that no
/// import comes from `env` or `wasi_snapshot_preview1`. If this fails
/// the failure message tells you exactly which shim to add.
#[test]
fn wasm_fixture_has_no_libc_imports() {
    let wasm_path = build_fixture(/*release=*/ true);
    let bytes = std::fs::read(&wasm_path)
        .unwrap_or_else(|e| panic!("read {}: {e}", wasm_path.display()));
    let imports = collect_imports(&bytes);

    let mut offenders = Vec::new();
    for (module, name) in &imports {
        match module.as_str() {
            "env" if !ALLOWED_ENV_IMPORTS.contains(&name.as_str()) => {
                offenders.push(format!("env.{name}"));
            }
            "wasi_snapshot_preview1" if !ALLOWED_WASI_IMPORTS.contains(&name.as_str()) => {
                offenders.push(format!("wasi_snapshot_preview1.{name}"));
            }
            _ => {}
        }
    }

    assert!(
        offenders.is_empty(),
        "fixture cdylib has {} disallowed wasm import(s). Add a pure-Rust \
         shim to `src/wasm_runtime_shims.rs` (or `src/wasm_cxa_stubs.rs` \
         for C++ ABI symbols) for each of the following, or update the \
         ALLOWED_* lists in tests/wasm_smoke_test.rs if the symbol is \
         a legitimate wasm-bindgen glue import:\n  {}\n\n\
         Diagnostic: run\n    \
         wasm-objdump -j Import -x {}",
        offenders.len(),
        offenders.join("\n  "),
        wasm_path.display(),
    );
}

/// Return true if `bin` exists on `PATH`.
fn has_tool(bin: &str) -> bool {
    Command::new(bin)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// **Test 2 — Bug B guard.** Build the fixture with `wasm-pack` (which
/// invokes `wasm-bindgen` under the hood), then call the exported
/// function many times from Node. A `command_export` trap fires on
/// every call, so 100 iterations is plenty to catch it — but a single
/// call is also enough in practice.
///
/// Skipped (with a `println!` diagnostic) if `wasm-pack` or `node`
/// isn't on PATH. Test 1 already covers the root cause; this one is
/// belt-and-suspenders for future wasm-bindgen regressions.
#[test]
fn wasm_fixture_instantiates_and_runs() {
    if !has_tool("wasm-pack") {
        eprintln!(
            "SKIP wasm_fixture_instantiates_and_runs: `wasm-pack` not found on PATH. \
             Install it with `cargo install wasm-pack` to run this test."
        );
        return;
    }
    if !has_tool("node") {
        eprintln!(
            "SKIP wasm_fixture_instantiates_and_runs: `node` not found on PATH. \
             Install Node.js 20+ to run this test."
        );
        return;
    }
    require_wasm_target();

    // Build with wasm-pack targeting nodejs. Use an isolated out-dir
    // inside the fixture's own target/ so nested runs don't collide.
    let fixture = fixture_dir();
    let out_dir = fixture.join("target").join("wasm-pack-nodejs");
    // Best effort cleanup of stale artifacts — ignore failures.
    let _ = std::fs::remove_dir_all(&out_dir);

    let status = Command::new("wasm-pack")
        .args(["build", "--target", "nodejs", "--dev", "--out-dir"])
        .arg(&out_dir)
        .current_dir(&fixture)
        .env_remove("RUSTFLAGS")
        .env_remove("CARGO_ENCODED_RUSTFLAGS")
        .status()
        .expect("spawn wasm-pack");
    assert!(status.success(), "wasm-pack build failed");

    // wasm-pack writes `<crate_name>.js` as the nodejs entry point.
    // The fixture crate is named `blast-wasm-smoke-fixture`, which
    // wasm-pack mangles to `blast_wasm_smoke_fixture`.
    let entry = out_dir.join("blast_wasm_smoke_fixture.js");
    assert!(
        entry.exists(),
        "wasm-pack produced no nodejs entry at {}",
        entry.display()
    );

    // Drive the `blast_wasm_smoke` export from Node. The fixture
    // annotates it with `#[wasm_bindgen]` so wasm-pack's generated
    // `.js` wrapper re-exports it as a top-level function. Calling
    // *any* wasm-bindgen wrapper is what trips Bug B: wasm-bindgen
    // wraps each export with a `__wasm_call_dtors` post-call, which
    // walks an uninitialised atexit table as soon as wasi imports
    // are present.
    let entry_literal = entry.to_string_lossy().replace('\\', "\\\\");
    let driver = format!(
        "const pkg = require('{entry_literal}');\n\
         if (typeof pkg.blast_wasm_smoke !== 'function') {{\n\
             console.error('no blast_wasm_smoke export; pkg keys:', Object.keys(pkg));\n\
             process.exit(2);\n\
         }}\n\
         // First call — most likely to hit any startup trap.\n\
         const first = pkg.blast_wasm_smoke();\n\
         if (!(first >= 0)) {{ console.error('first call returned', first); process.exit(3); }}\n\
         // Hammer it. Bug B fires on every call once wasi imports\n\
         // are present, so a modest loop is enough.\n\
         for (let i = 0; i < 100; i++) {{ pkg.blast_wasm_smoke(); }}\n\
         console.log('ok', first);\n"
    );

    let out = Command::new("node")
        .args(["-e", &driver])
        .output()
        .expect("spawn node");
    assert!(
        out.status.success(),
        "node driver failed (exit {:?})\n--- stdout ---\n{}\n--- stderr ---\n{}",
        out.status.code(),
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr),
    );
}

