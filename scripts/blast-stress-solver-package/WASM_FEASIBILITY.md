# WASM Packaging Notes

This staged package supports `wasm32-unknown-unknown` as a first-class Cargo
target.

## Consumer contract

- Downstream Rust applications depend on `blast-stress-solver` as a normal
  Cargo dependency.
- The downstream application emits a single final wasm file.
- The package does not require a JS loader module or a second sidecar wasm file
  from Blast.

## How the staged package achieves that

- The maintainer packaging flow builds the full Blast backend with Emscripten.
- The required C++ runtime pieces are consolidated into the packaged backend
  archive `artifacts/wasm32-unknown-unknown/libblast_stress_solver_ffi.a`.
- The Rust crate provides internal wasm-only runtime shims for the small WASI
  surface that survives the backend link.

## Maintainer prerequisites

- Emscripten must be available via `BLAST_STRESS_SOLVER_EMPP`, `EMSDK`, or one
  of the standard install paths checked by the packaging script.
- The release flow builds and packages wasm artifacts; consumers do not.

## Verification expectation

`scripts/assemble-blast-stress-solver-package.sh` verifies that a clean
downstream Rust crate can:

1. depend on the staged package by path,
2. build for `wasm32-unknown-unknown`,
3. produce one final wasm file, and
4. instantiate and run a smoke-test export with no imports required from Blast.
