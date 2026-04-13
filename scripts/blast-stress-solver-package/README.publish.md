# blast-stress-solver

Staged publish package for the Rust Blast stress solver.

This package is assembled from the PhysX monorepo by
`scripts/assemble-blast-stress-solver-package.sh`. It exists so the published
crate can stay self-contained without reorganizing the monorepo source tree.

## Supported packaged targets

- `aarch64-apple-darwin`
- `wasm32-unknown-unknown`

For those targets, consumers should be able to add `blast-stress-solver` as a
normal Cargo dependency and build without a separate Blast checkout, C++ tool
chain install, or Emscripten install.

## What is packaged

- The Rust wrapper crate source.
- A prebuilt backend archive under `artifacts/<target>/libblast_stress_solver_ffi.a`.
- A package-local `build.rs` that links packaged artifacts instead of reaching
  into sibling monorepo paths.
- Internal wasm runtime shims so downstream browser-oriented
  `wasm32-unknown-unknown` applications still emit a single final wasm file.

## Maintainer notes

- This staged crate is the release artifact. The monorepo crate source remains
  source-building for local development.
- Unsupported targets fail at build time with a message listing the packaged
  targets present in `artifacts/`.
