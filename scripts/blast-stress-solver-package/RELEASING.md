# Releasing `blast-stress-solver`

The release flow is tag-driven and publishes from a staged crate, not directly
from `blast/blast-stress-solver-rs`.

Do not run `cargo publish --manifest-path blast/blast-stress-solver-rs/Cargo.toml`
directly. That source crate is marked `publish = false` on purpose so local
publishes go through the staged package path instead of accidentally shipping
the monorepo build layout.

## Prerequisites

- GitHub Actions secret: `CARGO_REGISTRY_TOKEN`
- `blast/blast-stress-solver-rs/Cargo.toml` version updated to the release
  version

## Tag format

Push a tag in this form:

```bash
git tag blast-stress-solver-v<version>
git push origin blast-stress-solver-v<version>
```

The workflow validates that the tag version matches the crate version.

## What the workflow does

1. Builds the staged publish crate on `macos-14`.
2. Builds and packages:
   - `aarch64-apple-darwin`
   - `wasm32-unknown-unknown`
   - bundled native C++ sources for the Apple/Linux source-build fallback
3. Verifies:
   - docs build
   - downstream native consumer using the packaged archive
   - downstream native consumer with `BLAST_STRESS_SOLVER_FORCE_SOURCE_BUILD=1`
   - downstream wasm consumer with one final wasm and no imports
   - `blast/blast-stress-demo-rs` consuming the staged package in the headless
     Rapier fracture test
4. Runs `cargo publish --dry-run`.
5. Publishes to crates.io.
6. Creates a GitHub Release and uploads:
   - the `.crate`
   - the `.sha256`
   - package contents proof
   - wasm imports proof
   - demo consumer proof log

## Local End-to-End Proof

To prove the publish-style package against a real downstream app before tagging,
run:

```bash
scripts/assemble-blast-stress-solver-package.sh --verify-demo-consumer
```

That one command verifies:

- docs build
- packaged native consumer smoke
- bundled-source native fallback smoke
- packaged wasm consumer smoke
- `blast/blast-stress-demo-rs` running the real headless Rapier fracture test
  against the staged package instead of the monorepo path dependency

## Local Publish

To publish locally through the same staged package flow, run:

```bash
scripts/publish-blast-stress-solver.sh
```

For a local crates.io preflight without publishing:

```bash
scripts/publish-blast-stress-solver.sh --dry-run
```
