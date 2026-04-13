# Releasing `blast-stress-solver`

The release flow is tag-driven and publishes from a staged crate, not directly
from `blast/blast-stress-solver-rs`.

## Prerequisites

- GitHub Actions secret: `CARGO_REGISTRY_TOKEN`
- `blast/blast-stress-solver-rs/Cargo.toml` version updated to the release
  version

## Tag format

Push a tag in this form:

```bash
git tag blast-stress-solver-v0.1.0
git push origin blast-stress-solver-v0.1.0
```

The workflow validates that the tag version matches the crate version.

## What the workflow does

1. Builds the staged publish crate on `macos-14`.
2. Builds and packages:
   - `aarch64-apple-darwin`
   - `wasm32-unknown-unknown`
3. Verifies:
   - docs build
   - downstream native consumer
   - downstream wasm consumer with one final wasm and no imports
4. Runs `cargo publish --dry-run`.
5. Publishes to crates.io.
6. Creates a GitHub Release and uploads:
   - the `.crate`
   - the `.sha256`
   - package contents proof
   - wasm imports proof
