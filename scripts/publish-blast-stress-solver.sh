#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ASSEMBLE_SCRIPT="$SCRIPT_DIR/assemble-blast-stress-solver-package.sh"

STAGE_DIR=""
KEEP_STAGE=0
PUBLISH_ARGS=()

usage() {
  cat <<'EOF'
Usage: scripts/publish-blast-stress-solver.sh [options]

Build the staged publish crate, run the downstream consumer proofs, then
publish that staged crate to crates.io.

Options:
  --dry-run       Run `cargo publish --dry-run` instead of a real publish.
  --keep-stage    Keep the staged crate directory on exit.
  --stage-dir     Use a specific stage directory instead of mktemp.
  -h, --help      Show this help text.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      PUBLISH_ARGS+=(--dry-run)
      shift
      ;;
    --keep-stage)
      KEEP_STAGE=1
      shift
      ;;
    --stage-dir)
      STAGE_DIR="$2"
      shift 2
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

cleanup() {
  if [[ $KEEP_STAGE -eq 0 && -n "${STAGE_DIR:-}" && -d "$STAGE_DIR" ]]; then
    rm -rf "$STAGE_DIR"
  fi
}

if [[ -z "$STAGE_DIR" ]]; then
  STAGE_DIR="$(mktemp -d "${TMPDIR:-/tmp}/blast-stress-solver-publish.XXXXXX")"
else
  rm -rf "$STAGE_DIR"
  mkdir -p "$STAGE_DIR"
fi

trap cleanup EXIT

"$ASSEMBLE_SCRIPT" \
  --stage-dir "$STAGE_DIR" \
  --verify-demo-consumer \
  --keep-stage

cargo publish "${PUBLISH_ARGS[@]}" --manifest-path "$STAGE_DIR/Cargo.toml"

if [[ $KEEP_STAGE -eq 1 ]]; then
  echo "Kept staged crate at: $STAGE_DIR"
fi
