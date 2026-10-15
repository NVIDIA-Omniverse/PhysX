<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-TESTDEPS-001
title: Python Test Harness Resolves ovstage From An Index, Verified By The Lock
status: implemented
owner: ovphysx
---

## Description

Python tests and benchmarks install OVStage from a named index with wheel hashes
in their uv locks. CMake drivers supply the index configuration explicitly when
`UV_NO_CONFIG=1` is set. Unlocked sample, wheel, and no-GPU installs use only the
OVStage package page alongside PyPI, because `uv pip` does not honor project
source mappings.

## Acceptance Criteria

- AC-1: **Every ovstage wheel in every checked-in lock carries a sha256.** A lock that
  records ovstage from a local path registry, or that lists a wheel without a valid SHA-256, is
  rejected by `scripts/validate_python_test_uv_lock.py`, which `ci_validate.cmake` runs.

  The guarantee is scoped to lock-driven runs. `uv pip install` of a built wheel has no
  lock, so it resolves ovstage from the index on the index's own integrity -- the same
  position an end user is in.

- AC-2: **Resolution works with `UV_NO_CONFIG=1`.** Locked projects receive both
  PyPI and the configured OVStage index. Unlocked installs receive PyPI and only
  the OVStage package page, so unrelated internal packages cannot shadow PyPI.

- AC-3: **The ovstage index has exactly one source of truth per project.** The URL lives in
  each project's `pyproject.toml` as a `[[tool.uv.index]]` named `ovstage`; the drivers read
  it from there via `scripts/ovstage_index.cmake` rather than repeating it.

- AC-4: **Project metadata marks the ovstage index `explicit`.** Only packages
  assigned through `[tool.uv.sources]` resolve there when generating project locks.
  Unlocked installs enforce the same boundary through AC-2's package-only source.

- AC-5: **One bump reaches every copy of the pin, and no hand-written list is the last
  word.** `scripts/sync_ovstage_version.py --version` stamps `python/pyproject.toml` and
  regenerates every uv lock that records ovstage, discovering that set by scanning the tree.
  Locks are regenerated with `uv lock`, never text-stamped: their ovstage entry now carries
  URLs and per-wheel hashes, which a textual version rewrite would leave pointing at the
  previous build's bytes.

- AC-6: **A channel switch moves both halves together.** `--source` rewrites the native
  archive URL and the `ovstage` index in every project, then regenerates the locks, so an
  internal artifact is never paired with a public one.

## Test References

- TEST-PACKAGING-TESTDEPS-001

## Code References

- `ovphysx/scripts/validate_python_test_uv_lock.py` — lock shape and hash enforcement,
  and the tree scan that finds every lock recording ovstage (AC-1, AC-5)
- `ovphysx/scripts/ovstage_index.cmake` — reads the index from the project being
  configured and emits the appropriate uv source variables (AC-2, AC-3)
- `ovphysx/scripts/test_python_runtime.cmake`, `test_benchmarks_python.cmake`,
  `test_python_samples.cmake`, `test_python_wheel.cmake`,
  `ci/ovphysx-nogpu/run_python_smoke.cmake` — the resolving entrypoints (AC-2)
- `ovphysx/scripts/run_uv_pytest.py` — the developer wrapper. It passes `--locked`, so it
  consumes the hash-verified lock rather than resolving, and deliberately leaves a
  developer's own `uv` configuration in effect (AC-1)
- `ovphysx/tests/python_tests/pyproject.toml`, `tests/python_benchmarks/pyproject.toml`,
  `python/pyproject.toml` — the `[[tool.uv.index]]` and `[tool.uv.sources]` pins
  (AC-3, AC-4)
- `ovphysx/scripts/sync_ovstage_version.py` — pin propagation and relocking (AC-5, AC-6)
- `ovphysx/tools/internal/relay_ci_wheel_to_ngc.py` — selects the pinned wheel from
  the source commit's lock and checks its SHA-256 before NGC relay (AC-1)

## Dependencies

- ADR-0033 (`ovphysx/ovruntime/plc/adr/ADR-0033-ovstage-native-dependency-delivery.md`)
- REQ-PACKAGING-OVSTAGE-001 — the native archive half of the same pin
