<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-PYTESTUSD-001
maps_to: REQ-PACKAGING-PYTESTUSD-001
type: integration
---

## Coverage status

**Partly automated.**

- **Automated (offline, no build needed):** AC-1's declaration half and AC-3 —
  `tests/python_tests/test_pytest_usd_source.py` reads the project metadata and the
  suite's `pxr`-import inventory. It runs in the `python-runtime` pytest session and
  also passes standalone (it imports neither ovphysx nor `pxr`).
- **Behavioral (CTest `python-runtime`, runs in CI):** the runtime half of AC-1,
  AC-2, and AC-4's skip — the single `python-runtime` CTest entry exercises them by
  construction: a run that fetched or swapped a monolith, or whose `from pxr import`
  resolved to one instead of the venv `usd-core`, would not reach a passing test.
  Asserted by the run completing, plus the one-line `grep` pass below, rather than by
  a dedicated assertion.

There is no opt-in/second pass: stock `usd-core` is the only supported USD, so a
single configuration covers the requirement.

## Scenario

The python-test run sources python USD from stock pip `usd-core` and fetches/swaps
no internal monolith, and no module imports `pxr` in-process next to ovstage's
resident USD (REQ AC-1, AC-2, AC-3). Where PyPI ships no `usd-core` (linux-aarch64)
the `pxr`-dependent checks skip rather than fail (AC-4).

## Given

- The `ovphysx` python-tests project at `tests/python_tests/` with its
  `pyproject.toml` and checked-in `uv.lock`.
- `tests/python_tests/test_pytest_usd_source.py`, and the suite's fixed inventory of
  modules that may name `pxr` (`_PXR_ALLOWED` = `{test_documentation_contracts.py}`),
  plus `_ISOLATED_PXR_DIRS` (`utils_tests/`) — a dedicated ovstage-free process that is
  exempt from the scan.
- For the behavioral half: a built/installed SDK and a staged ovstage wheel.

## When

- The guard test reads `pyproject.toml` / `uv.lock` and walks every `*.py` under
  `tests/python_tests/` (excluding `.venv`, `__pycache__`, `*.egg-info`, the
  `_ISOLATED_PXR_DIRS` (`utils_tests/`), and itself, keyed by path relative to that
  directory so a nested file is not exempted by name), stripping comments, matching
  literal `from pxr` / `import pxr` plus the dynamic forms
  `importlib.import_module("pxr")` and `__import__("pxr")`.
- `scripts/test_python_runtime.cmake` runs the pytest session.

## Then

- `pyproject.toml` declares `usd-core; platform_machine != 'aarch64'`, and `uv.lock`
  records a `usd-core` leaf package whose dependency edge carries the same marker
  (REQ AC-1, `test_pyproject_declares_stock_usd_core`,
  `test_uv_lock_records_marked_usd_core`).
- The set of modules naming `pxr` in the scanned (ovstage-resident) suites is a subset
  of `_PXR_ALLOWED` (`test_documentation_contracts.py`, which names it only inside
  child-interpreter source strings); the isolated `utils_tests/` suite is scanned-exempt
  (its own ovstage-free process). Any other module naming `pxr` fails the check
  (REQ AC-3, `test_no_unexpected_in_process_pxr_users`).
- Every directory in `_ISOLATED_PXR_DIRS` is passed as `--ignore=<dir>` to the main
  pytest session in `scripts/test_python_runtime.cmake`, so the exempted `pxr`
  imports never land in the process that loads ovstage (REQ AC-3,
  `test_isolated_pxr_dirs_are_excluded_from_the_main_session`).
- `fetch_deps.cmake` pulls no `usd-py-tests` package and creates no
  `_build/target-deps/usd-py-tests/*`; the pytest run completes with `from pxr
  import` resolving to the venv `usd-core`, no overlay swap, and no monolith path on
  `PYTHONPATH`/`PATH`/`LD_LIBRARY_PATH` (REQ AC-1 runtime, AC-2).
- On linux-aarch64 (no `usd-core` wheel) the two mass-unit doc-contract tests report
  `skipped`, not `failed`, and the run logs that the `ovphysx.utils` pytest was not
  launched instead of failing on an empty collection (REQ AC-4).

```bash
# from ovphysx/ -- the offline guard (AC-1 declaration, AC-3)
_build/target-deps/python/bin/python3 -m pytest tests/python_tests/test_pytest_usd_source.py -v

# AC-2 by hand after fetch_deps: no monolith tree was pulled
test ! -e _build/target-deps/usd-py-tests && echo "AC-2: no monolith"
```
