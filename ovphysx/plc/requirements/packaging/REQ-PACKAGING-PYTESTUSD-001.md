<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-PYTESTUSD-001
title: The python-test run resolves USD from stock pip usd-core, the only supported source
status: implemented
owner: ovphysx
---

## Description

ovphysx links no USD and hands ovstage a file path, which ovstage parses natively
with its own py-less namespaced monolith, so almost nothing in the main pytest
session imports `pxr`. Its one exception, `test_documentation_contracts.py`, opens
its stage in a **clean child interpreter** where no ovstage monolith is resident, so
a stock USD build is safe there. The one suite that imports `pxr` in-process,
`utils_tests/` for the pure-`pxr` `ovphysx.utils` subpackage, runs in its **own
pytest process** that loads no ovstage (`scripts/test_python_runtime.cmake` passes
`--ignore=utils_tests` to the main session and runs the directory separately), so
stock `usd-core` is the only USD resident there too.

This requirement makes stock `usd-core` from PyPI the **only** supported source of
python USD for the tests (OMPE-108818). No internal USD monolith is fetched or
swapped, and there is no env or CMake switch that selects a different USD. ovphysx
also does not test OmniClient or remote-USD loading: resolving a remote asset needs
a py-enabled NVIDIA USD resident in-process (which stock `usd-core` cannot be) and
exercises the application's USD stack, not ovphysx's — the application owns USD and
its resolver. See [ADR-0029](../../../ovruntime/plc/adr/ADR-0029-python-test-usd-stock-only.md).

Two different USD builds cannot coexist in one process, so the
one-USD-runtime-per-process rule (REQ-PACKAGING-USDFREE-001) reduces here to a
single invariant: no test imports `pxr` in-process while ovstage is resident.
`usd-core` publishes no linux-aarch64 wheel, so on that platform the `pxr`-dependent
doc-contract checks skip and the `ovphysx.utils` suite is not launched; everywhere
else a missing `pxr` is a failure, not a skip.

## Acceptance Criteria

- AC-1: **Python USD is stock pip `usd-core`.** `tests/python_tests/pyproject.toml`
  declares `usd-core` gated `platform_machine != 'aarch64'`, and
  `tests/python_tests/uv.lock` records it as a leaf package carrying that same
  marker on its dependency edge. A `scripts/test_python_runtime.cmake` run installs
  it into the uv venv, so `from pxr import Usd` in the venv interpreter resolves to
  `usd-core`.
- AC-2: **No internal USD monolith is fetched or swapped, ever.**
  `scripts/fetch_deps.cmake` pulls no py-USD package and creates no
  `_build/target-deps/usd-py-tests/*` tree. `scripts/test_python_runtime.cmake` runs
  the test body directly against the venv: no overlay swap, no `OVPHYSX_PYUSD_DIR`,
  and no monolith `lib/python` on `PYTHONPATH` nor monolith `lib` on `PATH` /
  `LD_LIBRARY_PATH`. There is no env or CMake switch that changes this.
- AC-3: **One USD image per process.** No module imports `pxr` in-process while
  ovstage is resident; a module that does is a violation. In the ovstage-resident
  suites only `test_documentation_contracts.py` names `pxr`, and only inside source
  strings executed in a clean child interpreter (`_stage_info_keys`), never
  in-process. The `utils_tests/` suite (`ovphysx.utils`, pure `pxr`, no PhysX/ovstage)
  is exempt from the scan: it runs in its own pytest process that never loads ovstage,
  so `pxr` there is the only USD in the process.
- AC-4: **aarch64 skips the pxr-dependent checks rather than failing.** `usd-core`
  publishes no linux-aarch64 PyPI wheel, so no python USD is installed there; the
  `pxr`-dependent doc-contract tests
  (`test_physics_scene_usda_template_authors_the_mass_unit`,
  `test_reference_scene_authors_the_mass_unit`) skip on aarch64 — `_REQUIRES_PXR`
  gates purely on `platform.machine()`, matching the pyproject `platform_machine !=
  'aarch64'` marker. `scripts/test_python_runtime.cmake` does not launch the
  separate `utils_tests/` process on aarch64 at all (same `ARCH_NAME` gate as the
  marker): with no `pxr` every module there would `importorskip`, pytest would
  collect nothing and exit non-zero, and the suite's own
  `OVPHYSX_REQUIRE_CODELESS_SCHEMAS=1` guard would rightly refuse to run without a
  USD. Everywhere else a missing `pxr` is a failure, not a skip, so a broken venv or
  a slow-machine timeout cannot silently drop this enforcement on the platforms that
  ship a wheel.

## Known limitations

- On linux-aarch64 there is no python USD (PyPI ships no `usd-core` wheel), so the
  two mass-unit doc-contract checks skip there and the whole `utils_tests/`
  (`ovphysx.utils`, pure `pxr`) suite is not launched (AC-4); `ovphysx.utils` has no
  aarch64 test coverage. This gap was accepted rather than tying the suite to an
  internal package.
- ovphysx runs no remote-USD / OmniUsdResolver / OmniClient test. That coverage
  needed a py-enabled NVIDIA USD resident in-process, which stock `usd-core` cannot
  be, and it exercised the application's USD stack rather than ovphysx's. ADR-0029
  records the decision and the preferred way to reintroduce resolver-state coverage
  if it is ever wanted — a native ovstage query, never a second in-process USD.

## Test References

- [TEST-PACKAGING-PYTESTUSD-001](../../tests/packaging/TEST-PACKAGING-PYTESTUSD-001.md)
  — AC-1 (declaration half) and AC-3 automated by a pytest over the project metadata
  and the suite's `pxr`-import inventory (`test_pytest_usd_source.py`); the runtime
  behavior (AC-1 runtime, AC-2 no-monolith, AC-4 skip) verified by the
  `python-runtime` CTest.

## Code References

- ovphysx/tests/python_tests/pyproject.toml (`usd-core` dependency and its aarch64-exclusion marker — AC-1, AC-4)
- ovphysx/tests/python_tests/uv.lock (`usd-core` leaf package and its marked dependency edge — AC-1)
- ovphysx/scripts/fetch_deps.cmake (pulls no py-USD monolith package — AC-2)
- ovphysx/scripts/test_python_runtime.cmake (runs the suite against the venv's stock usd-core: no overlay, no OVPHYSX_PYUSD_DIR, no monolith on PYTHONPATH/PATH/LD_LIBRARY_PATH — AC-2, AC-3)
- ovphysx/tests/python_tests/test_pytest_usd_source.py (declares usd-core and pins the one-USD-per-process `pxr` allow-list — AC-1, AC-3)
- ovphysx/tests/python_tests/test_documentation_contracts.py (`_stage_info_keys` clean-subprocess read; `_REQUIRES_PXR` platform-gated skip — AC-3, AC-4)
- ovphysx/tests/python_tests/utils_tests/conftest.py (the separate-process pure-`pxr` suite; its `pytest_configure` refuses a `pxr`-less strict run — AC-3, AC-4)

## Dependencies

- REQ-PACKAGING-USDFREE-001 — the one-USD-per-process rule and the USD-free product payload this test policy sits alongside; stock `usd-core` never enters the shipped SDK or wheel.
- REQ-PACKAGING-TESTDEPS-001 — the uv-resolution mechanics (relative find-links, accepted lock) the `usd-core` dependency must not disturb.
- ADR-0029 — the decision to make stock `usd-core` the only python-test USD and to drop remote-USD/OmniClient testing from ovphysx.
