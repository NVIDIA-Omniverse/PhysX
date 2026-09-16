<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-RPATH-001
maps_to: REQ-PACKAGING-RPATH-001
type: integration
---

## Coverage status

Partly automated.

- **Automated:** AC-2 — the existing `cpp-samples` CTest suite, driven by
  `scripts/test_cpp_samples.cmake` (CTest label `cpp-samples`, run via
  `cmake -P scripts/validate_all.cmake` or `ctest -L cpp-samples`). Every
  sample under `tests/c_samples/` builds solely via `find_package(ovphysx)` +
  `target_link_libraries(<sample> ovphysx::ovphysx)` and is run with no
  runtime-path override on Linux (`_ENV_OVERRIDE` is the empty
  `LD_LIBRARY_PATH=`) — a pass is direct evidence the installed target
  resolves its own transitive runtime dependencies unaided. No new test code
  is needed: this suite already covers AC-2 by construction now that the
  underlying fix is in place, and it is what caught the regression this REQ
  documents (it failed before the fix, in the same run that surfaced ovstage
  0.2.0.375783 retargeting `ovstage::ovstage` to a loader stub).
- **Manual:** AC-1 — the `readelf -d` check that the installed target emits
  `DT_RPATH`, not `DT_RUNPATH`. Nothing in the suite inspects the ELF dynamic
  section directly; a regression that reintroduced `DT_RUNPATH` would only be
  caught if it also happened to break AC-2's runtime behavior, which is not
  guaranteed for every possible dependency shape. See *Why the ELF property
  is not automated*.

## Scenario

The installed `ovphysx::ovphysx` CMake target must let a consumer built with
nothing beyond `find_package(ovphysx)` load and run successfully on Linux,
even when `libovphysx.so`'s own transitive shared-library dependencies (e.g.
`libovstage.so`) are not otherwise preloaded into the consumer process. This
requires the consumer to link with classic `DT_RPATH` rather than the
non-transitive `DT_RUNPATH` linkers emit by default. Covers
REQ-PACKAGING-RPATH-001 AC-1 and AC-2.

## Given

- An `ovphysx/` checkout with dependencies fetched and an installed SDK at
  `_install/` (`cmake -P scripts/install.cmake`, after a normal build).
- `readelf` on `PATH` (for the manual half).

## When

- **Automated pass:** `cmake -P scripts/validate_all.cmake` (or, to run just
  this suite against an existing install, `cmake -P
  scripts/test_cpp_samples.cmake`).
- **Manual pass:** after the automated pass has built
  `tests/c_samples/clone_c` into `_build/sample_tests/c_samples/clone_c/`,
  run `readelf -d <that executable> | grep -E 'RPATH|RUNPATH'`.

## Then

- Every sample in `tests/c_samples/` builds and runs to completion with exit
  code 0, including `clone_c`, which exercises `ovphysx_create_instance()`
  through to a running simulation step (REQ AC-2). A regression that dropped
  `LINKER:--disable-new-dtags` from `ovphysxConfig.cmake.in`, or that
  reintroduced a dependency the loader can no longer resolve transitively,
  reproduces the original failure mode: `error while loading shared
  libraries: libovstage.so: cannot open shared object file`, exit code 127.
- The `readelf -d` output for `clone_c` contains a `(RPATH)` entry and no
  `(RUNPATH)` entry (REQ AC-1).

## Why the ELF property is not automated

`readelf -d`'s output is not parsed by any CTest case — the suite only
observes whether the sample *runs*, not the specific ELF tag structure that
makes it run. A future change could coincidentally keep AC-2 passing (e.g. by
restoring the old "already resident" accident this REQ documents, if some
other linked target happens to preload the same library again) while quietly
reintroducing `DT_RUNPATH`, leaving AC-1 silently unverified until the next
dependency-shape change exposes it. Wiring a `readelf -d` grep into
`test_cpp_samples.cmake` (or a dedicated CTest case) would close this gap
directly; it has not been added because the failure mode it would catch
requires two independent changes to coincide, and the AC-2 automation already
gives fast, high-value coverage of the same underlying fix.
