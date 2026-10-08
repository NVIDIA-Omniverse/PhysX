<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-OVSTAGEDEP-001
title: ovruntime's Internal Ovstage Resolution Survives A Missing Plain Import Library
status: implemented
owner: ovphysx
---

## Description

`ovphysx/ovruntime/cmake/OvstageDependency.cmake` resolves the ovstage
dependency for ovphysx's own internal build directly from the filesystem —
it bypasses ovstage's own CMake package config (`ovstageConfig.cmake`,
consumed only by external `find_package(ovphysx)` consumers via
`ovphysxConfig.cmake.in`; see REQ-PACKAGING-RPATH-001) and globs for a
platform-specific link library by name.

Bootstrapped from a build break: ovstage 0.2.0.375783 removed the plain
Windows import library `ovstage.lib` entirely, shipping only
`ovstage-static.lib` and a new `ovstage-dynamic.lib` (the import lib for a
thin loader DLL that opens the real `ovstage.dll` at first use — the same
loader-stub split REQ-PACKAGING-RPATH-001 describes for Linux's
`libovstage.so` / `libovstage-dynamic.so`). `OvstageDependency.cmake`'s
Windows glob only ever looked for `ovstage.lib`, so it failed configuration
outright on Windows CI (`ovstage layout ... is incomplete`) — a hard build
break, not merely a runtime loading gap, because there was no plain import
library left to find at all.

## Acceptance Criteria

- AC-1: **Windows resolution falls back to the `-dynamic` import library
  when the plain one is absent.** `OvstageDependency.cmake`'s
  `_ovstage_lib_candidates` glob on `WIN32` tries `ovstage.lib` first (every
  legacy/current layout it already supported); if that glob is empty, it
  retries with `ovstage-dynamic.lib` across the same set of layout roots
  before falling through to the "layout incomplete" `FATAL_ERROR`.
- AC-2: **Linux resolution has the same fallback**, from `libovstage.so` to
  `libovstage-dynamic.so`, guarding against a future ovstage release
  dropping the plain `.so` the way 0.2.0.375783 dropped Windows's plain
  `.lib` — this has not happened yet (0.2.0.375783 still ships
  `libovstage.so`), so this arm is currently unexercised by any real
  package, but keeps the two platform arms symmetric.
- AC-3: **A layout with neither the plain nor the `-dynamic` library still
  fails configuration with the existing, actionable `FATAL_ERROR`** naming
  the incomplete root and directing the caller to `-DOVSTAGE_DIR`, rather
  than silently resolving to a stale or wrong library.

## Known limitations

`OVSTAGE_LIBRARY_DIR`'s Windows detection (`ovstage.dll` present in the
candidate `bin/` directory) is unchanged and still keys off the real DLL's
presence, not the loader's — this is correct as long as ovstage keeps
shipping the real `ovstage.dll` colocated with `ovstage-dynamic.dll`, which
0.2.0.375783 does. If a future release ships the loader without the real DLL
alongside it, this detection would need to key off `ovstage-dynamic.dll`
instead; that is a distinct, currently-hypothetical failure mode this REQ
does not cover.

## Test References

- Not yet automated. No CI runner in this repository builds against an
  ovstage layout that lacks the plain import library by construction — the
  regression this REQ documents was caught by the real ovstage 0.2.0.375783
  package on Windows CI (`build-win-ovruntime`), not by a synthetic test
  fixture. Verified by hand: the fallback glob pattern was checked against
  an extracted `ovstage-0.2.0.375783-py3-none-win_amd64.whl` (confirms
  `ovstage/lib/ovstage-dynamic.lib` exists where `ovstage/lib/ovstage.lib`
  does not) and the equivalent Linux package (confirms
  `ovstage/bin/libovstage.so` still exists, so AC-2's fallback arm is inert
  for this release). CI re-running the Windows build jobs against this fix
  is the actual verification for AC-1 and AC-3.

## Code References

- `ovphysx/ovruntime/cmake/OvstageDependency.cmake` — the `WIN32` and
  non-`WIN32` `_ovstage_lib_candidates` glob blocks and their `-dynamic`
  fallback (AC-1, AC-2), and the unchanged `FATAL_ERROR` when no candidate
  resolves (AC-3).

## Dependencies

None. Documents the same ovstage packaging change as REQ-PACKAGING-RPATH-001
but a distinct obligation (ovphysx's own internal build-time library
discovery, not the installed SDK's consumer-facing runtime linking).
