<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-RPATH-001
title: Installed ovphysx::ovphysx Target Resolves Transitive Runtime Dependencies On Linux
status: implemented
owner: ovphysx
---

## Description

`libovphysx.so` has a direct ELF `NEEDED` on `libovstage.so`, but only carries
`RPATH=$ORIGIN` on itself — it depends on whatever process loads it to also
make `libovstage.so`'s directory available at load time. On Linux, the
installed `ovphysxConfig.cmake` (generated from `cmake/ovphysxConfig.cmake.in`)
gives every `find_package(ovphysx)` consumer an `INTERFACE_LINK_OPTIONS` rpath
list pointing at ovphysx's own install `lib/` and `plugins/`
directories, plus (via CMake's automatic import-target RPATH) the directory of
whatever library the `ovstage::ovstage` imported target resolves to.

That rpath list alone is not sufficient: modern linkers default to emitting
`DT_RUNPATH`, which the ELF loader only consults to resolve an object's own
*direct* `NEEDED` entries — it is not inherited by that object's own
dependencies. A consumer executable's `DT_RUNPATH` therefore cannot satisfy
`libovphysx.so`'s `NEEDED` on `libovstage.so` unless `libovstage.so` happens
to already be resident in the process for some other reason (e.g. the
consumer also links a target that resolves to that same physical file).

This was latent until ovstage 0.2.0.375783 retargeted the `ovstage::ovstage`
CMake target from `libovstage.so` (the real implementation) to a new
`libovstage-dynamic.so` loader stub, while `libovphysx.so` — built via
ovphysx's own internal, non-CMake-config resolution of ovstage
(`ovphysx/ovruntime/cmake/OvstageDependency.cmake`) — kept its `NEEDED` on the
original `libovstage.so`. A consumer that links `ovphysx::ovphysx` alone no
longer pulls `libovstage.so` in as a direct dependency, so the "already
resident" accident stopped hiding the gap: `find_package(ovphysx)` consumers
failed at process startup with `error while loading shared libraries:
libovstage.so: cannot open shared object file`. `ovphysx/tests/c_unittests/CMakeLists.txt`
already carries the same fix for its own binaries, with a longer comment
on the same DT_RUNPATH-non-transitivity mechanism.

## Acceptance Criteria

- AC-1: **The installed `ovphysx::ovphysx` CMake target links Linux consumers
  with old-style `DT_RPATH`, not `DT_RUNPATH`.** `ovphysxConfig.cmake.in`
  appends `LINKER:--disable-new-dtags` to `ovphysx::ovphysx`'s
  `INTERFACE_LINK_OPTIONS` under `if(UNIX AND NOT APPLE)`, alongside its
  existing `-Wl,-rpath,...` entries. `DT_RPATH` is honored by the ELF loader
  when resolving the `NEEDED` entries of every object loaded into the
  process, not only the direct `NEEDED` entries of the object that carries
  it — so a consumer's rpath list also resolves `libovphysx.so`'s own
  transitive dependencies (e.g. `libovstage.so`), which a `DT_RUNPATH` would
  not.
- AC-2: **A consumer that does nothing beyond `find_package(ovphysx)` +
  `target_link_libraries(consumer ovphysx::ovphysx)` runs successfully**,
  with no hand-added RPATH, `LD_LIBRARY_PATH`, or other runtime-path setup of
  its own — including against an ovstage package whose `ovstage::ovstage`
  CMake target resolves to a different physical library file than the one
  `libovphysx.so` itself was built against (e.g. ovstage's loader-stub
  packaging).

## Test References

- [TEST-PACKAGING-RPATH-001](../../tests/packaging/TEST-PACKAGING-RPATH-001.md)
  — partly automated. AC-2 is automated by the existing `cpp-samples` CTest
  suite (`scripts/test_cpp_samples.cmake`), which already exercises this path
  by construction. AC-1 (the `DT_RPATH` vs `DT_RUNPATH` ELF property itself)
  is verified by hand with `readelf -d`; see that TEST for the exact command
  and why it is not yet wired into CTest.

## Code References

- `ovphysx/cmake/ovphysxConfig.cmake.in` — the `if(UNIX AND NOT APPLE)` block
  setting `ovphysx::ovphysx`'s `INTERFACE_LINK_OPTIONS` (AC-1, AC-2).
- `ovphysx/tests/c_unittests/CMakeLists.txt` — the pre-existing analogous fix
  for internal test binaries, and its longer explanatory comment on the same
  `DT_RUNPATH` non-transitivity mechanism.
- `ovphysx/ovruntime/cmake/OvstageDependency.cmake` — the internal,
  non-CMake-config resolution of ovstage that gives `libovphysx.so` its direct
  `NEEDED` on `libovstage.so`.
- `ovphysx/tests/c_samples/*/CMakeLists.txt` — sample executables that
  exercise this path purely through `find_package(ovphysx)` (AC-2).

## Dependencies

None.
