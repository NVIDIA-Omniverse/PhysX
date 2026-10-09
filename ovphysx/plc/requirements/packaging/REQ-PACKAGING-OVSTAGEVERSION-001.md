<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-OVSTAGEVERSION-001
title: SDK OVStage Requirement Follows the Release Pin
status: implemented
owner: ovphysx
---

## Description

The installed CMake package derives its OVStage requirement from
`OVSTAGE_VERSION` in `scripts/fetch_ovstage_release.py`, the same pin used
by source builds and Python dependencies. SDK users obtain the required
release from package metadata rather than a separately maintained version
in the getting-started documentation.

## Acceptance Criteria

- AC-1: **Generated metadata.** `ovphysxConfig.cmake` exposes the full pin
  as `ovphysx_OVSTAGE_VERSION` and its first three components as
  `ovphysx_OVSTAGE_CMAKE_VERSION`. Changing the source pin causes CMake
  configuration to regenerate the package metadata.
- AC-2: **Consumer version selection.** `find_package(ovphysx)` requests
  an exact major/minor/patch match from OVStage's CMake package. A different
  release fails required dependency resolution during configuration. The
  existing schema-registration capability check remains in force.
- AC-3: **Invalid pins fail.** Package generation fails when the pin source
  is missing, the assignment is missing or duplicated, or its value is not
  `major.minor.patch[.build[.hash]]` with numeric version components.

## Known limitations

OVStage's native CMake package reports only major/minor/patch. Builds of
the same release cannot be distinguished by that version check; the full
pin remains available in the generated metadata to select the download.

## Test References

- [TEST-PACKAGING-OVSTAGEVERSION-001](../../tests/packaging/TEST-PACKAGING-OVSTAGEVERSION-001.md)

## Code References

- `ovphysx/cmake/ReadOvstageVersion.cmake` — parses the authoritative pin
  and rejects invalid input (AC-1, AC-3).
- `ovphysx/CMakeLists.txt` — reads the pin, tracks it as a configure
  dependency, and generates the installed config (AC-1, AC-3).
- `ovphysx/cmake/ovphysxConfig.cmake.in` — exposes metadata and enforces
  the native dependency requirement (AC-1, AC-2).

## Dependencies

- REQ-PACKAGING-TESTDEPS-001 — Python dependency copies use the same pin.
