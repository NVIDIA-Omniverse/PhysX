<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-OMNICLIENT-001
title: OVStage owns OmniClient in ovphysx applications
status: implemented
owner: ovphysx
---

## Description

ovphysx consumes an already-populated OVStage and has no OmniClient API or
asset-loading responsibility. The application-supplied OVStage distribution
owns OmniClient and its connection library. This boundary applies regardless
of whether `PhysX()` or OVStage population runs first.

## Acceptance Criteria

- AC-1: **No bundled client.** The ovphysx SDK and wheel contain no OmniClient
  library, OmniClient connection library, or OmniClient provenance/version
  file.
- AC-2: **No client bootstrap.** Constructing `PhysX()` neither loads
  OmniClient nor rejects an OmniClient already loaded by the host based on its
  version.
- AC-3: **OVStage remains functional.** A local USD scene can be populated by
  the application-supplied OVStage, attached, and stepped when either
  `PhysX()` or OVStage population runs first; any OmniClient runtime observed
  after population comes from OVStage rather than the ovphysx package.

## Test References

- [TEST-PACKAGING-OMNICLIENT-001](../../tests/packaging/TEST-PACKAGING-OMNICLIENT-001.md)

## Code References

- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp (`CarboniteLoader::initialize`
  performs no OmniClient load or version check - AC-2)
- ovphysx/scripts/package_deps.py (`package_deps` does not copy OmniClient,
  its connection library, or a version file - AC-1)
- ovphysx/scripts/verify_pyless_closure.py (`FORBIDDEN_FILENAMES`,
  `FORBIDDEN_DT_NEEDED`, and `FORBIDDEN_PE_IMPORTS` fail packaging if an
  OmniClient payload or direct dependency reappears - AC-1, AC-2)
- ovphysx/tests/python_tests/test_package_deps.py (package absence and
  missing-client acceptance - AC-1)
- ovphysx/tests/python_tests/test_ovstage_runtime_singletons.py (process-isolated
  startup-order, real USD population, attach, and step coverage - AC-2, AC-3)
- ovphysx/tests/python_tests/test_verify_pyless_closure.py (offline verifier
  policy coverage - AC-1, AC-2)

## Dependencies

None.
