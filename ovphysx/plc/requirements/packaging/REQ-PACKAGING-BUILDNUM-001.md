<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-BUILDNUM-001
title: CI Artifacts Carry The Build Number As A Fourth Version Component
status: implemented
owner: ovphysx
---

## Description

`ovphysx/VERSION` names the release (`0.6.3`) and is the only version a person
edits. Every artifact a CI pipeline produces additionally carries that
pipeline's id as a fourth PEP 440 release component — `0.6.3.1234567` — so an
artifact in a consumer's hands identifies the pipeline that built it without
any external record.

The number is the **parent** pipeline id, forwarded into the ovphysx child
pipeline as `OVPHYSX_BUILD_NUMBER`. A child pipeline has its own
`CI_PIPELINE_ID`, which no consumer-facing URL refers to and which would differ
between the parent bridge and the job that builds the wheel.

The whole obligation is that the number is applied *consistently*. The wheel
filename, the SDK archive name and the Artifactory publish path are each
computed by a different script from the same `VERSION` file; if one of them
applies the number and another does not, the publish step looks for a file that
was never built. `apply_build_number()` is the single implementation all of
them call.

A build with no build number in its environment — any developer build — keeps
the bare `VERSION` value, so local artifacts stay stable across rebuilds.

## Acceptance Criteria

- AC-1: `apply_build_number()` appends `.<number>` to a base version matching
  `^[0-9]+\.[0-9]+\.[0-9]+$`. It sources the number from the
  `OVPHYSX_BUILD_NUMBER` CMake variable, then the `OVPHYSX_BUILD_NUMBER`
  environment variable, then `CI_PIPELINE_ID`, and returns the base version
  unchanged when none is set.

- AC-2: A non-numeric build number is ignored with a warning, and a base
  version that is not exactly `X.Y.Z` (a pre-release such as `0.6.3.rc1`) is
  returned unchanged. Neither case fails the build, and neither emits a version
  the packaging tools cannot parse.

- AC-3: The build number is applied before the branch-local segment, never
  after: a trunk wheel is `0.6.3.1234567+trunk.<sha8>`, so the PEP 440 local
  `+` segment remains last.

- AC-4: The Python wheel version, `ovphysx.__version__`, the SDK archive
  filename, and the Artifactory repository path and `;version=` property for
  both the binary artifacts and the public source distro all resolve to the
  same build-numbered version within one pipeline.

- AC-5: `OVPHYSX_VERSION_STRING` and, on Windows, the `FileVersion` and
  `ProductVersion` resource fields carry the build-numbered version.
  `OVPHYSX_VERSION_MAJOR` / `_MINOR` / `_PATCH` and the CMake `project()`
  version stay the three-component `VERSION` value.

- AC-6: `ovphysx/VERSION` is never rewritten by the build. The build number
  exists only in generated and stamped outputs.

- AC-7: `_check_version_match()` compares only the first three release
  components. A Python package and native library from the same release but
  different builds — which a source-tree or editable install always produces,
  because `_version.py` exists only in a staged wheel — logs a warning and
  continues; a genuine release difference (`0.6.3` against `0.6.4`) still
  raises.

## Test References

- TEST-PACKAGING-BUILDNUM-001

## Code References

- ovphysx/scripts/crossplatform_helpers.cmake — `apply_build_number()`
  (AC-1, AC-2), `append_branch_local_version()` ordering (AC-3)
- ovphysx/CMakeLists.txt — `OVPHYSX_VERSION_FULL` (AC-5)
- ovphysx/include/ovphysx/version.h.in — `OVPHYSX_VERSION_STRING` (AC-5)
- ovphysx/src/ovphysx/ovphysx.rc.in — Windows resource fields (AC-5)
- ovphysx/scripts/build_common.cmake — `generate_python_version_file()` (AC-4)
- ovphysx/scripts/package_sdk.cmake — SDK archive name (AC-4)
- ovphysx/scripts/push_artifacts.cmake — wheel/archive names, publish paths (AC-4)
- ovphysx/scripts/read_artifact_version.cmake — shared CMake version resolver
  for the aggregate publisher (AC-1, AC-2, AC-4, AC-6)
- ovphysx/scripts/publish_release_artifacts.py — aggregate staged-set validation
  and publication paths (AC-4)
- ovphysx/scripts/push_public_source_distro.cmake — source distro path (AC-4)
- ovphysx/python/ovphysx/api.py — `_check_version_match()` (AC-7)
- ovphysx/tools/internal/relay_ci_wheel_to_ngc.py — `_artifact_version()`,
  `_expected_wheel_version()`, `verify_pdx_publication()` (AC-4)

## Dependencies

- None
