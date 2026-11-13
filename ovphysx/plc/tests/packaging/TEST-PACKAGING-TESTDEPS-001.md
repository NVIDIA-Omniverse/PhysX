<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-TESTDEPS-001
maps_to: REQ-PACKAGING-TESTDEPS-001
type: unit
---

## Scenario

Validate OVStage lock integrity, channel switching, and CMake's uv source settings.

## Given

- Synthetic uv lock entries and all repository locks that record OVStage.
- Temporary uv projects with named OVStage indexes.
- `scripts/ovstage_index.cmake` evaluated with CMake when available.

## When

- The validator reads HTTPS registries, local paths, and missing or malformed hashes.
- Pin and source-only edits run against temporary projects.
- The CMake helper emits settings for locked and package-only installs.
- A temporary project that does not name the `ovstage` index is passed to the helper.
- Every `scripts/*.cmake`, and `ci/ovphysx-nogpu/run_python_smoke.cmake` when present, that runs
  uv with `UV_NO_CONFIG=1` is read as text, without its comment lines.

## Then

- HTTPS registry entries with a SHA-256 per wheel pass, including PyPI entries with
  size and upload-time fields. Local registries and invalid hashes fail (AC-1).
- Discovery and pin updates cover the same project set (AC-5).
- Source-only switches update both native and wheel channels without changing the
  pinned version; combined edits preserve both changes (AC-5, AC-6).
- The NGC relay uses the OVPhysX wheel's exact dependency pin and the source
  lock without requiring an OVStage wheel filename in the build log (AC-1).
- Locked settings supply PyPI, an empty extra index and an empty `UV_INDEX`. Package-only settings add only
  the selected project's `/ovstage/` page. Neither exports the whole index as an extra
  index, and a project that does not name the `ovstage` index is rejected (AC-2, AC-3, AC-4).
- Every uv driver that uses the helper splices its output, and no driver sets an index
  variable or `UV_HTTP_RETRIES` itself (AC-2).

## Not Covered

Unit tests do not contact package indexes. CI exercises real resolution and
installation with its pinned uv version. Public-channel publication and native
archive byte identity are release checks.

## Code References

- ovphysx/tests/python_tests/test_ovstage_version_sync.py
- ovphysx/scripts/validate_python_test_uv_lock.py
- ovphysx/scripts/ovstage_index.cmake
- ovphysx/tests/python_tests/test_relay_ci_wheel_to_ngc.py
