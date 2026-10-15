<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-OVSTAGE-001
title: OVStage Native Dependency Acquisition
status: implemented
owner: ovphysx
---

## Description

ovphysx acquires its mandatory OVStage dependency from the artifacts OVStage
publishes for native consumers: a per-platform native zip for the C/C++ package
and a Python wheel for the Python package. A committed SHA-256 identifies each
native archive because its CMake metadata carries no build number or git hash.
Locked Python installs verify the wheel against the hash in `uv.lock`;
unlocked installs use normal index integrity checks.

OVStage remains application-owned. An application that already provides an
OVStage package keeps it; ovphysx never introduces a second provider. An SDK
consumer that has none can build without supplying one, because the SDK ships
the fetcher and its CMake package invokes it.

## Acceptance Criteria

- AC-1: The native package is obtained from the released archive
  `ovstage@<numeric>.<githash>.<platform>.zip`, extracted so that the package
  prefix (`include/`, `lib/cmake/ovstage/`, `bin/`) lands at the OVStage root.
  Extraction rejects an archive whose entries escape the destination, that lacks
  `include/` or `bin/`, or that lacks the `THIRD-PARTY-NOTICES.txt` the SDK
  install's license check requires.

- AC-2: The native archive is verified against a per-platform SHA-256 committed
  in the repository; a mismatch or a platform with no committed digest is fatal. An unpinned
  platform names the regeneration tool in its error.

  This covers the native half only. The Python wheel is verified where a lock is
  in play -- `uv run --locked` checks it against the per-wheel hash in
  `uv.lock` -- and not on the `uv pip install` paths, which resolve from the
  index like a `pip install ovphysx` consumer.

- AC-3: The internal and public channels serve the same archive filename, and
  are switched together as a pair (native zip and wheel index) by internal-only
  release tooling. The shipped fetcher exposes no consumer-facing mirror choice.
  A source-only switch verifies every target-channel native archive against the
  existing digests and fails if the bytes differ.
  Bumping the pin regenerates every digest and fails if any platform's artifact
  is unavailable, so a partially-published build cannot be pinned.

- AC-4: `find_package(ovphysx)` resolves OVStage without consumer-supplied
  flags. It honours a compatible OVStage already on `CMAKE_PREFIX_PATH`
  and only bootstraps when none is found; `OVPHYSX_FETCH_OVSTAGE=OFF`
  forbids the download. When OVStage cannot be resolved, ovphysx reports a
  not-found package rather than raising, so `find_package(ovphysx QUIET)` and
  optional-feature probes degrade cleanly.

- AC-5: A consumer can launch a built executable without naming OVStage paths by
  hand: its runtime directories are added to the RPATH on Linux, and are
  obtainable on Windows through the package's runtime-path helpers. The generated
  Windows launcher preserves the caller's `PATH` and returns the child's exit code.

- AC-6: The Python wheel is used only by consumers that install it as a Python
  distribution. No build, packaging, or CI path reconstructs a native OVStage
  package by extracting the wheel. How that wheel is then resolved and verified
  is REQ-PACKAGING-TESTDEPS-001's subject, not restated here.

- AC-7: Source staging preserves the configured OVStage native URL, version,
  digests, Python indexes and locks. Only the explicit source-switch tool changes
  channels. Private staging scans may allow the canonical native Artifactory URL
  assignment in the OVStage fetcher; other internal references remain errors.
  Release-tree validation defaults to public mode and requires an explicit option
  for this private staging exception.

## Test References

- TEST-PACKAGING-OVSTAGE-001
- TEST-PACKAGING-OVSTAGE-002
- TEST-PACKAGING-OVSTAGE-003

## Code References

- ovphysx/scripts/fetch_ovstage_release.py
- ovphysx/scripts/sync_ovstage_version.py
- ovphysx/tools/copy_to_opensource.py
- ovphysx/tools/oss_release_checks.py
- ovphysx/tools/validate_oss_release_tree.py
- ovphysx/cmake/OvstageBootstrap.cmake
- ovphysx/cmake/ovphysxConfig.cmake.in
- ovphysx/CMakeLists.txt
- ovphysx/scripts/fetch_deps.cmake
- ovphysx/scripts/test_basic_workflow.py
- ovphysx/scripts/test_cpp_samples.cmake
- ovphysx/scripts/ovstage_index.cmake
- ovphysx/scripts/validate_python_test_uv_lock.py
- ovphysx/tests/python_tests/pyproject.toml
- ovphysx/tests/python_tests/test_cmake_package_config.py
- ovphysx/tests/python_tests/test_basic_workflow_smoke.py
- ovphysx/tests/python_benchmarks/pyproject.toml
- ci/ovphysx-nogpu/setup_nogpu_env.cmake

## Dependencies

- ADR-0033 (ovphysx/ovruntime/plc/adr/ADR-0033-ovstage-native-dependency-delivery.md)
- REQ-PACKAGING-TESTDEPS-001 -- the Python wheel half of the same pin
