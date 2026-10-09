<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-BUILDNUM-001
maps_to: REQ-PACKAGING-BUILDNUM-001
type: unit
---

## Scenario

`apply_build_number()` is the single point every packaging script calls, so the
consistency AC-4 asks for follows from this one function behaving the same way
in each of them. The test drives it directly in CMake script mode rather than
inspecting a built artifact: it is the only way to cover the sources,
precedence and rejection rules without running five publish paths.

## Given

- `ovphysx/scripts/crossplatform_helpers.cmake` on `PATH`-resolvable `cmake`.
- A controlled environment, so a real `CI_PIPELINE_ID` in the developer's shell
  cannot reach the probe and turn a negative case green.

## When

- `test_build_number.py` writes a one-line CMake script that includes the
  helpers, calls `apply_build_number()` with a chosen base version, and prints
  the result, then runs it under a chosen environment and `-D` cache variable.
- One case additionally calls `append_branch_local_version()` on the result to
  check the segment order.
- `test_main_accepts_the_build_numbered_staged_set` drives the aggregate
  publisher's entry point against six staged artifacts. It covers a local
  version, the child pipeline fallback, the parent pipeline override,
  a rejected non-numeric number, and a pre-release version,
  while replacing remote publication with local staged-set validation.
- The publisher parser is exercised with CMake status output, missing or
  duplicate version markers, and a failed CMake process. The real CMake
  integration cases can also run with CMake 3.16 on `PATH`.
- The NGC relay's lock-based dependency test supplies a parent pipeline ID
  and a numbered trunk wheel, using the real expected-version calculation.
- `test_version_mismatch.py` calls `_check_version_match()` directly with
  controlled Python and native version strings, without constructing a PhysX
  instance. It covers unnumbered/numbered pairs in both directions, different
  build numbers, matching builds, branch-local suffixes, and differing release
  components. Warning cases capture the Python logger.

## Then

- With no build number in the environment the base version is returned
  unchanged; `CI_PIPELINE_ID` is appended when present; the
  `OVPHYSX_BUILD_NUMBER` environment variable overrides it; and the
  `OVPHYSX_BUILD_NUMBER` CMake variable overrides both (REQ AC-1).
- A non-numeric build number (`abc`, `12a`, `1.2`) is dropped and the base
  version is returned unchanged (REQ AC-2).
- A base version that is not exactly `X.Y.Z` is returned unchanged. This
  includes the function's own output (`0.6.3.1234567`), so a second
  application cannot produce a five-component version (REQ AC-2).
- On a non-release branch the result is `0.6.3.1234567+dev.someone.thing.abc12345`
  — the build number ahead of the PEP 440 local segment, never after it
  (REQ AC-3).
- The aggregate publisher accepts all six files under the same numbered
  version as staging, prefers the parent number over the child's ID, and
  leaves `VERSION` unchanged (REQ AC-4, AC-6).
- CMake diagnostics cannot become part of an artifact version. Missing,
  empty or ambiguous output and CMake errors stop publication (REQ AC-4).
- The relay accepts the parent-numbered trunk wheel while resolving its
  ovstage dependency from the lock rather than the build trace (REQ AC-4).
- An unnumbered Python version `0.6.3` and numbered native version
  `0.6.3.12345678` (or the reverse) pass and log a warning identifying both
  versions. Different build numbers within `0.6.3` also pass and warn;
  matching builds do not warn, including a Python branch-local suffix.
  Different major, minor or patch components raise, including `0.6.3` versus
  `0.6.4`, even when the build numbers match (REQ AC-7).

## Known gaps

AC-4 now has an automated aggregate-publisher integration test, but its full
build-to-publication chain and AC-5 (the stamped `OVPHYSX_VERSION_STRING` and
Windows resource fields) still need a full build and publish, which no test in this
suite performs; they are verified by the existing version-mismatch guards in
`push_artifacts.cmake`, which fail the publish when the wheel or archive name
disagrees with the computed version.

## Code References

- ovphysx/tests/python_tests/test_build_number.py
- ovphysx/tests/python_tests/test_version_mismatch.py
- ovphysx/tests/python_tests/test_publish_release_artifacts.py
- ovphysx/tests/python_tests/test_relay_ci_wheel_to_ngc.py
- ovphysx/scripts/read_artifact_version.cmake
- ovphysx/scripts/crossplatform_helpers.cmake
