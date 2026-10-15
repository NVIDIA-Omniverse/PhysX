<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-TESTRESULT-001
title: Installed-SDK test drivers reject unexpected zero-pass results
status: implemented
owner: ovphysx
---

## Description

The GTest and pytest result parsers must distinguish a successful test pass
from a suite with no cases completed successfully. Skips can occur after
useful assertions; zero passed cases does not imply that no assertions ran.
This baseline captures the test-driver safeguard from GitLab !8083 and
GitHub #95; it changes no library API.

## Acceptance Criteria

- **AC-1: No silent success with zero passed cases.** `parse_gtest_results` and
  `parse_pytest_junit` reject a nonempty result with zero passed cases unless
  its exact label is in that driver's documented fully-skipped allowlist or
  it matches a narrowly defined hardware prerequisite in AC-2.
- **AC-2: Explicit exception.** An allowlisted zero-pass result emits a warning
  stating that its complete test scenario remains unverified. The current C++
  exception is
  `cpu-no-cuda-context`; the Python allowlist is empty. The isolated
  `cuda-selection-nonzero-ordinal` pass also permits exactly one skipped,
  non-disabled `ActiveCudaGpusAttachTest.DirectGpuHostReadOnNonZeroOrdinal`
  case only when its GTest skip message states that at least two CUDA devices
  are needed to select a non-zero ordinal. This emits a hardware-prerequisite
  warning stating that the non-zero-ordinal scenario was not exercised on this
  runner. Each isolated `solver-threads-N` pass, for N in {1, 2, 4, 8}, also
  permits exactly one skipped, non-disabled `SolverThreadCount.WorkersN` case
  only when its GTest skip message states "Requested count exceeds this host's
  default tasking capacity". Its hardware-prerequisite warning states that the
  N-worker scenario was not exercised on this runner, preserving the capacity
  prerequisite in REQ-CAPI-THREADS-001 AC-1. Other reasons, cases or counts do
  not receive these exceptions.
- **AC-3: Preserve validation.** Allowlisting does not waive zero-test,
  malformed/inconsistent-count, or failure/error checks. Healthy and partially
  skipped results with at least one passed case retain their successful outcome.
  When GTest omits an aggregate skipped count, the parser sums skipped counts
  from every child suite before deciding how many cases passed.

## Test References

- [TEST-PACKAGING-TESTRESULT-001](../../tests/packaging/TEST-PACKAGING-TESTRESULT-001.md)

## Code References

- `ovphysx/scripts/test_cpp.cmake`: `parse_gtest_results` (AC-1 through AC-3).
- `ovphysx/scripts/test_python_runtime.cmake`: `parse_pytest_junit` (AC-1 through AC-3).
- `ovphysx/tests/cmake/test_result_guards.cmake`: parser regression fixtures.
- `ovphysx/scripts/ci_validate.cmake`: runs the regression fixtures in CI.

## Dependencies

None.
