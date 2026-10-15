<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-TESTRESULT-001
maps_to: REQ-PACKAGING-TESTRESULT-001
type: unit
---

## Scenario

Feed synthetic result XML to the actual CMake parser functions in isolated
processes, verifying zero-pass handling without building an SDK or using a GPU.

## Given

- The parser bodies and allowlist declarations extracted from the two current drivers.
- Fifty-two fixtures covering full/partial skips, healthy results, missing/invalid
  counts, failures, errors, disabled tests, the named C++ exception and the
  exact multi-GPU and solver-tasking-capacity prerequisite skips.

## When

- Run `cmake -P tests/cmake/test_result_guards.cmake` from `ovphysx/`.
- The same test runs from `scripts/ci_validate.cmake`.

## Then

- Unexpected all-skipped and all-disabled results exit nonzero (REQ AC-1).
- The C++ `cpu-no-cuda-context` full skip succeeds with a documented fully-skipped exception warning;
  Python grants no exception for that label (REQ AC-2). The warning describes
  the unverified complete scenario without claiming that no assertions ran.
- The single-case multi-GPU prerequisite skip succeeds with a warning that
  the targeted scenario was not exercised, only for the exact pass, test
  identity and skip reason; missing or
  different reasons, another case or label, disabled tests and additional
  cases fail. A healthy multi-GPU pass still succeeds (REQ AC-1, AC-2).
- Each solver count (1, 2, 4 or 8) accepts its single matching capacity skip
  with a warning that the scenario was not exercised. Unsupported counts,
  mismatched pass/case/suite identities, missing or different reasons,
  disabled or additional cases, zero tests, inconsistent counts and failures
  still fail; a healthy solver pass still succeeds (REQ AC-1 through AC-3).
- Healthy and partial-skip fixtures succeed; empty, malformed, inconsistent,
  failure and error fixtures fail, including empty/failing allowlisted passes
  (REQ AC-3).
- Multi-suite GTest reports without a root skipped count sum every child's
  skipped count: all-skipped children fail, and a partially skipped report
  reports the exact remaining passed cases (REQ AC-1, AC-3).

This unit test validates result classification, not the runtime cause of any skip.
