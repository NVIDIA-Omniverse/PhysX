<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-ASYNC-001
title: Native Async Timeout Contract
status: implemented
owner: ovphysx
---

## Description

OVPhysX exposes one named native timeout type and named poll and infinite values
for timeout-bearing C and C++ APIs. Operation waits honor poll, finite, and
literal infinite semantics without consuming an operation that is still
pending.

The logging delivery barrier has additional behavior specified by
`REQ-CAPI-LOG-001`; this requirement owns the shared native timeout vocabulary
and the simulation-operation wait behavior.

## Acceptance Criteria

- AC-1: `ovphysx_timeout_t` is byte-identical to `uint64_t`, represents
  nanoseconds, and defines `OVPHYSX_TIMEOUT_POLL` as zero and
  `OVPHYSX_TIMEOUT_INFINITE` as `UINT64_MAX`.
- AC-2: Every public timeout-bearing C or C++ API uses
  `ovphysx_timeout_t`; changing from the prior `uint64_t` spelling does not
  change the C ABI.
- AC-3: A simulation-operation wait with `OVPHYSX_TIMEOUT_POLL` performs one
  readiness check without waiting for readiness. If the operation is pending,
  it returns `OVPHYSX_API_TIMEOUT`, reports the lowest pending operation, and
  leaves that operation unconsumed.
- AC-4: A simulation-operation wait with a finite positive timeout uses the
  generic tracked-operation path and waits for readiness only up to that budget.
  If the budget expires while the operation remains pending, it returns
  `OVPHYSX_API_TIMEOUT`, reports the lowest pending operation, and leaves that
  operation unconsumed. Readiness observed by the final boundary check proceeds
  to synchronous result finalization and reports the terminal result, even if
  finalization extends total call duration beyond the readiness budget.
- AC-5: A simulation-operation wait with `OVPHYSX_TIMEOUT_INFINITE` treats the
  sentinel literally as unbounded and returns only after the operation reaches
  a terminal state. An eligible single simulation operation may use the direct
  blocking-sync fast path only for this infinite-wait case.

## Test References

- TEST-CAPI-ASYNC-001

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/include/ovphysx/experimental/ovphysx.hpp
- ovphysx/src/include/internal/sdk/ovphysxAsyncWait.hpp
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/src/ovphysx/LogManager.cpp
- ovphysx/src/ovphysx/PhysXWrapper.cpp
- ovphysx/CMakeLists.txt
- ovphysx/tests/c_unittests/CMakeLists.txt
- ovphysx/tests/c_unittests/test_c_api_compatibility.c
- ovphysx/tests/c_unittests/test_physx_async.cpp

## Dependencies

- None
