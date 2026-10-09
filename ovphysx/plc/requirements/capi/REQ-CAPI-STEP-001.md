<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-STEP-001
title: GPU step failure reporting
status: implemented
owner: ovphysx
---

## Description

Simulation completion reports a GPU failure to the caller without requiring
log inspection. This contract covers synchronous steps and batches, and waits
for an asynchronous step; it does not change asynchronous dispatch bookkeeping.

## Acceptance Criteria

- **AC-1: Failed synchronous completion.** A nonzero PhysX CUDA-context error
  after simulation completion makes `ovphysx_step_sync` and `ovphysx_step_n_sync`
  return `OVPHYSX_API_ERROR`. A batch stops at the failed step. The immediate
  `ovphysx_get_last_error()` identifies the CUDA failure even with logging
  disabled. Both GPU readback and DirectGPU modes obey this contract.
- **AC-2: Successful completion.** A healthy step returns `OVPHYSX_API_SUCCESS`.
  A recoverable capacity-overflow report alone does not fail the call when the
  PhysX CUDA context has no error.
- **AC-3: Failed asynchronous completion.** `ovphysx_wait_op` reports an admitted
  step's CUDA-context failure as `OVPHYSX_API_ERROR`, identifies its failed
  operation index, and retains the CUDA diagnostic for `ovphysx_get_last_op_error`
  until the next wait on that thread, even with logging disabled. This applies
  to both the direct single-operation infinite wait and generic tracked waits.

## Test References

- [TEST-CAPI-STEP-001](../../tests/capi/TEST-CAPI-STEP-001.md)
- [TEST-CAPI-STEP-002](../../tests/capi/TEST-CAPI-STEP-002.md)

## Code References

- `ovphysx/include/ovphysx/ovphysx.h`: step and operation-wait completion contracts.
- `ovphysx/src/ovphysx/ovphysx.cpp`: synchronous and asynchronous completion error capture.
- `ovphysx/tests/c_unittests/test_step_errors.cpp`: GPU abort-state regression.

## Dependencies

None.
