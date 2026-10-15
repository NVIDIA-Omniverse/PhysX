<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-STEP-001
title: Synchronous GPU step failure reporting
status: implemented
owner: ovphysx
---

## Description

`ovphysx_step_sync` reports a GPU simulation failure to its caller without
requiring log inspection.

## Acceptance Criteria

- **AC-1: Failed GPU completion.** A nonzero PhysX CUDA-context error after
  simulation completion makes `ovphysx_step_sync` return `OVPHYSX_API_ERROR`.
  `ovphysx_get_last_error()` identifies the CUDA failure even with logging
  disabled. Both GPU readback and DirectGPU modes obey this contract.
- **AC-2: Successful completion.** A healthy step returns `OVPHYSX_API_SUCCESS`.
  A recoverable capacity-overflow report alone does not fail the call when the
  PhysX CUDA context has no error.

## Test References

- [TEST-CAPI-STEP-001](../../tests/capi/TEST-CAPI-STEP-001.md)

## Code References

- `ovphysx/include/ovphysx/ovphysx.h`: `ovphysx_step_sync` completion contract.
- `ovphysx/src/ovphysx/ovphysx.cpp`: synchronous completion error capture.
- `ovphysx/tests/c_unittests/test_step_errors.cpp`: GPU abort-state regression.

## Dependencies

None.
