<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-UPDATE-002
title: OVStage Update Failure Diagnostics
status: implemented
owner: ovphysx
---

## Description

`ovphysx_update_from_ovstage()` preserves available synchronous runtime error
detail when the native update fails. The native result remains authoritative;
capturing a diagnostic alone does not make an update fail.

## Acceptance Criteria

- AC-1: **Failure cause.** When `IPhysxSimulation::updateFromOvStage()` returns
  false, the C API returns `OVPHYSX_API_ERROR` and `ovphysx_get_last_error()`
  contains the update context and the first available runtime error captured
  during that call.
- AC-2: **Context fallback.** If the failed native update records no detail,
  the public error contains the update context without reusing an earlier
  operation's diagnostic.
- AC-3: **Successful update.** When the native update returns true, the C API
  returns `OVPHYSX_API_SUCCESS` and clears the public error, including when a
  recoverable runtime diagnostic was captured during the call.

## Test References

- [TEST-CAPI-OVSTAGE-UPDATE-002](../../tests/capi/TEST-CAPI-OVSTAGE-UPDATE-002.md)
  is specification only; direct automated update-diagnostic coverage is not
  implemented yet.

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_update_from_ovstage`; AC-1, AC-2, AC-3)
- ovphysx/src/include/internal/sdk/ovphysxSDK.hpp (shared `set_runtime_error`
  and `success` helpers used by the update boundary)

## Dependencies

- REQ-RUNTIME-ERROR-001 (synchronous first-error capture and scope isolation)
