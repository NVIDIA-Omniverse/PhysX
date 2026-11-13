<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-RELEASE-001
title: Sidecar internal release-status contract
status: implemented
owner: physics-runtime
---

## Description
The internal sidecar release status distinguishes confirmed cleanup from an
unconfirmed attempt: `1` confirms success, `0` confirms terminal failure, and
`-1` means cleanup completion is unconfirmed. Query release can report terminal
failure after consuming the query. Group release can instead reject runtime
admission while leaving the group live, so its descriptor must be retained.

## Acceptance Criteria
- AC-1: `ovphysx_internal_release_query` returns `0` when the runtime accessor
  reports terminal failure, `1` on success, and `-1` when the accessor is
  unavailable or a thrown exception is caught.
- AC-2: `ovphysx_internal_release_group` retains the group's descriptor and
  returns `-1` when the runtime accessor rejects release. A later internal
  cleanup attempt must reach the accessor again. Confirmed success returns `1`
  and removes the descriptor; further releases succeed without another backend
  call. An unavailable accessor or caught exception also returns `-1` and
  retains the descriptor. This does not permit callers to retry an accepted
  public release; cleanup remains the SDK's responsibility.

## Test References
- TEST-CAPI-RELEASE-001

## Code References
- ovphysx/src/ovphysxInternal/ovphysxInternalInterop.cpp
- ovphysx/src/include/internal/sidecar/ovphysxInternalInterop.h

## Dependencies
- None
