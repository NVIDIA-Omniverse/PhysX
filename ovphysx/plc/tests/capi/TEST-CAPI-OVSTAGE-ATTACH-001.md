<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-ATTACH-001
maps_to: REQ-CAPI-OVSTAGE-ATTACH-001
type: integration
---

## Scenario

Direct attachment fails closed when required articulation and joint schema data
is unsealed, while a seal that covers selected physics data remains sufficient.
`read_ordinal` 0 is rejected before the runtime attach.

## Given

- The two-articulation fixture is populated at ordinal 1.
- One Stage is unsealed; another leaves only an unrelated matched-root
  attribute outside an attribute-scoped seal.
- A caller-owned Stage is created for the zero-ordinal argument check.

## When

- `ovphysx_attach_ovstage()` is called on the unsealed Stage, then retried on
  the same Stage after sealing.
- `ovphysx_attach_ovstage()` is called on the attribute-scoped sealed Stage.
- `ovphysx_attach_ovstage()` is called with `read_ordinal == 0`.

## Then

- The unsealed attach returns an accepted task whose wait fails with no partial
  attachment. Its retained error names the schema scan,
  `ovstage_read_attributes` and ordinal 1. The sealed retry's task completes
  successfully (REQ AC-1).
- The attribute-scoped sealed attach succeeds (REQ AC-2).
- The zero-ordinal attach returns `OVPHYSX_API_INVALID_ARGUMENT` and a later
  valid attach on the same handle still succeeds (REQ AC-3).
- The C API's attach path supplies compatibility stage id zero. Source and
  binary inspection find no dynamic resolution or import of
  `ovstage_get_usd_stage_id` in `libovphysx` or the production runtime library
  (REQ AC-4; manual verification alongside the automated attach cases).
- `ovphysx/tests/c_unittests/test_usd_loading.cpp`:
  `UnsealedAttachFailsAndSealedRetrySucceeds`,
  `ScopedSealAllowsUnrelatedUnsealedData`,
  `AttachOvstageRejectsZeroReadOrdinal`, and
  `AttachAndUpdateReturnCollectableTasks` (accepted identity, duplicate-attach
  rejection and collect-once completion for REQ AC-5).
- `ovphysx/tests/c_unittests/test_task_completion.cpp`:
  `LifecycleInvalidArgumentsAndMissingAttachmentRejectWithoutTaskIdentity`
  verifies a null-Stage attach rejects without a task (REQ AC-5).
