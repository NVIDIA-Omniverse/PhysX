<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-ATTACH-001
maps_to: REQ-CAPI-OVSTAGE-ATTACH-001
type: integration
---

## Scenario

Direct attachment fails closed when required articulation and joint schema or relationship data
is unsealed, while a seal that covers selected physics data remains sufficient.
`read_ordinal` 0 and the negative ordinals are rejected before the runtime
attach; Python also rejects ordinals above `ovstage.ORDINAL_MAX` and
non-integer ordinals before conversion.

## Given

- The two-articulation fixture is populated at ordinal 1.
- One Stage is unsealed; partial seals either include only `usd-schemas`
  or exclude `physics:body0` and `physics:body1`. Another Stage leaves only an
  unrelated matched-root attribute outside an attribute-scoped seal.
- A caller-owned Stage is created for the non-positive ordinal argument check.
- Python has a sealed Stage and invalid negative (including
  `ovstage.ORDINAL_MIN`), zero, int64-overflow (`ovstage.ORDINAL_MAX + 1`,
  `2**64 - 1`), uint64-overflow, float and string ordinal values.

## When

- `ovphysx_attach_ovstage()` is called on each unsealed or partially sealed
  Stage, its task is collected, and the same Stage is retried after sealing.
- `ovphysx_attach_ovstage()` is called on the attribute-scoped sealed Stage.
- `ovphysx_attach_ovstage()` is called with `read_ordinal` 0, -1 and
  `OVSTAGE_ORDINAL_MIN`.

- Python attempts attachment with each invalid value, then retries at ordinal 1.

## Then

- The unsealed attach returns an accepted task whose wait fails with no partial
  attachment. Its retained error names the schema scan,
  `ovstage_read_attributes` and ordinal 1. The sealed retry's task completes
  successfully (REQ AC-1).
- Both partial-seal tasks fail and `ovphysx_is_stage_attached` reports false.
  The error names ordinal 1; when only joint endpoints are unreadable it also
  names the relationship read, attribute and prim. Each sealed retry succeeds
  and its articulation query reports two roots (REQ AC-1).
- The valid attribute-scoped sealed attach succeeds and its articulation query
  reports two roots (REQ AC-2).
- Each non-positive attach returns `OVPHYSX_API_INVALID_ARGUMENT` without a
  task and a later valid attach on the same handle still succeeds (REQ AC-3).
- The C API's attach path supplies compatibility stage id zero. Source and
  binary inspection find no dynamic resolution or import of
  `ovstage_get_usd_stage_id` in `libovphysx` or the production runtime library
  (REQ AC-4; manual verification alongside the automated attach cases).
- `ovphysx/tests/c_unittests/test_usd_loading.cpp`:
  `UnsealedAttachFailsAndSealedRetrySucceeds`,
  `PartialSealAttachTest.FailedAttachCanRetryAfterSealingJointRelationships`
  (schema-only and excluded-relationship parameters),
  `ScopedSealAllowsUnrelatedUnsealedData`,
  `AttachOvstageRejectsZeroReadOrdinal` (zero and negative ordinals), and
  `AttachAndUpdateReturnCollectableTasks` (accepted identity, duplicate-attach
  failure at collection and collect-once completion for REQ AC-5).
- `ovphysx/tests/c_unittests/test_task_completion.cpp`:
  `LifecycleInvalidArgumentsAndMissingAttachmentRejectWithoutTaskIdentity`
  verifies a null-Stage attach rejects without a task (REQ AC-5).
- `ovphysx/tests/python_tests/cpu_tests/test_attach_ordinal.py` rejects invalid
  values without taking Stage ownership; a valid retry succeeds (REQ AC-3).
