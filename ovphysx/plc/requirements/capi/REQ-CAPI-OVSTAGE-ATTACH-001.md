<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-ATTACH-001
title: Direct OVStage Attachment
status: implemented
owner: ovphysx
---

## Description

Direct `ovphysx_attach_ovstage()` must not report success after an unsealed
initial whole-stage scan drops articulation and joint schema or relationship data. The producer
can seal and retry the caller-owned Stage without changing the C ABI or OVStage
sealing semantics. `read_ordinal` 0 is reserved as the runtime skip-cursor
sentinel and is not a valid public attach ordinal. OVStage ordinals are
`int64_t`; the runtime keeps its ordinals unsigned, so the negative ordinals
OVStage also writes are not attachable either.

## Acceptance Criteria

- AC-1: An unreadable `usd-schemas` query/read or required joint relationship
  read during initial articulation and joint enumeration fails an accepted task
  and leaves no partial attachment. The collected task error identifies the scan
  cause and ordinal; a relationship-read error also identifies the attribute and
  prim. The same Stage can be retried after sealing; the retry's task completes
  successfully with its articulations intact.
- AC-2: Attachment does not reject solely because the Stage's global minimum
  write floor is below the requested ordinal. A valid attribute-scoped seal
  succeeds when it covers all data selected by the physics scan, even if an
  unrelated attribute remains unsealed.
- AC-3: `ovphysx_attach_ovstage()` with `read_ordinal <= 0` returns
  `OVPHYSX_API_INVALID_ARGUMENT`, leaves the instance unattached, and does not
  call through to runtime attach. Python rejects ordinals outside
  `[1, ovstage.ORDINAL_MAX]` with `ValueError`, and non-integers with
  `TypeError`, before conversion or native submission; the runtime attach
  itself fails for an unsigned ordinal above `OVSTAGE_ORDINAL_MAX`. A
  successful attachment at `OVSTAGE_ORDINAL_MAX` marks every representable
  ordinal consumed, so subsequent update ranges are successful no-ops.
  Continuing edits require an actual sealed attach ordinal below that maximum.

- AC-4: `ovphysx_attach_ovstage()` always supplies a zero compatibility USD
  stage id and never resolves or calls `ovstage_get_usd_stage_id`. It attaches
  through the caller-owned ovstage data and sealed ordinal alone.

- AC-5: Attach returns `ovphysx_enqueue_result_t`. Argument, handle and
  ordinal checks reject without a task or attachment mutation. Already-attached
  state and runtime attach failures fail the accepted task, so a queued detach
  followed by attach can succeed without an intermediate wait. The caller retains the Stage and readable
  sealed snapshot through completion, and retains an attached Stage until
  successful detach or destruction. Failed rollback may retain attachment
  ownership; the synchronous attachment-state query reports that ownership.

## Test References

- TEST-CAPI-OVSTAGE-ATTACH-001

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_attach_ovstage`)
- ovphysx/tests/c_unittests/test_usd_loading.cpp (C API regressions)
- ovphysx/python/ovphysx/api.py (`PhysX.attach_ovstage` argument validation)
- ovphysx/ovruntime/source/omni.physx/plugins/PhysX.cpp (`attachOvstageWithAdmission` ordinal bound)
- ovphysx/tests/python_tests/cpu_tests/test_attach_ordinal.py (Python range and retry checks)

## Dependencies

- ovphysx/ovruntime/source/omni.physics.parse/ScanBackend.cpp (runtime scan failure detection)
- ovphysx/ovruntime/source/omni.physx/plugins/usdLoad/LoadUsd.cpp (attach-result propagation and rollback)
- ovphysx/ovruntime/include/omni/physx/RuntimeError.h (runtime cause captured at the synchronous attach boundary)
