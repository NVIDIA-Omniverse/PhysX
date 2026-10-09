<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-ATTACH-001
title: Fail-Closed Unsealed Articulation Attach
status: implemented
owner: ovphysx
---

## Description

Direct `ovphysx_attach_ovstage()` must not report success after an unsealed
initial whole-stage scan drops articulation and joint schema data. The producer
can seal and retry the caller-owned Stage without changing the C ABI or OVStage
sealing semantics. `read_ordinal` 0 is reserved as the runtime skip-cursor
sentinel and is not a valid public attach ordinal.

## Acceptance Criteria

- AC-1: An unreadable `usd-schemas` query/read during initial articulation and
  joint enumeration returns `OVPHYSX_API_ERROR`, leaves no partial attachment,
  and preserves the schema-scan cause in `ovphysx_get_last_error()`. The same
  Stage can be retried after sealing; a successful retry clears that error.
- AC-2: Attachment does not reject solely because the Stage's global minimum
  write floor is below the requested ordinal. A valid attribute-scoped seal
  succeeds when it covers all data selected by the physics scan, even if an
  unrelated attribute remains unsealed.
- AC-3: `ovphysx_attach_ovstage()` with `read_ordinal == 0` returns
  `OVPHYSX_API_INVALID_ARGUMENT`, leaves the instance unattached, and does not
  call through to runtime attach.

## Test References

- TEST-CAPI-OVSTAGE-ATTACH-001

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_attach_ovstage`)
- ovphysx/tests/c_unittests/test_usd_loading.cpp (C API regressions)

## Dependencies

- ovphysx/ovruntime/source/omni.physics.parse/ScanBackend.cpp (runtime scan failure detection)
- ovphysx/ovruntime/source/omni.physx/plugins/usdLoad/LoadUsd.cpp (attach-result propagation and rollback)
- ovphysx/ovruntime/include/omni/physx/RuntimeError.h (runtime cause captured at the synchronous attach boundary)
