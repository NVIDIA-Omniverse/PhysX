<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OBJCHANGE-002
title: Object-Change Pointer Invalidation
status: implemented
owner: ovphysx
---

## Description

Object-change notifications stay callbacks. They are how a caller observes
destruction before reusing a native pointer obtained from a `physxPointer` read.
They are not query or read records.

A per-object destruction callback means the cached pointer for that path and
type is unusable once the callback returns. The caller drops it and does not
`release()` it. Bulk teardown is only the all-objects callback; the caller
drops every cached pointer before that callback returns. A creation callback
does not hand out a pointer. The caller fetches one only after the triggering
call, or its task wait, returns.

Initial ovstage population and `ovphysx_clone` do not notify. Caches that must
include cloned objects are refreshed after the clone task wait returns.

## Acceptance Criteria

- AC-1: `ovphysx_object_destroyed_fn`, `ovphysx_all_objects_destroyed_fn`, and
  `ovphysx_subscribe_object_changes` document that a cached pointer is unusable
  once the matching callback returns, that the caller must not `release()` it,
  and that a `physxPointer` read is not performed from inside a creation callback.
- AC-2: `ovphysx_detach_ovstage` delivers `on_all_objects_destroyed` and does not
  deliver per-object destruction callbacks for that teardown.
- AC-3: Attaching and initially populating a stage, including a replay of the
  attach ordinal, delivers no per-object created or destroyed callbacks to a
  subscription registered before that population.
- AC-4: A successful `ovphysx_clone` delivers no object-change callbacks.
  Callers refresh pointer caches after `ovphysx_wait_task` returns on that clone.

## Test References

- [TEST-CAPI-OBJCHANGE-002](../../tests/capi/TEST-CAPI-OBJCHANGE-002.md)

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_subscribe_object_changes`)
- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_object_destroyed_fn`, `ovphysx_all_objects_destroyed_fn`)
- ovphysx/docs/developer_guide.md (Object-Change Notifications)
- ovphysx/tests/c_unittests/test_object_change_callbacks.cpp (`ResetDeliversNoPerObjectDestruction`, `InitialPopulationDeliversNoCreatedCallbacks`, `CloneDeliversNoObjectChangeCallbacks`)

## Dependencies

- REQ-CAPI-OBJCHANGE-001
- ADR-0047
