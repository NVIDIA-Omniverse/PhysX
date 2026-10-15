<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-INSTANCE-SCOPE-001
title: Instance Scope for OVPhysX Public Operations
status: implemented
owner: ovphysx
---

## Description

Every public OVPhysX operation is either instance-scoped or explicitly global.
Instance-scoped operations take `ovphysx_instance_handle_t` first, keep wrapper-owned
state on that handle, and never operate on subordinate resources owned by another
instance. Global operations do not take an instance and must not observe or mutate
per-handle simulation state.

Several instances may each hold one live attach at the same time
(REQ-CAPI-ATTACH-OWNER-001). Wrapper-owned resources stay on the handle that
created them.

**Global (no instance argument):** `ovphysx_initialize`, `ovphysx_shutdown`,
`ovphysx_create_instance`, `ovphysx_get_version`, `ovphysx_get_version_string`,
`ovphysx_set_cpu_mode`, `ovphysx_get_cpu_mode`, `ovphysx_set_global_config`,
`ovphysx_get_global_config`, and the legacy global config entry/getters,
`ovphysx_set_log_level`, `ovphysx_get_log_level`,
`ovphysx_enable_default_log_output`, `ovphysx_set_log_callback`,
`ovphysx_flush_log`, `ovphysx_get_last_error`, `ovphysx_destroy_wait_task_result`,
`ovphysx_writability`,
and the dedicated `ovphysx_companion_*_acquire`
table-discovery family (including `ovphysx_companion_acquire`), except
`ovphysx_companion_custom_interfaces_acquire`. That native SDK entry point takes
a source instance and creates a retained service lease; its paired
`ovphysx_companion_custom_interfaces_release` takes that lease, not an instance.
The dedicated companion table uses explicit owner-bound sessions; its lifetime
contract is [REQ-CAPI-COMPANION-001](REQ-CAPI-COMPANION-001.md).

**Resource-scoped exception:** `ovphysx_companion_custom_interfaces_release`
releases its explicit service lease.

**Instance-scoped:** every other public `OVPHYSX_API` function, including
`ovphysx_set_config`, `ovphysx_get_config`, and
deprecated tensor, contact, scene-query, and SDF surfaces.
`ovphysx_subscribe_object_changes`, `ovphysx_unsubscribe_object_changes`, and
`ovphysx_get_last_task_error` take the instance as the first argument.

## Acceptance Criteria

- AC-1: Every public `OVPHYSX_API` function is classified instance-scoped, global, or
  an explicit service-lease exception as in Description. Global functions omit the instance; instance-scoped
  functions take `ovphysx_instance_handle_t` first.
- AC-2: Passing a subordinate query, read, write, tensor-binding, contact-binding,
  SDF-view, or subscription identifier that belongs to another live instance
  never operates on or invalidates the owning instance's resource. Query, read,
  write, tensor-binding, and subscription operations return `OVPHYSX_API_NOT_FOUND`
  (or `OVPHYSX_API_INVALID_ARGUMENT` for a null or invalid instance).
  Idempotent destroy/release APIs, including `ovphysx_destroy_sdf_view`, may
  instead return success for a resource absent from the supplied instance.
  Legacy contact-binding operations may return `OVPHYSX_API_ERROR` for an
  invalid instance rather than `INVALID_ARGUMENT`.
- AC-3: `ovphysx_get_last_task_error(handle, task)` returns the error retained
  by that instance's live wait result. A foreign or unknown task receipt returns
  a stable empty string and does not alter the peer instance's retained error.
- AC-4: Object-change callbacks registered on instance A are not delivered for
  attach-owned work of instance B. Unsubscribing A's subscription through B
  returns `OVPHYSX_API_NOT_FOUND` and leaves A's subscription active.
  Destroying A unsubscribes only A's remaining subscriptions and releases
  frontend callback keepalives once native cleanup is confirmed. Subscriptions
  may precede attach and retain their public IDs across detach and reattach.
- AC-5: Debug-render parameter and scale values cached through instance A are
  not overwritten by sets on instance B. Scene-query hit buffers remain
  per-instance. Debug-render enablement survives reattachment on the same
  instance; dictionary-owned visualization scope tokens are cleared.
- AC-6: Destroying or detaching instance A does not invalidate instance B's
  handle, subscriptions that B still owns, or B's live attach. B can also
  attach while A is still attached.

## Test References

- [TEST-CAPI-INSTANCE-SCOPE-001](../../tests/capi/TEST-CAPI-INSTANCE-SCOPE-001.md)
- [TEST-CAPI-INSTANCE-SCOPE-002](../../tests/capi/TEST-CAPI-INSTANCE-SCOPE-002.md)
- [TEST-CAPI-INSTANCE-SCOPE-003](../../tests/capi/TEST-CAPI-INSTANCE-SCOPE-003.md)

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_subscribe_object_changes`, `ovphysx_unsubscribe_object_changes`, `ovphysx_get_last_task_error`)
- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_get_last_task_error`, `ovphysx_wait_task`, `omni_sdk_physx_destroy`)
- ovphysx/src/ovphysx/ovphysxObjectChangeCallbacks.cpp
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (query/read/write owner checks)
- ovphysx/ovruntime/source/omni.physx/plugins/DebugRenderSession.cpp (per-instance debug-render settings)
- ovphysx/src/include/internal/sdk/ovphysxSDK.hpp (`InstanceData`)
- ovphysx/include/ovphysx/experimental/ovphysx.hpp (`subscribeObjectChanges`)
- ovphysx/tests/c_unittests/test_debug_render_e2e.cpp (`ScopeClearsAcrossStageReplacement`)
- ovphysx/tests/c_unittests/test_multi_instance.cpp
- ovphysx/tests/c_unittests/test_object_change_callbacks.cpp
- ovphysx/tests/python_tests/lifecycle_tests/test_multi_instance.py

## Dependencies

- REQ-CAPI-ATTACH-OWNER-001
- ADR-0038
