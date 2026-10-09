<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BINDING-SELECTION-001
title: Tensor Bindings Reuse Only Current Compatible Selections
status: implemented
owner: ovphysx
---

## Description

Tensor bindings for different attributes can select the same physics objects. They may
share the native simulation view and child view that implement that selection, while
each binding retains its own public handle, tensor type and specification.

Sharing must preserve the result of creating a new binding. A view that remains valid
for reading its original objects can nevertheless be outdated for a new request after
matching objects are added or become eligible. Reuse therefore requires both lifetime
validity and current selection membership. The optimization adds no public API or
configuration setting; tensor bindings remain deprecated as documented in the API.

## Acceptance Criteria

- AC-1: **Compatible selection.** Native selection reuse is confined to one instance, its current attachment,
  the same native view family and the same ordered path-pattern list. The candidate
  view must be valid and report that its selection is current. The search does not
  retain a process-wide cache or outlive the bindings that own the native views.
- AC-2: **Independent lifetime.** Each successful creation returns an independent binding handle. Destroying
  one binding does not invalidate another binding sharing its selection. Shared child
  and simulation views are released exactly once after their final owner releases
  them; instance teardown also releases those resources. On GPU, rigid-body disable
  bindings keep their own views, and host-property bindings do not share views with
  GPU-row bindings. Disabling a body or refreshing a sibling's GPU mapping therefore
  does not invalidate otherwise readable host properties.
- AC-3: **Current membership.** After changes are applied to the live simulation, a new binding selects the
  currently eligible matching objects, even if an older binding for the same pattern
  remains alive. This includes added or cloned objects and re-enabled GPU rigid bodies.
  Rejecting the older view for reuse does not itself invalidate its existing bindings.
- AC-4: **Per-binding validation.** Reuse preserves attribute-specific validation, tensor shape and result order.
  A rejected binding leaves its existing siblings usable. Reordered pattern lists are
  evaluated in the requested order rather than reusing a differently ordered selection.

## Test References

- [TEST-CAPI-BINDING-SELECTION-001](../../tests/capi/TEST-CAPI-BINDING-SELECTION-001.md)
  (AC-1, AC-2, AC-3, AC-4)

## Code References

- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp
- ovphysx/src/include/internal/sdk/ovphysxSDK.hpp
- ovphysx/include/ovphysx/ovphysx.h

## Dependencies

- [REQ-CAPI-BINDING-STALE-001](REQ-CAPI-BINDING-STALE-001.md)
  (bindings retained across detach and reattach)
