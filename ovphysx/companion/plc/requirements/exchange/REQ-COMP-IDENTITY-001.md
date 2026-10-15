<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-IDENTITY-001
title: "Typed scene identity and source revision validation"
status: implemented
owner: ovphysx
---

## Description

A companion identity combines runtime owner, attachment generation, source object identity and instance identity where needed. Authored paths are diagnostic/mapping strings, not interchangeable runtime keys.

## Acceptance Criteria

- AC-1: **Namespaces.** Conversions between USD path strings, ovstage dictionary tokens, source ObjectKeys and PhysX object IDs are explicit and resolve against the supplied source/owner.
- AC-2: **Instances.** Results and edits distinguish multiple instances of one authored prim and include the generation required to reject recreated objects.
- AC-3: **Revision.** Computed results carry source identity, input revision and relevant topology versions; application rejects a mismatch before mutation.
- AC-4: **Missing objects.** Deleted, remapped or foreign-source objects produce an explicit missing/stale result; no operation falls back to a process-global active stage.

## Implementation scope

USD snapshots carry edit-session identity, revision and topology stamps. Native
references carry exact attachment, source key, record generation, kind and point
instance. Lookup/reverse lookup, queries, edits and deletion/recreation resolve
against that owner; equal paths in different owners are not interchangeable.
Point instances and authored clones have independent state. Unsupported USD
instance authoring fails explicitly; stale topology/source output never mutates USD.

## Test References

- [TEST-COMP-IDENTITY-002](../../tests/exchange/TEST-COMP-IDENTITY-002.md) - executed Python subset; see its scenario and limitations.
- [TEST-COMP-IDENTITY-001](../../tests/exchange/TEST-COMP-IDENTITY-001.md) - executed supported-contract scenario.

## Code References

Companion implementation:

- [usd/session.py](../../../python/ovphysx_companion/usd/session.py)
- [usd/meshes.py](../../../python/ovphysx_companion/usd/meshes.py)
- [Executable subset tests](../../../tests/test_usd_helpers.py)

Existing integration anchors; these do not establish native implementation:

- [ovphysx/ovruntime/include/omni/physx/IPhysxCustomJoint.h](../../../../ovruntime/include/omni/physx/IPhysxCustomJoint.h)
- [ovphysx/ovruntime/source/omni.physics.ovstage/OvstageWalker.cpp](../../../../ovruntime/source/omni.physics.ovstage/OvstageWalker.cpp)
- [ovphysx/include/ovphysx/ovphysx.h](../../../../include/ovphysx/ovphysx.h)

- [inspection.py](../../../python/ovphysx_companion/inspection.py)
- [test_inspection.py](../../../tests_native/test_inspection.py)
- [test_interaction.py](../../../tests_native/test_interaction.py)
- [test_plugins.py](../../../tests_native/test_plugins.py)

## Dependencies

- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [Companion ADR-0003](../../adr/ADR-0003-data-services-and-jobs.md)
