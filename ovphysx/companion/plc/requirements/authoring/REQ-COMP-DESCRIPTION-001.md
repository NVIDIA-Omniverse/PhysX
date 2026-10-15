<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-DESCRIPTION-001
title: "Stopped physics descriptions for inspection"
status: implemented
owner: ovphysx
---

## Description

Replace private USD-load descriptors with owned source-neutral descriptions for stopped collision, tendon, vehicle, particle and articulation visualization.

## Acceptance Criteria

- AC-1: **Families.** The description query reports supported shape, articulation/joint, tendon, vehicle and particle topology families individually and returns explicit unsupported diagnostics for absent families.
- AC-2: **Stopped access.** Descriptions are derived from immutable input snapshots without forcing simulation attachment or materialization.
- AC-3: **Public layout.** Descriptions contain documented relationships, transforms, units and stable identities, and expose no private descriptor layout or caller-managed runtime allocator.
- AC-4: **Revision lifetime.** Descriptions remain readable for their result lifetime and cannot be applied as current after their source revision changes.

## Implementation scope

Customer-USD capture copies composed input into immutable public descriptions;
queries then operate on those snapshots without USD or runtime initialization.
Supported families are rigid body, shape, joint, articulation root, tendon, vehicle/wheel and
particle schemas. Descriptions retain public schema parameter names and types,
defaults/authored provenance, relationships, transforms, units and instance paths.
They describe input topology, not realized solver objects. Unsupported query
families/value conversions and missing prims return stable diagnostics. Retained
results survive stage close but revision validation rejects application after edits.

## Test References

- [TEST-COMP-DESCRIPTION-001](../../tests/authoring/TEST-COMP-DESCRIPTION-001.md) - executable family, schema default, transform, identity and revision coverage.

## Code References

- [Immutable data/query contracts](../../../python/ovphysx_companion/descriptions.py)
- [Customer-USD capture](../../../python/ovphysx_companion/usd/descriptions.py)
- [Stopped Inspector sample](../../../samples/stopped_inspection.py)

Existing native input integration anchors:

- [ovphysx/ovruntime/source/omni.physics.ovstage/OvstageWalker.cpp](../../../../ovruntime/source/omni.physics.ovstage/OvstageWalker.cpp)
- [ovphysx/ovruntime/include/omni/physx/PhysXRuntime.h](../../../../ovruntime/include/omni/physx/PhysXRuntime.h)

## Dependencies

- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-IDENTITY-001](../exchange/REQ-COMP-IDENTITY-001.md)
- [REQ-COMP-BUFFERS-001](../exchange/REQ-COMP-BUFFERS-001.md)
- [Companion ADR-0003](../../adr/ADR-0003-data-services-and-jobs.md)
