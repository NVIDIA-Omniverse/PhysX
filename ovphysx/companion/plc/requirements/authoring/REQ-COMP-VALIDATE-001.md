<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-VALIDATE-001
title: "Classified validation and explicit repair application"
status: implemented
owner: ovphysx
---

## Description

Validation separates pure USD/schema checks, geometry/cooking checks and checks that require simulation. Every repair is explicit data applied by Python.

## Acceptance Criteria

- AC-1: **Classification.** Each validator declares whether it needs only USD data, cooking or a simulation session; pure checks perform no attach or runtime initialization.
- AC-2: **Diagnostics.** Results identify objects, severity, stable diagnostic code, source revision and suggested repair; computation does not silently mutate input.
- AC-3: **Simulation ownership.** Simulation-dependent checks use a dedicated companion-native attachment with a distinct ovstage and the same resource/lifetime admission as preview; a worker or explicit exclusive-owner mode is a fallback. Checks cannot displace or consume work from the live playback attachment.
- AC-4: **Repairs.** Python applies selected repairs through revision-checked reversible edits, with Kit undo supplied by the host and rejected/stale repairs leaving authored state unchanged.

## Implementation scope

The declared check catalog separates USD-only joint/mass/transform checks, collision
cooking and dedicated-preview motion checks. Results carry stable object/revision
codes and selected repairs are reversible and stale-checked. Tests cover all three
dependency classes while playback retains its receipt. This is the catalog in
validation.CHECKS, not a claim that every historical legacy-validator rule or a
GPU-convex predicate is implemented.

## Test References

- [TEST-COMP-VALIDATE-001](../../tests/authoring/TEST-COMP-VALIDATE-001.md) - executable classification, cooking, isolated simulation and selected-repair subset.

## Code References

- [Classified checks and isolated simulation](../../../python/ovphysx_companion/validation.py)
- [Revision-checked customer-USD repairs](../../../python/ovphysx_companion/usd/validation.py)
- [Immutable input descriptions](../../../python/ovphysx_companion/descriptions.py)
- [Extension repair sample](../../../samples/asset_validation.py)

Existing integration anchors:

- [ovphysx/ovruntime/source/omni.physics.ovstage/OvstageWalker.cpp](../../../../ovruntime/source/omni.physics.ovstage/OvstageWalker.cpp)
- [ovphysx/ovruntime/include/omni/physx/IPhysxCookingService.h](../../../../ovruntime/include/omni/physx/IPhysxCookingService.h)
- [ovphysx/include/ovphysx/ovphysx.h](../../../../include/ovphysx/ovphysx.h)

- [ovphysx/ovruntime/include/omni/physx/AttachOptions.h](../../../../ovruntime/include/omni/physx/AttachOptions.h)
- [ovphysx/ovruntime/include/omni/physx/IPhysxSimulation.h](../../../../ovruntime/include/omni/physx/IPhysxSimulation.h)

## Dependencies

- [REQ-COMP-USD-EDIT-001](../python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-PREVIEW-001](../live/REQ-COMP-PREVIEW-001.md)
- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)

## Sampled repairs

Copied descriptions and repair proposals retain their captured USD time. A repair
writes that exact default/numeric time, preserving other authored samples and
supporting the same revision checks and undo as default-time repairs.
