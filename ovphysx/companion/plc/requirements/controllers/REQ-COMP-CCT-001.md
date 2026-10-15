<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-CCT-001
title: "Optional native character controller lifecycle"
status: implemented
owner: ovphysx
---

## Description

A CCT service supplies controller creation and movement through ovstage plus supported native controller integration; ordinary rigid-body writes are not a substitute.

## Acceptance Criteria

- AC-1: **Creation.** Supported controller descriptors populated through ovstage create a native controller with explicit identity, dimensions and collision-filter policy.
- AC-2: **Movement.** move, gravity, position/size changes and collision results have documented units, timestep and safe execution phase.
- AC-3: **Input separation.** Host input and first-person camera policy stay above the native service; Python/native clients can move a controller without Kit.
- AC-4: **Lifecycle.** Controller deletion, recreation and session close release native state, invalidate handles and stop collision delivery; unsupported backend modes fail capability checks.

## Test References

- [TEST-COMP-CCT-001](../../tests/controllers/TEST-COMP-CCT-001.md) - native CPU movement and lifecycle cases are executable.

## Code References

- [Separate C table](../../../include/ovphysx_companion/cct.h)
- [Native bridge](../../../src/Cct.cpp)
- [Python controller](../../../python/ovphysx_companion/cct.py)
- [Extension loop sample](../../../samples/character_controller.py)
- [Native integration tests](../../../tests_native/test_cct.py)
- [Runtime CCT control](../../../../ovruntime/include/omni/physx/NativeCct.h)

## Implementation scope

CPU capsule controllers populated through ovstage support generation-checked
move, teleport, resize and removal. Python adds optional gravity integration;
the host owns input/camera policy and calls movement before its next simulation
step. Copied collision flags use explicit static/dynamic/controller query filters.
Native calls require idle exact-owner admission. GPU scenes report unavailable.
Creation requires PHYSICS plus RENDERING population for a CCT-only capsule.
Collision flags are synchronous move results; queued contact delivery belongs to
EVENTS. Same-owner replacement and callback reentry pass runtime tests for both
source backends. CPU capsule control is the advertised backend; GPU/DirectGPU
controllers are explicitly unavailable. Creation/descriptor edits use the normal
ovstage input path, as demonstrated by the sample scene and native change tests.

## Dependencies

- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-EVENTS-001](../live/REQ-COMP-EVENTS-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)
