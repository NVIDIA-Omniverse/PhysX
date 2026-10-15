<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-CCT-001
maps_to: REQ-COMP-CCT-001
type: integration
---

## Coverage status

Executable CPU capsule scenarios; GPU controllers are not advertised. Covers [REQ-COMP-CCT-001](../../requirements/controllers/REQ-COMP-CCT-001.md) AC-1 through AC-4.

## Scenario

Create a controller from ovstage, move against a wall and recreate it.

## Given

- An ovstage containing a controller descriptor, ground and wall with known dimensions.
- Explicit movement inputs and collision-event recorder, with no Kit loaded.

## When

- Create, move into the wall, change dimensions at the documented phase, then delete/recreate and reuse the stale handle.

## Then

- Creation produces the requested native dimensions and source identity (REQ AC-1).
- Movement respects the wall and reports documented collision flags with timestep/units semantics (REQ AC-2).
- No input-device or camera/Kit service is required (REQ AC-3).
- Old handles/events become invalid, resources release once and unsupported modes report unsupported (REQ AC-4).

## Executable coverage

[Native tests](../../../tests_native/test_cct.py) exercise real floor/wall collisions,
gravity, foot-preserving resize, teleport, category filtering, pending Busy,
ABI/output canaries, stale generations, owner retirement, peer isolation and the
[runnable extension loop](../../../samples/character_controller.py).
Runtime TestCharacterController also tests same-owner source deletion/recreation
and unrelated record additions. The rebuilt runtime shard also passes callback reentry with preserved outputs.
Broad runtime qualification gaps are recorded separately in the implementation plan.
