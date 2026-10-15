<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-QUERY-001
maps_to: REQ-COMP-QUERY-001
type: integration
---

## Coverage status

Executed: nine native C/Python integration scenarios (including the Inspector
sample and convex geometry) and the native lifecycle case on both source backends. Covers
[REQ-COMP-QUERY-001](../../requirements/live/REQ-COMP-QUERY-001.md) AC-1 through AC-4.

## Scenario

Query and inspect a scene containing instanced colliders, a back-facing triangle and a moving articulation.

## Given

- Analytically positioned colliders with known hit distances, two instances of one source prim, and bounded result capacity.
- A completed simulation step followed by deletion of one hit object.

## When

- Run advertised query variants and inspection reads, compare against the core operation, then reuse a stale object handle.

## Then

- Any/closest/all/filter/sidedness/capacity results match declared semantics, with explicit truncation or unsupported status (REQ AC-1).
- Hits identify the correct instance and expected geometric values/step (REQ AC-2).
- Inspection returns typed state and frame conversion with no acquisition of a unit-test API (REQ AC-3).
- Results are collected after completion and the deleted-object read fails safely (REQ AC-4).

## Executable coverage

- [test_queries.py](../../../tests_native/test_queries.py): analytical/core parity for ray/sweep, any/closest/all, inclusion/exclusion, static/dynamic flags, explicit capacity truncation, primitive and bound geometry, shared-shape point instances, triangle backfaces, unsupported variants, ABI output canaries and pending/foreign-owner isolation.
- [test_inspection.py](../../../tests_native/test_inspection.py): typed mass/material/body state, joint world-frame conversion, moving articulation DOF/link state, copied lifetime, owner retirement, ABI sizing and the executable Inspector sample.
- [test_camera.py](../../../tests_native/test_camera.py): existing supported core read-session body output.

CPU is qualified. No GPU query/inspection capability is advertised; the native
service rejects GPU/DirectGPU live state. Material inspection is host metadata.
