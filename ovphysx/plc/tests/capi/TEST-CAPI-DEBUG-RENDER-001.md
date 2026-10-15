<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-DEBUG-RENDER-001
maps_to: REQ-CAPI-DEBUG-RENDER-001
type: integration
---

## Scenario (AC-1, AC-2, AC-3, AC-4, AC-6, AC-7, AC-8, AC-9)

A bare instance, with no stage attached, uses the debug-render query to store settings and to read geometry.

## Given

- Process lifecycle is initialized.
- One ovphysx instance exists and has no attached stage.

## When

- The instance opens an `OVPHYSX_QUERY_TARGET_DEBUG_RENDER` query.
- It writes `bodyAxes` = 2.5 and `scale` = 0.25, then reads them back.
- It writes a negative `bodyAxes` value and a NaN through the write task.
- It stores `bodyAxes`, then commits a negative value on one write group and commits that same group again.
- It reads `linePosition` and `lineColor`.
- It reads `linePosition`, an unknown name, and `lineColor`, then reads `scale` and `enabled`.
- It reads a token-only attribute with no stage attached.
- It writes `scope` tokens 11 and 22, then a zero token, then an empty scope, then a count of 4097.
- It reads `cullingBox`, stores a box, reads it back, then commits element 0 as 0.
- It opens a debug-render query with an optional summary and checks that summary is empty when no stage is attached. With a stage attached, it resolves attribute tokens through that stage's path dictionary.
- `ovphysx_writability` is queried for `bodyAxes` and `pointPosition`.
- A write of `notAnAttribute` is opened on the debug-render query.

## Then

- The query handle is non-zero (AC-1).
- The reads return 2.5 and 0.25 (AC-2).
- The negative and NaN writes fail as task errors and leave the stored value unchanged (AC-2). The repeated commit of the rejected group is refused, and the stored value stays 2.5 (AC-2).
- Both geometry columns have leading shape 0 (AC-3).
- `bodyAxes` is writable and `pointPosition` is read-only (AC-4).
- The unknown read fails. The following `scale` and `enabled` read still returns those two columns, in that order, with attribute token 0 (AC-9). The token-only read fails (AC-9).
- The zero token and the count above 4096 fail as task errors. The empty scope write succeeds (AC-6).
- The first `cullingBox` read has leading shape 0. After a box is stored, a read returns a non-zero leading shape. After element 0 is committed as 0, the next read has leading shape 0 again (AC-7). The PhysX scene box is checked by the runtime test for REQ-PUBLICAPI-004.
- The shared dictionary is null (AC-8). Attached-stage coverage includes `tests/c_unittests/test_debug_render_e2e.cpp` `SharedDictionaryMatchesAttachedStage` and the Python scenario below.
- The unknown write fails (AC-4).

## Python attached-stage regression (AC-3, AC-8, AC-9)

Given a Python instance with `boxes_falling_on_groundplane.usda` attached:

- Open a debug-render query and resolve `linePosition` through its dictionary.
- Write `enabled`, `scale`, and `collisionShapes`, then step once.
- Read line positions and colors by name, then read positions by token.

The query summary is empty. Every write and read group's attribute token resolves
through the attached stage to the requested name. Geometry is nonempty, position
and color row counts match, and token reads return the same positions. After
detach, the Python query and read session reject further use as invalidated.
The no-stage Python control still returns empty geometry and attribute tokens 0.
Attribute tokens are also 0 when native detach has completed but its Python task
has not yet been collected; the retained old Stage must not supply a dictionary.

## Coverage

- AC-5 is covered by the scene filter in the runtime copy path and by `tests/c_unittests/test_debug_render_e2e.cpp`, which reads geometry after a stepped attach. This test does not construct a second attach.

## Code References

- ovphysx/tests/c_unittests/test_debug_render.cpp
- ovphysx/tests/c_unittests/test_debug_render_e2e.cpp
- ovphysx/tests/python_tests/test_debug_render.py
