<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-DEBUG-RENDER-001
title: Instance Debug Render Sessions
status: implemented
owner: ovphysx
---

## Description

Debug rendering is an `OVPHYSX_QUERY_TARGET_DEBUG_RENDER` query on one ovphysx instance.
The target is the instance, not a prim. Settings are writable and readable.
Point, line, and triangle geometry is read-only. A read copies primitives only
from PhysX scenes whose attach handle equals the instance's attach handle.
An attach handle of 0 matches no scene, so the geometry columns are empty.
If that attach is live but its owner cannot be resolved, a geometry read fails
instead of returning an empty snapshot.

Geometry retains PhysX's [debug-render timing](../../../docs/read_write/readable.md#debug-render);
a read copies the buffer without recomputing poses.

Position and color for one primitive type come from the same read. Index `i`
is the same primitive in both columns. An empty column (`shape[0] == 0`) means
no geometry. Settings are one row. `scope` is a list of uint64 path tokens;
length 0 draws every object, and the write buffer holds at most 4096 tokens.
A parameter value of 0 turns that parameter off. A `cullingBox` read with leading shape 0 means no box is set. A `cullingBox` write is float32 shape `[7]`: element 0 equal to 0 clears the box, and element 0 equal to 1 stores the next six finite floats when min <= max. A `scope` write is uint64 shape `[4097]`: element 0 is the count, at most 4096, and elements 1..count are the tokens. An unknown read attribute fails the read and names that attribute. With a stage attached, each returned group's attribute token resolves through that stage's path dictionary. With no stage the token is 0, and a token-only attribute cannot be resolved. The optional query summary is empty, because this target is not a prim set. With no stage, attribute tokens are 0. With a stage attached, attribute tokens resolve through that stage's path dictionary. Each name is published as a
const `ovx_string_or_token_t` (`OVPHYSX_NAME_DEBUG_*`). Callers pass that
object's address.

## Acceptance Criteria

- AC-1: `ovphysx_query` accepts `OVPHYSX_QUERY_TARGET_DEBUG_RENDER` without an attached stage and returns a non-zero query handle.
- AC-2: Writing `bodyAxes` or `scale` on that query stores the value on that instance. A later read of the same attribute on that instance returns the stored value. A write of a non-finite or negative value fails as a task error and leaves the stored value unchanged. That failed commit spends the group, and a later commit of the same group is refused.
- AC-3: `pointPosition` / `pointColor`, `linePosition` / `lineColor`, and `trianglePosition` / `triangleColor` are read-only. A read with no matching scene returns a column whose leading shape is 0. Position index `i` and color index `i` describe the same primitive.
- AC-4: `ovphysx_writability` reports settings as `OVPHYSX_WRITABILITY_WRITABLE` and the six geometry names as `OVPHYSX_WRITABILITY_READ_ONLY`. An unknown name is `OVPHYSX_WRITABILITY_UNCLASSIFIED`, and `ovphysx_write` of that name fails.
- AC-5: A geometry read or settings apply for instance A does not include scenes whose attach handle differs from A's. Spline debug lines stored on A's attach are included with those scenes.
- AC-6: `scope` round-trips the tokens written. A zero token is rejected as a task error and does not replace the stored scope. Count 0 clears it. A count above 4096 is rejected as a task error.
- AC-7: A `cullingBox` read with no box set has leading shape 0. Committing element 0 as 0 clears a previously stored box, including the box on that attach's PhysX scenes. Committing element 0 as 1 with six finite floats and min <= max stores the box, and a later read returns that row.
- AC-9: A read that names an unknown attribute fails and does not return the remaining columns in shifted positions. A string read and a token read of the same debug-render attribute return the same column, and the group's attribute token resolves to that name when a stage is attached.
- AC-8: A debug-render query's optional summary is empty. With no stage, attribute tokens are 0. With a stage attached, attribute tokens resolve through that stage's path dictionary.

## Known limitations

- After detach, scale and parameter settings remain on the instance and are pushed to new scenes on the next attach. Scope tokens are cleared on detach because they depend on the attached stage path dictionary.

## Test References

- [TEST-CAPI-DEBUG-RENDER-001](../../tests/capi/TEST-CAPI-DEBUG-RENDER-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`OVPHYSX_QUERY_TARGET_DEBUG_RENDER`)
- ovphysx/include/ovphysx/ovphysx_attributes.h (`OVPHYSX_ATTR_DEBUG_*`)
- ovphysx/ovruntime/source/omni.physx/plugins/DebugRenderSession.cpp (per-instance settings)
- ovphysx/ovruntime/source/omni.physx/plugins/OvxPhysicsWrite.cpp (commit, including a rejected value)
- ovphysx/src/ovphysx/ovphysxDebugRenderSession.cpp (published name constants)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (query, read, write, commit, release dispatch)
- ovphysx/tests/c_unittests/test_debug_render.cpp

## Dependencies

- [ADR-0044](../../../ovruntime/plc/adr/ADR-0044-debug-render-sessions.md)
- [REQ-CAPI-INSTANCE-SCOPE-001](REQ-CAPI-INSTANCE-SCOPE-001.md)
