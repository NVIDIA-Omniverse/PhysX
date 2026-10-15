<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-INSTANCE-SCOPE-003
maps_to: REQ-CAPI-INSTANCE-SCOPE-001
type: integration
---

## Scenario

Visualization stays enabled across reattachment while its old dictionary scope is retired.

## Given

- A stepped instance with debug rendering enabled and nonempty body-axis geometry.
- A token scope limiting visualization to one cube, reducing its geometry count.

## When

- Reset the instance and replace its stage with a fresh copy of the same scene.
- Set scale and body-axis parameters again, without enabling rendering again.
- Step and fetch the debug geometry.

## Then

- Geometry matches the original unscoped count: enablement survives and the old
  dictionary's scope does not restrict the replacement (REQ AC-5).

Implemented by `ovphysx/tests/c_unittests/test_debug_render_e2e.cpp`,
`DebugRenderE2E.ScopeClearsAcrossStageReplacement`.
